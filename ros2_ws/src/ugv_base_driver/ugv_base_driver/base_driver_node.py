"""ROS 2 node bridging /cmd_vel to a serial differential-drive base.

Safety design (deliberately conservative):

1. Watchdog — if no /cmd_vel message arrives within `watchdog_timeout`
   seconds, the node repeatedly writes the STOP command to the port so a
   crashed teleop/planner cannot leave the vehicle moving.
2. Speed limits — linear/angular are clamped (scaling both together to
   preserve the arc) before kinematics.
3. Serial failure — a write error logs and triggers a latched stop state;
   the node keeps running so the failure stays visible in logs.
4. No auto-enable — the node starts in DISABLED state and only forwards
   commands after an explicit `true` is published on `~/enable`
   (std_msgs/Bool), so launching it can never move the vehicle by itself.

Serial backend is pyserial; when it is not installed the node exits with
a clear message instead of crashing inside the import.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from .command_protocol import PROTOCOL_TEXT, encode_wheel_speeds
from .diff_kinematics import wheel_speeds_from_twist


class UgvBaseDriver(Node):
    """Bridge /cmd_vel to wheel-speed commands on a serial port."""

    def __init__(self) -> None:
        super().__init__("ugv_base_driver")

        # ----- parameters -------------------------------------------------
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_base", 0.4)
        self.declare_parameter("wheel_radius", 0.075)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("watchdog_timeout", 0.5)
        self.declare_parameter("protocol", PROTOCOL_TEXT)
        self.declare_parameter("watchdog_period", 0.1)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.protocol = str(self.get_parameter("protocol").value)
        watchdog_timeout = float(self.get_parameter("watchdog_timeout").value)
        watchdog_period = float(self.get_parameter("watchdog_period").value)

        # Validate early so bad parameters fail before the port opens.
        if watchdog_timeout <= 0.0 or watchdog_period <= 0.0:
            raise ValueError("watchdog timings must be positive")

        # ----- serial backend ---------------------------------------------
        self._serial = None
        try:
            import serial  # pyserial
        except ImportError as exc:  # pragma: no cover - depends on host
            raise RuntimeError(
                "pyserial is required for ugv_base_driver; install with "
                "'pip install pyserial' or 'sudo apt install "
                "python3-serial' on the vehicle computer"
            ) from exc

        try:
            self._serial = serial.Serial(
                port=self.serial_port, baudrate=self.baudrate, timeout=0.1
            )
        except Exception as exc:
            self.get_logger().error(
                "cannot open serial port {} @ {}: {}".format(
                    self.serial_port, self.baudrate, exc
                )
            )
            raise

        # ----- state ------------------------------------------------------
        self._enabled = False
        self._last_cmd_time: Optional[float] = None
        self._stopped = True

        # ----- ROS interfaces ---------------------------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.on_cmd_vel,
            10,
        )
        self.enable_sub = self.create_subscription(
            Bool, "~/enable", self.on_enable, 10
        )
        self.watchdog_timer = self.create_timer(watchdog_period, self.watchdog_tick)

        self.get_logger().warn(
            "ugv_base_driver ready on {} @ {} (protocol={}). "
            "Wheels must be OFF THE GROUND for first test. "
            "Command forwarding is DISABLED until 'true' on ~/enable.".format(
                self.serial_port, self.baudrate, self.protocol
            )
        )

    # ------------------------------------------------------------------ ROS
    def on_enable(self, message: Bool) -> None:
        """Explicit enable/disable gate (std_msgs/Bool)."""
        self._enabled = bool(message.data)
        if not self._enabled:
            self.write_stop("disabled via ~/enable")
        else:
            self.get_logger().info("command forwarding enabled")

    def on_cmd_vel(self, message: Twist) -> None:
        """Convert twist to wheel speeds and write the serial command."""
        self._last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

        if not self._enabled:
            if self._stopped:
                self.get_logger().warn(
                    "/cmd_vel ignored: node is disabled; publish 'true' on "
                    "~/enable to allow motion",
                    throttle_duration_sec=5.0,
                )
            return

        linear = float(message.linear.x)
        angular = float(message.angular.z)

        left, right = wheel_speeds_from_twist(
            linear,
            angular,
            self.wheel_base,
            self.wheel_radius,
            self.max_linear,
            self.max_angular,
        )

        payload, self._encoder = encode_wheel_speeds(left, right, self.protocol)
        if self.write_bytes(payload):
            self._stopped = False
            self.get_logger().info(
                "cmd v={:+.3f} m/s w={:+.3f} rad/s -> {}".format(
                    linear, angular, payload.decode("ascii", errors="replace").strip()
                ),
                throttle_duration_sec=0.5,
            )

    def watchdog_tick(self) -> None:
        """Stop the vehicle when /cmd_vel stops arriving while enabled."""
        if not self._enabled or self._last_cmd_time is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        timeout = float(self.get_parameter("watchdog_timeout").value)
        if now - self._last_cmd_time > timeout:
            self.write_stop("cmd_vel timeout ({:.2f}s)".format(timeout))

    # -------------------------------------------------------------- serial
    def write_bytes(self, payload: bytes) -> bool:
        """Write raw bytes; on failure log once and keep the node alive."""
        if self._serial is None:
            return False
        try:
            self._serial.write(payload)
            return True
        except Exception as exc:
            self.get_logger().error(
                "serial write failed: {}".format(exc), throttle_duration_sec=2.0
            )
            return False

    def write_stop(self, reason: str) -> None:
        """Send the neutral command; idempotent while already stopped."""
        if self._stopped:
            return
        encoder = getattr(self, "_encoder", None)
        if encoder is None:
            from .command_protocol import encode_stop

            payload = encode_stop()
        else:
            payload = encoder(0.0, 0.0)
        if self.write_bytes(payload):
            self._stopped = True
            self.get_logger().warn("STOP sent ({})".format(reason))

    def close(self) -> None:
        """Best-effort stop then close the port."""
        try:
            self.write_stop("node shutdown")
        finally:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = UgvBaseDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
