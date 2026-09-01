"""Fail-closed ROS 2 serial driver for a differential-drive UGV base."""

from __future__ import annotations

import json
import math
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .command_protocol import PROTOCOL_TEXT, get_encoder
from .diff_kinematics import (
    configure_wheel_directions,
    limit_body_twist,
    limit_twist_rate,
    wheel_speeds_from_twist,
)


class UgvBaseDriver(Node):
    """Bridge a guarded Twist stream to the configured serial protocol."""

    def __init__(self) -> None:
        super().__init__("ugv_base_driver")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_base", 0.4)
        self.declare_parameter("wheel_radius", 0.075)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("max_wheel_angular_speed", 18.0)
        self.declare_parameter("max_linear_accel", 0.8)
        self.declare_parameter("max_angular_accel", 1.5)
        self.declare_parameter("left_wheel_sign", 1)
        self.declare_parameter("right_wheel_sign", 1)
        self.declare_parameter("swap_wheels", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("enable_topic", "~/enable")
        self.declare_parameter("estop_topic", "~/estop")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("watchdog_timeout", 0.5)
        self.declare_parameter("watchdog_period", 0.1)
        self.declare_parameter("stop_repeat_period", 0.5)
        self.declare_parameter("status_period", 0.5)
        self.declare_parameter("protocol", PROTOCOL_TEXT)

        self.serial_port = str(self.get_parameter("serial_port").value).strip()
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.max_wheel_angular = float(
            self.get_parameter("max_wheel_angular_speed").value
        )
        self.max_linear_accel = float(
            self.get_parameter("max_linear_accel").value
        )
        self.max_angular_accel = float(
            self.get_parameter("max_angular_accel").value
        )
        self.left_wheel_sign = int(self.get_parameter("left_wheel_sign").value)
        self.right_wheel_sign = int(self.get_parameter("right_wheel_sign").value)
        self.swap_wheels = bool(self.get_parameter("swap_wheels").value)
        self.watchdog_timeout = float(
            self.get_parameter("watchdog_timeout").value
        )
        self.watchdog_period = float(self.get_parameter("watchdog_period").value)
        self.stop_repeat_period = float(
            self.get_parameter("stop_repeat_period").value
        )
        self.status_period = float(self.get_parameter("status_period").value)
        self.protocol = str(self.get_parameter("protocol").value).strip()

        self._validate_parameters()
        self._encoder = get_encoder(self.protocol)

        try:
            import serial
        except ImportError as exc:  # pragma: no cover - host dependency
            raise RuntimeError(
                "pyserial is required; install python3-serial on the vehicle"
            ) from exc
        self._serial_backend = serial
        self._serial = None

        self._enabled = False
        self._estop_latched = False
        self._fault_latched = False
        self._fault_reason = ""
        self._stopped = False
        self._last_cmd_time: Optional[float] = None
        self._last_apply_time: Optional[float] = None
        self._last_stop_time: Optional[float] = None
        self._applied_linear = 0.0
        self._applied_angular = 0.0

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        enable_topic = str(self.get_parameter("enable_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.cmd_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.on_cmd_vel, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.on_enable, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, estop_topic, self.on_estop, 10
        )
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.reset_service = self.create_service(
            Trigger, "~/reset_fault", self.on_reset_fault
        )
        self.watchdog_timer = self.create_timer(
            self.watchdog_period, self.watchdog_tick
        )
        self.status_timer = self.create_timer(
            self.status_period, self.publish_status
        )

        self._open_serial()
        if not self.write_stop("startup", force=True):
            raise RuntimeError("serial port opened but startup STOP failed")

        self.get_logger().warn(
            "UGV base ready on {} @ {} (protocol={}); motion remains disabled "
            "until {} receives true".format(
                self.serial_port, self.baudrate, self.protocol, enable_topic
            )
        )

    def _validate_parameters(self) -> None:
        positive = {
            "wheel_base": self.wheel_base,
            "wheel_radius": self.wheel_radius,
            "max_linear_speed": self.max_linear,
            "max_angular_speed": self.max_angular,
            "max_wheel_angular_speed": self.max_wheel_angular,
            "max_linear_accel": self.max_linear_accel,
            "max_angular_accel": self.max_angular_accel,
            "watchdog_timeout": self.watchdog_timeout,
            "watchdog_period": self.watchdog_period,
            "stop_repeat_period": self.stop_repeat_period,
            "status_period": self.status_period,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if not self.serial_port:
            raise ValueError("serial_port must not be empty")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if self.left_wheel_sign not in (-1, 1) or self.right_wheel_sign not in (
            -1,
            1,
        ):
            raise ValueError("wheel signs must be either -1 or 1")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _open_serial(self) -> None:
        try:
            self._serial = self._serial_backend.serial_for_url(
                self.serial_port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.2,
            )
        except Exception as exc:
            self._serial = None
            raise RuntimeError(
                "cannot open serial port {} @ {}: {}".format(
                    self.serial_port, self.baudrate, exc
                )
            ) from exc

    def on_enable(self, message: Bool) -> None:
        requested = bool(message.data)
        if not requested:
            self._enabled = False
            self._last_cmd_time = None
            self._last_apply_time = None
            self.write_stop("disabled", force=True)
            self.publish_status()
            return

        if self._estop_latched or self._fault_latched:
            self.get_logger().error(
                "enable rejected: estop={} fault={} ({})".format(
                    self._estop_latched,
                    self._fault_latched,
                    self._fault_reason or "none",
                )
            )
            self.write_stop("enable rejected", force=True)
            self.publish_status()
            return

        if not self.write_stop("enable precondition", force=True):
            self.publish_status()
            return
        self._last_cmd_time = None
        self._last_apply_time = self._now()
        self._enabled = True
        self.get_logger().warn("command forwarding enabled; awaiting fresh cmd_vel")
        self.publish_status()

    def on_estop(self, message: Bool) -> None:
        if not bool(message.data):
            self.get_logger().warn(
                "false on estop topic does not clear the latch; call ~/reset_fault"
            )
            return
        self._estop_latched = True
        self._enabled = False
        self._fault_reason = "emergency stop requested"
        self.write_stop("emergency stop", force=True)
        self.publish_status()

    def on_reset_fault(self, _request: Trigger.Request, response: Trigger.Response):
        if self._enabled:
            response.success = False
            response.message = "disable the driver before resetting faults"
            return response

        if self._serial is None:
            try:
                self._open_serial()
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

        self._estop_latched = False
        self._fault_latched = False
        self._fault_reason = ""
        if not self.write_stop("fault reset", force=True):
            response.success = False
            response.message = self._fault_reason or "STOP failed during reset"
            return response

        response.success = True
        response.message = "faults cleared; driver remains disabled"
        self.publish_status()
        return response

    def on_cmd_vel(self, message: Twist) -> None:
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self._latch_fault("non-finite cmd_vel rejected")
            self.write_stop("invalid cmd_vel", force=True)
            self.publish_status()
            return

        if not self._enabled:
            self.get_logger().warn(
                "cmd_vel ignored while driver is disabled",
                throttle_duration_sec=5.0,
            )
            return

        now = self._now()
        target_linear, target_angular = limit_body_twist(
            linear, angular, self.max_linear, self.max_angular
        )
        dt = max(0.0, now - (self._last_apply_time or now))
        applied_linear, applied_angular = limit_twist_rate(
            self._applied_linear,
            self._applied_angular,
            target_linear,
            target_angular,
            self.max_linear_accel,
            self.max_angular_accel,
            dt,
        )
        left, right = wheel_speeds_from_twist(
            applied_linear,
            applied_angular,
            self.wheel_base,
            self.wheel_radius,
            self.max_linear,
            self.max_angular,
            self.max_wheel_angular,
        )
        left, right = configure_wheel_directions(
            left,
            right,
            swap_wheels=self.swap_wheels,
            left_sign=self.left_wheel_sign,
            right_sign=self.right_wheel_sign,
        )
        payload = self._encoder(left, right)
        if not self.write_bytes(payload):
            self.publish_status()
            return

        self._applied_linear = applied_linear
        self._applied_angular = applied_angular
        self._last_apply_time = now
        self._last_cmd_time = now
        self._stopped = (
            abs(applied_linear) < 1e-6 and abs(applied_angular) < 1e-6
        )
        self.get_logger().info(
            "cmd target=({:+.3f},{:+.3f}) applied=({:+.3f},{:+.3f}) "
            "wheels=({:+.2f},{:+.2f})".format(
                linear,
                angular,
                applied_linear,
                applied_angular,
                left,
                right,
            ),
            throttle_duration_sec=0.5,
        )

    def watchdog_tick(self) -> None:
        now = self._now()
        if self._enabled:
            if self._last_cmd_time is None:
                self.write_stop("enabled awaiting fresh cmd_vel")
                return
            age = now - self._last_cmd_time
            if age > self.watchdog_timeout:
                self._enabled = False
                self._last_cmd_time = None
                self.write_stop(
                    "cmd_vel timeout {:.2f}s; re-enable required".format(age),
                    force=True,
                )
                self.publish_status()
                return
        else:
            self.write_stop("disabled safety hold")

    def write_bytes(self, payload: bytes) -> bool:
        if self._serial is None:
            self._latch_fault("serial port is not open")
            return False
        try:
            written = self._serial.write(payload)
            if written is not None and int(written) != len(payload):
                raise IOError(
                    "short serial write: {} of {} bytes".format(
                        written, len(payload)
                    )
                )
            return True
        except Exception as exc:
            self._latch_fault("serial write failed: {}".format(exc))
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            return False

    def _latch_fault(self, reason: str) -> None:
        self._enabled = False
        self._fault_latched = True
        self._fault_reason = reason
        self.get_logger().error(reason, throttle_duration_sec=2.0)

    def write_stop(self, reason: str, *, force: bool = False) -> bool:
        now = self._now()
        if (
            not force
            and self._stopped
            and self._last_stop_time is not None
            and now - self._last_stop_time < self.stop_repeat_period
        ):
            return True
        if not self.write_bytes(self._encoder(0.0, 0.0)):
            return False
        self._applied_linear = 0.0
        self._applied_angular = 0.0
        self._last_apply_time = now
        self._last_stop_time = now
        self._stopped = True
        self.get_logger().warn(
            "STOP sent ({})".format(reason), throttle_duration_sec=1.0
        )
        return True

    def publish_status(self) -> None:
        now = self._now()
        age = None
        if self._last_cmd_time is not None:
            age = max(0.0, now - self._last_cmd_time)
        message = String()
        message.data = json.dumps(
            {
                "enabled": self._enabled,
                "estop_latched": self._estop_latched,
                "fault_latched": self._fault_latched,
                "fault_reason": self._fault_reason,
                "stopped": self._stopped,
                "serial_open": self._serial is not None,
                "protocol": self.protocol,
                "last_cmd_age_sec": age,
                "applied_linear_mps": self._applied_linear,
                "applied_angular_rps": self._applied_angular,
            },
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def close(self) -> None:
        self._enabled = False
        if self._serial is not None:
            self.write_stop("node shutdown", force=True)
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
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
