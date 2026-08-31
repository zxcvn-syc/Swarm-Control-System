"""Bridge planner paths to MAVROS setpoints and PX4 OFFBOARD control."""

from __future__ import annotations

from typing import ClassVar

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class PX4OffboardBridge(Node):
    """Stream setpoints, ARM the vehicle, then switch PX4 to OFFBOARD."""

    PARAMS: ClassVar[dict[str, str | float | bool | int]] = {
        "path_topic": "/planned_path", "setpoint_topic": "/mavros/setpoint_raw/local",
        "state_topic": "/mavros/state", "arm_service": "/mavros/cmd/arming",
        "mode_service": "/mavros/set_mode", "publish_period": 0.05,
        "prestream_seconds": 3.0, "command_retry_seconds": 1.0,
        "offboard_mode": "OFFBOARD", "auto_arm": False,
        "enable_setpoint_streaming": False, "drone_id": -1,
        "hold_x": 0.0, "hold_y": 0.0, "hold_z": 2.0,
        "local_pose_topic": "/mavros/local_position/pose",
        "safety_hold_enabled": False,
        "initial_safety_hold": False,
        "safety_hold_topic": "/flight_safety/hold_request",
        "coordinate_frame": PositionTarget.FRAME_LOCAL_NED,
    }
    TYPE_MASK = (
        PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
        | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
    )

    def __init__(self) -> None:
        super().__init__("px4_offboard_bridge")
        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)

        self.enable_setpoint_streaming = bool(
            self.get_parameter("enable_setpoint_streaming").value
        )
        requested_auto_arm = bool(self.get_parameter("auto_arm").value)
        self.auto_arm = requested_auto_arm and self.enable_setpoint_streaming
        self.drone_id = int(self.get_parameter("drone_id").value)
        self.offboard_mode = str(self.get_parameter("offboard_mode").value)
        self.prestream_seconds = max(1.0, self._float_param("prestream_seconds"))
        self.retry_seconds = max(0.2, self._float_param("command_retry_seconds"))
        self._hold = tuple(
            self._float_param(name) for name in ("hold_x", "hold_y", "hold_z")
        )
        self.safety_hold_enabled = bool(
            self.get_parameter("safety_hold_enabled").value
        )
        self._safety_hold = self.safety_hold_enabled and bool(
            self.get_parameter("initial_safety_hold").value
        )
        self._safety_hold_position: tuple[float, float, float] | None = None
        self._local_position: tuple[float, float, float] | None = None
        self._waypoints: list[tuple[float, float, float]] = []
        self._state: State | None = None
        self._index = 0
        self._phase = "disabled" if not self.enable_setpoint_streaming else (
            "manual" if not self.auto_arm else "wait_fcu"
        )
        self._stream_started_at: float | None = None
        self._next_command_at = 0.0
        self._arm_future = None
        self._offboard_future = None
        self._warned_path_scope = False

        self.create_subscription(Path, self._str_param("path_topic"), self.on_path, 10)
        self.create_subscription(State, self._str_param("state_topic"), self.on_state, 10)
        self.create_subscription(
            PoseStamped, self._str_param("local_pose_topic"), self.on_local_pose, 10
        )
        self._safety_hold_sub = None
        if self.safety_hold_enabled:
            self._safety_hold_sub = self.create_subscription(
                Bool, self._str_param("safety_hold_topic"), self.on_safety_hold, 10
            )
        self.pub = self.create_publisher(PositionTarget, self._str_param("setpoint_topic"), 10)
        self.arm_client = self.create_client(CommandBool, self._str_param("arm_service"))
        self.mode_client = self.create_client(SetMode, self._str_param("mode_service"))
        self.create_timer(max(0.01, self._float_param("publish_period")), self.tick)
        if requested_auto_arm and not self.enable_setpoint_streaming:
            self.get_logger().warning(
                "auto_arm ignored because enable_setpoint_streaming is false"
            )
        self.get_logger().info(
            "offboard bridge ready: "
            f"streaming={self.enable_setpoint_streaming}, "
            f"drone_id={self.drone_id}, auto_arm={self.auto_arm}, "
            f"prestream={self.prestream_seconds:.1f}s, "
            f"safety_hold={self.safety_hold_enabled}"
        )

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def on_path(self, message: Path) -> None:
        if getattr(self, "_safety_hold", False):
            # A path generated before a hold release must never resume by
            # itself. A planner must emit a fresh scoped path after release.
            return
        if self.drone_id < 0:
            self._waypoints = []
            self._index = 0
            if message.poses and not self._warned_path_scope:
                self._warned_path_scope = True
                self.get_logger().warning(
                    "ignoring /planned_path until a non-negative drone_id is set"
                )
            return
        expected_frame = f"drone_{self.drone_id}"
        self._waypoints = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
            if str(getattr(pose.header, "frame_id", "")) == expected_frame
        ]
        self._index = 0
        if message.poses and not self._waypoints and not self._warned_path_scope:
            self._warned_path_scope = True
            self.get_logger().warning(
                f"ignoring /planned_path with no poses for {expected_frame}"
            )
        elif self._waypoints:
            self._warned_path_scope = False

    def on_state(self, message: State) -> None:
        self._state = message

    def on_local_pose(self, message: PoseStamped) -> None:
        self._local_position = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )
        if self._safety_hold and self._safety_hold_position is None:
            self._safety_hold_position = self._local_position
            self.get_logger().warning("safety hold captured first available local pose")

    def on_safety_hold(self, message: Bool) -> None:
        """Hold at the observed local position and discard stale paths.

        This is a software interlock only. If local pose is unavailable, the
        bridge intentionally stops sending setpoints so PX4's configured
        Offboard-loss failsafe remains authoritative rather than guessing a
        potentially unsafe position.
        """

        requested = bool(message.data)
        if requested == self._safety_hold:
            return
        self._safety_hold = requested
        self._waypoints = []
        self._index = 0
        if requested:
            self._safety_hold_position = self._local_position
            if self._safety_hold_position is None:
                self.get_logger().warning(
                    "safety hold requested without local pose; relying on PX4 failsafe"
                )
            else:
                self.get_logger().warning(
                    "safety hold requested; path cleared and position captured"
                )
        else:
            self._safety_hold_position = None
            self.get_logger().info("safety hold released; waiting for a fresh path")

    def _seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _set_phase(self, phase: str) -> None:
        if phase != self._phase:
            self._phase = phase
            self.get_logger().info(f"offboard phase -> {phase}")

    def _publish_setpoint(self) -> bool:
        if getattr(self, "_safety_hold", False):
            position = self._safety_hold_position
            if position is None:
                return False
        else:
            position = self._hold
        if self._waypoints and not getattr(self, "_safety_hold", False):
            position = self._waypoints[min(self._index, len(self._waypoints) - 1)]
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = int(self.get_parameter("coordinate_frame").value)
        target.type_mask = self.TYPE_MASK
        target.position.x, target.position.y, target.position.z = position
        target.yaw = 0.0
        self.pub.publish(target)
        ready_to_follow = self._phase == "active" or not self.auto_arm
        if self._index + 1 < len(self._waypoints) and ready_to_follow:
            self._index += 1
        return True

    def _request_command(self, kind: str, now: float) -> None:
        future_name = f"_{kind}_future"
        if now < self._next_command_at or getattr(self, future_name) is not None:
            return
        client = self.arm_client if kind == "arm" else self.mode_client
        if not client.service_is_ready():
            return
        if kind == "arm":
            request = CommandBool.Request()
            request.value = True
        else:
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = self.offboard_mode
        future = client.call_async(request)
        setattr(self, future_name, future)
        future.add_done_callback(lambda done: self._command_done(kind, done))
        self._next_command_at = now + self.retry_seconds

    def _command_done(self, kind: str, future) -> None:
        setattr(self, f"_{kind}_future", None)
        error = future.exception()
        if error is not None:
            self.get_logger().warning(f"{kind} service failed: {error}")
            return
        result = future.result()
        accepted = result.success if kind == "arm" else result.mode_sent
        if not accepted:
            command = "ARM" if kind == "arm" else self.offboard_mode
            self.get_logger().warning(f"PX4 rejected {command}; retrying")

    def tick(self) -> None:
        if not self.enable_setpoint_streaming:
            return
        published = self._publish_setpoint()
        if getattr(self, "_safety_hold", False):
            # Never request ARM/OFFBOARD while an upstream safety gate is
            # closed. Existing flight mode and physical failsafes stay intact.
            return
        if not published:
            return
        if not self.auto_arm:
            return
        now = self._seconds()
        if self._state is None or not self._state.connected:
            self._stream_started_at = None
            self._set_phase("wait_fcu")
            return
        if self._stream_started_at is None:
            self._stream_started_at = now
            self._set_phase("prestream")
        if now - self._stream_started_at < self.prestream_seconds:
            return
        if not self._state.armed:
            self._set_phase("arming")
            self._request_command("arm", now)
            return
        if self._state.mode != self.offboard_mode:
            self._set_phase("offboard")
            self._request_command("offboard", now)
            return
        self._set_phase("active")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PX4OffboardBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
