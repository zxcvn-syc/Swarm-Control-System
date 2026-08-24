"""Bridge planner paths to MAVROS setpoints and PX4 OFFBOARD control."""

from __future__ import annotations

from typing import ClassVar

import rclpy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


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
            f"prestream={self.prestream_seconds:.1f}s"
        )

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def on_path(self, message: Path) -> None:
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

    def _seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _set_phase(self, phase: str) -> None:
        if phase != self._phase:
            self._phase = phase
            self.get_logger().info(f"offboard phase -> {phase}")

    def _publish_setpoint(self) -> None:
        position = self._hold
        if self._waypoints:
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
        self._publish_setpoint()
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
