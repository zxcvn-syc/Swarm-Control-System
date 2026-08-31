"""ROS2 gate for supervised containment commands.

This node gates ``EnclosureCommandArray`` messages before they reach the
planner bridge and publishes a durable status/hold signal for the Offboard
bridge.  It intentionally never calls MAVROS arm, disarm, mode, RTL, or land
services.  PX4 configuration, RC, and a physical emergency stop remain the
authoritative flight-safety layers.
"""

from __future__ import annotations

import secrets
from typing import Iterable

import rclpy
from mavros_msgs.msg import State
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from swarm_interfaces.msg import (
    DroneStateArray,
    EnclosureCommandArray,
    FlightSafetyStatus,
    TargetTrackArray,
)
from swarm_interfaces.srv import SafetyControl

from .flight_safety import (
    ActivationMode,
    FlightSafetyController,
    SafetyCommand,
    SafetySnapshot,
    validate_enclosure_payload,
)


def _stamp_seconds(stamp) -> float | None:
    """Convert a ROS stamp to seconds, rejecting an unset stamp."""

    if stamp is None:
        return None
    seconds = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9
    return seconds if seconds > 0.0 else None


class FlightSafetySupervisor(Node):
    """Fail-closed ROS adapter around :class:`FlightSafetyController`."""

    def __init__(self) -> None:
        super().__init__("flight_safety_supervisor")
        self._declare_parameters()
        self._controller = FlightSafetyController(
            drone_state_timeout=self._float_param("drone_state_timeout"),
            target_timeout=self._float_param("target_timeout"),
            command_timeout=self._float_param("command_timeout"),
            mavros_timeout=self._float_param("mavros_timeout"),
            max_command_future_skew=self._float_param("max_command_future_skew"),
            target_lock_observations=self._int_param("target_lock_observations"),
            require_mavros_connection=self._bool_param("require_mavros_connection"),
            require_target_lock_in_manual=self._bool_param("require_target_lock_in_manual"),
            require_ground_confirmation_for_reset=self._bool_param(
                "require_ground_confirmation_for_reset"
            ),
            session_id=secrets.randbits(64) or 1,
        )
        self._max_command_abs_coordinate = self._float_param("max_command_abs_coordinate")
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            FlightSafetyStatus, self._str_param("status_topic"), status_qos
        )
        self._hold_pub = self.create_publisher(
            Bool, self._str_param("hold_topic"), status_qos
        )
        self._gated_command_pub = self.create_publisher(
            EnclosureCommandArray, self._str_param("gated_command_topic"), 10
        )
        self.create_subscription(
            EnclosureCommandArray,
            self._str_param("raw_command_topic"),
            self.on_command,
            10,
        )
        self.create_subscription(
            TargetTrackArray,
            self._str_param("target_topic"),
            self.on_target,
            10,
        )
        self.create_subscription(
            DroneStateArray,
            self._str_param("drone_states_topic"),
            self.on_drone_states,
            10,
        )
        mavros_topic = self._str_param("mavros_state_topic")
        self._mavros_sub = None
        if mavros_topic:
            self._mavros_sub = self.create_subscription(State, mavros_topic, self.on_mavros_state, 10)
        self.create_service(SafetyControl, self._str_param("control_service"), self.on_control)
        self.create_timer(max(self._float_param("status_period"), 0.05), self.on_timer)
        self._publish_status(self._now())
        self.get_logger().info(
            "flight safety supervisor started locked: "
            f"raw={self._str_param('raw_command_topic')} -> "
            f"gated={self._str_param('gated_command_topic')}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "raw_command_topic": "/enclosure_command",
            "gated_command_topic": "/flight_safety/enclosure_command",
            "status_topic": "/flight_safety/status",
            "hold_topic": "/flight_safety/hold_request",
            "control_service": "/flight_safety/control",
            "target_topic": "/target_track_world",
            "drone_states_topic": "/drone_states",
            "mavros_state_topic": "/uav0/mavros/state",
            "drone_state_timeout": 1.0,
            "target_timeout": 1.0,
            "command_timeout": 1.0,
            "mavros_timeout": 1.0,
            "max_command_future_skew": 0.25,
            "target_lock_observations": 2,
            "min_target_confidence": 0.5,
            "max_command_abs_coordinate": 10000.0,
            "require_mavros_connection": False,
            "require_target_lock_in_manual": False,
            "require_ground_confirmation_for_reset": True,
            "status_period": 0.1,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value).strip()

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int_param(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool_param(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def on_target(self, message: TargetTrackArray) -> None:
        now = self._now()
        source_stamp = _stamp_seconds(getattr(getattr(message, "header", None), "stamp", None))
        if (
            source_stamp is None
            or source_stamp > now + self._controller.max_command_future_skew
            or now - source_stamp > self._controller.target_timeout
        ):
            self._controller.observe_target(None, now=now)
            self._publish_status(now)
            return
        target_id = self._select_target(getattr(message, "tracks", []))
        self._controller.observe_target(target_id, now=now)
        self._publish_status(now)

    def _select_target(self, tracks: Iterable) -> int | None:
        min_confidence = self._float_param("min_target_confidence")
        eligible = [
            track
            for track in tracks
            if bool(getattr(track, "is_confirmed", False))
            and float(getattr(track, "confidence", 0.0)) >= min_confidence
        ]
        if not eligible:
            return None
        current = self._controller.snapshot(now=self._now()).locked_target_id
        if current is not None:
            for track in eligible:
                if int(getattr(track, "target_id", -1)) == current:
                    return current
        return int(max(eligible, key=lambda item: float(item.confidence)).target_id)

    def on_drone_states(self, message: DroneStateArray) -> None:
        states = list(getattr(message, "drones", []))
        self._controller.observe_drone_states(
            has_available_platform=any(bool(getattr(state, "available", False)) for state in states),
            now=self._now(),
        )
        self._publish_status(self._now())

    def on_mavros_state(self, message: State) -> None:
        self._controller.observe_mavros_state(connected=bool(message.connected), now=self._now())
        self._publish_status(self._now())

    def on_command(self, message: EnclosureCommandArray) -> None:
        now = self._now()
        source_stamp = _stamp_seconds(getattr(getattr(message, "header", None), "stamp", None))
        sequence = int(getattr(message, "sequence", 0))
        accepted = self._controller.observe_command(
            sequence=sequence,
            source_stamp=source_stamp if source_stamp is not None else 0.0,
            valid_payload=source_stamp is not None and self._valid_command_payload(message),
            now=now,
        )
        snapshot = self._controller.tick(now=now)
        if accepted and snapshot.containment_enabled:
            self._gated_command_pub.publish(message)
        self._publish_snapshot(snapshot, now)

    def _valid_command_payload(self, message: EnclosureCommandArray) -> bool:
        return validate_enclosure_payload(
            getattr(message, "commands", []),
            max_abs_coordinate=self._max_command_abs_coordinate,
        )

    def on_control(self, request: SafetyControl.Request, response: SafetyControl.Response):
        now = self._now()
        expires_at = _stamp_seconds(request.expires_at)
        result = self._controller.request(
            int(request.command),
            session_id=int(request.session_id),
            request_id=int(request.request_id),
            expires_at=expires_at if expires_at is not None else 0.0,
            operator_id=str(request.operator_id),
            ground_confirmed=bool(request.ground_confirmed),
            now=now,
        )
        snapshot = self._controller.tick(now=now)
        response.accepted = bool(result.accepted)
        response.state = int(snapshot.state)
        response.activation_mode = int(snapshot.activation_mode)
        response.accepted_request_id = int(request.request_id) if result.accepted else 0
        response.reason = result.reason
        self._publish_snapshot(snapshot, now)
        return response

    def on_timer(self) -> None:
        self._publish_status(self._now())

    def _publish_status(self, now: float) -> None:
        self._publish_snapshot(self._controller.tick(now=now), now)

    def _publish_snapshot(self, snapshot: SafetySnapshot, now: float) -> None:
        status = FlightSafetyStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = int(snapshot.state)
        status.activation_mode = int(snapshot.activation_mode)
        status.containment_enabled = bool(snapshot.containment_enabled)
        status.hold_requested = bool(snapshot.hold_requested)
        status.target_locked = bool(snapshot.target_locked)
        status.locked_target_id = int(snapshot.locked_target_id or 0)
        status.drone_states_fresh = bool(snapshot.drone_states_fresh)
        status.command_fresh = bool(snapshot.command_fresh)
        status.mavros_fresh = bool(snapshot.mavros_fresh)
        status.mavros_connected = bool(snapshot.mavros_connected)
        status.session_id = int(snapshot.session_id)
        status.last_control_request_id = int(snapshot.last_control_request_id)
        status.last_command_sequence = int(snapshot.last_command_sequence)
        status.fault_mask = int(snapshot.fault_mask)
        status.reason = snapshot.reason
        self._status_pub.publish(status)
        hold = Bool()
        hold.data = bool(snapshot.hold_requested)
        self._hold_pub.publish(hold)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FlightSafetySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
