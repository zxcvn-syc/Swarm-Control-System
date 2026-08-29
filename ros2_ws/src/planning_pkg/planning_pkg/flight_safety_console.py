"""Operator-facing terminal console for the flight-safety supervisor.

The console intentionally exposes only the supervisor's command-gate state.
It neither publishes MAVROS setpoints nor offers arm, mode, RTL, or landing
controls.  Flight-critical actions remain on the configured PX4/RC path.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from swarm_interfaces.msg import FlightSafetyStatus
from swarm_interfaces.srv import SafetyControl


STATE_NAMES = {
    FlightSafetyStatus.STATE_LOCKED: "LOCKED",
    FlightSafetyStatus.STATE_MANUAL_READY: "MANUAL_READY",
    FlightSafetyStatus.STATE_AUTO_READY: "AUTO_READY",
    FlightSafetyStatus.STATE_ACTIVE: "ACTIVE",
    FlightSafetyStatus.STATE_FAULT: "FAULT",
    FlightSafetyStatus.STATE_EMERGENCY_HOLD: "EMERGENCY_HOLD",
}
MODE_NAMES = {
    FlightSafetyStatus.MODE_NONE: "NONE",
    FlightSafetyStatus.MODE_MANUAL: "MANUAL",
    FlightSafetyStatus.MODE_AUTO: "AUTO",
}
COMMANDS = {
    "enable-manual": SafetyControl.Request.COMMAND_ENABLE_MANUAL,
    "enable-auto": SafetyControl.Request.COMMAND_ENABLE_AUTO,
    "disable": SafetyControl.Request.COMMAND_DISABLE,
    "emergency-hold": SafetyControl.Request.COMMAND_EMERGENCY_HOLD,
    "reset-fault": SafetyControl.Request.COMMAND_RESET_FAULT,
}


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _set_stamp(stamp, seconds: float) -> None:
    seconds = max(float(seconds), 0.0)
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1_000_000_000.0))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nanosec


class FlightSafetyConsole(Node):
    """Read current status and send one replay-resistant operator request."""

    def __init__(self, status_topic: str, control_service: str) -> None:
        super().__init__("flight_safety_console")
        self._status: FlightSafetyStatus | None = None
        self._last_printed: tuple | None = None
        self._last_print_at = 0.0
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(FlightSafetyStatus, status_topic, self.on_status, status_qos)
        self._client = self.create_client(SafetyControl, control_service)

    def on_status(self, status: FlightSafetyStatus) -> None:
        self._status = status

    @property
    def status(self) -> FlightSafetyStatus | None:
        return self._status

    def render_status(self, *, force: bool = False) -> None:
        status = self._status
        if status is None:
            return
        fields = (
            int(status.state),
            int(status.activation_mode),
            bool(status.containment_enabled),
            bool(status.hold_requested),
            bool(status.target_locked),
            int(status.locked_target_id),
            bool(status.drone_states_fresh),
            bool(status.command_fresh),
            bool(status.mavros_fresh),
            bool(status.mavros_connected),
            int(status.fault_mask),
            str(status.reason),
        )
        now = time.monotonic()
        if not force and fields == self._last_printed and now - self._last_print_at < 1.0:
            return
        self._last_printed = fields
        self._last_print_at = now
        lock = str(status.locked_target_id) if status.target_locked else "-"
        print(
            "state={state} mode={mode} containment={containment} hold={hold} "
            "target_lock={target} drone_fresh={drone} command_fresh={command} "
            "mavros={mavros} fault=0x{fault:x} reason={reason}".format(
                state=STATE_NAMES.get(int(status.state), str(status.state)),
                mode=MODE_NAMES.get(int(status.activation_mode), str(status.activation_mode)),
                containment=bool(status.containment_enabled),
                hold=bool(status.hold_requested),
                target=lock,
                drone=bool(status.drone_states_fresh),
                command=bool(status.command_fresh),
                mavros=bool(status.mavros_connected and status.mavros_fresh),
                fault=int(status.fault_mask),
                reason=status.reason,
            ),
            flush=True,
        )

    def wait_for_status(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(timeout, 0.1)
        while self.status is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.status is not None

    def send_control(
        self,
        command: int,
        *,
        operator_id: str,
        ttl_seconds: float,
        ground_confirmed: bool,
        timeout: float,
    ) -> SafetyControl.Response:
        status = self.status
        if status is None:
            raise RuntimeError("flight-safety status is unavailable")
        if not self._client.wait_for_service(timeout_sec=max(timeout, 0.1)):
            raise RuntimeError("flight-safety control service is unavailable")
        request = SafetyControl.Request()
        request.command = int(command)
        request.session_id = int(status.session_id)
        last_request_id = int(status.last_control_request_id)
        if last_request_id >= (1 << 64) - 1:
            raise RuntimeError("request-id space exhausted; restart the supervisor")
        request.request_id = max(last_request_id + 1, time.time_ns())
        base_time = _stamp_seconds(status.header.stamp)
        if base_time <= 0.0:
            base_time = self.get_clock().now().nanoseconds / 1_000_000_000.0
        _set_stamp(request.expires_at, base_time + max(ttl_seconds, 1.0))
        request.operator_id = operator_id
        request.ground_confirmed = bool(ground_confirmed)
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(timeout, 0.1))
        if not future.done():
            raise RuntimeError("flight-safety control service timed out")
        error = future.exception()
        if error is not None:
            raise RuntimeError(f"flight-safety control service failed: {error}")
        return future.result()


def _arguments(args: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Observe or control the flight-safety gate")
    parser.add_argument(
        "command",
        choices=["watch", *COMMANDS],
        help="watch state or issue a single containment-gate command",
    )
    parser.add_argument("--operator-id", help="nonempty audited operator identifier")
    parser.add_argument("--ground-confirmed", action="store_true")
    parser.add_argument("--status-topic", default="/flight_safety/status")
    parser.add_argument("--control-service", default="/flight_safety/control")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--ttl", type=float, default=10.0)
    return parser.parse_known_args(args)


def main(args: list[str] | None = None) -> None:
    parsed, ros_args = _arguments(args)
    if parsed.command != "watch" and not (parsed.operator_id or "").strip():
        raise SystemExit("--operator-id is required for every control command")
    if parsed.command == "reset-fault" and not parsed.ground_confirmed:
        raise SystemExit("reset-fault requires --ground-confirmed")

    rclpy.init(args=ros_args)
    node = FlightSafetyConsole(parsed.status_topic, parsed.control_service)
    try:
        if not node.wait_for_status(parsed.timeout):
            raise SystemExit("flight-safety status was not received before timeout")
        node.render_status(force=True)
        if parsed.command == "watch":
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.25)
                node.render_status()
            return
        response = node.send_control(
            COMMANDS[parsed.command],
            operator_id=parsed.operator_id.strip(),
            ttl_seconds=parsed.ttl,
            ground_confirmed=parsed.ground_confirmed,
            timeout=parsed.timeout,
        )
        print(
            "accepted={accepted} state={state} mode={mode} request_id={request_id} reason={reason}".format(
                accepted=bool(response.accepted),
                state=STATE_NAMES.get(int(response.state), str(response.state)),
                mode=MODE_NAMES.get(int(response.activation_mode), str(response.activation_mode)),
                request_id=int(response.accepted_request_id),
                reason=response.reason,
            ),
            flush=True,
        )
        if not response.accepted:
            raise SystemExit(2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
