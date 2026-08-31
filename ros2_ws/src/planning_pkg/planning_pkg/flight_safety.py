"""Pure safety state machine for the ROS2 flight-safety supervisor.

The controller deliberately controls only the *software command gate*.  It
never arms, disarms, changes a PX4 flight mode, commands landing, or replaces
PX4/RC/geofence/battery failsafes.  Keeping this logic free of ROS makes the
failure transitions deterministic and directly unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import math
from typing import Iterable


class SafetyState(IntEnum):
    """Supervisor state exposed through ``FlightSafetyStatus``."""

    LOCKED = 0
    MANUAL_READY = 1
    AUTO_READY = 2
    ACTIVE = 3
    FAULT = 4
    EMERGENCY_HOLD = 5


class ActivationMode(IntEnum):
    """How the currently armed software gate may activate containment."""

    NONE = 0
    MANUAL = 1
    AUTO = 2


class SafetyCommand(IntEnum):
    """Commands accepted by the ``SafetyControl`` service."""

    ENABLE_MANUAL = 1
    ENABLE_AUTO = 2
    DISABLE = 3
    EMERGENCY_HOLD = 4
    RESET_FAULT = 5


class Fault(IntFlag):
    """Latched reasons that close the software command gate."""

    NONE = 0
    DRONE_STATES_STALE = 1
    NO_AVAILABLE_PLATFORM = 2
    MAVROS_STATE_STALE = 4
    MAVROS_DISCONNECTED = 8
    TARGET_STALE = 16
    TARGET_UNLOCKED = 32
    COMMAND_STALE = 64
    COMMAND_REPLAY = 128
    COMMAND_INVALID = 256
    EMERGENCY_HOLD = 512


@dataclass(frozen=True)
class ControlResult:
    """Outcome returned after a manual control request."""

    accepted: bool
    reason: str


def validate_enclosure_payload(
    commands: Iterable[object], *, max_abs_coordinate: float
) -> bool:
    """Validate active containment points while allowing reserved standby entries.

    Layer 2 is the existing human-command layer.  Its producer deliberately
    publishes NaN coordinates while that platform waits for manual override,
    so it is not a flight target and must not make an otherwise valid plan
    fail.  Every active monitor/block point still has to be finite, bounded,
    and have a non-negative bounded radius.
    """

    try:
        limit = float(max_abs_coordinate)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(limit) or limit <= 0.0:
        return False

    active_commands = 0
    for command in commands:
        try:
            layer = int(command.layer)
        except (AttributeError, TypeError, ValueError):
            return False
        if layer == 2:
            continue
        if layer not in {0, 1}:
            return False
        try:
            x = float(command.target_x)
            y = float(command.target_y)
            z = float(command.target_z)
            radius = float(command.enclosure_radius)
        except (AttributeError, TypeError, ValueError):
            return False
        if not all(math.isfinite(value) for value in (x, y, z, radius)):
            return False
        if any(abs(value) > limit for value in (x, y, z)):
            return False
        if radius < 0.0 or radius > limit:
            return False
        active_commands += 1
    return active_commands > 0


@dataclass(frozen=True)
class SafetySnapshot:
    """All state needed by the ROS adapter to publish a status message."""

    state: SafetyState
    activation_mode: ActivationMode
    containment_enabled: bool
    hold_requested: bool
    target_locked: bool
    locked_target_id: int | None
    drone_states_fresh: bool
    command_fresh: bool
    mavros_fresh: bool
    mavros_connected: bool
    session_id: int
    last_control_request_id: int
    last_command_sequence: int
    fault_mask: Fault
    reason: str


class FlightSafetyController:
    """Fail-closed supervisor for a stream of enclosure commands.

    A caller must explicitly enable manual or automatic containment using a
    fresh, monotonically increasing request id.  Commands are forwarded only
    during ``ACTIVE`` and only while all configured safety observations are
    fresh.  Emergency hold and faults latch until a separate, newer reset.
    """

    def __init__(
        self,
        *,
        drone_state_timeout: float = 1.0,
        target_timeout: float = 1.0,
        command_timeout: float = 1.0,
        mavros_timeout: float = 1.0,
        max_command_future_skew: float = 0.25,
        target_lock_observations: int = 2,
        require_mavros_connection: bool = False,
        require_target_lock_in_manual: bool = False,
        require_ground_confirmation_for_reset: bool = True,
        session_id: int = 1,
    ) -> None:
        self.drone_state_timeout = max(float(drone_state_timeout), 0.01)
        self.target_timeout = max(float(target_timeout), 0.01)
        self.command_timeout = max(float(command_timeout), 0.01)
        self.mavros_timeout = max(float(mavros_timeout), 0.01)
        self.max_command_future_skew = max(float(max_command_future_skew), 0.0)
        self.target_lock_observations = max(int(target_lock_observations), 1)
        self.require_mavros_connection = bool(require_mavros_connection)
        self.require_target_lock_in_manual = bool(require_target_lock_in_manual)
        self.require_ground_confirmation_for_reset = bool(
            require_ground_confirmation_for_reset
        )
        self.session_id = max(int(session_id), 1)

        self.state = SafetyState.LOCKED
        self.activation_mode = ActivationMode.NONE
        self.fault_mask = Fault.NONE
        self.reason = "startup_locked"
        self._last_control_request_id = 0
        self._last_command_sequence = 0
        self._last_command_at: float | None = None
        self._last_drone_states_at: float | None = None
        self._has_available_platform = False
        self._last_mavros_at: float | None = None
        self._mavros_connected = False
        self._last_target_at: float | None = None
        self._target_candidate_id: int | None = None
        self._target_observations = 0
        self._locked_target_id: int | None = None

    def observe_drone_states(self, *, has_available_platform: bool, now: float) -> None:
        self._last_drone_states_at = float(now)
        self._has_available_platform = bool(has_available_platform)

    def observe_mavros_state(self, *, connected: bool, now: float) -> None:
        self._last_mavros_at = float(now)
        self._mavros_connected = bool(connected)

    def observe_target(self, target_id: int | None, *, now: float) -> None:
        """Update target-lock evidence from one fresh tracker sample."""

        self._last_target_at = float(now)
        if target_id is None:
            self._target_candidate_id = None
            self._target_observations = 0
            self._locked_target_id = None
            return
        target_id = int(target_id)
        if target_id == self._target_candidate_id:
            self._target_observations += 1
        else:
            self._target_candidate_id = target_id
            self._target_observations = 1
        if self._target_observations >= self.target_lock_observations:
            self._locked_target_id = target_id

    def observe_command(
        self,
        *,
        sequence: int,
        source_stamp: float,
        valid_payload: bool,
        now: float,
    ) -> bool:
        """Accept one source command heartbeat when it is fresh and monotonic."""

        now = float(now)
        if not valid_payload:
            self._fault(Fault.COMMAND_INVALID, "invalid_enclosure_command")
            return False
        if source_stamp > now + self.max_command_future_skew:
            self._fault(Fault.COMMAND_INVALID, "future_enclosure_command")
            return False
        if now - source_stamp > self.command_timeout:
            self._fault(Fault.COMMAND_STALE, "stale_enclosure_command")
            return False
        if int(sequence) <= self._last_command_sequence:
            self._fault(Fault.COMMAND_REPLAY, "replayed_enclosure_command")
            return False

        self._last_command_sequence = int(sequence)
        self._last_command_at = now
        return True

    def request(
        self,
        command: SafetyCommand | int,
        *,
        session_id: int,
        request_id: int,
        expires_at: float,
        operator_id: str,
        ground_confirmed: bool,
        now: float,
    ) -> ControlResult:
        """Apply one authenticated-at-the-ROS-boundary operator request.

        Monotonic ids and expiry prevent accidental/replayed requests.  They
        are not a substitute for DDS authentication/authorization, which is
        required to protect against a malicious ROS graph participant.
        """

        now = float(now)
        try:
            command = SafetyCommand(int(command))
        except (TypeError, ValueError):
            return ControlResult(False, "unknown_control_command")
        if not str(operator_id).strip():
            return ControlResult(False, "operator_id_required")
        if int(session_id) != self.session_id:
            return ControlResult(False, "control_session_mismatch")
        if int(request_id) <= self._last_control_request_id:
            return ControlResult(False, "control_request_replayed")
        if float(expires_at) <= now:
            return ControlResult(False, "control_request_expired")

        # Any syntactically valid request consumes its id, including a denied
        # one.  A client cannot retry the same id after changing its payload.
        self._last_control_request_id = int(request_id)

        if command == SafetyCommand.EMERGENCY_HOLD:
            self._fault(Fault.EMERGENCY_HOLD, "operator_emergency_hold")
            self.state = SafetyState.EMERGENCY_HOLD
            return ControlResult(True, self.reason)

        if command == SafetyCommand.RESET_FAULT:
            if self.state not in {SafetyState.FAULT, SafetyState.EMERGENCY_HOLD}:
                return ControlResult(False, "reset_requires_fault_or_emergency_hold")
            if self.require_ground_confirmation_for_reset and not ground_confirmed:
                return ControlResult(False, "ground_confirmation_required")
            self.state = SafetyState.LOCKED
            self.activation_mode = ActivationMode.NONE
            self.fault_mask = Fault.NONE
            self.reason = "operator_reset_locked"
            self._last_command_at = None
            return ControlResult(True, self.reason)

        if command == SafetyCommand.DISABLE:
            if self.state == SafetyState.EMERGENCY_HOLD:
                return ControlResult(False, "emergency_hold_requires_reset")
            self.state = SafetyState.LOCKED
            self.activation_mode = ActivationMode.NONE
            self.reason = "operator_disabled"
            self._last_command_at = None
            return ControlResult(True, self.reason)

        if self.state in {SafetyState.FAULT, SafetyState.EMERGENCY_HOLD}:
            return ControlResult(False, "fault_or_emergency_hold_requires_reset")
        fault = self._base_fault(now)
        if fault != Fault.NONE:
            return ControlResult(False, self._fault_reason(fault))

        self._last_command_at = None  # A plan must arrive after operator enablement.
        if command == SafetyCommand.ENABLE_MANUAL:
            self.state = SafetyState.MANUAL_READY
            self.activation_mode = ActivationMode.MANUAL
            self.reason = "operator_enabled_manual"
            return ControlResult(True, self.reason)
        if command == SafetyCommand.ENABLE_AUTO:
            self.state = SafetyState.AUTO_READY
            self.activation_mode = ActivationMode.AUTO
            self.reason = "operator_enabled_auto"
            return ControlResult(True, self.reason)
        return ControlResult(False, "unsupported_control_command")

    def tick(self, *, now: float) -> SafetySnapshot:
        """Evaluate timeouts and return the current fail-closed decision."""

        now = float(now)
        if self.state in {SafetyState.LOCKED, SafetyState.FAULT, SafetyState.EMERGENCY_HOLD}:
            return self.snapshot(now=now)

        fault = self._base_fault(now)
        if fault != Fault.NONE:
            self._fault(fault, self._fault_reason(fault))
            return self.snapshot(now=now)

        target_required = self.activation_mode == ActivationMode.AUTO or (
            self.activation_mode == ActivationMode.MANUAL
            and self.require_target_lock_in_manual
        )
        target_fresh = self._target_is_fresh(now)
        target_locked = target_fresh and self._locked_target_id is not None
        if self.state == SafetyState.ACTIVE:
            if not self._command_is_fresh(now):
                self._fault(Fault.COMMAND_STALE, "enclosure_command_timeout")
            elif target_required and not target_fresh:
                self._fault(Fault.TARGET_STALE, "target_timeout")
            elif target_required and not target_locked:
                self._fault(Fault.TARGET_UNLOCKED, "target_lock_lost")
        elif self._command_is_fresh(now) and (not target_required or target_locked):
            self.state = SafetyState.ACTIVE
            self.reason = "containment_command_active"
        return self.snapshot(now=now)

    def snapshot(self, *, now: float) -> SafetySnapshot:
        """Return an immutable status view without changing state."""

        target_fresh = self._target_is_fresh(now)
        active = self.state == SafetyState.ACTIVE
        return SafetySnapshot(
            state=self.state,
            activation_mode=self.activation_mode,
            containment_enabled=active,
            hold_requested=not active,
            target_locked=target_fresh and self._locked_target_id is not None,
            locked_target_id=self._locked_target_id if target_fresh else None,
            drone_states_fresh=self._drone_states_are_fresh(now),
            command_fresh=self._command_is_fresh(now),
            mavros_fresh=self._mavros_is_fresh(now),
            mavros_connected=self._mavros_connected,
            session_id=self.session_id,
            last_control_request_id=self._last_control_request_id,
            last_command_sequence=self._last_command_sequence,
            fault_mask=self.fault_mask,
            reason=self.reason,
        )

    def _base_fault(self, now: float) -> Fault:
        if not self._drone_states_are_fresh(now):
            return Fault.DRONE_STATES_STALE
        if not self._has_available_platform:
            return Fault.NO_AVAILABLE_PLATFORM
        if self.require_mavros_connection:
            if not self._mavros_is_fresh(now):
                return Fault.MAVROS_STATE_STALE
            if not self._mavros_connected:
                return Fault.MAVROS_DISCONNECTED
        return Fault.NONE

    def _drone_states_are_fresh(self, now: float) -> bool:
        return self._is_fresh(self._last_drone_states_at, self.drone_state_timeout, now)

    def _target_is_fresh(self, now: float) -> bool:
        return self._is_fresh(self._last_target_at, self.target_timeout, now)

    def _command_is_fresh(self, now: float) -> bool:
        return self._is_fresh(self._last_command_at, self.command_timeout, now)

    def _mavros_is_fresh(self, now: float) -> bool:
        return self._is_fresh(self._last_mavros_at, self.mavros_timeout, now)

    @staticmethod
    def _is_fresh(timestamp: float | None, timeout: float, now: float) -> bool:
        return timestamp is not None and 0.0 <= now - timestamp <= timeout

    def _fault(self, fault: Fault, reason: str) -> None:
        self.fault_mask |= fault
        self.state = SafetyState.FAULT
        self.activation_mode = ActivationMode.NONE
        self.reason = reason

    @staticmethod
    def _fault_reason(fault: Fault) -> str:
        return {
            Fault.DRONE_STATES_STALE: "drone_states_timeout",
            Fault.NO_AVAILABLE_PLATFORM: "no_available_platform",
            Fault.MAVROS_STATE_STALE: "mavros_state_timeout",
            Fault.MAVROS_DISCONNECTED: "mavros_disconnected",
            Fault.TARGET_STALE: "target_timeout",
            Fault.TARGET_UNLOCKED: "target_lock_lost",
            Fault.COMMAND_STALE: "enclosure_command_timeout",
            Fault.COMMAND_REPLAY: "replayed_enclosure_command",
            Fault.COMMAND_INVALID: "invalid_enclosure_command",
        }.get(fault, "flight_safety_fault")
