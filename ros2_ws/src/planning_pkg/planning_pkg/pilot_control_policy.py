"""Fail-closed policy for intentional, human-issued MAVROS pilot commands."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum


class PilotAction(str, Enum):
    """The small, auditable command surface exposed by the web console."""

    ARM = "arm"
    DISARM = "disarm"
    POSITION = "position"
    ALTITUDE = "altitude"
    OFFBOARD = "offboard"


ACTION_CONFIRMATIONS = {
    PilotAction.ARM: "ARM",
    PilotAction.DISARM: "DISARM",
    PilotAction.POSITION: "POSCTL",
    PilotAction.ALTITUDE: "ALTCTL",
    PilotAction.OFFBOARD: "OFFBOARD",
}


@dataclass(frozen=True)
class MavrosSnapshot:
    """Latest FCU state as observed by the dashboard, never a command result."""

    available: bool
    connected: bool
    armed: bool
    mode: str
    age_seconds: float | None


@dataclass(frozen=True)
class SafetySnapshot:
    """Latest command-gate status used to constrain high-risk mode selection."""

    available: bool
    state_name: str
    containment_enabled: bool
    hold_requested: bool
    target_locked: bool
    age_seconds: float | None


@dataclass(frozen=True)
class PilotDecision:
    """Policy outcome plus the exact MAVROS request type to be issued."""

    allowed: bool
    reason: str
    action: str
    service: str | None = None
    arm_value: bool | None = None
    custom_mode: str | None = None

    def audit_fields(self) -> dict[str, object]:
        """Return JSON-safe data suitable for a command audit record."""

        return asdict(self)


def action_confirmation(action: str) -> str | None:
    """Return the exact phrase required for an action, if it is supported."""

    try:
        return ACTION_CONFIRMATIONS[PilotAction(action)]
    except ValueError:
        return None


def decide_pilot_action(
    action: str,
    *,
    confirmation: object,
    ground_confirmed: bool,
    mavros: MavrosSnapshot,
    safety: SafetySnapshot,
    mavros_max_age_seconds: float,
    safety_max_age_seconds: float,
    position_mode: str,
    altitude_mode: str,
    offboard_mode: str,
) -> PilotDecision:
    """Validate one explicit pilot action without contacting MAVROS.

    The policy intentionally keeps physical recovery with the RC/PX4 path.
    It does not expose takeoff, land, RTL, position targets, velocity targets,
    mission uploads, or PX4 parameter changes through the browser.
    """

    try:
        selected = PilotAction(action)
    except ValueError:
        return PilotDecision(False, "pilot_action_unsupported", str(action))

    expected_confirmation = ACTION_CONFIRMATIONS[selected]
    if confirmation != expected_confirmation:
        return PilotDecision(False, "pilot_confirmation_mismatch", selected.value)
    if not mavros.available or mavros.age_seconds is None:
        return PilotDecision(False, "pilot_mavros_state_unavailable", selected.value)
    if mavros.age_seconds > mavros_max_age_seconds:
        return PilotDecision(False, "pilot_mavros_state_stale", selected.value)
    if not mavros.connected:
        return PilotDecision(False, "pilot_mavros_disconnected", selected.value)

    if selected is PilotAction.ARM:
        if mavros.armed:
            return PilotDecision(False, "pilot_vehicle_already_armed", selected.value)
        if str(mavros.mode).strip().upper() == str(offboard_mode).strip().upper():
            return PilotDecision(False, "pilot_arm_offboard_rejected", selected.value)
        if not _safety_locked_for_arm(safety, safety_max_age_seconds):
            return PilotDecision(False, "pilot_safety_gate_not_locked", selected.value)
        return PilotDecision(True, "pilot_arm_request_ready", selected.value, "arm", True)

    if selected is PilotAction.DISARM:
        if not ground_confirmed:
            return PilotDecision(False, "pilot_ground_confirmation_required", selected.value)
        if not mavros.armed:
            return PilotDecision(False, "pilot_vehicle_already_disarmed", selected.value)
        return PilotDecision(True, "pilot_disarm_request_ready", selected.value, "arm", False)

    if selected is PilotAction.POSITION:
        return _mode_decision(selected, position_mode)
    if selected is PilotAction.ALTITUDE:
        return _mode_decision(selected, altitude_mode)

    if not mavros.armed:
        return PilotDecision(False, "pilot_offboard_requires_armed", selected.value)
    if not _safety_active_for_offboard(safety, safety_max_age_seconds):
        return PilotDecision(False, "pilot_offboard_safety_gate_inactive", selected.value)
    return _mode_decision(selected, offboard_mode)


def _mode_decision(action: PilotAction, mode: str) -> PilotDecision:
    requested = str(mode).strip()
    if not requested:
        return PilotDecision(False, "pilot_mode_not_configured", action.value)
    return PilotDecision(
        True,
        "pilot_mode_request_ready",
        action.value,
        "mode",
        custom_mode=requested,
    )


def _safety_locked_for_arm(safety: SafetySnapshot, max_age_seconds: float) -> bool:
    return (
        safety.available
        and safety.age_seconds is not None
        and safety.age_seconds <= max_age_seconds
        and safety.state_name == "LOCKED"
        and safety.hold_requested
    )


def _safety_active_for_offboard(safety: SafetySnapshot, max_age_seconds: float) -> bool:
    return (
        safety.available
        and safety.age_seconds is not None
        and safety.age_seconds <= max_age_seconds
        and safety.state_name == "ACTIVE"
        and safety.containment_enabled
        and not safety.hold_requested
        and safety.target_locked
    )
