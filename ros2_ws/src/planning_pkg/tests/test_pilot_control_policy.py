from planning_pkg.pilot_control_policy import (
    MavrosSnapshot,
    SafetySnapshot,
    action_confirmation,
    decide_pilot_action,
)


def _mavros(*, armed=False, mode="POSCTL"):
    return MavrosSnapshot(True, True, armed, mode, 0.1)


def _safety(*, state="LOCKED", hold=True, active=False, target=False):
    return SafetySnapshot(True, state, active, hold, target, 0.1)


def _decide(action, **kwargs):
    return decide_pilot_action(
        action,
        confirmation=kwargs.pop("confirmation", action_confirmation(action)),
        ground_confirmed=kwargs.pop("ground_confirmed", False),
        mavros=kwargs.pop("mavros", _mavros()),
        safety=kwargs.pop("safety", _safety()),
        mavros_max_age_seconds=1.0,
        safety_max_age_seconds=1.0,
        position_mode="POSCTL",
        altitude_mode="ALTCTL",
        offboard_mode="OFFBOARD",
        **kwargs,
    )


def test_arm_requires_exact_confirmation_and_locked_safety_gate():
    mismatch = _decide("arm", confirmation="arm")
    unlocked = _decide("arm", safety=_safety(state="ACTIVE", hold=False, active=True))
    allowed = _decide("arm")

    assert mismatch.reason == "pilot_confirmation_mismatch"
    assert unlocked.reason == "pilot_safety_gate_not_locked"
    assert allowed.allowed
    assert allowed.service == "arm"
    assert allowed.arm_value is True


def test_disarm_requires_explicit_ground_confirmation():
    unsafe = _decide("disarm", mavros=_mavros(armed=True))
    allowed = _decide("disarm", mavros=_mavros(armed=True), ground_confirmed=True)

    assert unsafe.reason == "pilot_ground_confirmation_required"
    assert allowed.allowed
    assert allowed.arm_value is False


def test_recovery_modes_only_require_fresh_connected_mavros():
    position = _decide("position")
    altitude = _decide("altitude")
    stale = _decide("position", mavros=MavrosSnapshot(True, True, True, "OFFBOARD", 2.0))

    assert position.allowed and position.custom_mode == "POSCTL"
    assert altitude.allowed and altitude.custom_mode == "ALTCTL"
    assert stale.reason == "pilot_mavros_state_stale"


def test_offboard_requires_armed_active_unheld_target_locked_safety_gate():
    inactive = _decide("offboard", mavros=_mavros(armed=True))
    allowed = _decide(
        "offboard",
        mavros=_mavros(armed=True),
        safety=_safety(state="ACTIVE", hold=False, active=True, target=True),
    )

    assert inactive.reason == "pilot_offboard_safety_gate_inactive"
    assert allowed.allowed
    assert allowed.service == "mode"
    assert allowed.custom_mode == "OFFBOARD"
