"""Deterministic failure-transition tests for the flight safety gate."""

from planning_pkg.flight_safety import (
    ActivationMode,
    Fault,
    FlightSafetyController,
    SafetyCommand,
    SafetyState,
    validate_enclosure_payload,
)
from types import SimpleNamespace


def _healthy(controller: FlightSafetyController, now: float) -> None:
    controller.observe_drone_states(has_available_platform=True, now=now)
    controller.observe_mavros_state(connected=True, now=now)


def _enable_manual(controller: FlightSafetyController, now: float, request_id: int = 1) -> None:
    result = controller.request(
        SafetyCommand.ENABLE_MANUAL,
        session_id=controller.session_id,
        request_id=request_id,
        expires_at=now + 5.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=now,
    )
    assert result.accepted


def _fresh_command(controller: FlightSafetyController, now: float, sequence: int) -> None:
    assert controller.observe_command(
        sequence=sequence,
        source_stamp=now,
        valid_payload=True,
        now=now,
    )


def test_starts_locked_and_rejects_control_replay():
    controller = FlightSafetyController()
    _healthy(controller, 1.0)
    _enable_manual(controller, 1.0)
    replay = controller.request(
        SafetyCommand.DISABLE,
        session_id=controller.session_id,
        request_id=1,
        expires_at=8.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=2.0,
    )

    assert controller.state == SafetyState.MANUAL_READY
    assert not replay.accepted
    assert replay.reason == "control_request_replayed"


def test_control_from_a_previous_supervisor_session_is_rejected():
    controller = FlightSafetyController(session_id=77)
    _healthy(controller, 1.0)

    result = controller.request(
        SafetyCommand.ENABLE_MANUAL,
        session_id=76,
        request_id=1,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=1.0,
    )

    assert not result.accepted
    assert result.reason == "control_session_mismatch"
    assert controller.state == SafetyState.LOCKED


def test_manual_activation_requires_new_command_after_enablement():
    controller = FlightSafetyController()
    _healthy(controller, 1.0)
    _fresh_command(controller, 1.0, sequence=1)
    _enable_manual(controller, 1.1)

    assert controller.tick(now=1.1).state == SafetyState.MANUAL_READY
    _fresh_command(controller, 1.2, sequence=2)
    snapshot = controller.tick(now=1.2)

    assert snapshot.state == SafetyState.ACTIVE
    assert snapshot.activation_mode == ActivationMode.MANUAL
    assert snapshot.containment_enabled
    assert not snapshot.hold_requested


def test_command_timeout_latches_fault_and_recloses_gate():
    controller = FlightSafetyController(command_timeout=0.5)
    _healthy(controller, 1.0)
    _enable_manual(controller, 1.0)
    _fresh_command(controller, 1.1, sequence=1)
    assert controller.tick(now=1.1).containment_enabled

    snapshot = controller.tick(now=1.7)

    assert snapshot.state == SafetyState.FAULT
    assert snapshot.hold_requested
    assert snapshot.fault_mask & Fault.COMMAND_STALE
    assert snapshot.reason == "enclosure_command_timeout"


def test_auto_mode_requires_stable_target_lock_and_faults_on_loss():
    controller = FlightSafetyController(command_timeout=1.0, target_timeout=0.5)
    _healthy(controller, 1.0)
    enabled = controller.request(
        SafetyCommand.ENABLE_AUTO,
        session_id=controller.session_id,
        request_id=1,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=1.0,
    )
    assert enabled.accepted
    controller.observe_target(42, now=1.1)
    _fresh_command(controller, 1.1, sequence=1)
    assert controller.tick(now=1.1).state == SafetyState.AUTO_READY

    controller.observe_target(42, now=1.2)
    _fresh_command(controller, 1.2, sequence=2)
    assert controller.tick(now=1.2).state == SafetyState.ACTIVE
    _fresh_command(controller, 1.5, sequence=3)
    snapshot = controller.tick(now=1.8)

    assert snapshot.state == SafetyState.FAULT
    assert snapshot.fault_mask & Fault.TARGET_STALE
    assert snapshot.reason == "target_timeout"


def test_replayed_enclosure_command_is_never_forwarded():
    controller = FlightSafetyController()
    _healthy(controller, 1.0)
    _enable_manual(controller, 1.0)
    _fresh_command(controller, 1.1, sequence=8)
    assert controller.tick(now=1.1).state == SafetyState.ACTIVE

    assert not controller.observe_command(
        sequence=8,
        source_stamp=1.2,
        valid_payload=True,
        now=1.2,
    )

    snapshot = controller.tick(now=1.2)
    assert snapshot.state == SafetyState.FAULT
    assert snapshot.fault_mask & Fault.COMMAND_REPLAY
    assert snapshot.hold_requested


def test_future_dated_command_is_rejected_and_latches_fault():
    controller = FlightSafetyController(max_command_future_skew=0.1)
    _healthy(controller, 1.0)
    _enable_manual(controller, 1.0)

    accepted = controller.observe_command(
        sequence=1,
        source_stamp=1.2,
        valid_payload=True,
        now=1.0,
    )

    snapshot = controller.tick(now=1.0)
    assert not accepted
    assert snapshot.state == SafetyState.FAULT
    assert snapshot.fault_mask & Fault.COMMAND_INVALID
    assert snapshot.reason == "future_enclosure_command"


def test_emergency_hold_requires_confirmed_manual_reset():
    controller = FlightSafetyController()
    _healthy(controller, 1.0)
    _enable_manual(controller, 1.0)
    emergency = controller.request(
        SafetyCommand.EMERGENCY_HOLD,
        session_id=controller.session_id,
        request_id=2,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=1.1,
    )
    rejected_reset = controller.request(
        SafetyCommand.RESET_FAULT,
        session_id=controller.session_id,
        request_id=3,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=1.2,
    )
    accepted_reset = controller.request(
        SafetyCommand.RESET_FAULT,
        session_id=controller.session_id,
        request_id=4,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=True,
        now=1.3,
    )

    assert emergency.accepted
    assert controller.state == SafetyState.LOCKED
    assert not rejected_reset.accepted
    assert rejected_reset.reason == "ground_confirmation_required"
    assert accepted_reset.accepted


def test_required_mavros_connection_blocks_enablement_until_healthy():
    controller = FlightSafetyController(require_mavros_connection=True)
    controller.observe_drone_states(has_available_platform=True, now=1.0)

    denied = controller.request(
        SafetyCommand.ENABLE_MANUAL,
        session_id=controller.session_id,
        request_id=1,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=1.0,
    )
    controller.observe_mavros_state(connected=True, now=1.1)
    accepted = controller.request(
        SafetyCommand.ENABLE_MANUAL,
        session_id=controller.session_id,
        request_id=2,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=1.1,
    )

    assert not denied.accepted
    assert denied.reason == "mavros_state_timeout"
    assert accepted.accepted


def test_reserved_command_layer_allows_nan_standby_but_needs_active_target():
    standby = SimpleNamespace(
        layer=2,
        target_x=float("nan"),
        target_y=float("nan"),
        target_z=float("nan"),
        enclosure_radius=0.0,
    )
    active = SimpleNamespace(
        layer=0,
        target_x=10.0,
        target_y=-4.0,
        target_z=5.0,
        enclosure_radius=8.0,
    )

    assert validate_enclosure_payload([active, standby], max_abs_coordinate=100.0)
    assert not validate_enclosure_payload([standby], max_abs_coordinate=100.0)


def test_rejected_well_formed_control_request_cannot_be_reused():
    controller = FlightSafetyController()
    _healthy(controller, 1.0)
    denied = controller.request(
        SafetyCommand.RESET_FAULT,
        session_id=controller.session_id,
        request_id=1,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=True,
        now=1.0,
    )
    replay = controller.request(
        SafetyCommand.ENABLE_MANUAL,
        session_id=controller.session_id,
        request_id=1,
        expires_at=6.0,
        operator_id="safety_pilot",
        ground_confirmed=False,
        now=1.1,
    )

    assert not denied.accepted
    assert denied.reason == "reset_requires_fault_or_emergency_hold"
    assert not replay.accepted
    assert replay.reason == "control_request_replayed"
