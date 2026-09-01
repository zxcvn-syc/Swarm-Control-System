from __future__ import annotations

from perception_pkg.target_lock import (
    LockObservation,
    LockState,
    TargetLockConfig,
    TargetLockManager,
)


def _observation(target_id: int = 7, *, x: float = 0.0, measured: bool = True,
                 embedding=(1.0, 0.0)) -> LockObservation:
    return LockObservation(
        target_id=target_id,
        x=x,
        y=0.0,
        vx=0.0,
        vy=0.0,
        confidence=0.9,
        confirmed=True,
        measured=measured,
        covariance=(4.0, 0.0, 4.0),
        embedding=embedding,
    )


def _manager() -> TargetLockManager:
    return TargetLockManager(TargetLockConfig(
        acquire_frames=2,
        reacquire_frames=2,
        lock_missed_frames=1,
        suspect_timeout_frames=3,
        min_reid_similarity=0.8,
        covariance_growth_per_s=4.0,
    ))


def test_acquisition_requires_consecutive_measured_confirmed_frames() -> None:
    manager = _manager()
    manager.select_target(7)

    first = manager.update([_observation()], 1.0)
    second = manager.update([_observation()], 1.1)

    assert first.state is LockState.ACQUIRING
    assert not first.command_eligible
    assert second.state is LockState.LOCKED
    assert second.command_eligible


def test_prediction_only_track_never_remains_command_eligible() -> None:
    manager = _manager()
    manager.select_target(7)
    manager.update([_observation()], 1.0)
    manager.update([_observation()], 1.1)

    grace = manager.update([_observation(measured=False)], 1.2)
    suspect = manager.update([_observation(measured=False)], 1.3)

    assert grace.state is LockState.LOCKED
    assert not grace.command_eligible
    assert suspect.state is LockState.SUSPECT
    assert not suspect.command_eligible


def test_reacquisition_rejects_wrong_appearance_even_with_motion_match() -> None:
    manager = _manager()
    manager.select_target(7)
    manager.update([_observation()], 1.0)
    manager.update([_observation()], 1.1)
    manager.update([], 1.2)
    manager.update([], 1.3)

    decision = manager.update([_observation(9, embedding=(0.0, 1.0))], 1.4)

    assert decision.state is LockState.SUSPECT
    assert decision.selected_target_id == 7
    assert not decision.command_eligible


def test_reacquisition_requires_reid_and_motion_for_multiple_frames() -> None:
    manager = _manager()
    manager.select_target(7)
    manager.update([_observation()], 1.0)
    manager.update([_observation()], 1.1)
    manager.update([], 1.2)
    manager.update([], 1.3)

    first = manager.update([_observation(9)], 1.4)
    second = manager.update([_observation(9)], 1.5)

    assert first.state is LockState.SUSPECT
    assert not first.command_eligible
    assert second.state is LockState.LOCKED
    assert second.command_eligible
    assert second.selected_target_id == 9


def test_stale_timestamp_does_not_change_a_lock() -> None:
    manager = _manager()
    manager.select_target(7)
    manager.update([_observation()], 1.0)
    manager.update([_observation()], 1.1)

    decision = manager.update([], 1.0)

    assert decision.state is LockState.LOCKED
    assert decision.reason == "stale_timestamp"
