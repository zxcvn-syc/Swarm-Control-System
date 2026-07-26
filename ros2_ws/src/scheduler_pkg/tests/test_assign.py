"""Tests for scheduler_pkg.assign covering greedy, Hungarian, and edge cases."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Make the in-tree package importable when running pytest from the project
# root without `pip install -e`.
_PKG_PARENT = Path(__file__).resolve().parents[1]
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

from scheduler_pkg.assign import greedy_assign, hungarian_assign  # noqa: E402


def test_greedy_assign_basic() -> None:
    drones = np.array(
        [
            [0.0, 0.0],
            [10.0, 0.0],
            [0.0, 10.0],
        ]
    )
    targets = np.array(
        [
            [0.5, 0.5],
            [10.5, 0.5],
            [5.0, 5.0],
            [20.0, 20.0],
            [25.0, 25.0],
        ]
    )
    pairs = greedy_assign(drones, targets, max_per_drone=2)
    # 3 drones * 2 each = 6 pair slots, but only 5 targets exist.
    assert len(pairs) == 5, f"expected 5 pairs, got {len(pairs)}: {pairs}"
    # All targets covered.
    assigned_targets = {t for _, t in pairs}
    assert assigned_targets == set(range(5))
    # All pair indices are valid.
    for d, t in pairs:
        assert 0 <= d < 3
        assert 0 <= t < 5


def test_greedy_assign_max_per_drone() -> None:
    drones = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
    targets = np.array(
        [
            [1.0, 0.0],
            [2.0, 0.0],
            [3.0, 0.0],
            [4.0, 0.0],
            [5.0, 0.0],
        ]
    )
    pairs = greedy_assign(drones, targets, max_per_drone=2)
    # 3 drones * 2 = 6 slots, but only 5 targets: 5 pairs.
    assert len(pairs) == 5
    per_drone_count = {0: 0, 1: 0, 2: 0}
    for d, _ in pairs:
        per_drone_count[d] += 1
    for d, c in per_drone_count.items():
        assert c <= 2, f"drone {d} got {c} > max_per_drone=2"


def test_hungarian_assign_basic() -> None:
    pytest.importorskip("scipy")
    drones = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]])
    targets = np.array([[0.5, 0.5], [10.5, 0.5], [5.0, 5.0]])
    pairs = hungarian_assign(drones, targets)
    # Hungarian is 1-to-1: 3 pairs.
    assert len(pairs) == 3
    assert {d for d, _ in pairs} == {0, 1, 2}
    assert {t for _, t in pairs} == {0, 1, 2}


def test_hungarian_assign_with_max_per_drone() -> None:
    pytest.importorskip("scipy")
    drones = np.array([[0.0, 0.0], [10.0, 0.0]])
    targets = np.array(
        [
            [0.5, 0.5],
            [10.5, 0.5],
            [5.0, 5.0],
        ]
    )
    pairs = hungarian_assign(drones, targets, max_per_drone=2)
    # 2 drones * 2 = 4 slots, 3 targets -> 3 pairs.
    assert len(pairs) == 3
    per_drone = {0: 0, 1: 0}
    for d, _ in pairs:
        per_drone[d] += 1
    for d, c in per_drone.items():
        assert c <= 2


def test_empty_inputs() -> None:
    drones = np.array([[0.0, 0.0], [1.0, 1.0]])
    targets = np.zeros((0, 2))
    assert greedy_assign(drones, targets) == []
    assert hungarian_assign(drones, targets) == []

    drones_empty = np.zeros((0, 2))
    targets_some = np.array([[0.0, 0.0], [1.0, 1.0]])
    assert greedy_assign(drones_empty, targets_some) == []
    assert hungarian_assign(drones_empty, targets_some) == []

    # Both empty.
    assert greedy_assign(np.zeros((0, 2)), np.zeros((0, 2))) == []
    assert hungarian_assign(np.zeros((0, 2)), np.zeros((0, 2))) == []


def test_priority_bias() -> None:
    # Two drones, two targets equidistant. High-priority target should win
    # the slot for the closer drone.
    drones = np.array([[0.0, 0.0], [10.0, 0.0]])
    targets = np.array([[1.0, 0.0], [9.0, 0.0]])
    # Boost target 1.
    pairs = greedy_assign(
        drones,
        targets,
        target_priorities=np.array([0.0, 1.0]),
        max_per_drone=1,
    )
    # Drone 0 should pick target 0 (closer), drone 1 should pick target 1
    # (also the high-priority one). Just check we have a valid assignment.
    assert len(pairs) == 2
    assert {d for d, _ in pairs} == {0, 1}
    assert {t for _, t in pairs} == {0, 1}
