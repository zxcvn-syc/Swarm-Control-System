"""Node-level tests that run without a sourced ROS2 environment."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

_PKG_PARENT = Path(__file__).resolve().parents[1]
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

import rclpy

import scheduler_pkg.scheduler_node as scheduler_node
from scheduler_pkg.scheduler_node import SchedulerNode, normalize_strategy, uint32


# ============================================================
# rclpy fixture — initializes ROS2 before any test runs
# ============================================================
@pytest.fixture(autouse=True)
def rclpy_fixture():
    """Ensure rclpy is initialized before each test and shutdown after."""
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def track(target_id, x, y, confidence=0.5, is_confirmed=False):
    return SimpleNamespace(
        target_id=target_id,
        x=x,
        y=y,
        confidence=confidence,
        is_confirmed=is_confirmed,
    )


def drone(drone_id, x, y, available=True, platform_type=0):
    return SimpleNamespace(
        drone_id=drone_id,
        x=x,
        y=y,
        available=available,
        platform_type=platform_type,
    )


def target_array(*tracks):
    return SimpleNamespace(tracks=list(tracks))


def drone_array(*drones):
    return SimpleNamespace(drones=list(drones))


def make_node(monkeypatch, strategy="greedy"):
    node = SchedulerNode()
    node._parameters["assignment_strategy"] = strategy
    node.strategy = normalize_strategy(strategy)
    return node


def test_on_target_replaces_snapshot_and_calculates_priority(monkeypatch):
    node = make_node(monkeypatch)
    node.on_target(target_array(track(7, 1.25, -2.0, 0.8, True)))
    assert node._targets == {7: (1.25, -2.0, pytest.approx(0.9))}

    node.on_target(target_array(track(8, 3.0, 4.0, 0.2, False)))
    assert set(node._targets) == {8}


def test_on_drone_replaces_snapshot_and_filters_unavailable(monkeypatch):
    node = make_node(monkeypatch)
    node.on_drone(
        drone_array(
            drone(1, 2.0, 3.0),
            drone(2, 9.0, 9.0, False, platform_type=1),
        )
    )
    assert node._drones == {1: (2.0, 3.0, 0)}

    node.on_drone(drone_array())
    assert node._drones == {}


def test_empty_drone_snapshot_seeds_default_grid(monkeypatch):
    node = make_node(monkeypatch)
    node.num_drones = 8
    node._seed_default_drones()
    assert len(node._drones) == 8
    assert node._drones[0] == (0.0, 0.0, 0)
    assert node._drones[7] == (5.0, 10.0, 0)


def test_tick_publishes_one_assignment_per_target(monkeypatch):
    node = make_node(monkeypatch)
    published = []
    node.pub_task.publish = published.append
    node.on_target(target_array(track(10, 0.1, 0.1), track(20, 5.0, 0.0)))
    node.on_drone(drone_array(drone(3, 0.0, 0.0), drone(4, 10.0, 0.0)))

    node.tick()

    assert len(published) == 2
    assert {message.target_id for message in published} == {10, 20}
    assert {message.drone_id for message in published} <= {3, 4}


def test_auction_uses_sorted_drone_ids_for_platform_mapping(monkeypatch):
    node = make_node(monkeypatch, strategy="auction")
    node._drones = {
        2: (20.0, 0.0, 1),
        1: (10.0, 0.0, 0),
    }
    node._targets = {10: (1.0, 1.0, 1.0)}
    observed_agents = []

    class FakeAuctionEngine:
        def __init__(self, agents, _tasks):
            observed_agents.extend(agents)

        def bid_allocation(self):
            return {"T010": "UGV2"}

    monkeypatch.setattr(scheduler_node, "AuctionEngine", FakeAuctionEngine)

    pairs = node._run_auction_assign(
        [1, 2],
        None,
        [10],
        None,
        None,
    )

    assert [agent.aid for agent in observed_agents] == ["UAV1", "UGV2"]
    assert pairs == [(1, 0)]


def test_tick_publishes_platform_specific_task_types(monkeypatch):
    node = make_node(monkeypatch, strategy="auction")
    node._drones = {
        2: (20.0, 0.0, 1),
        1: (10.0, 0.0, 0),
    }
    node._targets = {
        10: (1.0, 1.0, 1.0),
        20: (2.0, 2.0, 1.0),
    }
    node._run_auction_assign = lambda *_args: [(0, 0), (1, 1)]
    published = []
    node.pub_task.publish = published.append

    node.tick()

    tasks_by_drone = {message.drone_id: message.task_type for message in published}
    assert tasks_by_drone == {1: "track_aerial", 2: "track_ground"}


def test_uint32_wraps_negative_and_large_values():
    assert uint32(-1) == 0xFFFFFFFF
    assert uint32(0x1_0000_0001) == 1


def test_invalid_strategy_falls_back_to_greedy(monkeypatch):
    node = make_node(monkeypatch, strategy="not-a-strategy")
    assert node.strategy == "greedy"
    assert normalize_strategy("not-a-strategy") == "greedy"
