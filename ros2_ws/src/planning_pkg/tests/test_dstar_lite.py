"""Tests for ``planning_pkg.dstar_lite`` covering both static and dynamic
behaviour of the planner.

Critical assertion: the public method ``update_obstacles`` must produce a
plan that differs from the original when a wall is inserted into the
diagonal.  This is the contract our swarm depends on.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_PKG_PARENT = Path(__file__).resolve().parents[1]
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

from planning_pkg.dstar_lite import DStarLite  # noqa: E402


def _path_cost(path):
    """Total octile length of a path (Chebyshev + diagonal)."""
    cost = 0.0
    for (a, b), (c, d) in zip(path, path[1:]):
        dx = abs(a - c)
        dy = abs(b - d)
        cost += 1.0 if dx + dy == 1 else float(np.sqrt(2.0))
    return cost


# ---------------------------------------------------------------------------
# Static tests
# ---------------------------------------------------------------------------
def test_initial_plan_on_free_grid():
    grid = np.zeros((10, 10), dtype=np.int8)
    planner = DStarLite(grid, (0, 0), (9, 9))
    path = planner.plan()
    assert path, "empty path on free grid"
    assert path[0] == (0, 0)
    assert path[-1] == (9, 9)
    # Continuous moves.
    for (a, b), (c, d) in zip(path, path[1:]):
        assert max(abs(a - c), abs(b - d)) == 1


def test_obstacle_detour():
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[3, 1:7] = 1
    planner = DStarLite(grid, (0, 0), (7, 7))
    path = planner.plan()
    assert path, "expected a path around the wall"
    # No path node should occupy the wall cells.
    for x, y in path:
        if y == 3:
            assert x in (0, 7), f"path entered wall at (x={x}, y={y}): {path}"


def test_unreachable_goal_returns_empty():
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[:, 4] = 1   # vertical wall down the middle (with 1-cell bypass gaps)
    grid[0, 4] = 0   # top-left bypass only
    # Goal is bottom-right pocket sealed off from top-left bypass by walls.
    # We use a strict seal:
    grid = np.zeros((7, 7), dtype=np.int8)
    for x in range(0, 7):
        grid[3, x] = 1
    for y in range(0, 7):
        grid[y, 3] = 1
    planner = DStarLite(grid, (0, 0), (6, 6))
    path = planner.plan()
    assert path == []


def test_start_equals_goal():
    grid = np.zeros((5, 5), dtype=np.int8)
    planner = DStarLite(grid, (2, 2), (2, 2))
    path = planner.plan()
    assert path == [(2, 2)]


def test_goal_on_obstacle_does_not_crash():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[4, 4] = 1  # goal cell blocked; A* recovers via nearest free cell
    planner = DStarLite(grid, (0, 0), (4, 4))
    path = planner.plan()
    assert path and path[0] == (0, 0)


def test_initial_path_cost_matches_oracle():
    """Static plan must agree with the A* oracle on cost."""
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[3, 1:7] = 1
    planner = DStarLite(grid, (0, 0), (7, 7))
    path = planner.plan()
    from planning_pkg.astar import astar as _astar
    oracle = _astar(grid, (0, 0), (7, 7))
    assert _path_cost(path) == pytest.approx(_path_cost(oracle)), (
        f"D* Lite path cost differs from A*: {path} vs {oracle}"
    )


# ---------------------------------------------------------------------------
# Dynamic tests (the contract)
# ---------------------------------------------------------------------------
def test_update_obstacles_changes_path():
    """Inserting a wall must produce a different (or at least non-trivially altered) plan."""
    grid = np.zeros((10, 10), dtype=np.int8)
    planner = DStarLite(grid, (0, 0), (9, 9))
    initial = planner.plan()
    assert initial, "free-grid plan should succeed"

    # Drop a wall across the entire diagonal route (x=4, y=2..7).
    wall = [((4, y), 1) for y in range(1, 9)]
    planner.update_obstacles(wall)
    after = planner.get_path()
    assert after, "expected a path after dynamic update"
    assert tuple(initial) != tuple(after), (
        f"path unchanged after wall insert: {initial}"
    )
    # The new path must still end at the goal.
    assert after[-1] == (9, 9)
    # And it must avoid the freshly blocked column.
    for x, y in after:
        if x == 4 and 1 <= y <= 8:
            raise AssertionError(f"path crosses the new wall: {after}")


def test_update_obstacles_remove_block_recovers_path():
    """Removing a wall should yield a path no longer than before."""
    grid = np.zeros((10, 10), dtype=np.int8)
    # A short partial wall rather than a full column: blocks the
    # diagonal route but leaves enough bypass for the planner to find
    # a path through y=0 or y=9.
    for y in range(2, 8):
        grid[y, 4] = 1
    planner = DStarLite(grid, (0, 0), (9, 9))
    bad = planner.plan()
    assert bad, "planner must still find a path with partial wall"

    # Now clear the wall.
    cleared = [((4, y), 0) for y in range(2, 8)]
    planner.update_obstacles(cleared)
    recovered = planner.get_path()
    assert recovered and recovered[-1] == (9, 9)
    # Recovered path must be at most as long as the detour path.
    assert len(recovered) <= len(bad), (
        f"cleared path unexpectedly long: {recovered}"
    )


def test_update_obstacles_disconnects_goal():
    """Sealing off the goal after a successful plan must update ``get_path`` to ``[]``."""
    grid = np.zeros((6, 6), dtype=np.int8)
    planner = DStarLite(grid, (0, 0), (5, 5))
    initial = planner.plan()
    assert initial
    # Surround the goal with an unbroken loop of obstacles to disconnect it.
    seal_cells = set()
    for y in range(6):
        seal_cells.add((4, y))
        seal_cells.add((y, 4))
    seal = [(cell, 1) for cell in seal_cells]
    planner.update_obstacles(seal)
    assert planner.get_path() == []


def test_update_obstacles_no_op_is_idempotent():
    """Updating with cells whose state matches the grid must not corrupt the plan."""
    grid = np.zeros((6, 6), dtype=np.int8)
    planner = DStarLite(grid, (0, 0), (5, 5))
    initial = planner.plan()
    # Pass pairs whose state already matches the grid (all cells free).
    no_op = [((x, y), 0) for x in range(6) for y in range(6)]
    planner.update_obstacles(no_op)
    after = planner.get_path()
    assert tuple(after) == tuple(initial), (
        "no-op update_obstacles changed the path"
    )


# ---------------------------------------------------------------------------
# Robustness
# ---------------------------------------------------------------------------
def test_invalid_grid_shape():
    with pytest.raises(ValueError):
        DStarLite(np.zeros(5), (0, 0), (4, 4))


def test_update_obstacles_out_of_bounds_is_ignored():
    grid = np.zeros((5, 5), dtype=np.int8)
    planner = DStarLite(grid, (0, 0), (4, 4))
    initial = planner.plan()
    # Should not raise or corrupt state.
    planner.update_obstacles([((99, 99), 1)])
    assert planner.get_path() == initial
