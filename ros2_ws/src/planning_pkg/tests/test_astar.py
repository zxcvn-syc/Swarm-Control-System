"""Tests for ``planning_pkg.astar`` (A* on a 2D occupancy grid).

We cover the canonical scenarios required by the team:

* straight-line free path (and diagonal-shortest)
* obstacle detour (wall in the middle, must deflect the route)
* unreachable goal (start and goal live in disjoint free regions)
* start == goal (trivial one-cell path)
* start on an obstacle (algorithm must pick a sane recovery)
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Make the in-tree package importable when running pytest from the
# project root without `pip install -e .`.  Mirror the same shim used by
# scheduler_pkg/containment_pkg.
_PKG_PARENT = Path(__file__).resolve().parents[1]
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

from planning_pkg.astar import astar  # noqa: E402


def test_straight_diagonal_path() -> None:
    """Free grid should give the 5-node diagonal path on 5x5."""
    grid = np.zeros((5, 5), dtype=np.int8)
    path = astar(grid, (0, 0), (4, 4))
    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)
    # 5 nodes is the diagonal Chebyshev optimum (length = max(dx, dy) + 1).
    assert len(path) == 5, path
    # Each step must be a valid 8-neighbour move.
    for (a, b), (c, d) in zip(path, path[1:]):
        assert max(abs(a - c), abs(b - d)) == 1


def test_straight_4_neighbour_path() -> None:
    """With ``diagonal=False`` we expect a Manhattan-style trip."""
    grid = np.zeros((3, 5), dtype=np.int8)
    path = astar(grid, (0, 0), (4, 0), diagonal=False)
    assert path[0] == (0, 0) and path[-1] == (4, 0)
    assert len(path) == 5  # 4 right moves plus the start node
    for (a, b), (c, d) in zip(path, path[1:]):
        assert abs(a - c) + abs(b - d) == 1  # Manhattan move


def test_obstacle_detour() -> None:
    """A middle wall leaves only the edges open; A* must take a detour."""
    grid = np.zeros((6, 6), dtype=np.int8)
    grid[2, 1:5] = 1  # segment leaving columns 0 and 5 free
    path = astar(grid, (0, 0), (5, 5))
    assert path and path[0] == (0, 0) and path[-1] == (5, 5)
    # Path must avoid the wall cells (row 2, cols 1..4).
    for x, y in path:
        if y == 2:
            assert x in (0, 5), (
                f"path crosses the wall at (x={x}, y={y}): {path}"
            )


def test_unreachable_goal_returns_empty() -> None:
    """Two disjoint free regions must yield ``[]`` rather than infinite-loop."""
    grid = np.ones((9, 9), dtype=np.int8)
    grid[0, 0] = 0  # start region
    grid[8, 8] = 0  # isolated goal cell
    grid[7, 8] = 0
    grid[8, 7] = 0
    grid[7, 7] = 0  # 2x2 island around (8, 8)
    assert astar(grid, (0, 0), (8, 8)) == []


def test_unreachable_4_neighbour_returns_empty() -> None:
    """Cross-shaped blockage forces a Manhattan-unreachable goal."""
    grid = np.zeros((5, 5), dtype=np.int8)
    # Block the only column that could carry us across.
    grid[1:4, 2] = 1  # vertical wall
    # Combine with row 0 / row 4 walls to seal top/bottom.
    grid[0, :] = 1
    grid[4, :] = 1
    assert astar(grid, (0, 1), (4, 1), diagonal=False) == []


def test_start_equals_goal_returns_single_cell() -> None:
    grid = np.zeros((4, 4), dtype=np.int8)
    assert astar(grid, (2, 2), (2, 2)) == [(2, 2)]


def test_start_on_obstacle_recovers_to_free_neighbour() -> None:
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 2] = 1  # start cell blocked
    path = astar(grid, (2, 2), (4, 4))
    # The algorithm finds the nearest free cell to the requested start
    # and routes from there.
    assert path and path[-1] == (4, 4)
    assert path[0] != (2, 2), f"start should have been relocated, got {path}"


def test_goal_on_obstacle_recovers_to_free_neighbour() -> None:
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 4] = 1  # goal cell blocked
    path = astar(grid, (0, 0), (2, 4))
    # The goal is relocated to (1, 3) or (3, 3) etc.; the algorithm
    # must produce some valid path ending at the relocated goal.
    assert path and path[0] == (0, 0)
    end_x, end_y = path[-1]
    assert int(grid[end_y, end_x]) == 0  # at a free cell


def test_starts_outside_grid_raises() -> None:
    grid = np.zeros((3, 3), dtype=np.int8)
    with pytest.raises(ValueError):
        astar(grid, (99, 0), (0, 0))


def test_non_2d_grid_raises() -> None:
    with pytest.raises(ValueError):
        astar(np.zeros(5), (0, 0), (4, 4))


def test_path_is_continuous_and_free() -> None:
    """For every random grid, the returned path must be 8-connected and free."""
    rng = np.random.default_rng(0xA57A)
    grid = (rng.random((20, 20)) < 0.20).astype(np.int8)  # 20% obstacles
    # Guarantee free start/end so we don't depend on recovery behaviour here.
    grid[0, 0] = 0
    grid[19, 19] = 0
    path = astar(grid, (0, 0), (19, 19))
    if not path:
        return  # unreachable; acceptable.
    for (a, b), (c, d) in zip(path, path[1:]):
        assert max(abs(a - c), abs(b - d)) == 1, f"non-contiguous: {path}"
        assert int(grid[b, a]) == 0, f"path goes through obstacle: {path}"
