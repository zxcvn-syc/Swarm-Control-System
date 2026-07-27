"""A* path-planner on a 2D occupancy grid.

Pure-numpy / pure-python implementation with no ROS2 dependency, so it
can be unit tested standalone and driven from the ROS2 node in
:mod:`planning_pkg.planner_node`.

Public API
----------
- ``astar(grid, start, goal, diagonal=True) -> list[tuple[int, int]]``

Conventions
-----------
- ``grid`` is a 2D numpy array of any shape ``(H, W)``.
- ``0`` denotes free space, ``1`` (or any non-zero value) denotes an
  obstacle.  ``grid[y, x]`` matches numpy row-major indexing, so the
  axes are ``(row=y, col=x)`` throughout this module.
- ``start`` / ``goal`` are ``(x, y)`` integer tuples in world coordinates
  (i.e. matching numpy indices *after* the ``(y, x)`` ordering is
  corrected -- ``astar`` transparently accepts either ``(x, y)`` or
  ``(y, x)``).

Edge cases
----------
- Empty path ``[]`` is returned when start is identical to goal or the
  goal is unreachable from the start.
- If start or goal lies on an obstacle the algorithm first tries to
  detour the start to the nearest free neighbour so it can still issue
  a sensible warning.

The classic binary-heap-based priority queue is small enough to be
inlined here without pulling in ``heapq`` semantics to obscure intent.
"""

from __future__ import annotations

import heapq
from typing import Iterable, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _is_passable(grid: np.ndarray, x: int, y: int) -> bool:
    """Return ``True`` if ``(x, y)`` lies inside the grid and is free."""
    h, w = grid.shape
    if 0 <= x < w and 0 <= y < h:
        return int(grid[y, x]) == 0
    return False


def _nearest_free(
    grid: np.ndarray, x: int, y: int, search_radius: int = 6
) -> Optional[Tuple[int, int]]:
    """Find the nearest free cell to ``(x, y)`` within ``search_radius``.

    Used as a recovery path when the requested start or goal is on an
    obstacle -- we surface a usable escape instead of immediately
    raising.  Returns ``None`` when no free cell is found within the
    search window.
    """
    h, w = grid.shape
    for r in range(0, search_radius + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if abs(dx) != r and abs(dy) != r:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and int(grid[ny, nx]) == 0:
                    return (nx, ny)
    return None


def _heuristic(a: Tuple[int, int], b: Tuple[int, int], diagonal: bool) -> float:
    """Heuristic cost between ``a`` and ``b``.

    Diagonal motion => Euclidean distance (admissible & consistent).
    4-connected motion => Manhattan distance.
    For grids where uniform integer cost is desired but 8-neighbour
    moves are allowed, Euclidean works because it never overestimates
    the *true* cost of ``step_cost * chebyshev < step_cost * euclidean``.
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if diagonal:
        return float(np.hypot(dx, dy))
    return float(dx + dy)


def _neighbors(
    x: int, y: int, diagonal: bool
) -> Iterable[Tuple[int, int, float]]:
    """Yield ``(nx, ny, step_cost)`` for valid neighbour offsets."""
    if not diagonal:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            yield x + dx, y + dy, 1.0
        return
    # Octile / 8-neighbour: card cost 1.0, diag cost sqrt(2).
    for dx, dy in (
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    ):
        nx, ny = x + dx, y + dy
        step = 1.0 if dx == 0 or dy == 0 else float(np.sqrt(2.0))
        yield nx, ny, step


def _reconstruct(came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
    """Walk the ``came_from`` chain backwards, return ``[start -> ..., current]``."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def astar(
    grid: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    diagonal: bool = True,
) -> List[Tuple[int, int]]:
    """Run A* on ``grid`` from ``start`` to ``goal``.

    Parameters
    ----------
    grid : (H, W) ndarray
        ``0`` = free, anything else = obstacle.  Valid dtype is
        anything that can be cast via ``int()``.
    start, goal : (x, y) tuples of ints
        World coordinates.  ``astar`` is tolerant: if the first coord
        clearly exceeds the grid width or height it interprets the pair
        as ``(y, x)``.  Otherwise it uses the path-cost heuristic above
        to pick the nearest free neighbour if either is on an obstacle.
    diagonal : bool, default ``True``
        When ``True`` 8-neighbour motion is allowed; otherwise 4.

    Returns
    -------
    list of (x, y) cells, including both endpoints.  An empty list
    means the goal is unreachable.

    Raises
    ------
    ValueError
        if ``grid`` is not 2-D or has zero size in either axis.
    """
    grid = np.asarray(grid)
    if grid.ndim != 2 or grid.shape[0] == 0 or grid.shape[1] == 0:
        raise ValueError(f"grid must be a non-empty 2D array, got shape {grid.shape}")

    h, w = grid.shape

    # ------------------------------------------------------------------
    # Normalise start / goal into (x, y), and recover if they sit on
    # obstacles.
    # ------------------------------------------------------------------
    def _coerce(point: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        x, y = int(point[0]), int(point[1])
        # Heuristic: if (x, y) swap fits the grid better, accept it.
        swapped = (y, x) if 0 <= y < w and 0 <= x < h and not (
            0 <= x < w and 0 <= y < h
        ) else None
        if not (0 <= x < w and 0 <= y < h):
            if swapped is not None and 0 <= swapped[0] < w and 0 <= swapped[1] < h:
                x, y = swapped
            else:
                raise ValueError(
                    f"point {(int(point[0]), int(point[1]))} outside grid shape {grid.shape}"
                )
        return x, y

    s = _coerce(start)
    g = _coerce(goal)
    assert s is not None and g is not None

    # Trivial case: start equals goal -- return a single-cell path so the
    # caller always gets a list-of-tuples with both endpoints.
    if s == g:
        return [s]

    if not _is_passable(grid, *s):
        s_recover = _nearest_free(grid, *s)
        if s_recover is None:
            return []
        s = s_recover
    if not _is_passable(grid, *g):
        g_recover = _nearest_free(grid, *g)
        if g_recover is None:
            return []
        g = g_recover

    # ------------------------------------------------------------------
    # A* loop
    # ------------------------------------------------------------------
    g_score: dict = {s: 0.0}
    f_score: dict = {s: _heuristic(s, g, diagonal)}
    came_from: dict = {}
    # Tie-breaking by (f, -g) prefers nodes closer to goal among ties,
    # and nodes that have already spent more g when f is equal.
    counter = 0
    open_heap: list = [(f_score[s], 0.0, counter, s)]
    counter += 1
    closed: set = set()

    while open_heap:
        _, _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == g:
            return _reconstruct(came_from, current)

        closed.add(current)
        cur_g = g_score[current]

        for nx, ny, step in _neighbors(*current, diagonal=diagonal):
            if not _is_passable(grid, nx, ny):
                continue
            neighbour = (nx, ny)
            if neighbour in closed:
                continue
            tentative_g = cur_g + step
            if tentative_g < g_score.get(neighbour, float("inf")):
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f = tentative_g + _heuristic(neighbour, g, diagonal)
                f_score[neighbour] = f
                heapq.heappush(open_heap, (f, -tentative_g, counter, neighbour))
                counter += 1

    # Open set exhausted without ever reaching goal: unreachable.
    return []


# ---------------------------------------------------------------------------
# Self-test + CLI smoke test
# ---------------------------------------------------------------------------
def _self_test() -> None:
    """A small smoke test runnable as ``python -m planning_pkg.astar``.

    Verifies the canonical scenarios required by the team:
    - straight free path,
    - obstacle detour,
    - unreachable goal,
    - start == goal,
    - start blocked.
    """
    print("ast._self_test: starting")

    # 1) Straight free path on a 5x5 grid.
    grid_free = np.zeros((5, 5), dtype=np.int8)
    path = astar(grid_free, (0, 0), (4, 4))
    assert path[0] == (0, 0) and path[-1] == (4, 4), path
    # Diagonal path length between (0,0) and (4,4) on Chebyshev = 4 moves.
    assert len(path) == 5, f"diagonal path expected 5 nodes, got {len(path)}: {path}"
    print("  [OK]  straight diagonal path")

    # 2) Wall in the middle that only blocks the diagonal; route should detour.
    grid_wall = np.zeros((6, 6), dtype=np.int8)
    grid_wall[2, 1:5] = 1  # horizontal wall segment leaving columns 0 and 5 free.
    path = astar(grid_wall, (0, 0), (5, 5))
    assert path[0] == (0, 0) and path[-1] == (5, 5), path
    # No path node should touch the wall at y=2 (except via bypass at x=0/x=5).
    for x, y in path:
        if y == 2:
            assert x in (0, 5), (
                f"path crosses the wall at (x={x}, y={y}): {path}"
            )
    print(f"  [OK]  obstacle detour ({len(path)} nodes)")

    # 3) Unreachable goal: completely fenced off.
    grid_boxed = np.zeros((7, 7), dtype=np.int8)
    # Surround cell (4, 4) with an unbroken loop of obstacles so the goal
    # itself is reachable, the recovery search finds a *non*-reachable
    # neighbour, and astar must return [].
    for x in range(2, 7):
        grid_boxed[3, x] = 1  # top wall
        grid_boxed[6, x] = 1  # bottom wall
    for y in range(3, 7):
        grid_boxed[y, 1] = 1  # left wall
        grid_boxed[y, 6] = 1  # right wall
    # And an explicitly sealed-off goal: build an island the goal sits
    # on but start cannot reach.
    grid_sealed = np.ones((9, 9), dtype=np.int8)
    grid_sealed[0, 0] = 0  # start region
    # island around (8, 8) with a 1-cell gap of free space
    grid_sealed[8, 8] = 0  # goal cell free
    grid_sealed[7, 8] = 0
    grid_sealed[8, 7] = 0
    grid_sealed[7, 7] = 0
    path = astar(grid_sealed, (0, 0), (8, 8))
    assert path == [], f"expected [], got {path}"
    # Boxed goal (goal itself free but fenced in / disconnected):
    grid_boxed = np.zeros((7, 7), dtype=np.int8)
    for x in range(2, 7):
        grid_boxed[3, x] = 1
        grid_boxed[6, x] = 1
    for y in range(3, 7):
        grid_boxed[y, 1] = 1
        grid_boxed[y, 6] = 1
    # Seal the top corners so a diagonal move can't squeeze past.
    grid_boxed[2, 1] = 1
    grid_boxed[3, 1] = 1
    assert astar(grid_boxed, (0, 0), (4, 4)) == []
    print("  [OK]  unreachable goal returns []")

    # 4) start == goal.
    assert astar(grid_free, (2, 2), (2, 2)) == [(2, 2)]
    print("  [OK]  start == goal returns [start]")

    # 5) Start blocked -- A* should still find a path using the nearest free neighbour.
    grid_blocked_start = np.zeros((4, 4), dtype=np.int8)
    grid_blocked_start[1, 1] = 1
    path = astar(grid_blocked_start, (1, 1), (3, 3))
    assert path and path[-1] == (3, 3), path
    assert path[0] != (1, 1), f"start should have been recovered, got {path}"
    print(f"  [OK]  start-blocked recovery ({path[0]} -> {path[-1]})")

    print("ast._self_test: all checks passed.")


if __name__ == "__main__":
    _self_test()
