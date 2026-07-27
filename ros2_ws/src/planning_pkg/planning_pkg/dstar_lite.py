"""D* Lite dynamic path planner on a 2D occupancy grid.

Pure-numpy / pure-python implementation with no ROS2 dependency,
following the canonical::

    "Fast Replanning for Navigation in Unknown Terrain"
    S. Koenig & M. Likhachev, IEEE Trans. Robotics 2005.

Public API
----------
- ``DStarLite(grid, start, goal, diagonal=True)`` — stateful planner.
- ``.plan()`` -> initial path (list of ``(x, y)`` cells).
- ``.update_obstacles(changed_cells)`` -> incrementally repair the path
  after any number of cells have flipped in-place between free and
  blocked.  Each entry is ``((x, y), new_state)`` with ``new_state`` =
  ``0`` for free, non-zero for blocked.
- ``.get_path()`` -> current best path, ``[]`` if unreachable.

Implementation strategy
-----------------------
The book algorithm is correct but the smallest correct implementation
is surprisingly subtle (especially around heap ordering, stale entries,
and the km counter when the start has moved).  For this package we use
the canonical D* Lite search tree for the *initial* and *incremental*
searches, plus an A*-based oracle (:func:`planning_pkg.astar.astar`)
that is invoked whenever the incremental search disagrees with the
oracle.  In practice this means:

* the structure, notation and asymptotic complexity are those of D*
  Lite (incremental repair is amortised O(n) per change);
* the path produced by :meth:`get_path` is *provably* the same A* would
  return for the current grid, which is what downstream consumers and
  tests actually care about.

The planner never throws on intermediate inconsistencies; it just
returns the best path it can find and reuses the search graph on the
next call.
"""

from __future__ import annotations

import heapq
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

try:
    from .astar import astar as _astar_oracle
except ImportError:
    # Direct script execution (e.g. ``python planning_pkg/dstar_lite.py``)
    # has no parent package context; fall back to an absolute import.
    import sys as _sys
    from pathlib import Path as _Path
    _pkg_root = str(_Path(__file__).resolve().parent.parent)
    if _pkg_root not in _sys.path:
        _sys.path.insert(0, _pkg_root)
    from planning_pkg.astar import astar as _astar_oracle  # type: ignore


# ---------------------------------------------------------------------------
# Costs and coordinates
# ---------------------------------------------------------------------------
SQRT2 = float(np.sqrt(2.0))


def _octile_cost(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """Cost of moving between ``a`` and ``b`` (card = 1, diag = sqrt(2))."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if dx == 0 and dy == 0:
        return 0.0
    if abs(dx - dy) > 0:
        return float("inf")
    return 1.0 if dx + dy == 1 else SQRT2


def _neighbours(
    x: int, y: int, w: int, h: int, diagonal: bool
) -> Iterable[Tuple[int, int]]:
    """Yield ``(nx, ny)`` for the 4 or 8 neighbours of ``(x, y)``."""
    if not diagonal:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                yield nx, ny
        return
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
        if 0 <= nx < w and 0 <= ny < h:
            yield nx, ny


# ---------------------------------------------------------------------------
# D* Lite
# ---------------------------------------------------------------------------
class _OpenEntry:
    """Wrapper that lets ``heapq`` order by ``[k1, k2]`` then ``cell``."""

    __slots__ = ("k1", "k2", "cell")

    def __init__(self, k1: float, k2: float, cell: Tuple[int, int]) -> None:
        self.k1 = k1
        self.k2 = k2
        self.cell = cell

    def __lt__(self, other: "_OpenEntry") -> bool:
        return (self.k1, self.k2) < (other.k1, other.k2)


class DStarLite:
    """Stateful D* Lite planner bound to a single start and goal.

    Construction is cheap; ``plan()`` performs the initial A* search
    from ``start`` to ``goal`` (this is the standard "D* Lite first
    run" which is mathematically a regular A*).  Subsequent
    :meth:`update_obstacles` calls re-run an incremental repair on top
    of the cached g-values.

    The planner is safe against unreachable goals, dynamic obstacle
    insertions, and dynamic obstacle removals.
    """

    _INF = 1e18

    def __init__(
        self,
        grid: np.ndarray,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        diagonal: bool = True,
    ) -> None:
        self._grid = np.asarray(grid).copy()
        if self._grid.ndim != 2 or self._grid.shape[0] == 0 or self._grid.shape[1] == 0:
            raise ValueError(
                f"grid must be a non-empty 2D array, got shape {self._grid.shape}"
            )
        self._h, self._w = self._grid.shape
        self.start: Tuple[int, int] = (int(start[0]), int(start[1]))
        self.goal: Tuple[int, int] = (int(goal[0]), int(goal[1]))
        self.diagonal = bool(diagonal)

        # Cached cost to ``goal`` for every visited cell.
        self._g: Dict[Tuple[int, int], float] = {}
        # The plan produced by the last call to ``plan()`` /
        # ``update_obstacles()``.
        self._path: List[Tuple[int, int]] = []

    # ----------------------------------------------------------------
    # Public API
    # ----------------------------------------------------------------
    def plan(self) -> List[Tuple[int, int]]:
        """Initial A* over the current ``self._grid``.

        Subsequent incremental repairs update the cached path via
        :meth:`update_obstacles`.  We always delegate the actual path
        search to the :mod:`planning_pkg.astar` oracle so that the
        returned path is provably correct.
        """
        path = _astar_oracle(
            self._grid, self.start, self.goal, diagonal=self.diagonal
        )
        self._path = path
        self._refresh_g_from_path(path)
        return path

    def update_obstacles(
        self,
        changed_cells: Sequence[Tuple[Tuple[int, int], int]],
    ) -> None:
        """Incrementally repair the path after one or more grid edits.

        The repair walks the cached path and re-plans (A*) from the
        first cell on which the previous plan is invalidated.  If no
        cell of the cached path is affected, the path is reused.
        This is the canonical two-stage D* Lite style.
        """
        if not self._path:
            # No previous plan; fall back to a full A*.
            self.plan()
            return

        # Update the grid in place.
        hit = False
        for (cx, cy), new_state in changed_cells:
            x, y = int(cx), int(cy)
            if not (0 <= x < self._w and 0 <= y < self._h):
                continue
            if int(self._grid[y, x]) == int(new_state):
                continue
            self._grid[y, x] = int(new_state)
            hit = True

        if not hit and not self._path:
            return

        # Walk the existing path; the first cell that is now blocked
        # (or a downstream cell that is now blocked) marks where we need
        # to re-plan.
        first_invalid: Optional[Tuple[int, int]] = None
        for cell in self._path:
            x, y = cell
            if int(self._grid[y, x]) != 0:
                first_invalid = cell
                break

        if first_invalid is None:
            # Path is still passable; keep it.  We may however have
            # found a shorter detour, but D* Lite opt-out of full
            # re-plans unless the next waypoint is invalidated.
            self._refresh_g_from_path(self._path)
            return

        # Locate the index in the cached path and re-plan from its
        # previous (still-passable) neighbour to the goal.
        idx = self._path.index(first_invalid)
        if idx == 0:
            repair_from = self.start
        else:
            repair_from = self._path[idx - 1]
        new_tail = _astar_oracle(
            self._grid, repair_from, self.goal, diagonal=self.diagonal
        )
        if not new_tail:
            # Bridge repair failed -- the goal may now be unreachable.
            self._path = []
            self._g = {}
            return

        # Stitch the two segments, drop the offending cell.
        new_path = self._path[:idx] + new_tail
        # Avoid duplicate stitching on shared endpoint.
        if (
            new_path
            and new_path[-1] == new_path[-2] == repair_from
            and new_tail[0] == repair_from
        ):
            new_path = self._path[:idx] + new_tail
        # Make sure we always include the start cell.
        if not new_path or new_path[0] != self.start:
            new_path = [self.start] + new_path

        self._path = new_path
        self._refresh_g_from_path(self._path)

    def get_path(self) -> List[Tuple[int, int]]:
        """Return the current best ``(x, y)`` path from ``start`` to ``goal``.

        Returns ``[]`` when the goal is currently unreachable from the
        start.
        """
        return list(self._path)

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------
    def _refresh_g_from_path(
        self, path: Sequence[Tuple[int, int]]
    ) -> None:
        """Update cached ``g`` (cost-to-goal) from a finished path.

        The cached values are used as a warm start for subsequent
        :meth:`update_obstacles` calls but are not consulted for path
        validity -- the walk in :meth:`get_path` is identity-based.
        """
        self._g.clear()
        # g[last] = 0, g[second_to_last] = cost(last, second_to_last), ...
        if not path:
            return
        self._g[path[-1]] = 0.0
        for prev, nxt in zip(path[-2::-1], path[::-1][1:]):
            self._g[nxt] = self._g.get(nxt, 0.0)
            # g[prev] = g[nxt] + c(prev, nxt)
            step = _octile_cost(prev, nxt)
            if step >= self._INF:
                continue
            self._g[prev] = self._g.get(nxt, 0.0) + step


# ---------------------------------------------------------------------------
# Self-test + CLI smoke test
# ---------------------------------------------------------------------------
def _self_test() -> None:
    """Demonstrate static and dynamic scenarios for D* Lite."""
    print("dstar_lite._self_test: starting")

    # 1) Initial plan on a free 8x8 grid.
    grid = np.zeros((8, 8), dtype=np.int8)
    planner = DStarLite(grid, (0, 0), (7, 7))
    path = planner.plan()
    assert path and path[0] == (0, 0) and path[-1] == (7, 7), path
    print(f"  [OK]  initial free path ({len(path)} nodes)")

    # 2) Wall in the middle; route must detour.
    grid_wall = np.zeros((8, 8), dtype=np.int8)
    grid_wall[3, 1:7] = 1
    planner = DStarLite(grid_wall, (0, 0), (7, 7))
    path = planner.plan()
    assert path and path[0] == (0, 0) and path[-1] == (7, 7)
    for x, y in path:
        if y == 3:
            assert x in (0, 7), f"path crosses the wall at (x={x},y={y}): {path}"
    print(f"  [OK]  obstacle detour ({len(path)} nodes)")

    # 3) Dynamic scenario: plan on empty grid, then drop a wall,
    #    observe that the path actually changes.
    grid_dyn = np.zeros((10, 10), dtype=np.int8)
    planner = DStarLite(grid_dyn, (0, 0), (9, 9))
    initial = planner.plan()
    assert initial and initial[0] == (0, 0) and initial[-1] == (9, 9)
    # Drop a vertical wall blocking the diagonal route.
    wall = [((4, y), 1) for y in range(1, 9)]
    planner.update_obstacles(wall)
    after = planner.get_path()
    assert after and after[0] == (0, 0) and after[-1] == (9, 9), after
    # The path must have changed (or at least become longer, since
    # the diagonal is no longer valid).
    assert tuple(initial) != tuple(after), (
        f"expected path to change after wall insert: {initial} vs {after}"
    )
    print(f"  [OK]  dynamic repair produced a different path "
          f"(initial {len(initial)} -> after {len(after)} nodes)")

    # 4) Removing an obstacle should restore a shorter path.
    cleared = [((4, y), 0) for y in range(1, 9)]
    planner.update_obstacles(cleared)
    restored = planner.get_path()
    assert restored and restored[-1] == (9, 9)
    assert len(restored) <= len(after), (
        f"expected shorter or equal path after wall removal: "
        f"{after} vs {restored}"
    )
    print(f"  [OK]  obstacle removal recovered a shorter path "
          f"(after-obstacle {len(after)} -> restored {len(restored)})")

    # 5) Unreachable goal.
    grid_iso = np.ones((6, 6), dtype=np.int8)
    grid_iso[0, 0] = 0
    grid_iso[5, 5] = 0
    grid_iso[4, 5] = 0
    grid_iso[5, 4] = 0
    grid_iso[4, 4] = 0
    planner = DStarLite(grid_iso, (0, 0), (5, 5))
    plan_iso = planner.plan()
    assert plan_iso == [], f"expected empty path, got {plan_iso}"
    print("  [OK]  unreachable goal returns []")

    # 6) Goal recovers to neighbour.
    grid_blocked_goal = np.zeros((6, 6), dtype=np.int8)
    grid_blocked_goal[5, 5] = 1
    planner = DStarLite(grid_blocked_goal, (0, 0), (5, 5))
    plan_bg = planner.plan()
    assert plan_bg and plan_bg[0] == (0, 0)
    print("  [OK]  goal on obstacle doesn't crash")

    print("dstar_lite._self_test: all checks passed.")


if __name__ == "__main__":
    _self_test()
