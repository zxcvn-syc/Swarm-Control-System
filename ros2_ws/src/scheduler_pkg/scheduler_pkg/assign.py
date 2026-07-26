"""Greedy / Hungarian target-to-drone assignment.

Pure-numpy / scipy implementation, no ROS2 dependency, so it can be unit
tested on any machine and driven from a YAML config in stand-alone mode.

Public API
----------
- greedy_assign(drones, targets, target_priorities=None, max_per_drone=2)
- hungarian_assign(drones, targets, target_priorities=None, max_per_drone=1)
- load_problem_from_yaml(path) -> (drones, targets, priorities)
- main()  CLI entry: read YAML, run algorithm, print assignments
"""

from __future__ import annotations

import argparse
from typing import List, Optional, Tuple

import numpy as np

# scipy is the only optional dependency; we keep the import local so the
# module can be imported even in a minimal CI image (greedy tests would
# still pass).
try:
    from scipy.optimize import linear_sum_assignment  # type: ignore
    _HAS_SCIPY = True
except Exception:  # pragma: no cover - exercised only if scipy is missing
    _HAS_SCIPY = False


# ---------------------------------------------------------------------------
# Self-checks (very lightweight smoke tests, see also tests/test_assign.py)
# ---------------------------------------------------------------------------
def _self_test() -> None:  # pragma: no cover - exercised by __main__
    drones = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]])
    targets = np.array([[0.5, 0.5], [10.5, 0.5], [5.0, 5.0], [20.0, 20.0]])
    g = greedy_assign(drones, targets, max_per_drone=2)
    # 3 drones * 2 slots = 6, but only 4 targets, so 4 pairs.
    assert len(g) == 4, g
    if _HAS_SCIPY:
        h = hungarian_assign(drones, targets)
        assert len(h) == 3, h
    assert greedy_assign(np.zeros((0, 2)), targets) == []
    assert greedy_assign(drones, np.zeros((0, 2))) == []


# ---------------------------------------------------------------------------
# Core algorithms
# ---------------------------------------------------------------------------
def _cost_matrix(
    drones: np.ndarray,
    targets: np.ndarray,
    target_priorities: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Squared distance between every (drone, target) pair, weighted by priority.

    Higher priority -> lower cost (so the algorithm prefers it). Priorities
    are scaled to lie in [0.5, 1.5] to avoid blowing up the cost landscape.
    """
    if drones.size == 0 or targets.size == 0:
        return np.zeros((drones.shape[0], targets.shape[0]), dtype=float)

    # Pairwise squared Euclidean distance.
    diff = drones[:, None, :] - targets[None, :, :]
    cost = np.sum(diff * diff, axis=-1)

    if target_priorities is not None:
        priorities = np.asarray(target_priorities, dtype=float).reshape(-1)
        if priorities.shape[0] != targets.shape[0]:
            raise ValueError(
                f"target_priorities length {priorities.shape[0]} != "
                f"num targets {targets.shape[0]}"
            )
        # Map priority -> weight in [0.5, 1.5]. priority <= 0 -> 1.5 (de-prioritised),
        # priority >= 1 -> 0.5 (strongly preferred). Linear in between.
        weight = np.clip(1.5 - priorities, 0.5, 1.5)
        cost = cost * weight[None, :]
    return cost


def greedy_assign(
    drones: np.ndarray,
    targets: np.ndarray,
    target_priorities: Optional[np.ndarray] = None,
    max_per_drone: int = 2,
) -> List[Tuple[int, int]]:
    """Greedy nearest-target assignment.

    For each drone (in input order), pick the nearest unassigned target up to
    ``max_per_drone`` times. Priorities bias the cost; no priority is the same
    as a uniform weight of 1.0.

    Parameters
    ----------
    drones : (N, 2) array
        Drone XY positions.
    targets : (M, 2) array
        Target XY positions.
    target_priorities : (M,) array, optional
        Per-target priority in [0, 1]. Higher = preferred.
    max_per_drone : int
        Hard cap on how many targets a single drone is given.

    Returns
    -------
    list of (drone_idx, target_idx)
    """
    drones = np.asarray(drones, dtype=float).reshape(-1, 2)
    targets = np.asarray(targets, dtype=float).reshape(-1, 2)
    if drones.size == 0 or targets.size == 0:
        return []
    if max_per_drone <= 0:
        return []

    cost = _cost_matrix(drones, targets, target_priorities)
    assigned_targets = set()
    assignments: List[Tuple[int, int]] = []
    n_targets = cost.shape[1]

    for d in range(cost.shape[0]):
        # Pick the cheapest target that is still free, repeating up to
        # max_per_drone times.
        local = cost[d].copy()
        for _ in range(max_per_drone):
            # Mask out already-assigned targets.
            for t in assigned_targets:
                local[t] = np.inf
            if not np.isfinite(local).any():
                break
            target_idx = int(np.argmin(local))
            assignments.append((d, target_idx))
            assigned_targets.add(target_idx)
    return assignments


def hungarian_assign(
    drones: np.ndarray,
    targets: np.ndarray,
    target_priorities: Optional[np.ndarray] = None,
    max_per_drone: int = 1,
) -> List[Tuple[int, int]]:
    """Hungarian assignment via :func:`scipy.optimize.linear_sum_assignment`.

    The Hungarian algorithm solves a 1-to-1 bipartite matching. To support
    ``max_per_drone > 1`` we replicate each drone column that many times
    and pick the resulting unique targets.

    Returns at most ``min(N * max_per_drone, M)`` pairs.
    """
    if not _HAS_SCIPY:
        raise RuntimeError(
            "scipy is required for hungarian_assign; "
            "pip install scipy or use greedy_assign instead."
        )

    drones = np.asarray(drones, dtype=float).reshape(-1, 2)
    targets = np.asarray(targets, dtype=float).reshape(-1, 2)
    if drones.size == 0 or targets.size == 0:
        return []
    if max_per_drone <= 0:
        return []

    if max_per_drone == 1:
        cost = _cost_matrix(drones, targets, target_priorities)
        row_ind, col_ind = linear_sum_assignment(cost)
        return [(int(r), int(c)) for r, c in zip(row_ind, col_ind)]

    # Replicate each drone max_per_drone times so that linear_sum_assignment
    # can assign multiple targets to the same drone. Targets remain unique.
    cost = _cost_matrix(drones, targets, target_priorities)
    # cost shape: (N, M). Tile along axis 0 to get (N*max_per_drone, M).
    big = np.tile(cost, (max_per_drone, 1))
    rows, cols = linear_sum_assignment(big)
    # rows[r] is the column index in the tiled matrix (== original drone id).
    drone_ids = rows % drones.shape[0]
    pairs = sorted(
        {(int(d), int(c)) for d, c in zip(drone_ids, cols)},
        key=lambda p: (p[0], p[1]),
    )
    return pairs


# ---------------------------------------------------------------------------
# YAML loader + CLI
# ---------------------------------------------------------------------------
def load_problem_from_yaml(
    path: str,
) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
    """Load a YAML file with the shape::

        drones:
          - [x, y]
        targets:
          - [x, y]
        priorities:        # optional
          - 0.8

    Returns ``(drones, targets, priorities_or_None)``.
    """
    try:
        import yaml  # type: ignore
    except ImportError as e:  # pragma: no cover
        raise RuntimeError("PyYAML is required to load YAML configs") from e

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    drones = np.asarray(data.get("drones", []), dtype=float).reshape(-1, 2)
    targets = np.asarray(data.get("targets", []), dtype=float).reshape(-1, 2)
    prios = data.get("priorities", None)
    if prios is None:
        priorities: Optional[np.ndarray] = None
    else:
        priorities = np.asarray(prios, dtype=float).reshape(-1)
    return drones, targets, priorities


def _format_assignments(
    assignments: List[Tuple[int, int]],
    drones: np.ndarray,
    targets: np.ndarray,
) -> List[str]:
    lines = []
    for d, t in assignments:
        dx, dy = drones[d]
        tx, ty = targets[t]
        dist = float(np.hypot(tx - dx, ty - dy))
        lines.append(
            f"drone[{d}] @ ({dx:.2f}, {dy:.2f}) -> target[{t}] @ ({tx:.2f}, {ty:.2f}) "
            f"dist={dist:.2f}"
        )
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="scheduler_pkg assignment CLI")
    parser.add_argument(
        "--yaml",
        required=True,
        help="Path to YAML with 'drones', 'targets', optional 'priorities'.",
    )
    parser.add_argument(
        "--strategy",
        choices=("greedy", "hungarian"),
        default="greedy",
    )
    parser.add_argument("--max-per-drone", type=int, default=2)
    args = parser.parse_args()

    drones, targets, priorities = load_problem_from_yaml(args.yaml)
    print(
        f"Loaded {drones.shape[0]} drones, {targets.shape[0]} targets "
        f"(strategy={args.strategy}, max_per_drone={args.max_per_drone})."
    )
    if args.strategy == "greedy":
        assignments = greedy_assign(
            drones, targets, priorities, max_per_drone=args.max_per_drone
        )
    else:
        assignments = hungarian_assign(
            drones, targets, priorities, max_per_drone=args.max_per_drone
        )
    for line in _format_assignments(assignments, drones, targets):
        print(line)
    return 0


if __name__ == "__main__":  # pragma: no cover
    _self_test()
    raise SystemExit(main())
