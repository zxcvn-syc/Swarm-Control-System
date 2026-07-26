"""scheduler_pkg entry point.

Run as:
    python3 -m scheduler_pkg                # smoke test
    python3 -m scheduler_pkg.assign --yaml  # CLI
"""

from __future__ import annotations

from .assign import greedy_assign, hungarian_assign


def _run() -> None:
    import numpy as np

    drones = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]])
    targets = np.array([[0.5, 0.5], [10.5, 0.5], [5.0, 5.0], [20.0, 20.0]])
    g = greedy_assign(drones, targets, max_per_drone=2)
    assert len(g) == 4, f"greedy expected 4 pairs, got {len(g)}"
    h = hungarian_assign(drones, targets)
    assert len(h) == 3, f"hungarian expected 3 pairs, got {len(h)}"
    empty = greedy_assign(np.zeros((0, 2)), targets)
    assert empty == [], f"empty drones expected [], got {empty}"
    print("assign.py smoke tests passed:", g, h, empty)


if __name__ == "__main__":
    _run()
