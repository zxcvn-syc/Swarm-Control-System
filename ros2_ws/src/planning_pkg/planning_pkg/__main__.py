"""planning_pkg entry point.

Run as::

    python3 -m planning_pkg                # algorithm smoke test
    python3 -m planning_pkg.astar          # A* self-test only
    python3 -m planning_pkg.dstar_lite     # D* Lite self-test only

The ROS2 node itself is launched via ``ros2 run planning_pkg
planner_node`` (see ``launch/planning.launch.py``).
"""

from __future__ import annotations

import sys
from pathlib import Path as _Path


def _run() -> None:
    # Late imports so running ``python3 planning_pkg/__main__.py``
    # directly still works (relative imports otherwise fail).
    parent = str(_Path(__file__).resolve().parent.parent)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    if __package__ in (None, ""):
        # ``python3 planning_pkg/__main__.py`` -- need absolute import.
        from planning_pkg.astar import _self_test as _astar_self_test
        from planning_pkg.dstar_lite import (
            _self_test as _dstar_self_test,
        )
    else:
        from .astar import _self_test as _astar_self_test
        from .dstar_lite import _self_test as _dstar_self_test

    _astar_self_test()
    _dstar_self_test()
    print("planning_pkg smoke: OK")


if __name__ == "__main__":
    _run()
