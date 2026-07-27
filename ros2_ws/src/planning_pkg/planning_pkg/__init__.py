"""planning_pkg: path planning for the swarm control system.

Submodules
----------
- :mod:`planning_pkg.astar`     pure-numpy A* on a 2D occupancy grid.
- :mod:`planning_pkg.dstar_lite` incremental D* Lite planner.
- :mod:`planning_pkg.planner_node` ROS2 adapter.
"""

from .astar import astar
from .dstar_lite import DStarLite

__all__ = ["astar", "DStarLite"]

__version__ = "0.1.0"
