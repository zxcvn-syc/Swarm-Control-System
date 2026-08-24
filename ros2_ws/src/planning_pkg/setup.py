import os
from pathlib import Path

from setuptools import find_packages, setup

package_name = "planning_pkg"
package_root = Path(__file__).resolve().parent
repo_root = package_root.parents[2]
world_file = repo_root / "simulation" / "worlds" / "swarm_field.world"
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/config", ["config/planning.yaml"]),
    (
        "share/" + package_name + "/launch",
        [
            "launch/planning.launch.py",
            "launch/px4_sitl.launch.py",
            "launch/sitl_test.launch.py",
        ],
    ),
]
if world_file.is_file():
    data_files.append(
        (
            "share/" + package_name + "/worlds",
            [os.path.relpath(world_file, package_root)],
        )
    )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    zip_safe=True,
    maintainer="Swarm Control System Team",
    maintainer_email="swarm@example.com",
    description=(
        "Planning package: A* / D* Lite path planners, ROS2 node that "
        "publishes swarm_interfaces/DroneStateArray and /planned_path."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "planner_node = planning_pkg.planner_node:main",
            "grid_map_node = planning_pkg.grid_map_node:main",
            "ugv_state_pub = planning_pkg.ugv_state_publisher:main",
            "px4_offboard_bridge = planning_pkg.px4_offboard_bridge:main",
            "sitl_pose_bridge = planning_pkg.sitl_pose_bridge:main",
            "rflysim_follower = planning_pkg.rflysim_follower:main",
            "dstar_benchmark = planning_pkg.dstar_benchmark_node:main",
        ],
    },
)
