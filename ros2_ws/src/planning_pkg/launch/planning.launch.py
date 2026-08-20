"""Launch file for ``planner_node``.

This file mirrors the style of
``perception_pkg/launch/tracker_node.launch.py`` (LaunchDescription +
DeclareLaunchArgument + Node with a YAML config overlay).

Typical invocations::

    ros2 launch planning_pkg planning.launch.py                       # default YAML
    ros2 launch planning_pkg planning.launch.py \\
        planner:=dstar_lite grid_size:=120 num_drones:=12
    ros2 launch planning_pkg planning.launch.py config:=/abs/override.yaml

    # With PX4 SITL (Gazebo + PX4 + MAVROS + bridges) wired in:
    PX4_SITL_ROOT=$HOME/src/PX4-Autopilot \\
        ros2 launch planning_pkg planning.launch.py include_sitl:=true
"""

from __future__ import annotations

import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Optional path to a planning.yaml override file.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Optional namespace prefix (kept empty by default to "
            "match scheduler/enclosure usage).",
        ),
        DeclareLaunchArgument(
            "planner",
            default_value="astar",
            choices=["astar", "dstar_lite"],
            description="Path-planner algorithm.",
        ),
        DeclareLaunchArgument(
            "num_drones",
            default_value="8",
            description="Number of drones for the demo (also seeds the grid).",
        ),
        DeclareLaunchArgument(
            "grid_size",
            default_value="100",
            description="Square grid side length in cells.",
        ),
        DeclareLaunchArgument(
            "tick_period",
            default_value="0.5",
            description="Seconds between planner ticks.",
        ),
        DeclareLaunchArgument(
            "include_sitl",
            default_value="false",
            choices=["true", "false"],
            description="If true, also launch px4_sitl.launch.py (Gazebo + PX4 + MAVROS + bridges).",
        ),
        DeclareLaunchArgument(
            "sitl_num_uav",
            default_value="1",
            description="Number of PX4 SITL instances; the reproducible profile currently supports one.",
        ),
        DeclareLaunchArgument(
            "sitl_headless",
            default_value="false",
            description="Run SITL Gazebo headless (used when include_sitl is true).",
        ),
    ]

    inline_overrides = {
        "planner": LaunchConfiguration("planner"),
        "num_drones": LaunchConfiguration("num_drones"),
        "grid_size": LaunchConfiguration("grid_size"),
        "tick_period": LaunchConfiguration("tick_period"),
    }

    node = Node(
        package="planning_pkg",
        executable="planner_node",
        name="planner_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            inline_overrides,
            # Optional YAML override.
            LaunchConfiguration("config"),
        ],
    )

    ugv_node = Node(
        package="planning_pkg",
        executable="ugv_state_pub",
        name="ugv_state_publisher",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"num_ugv": 2}],
    )

    px4_offboard_bridge_node = Node(
        package="planning_pkg",
        executable="px4_offboard_bridge",
        name="px4_offboard_bridge",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{}],
        condition=UnlessCondition(LaunchConfiguration("include_sitl")),
    )

    sitl_pose_bridge_node = Node(
        package="planning_pkg",
        executable="sitl_pose_bridge",
        name="sitl_pose_bridge",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"platform_type": 1}],
        condition=UnlessCondition(LaunchConfiguration("include_sitl")),
    )

    actions: List[object] = args + [
        node,
        ugv_node,
        px4_offboard_bridge_node,
        sitl_pose_bridge_node,
    ]

    if _sitl_launch_available():
        sitl_include = IncludeLaunchDescription(
            launch_description_source=_get_sitl_launch_source(),
            launch_arguments=[
                ("num_uav", LaunchConfiguration("sitl_num_uav")),
                ("headless", LaunchConfiguration("sitl_headless")),
            ],
            condition=IfCondition(LaunchConfiguration("include_sitl")),
        )
        actions.append(sitl_include)

    return LaunchDescription(actions)


def _sitl_launch_available() -> bool:
    """Check whether ``px4_sitl.launch.py`` is in the package share.

    The check is purely a hint for the no-SITL fallback.  When the
    package hasn't been built yet (e.g. running ``colcon build`` on a
    fresh checkout) we silently skip the include; once ``colcon build
    --packages-select planning_pkg`` runs, the share directory is
    populated and the include becomes active.
    """
    prefix_path = os.environ.get("AMENT_PREFIX_PATH", "")
    for prefix in prefix_path.split(":"):
        candidate = os.path.join(prefix, "share", "planning_pkg", "launch", "px4_sitl.launch.py")
        if os.path.exists(candidate):
            return True
    return False


def _get_sitl_launch_source():
    """Return the package-relative launch description source for SITL."""
    from launch.launch_description_sources import (
        PythonLaunchDescriptionSource,
    )

    prefix_path = os.environ.get("AMENT_PREFIX_PATH", "")
    for prefix in prefix_path.split(":"):
        candidate = os.path.join(prefix, "share", "planning_pkg", "launch", "px4_sitl.launch.py")
        if os.path.exists(candidate):
            return PythonLaunchDescriptionSource(candidate)

    return PythonLaunchDescriptionSource(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "px4_sitl.launch.py",
        )
    )
