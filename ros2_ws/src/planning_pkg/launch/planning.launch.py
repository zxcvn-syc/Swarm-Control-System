"""Launch file for ``planner_node``.

This file mirrors the style of
``perception_pkg/launch/tracker_node.launch.py`` (LaunchDescription +
DeclareLaunchArgument + Node with a YAML config overlay).

Typical invocations::

    ros2 launch planning_pkg planning.launch.py                       # default YAML
    ros2 launch planning_pkg planning.launch.py \
        planner:=dstar_lite grid_size:=120 num_drones:=12
    ros2 launch planning_pkg planning.launch.py config:=/abs/override.yaml
"""

from __future__ import annotations

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription(args + [node, ugv_node])
