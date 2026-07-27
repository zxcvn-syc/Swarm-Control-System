"""Top-level launch file: bring up the swarm's three (well, four) nodes.

Nodes
-----
1. ``tracker_node``        (perception_pkg)  publishes ``/target_track``
2. ``scheduler_node``      (scheduler_pkg)   publishes ``/task_assignment``
3. ``planner_node``        (planning_pkg)    publishes ``/drone_states``,
                                          ``/planned_path``
4. ``enclosure_node``      (containment_pkg) publishes ``/enclosure_command``

Namespacing
-----------
We follow the convention used by ``perception_pkg`` and friends: the
default launch keeps the global topic names (``/target_track``,
``/drone_states``, etc.) so the planner/containment code below subscribes
without prefix.  Operators that want full isolation can pass
``namespace:=/`` and the ``perception_pkg`` launch -- here we just
expose the option for consistency.

Usage::

    ros2 launch Swarm-Control-System/launch/three_links.launch.py
    ros2 launch Swarm-Control-System/launch/three_links.launch.py planner:=dstar_lite

This file is *not* an ament_python install artefact; it lives in the
repository's top-level ``launch/`` directory and is invoked by ROS2's
launch system using its absolute path.
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
            "planner", default_value="astar",
            choices=["astar", "dstar_lite"],
            description="Path planner to use (forwarded into planning_pkg).",
        ),
        DeclareLaunchArgument(
            "num_drones", default_value="8",
            description="Number of drones (forwarded to planning + scheduler).",
        ),
        DeclareLaunchArgument(
            "scheduler_strategy", default_value="greedy",
            choices=["greedy", "hungarian"],
            description="Assignment strategy for scheduler_node.",
        ),
    ]

    # ---------- tracker ---------------------------------------------------
    tracker_node = Node(
        package="perception_pkg",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        # In three-links mode we drive the tracker off a video file from
        # the project's own ``test_videos`` directory.  Operators running
        # the demo on a workstation without a webcam can override via:
        #     video_source:=/path/to/file.mp4
        parameters=[
            {
                "input_mode": "video",
                "video_source": "",
                "track_topic": "/target_track",
                "frame_id": "camera_optical_frame",
                "publish_rate_hz": 10.0,
                "loop_video": True,
            }
        ],
    )

    # ---------- scheduler -------------------------------------------------
    scheduler_node = Node(
        package="scheduler_pkg",
        executable="scheduler_node",
        name="scheduler_node",
        output="screen",
        parameters=[
            {
                "num_drones": LaunchConfiguration("num_drones"),
                "assignment_strategy": LaunchConfiguration("scheduler_strategy"),
                "max_per_drone": 2,
                "tick_period": 0.5,
                "log_interval_sec": 5.0,
                "target_topic": "/target_track",
                "drone_topic": "/drone_states",
                "output_topic": "/task_assignment",
                "default_task_type": "track",
            }
        ],
    )

    # ---------- planner ---------------------------------------------------
    planner_node = Node(
        package="planning_pkg",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            {
                "planner": LaunchConfiguration("planner"),
                "num_drones": LaunchConfiguration("num_drones"),
                "grid_size": 100,
                "tick_period": 0.5,
                "publish_path": True,
                "sim_tick_speed": 1.0,
                "task_topic": "/task_assignment",
                "grid_topic": "/grid_map",
                "drone_states_topic": "/drone_states",
                "planned_path_topic": "/planned_path",
            }
        ],
    )

    # ---------- enclosure -------------------------------------------------
    enclosure_node = Node(
        package="containment_pkg",
        executable="enclosure_node",
        name="enclosure_node",
        output="screen",
        parameters=[
            {
                "enclosure_radius": 25.0,
                "min_dist": 5.0,
                "update_period": 1.0,
            }
        ],
    )

    return LaunchDescription(args + [
        tracker_node,
        scheduler_node,
        planner_node,
        enclosure_node,
    ])
