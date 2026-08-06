"""Top-level launch file: bring up the swarm's five nodes.
Nodes
-----
1. ``tracker_node``        (perception_pkg)  publishes ``/target_track`` + ``/enclosure_targets``
2. ``coord_transform_node``(perception_pkg)  publishes ``/target_track_world`` (pixel→world)
3. ``scheduler_node``      (scheduler_pkg)   publishes ``/task_assignment``
4. ``planner_node``        (planning_pkg)    publishes ``/drone_states``,
                                          ``/planned_path``
5. ``enclosure_node``      (containment_pkg) publishes ``/enclosure_command``

All nodes use global (``/``-prefixed) topic names by default so that
cross-package wiring works without additional remapping. If namespace
isolation is needed, add a ``namespace`` argument to each ``Node(...)``
declaration; this launch file intentionally does not expose a top-level
``namespace`` launch argument.

NOTE on topic parameterization:
- tracker_node, scheduler_node, coord_transform_node, planner_node all
  expose topic names as ROS parameters and are wired explicitly here.
- enclosure_node does NOT parameterize topic names (topics are hardcoded
  in the node source), so we only pass radius/min_dist/update_period.

Usage::
    ros2 launch Swarm-Control-System/launch/three_links.launch.py \\
        video_source:=/path/to/video.mp4
    ros2 launch Swarm-Control-System/launch/three_links.launch.py planner:=dstar_lite
    ros2 launch Swarm-Control-System/launch/three_links.launch.py scheduler_strategy:=auction

This file is *not* an ament_python install artefact; it lives in the
repository's top-level ``launch/`` directory and is invoked by ROS2's
launch system using its absolute path.

Update history (v2.3, 2026-08-06):
- Added coord_transform_node for permanent pixel→world coordinate transformation
- Added auction strategy option (merged to main in commit 80d2a1e)
- Now supports all three scheduler strategies: greedy/hungarian/auction
- Added all missing launch arguments (video_source, frame_id, publish_rate_hz,
  enclosure_radius, min_dist) to match ros2_ws/launch/ version
- Unified tracker_node parameters across both launch files
- FIXED: planner_node parameter ``target_topic`` → ``target_track_world_topic``
- FIXED: removed non-existent topic params from enclosure_node (topics hardcoded)
- Added log_interval_sec to planner_node parameters
- Removed misleading namespace:=/ documentation (no such argument declared)
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
            "video_source",
            default_value="",
            description="Local video fed to tracker_node (input_mode=video). REQUIRED.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_optical_frame",
            description="header.frame_id on the published TargetTrackArray.",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="10.0",
            description="tracker_node publish rate cap.",
        ),
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
            choices=["greedy", "hungarian", "auction"],
            description="Assignment strategy: greedy (nearest-first), hungarian (optimal 1-to-1), or auction (market-based multi-agent).",
        ),
        DeclareLaunchArgument(
            "enclosure_radius",
            default_value="25.0",
            description="Voronoi enclosure radius (m).",
        ),
        DeclareLaunchArgument(
            "min_dist",
            default_value="5.0",
            description="Voronoi min-dist (m).",
        ),
    ]

    # ---------- tracker ---------------------------------------------------
    tracker_node = Node(
        package="perception_pkg",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        parameters=[
            {
                "input_mode": "video",
                "video_source": LaunchConfiguration("video_source"),
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "loop_video": True,
                "track_topic": "/target_track",
                "tracker.kind": "deepsort_cascade",
                "enclosure.enabled": True,
                "enclosure.topic": "/enclosure_targets",
                "enclosure.publish_rate_hz": 5.0,
            }
        ],
    )

    # ---------- coord transform (pixel → world) ---------------------------
    coord_transform_node = Node(
        package="perception_pkg",
        executable="coord_transform_node",
        name="coord_transform_node",
        output="screen",
        parameters=[
            {
                "enabled": True,
                "input_topic": "/target_track",
                "output_topic": "/target_track_world",
                "ground_altitude": 0.0,
                "camera_mount_pitch": 0.0,
                "max_pose_age_s": 0.5,
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

    # ---------- planner (real A*/D*Lite) ----------------------------------
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
                "log_interval_sec": 5.0,
                "publish_path": True,
                "sim_tick_speed": 1.0,
                "task_topic": "/task_assignment",
                "grid_topic": "/grid_map",
                "target_track_world_topic": "/target_track_world",
                "drone_states_topic": "/drone_states",
                "planned_path_topic": "/planned_path",
            }
        ],
    )

    # ---------- enclosure -------------------------------------------------
    # NOTE: enclosure_node topics are HARDCODED (not parameterized) —
    # subscribes /target_track + /enclosure_targets + /drone_states,
    # publishes /enclosure_command. Only these 3 params are declared.
    enclosure_node = Node(
        package="containment_pkg",
        executable="enclosure_node",
        name="enclosure_node",
        output="screen",
        parameters=[
            {
                "enclosure_radius": LaunchConfiguration("enclosure_radius"),
                "min_dist": LaunchConfiguration("min_dist"),
                "update_period": 1.0,
            }
        ],
    )

    return LaunchDescription(args + [
        tracker_node,
        coord_transform_node,
        scheduler_node,
        planner_node,
        enclosure_node,
    ])
