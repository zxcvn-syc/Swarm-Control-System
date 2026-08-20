"""three_links.launch.py — bring up the full eight-node integration.
Launches:
* ``tracker_node``           (perception_pkg)    → /target_track + /enclosure_targets
* ``coord_transform_node``   (perception_pkg)    → /target_track_world (pixel→world)
* ``scheduler_node``         (scheduler_pkg)     ← /target_track + /drone_states
                                                     → /task_assignment
* ``planner_node``           (planning_pkg)      ← /task_assignment + /target_track_world
                                                     → /drone_states + /planned_path
* ``enclosure_node``         (containment_pkg)   ← /enclosure_targets + /drone_states
                                                     → /enclosure_command
* ``ugv_state_publisher``    (planning_pkg)      → /ugv_states
* ``px4_offboard_bridge``    (planning_pkg)      ← /planned_path
                                                     → /uav0/mavros/setpoint_raw/local
* ``sitl_pose_bridge``       (planning_pkg)      ← /uav0/mavros/local_position/pose
                                                     → /drone_pose_external
The integration test (``ros2_ws/test_three_links.py``) consumes the
same topic map; the values in this file are the binding truth and are
duplicated in ``docs/integration/interface_alignment.md`` (table
"Topic contract").
Update history (v2.3, 2026-08-06):
- Replaced ``planner_stub_node`` with real ``planner_node`` from planning_pkg
  (A*/D*Lite path planning is now fully implemented with 23 tests passing)
- Added ``coord_transform_node`` as a permanent resident node for pixel→world
  coordinate transformation
- Added ``auction`` strategy option (auction algorithm merged to main in commit 80d2a1e,
  scheduler_node now supports greedy/hungarian/auction three strategies)
- Removed outdated docstring about planner_node "not landing"
- default video_source is empty; ALWAYS pass ``video_source:=`` explicitly
  or use ``./scripts/three_links_demo.sh`` which handles this automatically.
- FIXED: planner_node parameter name ``target_topic`` → ``target_track_world_topic``
  (matches actual declare_parameter in planner_node.py)
- FIXED: removed non-existent ``target_topic``/``drone_topic`` params from
  enclosure_node (topics are hardcoded in enclosure_node.py, not parameterized)
Usage::
    cd /path/to/Swarm-Control-System
    source ros2_ws/install/setup.bash
    # Dry-run (build + validate launch args)
    ./scripts/three_links_demo.sh --dry-run
    # Run with default video (videos/test_multi_target_tracking.mp4)
    ./scripts/three_links_demo.sh
    # Run with auction strategy
    ros2 launch ros2_ws/launch/three_links.launch.py \\
        video_source:=/abs/path/to/video.mp4 scheduler_strategy:=auction
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
            description="Local video fed to tracker_node (input_mode=video). REQUIRED; demo.sh passes this automatically.",
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
            "num_drones",
            default_value="8",
            description="Scheduler + planner swarm size.",
        ),
        DeclareLaunchArgument(
            "scheduler_strategy",
            default_value="greedy",
            choices=["greedy", "hungarian", "auction"],
            description="scheduler_node assignment strategy: greedy (nearest-first), hungarian (optimal 1-to-1), or auction (market-based multi-agent).",
        ),
        DeclareLaunchArgument(
            "planner",
            default_value="astar",
            choices=["astar", "dstar_lite"],
            description="Path planner to use (forwarded into planning_pkg).",
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

    params_common = {
        "num_drones": LaunchConfiguration("num_drones"),
    }

    # ---------- tracker (perception: pixel-level) --------------------------
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
                # Publish to both /target_track + /enclosure_targets.
                "enclosure.enabled": True,
                "enclosure.topic": "/enclosure_targets",
                "enclosure.publish_rate_hz": 5.0,
            },
        ],
    )

    # ---------- coord transform (pixel → world ENU) ------------------------
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
                "camera_mount_pitch": 0.0,  # nadir-facing by default
                "max_pose_age_s": 0.5,
            },
        ],
    )

    # ---------- scheduler (task assignment) --------------------------------
    scheduler_node = Node(
        package="scheduler_pkg",
        executable="scheduler_node",
        name="scheduler_node",
        output="screen",
        parameters=[
            {
                **params_common,
                "assignment_strategy": LaunchConfiguration("scheduler_strategy"),
                "max_per_drone": 2,
                "tick_period": 0.5,
                "log_interval_sec": 5.0,
                "target_topic": "/target_track",
                "drone_topic": "/drone_states",
                "output_topic": "/task_assignment",
                "default_task_type": "track",
            },
        ],
    )

    # ---------- planner (real A*/D*Lite path planning) ---------------------
    planner_node = Node(
        package="planning_pkg",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            {
                **params_common,
                "planner": LaunchConfiguration("planner"),
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
            },
        ],
    )

    # ---------- enclosure (Voronoi containment) ----------------------------
    # NOTE: enclosure_node.py does NOT parameterize topic names (topics are
    # hardcoded: subscribes /target_track + /enclosure_targets + /drone_states,
    # publishes /enclosure_command). Only radius/min_dist/update_period are
    # declared parameters.
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
            },
        ],
    )

    ugv_state_pub = Node(
        package="planning_pkg",
        executable="ugv_state_pub",
        name="ugv_state_publisher",
        output="screen",
        parameters=[{"num_ugv": 2}],
    )

    px4_offboard_bridge = Node(
        package="planning_pkg",
        executable="px4_offboard_bridge",
        name="px4_offboard_bridge",
        output="screen",
        parameters=[{}],
    )

    sitl_pose_bridge = Node(
        package="planning_pkg",
        executable="sitl_pose_bridge",
        name="sitl_pose_bridge",
        output="screen",
        parameters=[{"platform_type": 0}],
    )

    return LaunchDescription(
        args + [
            tracker_node,
            coord_transform_node,
            scheduler_node,
            planner_node,
            enclosure_node,
            ugv_state_pub,
            px4_offboard_bridge,
            sitl_pose_bridge,
        ]
    )
