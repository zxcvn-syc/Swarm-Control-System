"""three_links.launch.py — bring up the full four-node integration.

Launches:

* ``tracker_node``           (perception_pkg)    → /target_track + /enclosure_targets
* ``scheduler_node``         (scheduler_pkg)     ← /target_track + /drone_states
                                                     → /task_assignment
* ``planner_stub_node``      (planner_stub)      ← /task_assignment + /target_track
                                                     → /drone_states + /drone_state
* ``enclosure_node``         (containment_pkg)   ← /enclosure_targets + /drone_states
                                                     → /enclosure_command

The integration test (``ros2_ws/test_three_links.py``) consumes the
same topic map; the values in this file are the binding truth and are
duplicated in ``docs/integration/interface_alignment.md`` (table
"Topic contract").

Why ``planner_stub_node`` (not a real planner_node)?

The ``planning_pkg`` slot is still empty — 程维好's
``planner_node`` has not landed.  Until that happens, the
``planner_stub_node`` publishes a synthetic ``DroneStateArray`` so the
second/third links close and the integration can be exercised
end-to-end.  Once the real ``planner_node`` ships, swap that single
launch entry and delete the planner_stub package.

Usage::

    cd /home/hhh/Downloads/Swarm-Control-System
    source ros2_ws/install/setup.bash
    ros2 launch ros2_ws/launch/three_links.launch.py \\
        video_source:=/abs/path/to/test_multi_target_tracking.mp4
    # or
    ./scripts/three_links_demo.sh
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
            default_value="/home/hhh/Downloads/Swarm-Control-System/videos/test_multi_target_tracking.mp4",
            description="Local video fed to tracker_node (input_mode=video).",
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
            choices=["greedy", "hungarian"],
            description="scheduler_node assignment strategy.",
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
                "tracker.kind": "deepsort_cascade",
                # Publish to both /target_track + /enclosure_targets.
                "enclosure.enabled": True,
                "enclosure.topic": "/enclosure_targets",
                "enclosure.publish_rate_hz": 5.0,
            },
        ],
    )

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

    planner_stub_node = Node(
        package="planner_stub",
        executable="planner_stub_node",
        name="planner_stub_node",
        output="screen",
        parameters=[
            {
                **params_common,
                "max_speed": 2.0,
                "tick_period": 0.5,
                "altitude": 5.0,
                "min_sep": 3.0,
                "frame_id": "world",
                "seed_grid_spacing": 6.0,
                "assignment_topic": "/task_assignment",
                "target_topic": "/target_track",
                "drone_states_topic": "/drone_states",
                "drone_state_topic": "/drone_state",
            },
        ],
    )

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

    return LaunchDescription(
        args + [tracker_node, scheduler_node, planner_stub_node, enclosure_node]
    )
