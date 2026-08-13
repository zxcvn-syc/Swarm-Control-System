"""integration_test.launch.py — programmatic three-link end-to-end exercise.

Brings up:

1. ``tracker_node`` (perception_pkg)   — driven from a local video.
2. ``scheduler_node`` (scheduler_pkg)   — consumes tracker output.
3. ``planner_node`` (planning_pkg)   — real A*/D*Lite path planning.
4. ``coord_transform_node`` (perception_pkg) — pixel→world coordinate transform.
5. ``enclosure_node`` (containment_pkg) — closes the third link.

Plus auxiliary nodes that *only exist to make the link visible to the
outside world*:

5. ``topic_aggregator`` (test_aggregator_pkg) — records each of the
   four integration topics into a JSON snapshot file
   (``/tmp/integration_aggregator_<timestamp>.json``) once the test
   window elapses.  This is the harness that drives
   ``test_three_links.py``'s reporter without spawning
   ``rclpy``-from-CLI.

The video source and the test window are launch arguments so the same
launch can be reused for ``videos/test_multi_target_tracking.mp4``,
``videos/test_fast_motion.mp4`` etc.

Usage::

    cd /home/hhh/Downloads/Swarm-Control-System
    source ros2_ws/install/setup.bash
    ros2 launch ros2_ws/launch/integration_test.launch.py \\
        video_source:=/abs/path/to/test_multi_target_tracking.mp4 \\
        window_sec:=30 \\
        report_path:=/tmp/integration_report.json
"""

from __future__ import annotations

import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    repo_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )
    integration_script = os.path.join(repo_root, "ros2_ws", "test_three_links.py")

    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "video_source",
            default_value=os.path.join(
                repo_root,
                "videos",
                "test_multi_target_tracking.mp4",
            ),
            description="Video fed to tracker_node.",
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
            description="scheduler_node assignment strategy.",
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
            description="Voronoi enclosure radius.",
        ),
        DeclareLaunchArgument(
            "min_dist",
            default_value="5.0",
            description="Voronoi min-dist.",
        ),
        DeclareLaunchArgument(
            "window_sec",
            default_value="12.0",
            description="How long the in-process test_aggregator should run.",
        ),
        DeclareLaunchArgument(
            "report_path",
            default_value="/tmp/integration_test_report.json",
            description="Where the in-process aggregator writes its result.",
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
            },
        ],
    )

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

    # Run the Python integration test as a sibling process so the
    # user can `cat $report_path` afterwards to see the structured
    # pass/fail summary that drove by the three-link harness.  We
    # use an OpaqueFunction so the LaunchConfiguration values are
    # resolved at launch time, with a valid launch context.
    from launch.actions import OpaqueFunction

    def _spawn_integration_test(context, *args, **kwargs):  # noqa: ANN001
        import subprocess  # local import — kept out of static parsing
        window = float(LaunchConfiguration("window_sec").perform(context))
        report_dir = os.path.dirname(
            LaunchConfiguration("report_path").perform(context)
        ) or "."
        cmd = [
            "python3",
            integration_script,
            "--window",
            str(window),
            "--output",
            report_dir,
        ]
        return [
            ExecuteProcess(
                cmd=cmd,
                name="integration_test_runner",
                output="screen",
                shell=False,
            )
        ]

    integration_run = OpaqueFunction(function=_spawn_integration_test)

    return LaunchDescription(
        args
        + [
            tracker_node,
            coord_transform_node,
            scheduler_node,
            planner_node,
            enclosure_node,
            # OpaqueFunction spawns the integration test runner so the
            # LaunchConfiguration substitutions for window_sec /
            # report_path resolve correctly.  We give the four nodes
            # a 3-second head-start via a TimerAction to ensure the
            # publishers are registered before the test subscribes.
            TimerAction(period=3.0, actions=[integration_run]),
        ]
    )
