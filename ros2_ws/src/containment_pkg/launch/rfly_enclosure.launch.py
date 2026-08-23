"""Three-layer Voronoi containment for the RflySim air-ground scene.

Pairs with the CVTrack RflySim demo chain (rfly_ros_scene.py +
rfly_live_cvtrack.py):

    rfly_ros_scene -> /target_track_world  (vision-triggered world target)
    rfly_ros_scene -> /drone_pose_external (UAV states, platform_type=0)
    rfly_ros_scene -> /ground_vehicle_states (UGV states, platform_type=1)

This launch file starts:

1. ``platform_state_merger`` -- merges the two platform topics into the
   single ``/drone_states`` stream that enclosure_node subscribes to.
2. ``enclosure_node`` -- three-layer Voronoi containment:
   UAVs on the 25 m monitor ring, UGVs on the 15 m block ring.

The resulting ``/enclosure_command`` topic carries commands for all layers.
Downstream consumers (e.g. rfly_ros_scene.on_enclosure) should filter by
``enclosure_radius`` to pick the layer they actuate: UGV interceptors take
commands whose radius matches the block radius (15 m).
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    merger = Node(
        package="containment_pkg",
        executable="platform_state_merger",
        name="platform_state_merger",
        output="screen",
        parameters=[
            {
                "uav_topic": "/drone_pose_external",
                "ugv_topic": "/ground_vehicle_states",
                "output_topic": "/drone_states",
                "publish_period": 0.25,
            }
        ],
    )

    enclosure = Node(
        package="containment_pkg",
        executable="enclosure_node",
        name="enclosure_node",
        output="screen",
        parameters=[
            {
                # Three-layer mode: explicit radii override the legacy
                # single-radius enclosure_radius fallback.
                "monitor_radius": 25.0,
                "block_radius": 15.0,
                "min_dist": 5.0,
                "update_period": 0.25,
                "target_track_topic": "/target_track_world",
            }
        ],
    )

    return LaunchDescription(
        [
            merger,
            # Small delay so the merged /drone_states stream exists before
            # the enclosure node starts consuming it.
            TimerAction(period=1.0, actions=[enclosure]),
        ]
    )
