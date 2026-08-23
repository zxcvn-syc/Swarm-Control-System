"""Launch the UGV block-layer containment demo.

Runs three nodes needed to exercise the UGV (无人车) block layer end-to-end:

  target_pub        -> publishes a moving target on /target_track
  mock_platform_pub -> publishes 3 UAVs + 2 UGVs on /drone_states
  enclosure_node    -> computes layered Voronoi enclosure commands

Launch with:
    ros2 launch containment_pkg ugv_block_demo.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="containment_pkg",
            executable="target_pub",
            name="target_pub",
            output="screen",
            parameters=[{
                "period": 0.5,
                "num_targets": 1,
                "center_x": 0.0,
                "center_y": 0.0,
                "orbit_radius": 3.0,
                "orbit_speed": 0.3,
            }],
        ),
        Node(
            package="containment_pkg",
            executable="mock_platform_pub",
            name="mock_platform_pub",
            output="screen",
            parameters=[{
                "period": 0.5,
                "num_drones": 3,   # UAVs  -> monitor layer (25.0 m)
                "num_cars": 2,     # UGVs  -> block layer (15.0 m)
                "monitor_orbit": 30.0,
                "block_orbit": 18.0,
            }],
        ),
        Node(
            package="containment_pkg",
            executable="enclosure_node",
            name="enclosure_node",
            output="screen",
            parameters=[
                "config/containment.yaml",
                {"target_track_topic": "/target_track"},
            ],
        ),
    ])
