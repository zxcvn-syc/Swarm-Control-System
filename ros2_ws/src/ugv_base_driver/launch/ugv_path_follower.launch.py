"""Launch the UGV path follower (/planned_path -> /cmd_vel).

Typical pairing with the base driver:

    ros2 launch ugv_base_driver ugv_base_driver.launch.py
    ros2 launch ugv_base_driver ugv_path_follower.launch.py target_frame_id:=drone_4

The follower is deliberately a separate launch file: on the bench you
often want to drive the base with a manual ``/cmd_vel`` publisher while
testing the follower's output with ``ros2 topic echo /cmd_vel``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "path_topic",
                default_value="/planned_path",
                description="nav_msgs/Path published by planner_node",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/ugv_pose",
                description="UGV pose in the same world frame as the path",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Twist output consumed by ugv_base_driver",
            ),
            DeclareLaunchArgument(
                "target_frame_id",
                default_value="",
                description="Keep only poses tagged drone_<id>; empty keeps all",
            ),
            DeclareLaunchArgument(
                "max_linear_speed",
                default_value="0.5",
                description="Forward speed cap, m/s",
            ),
            DeclareLaunchArgument(
                "max_angular_speed",
                default_value="1.2",
                description="Yaw rate cap, rad/s",
            ),
            DeclareLaunchArgument(
                "lookahead_distance",
                default_value="0.6",
                description="Pure-pursuit lookahead, m",
            ),
            Node(
                package="ugv_base_driver",
                executable="ugv_path_follower",
                name="ugv_path_follower",
                output="screen",
                parameters=[
                    {
                        "path_topic": LaunchConfiguration("path_topic"),
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "target_frame_id": LaunchConfiguration("target_frame_id"),
                        "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                        "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                        "lookahead_distance": LaunchConfiguration("lookahead_distance"),
                        "goal_tolerance": 0.25,
                        "slowdown_radius": 1.0,
                        "pose_timeout": 0.5,
                        "path_timeout": 2.0,
                    },
                ],
            ),
        ]
    )
