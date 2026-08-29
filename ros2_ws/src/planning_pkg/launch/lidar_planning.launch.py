"""Launch one 2D LiDAR occupancy source together with the planner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("planning_pkg"), "config", "lidar.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("pose_topic", default_value="/drone_pose_external"),
        DeclareLaunchArgument("sensor_id", default_value="100"),
        Node(
            package="planning_pkg",
            executable="lidar_grid_node",
            name="lidar_grid_node",
            output="screen",
            parameters=[config, {
                "scan_topic": LaunchConfiguration("scan_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "sensor_id": LaunchConfiguration("sensor_id"),
            }],
        ),
        Node(
            package="planning_pkg",
            executable="planner_node",
            name="planner_node",
            output="screen",
            parameters=[config],
        ),
    ])
