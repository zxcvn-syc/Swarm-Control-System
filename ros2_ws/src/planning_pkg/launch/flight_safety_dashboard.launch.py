"""Launch the local browser dashboard for the flight-safety supervisor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("bind_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("status_topic", default_value="/flight_safety/status"),
        DeclareLaunchArgument("control_service", default_value="/flight_safety/control"),
        DeclareLaunchArgument("video_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("operator_token", default_value=""),
        DeclareLaunchArgument("allow_remote_control", default_value="false"),
        DeclareLaunchArgument("status_stale_timeout", default_value="3.0"),
    ]
    dashboard = Node(
        package="planning_pkg",
        executable="flight_safety_dashboard",
        name="flight_safety_dashboard",
        output="screen",
        parameters=[
            {
                "bind_address": LaunchConfiguration("bind_address"),
                "port": LaunchConfiguration("port"),
                "status_topic": LaunchConfiguration("status_topic"),
                "control_service": LaunchConfiguration("control_service"),
                "video_topic": LaunchConfiguration("video_topic"),
                "operator_token": LaunchConfiguration("operator_token"),
                "allow_remote_control": LaunchConfiguration("allow_remote_control"),
                "status_stale_timeout": LaunchConfiguration("status_stale_timeout"),
            }
        ],
    )
    return LaunchDescription(arguments + [dashboard])
