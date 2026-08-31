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
        DeclareLaunchArgument("operator_token_env", default_value="FLIGHT_SAFETY_TOKEN"),
        DeclareLaunchArgument("allow_remote_control", default_value="false"),
        DeclareLaunchArgument("status_stale_timeout", default_value="3.0"),
        DeclareLaunchArgument("enable_pilot_commands", default_value="false"),
        DeclareLaunchArgument("mavros_state_topic", default_value="/uav0/mavros/state"),
        DeclareLaunchArgument("arm_service", default_value="/uav0/mavros/cmd/arming"),
        DeclareLaunchArgument("mode_service", default_value="/uav0/mavros/set_mode"),
        DeclareLaunchArgument("pilot_state_stale_timeout", default_value="1.0"),
        DeclareLaunchArgument("pilot_safety_stale_timeout", default_value="1.0"),
        DeclareLaunchArgument("pilot_command_timeout", default_value="3.0"),
        DeclareLaunchArgument("pilot_audit_log", default_value=""),
        DeclareLaunchArgument("position_mode", default_value="POSCTL"),
        DeclareLaunchArgument("altitude_mode", default_value="ALTCTL"),
        DeclareLaunchArgument("offboard_mode", default_value="OFFBOARD"),
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
                "operator_token_env": LaunchConfiguration("operator_token_env"),
                "allow_remote_control": LaunchConfiguration("allow_remote_control"),
                "status_stale_timeout": LaunchConfiguration("status_stale_timeout"),
                "enable_pilot_commands": LaunchConfiguration("enable_pilot_commands"),
                "mavros_state_topic": LaunchConfiguration("mavros_state_topic"),
                "arm_service": LaunchConfiguration("arm_service"),
                "mode_service": LaunchConfiguration("mode_service"),
                "pilot_state_stale_timeout": LaunchConfiguration(
                    "pilot_state_stale_timeout"
                ),
                "pilot_safety_stale_timeout": LaunchConfiguration(
                    "pilot_safety_stale_timeout"
                ),
                "pilot_command_timeout": LaunchConfiguration("pilot_command_timeout"),
                "pilot_audit_log": LaunchConfiguration("pilot_audit_log"),
                "position_mode": LaunchConfiguration("position_mode"),
                "altitude_mode": LaunchConfiguration("altitude_mode"),
                "offboard_mode": LaunchConfiguration("offboard_mode"),
            }
        ],
    )
    return LaunchDescription(arguments + [dashboard])
