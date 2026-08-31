"""Start the locked safety gate and local browser pilot console for one UAV."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Keep the command surface explicit while defaulting to read-only operation."""

    arguments = [
        DeclareLaunchArgument("bind_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("operator_token", default_value=""),
        DeclareLaunchArgument("operator_token_env", default_value="FLIGHT_SAFETY_TOKEN"),
        DeclareLaunchArgument("allow_remote_control", default_value="false"),
        DeclareLaunchArgument("enable_pilot_commands", default_value="false"),
        DeclareLaunchArgument("pilot_audit_log", default_value=""),
        DeclareLaunchArgument("video_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("target_topic", default_value="/target_track_world"),
        DeclareLaunchArgument("drone_states_topic", default_value="/drone_states"),
        DeclareLaunchArgument("mavros_state_topic", default_value="/uav0/mavros/state"),
        DeclareLaunchArgument("arm_service", default_value="/uav0/mavros/cmd/arming"),
        DeclareLaunchArgument("mode_service", default_value="/uav0/mavros/set_mode"),
        DeclareLaunchArgument("position_mode", default_value="POSCTL"),
        DeclareLaunchArgument("altitude_mode", default_value="ALTCTL"),
        DeclareLaunchArgument("offboard_mode", default_value="OFFBOARD"),
    ]
    supervisor = Node(
        package="planning_pkg",
        executable="flight_safety_supervisor",
        name="flight_safety_supervisor",
        output="screen",
        parameters=[
            {
                "target_topic": LaunchConfiguration("target_topic"),
                "drone_states_topic": LaunchConfiguration("drone_states_topic"),
                "mavros_state_topic": LaunchConfiguration("mavros_state_topic"),
                "require_mavros_connection": True,
            }
        ],
    )
    dashboard = Node(
        package="planning_pkg",
        executable="flight_safety_dashboard",
        name="flight_safety_dashboard",
        output="screen",
        parameters=[
            {
                "bind_address": LaunchConfiguration("bind_address"),
                "port": LaunchConfiguration("port"),
                "video_topic": LaunchConfiguration("video_topic"),
                "operator_token": LaunchConfiguration("operator_token"),
                "operator_token_env": LaunchConfiguration("operator_token_env"),
                "allow_remote_control": LaunchConfiguration("allow_remote_control"),
                "enable_pilot_commands": LaunchConfiguration("enable_pilot_commands"),
                "pilot_audit_log": LaunchConfiguration("pilot_audit_log"),
                "mavros_state_topic": LaunchConfiguration("mavros_state_topic"),
                "arm_service": LaunchConfiguration("arm_service"),
                "mode_service": LaunchConfiguration("mode_service"),
                "position_mode": LaunchConfiguration("position_mode"),
                "altitude_mode": LaunchConfiguration("altitude_mode"),
                "offboard_mode": LaunchConfiguration("offboard_mode"),
            }
        ],
    )
    return LaunchDescription(arguments + [supervisor, dashboard])
