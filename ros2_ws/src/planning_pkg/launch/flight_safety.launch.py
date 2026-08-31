"""Start the fail-closed containment safety supervisor.

The supervisor begins in ``LOCKED`` and publishes a durable hold request.
It gates ``/enclosure_command`` onto ``/flight_safety/enclosure_command``;
point ``enclosure_command_bridge.command_topic`` at the latter topic when
running a supervised containment chain.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("raw_command_topic", default_value="/enclosure_command"),
        DeclareLaunchArgument("gated_command_topic", default_value="/flight_safety/enclosure_command"),
        DeclareLaunchArgument("target_topic", default_value="/target_track_world"),
        DeclareLaunchArgument("drone_states_topic", default_value="/drone_states"),
        DeclareLaunchArgument("mavros_state_topic", default_value="/uav0/mavros/state"),
        DeclareLaunchArgument("require_mavros_connection", default_value="false"),
    ]
    supervisor = Node(
        package="planning_pkg",
        executable="flight_safety_supervisor",
        name="flight_safety_supervisor",
        output="screen",
        parameters=[
            {
                "raw_command_topic": LaunchConfiguration("raw_command_topic"),
                "gated_command_topic": LaunchConfiguration("gated_command_topic"),
                "target_topic": LaunchConfiguration("target_topic"),
                "drone_states_topic": LaunchConfiguration("drone_states_topic"),
                "mavros_state_topic": LaunchConfiguration("mavros_state_topic"),
                "require_mavros_connection": LaunchConfiguration(
                    "require_mavros_connection"
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [supervisor])
