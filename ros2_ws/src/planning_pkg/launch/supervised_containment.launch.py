"""Run the enclosure producer and bridge through the fail-closed safety gate.

This launch deliberately does not start a vehicle or alter PX4 state. It is
the containment-side integration point for a separately launched planner and
Offboard bridge. The bridge receives only commands accepted by the safety
supervisor, and the enclosure producer emits a fresh heartbeat while active.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("target_topic", default_value="/target_track_world"),
        DeclareLaunchArgument("drone_states_topic", default_value="/drone_states"),
        DeclareLaunchArgument("update_period", default_value="1.0"),
        DeclareLaunchArgument("grid_size", default_value="100"),
        DeclareLaunchArgument("grid_resolution", default_value="1.0"),
        DeclareLaunchArgument("require_mavros_connection", default_value="false"),
        DeclareLaunchArgument("mavros_state_topic", default_value="/uav0/mavros/state"),
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
                "require_mavros_connection": LaunchConfiguration(
                    "require_mavros_connection"
                ),
                "mavros_state_topic": LaunchConfiguration("mavros_state_topic"),
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
                "target_track_topic": LaunchConfiguration("target_topic"),
                "update_period": LaunchConfiguration("update_period"),
                "publish_heartbeat": True,
            }
        ],
    )
    bridge = Node(
        package="containment_pkg",
        executable="enclosure_command_bridge",
        name="enclosure_command_bridge",
        output="screen",
        parameters=[
            {
                "command_topic": "/flight_safety/enclosure_command",
                "output_topic": "/task_assignment",
                "grid_size": LaunchConfiguration("grid_size"),
                "resolution": LaunchConfiguration("grid_resolution"),
                "task_type": "enclose",
            }
        ],
    )
    return LaunchDescription(arguments + [supervisor, enclosure, bridge])
