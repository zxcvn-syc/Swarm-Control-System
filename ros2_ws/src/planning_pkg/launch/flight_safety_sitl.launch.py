"""Launch PX4 SITL with the fail-closed flight-safety interlock enabled."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("planning_pkg"))
    px4_sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "px4_sitl.launch.py")),
        # Containment activation must not arm the vehicle.  A SITL operator
        # may deliberately arm/select Offboard through the usual PX4 path.
        launch_arguments={
            "enable_flight_safety": "true",
            "auto_arm": "false",
        }.items(),
    )
    supervisor = Node(
        package="planning_pkg",
        executable="flight_safety_supervisor",
        name="flight_safety_supervisor",
        output="screen",
        parameters=[
            {
                "mavros_state_topic": "/uav0/mavros/state",
                "require_mavros_connection": True,
            }
        ],
    )
    return LaunchDescription([supervisor, px4_sitl])
