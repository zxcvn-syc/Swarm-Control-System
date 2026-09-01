"""Launch only the guarded serial base driver for manual bench control."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("ugv_base_driver"),
        "config",
        "ugv_base_driver.yaml",
    )
    arguments = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("vehicle_namespace", default_value="ugv_100"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument(
            "protocol",
            default_value="text",
            choices=["text", "text_rpm"],
        ),
    ]
    driver = Node(
        package="ugv_base_driver",
        executable="ugv_base_driver",
        name="ugv_base_driver",
        namespace=LaunchConfiguration("vehicle_namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config"),
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baudrate": ParameterValue(
                    LaunchConfiguration("baudrate"), value_type=int
                ),
                "protocol": LaunchConfiguration("protocol"),
            },
        ],
    )
    return LaunchDescription(arguments + [driver])
