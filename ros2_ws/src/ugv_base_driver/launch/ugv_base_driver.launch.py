"""Launch the UGV base driver with configurable serial parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device of the base board",
            ),
            DeclareLaunchArgument(
                "baudrate",
                default_value="115200",
                description="Serial baudrate",
            ),
            DeclareLaunchArgument(
                "protocol",
                default_value="text",
                description="Serial command protocol: text | text_rpm",
            ),
            Node(
                package="ugv_base_driver",
                executable="ugv_base_driver",
                name="ugv_base_driver",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchArgument("serial_port"),
                        "baudrate": LaunchArgument("baudrate"),
                        "protocol": LaunchArgument("protocol"),
                    },
                    # geometry: change to match the actual vehicle
                    {"wheel_base": 0.4},
                    {"wheel_radius": 0.075},
                    {"max_linear_speed": 1.0},
                    {"max_angular_speed": 1.0},
                    {"watchdog_timeout": 0.5},
                ],
            ),
        ]
    )
