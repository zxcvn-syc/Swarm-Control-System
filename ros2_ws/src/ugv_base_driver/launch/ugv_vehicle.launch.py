"""Launch the complete planned-path-to-serial control chain for one UGV."""

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
        DeclareLaunchArgument("vehicle_id", default_value="100"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument(
            "protocol",
            default_value="text",
            choices=["text", "text_rpm"],
        ),
        DeclareLaunchArgument("path_topic", default_value="/planned_path"),
        DeclareLaunchArgument("odom_topic", default_value="odom"),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument(
            "depth_topic", default_value="camera/depth/image_raw"
        ),
        DeclareLaunchArgument(
            "state_topic", default_value="/ground_vehicle_states"
        ),
        DeclareLaunchArgument("output_frame", default_value="world"),
        DeclareLaunchArgument("path_resolution", default_value="0.5"),
        DeclareLaunchArgument("require_lidar", default_value="true"),
        DeclareLaunchArgument("require_depth", default_value="true"),
    ]
    config = LaunchConfiguration("config")
    namespace = LaunchConfiguration("vehicle_namespace")
    vehicle_id = ParameterValue(
        LaunchConfiguration("vehicle_id"), value_type=int
    )

    state_bridge = Node(
        package="ugv_base_driver",
        executable="ugv_odom_state_bridge",
        name="ugv_odom_state_bridge",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            config,
            {
                "vehicle_id": vehicle_id,
                "odom_topic": LaunchConfiguration("odom_topic"),
                "state_topic": LaunchConfiguration("state_topic"),
                "output_frame": LaunchConfiguration("output_frame"),
            },
        ],
    )
    follower = Node(
        package="ugv_base_driver",
        executable="ugv_path_follower",
        name="ugv_path_follower",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            config,
            {
                "vehicle_id": vehicle_id,
                "path_topic": LaunchConfiguration("path_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "path_resolution": ParameterValue(
                    LaunchConfiguration("path_resolution"), value_type=float
                ),
            },
        ],
    )
    obstacle_guard = Node(
        package="ugv_base_driver",
        executable="ugv_obstacle_guard",
        name="ugv_obstacle_guard",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            config,
            {
                "scan_topic": LaunchConfiguration("scan_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "require_lidar": ParameterValue(
                    LaunchConfiguration("require_lidar"), value_type=bool
                ),
                "require_depth": ParameterValue(
                    LaunchConfiguration("require_depth"), value_type=bool
                ),
            },
        ],
    )
    driver = Node(
        package="ugv_base_driver",
        executable="ugv_base_driver",
        name="ugv_base_driver",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            config,
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baudrate": ParameterValue(
                    LaunchConfiguration("baudrate"), value_type=int
                ),
                "protocol": LaunchConfiguration("protocol"),
            },
        ],
    )
    return LaunchDescription(
        arguments + [state_bridge, follower, obstacle_guard, driver]
    )
