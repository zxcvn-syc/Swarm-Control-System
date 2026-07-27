from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="containment_pkg",
            executable="enclosure_node",
            name="enclosure_node",
            output="screen",
            parameters=["config/containment.yaml"],
        ),
    ])
