"""Launch the scheduler node with the checked-in default parameters."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = Path(__file__).resolve().parents[1] / "config" / "scheduler.yaml"
    return LaunchDescription([
        Node(
            package="scheduler_pkg",
            executable="scheduler_node",
            name="scheduler_node",
            output="screen",
            parameters=[str(config_file)],
        ),
    ])
