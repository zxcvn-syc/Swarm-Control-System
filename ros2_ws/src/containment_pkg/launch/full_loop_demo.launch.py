"""Full closed-loop demo: enclosure -> bridge -> task_assignment.

Brings up the containment half of the system so the closed loop can be
verified end-to-end without the full perception/planning stack:

    target_pub       -> /target_track
    mock_platform_pub -> /drone_states   (3 UAV + 2 UGV, platform_type set)
    enclosure_node    -> /enclosure_command  (three-layer Voronoi points)
    enclosure_command_bridge -> /task_assignment  (encoded for planner_node)

To connect a real planner_node (planning_pkg), replace mock_platform_pub
with:

    planner_node (platform_type=0, num_drones=3) -> /drone_states
    ugv_state_publisher (num_ugv=2)            -> /drone_states  (*)
    (* use a state_aggregator if both publish to the same topic)

The bridge encodes enclosure points as target_id = x + y*grid_size,
which planner_node._scatter_target decodes back to grid coordinates.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    target_pub = Node(
        package="containment_pkg",
        executable="target_pub",
        name="target_pub",
        output="screen",
        parameters=[{
            "publish_rate": 1.0,
            "center_x": 50.0,
            "center_y": 50.0,
            "radius": 15.0,
            "speed": 0.3,
        }],
    )

    mock_platform_pub = Node(
        package="containment_pkg",
        executable="mock_platform_pub",
        name="mock_platform_pub",
        output="screen",
        parameters=[{
            "publish_rate": 1.0,
            "center_x": 50.0,
            "center_y": 50.0,
            "drone_radius": 30.0,
            "car_radius": 15.0,
        }],
    )

    enclosure_node = Node(
        package="containment_pkg",
        executable="enclosure_node",
        name="enclosure_node",
        output="screen",
        parameters=[{
            "monitor_radius": 25.0,
            "block_radius": 15.0,
            "min_dist": 5.0,
            "update_period": 1.0,
        }],
    )

    enclosure_command_bridge = Node(
        package="containment_pkg",
        executable="enclosure_command_bridge",
        name="enclosure_command_bridge",
        output="screen",
        parameters=[{
            "command_topic": "/enclosure_command",
            "output_topic": "/task_assignment",
            "grid_size": 100,
            "task_type": "enclose",
        }],
    )

    return LaunchDescription([
        target_pub,
        mock_platform_pub,
        enclosure_node,
        enclosure_command_bridge,
    ])
