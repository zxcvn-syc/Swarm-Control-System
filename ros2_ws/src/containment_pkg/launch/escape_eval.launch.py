"""Headless single-scene enclosure test: mock platforms + escape target.

Brings up the containment half of the system (no real tracker / SITL) so the
evaluator can verify the SUCCESS/FAIL judging logic end-to-end:

    mock_platform_pub   -> /drone_states      (3 UAV + 2 UGV)
    escape_test_node    -> /enclosure_targets (scripted moving target)
    enclosure_node      -> /enclosure_command (three-layer Voronoi points)
    containment_evaluator -> verdict + CSV row

Run one scenario:
  ros2 launch containment_pkg escape_eval.launch.py \
      scene:=park direction:=2 trajectory:=return \
      monitor_radius:=25.0 block_radius:=15.0 \
      result_csv:=./eval_results.csv

The evaluator process self-terminates with a [VERDICT] log line and writes one
row to ``result_csv``.  For the 8.27-8.28 batch runs, drive this launch from a
shell loop (one launch per scene/direction repeat) instead of running it
interactively.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scene = LaunchConfiguration("scene")
    direction = LaunchConfiguration("direction")
    trajectory = LaunchConfiguration("trajectory")
    speed = LaunchConfiguration("speed")
    monitor_radius = LaunchConfiguration("monitor_radius")
    block_radius = LaunchConfiguration("block_radius")
    test_duration = LaunchConfiguration("test_duration")
    result_csv = LaunchConfiguration("result_csv")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="park"),
        DeclareLaunchArgument("direction", default_value="2"),
        DeclareLaunchArgument("trajectory", default_value="return"),
        DeclareLaunchArgument("speed", default_value="2.0"),
        DeclareLaunchArgument("monitor_radius", default_value="25.0"),
        DeclareLaunchArgument("block_radius", default_value="15.0"),
        DeclareLaunchArgument("test_duration", default_value="20.0"),
        DeclareLaunchArgument("result_csv", default_value="./eval_results.csv"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("containment_pkg"),
                 "config", "three_scene_config.yaml"]
            ),
            description="Path to three_scene_config.yaml; per-scene params "
                        "override launch args when the scene is found.",
        ),

        # 3 UAV (monitor) + 2 UGV (block), orbiting the start point.
        Node(
            package="containment_pkg",
            executable="mock_platform_pub",
            name="mock_platform_pub",
            output="screen",
            parameters=[{
                "period": 0.5,
                "target_x": start_x,
                "target_y": start_y,
                "monitor_orbit": 25.0,
                "block_orbit": 15.0,
                "num_drones": 3,
                "num_cars": 2,
            }],
        ),

        # Scripted escape target.
        Node(
            package="containment_pkg",
            executable="escape_test_node",
            name="escape_test_node",
            output="screen",
            parameters=[{
                "period": 0.1,
                "start_x": start_x,
                "start_y": start_y,
                "scene_name": scene,
                "escape_direction": direction,
                "trajectory": trajectory,
                "speed": speed,
                "test_duration": test_duration,
                "config_file": config_file,
            }],
        ),

        # Three-layer enclosure.  Consume the escape target directly (disable
        # the /target_track_world primary source for the headless path).
        Node(
            package="containment_pkg",
            executable="enclosure_node",
            name="enclosure_node",
            output="screen",
            parameters=[{
                "monitor_radius": monitor_radius,
                "block_radius": block_radius,
                "min_dist": 5.0,
                "update_period": 0.5,
                "target_track_topic": "",
                "enclosure_target_topic": "/enclosure_targets",
            }],
        ),

        # Judge + record.
        Node(
            package="containment_pkg",
            executable="containment_evaluator",
            name="containment_evaluator",
            output="screen",
            parameters=[{
                "scene_name": scene,
                "escape_direction": direction,
                "trajectory": trajectory,
                "monitor_radius": monitor_radius,
                "block_radius": block_radius,
                "test_duration": test_duration,
                "result_csv": result_csv,
                "config_file": config_file,
            }],
        ),
    ])
