"""SITL single-scene enclosure test: real platform poses + scripted target.

Differs from ``escape_eval.launch.py`` only in the platform source:

* ``platform_state_merger`` merges ``/drone_pose_external`` (UAVs from the
  RflySim/PX4 SITL scene bridge) and ``/ground_vehicle_states`` (UGVs) into
  ``/drone_states`` for the evaluator.
* The other three nodes are reused unchanged:
    * ``escape_test_node`` -> ``/enclosure_targets`` (scripted intruder)
    * ``enclosure_node``   -> ``/enclosure_command`` (Voronoi assignment)
    * ``containment_evaluator`` -> verdict + CSV row

Prerequisites (start these in separate terminals BEFORE this launch):
    ./simulation/px4_sitl_3uav/start_3uav_sitl.sh   # or your SITL bringup
    # confirm real poses are flowing:
    ros2 topic echo /drone_pose_external --once

Run one scenario:
    ros2 launch containment_pkg escape_eval_sitl.launch.py \
        scene:=park direction:=2 trajectory:=return \
        result_csv:=./eval_results_sitl.csv

For the 8.27 SITL batch, drive this launch from ``run_batch_sitl.sh`` instead
of running it interactively.
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
    intercept_radius = LaunchConfiguration("intercept_radius")
    closed_loop = LaunchConfiguration("closed_loop")

    # Platform state merger inputs/outputs.
    uav_topic = LaunchConfiguration("uav_topic")
    ugv_topic = LaunchConfiguration("ugv_topic")
    output_topic = LaunchConfiguration("output_topic")
    publish_period = LaunchConfiguration("publish_period")

    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="park"),
        DeclareLaunchArgument("direction", default_value="2"),
        DeclareLaunchArgument("trajectory", default_value="sample",
                              description="Target trajectory for this run. A "
                                          "concrete mode (return|oscillate|"
                                          "straight) is FORCED; the sentinel "
                                          "'sample' draws from the scene's "
                                          "trajectory_distribution (used by "
                                          "the SITL batch)."),
        DeclareLaunchArgument("speed", default_value="2.0"),
        DeclareLaunchArgument("monitor_radius", default_value="25.0"),
        DeclareLaunchArgument("block_radius", default_value="15.0"),
        DeclareLaunchArgument("intercept_radius", default_value="5.0"),
        DeclareLaunchArgument("test_duration", default_value="20.0"),
        DeclareLaunchArgument("result_csv", default_value="./eval_results_sitl.csv"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("closed_loop", default_value="false",
                              description="escape_test_node real closed-loop "
                                          "mode: target only reverses after a "
                                          "platform intercepts it within "
                                          "intercept_radius."),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("containment_pkg"),
                 "config", "three_scene_config.yaml"]
            ),
            description="Path to three_scene_config.yaml; per-scene params "
                        "override launch args when the scene is found.",
        ),

        # ---- SITL platform source: merge real UAV + UGV states ---------------
        DeclareLaunchArgument("uav_topic", default_value="/drone_pose_external",
                              description="SITL UAV state topic (DroneStateArray)."),
        DeclareLaunchArgument("ugv_topic", default_value="/ground_vehicle_states",
                              description="SITL UGV state topic (DroneStateArray)."),
        DeclareLaunchArgument("output_topic", default_value="/drone_states",
                              description="Merged platform topic consumed by "
                                          "enclosure_node and containment_evaluator."),
        DeclareLaunchArgument("publish_period", default_value="0.25",
                              description="Republish period for the merged "
                                          "/drone_states stream."),

        Node(
            package="containment_pkg",
            executable="platform_state_merger",
            name="platform_state_merger",
            output="screen",
            parameters=[{
                "uav_topic": uav_topic,
                "ugv_topic": ugv_topic,
                "output_topic": output_topic,
                "publish_period": publish_period,
            }],
        ),

        # Scripted escape target (the "intruder" is still scripted; only the
        # defending platforms are real from SITL).
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
                "intercept_radius": intercept_radius,
                "closed_loop": closed_loop,
            }],
        ),

        # Three-layer enclosure (consumes the scripted /enclosure_targets).
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
                "intercept_radius": intercept_radius,
                "test_duration": test_duration,
                "result_csv": result_csv,
                "config_file": config_file,
            }],
        ),
    ])
