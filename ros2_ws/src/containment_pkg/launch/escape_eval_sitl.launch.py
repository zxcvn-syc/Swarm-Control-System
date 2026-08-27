"""SITL single-scene enclosure test: REAL platform poses + scripted target.

Differs from ``escape_eval.launch.py`` only in the platform source:

* Headless mode starts ``mock_platform_pub`` which publishes a scripted
  3 UAV + 2 UGV swarm on ``/drone_states``.
* SITL mode does NOT start any platform node here.  The real platform swarm
  is published on ``/drone_states`` by an EXTERNAL SITL bring-up
  (``simulation/px4_sitl_3uav/start_sitl_platform.sh`` -> 3 mavros nodes +
  ``sitl_state_publisher.py`` -> ``/drone_states`` with 3 real UAV + 2 mock
  UGV).  This launch just consumes that topic.

The remaining three nodes are reused unchanged from the headless launch:
    * ``escape_test_node`` -> ``/enclosure_targets`` (scripted intruder)
    * ``enclosure_node``   -> ``/enclosure_command`` (Voronoi assignment)
    * ``containment_evaluator`` -> verdict + CSV row

Optional merger
---------------
If the SITL scene bridge instead publishes two separate streams
(``/drone_pose_external`` for UAVs and ``/ground_vehicle_states`` for UGVs,
e.g. the RflySim ``rfly_ros_scene.py`` path), pass ``platform_source:=merger``
to start ``platform_state_merger`` which re-publishes the union on
``/drone_states``.  The default ``platform_source:=external`` assumes
``/drone_states`` already exists and starts nothing.

Prerequisites (start these in separate terminals BEFORE this launch):
    ./simulation/px4_sitl_3uav/start_3uav_sitl.sh          # PX4 x3 + Gazebo
    bash simulation/px4_sitl_3uav/start_sitl_platform.sh    # mavros x3 + /drone_states
    # confirm real poses are flowing:
    ros2 topic echo /drone_states --once

Run one scenario:
    ros2 launch containment_pkg escape_eval_sitl.launch.py \
        scene:=park direction:=2 trajectory:=return \
        result_csv:=./eval_results_sitl.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
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

    # Platform source selection.
    platform_source = LaunchConfiguration("platform_source")
    # merger inputs/outputs (only used when platform_source:=merger)
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
            "platform_source", default_value="external",
            description="Where /drone_states comes from. 'external' (default) "
                        "assumes the SITL bring-up already publishes "
                        "/drone_states (sitl_state_publisher / planner_node / "
                        "rfly merger) and starts NO platform node here. "
                        "'merger' starts platform_state_merger to fuse "
                        "/drone_pose_external + /ground_vehicle_states into "
                        "/drone_states (RflySim rfly_ros_scene.py path)."),
        DeclareLaunchArgument("uav_topic", default_value="/drone_pose_external",
                              description="UAV state topic for platform_source:=merger."),
        DeclareLaunchArgument("ugv_topic", default_value="/ground_vehicle_states",
                              description="UGV state topic for platform_source:=merger."),
        DeclareLaunchArgument("output_topic", default_value="/drone_states",
                              description="Merged platform topic for platform_source:=merger."),
        DeclareLaunchArgument("publish_period", default_value="0.25",
                              description="Republish period for the merged "
                                          "/drone_states stream (merger mode)."),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("containment_pkg"),
                 "config", "three_scene_config.yaml"]
            ),
            description="Path to three_scene_config.yaml; per-scene params "
                        "override launch args when the scene is found.",
        ),

        # ---- Optional platform merger (only for the RflySim scene path) ----
        Node(
            package="containment_pkg",
            executable="platform_state_merger",
            name="platform_state_merger",
            output="screen",
            condition=IfCondition(EqualsSubstitution(platform_source, "merger")),
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
        # NOTE: do NOT also start enclosure_node from the SITL bring-up, or the
        # two nodes with the same name will conflict. Use start_sitl_platform.sh
        # (no enclosure_node) as the platform bring-up.
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
