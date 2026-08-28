"""Headless 重规划场景：逃逸目标 + mock 平台 + planner（8.27 封控测试配套）。

把 planner 接入封控场景，使"目标逃逸 + 绕障重规划"成为场景内真日志：

    mock_platform_pub  -> /drone_states                    (3 UAV + 2 UGV)
    escape_test_node   -> /enclosure_targets               (脚本化移动目标)
    planner_node       -> 订阅 /enclosure_targets，重规划并打 [REPLAN] 日志

planner 会从 ``three_scene_config.yaml`` 读取当前 scene 的 ``obstacles``
（security 场景的建筑障碍物）播种到栅格，重规划路径会绕障。

运行一个场景（采集见 planning_pkg/scripts/run_with_logging.sh）::

    ros2 launch planning_pkg replan_eval.launch.py scene:=security \\
        config_file:=<...>/containment_pkg/config/three_scene_config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scene = LaunchConfiguration("scene")
    config_file = LaunchConfiguration("config_file")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    platform_type = LaunchConfiguration("platform_type")
    speed = LaunchConfiguration("speed")
    test_duration = LaunchConfiguration("test_duration")

    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="security"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument(
            "platform_type", default_value="0",
            description="0=UAV(无人机) 1=UGV(无人车)",
        ),
        DeclareLaunchArgument("speed", default_value="3.0"),
        DeclareLaunchArgument("test_duration", default_value="30.0"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("containment_pkg"),
                 "config", "three_scene_config.yaml"]
            ),
            description="Path to three_scene_config.yaml; seeds obstacles / "
                        "per-scene params (start, speed, trajectory).",
        ),

        # 平台状态（3 UAV + 2 UGV），供 planner 读取初始位置。
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
            # planner consumes external poses while it remains the sole
            # publisher of /drone_states for containment.
            remappings=[("/drone_states", "/drone_pose_external")],
        ),

        # 脚本化逃逸目标：位置随场景轨迹变化，驱动 planner 重规划。
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
                "escape_direction": -1,   # 按 scene 的方向分布加权随机
                "speed": speed,
                "test_duration": test_duration,
                "config_file": config_file,
            }],
        ),

        # planner：订阅 /enclosure_targets，目标移动即重规划（[REPLAN] 日志）。
        Node(
            package="planning_pkg",
            executable="planner_node",
            name="planner_node",
            output="screen",
            parameters=[{
                "scene_name": scene,
                "scene_config_file": config_file,
                "enclosure_targets_topic": "/enclosure_targets",
                "planner": "dstar_lite",
                "num_drones": 3,
                "platform_type": platform_type,
                "tick_period": 0.5,
                "log_interval_sec": 5.0,
            }],
        ),
    ])
