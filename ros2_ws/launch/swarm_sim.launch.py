import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 自动拉起：3台无人机 (UAV: ID 0, 1, 2，飞行高度 Z=2.0 米)
    uav_planner = Node(
        package='planning_pkg',
        executable='planner_node',
        name='uav_planner_node',
        output='screen',
        parameters=[{
            'num_drones': 3,
            'drone_id_offset': 0,        # 分配 ID: 0, 1, 2
            'platform_type': 0,          # 平台: 无人机
            'drone_z_default': 2.0,      # 高度: 2.0 米
            'sim_tick_speed': 1.0,
            'initial_positions': [10.0, 10.0, 20.0, 10.0, 30.0, 10.0],
        }]
    )

    # 自动拉起：2台无人车 (UGV: ID 3, 4，地面行驶高度 Z=0.0 米)
    ugv_planner = Node(
        package='planning_pkg',
        executable='planner_node',
        name='ugv_planner_node',
        output='screen',
        parameters=[{
            'num_drones': 2,
            'drone_id_offset': 3,        # 分配 ID: 3, 4
            'platform_type': 1,          # 平台: 无人车
            'drone_z_default': 0.0,      # 高度: 0.0 米
            'sim_tick_speed': 1.0,
            'min_turning_radius': 2.0,
            'initial_positions': [10.0, 30.0, 20.0, 30.0],
        }]
    )

    return LaunchDescription([
        uav_planner,
        ugv_planner,
    ])
