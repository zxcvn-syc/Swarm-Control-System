"""Launch one PX4 Gazebo Classic SITL instance with ROS2 bridges."""

from __future__ import annotations

import os
from pathlib import Path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _world_path() -> str:
    configured = os.environ.get("PX4_SITL_WORLD", "").strip()
    if configured:
        return configured
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("planning_pkg")) / "worlds" / "swarm_field.world"
        if installed.is_file():
            return str(installed)
    except Exception:
        pass
    source = Path(__file__).resolve().parents[4] / "simulation" / "worlds" / "swarm_field.world"
    return str(source) if source.is_file() else "none"


def _resolve_actions(context, *args, **kwargs):
    px4_root = os.environ.get("PX4_SITL_ROOT", "").strip()
    num_uav = int(LaunchConfiguration("num_uav").perform(context))
    safety_enabled = LaunchConfiguration("enable_flight_safety").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    auto_arm = LaunchConfiguration("auto_arm").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    headless = LaunchConfiguration("headless").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    if num_uav != 1:
        return [
            ExecuteProcess(
                cmd=["bash", "-lc", "echo '[px4_sitl] this reproducible profile supports num_uav:=1' >&2; exit 2"],
                name="px4_sitl_precheck",
                on_exit=Shutdown(),
            )
        ]
    if not px4_root:
        return [
            ExecuteProcess(
                cmd=["bash", "-lc", "echo '[px4_sitl] PX4_SITL_ROOT is not set' >&2; exit 2"],
                name="px4_sitl_precheck",
                on_exit=Shutdown(),
            )
        ]

    root = Path(px4_root)
    build_path = root / "build" / "px4_sitl_default"
    sitl_bin = build_path / "bin" / "px4"
    if not sitl_bin.is_file():
        sitl_bin = build_path / "px4_sitl_default"
    sitl_run = root / "Tools" / "simulation" / "gazebo-classic" / "sitl_run.sh"
    if not sitl_bin.is_file() or not sitl_run.is_file():
        message = f"[px4_sitl] missing built PX4 or sitl_run.sh under {root}"
        return [
            ExecuteProcess(
                cmd=["bash", "-lc", f"echo '{message}' >&2; exit 2"],
                name="px4_sitl_precheck",
                on_exit=Shutdown(),
            )
        ]

    env = os.environ.copy()
    env["PX4_SITL_ROOT"] = str(root)
    env["PX4_SITL_WORLD"] = _world_path()
    plugin_path = str(root / "Tools" / "simulation" / "gazebo-classic" / "sitl_gazebo-classic")
    env["GAZEBO_PLUGIN_PATH"] = os.pathsep.join(filter(None, [plugin_path, env.get("GAZEBO_PLUGIN_PATH", "")]))
    env["GAZEBO_MODEL_PATH"] = os.pathsep.join(filter(None, [plugin_path, env.get("GAZEBO_MODEL_PATH", "")]))
    env["LD_LIBRARY_PATH"] = os.pathsep.join(filter(None, [plugin_path + "/build", env.get("LD_LIBRARY_PATH", "")]))
    if headless:
        env["HEADLESS"] = "1"
        env["GAZEBO_HEADLESS"] = "true"
        env["DISPLAY"] = ""

    return [
        ExecuteProcess(
            cmd=[str(sitl_run), str(sitl_bin), "none", "iris", "none", str(root), str(build_path)],
            name="px4_sitl_gazebo",
            output="screen",
            env=env,
            on_exit=Shutdown(),
        ),
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "run", "mavros", "mavros_node", "--ros-args",
                        "-r", "__ns:=/uav0/mavros",
                        "-p", "fcu_url:=udp://:14540@127.0.0.1:14557",
                        "-p", "mocap/use_pose:=true",
                    ],
                    name="mavros_uav0",
                    output="screen",
                    env=env,
                )
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "run", "planning_pkg", "sitl_pose_bridge", "--ros-args",
                        "-p", "pose_topic:=/uav0/mavros/local_position/pose",
                        "-p", "state_topic:=/drone_pose_external",
                        "-p", "drone_id:=0",
                        "-p", "platform_type:=0",
                    ],
                    name="sitl_pose_bridge_uav0",
                    output="screen",
                    env=env,
                )
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="planning_pkg",
                    executable="px4_offboard_bridge",
                    name="px4_offboard_bridge_uav0",
                    output="screen",
                    env=env,
                    parameters=[
                        {
                            "path_topic": "/planned_path",
                            "setpoint_topic": "/uav0/mavros/setpoint_raw/local",
                            "state_topic": "/uav0/mavros/state",
                            "local_pose_topic": "/uav0/mavros/local_position/pose",
                            "arm_service": "/uav0/mavros/cmd/arming",
                            "mode_service": "/uav0/mavros/set_mode",
                            "enable_setpoint_streaming": True,
                            "drone_id": 0,
                            "auto_arm": auto_arm,
                            "safety_hold_enabled": safety_enabled,
                            "initial_safety_hold": safety_enabled,
                        }
                    ],
                )
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument("num_uav", default_value="1"),
        DeclareLaunchArgument("enable_flight_safety", default_value="false"),
        # Keep the legacy SITL profile unchanged; supervised SITL explicitly
        # overrides this to false so containment activation never arms PX4.
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument(
            "headless",
            default_value=EnvironmentVariable("GAZEBO_HEADLESS", default_value="false"),
        ),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_resolve_actions)])
