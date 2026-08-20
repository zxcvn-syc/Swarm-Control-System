"""Headless one-UAV SITL smoke test with an FCU connection watchdog."""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def _actions(context, *args, **kwargs):
    root_value = os.environ.get("PX4_SITL_ROOT", "").strip()
    root = Path(root_value)
    num_uav = int(LaunchConfiguration("num_uav").perform(context))
    timeout = int(LaunchConfiguration("timeout").perform(context))
    if num_uav != 1:
        message = "[sitl_test] this smoke test supports num_uav:=1"
        return [ExecuteProcess(cmd=["bash", "-lc", f"echo '{message}' >&2; exit 2"], on_exit=Shutdown())]

    build_path = root / "build" / "px4_sitl_default"
    sitl_bin = build_path / "bin" / "px4"
    if not sitl_bin.is_file():
        sitl_bin = build_path / "px4_sitl_default"
    sitl_run = root / "Tools" / "simulation" / "gazebo-classic" / "sitl_run.sh"
    if not root_value or not sitl_bin.is_file() or not sitl_run.is_file():
        message = f"[sitl_test] PX4_SITL_ROOT or SITL build missing: {root}"
        return [ExecuteProcess(cmd=["bash", "-lc", f"echo '{message}' >&2; exit 2"], on_exit=Shutdown())]

    env = os.environ.copy()
    plugin_path = str(root / "Tools" / "simulation" / "gazebo-classic" / "sitl_gazebo-classic")
    env["PX4_SITL_ROOT"] = str(root)
    env["GAZEBO_HEADLESS"] = "true"
    env["HEADLESS"] = "1"
    env["DISPLAY"] = ""
    env["GAZEBO_PLUGIN_PATH"] = os.pathsep.join(filter(None, [plugin_path, env.get("GAZEBO_PLUGIN_PATH", "")]))
    env["GAZEBO_MODEL_PATH"] = os.pathsep.join(filter(None, [plugin_path, env.get("GAZEBO_MODEL_PATH", "")]))
    env["LD_LIBRARY_PATH"] = os.pathsep.join(filter(None, [plugin_path + "/build", env.get("LD_LIBRARY_PATH", "")]))

    actions = [
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
                    ],
                    name="sitl_pose_bridge_uav0",
                    output="screen",
                    env=env,
                ),
                ExecuteProcess(
                    cmd=[
                        "ros2", "run", "planning_pkg", "px4_offboard_bridge", "--ros-args",
                        "-p", "path_topic:=/planned_path",
                        "-p", "setpoint_topic:=/uav0/mavros/setpoint_raw/local",
                    ],
                    name="px4_offboard_bridge_uav0",
                    output="screen",
                    env=env,
                ),
            ],
        ),
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash", "-lc",
                        "timeout " + str(timeout) + "s bash -lc "
                        "'until ros2 topic echo --no-daemon "
                        "/uav0/mavros/local_position/pose --once "
                        "--qos-reliability best_effort >/dev/null 2>&1; "
                        "do sleep 1; done'",
                    ],
                    name="sitl_pose_watchdog",
                    output="screen",
                    on_exit=Shutdown(),
                )
            ],
        ),
    ]
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_uav", default_value="1"),
            DeclareLaunchArgument(
                "timeout",
                default_value=EnvironmentVariable("SITL_TIMEOUT", default_value="120"),
            ),
            OpaqueFunction(function=_actions),
        ]
    )
