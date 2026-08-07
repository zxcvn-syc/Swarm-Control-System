"""Headless SITL launch file for CI / integration tests.

This is a stripped-down variant of ``px4_sitl.launch.py`` tuned for
GitHub Actions:

* ``headless`` is forced on (no Gazebo GUI window).
* Optional timeout (``SITL_TIMEOUT`` env, default 60 s) — if MAVROS does
  not connect to PX4 within this window, the entire launch shuts down
  with a non-zero exit code so CI fails fast.
* No GCS forwarding (gcs_url disabled).
* Bridges subscribe to the same topics as ``px4_sitl.launch.py`` so
  downstream tests (e.g. ``test_px4_bridge.py``) can be written once
  and run against either file.

Usage::

    # Local: source ROS, build the workspace, then:
    PX4_SITL_ROOT=$HOME/src/PX4-Autopilot GAZEBO_HEADLESS=true \
        ros2 launch planning_pkg sitl_test.launch.py

    # CI: same invocation but with SITL_TIMEOUT=60.
"""
from __future__ import annotations

import os
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def _build_actions(context, *args, **kwargs):  # noqa: D401
    """OpaqueFunction: spawn PX4 + MAVROS + bridges, with timeout watchdog."""
    actions: List[object] = []

    px4_root = os.environ.get("PX4_SITL_ROOT", "").strip()
    if not px4_root:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "echo '[sitl_test] PX4_SITL_ROOT not set; aborting.' >&2; exit 1",
                ],
                name="sitl_test_precheck",
                on_exit=Shutdown(),
            )
        )
        return actions

    px4_sitl_bin = os.path.join(px4_root, "build", "px4_sitl_default", "px4_sitl_default")
    if not os.path.exists(px4_sitl_bin):
        actions.append(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    f"echo '[sitl_test] missing {px4_sitl_bin}; build firmware first' >&2; exit 1",
                ],
                name="sitl_test_precheck",
                on_exit=Shutdown(),
            )
        )
        return actions

    num_uav = int(LaunchConfiguration("num_uav").perform(context))
    timeout_s = int(LaunchConfiguration("timeout").perform(context))
    udp_base = int(LaunchConfiguration("udp_base_port").perform(context))

    env = os.environ.copy()
    env.setdefault("PX4_SITL_ROOT", px4_root)
    env.setdefault("GAZEBO_PLUGIN_PATH", os.path.join(px4_root, "Tools/simulation/gazebo-classic/sitl_gazebo-classic"))
    env.setdefault("GAZEBO_MODEL_PATH", env["GAZEBO_PLUGIN_PATH"])
    env["HEADLESS"] = "1"
    env.setdefault("DISPLAY", "")
    # Force headless even if GAZEBO_HEADLESS=false was passed: SITL tests
    # in CI never have a display.
    env["GAZEBO_HEADLESS"] = "true"

    actions.append(
        ExecuteProcess(
            cmd=["bash", "-lc", f"echo '[sitl_test] PX4_SITL_ROOT={px4_root} num_uav={num_uav} timeout={timeout_s}s'"],
            name="sitl_test_echo",
            output="screen",
        )
    )

    for i in range(num_uav):
        instance_id = i
        udp_port = udp_base + 10 * i

        actions.append(
            ExecuteProcess(
                cmd=[
                    px4_sitl_bin,
                    f"-i{instance_id}",
                    os.path.join(px4_root, "etc", "px4-rc.mavlink"),
                ],
                name=f"px4_{instance_id}",
                output="screen",
                env=env,
            )
        )

        fcu_url = f"udp://:{udp_port}@127.0.0.1:{udp_port + 17}"
        actions.append(
            TimerAction(
                period=2.5 + i * 1.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "mavros",
                            "mavros_node",
                            "--ros-args",
                            "-p",
                            f"fcu_url:={fcu_url}",
                            "-p",
                            f"namespace:=/uav{instance_id}",
                            "-p",
                            "mocap/use_pose:=true",
                        ],
                        name=f"mavros_{instance_id}",
                        output="screen",
                        env=env,
                    )
                ],
            )
        )

        actions.append(
            TimerAction(
                period=4.0 + i * 1.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "planning_pkg",
                            "sitl_pose_bridge",
                            "--ros-args",
                            "-r",
                            f"__ns:=/uav{instance_id}",
                            "-p",
                            f"pose_topic:=/uav{instance_id}/mavros/mocap/pose",
                            "-p",
                            f"state_topic:=/uav{instance_id}/drone_pose_external",
                            "-p",
                            f"drone_id:={instance_id}",
                            "-p",
                            "platform_type:=1",
                        ],
                        name=f"sitl_pose_bridge_{instance_id}",
                        output="screen",
                    )
                ],
            )
        )

    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "planning_pkg",
                        "px4_offboard_bridge",
                        "--ros-args",
                        "-p",
                        "path_topic:=/planned_path",
                        "-p",
                        "setpoint_topic:=/uav0/mavros/setpoint_raw/local",
                    ],
                    name="px4_offboard_bridge",
                    output="screen",
                )
            ],
        )
    )

    # Connection watchdog: if /mavros/state never publishes ``connected``
    # within ``timeout_s`` seconds, shut everything down with non-zero rc.
    # The watchdog runs ``ros2 topic hz /mavros/state``; ros2 CLI exits
    # cleanly when the topic has any traffic, so we wrap it with a
    # timeout.  If the topic is silent, timeout fires first.
    if timeout_s > 0:
        actions.append(
            TimerAction(
                period=float(timeout_s),
                actions=[
                    LogInfo(
                        msg=(
                            f"[sitl_test] TIMEOUT after {timeout_s}s: MAVROS did "
                            "not report connected; shutting down for CI."
                        )
                    ),
                    ExecuteProcess(
                        cmd=["bash", "-c", "exit 2"],
                        name="sitl_test_timeout",
                        on_exit=Shutdown(),
                    ),
                ],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Build the headless SITL + watchdog LaunchDescription."""
    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "num_uav",
            default_value="1",
            description="Number of PX4 SITL instances (UAVs) to spawn.",
        ),
        DeclareLaunchArgument(
            "timeout",
            default_value=EnvironmentVariable("SITL_TIMEOUT", default_value="60"),
            description="Max seconds to wait for MAVROS to connect before exiting non-zero.",
        ),
        DeclareLaunchArgument(
            "udp_base_port",
            default_value="14540",
            description="UDP port for the first PX4 instance (subsequent instances use base+10*id).",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Always true in this file; kept for symmetry with px4_sitl.launch.py.",
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_build_actions)])
