"""Launch PX4 SITL + MAVROS + planning_pkg bridges.

This launch file brings up the simulation half of the P3 SITL pipeline:

* ``px4_sitl`` instance(s)               — runs PX4 firmware against Gazebo
                                          (``PX4_SITL=gz_iris``), one instance
                                          per UAV (``num_uav`` argument).
* ``mavros`` node per UAV                — bridges the PX4 UDP MAVLink stream
                                          to native ROS2 topics
                                          (``/mavros/state``, ``/mavros/setpoint_raw/local``,
                                          ``/mavros/mocap/pose``).
* ``px4_offboard_bridge``                — forwards ``/planned_path`` to the
                                          MAVROS setpoint topic
                                          (from ``planning_pkg``).
* ``sitl_pose_bridge`` per UAV           — republishes the SITL pose as
                                          ``DroneStateArray`` on
                                          ``/drone_pose_external`` for the
                                          planner to consume.

Environment / arguments:
    ``PX4_SITL_ROOT``    must point to a built PX4 firmware tree that
                         contains ``build/px4_sitl_default/...`` and the
                         ``Tools/simulation/gazebo-classic`` plugin set.
                         The build is reused as-is — we do not rebuild
                         firmware here.

Typical invocations::

    # Single drone (default)
    PX4_SITL_ROOT=$HOME/src/PX4-Autopilot \
        ros2 launch planning_pkg px4_sitl.launch.py

    # 3 drones, headless
    PX4_SITL_ROOT=$HOME/src/PX4-Autopilot GAZEBO_HEADLESS=true \
        ros2 launch planning_pkg px4_sitl.launch.py num_uav:=3

    # CI: timeout after 60s
    PX4_SITL_ROOT=$HOME/src/PX4-Autopilot SITL_TIMEOUT=60 \
        ros2 launch planning_pkg sitl_test.launch.py
"""
from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import time
from typing import List

from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)


def _resolve_px4_sitl(context, *args, **kwargs):  # noqa: D401
    """OpaqueFunction that builds per-UAV ExecuteProcess actions.

    We do this in Python instead of static XML so we can:
      * Spawn N px4 instances with distinct ``instance`` ports + UDP ports.
      * Start one MAVROS per instance, pointing at ``udp://:<port>@127.0.0.1:<port+1>``.
      * Start ``px4_offboard_bridge`` once (path is swarm-wide).
      * Start one ``sitl_pose_bridge`` per UAV with the right ``drone_id``.
    """
    actions = []

    px4_root = os.environ.get("PX4_SITL_ROOT", "").strip()
    if not px4_root:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "echo '[px4_sitl.launch] PX4_SITL_ROOT is not set; aborting.' >&2; exit 1",
                ],
                name="px4_sitl_precheck",
                on_exit=Shutdown(),
            )
        )
        return actions

    px4_sitl_bin = os.path.join(px4_root, "build", "px4_sitl_default", "px4_sitl_default")
    rc_mavlink = os.path.join(px4_root, "etc", "px4-rc.mavlink")
    models_dir = os.path.join(px4_root, "Tools", "simulation", "gazebo-classic", "sitl_gazebo-classic")

    if not os.path.exists(px4_sitl_bin):
        actions.append(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    (
                        f"echo '[px4_sitl.launch] PX4 SITL binary not found at {px4_sitl_bin}; "
                        "build firmware first (make px4_sitl_default).' >&2; exit 1"
                    ),
                ],
                name="px4_sitl_precheck",
                on_exit=Shutdown(),
            )
        )
        return actions

    num_uav = int(LaunchConfiguration("num_uav").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes", "on")
    use_udp = LaunchConfiguration("fcu_url").perform(context)  # template; we'll patch per instance
    gcs_url = LaunchConfiguration("gcs_url").perform(context)
    px4_sitl_root = px4_root

    # Make Gazebo / PX4 plugins visible.
    env = os.environ.copy()
    env.setdefault("PX4_SITL_ROOT", px4_sitl_root)
    env.setdefault("GAZEBO_PLUGIN_PATH", models_dir)
    env.setdefault("GAZEBO_MODEL_PATH", models_dir)
    if headless:
        env["HEADLESS"] = "1"
        env["DISPLAY"] = ""

    actions.append(
        ExecuteProcess(
            cmd=["bash", "-lc", f"echo '[px4_sitl.launch] PX4_SITL_ROOT={px4_sitl_root} num_uav={num_uav} headless={headless}'"],
            name="px4_sitl_echo",
            output="screen",
        )
    )

    for i in range(num_uav):
        instance_id = i  # PX4 expects 0..N-1
        udp_port = 14540 + 10 * i  # PX4 default mavlink start port for SITL
        # MAVROS default fcu_url is udp://:14540@127.0.0.1:14557.
        fcu_url = use_udp.replace("14540", str(udp_port)).replace("14557", str(udp_port + 17))

        px4_cmd = [
            px4_sitl_bin,
            f"-i{instance_id}",
            rc_mavlink,
        ]
        actions.append(
            ExecuteProcess(
                cmd=px4_cmd,
                name=f"px4_{instance_id}",
                output="screen",
                env=env,
                shell=False,
            )
        )

        # Give PX4 a moment to open its UDP port before MAVROS tries to connect.
        mavros_cmd = [
            "ros2",
            "run",
            "mavros",
            "mavros_node",
            "--ros-args",
            "-p",
            f"fcu_url:={fcu_url}",
            "-p",
            f"gcs_url:={gcs_url}",
            "-p",
            f"namespace:=/uav{instance_id}",
            "-p",
            "pluginlists_yaml:=mavros_plugins.yaml",
            "-p",
            "plugin_allowlist:[]",
            # Disable actuators; we only need state + setpoint + mocap.
            "-p",
            "system:=false",
            "-p",
            "setpoint_position:=-",
            "-p",
            f"mocap/use_pose:={LaunchConfiguration('mocap_use_pose').perform(context)}",
            "-p",
            "mocap/use_vision:=",
        ]
        actions.append(
            TimerAction(
                period=3.0 + i * 1.0,
                actions=[
                    ExecuteProcess(
                        cmd=mavros_cmd,
                        name=f"mavros_{instance_id}",
                        output="screen",
                        env=env,
                    )
                ],
            )
        )

        # sitl_pose_bridge: republish /uavN/mavros/mocap/pose -> /drone_pose_external
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

    # One shared px4_offboard_bridge for the swarm.  The planner publishes a
    # single /planned_path; we route the first UAV's setpoint topic to it.
    # Multi-drone extensions should split paths by drone_id — out of scope here.
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

    return actions


def generate_launch_description() -> LaunchDescription:
    """Build the LaunchDescription for PX4 SITL + MAVROS + bridges."""
    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "num_uav",
            default_value="1",
            description="Number of PX4 SITL instances (UAVs) to spawn (1..3).",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value=EnvironmentVariable("GAZEBO_HEADLESS", default_value="false"),
            description="Run Gazebo headless (no GUI).  Set GAZEBO_HEADLESS=true in CI.",
        ),
        DeclareLaunchArgument(
            "fcu_url",
            default_value="udp://:14540@127.0.0.1:14557",
            description="MAVROS fcu_url template (port is rewritten per UAV).",
        ),
        DeclareLaunchArgument(
            "gcs_url",
            default_value="udp://@127.0.0.1:14550",
            description="MAVROS gcs_url (forward telemetry to a GCS, optional).",
        ),
        DeclareLaunchArgument(
            "mocap_use_pose",
            default_value="true",
            description="Enable MAVROS mocap plugin for external pose.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Optional outer namespace prefix.",
        ),
    ]

    # Use OpaqueFunction to inject per-instance ExecuteProcess at expansion time.
    px4_sitl_group = OpaqueFunction(function=_resolve_px4_sitl)

    return LaunchDescription(args + [px4_sitl_group])
