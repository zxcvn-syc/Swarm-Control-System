"""Launch file for ``planner_node``.

This file mirrors the style of
``perception_pkg/launch/tracker_node.launch.py`` (LaunchDescription +
DeclareLaunchArgument + Node with a YAML config overlay).

Typical invocations::

    ros2 launch planning_pkg planning.launch.py                       # default YAML
    ros2 launch planning_pkg planning.launch.py \\
        planner:=dstar_lite grid_size:=120 num_drones:=12
    ros2 launch planning_pkg planning.launch.py config:=/abs/override.yaml

    # With PX4 SITL (Gazebo + PX4 + MAVROS + bridges) wired in:
    PX4_SITL_ROOT=$HOME/src/PX4-Autopilot \\
        ros2 launch planning_pkg planning.launch.py include_sitl:=true

    # P1-B: load the simulation grid (map / start / goal / planner)
    # from a YAML file rather than relying only on inline overrides.
    ros2 launch planning_pkg planning.launch.py \\
        grid_config:=/abs/path/to/grid_config.yaml
"""

from __future__ import annotations

import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Optional path to a planning.yaml override file.",
        ),
        DeclareLaunchArgument(
            "grid_config",
            default_value=_default_grid_config_path(),
            description=(
                "Path to the simulation grid YAML (map.width/height/resolution, "
                "start, goal, planner.connectivity/heuristic). Resolved relative "
                "to this launch file when AMENT_PREFIX_PATH has not been set."
            ),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Optional namespace prefix (kept empty by default to "
            "match scheduler/enclosure usage).",
        ),
        DeclareLaunchArgument(
            "planner",
            default_value="astar",
            choices=["astar", "dstar_lite"],
            description="Path-planner algorithm.",
        ),
        DeclareLaunchArgument(
            "num_drones",
            default_value="8",
            description="Number of drones for the demo (also seeds the grid).",
        ),
        DeclareLaunchArgument(
            "grid_size",
            default_value="100",
            description="Square grid side length in cells.",
        ),
        DeclareLaunchArgument(
            "tick_period",
            default_value="0.5",
            description="Seconds between planner ticks.",
        ),
        DeclareLaunchArgument(
            "include_sitl",
            default_value="false",
            choices=["true", "false"],
            description="If true, also launch px4_sitl.launch.py (Gazebo + PX4 + MAVROS + bridges).",
        ),
        DeclareLaunchArgument(
            "sitl_num_uav",
            default_value=LaunchConfiguration("num_drones"),
            description="Number of PX4 SITL instances to launch when include_sitl is true (capped 1..3).",
        ),
        DeclareLaunchArgument(
            "sitl_headless",
            default_value="false",
            description="Run SITL Gazebo headless (used when include_sitl is true).",
        ),
    ]

    inline_overrides = {
        "planner": LaunchConfiguration("planner"),
        "num_drones": LaunchConfiguration("num_drones"),
        "grid_size": LaunchConfiguration("grid_size"),
        "tick_period": LaunchConfiguration("tick_period"),
    }

    node = Node(
        package="planning_pkg",
        executable="planner_node",
        name="planner_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            inline_overrides,
            # Optional YAML override (legacy, higher priority than grid_config).
            LaunchConfiguration("config"),
            # P1-B: load the simulation grid YAML (map / start / goal /
            # planner).  When ``grid_config`` is the empty string ROS2
            # treats it as "no YAML" and the inline overrides remain
            # the sole source of parameters.
            LaunchConfiguration("grid_config"),
        ],
    )

    ugv_node = Node(
        package="planning_pkg",
        executable="ugv_state_pub",
        name="ugv_state_publisher",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"num_ugv": 2}],
    )

    # PX4 SITL bridge nodes are launched here unconditionally so the
    # ``/planned_path`` -> ``/mavros/setpoint_raw/local`` plumbing and
    # the ``/mavros/mocap/pose`` -> ``/drone_pose_external`` path are
    # always wired up.  When ``include_sitl:=false`` we still start the
    # bridges so they can be tested with a fake MAVROS / mocap feed;
    # they no-op when no messages arrive (see ``px4_offboard_bridge``).
    px4_offboard_bridge_node = Node(
        package="planning_pkg",
        executable="px4_offboard_bridge",
        name="px4_offboard_bridge",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{}],
    )

    sitl_pose_bridge_node = Node(
        package="planning_pkg",
        executable="sitl_pose_bridge",
        name="sitl_pose_bridge",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"platform_type": 1}],
    )

    actions: List[object] = args + [
        node,
        ugv_node,
        px4_offboard_bridge_node,
        sitl_pose_bridge_node,
    ]

    # Conditionally include the SITL launch (Gazebo + PX4 + MAVROS).
    # We do this via IncludeLaunchDescription so the SITL group has its
    # own PX4 process tree and lifecycle that can be torn down without
    # killing the planner.
    if _sitl_launch_available():
        sitl_include = IncludeLaunchDescription(
            # Use the package share path so the include works regardless
            # of the caller's working directory.  We resolve at runtime
            # via a shell command because we cannot import the package
            # here without coupling the launch to colcon_build order.
            launch_description_source=_get_sitl_launch_source(),
            launch_arguments=[
                ("num_uav", LaunchConfiguration("sitl_num_uav")),
                ("headless", LaunchConfiguration("sitl_headless")),
                ("namespace", LaunchConfiguration("namespace")),
            ],
            condition=IfCondition(LaunchConfiguration("include_sitl")),
        )
        actions.append(sitl_include)

    return LaunchDescription(actions)


def _default_grid_config_path() -> str:
    """Resolve the default ``grid_config`` path (P1-B).

    Layout assumption (matches the current repo)::

        <repo_root>/
            simulation/maps/grid_config.yaml
            ros2_ws/src/planning_pkg/launch/planning.launch.py

    We prefer ``AMENT_PREFIX_PATH`` (post ``colcon build``) so the YAML
    can be copied into the package share, but fall back to a
    repo-relative path (via ``__file__``) so the launch still works
    on a fresh checkout.  Both branches use ``os.path.join`` so the
    lookup is portable across ``catkin_pkg`` / ``ament_index`` /
    ``ros2 pkg`` discovery mechanisms.
    """
    # 1) Post-build: <prefix>/share/planning_pkg/grid_config.yaml
    prefix_path = os.environ.get("AMENT_PREFIX_PATH", "")
    for prefix in prefix_path.split(":"):
        candidate = os.path.join(
            prefix, "share", "planning_pkg", "grid_config.yaml"
        )
        if os.path.exists(candidate):
            return candidate

    # 2) Repo-relative fallback: simulation/maps/grid_config.yaml
    #    planning.launch.py lives at
    #    <repo>/ros2_ws/src/planning_pkg/launch/planning.launch.py
    #    so 4 ".." levels up gets us to the repo root.
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(
        os.path.join(here, "..", "..", "..", "..")
    )
    return os.path.join(
        repo_root, "simulation", "maps", "grid_config.yaml"
    )


def _sitl_launch_available() -> bool:
    """Check whether ``px4_sitl.launch.py`` is in the package share.

    The check is purely a hint for the no-SITL fallback.  When the
    package hasn't been built yet (e.g. running ``colcon build`` on a
    fresh checkout) we silently skip the include; once ``colcon build
    --packages-select planning_pkg`` runs, the share directory is
    populated and the include becomes active.
    """
    # AMENT_PREFIX_PATH is set by setup.bash after a successful build.
    prefix_path = os.environ.get("AMENT_PREFIX_PATH", "")
    for prefix in prefix_path.split(":"):
        candidate = os.path.join(prefix, "share", "planning_pkg", "launch", "px4_sitl.launch.py")
        if os.path.exists(candidate):
            return True
    return False


def _get_sitl_launch_source():
    """Return the package-relative launch description source for SITL."""
    # Prefer a package-relative path (works after `colcon build`); fall
    # back to a direct file path for `ros2 launch <file>` users.
    from launch.launch_description_sources import (
        PythonLaunchDescriptionSource,
    )

    prefix_path = os.environ.get("AMENT_PREFIX_PATH", "")
    for prefix in prefix_path.split(":"):
        candidate = os.path.join(prefix, "share", "planning_pkg", "launch", "px4_sitl.launch.py")
        if os.path.exists(candidate):
            return PythonLaunchDescriptionSource(candidate)

    # Fallback: assume the repo layout (works for `ros2 launch` from
    # the workspace root even before colcon install).
    return PythonLaunchDescriptionSource(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "px4_sitl.launch.py",
        )
    )
