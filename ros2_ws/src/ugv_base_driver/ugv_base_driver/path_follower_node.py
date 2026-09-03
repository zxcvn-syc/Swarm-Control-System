"""path_follower_node — /planned_path (metres) -> /cmd_vel for the UGV.

This node closes the loop between the team's existing planner and the
``ugv_base_driver`` base driver:

    planner_node ──/planned_path──▶ path_follower_node ──/cmd_vel──▶ ugv_base_driver ──serial──▶ motors

Why this node exists
--------------------
``planner_node`` publishes a ``nav_msgs/Path`` whose poses carry the
frame id ``drone_<id>`` — all platforms' paths share one message.  The
base driver, in turn, only understands ``geometry_msgs/Twist`` speed
commands.  Something must (a) pick the UGV's own waypoints out of the
shared path, (b) compare them against the UGV's current pose and (c)
emit a grounded ``(v, omega)`` — that is exactly this node.

Inputs
------
* ``path_topic`` (``nav_msgs/Path``) — the shared planned path, metres,
  world frame.  Poses tagged ``drone_<id>``; set ``target_frame_id``
  (e.g. ``drone_4``) to keep only this vehicle's points.  An empty
  ``target_frame_id`` keeps every pose (single-vehicle experiments).
* ``pose_topic`` (``geometry_msgs/PoseStamped``) — the UGV pose in the
  **same world frame** as the path.  Today nobody on the real vehicle
  publishes this yet (open-loop driver, no /odom); feed it from any
  external source — a teleop pose publisher on the bench, a motion-
  capture relay, or a future wheel-odometry node.

Outputs
-------
* ``cmd_vel_topic`` (``geometry_msgs/Twist``) — differential-drive
  speed command consumed by ``ugv_base_driver``.

Safety behaviour
----------------
* No pose yet, or pose older than ``pose_timeout`` -> publish zero.
* No path yet, or path older than ``path_timeout`` -> publish zero.
* Goal reached within ``goal_tolerance`` -> publish zero, stay stopped.
* Node shutdown (Ctrl+C) -> one final zero command on teardown.

The base driver adds its own layers on top (``~/enable`` gate, serial
watchdog), so a wedged follower can never keep the wheels turning.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

from .pure_pursuit import (
    PursuitCommand,
    dedupe_waypoints,
    extract_waypoints,
    paths_equal,
    pure_pursuit_command,
    quaternion_to_yaw,
)

Point2 = Tuple[float, float]


def _as_bool(value: object, default: bool = False) -> bool:
    """Coerce ROS parameter values, including launch-substitution strings."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off", ""}:
            return False
    return default


class PathFollowerNode(Node):
    """Pure-pursuit follower bridging /planned_path to /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("ugv_path_follower")

        # ---- parameters ------------------------------------------------
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("pose_topic", "/ugv_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # Keep only poses tagged with this frame id (e.g. "drone_4").
        # Empty string -> keep all poses in the shared path message.
        self.declare_parameter("target_frame_id", "")
        self.declare_parameter("lookahead_distance", 0.6)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("slowdown_radius", 1.0)
        self.declare_parameter("rotate_in_place_error", 1.5707963)
        self.declare_parameter("pose_timeout", 0.5)
        self.declare_parameter("path_timeout", 2.0)
        self.declare_parameter("control_period", 0.05)
        self.declare_parameter("publish_twist_stamps", False)

        path_topic = str(self.get_parameter("path_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._target_frame_id = str(self.get_parameter("target_frame_id").value).strip()

        self._lookahead = float(self.get_parameter("lookahead_distance").value)
        self._max_v = float(self.get_parameter("max_linear_speed").value)
        self._max_w = float(self.get_parameter("max_angular_speed").value)
        self._goal_tol = float(self.get_parameter("goal_tolerance").value)
        self._slowdown = float(self.get_parameter("slowdown_radius").value)
        self._rotate_err = float(self.get_parameter("rotate_in_place_error").value)
        self._pose_timeout = max(0.0, float(self.get_parameter("pose_timeout").value))
        self._path_timeout = max(0.0, float(self.get_parameter("path_timeout").value))
        control_period = max(0.01, float(self.get_parameter("control_period").value))
        self._stamp_twist = _as_bool(
            self.get_parameter("publish_twist_stamps").value, False,
        )

        for name, value in (
            ("lookahead_distance", self._lookahead),
            ("max_linear_speed", self._max_v),
            ("max_angular_speed", self._max_w),
            ("goal_tolerance", self._goal_tol),
            ("slowdown_radius", self._slowdown),
        ):
            if not math.isfinite(value):
                raise ValueError(f"parameter {name} must be finite")
        if self._lookahead <= 0.0:
            raise ValueError("lookahead_distance must be positive")

        # ---- state ------------------------------------------------------
        self._waypoints: List[Point2] = []
        self._last_index: int = 0
        self._done: bool = False
        self._path_stamp_ns: Optional[int] = None
        self._pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw
        self._pose_stamp_ns: Optional[int] = None

        # ---- ROS wiring ---------------------------------------------------
        path_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._path_sub = self.create_subscription(
            Path, path_topic, self._on_path, path_qos,
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, pose_qos,
        )
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, cmd_qos)

        self._timer = self.create_timer(control_period, self._control_tick)

        self.get_logger().info(
            f"ugv_path_follower ready: path={path_topic} pose={pose_topic} "
            f"cmd_vel={cmd_vel_topic} target_frame_id={self._target_frame_id or '<all>'} "
            f"lookahead={self._lookahead:.2f}m v_max={self._max_v:.2f}m/s "
            f"w_max={self._max_w:.2f}rad/s goal_tol={self._goal_tol:.2f}m"
        )

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_path(self, msg: Path) -> None:
        new_waypoints = dedupe_waypoints(
            extract_waypoints(msg.poses, self._target_frame_id),
        )
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        self._path_stamp_ns = stamp_ns

        if not new_waypoints:
            self.get_logger().warn(
                "received a path with no usable waypoints"
                + (f" for frame_id={self._target_frame_id!r}" if self._target_frame_id else ""),
                throttle_duration_sec=5.0,
            )
            return

        # planner_node republishes the path; only re-anchor when it changed
        if paths_equal(new_waypoints, self._waypoints):
            return
        self._waypoints = new_waypoints
        self._done = False
        # re-anchor the pursuit index with a fresh global search
        self._last_index = 0
        self.get_logger().info(
            f"new path: {len(self._waypoints)} waypoints, "
            f"goal=({self._waypoints[-1][0]:.2f}, {self._waypoints[-1][1]:.2f})"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn("ignoring non-finite UGV pose", throttle_duration_sec=5.0)
            return
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self._pose = (float(p.x), float(p.y), yaw)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        self._pose_stamp_ns = stamp_ns

    # ------------------------------------------------------------------
    # control loop
    # ------------------------------------------------------------------
    def _age_ok(self, stamp_ns: Optional[int], timeout_s: float, now_ns: int) -> bool:
        if stamp_ns is None:
            return False
        if timeout_s <= 0.0:
            return True
        return (now_ns - stamp_ns) <= int(timeout_s * 1e9)

    def _control_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if not self._age_ok(self._pose_stamp_ns, self._pose_timeout, now_ns):
            self._publish_zero()
            return
        if not self._age_ok(self._path_stamp_ns, self._path_timeout, now_ns):
            self._publish_zero()
            return

        assert self._pose is not None
        x, y, yaw = self._pose

        if self._done:
            self._publish_zero()
            return

        cmd: PursuitCommand = pure_pursuit_command(
            x, y, yaw,
            self._waypoints,
            last_index=self._last_index,
            lookahead_distance=self._lookahead,
            max_linear_speed=self._max_v,
            max_angular_speed=self._max_w,
            goal_tolerance=self._goal_tol,
            slowdown_radius=self._slowdown,
            rotate_in_place_error=self._rotate_err,
        )
        self._last_index = max(self._last_index, cmd.target_index)

        if cmd.mode == "DONE" and not self._done:
            self._done = True
            self.get_logger().info(
                f"goal reached (distance {cmd.goal_distance:.2f} m) -> stopping"
            )

        twist = Twist()
        twist.linear.x = float(cmd.v)
        twist.angular.z = float(cmd.omega)
        self._cmd_pub.publish(twist)

    def _publish_zero(self) -> None:
        twist = Twist()
        self._cmd_pub.publish(twist)

    def destroy_node(self) -> bool:
        # best-effort stop on teardown; base driver watchdog covers the rest
        try:
            self._publish_zero()
        except Exception:  # noqa: BLE001 — never block shutdown
            pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
