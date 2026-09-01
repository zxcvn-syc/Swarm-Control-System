"""Guarded pure-pursuit path follower for one ground vehicle."""

from __future__ import annotations

import json
import math
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .path_tracking import (
    Point2D,
    Pose2D,
    pure_pursuit_command,
    scale_path_points,
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = (value / norm for value in values)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


class UgvPathFollower(Node):
    """Convert this vehicle's planned path into a fail-closed Twist stream."""

    def __init__(self) -> None:
        super().__init__("ugv_path_follower")

        self.declare_parameter("vehicle_id", 100)
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("enable_topic", "~/enable")
        self.declare_parameter("estop_topic", "~/estop")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("vehicle_frame_prefix", "drone_")
        self.declare_parameter("accept_unlabeled_path", False)
        self.declare_parameter("enforce_frame_match", True)
        self.declare_parameter("path_resolution", 1.0)
        self.declare_parameter("path_origin_x", 0.0)
        self.declare_parameter("path_origin_y", 0.0)
        self.declare_parameter("control_period", 0.05)
        self.declare_parameter("status_period", 0.5)
        self.declare_parameter("path_timeout", 2.0)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("lookahead_distance", 0.8)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("min_linear_speed", 0.08)
        self.declare_parameter("slowdown_radius", 1.5)
        self.declare_parameter("rotate_in_place_angle", 0.8)
        self.declare_parameter("heading_gain", 1.8)
        self.declare_parameter("auto_disable_at_goal", True)

        self.vehicle_id = int(self.get_parameter("vehicle_id").value)
        self.vehicle_label = "{}{}".format(
            str(self.get_parameter("vehicle_frame_prefix").value),
            self.vehicle_id,
        )
        self.accept_unlabeled = bool(
            self.get_parameter("accept_unlabeled_path").value
        )
        self.enforce_frame_match = bool(
            self.get_parameter("enforce_frame_match").value
        )
        self.path_resolution = float(
            self.get_parameter("path_resolution").value
        )
        self.path_origin_x = float(self.get_parameter("path_origin_x").value)
        self.path_origin_y = float(self.get_parameter("path_origin_y").value)
        self.control_period = float(self.get_parameter("control_period").value)
        self.status_period = float(self.get_parameter("status_period").value)
        self.path_timeout = float(self.get_parameter("path_timeout").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.lookahead_distance = float(
            self.get_parameter("lookahead_distance").value
        )
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.min_linear_speed = float(
            self.get_parameter("min_linear_speed").value
        )
        self.slowdown_radius = float(
            self.get_parameter("slowdown_radius").value
        )
        self.rotate_in_place_angle = float(
            self.get_parameter("rotate_in_place_angle").value
        )
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.auto_disable_at_goal = bool(
            self.get_parameter("auto_disable_at_goal").value
        )
        self._validate_parameters()

        self._enabled = False
        self._estop_latched = False
        self._path: Tuple[Point2D, ...] = ()
        self._path_frame = ""
        self._path_time: Optional[float] = None
        self._pose: Optional[Pose2D] = None
        self._odom_frame = ""
        self._odom_time: Optional[float] = None
        self._progress_index = 0
        self._last_state = "disabled"

        path_topic = str(self.get_parameter("path_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        enable_topic = str(self.get_parameter("enable_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.path_sub = self.create_subscription(
            Path, path_topic, self.on_path, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, 20
        )
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.on_enable, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, estop_topic, self.on_estop, 10
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.reset_service = self.create_service(
            Trigger, "~/reset_estop", self.on_reset_estop
        )
        self.control_timer = self.create_timer(
            self.control_period, self.control_tick
        )
        self.status_timer = self.create_timer(
            self.status_period, self.publish_status
        )

        self.publish_stop("startup")
        self.get_logger().warn(
            "path follower vehicle_id={} label={} disabled; path={} odom={}".format(
                self.vehicle_id, self.vehicle_label, path_topic, odom_topic
            )
        )

    def _validate_parameters(self) -> None:
        if self.vehicle_id < 0:
            raise ValueError("vehicle_id must be non-negative")
        positive = {
            "path_resolution": self.path_resolution,
            "control_period": self.control_period,
            "status_period": self.status_period,
            "path_timeout": self.path_timeout,
            "odom_timeout": self.odom_timeout,
            "lookahead_distance": self.lookahead_distance,
            "goal_tolerance": self.goal_tolerance,
            "max_linear_speed": self.max_linear_speed,
            "max_angular_speed": self.max_angular_speed,
            "slowdown_radius": self.slowdown_radius,
            "rotate_in_place_angle": self.rotate_in_place_angle,
            "heading_gain": self.heading_gain,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if (
            not math.isfinite(self.min_linear_speed)
            or self.min_linear_speed < 0.0
            or self.min_linear_speed > self.max_linear_speed
        ):
            raise ValueError("min_linear_speed is outside the valid range")
        if not math.isfinite(self.path_origin_x) or not math.isfinite(
            self.path_origin_y
        ):
            raise ValueError("path origin must be finite")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_path(self, message: Path) -> None:
        selected = [
            pose
            for pose in message.poses
            if str(pose.header.frame_id) == self.vehicle_label
        ]
        if not selected and self.accept_unlabeled:
            selected = [
                pose
                for pose in message.poses
                if not str(pose.header.frame_id)
                or str(pose.header.frame_id) == str(message.header.frame_id)
            ]
        if not selected:
            self._path = ()
            self._path_frame = ""
            self._progress_index = 0
            self._path_time = self._now()
            self._last_state = "path_for_vehicle_missing"
            self.publish_stop(self._last_state)
            return

        try:
            self._path = scale_path_points(
                (
                    Point2D(
                        float(pose.pose.position.x),
                        float(pose.pose.position.y),
                    )
                    for pose in selected
                ),
                resolution=self.path_resolution,
                origin_x=self.path_origin_x,
                origin_y=self.path_origin_y,
            )
        except ValueError as exc:
            self._path = ()
            self._path_frame = ""
            self._path_time = self._now()
            self._progress_index = 0
            self._last_state = "invalid_path"
            self.get_logger().error("path rejected: {}".format(exc))
            self.publish_stop(self._last_state)
            return

        self._path_frame = str(message.header.frame_id)
        self._path_time = self._now()
        self._progress_index = 0
        self._last_state = "path_ready"

    def on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        try:
            pose = Pose2D(
                float(position.x),
                float(position.y),
                quaternion_to_yaw(
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                ),
            )
            if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
                raise ValueError("odometry pose is not finite")
        except ValueError as exc:
            self._last_state = "invalid_odometry"
            self.get_logger().error(
                "odometry rejected: {}".format(exc), throttle_duration_sec=2.0
            )
            self.publish_stop(self._last_state)
            return
        self._pose = pose
        self._odom_frame = str(message.header.frame_id)
        self._odom_time = self._now()

    def on_enable(self, message: Bool) -> None:
        if not bool(message.data):
            self._enabled = False
            self._last_state = "disabled"
            self.publish_stop(self._last_state)
            return
        if self._estop_latched:
            self._enabled = False
            self._last_state = "enable_rejected_estop"
            self.publish_stop(self._last_state)
            return
        self._enabled = True
        self._progress_index = 0
        self._last_state = "enabled"
        self.get_logger().warn("path following enabled")

    def on_estop(self, message: Bool) -> None:
        if not bool(message.data):
            self.get_logger().warn(
                "false on estop does not clear the latch; call ~/reset_estop"
            )
            return
        self._estop_latched = True
        self._enabled = False
        self._last_state = "emergency_stop"
        self.publish_stop(self._last_state)

    def on_reset_estop(
        self, _request: Trigger.Request, response: Trigger.Response
    ):
        if self._enabled:
            response.success = False
            response.message = "disable path following before reset"
            return response
        self._estop_latched = False
        self._last_state = "estop_reset"
        self.publish_stop(self._last_state)
        response.success = True
        response.message = "estop cleared; path follower remains disabled"
        return response

    def control_tick(self) -> None:
        now = self._now()
        if not self._enabled:
            self.publish_stop("disabled")
            return
        if self._estop_latched:
            self.publish_stop("emergency_stop")
            return
        if not self._path or self._path_time is None:
            self.publish_stop("path_missing")
            return
        if self._pose is None or self._odom_time is None:
            self.publish_stop("odometry_missing")
            return
        if now - self._path_time > self.path_timeout:
            self.publish_stop("path_stale")
            return
        if now - self._odom_time > self.odom_timeout:
            self.publish_stop("odometry_stale")
            return
        if (
            self.enforce_frame_match
            and self._path_frame
            and self._odom_frame
            and self._path_frame != self._odom_frame
        ):
            self.publish_stop(
                "frame_mismatch:{}!={}".format(
                    self._path_frame, self._odom_frame
                )
            )
            return

        try:
            command = pure_pursuit_command(
                self._path,
                self._pose,
                progress_index=self._progress_index,
                lookahead_distance=self.lookahead_distance,
                goal_tolerance=self.goal_tolerance,
                max_linear_speed=self.max_linear_speed,
                max_angular_speed=self.max_angular_speed,
                min_linear_speed=self.min_linear_speed,
                slowdown_radius=self.slowdown_radius,
                rotate_in_place_angle=self.rotate_in_place_angle,
                heading_gain=self.heading_gain,
            )
        except ValueError as exc:
            self.get_logger().error(
                "tracking command failed: {}".format(exc),
                throttle_duration_sec=2.0,
            )
            self.publish_stop("tracking_error")
            return

        self._progress_index = max(
            self._progress_index, command.closest_index
        )
        if command.goal_reached:
            self.publish_stop("goal_reached")
            if self.auto_disable_at_goal:
                self._enabled = False
            return

        output = Twist()
        output.linear.x = float(command.linear)
        output.angular.z = float(command.angular)
        self.cmd_pub.publish(output)
        self._last_state = "tracking"

    def publish_stop(self, reason: str) -> None:
        self.cmd_pub.publish(Twist())
        self._last_state = reason

    def publish_status(self) -> None:
        now = self._now()
        message = String()
        message.data = json.dumps(
            {
                "vehicle_id": self.vehicle_id,
                "enabled": self._enabled,
                "estop_latched": self._estop_latched,
                "state": self._last_state,
                "path_points": len(self._path),
                "progress_index": self._progress_index,
                "path_frame": self._path_frame,
                "odom_frame": self._odom_frame,
                "path_age_sec": None
                if self._path_time is None
                else max(0.0, now - self._path_time),
                "odom_age_sec": None
                if self._odom_time is None
                else max(0.0, now - self._odom_time),
            },
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def close(self) -> None:
        self._enabled = False
        if rclpy.ok():
            self.publish_stop("node_shutdown")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = UgvPathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
