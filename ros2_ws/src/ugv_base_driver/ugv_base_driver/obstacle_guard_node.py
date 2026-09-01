"""Fail-closed LaserScan and depth-image safety gate for one UGV."""

from __future__ import annotations

import json
import math
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .obstacle_safety import (
    guarded_twist,
    minimum_depth_distance,
    minimum_laser_distance,
)


class UgvObstacleGuard(Node):
    """Gate navigation velocity using fresh forward obstacle measurements."""

    def __init__(self) -> None:
        super().__init__("ugv_obstacle_guard")
        self.declare_parameter("cmd_vel_input_topic", "cmd_vel_nav")
        self.declare_parameter("cmd_vel_output_topic", "cmd_vel")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("depth_topic", "camera/depth/image_raw")
        self.declare_parameter("enable_topic", "enable")
        self.declare_parameter("estop_topic", "estop")
        self.declare_parameter("status_topic", "obstacle_status")
        self.declare_parameter("require_lidar", True)
        self.declare_parameter("require_depth", True)
        self.declare_parameter("control_period", 0.05)
        self.declare_parameter("status_period", 0.5)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("sensor_timeout", 0.4)
        self.declare_parameter("base_stop_distance", 0.6)
        self.declare_parameter("slowdown_distance", 1.8)
        self.declare_parameter("reaction_time", 0.2)
        self.declare_parameter("max_deceleration", 1.0)
        self.declare_parameter("lidar_forward_half_angle", 0.7)
        self.declare_parameter("lidar_confirmation_count", 2)
        self.declare_parameter("lidar_distance_offset", 0.0)
        self.declare_parameter("depth_roi_x_min", 0.25)
        self.declare_parameter("depth_roi_x_max", 0.75)
        self.declare_parameter("depth_roi_y_min", 0.3)
        self.declare_parameter("depth_roi_y_max", 0.8)
        self.declare_parameter("depth_sample_stride", 4)
        self.declare_parameter("depth_confirmation_count", 20)
        self.declare_parameter("depth_range_min", 0.15)
        self.declare_parameter("depth_range_max", 8.0)
        self.declare_parameter("depth_distance_offset", 0.0)
        self.declare_parameter("allow_reverse_without_rear_sensor", False)
        self.declare_parameter("allow_rotation_when_blocked", False)

        self.require_lidar = bool(self.get_parameter("require_lidar").value)
        self.require_depth = bool(self.get_parameter("require_depth").value)
        self.control_period = float(self.get_parameter("control_period").value)
        self.status_period = float(self.get_parameter("status_period").value)
        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        self.base_stop_distance = float(
            self.get_parameter("base_stop_distance").value
        )
        self.slowdown_distance = float(
            self.get_parameter("slowdown_distance").value
        )
        self.reaction_time = float(self.get_parameter("reaction_time").value)
        self.max_deceleration = float(
            self.get_parameter("max_deceleration").value
        )
        self.lidar_forward_half_angle = float(
            self.get_parameter("lidar_forward_half_angle").value
        )
        self.lidar_confirmation_count = int(
            self.get_parameter("lidar_confirmation_count").value
        )
        self.lidar_distance_offset = float(
            self.get_parameter("lidar_distance_offset").value
        )
        self.depth_roi = (
            float(self.get_parameter("depth_roi_x_min").value),
            float(self.get_parameter("depth_roi_x_max").value),
            float(self.get_parameter("depth_roi_y_min").value),
            float(self.get_parameter("depth_roi_y_max").value),
        )
        self.depth_sample_stride = int(
            self.get_parameter("depth_sample_stride").value
        )
        self.depth_confirmation_count = int(
            self.get_parameter("depth_confirmation_count").value
        )
        self.depth_range_min = float(
            self.get_parameter("depth_range_min").value
        )
        self.depth_range_max = float(
            self.get_parameter("depth_range_max").value
        )
        self.depth_distance_offset = float(
            self.get_parameter("depth_distance_offset").value
        )
        self.allow_reverse = bool(
            self.get_parameter("allow_reverse_without_rear_sensor").value
        )
        self.allow_rotation = bool(
            self.get_parameter("allow_rotation_when_blocked").value
        )
        self._validate_parameters()

        self._enabled = False
        self._estop_latched = False
        self._fault_latched = False
        self._fault_reason = ""
        self._command: Optional[Tuple[float, float]] = None
        self._command_time: Optional[float] = None
        self._lidar_distance: Optional[float] = None
        self._lidar_time: Optional[float] = None
        self._depth_distance: Optional[float] = None
        self._depth_time: Optional[float] = None
        self._state = "disabled"
        self._speed_scale = 0.0
        self._effective_stop_distance = self.base_stop_distance

        cmd_input = str(self.get_parameter("cmd_vel_input_topic").value)
        cmd_output = str(self.get_parameter("cmd_vel_output_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        enable_topic = str(self.get_parameter("enable_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.command_sub = self.create_subscription(
            Twist, cmd_input, self.on_command, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.on_depth, qos_profile_sensor_data
        )
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.on_enable, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, estop_topic, self.on_estop, 10
        )
        self.command_pub = self.create_publisher(Twist, cmd_output, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.reset_service = self.create_service(
            Trigger, "~/reset_fault", self.on_reset_fault
        )
        self.control_timer = self.create_timer(
            self.control_period, self.control_tick
        )
        self.status_timer = self.create_timer(
            self.status_period, self.publish_status
        )
        self.publish_stop("startup")
        self.get_logger().warn(
            "obstacle guard disabled; input={} output={} lidar={} depth={}".format(
                cmd_input, cmd_output, scan_topic, depth_topic
            )
        )

    def _validate_parameters(self) -> None:
        positive = {
            "control_period": self.control_period,
            "status_period": self.status_period,
            "command_timeout": self.command_timeout,
            "sensor_timeout": self.sensor_timeout,
            "slowdown_distance": self.slowdown_distance,
            "max_deceleration": self.max_deceleration,
            "lidar_forward_half_angle": self.lidar_forward_half_angle,
            "depth_range_min": self.depth_range_min,
            "depth_range_max": self.depth_range_max,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        non_negative = {
            "base_stop_distance": self.base_stop_distance,
            "reaction_time": self.reaction_time,
            "lidar_distance_offset": self.lidar_distance_offset,
            "depth_distance_offset": self.depth_distance_offset,
        }
        for name, value in non_negative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("{} must be finite and non-negative".format(name))
        if self.slowdown_distance <= self.base_stop_distance:
            raise ValueError("slowdown_distance must exceed base_stop_distance")
        if self.lidar_forward_half_angle > math.pi:
            raise ValueError("lidar_forward_half_angle must not exceed pi")
        if self.lidar_confirmation_count <= 0:
            raise ValueError("lidar_confirmation_count must be positive")
        if self.depth_confirmation_count <= 0 or self.depth_sample_stride <= 0:
            raise ValueError("depth sampling parameters must be positive")
        x_min, x_max, y_min, y_max = self.depth_roi
        if not (0.0 <= x_min < x_max <= 1.0):
            raise ValueError("depth horizontal ROI is invalid")
        if not (0.0 <= y_min < y_max <= 1.0):
            raise ValueError("depth vertical ROI is invalid")
        if self.depth_range_max <= self.depth_range_min:
            raise ValueError("depth range bounds are invalid")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_command(self, message: Twist) -> None:
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self._latch_fault("non-finite navigation command")
            return
        self._command = (linear, angular)
        self._command_time = self._now()

    def on_scan(self, message: LaserScan) -> None:
        try:
            distance = minimum_laser_distance(
                message.ranges,
                angle_min=float(message.angle_min),
                angle_increment=float(message.angle_increment),
                range_min=float(message.range_min),
                range_max=float(message.range_max),
                forward_half_angle=self.lidar_forward_half_angle,
                confirmation_count=self.lidar_confirmation_count,
            )
        except ValueError as exc:
            self.get_logger().error(
                "LaserScan rejected: {}".format(exc),
                throttle_duration_sec=2.0,
            )
            return
        self._lidar_distance = (
            None
            if distance is None
            else max(0.0, distance - self.lidar_distance_offset)
        )
        self._lidar_time = self._now()

    def on_depth(self, message: Image) -> None:
        x_min, x_max, y_min, y_max = self.depth_roi
        try:
            distance = minimum_depth_distance(
                message.data,
                width=int(message.width),
                height=int(message.height),
                step=int(message.step),
                encoding=str(message.encoding),
                is_bigendian=bool(message.is_bigendian),
                roi_x_min=x_min,
                roi_x_max=x_max,
                roi_y_min=y_min,
                roi_y_max=y_max,
                sample_stride=self.depth_sample_stride,
                range_min=self.depth_range_min,
                range_max=self.depth_range_max,
                confirmation_count=self.depth_confirmation_count,
            )
        except ValueError as exc:
            self.get_logger().error(
                "depth image rejected: {}".format(exc),
                throttle_duration_sec=2.0,
            )
            return
        self._depth_distance = (
            None
            if distance is None
            else max(0.0, distance - self.depth_distance_offset)
        )
        self._depth_time = self._now()

    def on_enable(self, message: Bool) -> None:
        if not bool(message.data):
            self._enabled = False
            self.publish_stop("disabled")
            return
        if self._estop_latched or self._fault_latched:
            self.publish_stop("enable_rejected_fault")
            return
        self._enabled = True
        self._state = "enabled_waiting_for_fresh_inputs"
        self.get_logger().warn("obstacle guard enabled")

    def on_estop(self, message: Bool) -> None:
        if not bool(message.data):
            self.get_logger().warn(
                "false on estop does not clear the latch; call ~/reset_fault"
            )
            return
        self._estop_latched = True
        self._enabled = False
        self.publish_stop("emergency_stop")

    def on_reset_fault(
        self, _request: Trigger.Request, response: Trigger.Response
    ):
        if self._enabled:
            response.success = False
            response.message = "disable obstacle guard before reset"
            return response
        self._estop_latched = False
        self._fault_latched = False
        self._fault_reason = ""
        self.publish_stop("fault_reset")
        response.success = True
        response.message = "faults cleared; obstacle guard remains disabled"
        return response

    def _sensor_clearances(self, now: float):
        clearances = []
        sensors = (
            (
                "lidar",
                self.require_lidar,
                self._lidar_time,
                self._lidar_distance,
            ),
            (
                "depth",
                self.require_depth,
                self._depth_time,
                self._depth_distance,
            ),
        )
        for name, required, timestamp, distance in sensors:
            if timestamp is None:
                if required:
                    return (), "{}_missing".format(name)
                continue
            if now - timestamp > self.sensor_timeout:
                if required:
                    return (), "{}_stale".format(name)
                continue
            if distance is None:
                if required:
                    return (), "{}_no_valid_ranges".format(name)
                continue
            clearances.append(distance)
        return tuple(clearances), ""

    def control_tick(self) -> None:
        now = self._now()
        if not self._enabled:
            self.publish_stop("disabled")
            return
        if self._estop_latched or self._fault_latched:
            self.publish_stop("fault_latched")
            return
        if self._command is None or self._command_time is None:
            self.publish_stop("command_missing")
            return
        if now - self._command_time > self.command_timeout:
            self.publish_stop("command_stale")
            return

        clearances, sensor_error = self._sensor_clearances(now)
        if sensor_error:
            self.publish_stop(sensor_error)
            return
        clearance = min(clearances) if clearances else math.inf
        linear, angular = self._command
        try:
            safe_linear, safe_angular, scale, stop_distance = guarded_twist(
                linear,
                angular,
                clearance,
                base_stop_distance=self.base_stop_distance,
                slowdown_distance=self.slowdown_distance,
                reaction_time=self.reaction_time,
                max_deceleration=self.max_deceleration,
                allow_reverse_without_rear_sensor=self.allow_reverse,
                allow_rotation_when_blocked=self.allow_rotation,
            )
        except ValueError as exc:
            self._latch_fault("safety calculation failed: {}".format(exc))
            self.publish_stop("safety_calculation_fault")
            return

        output = Twist()
        output.linear.x = float(safe_linear)
        output.angular.z = float(safe_angular)
        self.command_pub.publish(output)
        self._speed_scale = scale
        self._effective_stop_distance = stop_distance
        self._state = "clear" if scale >= 1.0 else (
            "blocked" if scale <= 0.0 else "slowing"
        )

    def _latch_fault(self, reason: str) -> None:
        self._enabled = False
        self._fault_latched = True
        self._fault_reason = reason
        self.get_logger().error(reason, throttle_duration_sec=2.0)

    def publish_stop(self, reason: str) -> None:
        self.command_pub.publish(Twist())
        self._state = reason
        self._speed_scale = 0.0

    def publish_status(self) -> None:
        now = self._now()

        def age(timestamp: Optional[float]):
            return None if timestamp is None else max(0.0, now - timestamp)

        message = String()
        message.data = json.dumps(
            {
                "enabled": self._enabled,
                "estop_latched": self._estop_latched,
                "fault_latched": self._fault_latched,
                "fault_reason": self._fault_reason,
                "state": self._state,
                "speed_scale": self._speed_scale,
                "effective_stop_distance_m": self._effective_stop_distance,
                "lidar_distance_m": self._lidar_distance,
                "depth_distance_m": self._depth_distance,
                "command_age_sec": age(self._command_time),
                "lidar_age_sec": age(self._lidar_time),
                "depth_age_sec": age(self._depth_time),
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
    node = UgvObstacleGuard()
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
