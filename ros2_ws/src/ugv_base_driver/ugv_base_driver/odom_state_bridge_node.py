"""Publish one real UGV odometry sample as a swarm DroneStateArray."""

from __future__ import annotations

import math
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from swarm_interfaces.msg import DroneState, DroneStateArray

from .path_follower_node import quaternion_to_yaw
from .path_tracking import body_velocity_to_world


class UgvOdomStateBridge(Node):
    """Adapt nav_msgs/Odometry to the heterogeneous swarm state contract."""

    def __init__(self) -> None:
        super().__init__("ugv_odom_state_bridge")
        self.declare_parameter("vehicle_id", 100)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("state_topic", "/ground_vehicle_states")
        self.declare_parameter("publish_period", 0.1)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("enforce_frame_match", True)
        self.declare_parameter("twist_in_body_frame", True)

        self.vehicle_id = int(self.get_parameter("vehicle_id").value)
        self.publish_period = float(
            self.get_parameter("publish_period").value
        )
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.enforce_frame_match = bool(
            self.get_parameter("enforce_frame_match").value
        )
        self.twist_in_body_frame = bool(
            self.get_parameter("twist_in_body_frame").value
        )
        if self.vehicle_id < 0:
            raise ValueError("vehicle_id must be non-negative")
        if self.publish_period <= 0.0 or self.odom_timeout <= 0.0:
            raise ValueError("bridge timing parameters must be positive")
        if not self.output_frame:
            raise ValueError("output_frame must not be empty")

        self._state: Optional[dict] = None
        self._last_odom_time: Optional[float] = None
        self._frame_matches = False

        odom_topic = str(self.get_parameter("odom_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, 20
        )
        self.state_pub = self.create_publisher(
            DroneStateArray, state_topic, 10
        )
        self.timer = self.create_timer(
            self.publish_period, self.publish_state
        )
        self.get_logger().info(
            "UGV odom state bridge id={} {} -> {}".format(
                self.vehicle_id, odom_topic, state_topic
            )
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, message: Odometry) -> None:
        source_frame = str(message.header.frame_id).strip()
        frame_matches = bool(source_frame) and source_frame == self.output_frame
        if self.enforce_frame_match and not frame_matches:
            self.get_logger().error(
                "odometry frame '{}' cannot be relabeled as '{}'; state is "
                "unavailable until transformed odometry is supplied".format(
                    source_frame or "<empty>", self.output_frame
                ),
                throttle_duration_sec=2.0,
            )
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        try:
            yaw = quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
            x = float(position.x)
            y = float(position.y)
            velocity_x = float(linear.x)
            velocity_y = float(linear.y)
            angular_z = float(message.twist.twist.angular.z)
            values = (x, y, velocity_x, velocity_y, angular_z)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("odometry contains non-finite values")
            if self.twist_in_body_frame:
                velocity_x, velocity_y = body_velocity_to_world(
                    velocity_x, velocity_y, yaw
                )
        except ValueError as exc:
            self.get_logger().error(
                "odometry rejected: {}".format(exc),
                throttle_duration_sec=2.0,
            )
            return

        self._state = {
            "x": x,
            "y": y,
            "vx": velocity_x,
            "vy": velocity_y,
            "yaw_rate": angular_z,
        }
        self._last_odom_time = self._now()
        self._frame_matches = frame_matches

    def publish_state(self) -> None:
        if self._state is None or self._last_odom_time is None:
            return
        now = self._now()
        fresh = now - self._last_odom_time <= self.odom_timeout
        frame_valid = self._frame_matches or not self.enforce_frame_match
        available = fresh and frame_valid

        state = DroneState()
        state.drone_id = self.vehicle_id
        state.x = float(self._state["x"])
        state.y = float(self._state["y"])
        state.z = 0.0
        state.vx = float(self._state["vx"]) if available else 0.0
        state.vy = float(self._state["vy"]) if available else 0.0
        state.vz = 0.0
        state.available = bool(available)
        state.platform_type = int(getattr(DroneState, "PLATFORM_CAR", 1))

        message = DroneStateArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.output_frame
        message.drones = [state]
        message.num_drones = 1
        self.state_pub.publish(message)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = UgvOdomStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
