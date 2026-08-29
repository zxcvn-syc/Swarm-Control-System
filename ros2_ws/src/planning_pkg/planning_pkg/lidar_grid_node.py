"""ROS2 adapter that turns a 2D ``sensor_msgs/LaserScan`` into a map.

This node deliberately does not claim to be SLAM.  It publishes an
instantaneous, horizontal-plane occupancy map in a configured world frame.
It needs a recent world-frame platform position from ``DroneStateArray`` and
uses a configured static yaw because the current ``DroneState`` message has no
orientation field.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from swarm_interfaces.msg import DroneStateArray

from .lidar_grid import GridGeometry, inflate_occupied, rasterize_scan


class LidarGridNode(Node):
    """Publish a geometry-preserving occupancy map from one LiDAR stream."""

    def __init__(self) -> None:
        super().__init__("lidar_grid_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("pose_topic", "/drone_pose_external")
        self.declare_parameter("occupancy_topic", "/lidar_occupancy")
        self.declare_parameter("sensor_id", 100)
        self.declare_parameter("sensor_yaw_rad", 0.0)
        self.declare_parameter("pose_stale_sec", 0.5)
        self.declare_parameter("map_frame", "world")
        self.declare_parameter("resolution", 0.25)
        self.declare_parameter("width", 200)
        self.declare_parameter("height", 200)
        self.declare_parameter("origin_x", -25.0)
        self.declare_parameter("origin_y", -25.0)
        self.declare_parameter("inflation_radius", 0.5)
        self.declare_parameter("max_range", 30.0)

        self.sensor_id = int(self.get_parameter("sensor_id").value)
        self.sensor_yaw = float(self.get_parameter("sensor_yaw_rad").value)
        self.pose_stale_sec = max(
            0.0, float(self.get_parameter("pose_stale_sec").value)
        )
        self.max_range = float(self.get_parameter("max_range").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.geometry = GridGeometry(
            width=int(self.get_parameter("width").value),
            height=int(self.get_parameter("height").value),
            resolution=float(self.get_parameter("resolution").value),
            origin_x=float(self.get_parameter("origin_x").value),
            origin_y=float(self.get_parameter("origin_y").value),
            frame_id=str(self.get_parameter("map_frame").value),
        )
        if not math.isfinite(self.max_range) or self.max_range <= 0.0:
            raise ValueError("max_range must be finite and positive")
        if not math.isfinite(self.inflation_radius) or self.inflation_radius < 0.0:
            raise ValueError("inflation_radius must be finite and non-negative")

        self._poses: Dict[int, Tuple[float, float]] = {}
        self._pose_received_at: Optional[float] = None
        self._last_drop_log_at = 0.0

        pose_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pose_sub = self.create_subscription(
            DroneStateArray,
            str(self.get_parameter("pose_topic").value),
            self._on_poses,
            pose_qos,
        )
        self._scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._publisher = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("occupancy_topic").value),
            pose_qos,
        )
        self.get_logger().info(
            "lidar_grid_node up: sensor_id=%d, scan=%s, pose=%s, map=%s "
            "(%dx%d @ %.3fm, static_yaw=%.3frad)"
            % (
                self.sensor_id,
                self.get_parameter("scan_topic").value,
                self.get_parameter("pose_topic").value,
                self.get_parameter("occupancy_topic").value,
                self.geometry.width,
                self.geometry.height,
                self.geometry.resolution,
                self.sensor_yaw,
            )
        )

    def _on_poses(self, message: DroneStateArray) -> None:
        poses: Dict[int, Tuple[float, float]] = {}
        for state in message.drones:
            try:
                x, y = float(state.x), float(state.y)
            except (AttributeError, TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                poses[int(state.drone_id)] = (x, y)
        if poses:
            self._poses.update(poses)
            self._pose_received_at = time.monotonic()

    def _on_scan(self, message: LaserScan) -> None:
        sensor_pose = self._recent_sensor_pose()
        if sensor_pose is None:
            self._log_dropped_scan("missing or stale platform pose")
            return
        range_max = min(self.max_range, float(message.range_max))
        range_min = max(0.0, float(message.range_min))
        if not math.isfinite(range_max) or range_max <= range_min:
            self._log_dropped_scan("invalid LaserScan range bounds")
            return
        try:
            occupancy = rasterize_scan(
                message.ranges,
                angle_min=float(message.angle_min),
                angle_increment=float(message.angle_increment),
                sensor_x=sensor_pose[0],
                sensor_y=sensor_pose[1],
                sensor_yaw=self.sensor_yaw,
                min_range=range_min,
                max_range=range_max,
                geometry=self.geometry,
            )
            occupancy = inflate_occupied(
                occupancy, self.geometry, self.inflation_radius
            )
        except ValueError as exc:
            self._log_dropped_scan(f"invalid scan: {exc}")
            return

        output = OccupancyGrid()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.geometry.frame_id
        output.info.width = self.geometry.width
        output.info.height = self.geometry.height
        output.info.resolution = self.geometry.resolution
        output.info.origin.position.x = self.geometry.origin_x
        output.info.origin.position.y = self.geometry.origin_y
        output.info.origin.orientation.w = 1.0
        output.data = occupancy.astype(np.int8, copy=False).reshape(-1).tolist()
        self._publisher.publish(output)
        self.get_logger().debug(
            "lidar map: occupied=%d unknown=%d sensor=(%.2f, %.2f)"
            % (
                int(np.count_nonzero(occupancy >= 100)),
                int(np.count_nonzero(occupancy < 0)),
                sensor_pose[0],
                sensor_pose[1],
            )
        )

    def _recent_sensor_pose(self) -> Optional[Tuple[float, float]]:
        if self._pose_received_at is None:
            return None
        if time.monotonic() - self._pose_received_at > self.pose_stale_sec:
            return None
        return self._poses.get(self.sensor_id)

    def _log_dropped_scan(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._last_drop_log_at >= 2.0:
            self.get_logger().warn(f"scan dropped: {reason}")
            self._last_drop_log_at = now


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarGridNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
