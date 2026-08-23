"""Publish calibration, virtual camera pose, and obstacles for video replay.

This node deliberately never publishes targets.  ``tracker_node`` remains the
only target producer; the fixture only supplies the non-image inputs required
by the real coordinate-transform and planning nodes during an offline replay.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import MultiArrayDimension, UInt8MultiArray


class VideoReplayFixture(Node):
    """Supply fixed, explicit calibration for a local video replay."""

    def __init__(self) -> None:
        super().__init__("video_replay_fixture")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("drone_pose_topic", "/drone_pose")
        self.declare_parameter("obstacle_topic", "/grid_obstacles")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("image_width", 1920)
        self.declare_parameter("image_height", 1080)
        self.declare_parameter("fx", 960.0)
        self.declare_parameter("fy", 960.0)
        self.declare_parameter("cx", 960.0)
        self.declare_parameter("cy", 540.0)
        self.declare_parameter("camera_x", 35.0)
        self.declare_parameter("camera_y", 20.0)
        self.declare_parameter("camera_altitude", 10.0)
        self.declare_parameter("grid_size", 40)
        self.declare_parameter("obstacle_x_min", 21)
        self.declare_parameter("obstacle_x_max", 26)
        self.declare_parameter("obstacle_y_min", 12)
        self.declare_parameter("obstacle_y_max", 28)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._width = int(self.get_parameter("image_width").value)
        self._height = int(self.get_parameter("image_height").value)
        self._fx = float(self.get_parameter("fx").value)
        self._fy = float(self.get_parameter("fy").value)
        self._cx = float(self.get_parameter("cx").value)
        self._cy = float(self.get_parameter("cy").value)
        self._camera_x = float(self.get_parameter("camera_x").value)
        self._camera_y = float(self.get_parameter("camera_y").value)
        self._camera_altitude = float(self.get_parameter("camera_altitude").value)
        self._grid_size = int(self.get_parameter("grid_size").value)
        self._obstacle_bounds = (
            int(self.get_parameter("obstacle_x_min").value),
            int(self.get_parameter("obstacle_x_max").value),
            int(self.get_parameter("obstacle_y_min").value),
            int(self.get_parameter("obstacle_y_max").value),
        )
        values = (
            self._fx,
            self._fy,
            self._cx,
            self._cy,
            self._camera_x,
            self._camera_y,
            self._camera_altitude,
        )
        if (
            self._width <= 0
            or self._height <= 0
            or self._grid_size <= 0
            or self._fx <= 0
            or self._fy <= 0
            or self._camera_altitude <= 0
            or not all(math.isfinite(value) for value in values)
        ):
            raise ValueError("invalid replay-camera or grid parameters")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._camera_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            qos,
        )
        self._pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("drone_pose_topic").value),
            qos,
        )
        self._obstacle_pub = self.create_publisher(
            UInt8MultiArray,
            str(self.get_parameter("obstacle_topic").value),
            qos,
        )
        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "video replay fixture ready: it publishes calibration, pose, and "
            "obstacles only; target tracks must come from tracker_node"
        )

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()

        camera = CameraInfo()
        camera.header.stamp = stamp
        camera.header.frame_id = self._frame_id
        camera.width = self._width
        camera.height = self._height
        camera.k = [
            self._fx, 0.0, self._cx,
            0.0, self._fy, self._cy,
            0.0, 0.0, 1.0,
        ]
        camera.p = [
            self._fx, 0.0, self._cx, 0.0,
            0.0, self._fy, self._cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self._camera_pub.publish(camera)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x = self._camera_x
        pose.pose.position.y = self._camera_y
        pose.pose.position.z = self._camera_altitude
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        obstacle = UInt8MultiArray()
        obstacle.layout.dim = [
            MultiArrayDimension(
                label="height",
                size=self._grid_size,
                stride=self._grid_size * self._grid_size,
            ),
            MultiArrayDimension(
                label="width", size=self._grid_size, stride=self._grid_size
            ),
        ]
        obstacle.data = [0] * (self._grid_size * self._grid_size)
        x_min, x_max, y_min, y_max = self._obstacle_bounds
        for y in range(max(0, y_min), min(self._grid_size - 1, y_max) + 1):
            for x in range(max(0, x_min), min(self._grid_size - 1, x_max) + 1):
                obstacle.data[y * self._grid_size + x] = 1
        self._obstacle_pub.publish(obstacle)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = VideoReplayFixture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
