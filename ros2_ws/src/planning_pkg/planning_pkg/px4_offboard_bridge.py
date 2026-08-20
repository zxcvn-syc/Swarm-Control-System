"""Bridge planner paths to MAVROS local-position setpoints."""

from __future__ import annotations

from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from mavros_msgs.msg import PositionTarget


class PX4OffboardBridge(Node):
    """Publish the active planned waypoint as a MAVROS position target."""

    def __init__(self) -> None:
        super().__init__("px4_offboard_bridge")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("setpoint_topic", "/mavros/setpoint_raw/local")
        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("coordinate_frame", PositionTarget.FRAME_LOCAL_NED)
        self._waypoints: List[Tuple[float, float, float]] = []
        self._index = 0
        self.sub = self.create_subscription(Path, str(self.get_parameter("path_topic").value), self.on_path, 10)
        self.pub = self.create_publisher(PositionTarget, str(self.get_parameter("setpoint_topic").value), 10)
        self.timer = self.create_timer(max(0.01, float(self.get_parameter("publish_period").value)), self.tick)

    def on_path(self, message: Path) -> None:
        """Replace the waypoint queue; empty paths safely clear the setpoint."""
        self._waypoints = [(float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z)) for p in message.poses]
        self._index = 0

    def tick(self) -> None:
        if not self._waypoints:
            return
        x, y, z = self._waypoints[min(self._index, len(self._waypoints) - 1)]
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = int(self.get_parameter("coordinate_frame").value)
        target.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
        target.type_mask |= PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        target.type_mask |= PositionTarget.IGNORE_YAW_RATE
        target.position.x, target.position.y, target.position.z = x, y, z
        target.yaw = 0.0
        self.pub.publish(target)
        if self._index + 1 < len(self._waypoints):
            self._index += 1


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PX4OffboardBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
