"""Republish MAVROS pose feedback as ``DroneStateArray``."""

from __future__ import annotations

from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from swarm_interfaces.msg import DroneState, DroneStateArray


class SITLPoseBridge(Node):
    """Convert a MAVROS mocap/vision pose into the planner state contract."""

    def __init__(self) -> None:
        super().__init__("sitl_pose_bridge")
        self.declare_parameter("pose_topic", "/mavros/mocap/pose")
        self.declare_parameter("state_topic", "/drone_pose_external")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("platform_type", 0)
        self.declare_parameter("frame_id", "world")
        self.pub = self.create_publisher(DroneStateArray, str(self.get_parameter("state_topic").value), 10)
        self.sub = self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self.on_pose, 10)
        self._last_pose = None

    def on_pose(self, message: PoseStamped) -> None:
        """Publish the latest pose, estimating velocity from consecutive samples."""
        now = self.get_clock().now().nanoseconds * 1e-9
        pose = message.pose.position
        vx = vy = vz = 0.0
        if self._last_pose is not None:
            previous_time, previous = self._last_pose
            dt = now - previous_time
            if dt > 1e-6:
                vx, vy, vz = (pose.x - previous.x) / dt, (pose.y - previous.y) / dt, (pose.z - previous.z) / dt
        self._last_pose = (now, pose)
        state = DroneState()
        state.drone_id = int(self.get_parameter("drone_id").value)
        state.x, state.y, state.z = float(pose.x), float(pose.y), float(pose.z)
        state.vx, state.vy, state.vz = float(vx), float(vy), float(vz)
        state.available = True
        state.platform_type = max(0, min(255, int(self.get_parameter("platform_type").value)))
        output = DroneStateArray()
        output.num_drones = 1
        output.drones = [state]
        output.header.stamp = message.header.stamp
        output.header.frame_id = str(self.get_parameter("frame_id").value) or message.header.frame_id
        self.pub.publish(output)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SITLPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
