"""odom_relay_node — /odom -> /ugv_pose PoseStamped relay.

Why this node exists
--------------------
``ugv_path_follower`` consumes ``geometry_msgs/PoseStamped`` on
``/ugv_pose``.  The HiWonder MentorPi stack already publishes a fused
wheel+laser odometry on ``/odom`` (``ekf_filter_node``, ~30 Hz), so the
real vehicle's pose source is available for free — it just has the
wrong message type.  This node is the thin adapter between the two.

Coordinate-frame note
---------------------
``/odom`` counts from wherever the robot was powered on, while the
planner works in a world frame shared with the drone.  The cheap bench
alignment ritual (documented in ``docs/operations/ugv_real_deployment.md``):
place the car at the field origin facing the world +X axis, then reset
odometry (``ros2 topic pub /set_odom ...`` or restart the container)
so odom == world.  For anything longer-running, publish a TF
``odom -> world`` instead and keep this relay as-is.

Parameters
----------
* ``odom_topic`` (``nav_msgs/Odometry``) — source odometry, default ``/odom``.
* ``pose_topic`` (``geometry_msgs/PoseStamped``) — output, default ``/ugv_pose``.
* ``frame_id`` (str) — output header frame; default ``""`` passes the
  source header through unchanged.
* ``restamp`` (bool) — when true, stamp output with this node's clock
  instead of the source stamp (useful when the source clock is stale
  or from a different machine; ``ugv_path_follower`` checks pose age
  against *its own* clock via ``pose_timeout``).

Safety
------
Pure relay: no outputs besides the pose topic; nothing here can move
the vehicle.  The follower's own pose/path timeouts handle staleness.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from .odom_relay import as_bool, odom_pose_fields


class OdomRelayNode(Node):
    """Relay ``nav_msgs/Odometry`` pose into ``geometry_msgs/PoseStamped``."""

    def __init__(self) -> None:
        super().__init__("ugv_odom_relay")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/ugv_pose")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("restamp", False)

        odom_topic = str(self.get_parameter("odom_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value).strip()
        self._restamp = as_bool(self.get_parameter("restamp").value, False)

        odom_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        pose_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)

        self._sub = self.create_subscription(Odometry, odom_topic, self._on_odom, odom_qos)
        self._pub = self.create_publisher(PoseStamped, pose_topic, pose_qos)

        self.get_logger().info(
            f"ugv_odom_relay ready: odom={odom_topic} pose={pose_topic} "
            f"frame_id={self._frame_id or '<passthrough>'} restamp={self._restamp}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        try:
            _stamp_sec, _stamp_ns, _src_frame, position, orientation = odom_pose_fields(msg)
        except ValueError:
            self.get_logger().warn("ignoring non-finite odometry pose", throttle_duration_sec=5.0)
            return

        out = PoseStamped()
        if self._restamp:
            out.header.stamp = self.get_clock().now().to_msg()
        else:
            out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id or msg.header.frame_id
        out.pose.position = position
        out.pose.orientation = orientation
        self._pub.publish(out)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = OdomRelayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
