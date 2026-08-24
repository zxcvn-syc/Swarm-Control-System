#!/usr/bin/env python3
"""sitl_state_publisher.py — Aggregate SITL mavros poses + mock UGV into DroneStateArray.

Subscribes to /uav{0,1,2}/mavros/local_position/pose (PoseStamped from mavros)
and publishes a DroneStateArray on /drone_states containing:
  - 3 UAVs (drone_id=0/1/2, platform_type=0) with real SITL positions
  - 2 UGVs (drone_id=100/101, platform_type=1) with mock orbiting positions

This replaces mock_platform_pub when running with real PX4 SITL vehicles.

Parameters:
  period          publish period in seconds (default 0.5)
  target_x        target X for UGV orbit center (default 0.0)
  target_y        target Y for UGV orbit center (default 0.0)
  block_orbit     UGV orbit radius (default 15.0)
  num_uav         number of UAVs (default 3)
  num_ugv         number of UGVs (default 2)
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from geometry_msgs.msg import PoseStamped
from swarm_interfaces.msg import DroneState, DroneStateArray

PLATFORM_DRONE = int(getattr(DroneState, "PLATFORM_DRONE", 0))
PLATFORM_CAR = int(getattr(DroneState, "PLATFORM_CAR", 1))


class SITLStatePublisher(Node):
    def __init__(self):
        super().__init__("sitl_state_publisher")

        self.declare_parameter("period", 0.5)
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("block_orbit", 15.0)
        self.declare_parameter("num_uav", 3)
        self.declare_parameter("num_ugv", 2)

        self._num_uav = int(self.get_parameter("num_uav").value)
        self._num_ugv = int(self.get_parameter("num_ugv").value)
        self._target_x = float(self.get_parameter("target_x").value)
        self._target_y = float(self.get_parameter("target_y").value)
        self._block_orbit = float(self.get_parameter("block_orbit").value)

        # mavros pose cache: drone_id -> (x, y, z)
        self._uav_poses = {}
        self._pose_count = 0
        self._pub_count = 0

        # best-effort QoS so we receive mavros' local_position/pose regardless
        # of whether mavros publishes reliable or best-effort.
        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribe to each UAV's mavros local position
        for i in range(self._num_uav):
            topic = f"/uav{i}/mavros/local_position/pose"
            self.create_subscription(
                PoseStamped, topic, self._make_callback(i), qos_profile=pose_qos
            )
            self.get_logger().info(f"subscribed to {topic} (best-effort QoS)")

        self._publisher = self.create_publisher(
            DroneStateArray, "/drone_states", 10
        )

        period = max(float(self.get_parameter("period").value), 0.05)
        self._phase = 0.0
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"sitl_state_publisher started: "
            f"{self._num_uav} UAV (real SITL) + "
            f"{self._num_ugv} UGV (mock)"
        )

    def _make_callback(self, drone_id):
        def callback(msg: PoseStamped):
            self._uav_poses[drone_id] = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            )
            self._pose_count += 1
            if self._pose_count <= 3 or self._pose_count % 20 == 0:
                self.get_logger().info(
                    f"received pose uav{drone_id}: "
                    f"({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, "
                    f"{msg.pose.position.z:.3f})"
                )
        return callback

    def _publish(self):
        self._phase += 0.05
        self._pub_count += 1
        msg = DroneStateArray()
        msg.drones = []

        # Real UAV states from mavros
        for i in range(self._num_uav):
            s = DroneState()
            s.drone_id = i
            s.platform_type = PLATFORM_DRONE
            if i in self._uav_poses:
                s.x, s.y, s.z = self._uav_poses[i]
            else:
                s.x = float(i * 3)  # fallback if mavros not yet connected
                s.y = 0.0
                s.z = 0.0
            s.vx = 0.0
            s.vy = 0.0
            s.vz = 0.0
            s.available = i in self._uav_poses
            msg.drones.append(s)

        # Mock UGV states orbiting the target
        for i in range(self._num_ugv):
            angle = self._phase + math.pi + 2.0 * math.pi * i / max(self._num_ugv, 1)
            s = DroneState()
            s.drone_id = 100 + i
            s.platform_type = PLATFORM_CAR
            s.x = self._target_x + self._block_orbit * math.cos(angle)
            s.y = self._target_y + self._block_orbit * math.sin(angle)
            s.z = 0.0
            s.vx = 0.0
            s.vy = 0.0
            s.vz = 0.0
            s.available = True
            msg.drones.append(s)

        self._publisher.publish(msg)

        if self._pub_count % 10 == 1:
            recv = sum(1 for i in range(self._num_uav) if i in self._uav_poses)
            self.get_logger().info(
                f"publish #{self._pub_count}: UAV pose received {recv}/{self._num_uav}"
            )


def main() -> None:
    rclpy.init(args=sys.argv)
    node = SITLStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
