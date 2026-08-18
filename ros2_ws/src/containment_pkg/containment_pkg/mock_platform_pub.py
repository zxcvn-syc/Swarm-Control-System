#!/usr/bin/env python3
"""Mock platform publisher for layered enclosure development.

Publishes a DroneStateArray containing ``num_drones`` UAVs (monitor layer)
and ``num_cars`` UGVs (block layer) orbiting a configurable target point.
This lets enclosure_node be developed and verified before the real
tracker -> scheduler -> planner chain is ready: replace this node with the
real pipeline later without touching enclosure_node (the consumer only
depends on the message interface).

Usage:
    ros2 run containment_pkg mock_platform_pub
"""

import math

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import DroneState, DroneStateArray

PLATFORM_DRONE = int(getattr(DroneState, "PLATFORM_DRONE", 0))
PLATFORM_CAR = int(getattr(DroneState, "PLATFORM_CAR", 1))


class MockPlatformPub(Node):
    def __init__(self):
        super().__init__("mock_platform_pub")
        self.declare_parameter("period", 0.5)
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("monitor_orbit", 30.0)  # UAV orbit radius
        self.declare_parameter("block_orbit", 18.0)    # UGV orbit radius
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("num_cars", 2)
        self._publisher = self.create_publisher(DroneStateArray, "/drone_states", 10)
        period = max(float(self.get_parameter("period").value), 0.05)
        self._timer = self.create_timer(period, self.publish)
        self._phase = 0.0

    def publish(self):
        self._phase += 0.05
        msg = DroneStateArray()
        msg.drones = []
        tx = float(self.get_parameter("target_x").value)
        ty = float(self.get_parameter("target_y").value)

        # UAVs on the monitor layer (outer ring)
        orbit = float(self.get_parameter("monitor_orbit").value)
        num = int(self.get_parameter("num_drones").value)
        for i in range(num):
            angle = self._phase + 2.0 * math.pi * i / max(num, 1)
            s = DroneState()
            s.drone_id = i
            s.x = tx + orbit * math.cos(angle)
            s.y = ty + orbit * math.sin(angle)
            s.z = 10.0
            s.vx = 0.0
            s.vy = 0.0
            s.vz = 0.0
            s.available = True
            if hasattr(s, "platform_type"):
                s.platform_type = PLATFORM_DRONE
            msg.drones.append(s)

        # UGVs on the block layer (inner ring)
        orbit = float(self.get_parameter("block_orbit").value)
        num = int(self.get_parameter("num_cars").value)
        for i in range(num):
            angle = self._phase + math.pi + 2.0 * math.pi * i / max(num, 1)
            s = DroneState()
            s.drone_id = 100 + i
            s.x = tx + orbit * math.cos(angle)
            s.y = ty + orbit * math.sin(angle)
            s.z = 0.0
            s.vx = 0.0
            s.vy = 0.0
            s.vz = 0.0
            s.available = True
            if hasattr(s, "platform_type"):
                s.platform_type = PLATFORM_CAR
            msg.drones.append(s)

        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockPlatformPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
