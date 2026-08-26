#!/usr/bin/env python3
"""Mock platform publisher for layered enclosure development.

Publishes a DroneStateArray containing ``num_drones`` UAVs (monitor layer)
and ``num_cars`` UGVs (block layer) orbiting the *escape target's home
position*.  The home position is taken from the first ``/enclosure_targets``
message so the patrol circle is centred on whatever start the active scene
declares in three_scene_config.yaml (e.g. security starts at [5, -3]) --
otherwise the platforms would orbit the origin and never get within
``intercept_radius`` of a target that starts off-origin, spuriously
producing INVALID verdicts.

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
from swarm_interfaces.msg import (
    DroneState,
    DroneStateArray,
    EnclosureTargetArray,
)

PLATFORM_DRONE = int(getattr(DroneState, "PLATFORM_DRONE", 0))
PLATFORM_CAR = int(getattr(DroneState, "PLATFORM_CAR", 1))


class MockPlatformPub(Node):
    def __init__(self):
        super().__init__("mock_platform_pub")
        self.declare_parameter("period", 0.5)
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_topic", "/enclosure_targets")
        self.declare_parameter("monitor_orbit", 30.0)  # UAV orbit radius
        self.declare_parameter("block_orbit", 18.0)    # UGV orbit radius
        self.declare_parameter(
            "phase_step", 0.2  # rad added per tick (period=0.5s -> 0.4 rad/s)
        )
        # A full 360 deg sweep takes 2*pi/0.4 ~= 15.7s, so within one 20-30s
        # test the patrol covers every bearing.  The ~0.6s close-approach
        # window is wider than the 0.5s /drone_states publish period, so the
        # evaluator's per-frame distance sample is guaranteed to catch the
        # moment a platform is within intercept_radius (no missed INVALID/SUCCESS).
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("num_cars", 2)
        self._publisher = self.create_publisher(
            DroneStateArray, "/drone_states", 10
        )
        # Orbit centre: initialised from target_x/target_y but overwritten by
        # the first /enclosure_targets message (the target's home position).
        self._tx = float(self.get_parameter("target_x").value)
        self._ty = float(self.get_parameter("target_y").value)
        self._centre_captured = False
        self._target_sub = self.create_subscription(
            EnclosureTargetArray,
            str(self.get_parameter("target_topic").value),
            self.on_target,
            10,
        )
        period = max(float(self.get_parameter("period").value), 0.05)
        self._timer = self.create_timer(period, self.publish)
        self._phase = 0.0

    def on_target(self, msg):
        """Capture the target's home position as the orbit centre (once)."""
        if self._centre_captured:
            return
        targets = list(getattr(msg, "targets", []) or [])
        if not targets:
            return
        tgt = targets[0]
        try:
            self._tx = float(getattr(tgt, "x", 0.0))
            self._ty = float(getattr(tgt, "y", 0.0))
        except (TypeError, ValueError):
            return
        self._centre_captured = True

    def publish(self):
        step = float(self.get_parameter("phase_step").value)
        self._phase += step
        msg = DroneStateArray()
        msg.drones = []
        tx = self._tx
        ty = self._ty

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
