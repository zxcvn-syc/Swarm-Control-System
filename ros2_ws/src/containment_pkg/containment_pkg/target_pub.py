#!/usr/bin/env python3
"""Mock target tracker for the layered-enclosure integration demo.

Publishes one (or more) slowly moving targets on ``/target_track`` so that
enclosure_node has something to enclose.  In the real pipeline this role is
played by perception_pkg/tracker_node (YOLOv8 + DeepSORT); this node lets the
UGV block layer be exercised end-to-end before the tracker chain is ready.

The target orbits a small circle so the Voronoi regions re-compute on every
tick -- demonstrating the *dynamic* behaviour of the UGV/UAV containment.
Replace this node with tracker_node later; the consumer (enclosure_node) only
depends on the TargetTrackArray message interface.

Usage:
    ros2 run containment_pkg target_pub
"""

import math

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import TargetTrack, TargetTrackArray

import std_msgs.msg


class TargetPub(Node):
    def __init__(self):
        super().__init__("target_pub")
        self.declare_parameter("period", 0.5)
        self.declare_parameter("num_targets", 1)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("orbit_radius", 3.0)
        self.declare_parameter("orbit_speed", 0.3)
        self._publisher = self.create_publisher(TargetTrackArray, "/target_track", 10)
        period = max(float(self.get_parameter("period").value), 0.05)
        self._timer = self.create_timer(period, self.publish)
        self._phase = 0.0
        self._frame = 0

    def publish(self):
        self._phase += float(self.get_parameter("orbit_speed").value)
        msg = TargetTrackArray()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.frame_idx = self._frame
        self._frame += 1
        msg.tracks = []

        cx = float(self.get_parameter("center_x").value)
        cy = float(self.get_parameter("center_y").value)
        radius = float(self.get_parameter("orbit_radius").value)
        num = int(self.get_parameter("num_targets").value)
        for i in range(num):
            angle = self._phase + 2.0 * math.pi * i / max(num, 1)
            t = TargetTrack()
            t.target_id = i + 1
            t.x = cx + radius * math.cos(angle)
            t.y = cy + radius * math.sin(angle)
            t.vx = -radius * math.sin(angle)
            t.vy = radius * math.cos(angle)
            t.confidence = 0.95
            t.cls = 0  # person
            t.is_confirmed = True
            t.speed = radius
            t.motion_mode = 2  # slow
            msg.tracks.append(t)
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
