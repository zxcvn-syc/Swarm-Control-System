import time
import math

import numpy as np
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import (
    DroneStateArray,
    EnclosureCommand,
    EnclosureCommandArray,
    EnclosureTargetArray,
    TargetTrackArray,
)

from .voronoi import voronoi_enclose


class EnclosureNode(Node):
    """Compute dynamic Voronoi enclosure commands from ROS2 state updates."""

    def __init__(self):
        super().__init__("enclosure_node")
        self.declare_parameter("enclosure_radius", 25.0)
        self.declare_parameter("min_dist", 5.0)
        self.declare_parameter("update_period", 1.0)
        self.declare_parameter("target_topic", "/target_track_world")
        self.declare_parameter("world_frame", "world")
        self._targets = []
        self._drones = []
        self._dirty = False
        self._last_update_time = None
        self._update_count = 0
        self._target_track_sub = self.create_subscription(
            TargetTrackArray,
            str(self.get_parameter("target_topic").value),
            self.on_target_track,
            10,
        )
        self._drone_sub = self.create_subscription(
            DroneStateArray, "/drone_states", self.on_drone, 10
        )
        self._publisher = self.create_publisher(
            EnclosureCommandArray, "/enclosure_command", 10
        )
        period = max(float(self.get_parameter("update_period").value), 0.01)
        self._timer = self.create_timer(period, self.tick)

    def on_target_track(self, message):
        """Accept only validated metre tracks from the world-frame bridge."""
        self._set_world_targets(message, "tracks")

    def on_enclosure_targets(self, message):
        """Compatibility callback that still enforces the world contract."""
        self._set_world_targets(message, "targets")

    def _set_world_targets(self, message, attribute):
        frame_id = str(
            getattr(getattr(message, "header", None), "frame_id", "")
        ).strip()
        expected_frame = str(self.get_parameter("world_frame").value).strip() or "world"
        if frame_id != expected_frame:
            self.get_logger().warn("ignoring targets outside the configured world frame")
            return

        targets = []
        for target in getattr(message, attribute, []):
            if not bool(getattr(target, "world_valid", False)):
                continue
            if str(getattr(target, "units", "")).strip() != "m":
                continue
            try:
                x, y = float(target.x), float(target.y)
            except (AttributeError, TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                targets.append(target)
        self._targets = targets
        self._dirty = True

    def on_drone(self, message):
        self._drones = self._state_list(message)
        self._dirty = True

    # Compatibility aliases for callers using the old callback names.
    _targets_callback = on_enclosure_targets
    _drones_callback = on_drone

    @staticmethod
    def _state_list(message):
        return list(getattr(message, "drones", getattr(message, "states", [])))

    def tick(self):
        """Recalculate at most once per timer period when state changed."""
        if not self._dirty or not self._targets or not self._drones:
            return False
        self._recalculate()
        return True

    def _recalculate(self):
        states = self._drones
        targets = self._targets
        target_xy = np.array([[target.x, target.y] for target in targets], dtype=float)
        drone_xy = np.array([[state.x, state.y] for state in states], dtype=float)
        start = time.perf_counter()
        drone_targets, radii = voronoi_enclose(
            target_xy,
            drone_xy,
            float(self.get_parameter("enclosure_radius").value),
            float(self.get_parameter("min_dist").value),
        )
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.get_logger().debug(f"Voronoi update completed in {elapsed_ms:.3f} ms")

        command = EnclosureCommandArray()
        command.num_drones = len(states)
        command.commands = []
        for index, state in enumerate(states):
            item = EnclosureCommand()
            item.drone_id = int(state.drone_id)
            item.target_x = float(drone_targets[index, 0])
            item.target_y = float(drone_targets[index, 1])
            item.target_z = float(getattr(state, "z", 0.0))
            item.enclosure_radius = float(radii[index])
            command.commands.append(item)
        self._publisher.publish(command)
        self._dirty = False
        self._last_update_time = self.get_clock().now()
        self._update_count += 1

    @staticmethod
    def _standby(state):
        item = EnclosureCommand()
        item.drone_id = int(state.drone_id)
        item.target_x = float("nan")
        item.target_y = float("nan")
        item.target_z = float("nan")
        item.enclosure_radius = 0.0
        return item


def main(args=None):
    rclpy.init(args=args)
    node = EnclosureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
