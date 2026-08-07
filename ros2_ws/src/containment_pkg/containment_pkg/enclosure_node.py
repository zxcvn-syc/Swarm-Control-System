import time

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

    # UAV = aerial platform (default enclosure_radius, 25 m).
    # UGV = ground platform; uses smaller ugv_enclosure_radius (12 m) for
    # tighter ground ring containment. Matches `DroneState.platform_type`
    # uint8 enum (0 = UAV, 1 = UGV) introduced in commit 36a1545.

    def __init__(self):
        super().__init__("enclosure_node")
        self.declare_parameter("enclosure_radius", 25.0)
        self.declare_parameter("ugv_enclosure_radius", 12.0)
        self.declare_parameter("min_dist", 5.0)
        self.declare_parameter("update_period", 1.0)
        # Default to world-coordinate tracker output (D-2 决议强化, P1-D).
        # Override with `target_track_topic:=/target_track` to keep the
        # legacy pixel-stream fallback path.
        self.declare_parameter("target_track_topic", "/target_track_world")
        self._targets = []
        self._drones = []
        self._dirty = False
        self._last_update_time = None
        self._update_count = 0
        # First-message INFO log so operators can verify topic wiring
        # without enabling DEBUG logging on launch.
        self._log_first_message = {"targets": False, "drones": False}
        target_topic = str(self.get_parameter("target_track_topic").value)
        self._target_track_sub = self.create_subscription(
            TargetTrackArray, target_topic, self.on_target_track, 10
        )
        self._enclosure_target_sub = self.create_subscription(
            EnclosureTargetArray, "/enclosure_targets", self.on_enclosure_targets, 10
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
        """Use tracker output as the primary target source.

        Expects world-coordinate meters from ``/target_track_world`` (D-2).
        The pixel ``/target_track`` stream is accepted as a degraded fallback
        when launched with ``target_track_topic:=/target_track``.
        """
        self._targets = list(getattr(message, "tracks", []))
        self._dirty = True
        if not self._log_first_message["targets"]:
            stamp = self._stamp_repr(message)
            self.get_logger().info(
                "first /target_track input: topic=%s tracks=%d stamp=%s"
                % (self._target_track_sub.topic_name, len(self._targets), stamp)
            )
            self._log_first_message["targets"] = True

    def on_enclosure_targets(self, message):
        """Accept the legacy enclosure-target message as a fallback input."""
        self._targets = list(getattr(message, "targets", []))
        self._dirty = True

    def on_drone(self, message):
        self._drones = self._state_list(message)
        self._dirty = True
        if not self._log_first_message["drones"]:
            stamp = self._stamp_repr(message)
            self.get_logger().info(
                "first /drone_states input: topic=%s drones=%d stamp=%s"
                % (self._drone_sub.topic_name, len(self._drones), stamp)
            )
            self._log_first_message["drones"] = True

    # Compatibility aliases for callers using the old callback names.
    _targets_callback = on_enclosure_targets
    _drones_callback = on_drone

    @staticmethod
    def _state_list(message):
        return list(getattr(message, "drones", getattr(message, "states", [])))

    @staticmethod
    def _stamp_repr(message):
        """Best-effort header timestamp string for first-message logs."""
        header = getattr(message, "header", None)
        if header is None:
            return "n/a"
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return "n/a"
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if sec is None or nanosec is None:
            return "n/a"
        return "%d.%09d" % (int(sec), int(nanosec))

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
        # Per-drone enclosure radius: UGVs use ugv_enclosure_radius (smaller
        # ground ring), UAVs use the default enclosure_radius. The Voronoi
        # kernel keeps its single-scalar signature (P1-D: no voronoi.py
        # changes), so we pass the UAV radius as the kernel's
        # ``enclosure_radius`` and override the per-drone radius in the
        # published EnclosureCommand based on `DroneState.platform_type`.
        # This preserves the EnclosureCommandArray output shape while
        # honouring heterogeneous platform scales.
        ugv_radius = float(self.get_parameter("ugv_enclosure_radius").value)
        uav_radius = float(self.get_parameter("enclosure_radius").value)
        start = time.perf_counter()
        drone_targets, _radii = voronoi_enclose(
            target_xy,
            drone_xy,
            uav_radius,
            float(self.get_parameter("min_dist").value),
        )
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.get_logger().debug(f"Voronoi update completed in {elapsed_ms:.3f} ms")

        # Per-drone radius lookup: 0 = UAV -> uav_radius, 1 = UGV -> ugv_radius.
        # Unknown platform_type values fall back to UAV radius to avoid
        # silently zeroing containment.
        per_drone_radius = []
        for state in states:
            ptype = int(getattr(state, "platform_type", 0))
            per_drone_radius.append(ugv_radius if ptype == 1 else uav_radius)

        command = EnclosureCommandArray()
        command.num_drones = len(states)
        active_count = min(len(states), len(targets))
        command.commands = []
        for index, state in enumerate(states):
            if index >= active_count:
                command.commands.append(self._standby(state))
                continue
            item = EnclosureCommand()
            item.drone_id = int(state.drone_id)
            item.target_x = float(drone_targets[index, 0])
            item.target_y = float(drone_targets[index, 1])
            item.target_z = float(getattr(state, "z", 0.0))
            item.enclosure_radius = float(per_drone_radius[index])
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
