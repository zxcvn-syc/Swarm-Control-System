import time

import numpy as np
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import (
    DroneStateArray,
    EnclosureCommand,
    EnclosureCommandArray,
    EnclosureTargetArray,
)

from .voronoi import voronoi_enclose


class EnclosureNode(Node):
    """ROS2 adapter for the Voronoi enclosure calculation."""

    def __init__(self):
        super().__init__("enclosure_node")
        self.declare_parameter("enclosure_radius", 25.0)
        self.declare_parameter("min_dist", 5.0)
        self.declare_parameter("update_period", 1.0)
        self._targets = None
        self._drones = None
        self._target_sub = self.create_subscription(
            EnclosureTargetArray, "/enclosure_targets", self._targets_callback, 10
        )
        self._drone_sub = self.create_subscription(
            DroneStateArray, "/drone_states", self._drones_callback, 10
        )
        self._publisher = self.create_publisher(
            EnclosureCommandArray, "/enclosure_command", 10
        )
        period = max(float(self.get_parameter("update_period").value), 0.01)
        self._timer = self.create_timer(period, self.tick)

    def _targets_callback(self, message):
        self._targets = message

    def _drones_callback(self, message):
        self._drones = message

    @staticmethod
    def _state_list(message):
        return list(getattr(message, "states", getattr(message, "drones", [])))

    def tick(self):
        if self._targets is None or self._drones is None:
            return
        states = self._state_list(self._drones)
        targets = list(getattr(self._targets, "targets", []))
        command = EnclosureCommandArray()
        command.num_drones = len(states)
        if not states:
            command.commands = []
            self._publisher.publish(command)
            return
        if not targets:
            command.commands = [self._standby(state) for state in states]
            self._publisher.publish(command)
            return

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
        self.get_logger().debug("Voronoi tick completed in %.3f ms", elapsed_ms)

        commands = []
        active_count = min(len(states), len(targets))
        for index, state in enumerate(states):
            if index >= active_count:
                commands.append(self._standby(state))
                continue
            item = EnclosureCommand()
            item.drone_id = int(state.drone_id)
            item.target_x = float(drone_targets[index, 0])
            item.target_y = float(drone_targets[index, 1])
            item.target_z = float(getattr(state, "z", 0.0))
            item.enclosure_radius = float(radii[index])
            commands.append(item)
        command.commands = commands
        self._publisher.publish(command)

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
