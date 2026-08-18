"""Bridge /enclosure_command to /task_assignment.

Subscribes to EnclosureCommandArray (output of enclosure_node) and
converts each enclosure point into a TaskAssignment that planner_node
can consume, closing the loop:

    enclosure_node -> /enclosure_command -> [this bridge] -> /task_assignment -> planner_node

The conversion encodes the float enclosure point (target_x, target_y)
into an integer target_id that planner_node._scatter_target decodes as
``x = target_id % grid_size``, ``y = target_id // grid_size``.
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import (
    EnclosureCommandArray,
    TaskAssignment,
)


class EnclosureCommandBridge(Node):
    """Convert enclosure commands into task assignments for planner_node."""

    def __init__(self) -> None:
        super().__init__("enclosure_command_bridge")
        self.declare_parameter("command_topic", "/enclosure_command")
        self.declare_parameter("output_topic", "/task_assignment")
        self.declare_parameter("grid_size", 100)
        self.declare_parameter("task_type", "enclose")

        command_topic = str(self.get_parameter("command_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._grid_size = max(1, int(self.get_parameter("grid_size").value))
        self._task_type = str(self.get_parameter("task_type").value)

        self._sub = self.create_subscription(
            EnclosureCommandArray, command_topic, self.on_command, 10
        )
        self._pub = self.create_publisher(TaskAssignment, output_topic, 10)

        self.get_logger().info(
            f"enclosure_command_bridge: {command_topic} -> {output_topic} "
            f"(grid_size={self._grid_size})"
        )

    def on_command(self, msg: EnclosureCommandArray) -> None:
        """Convert each enclosure command to a task assignment."""
        for cmd in msg.commands:
            if math.isnan(cmd.target_x) or math.isnan(cmd.target_y):
                continue
            task = TaskAssignment()
            task.drone_id = int(cmd.drone_id)
            gx = int(round(cmd.target_x))
            gy = int(round(cmd.target_y))
            gx = max(0, min(self._grid_size - 1, gx))
            gy = max(0, min(self._grid_size - 1, gy))
            task.target_id = gx + gy * self._grid_size
            task.task_type = self._task_type
            self._pub.publish(task)

    @property
    def grid_size(self) -> int:
        return self._grid_size


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = EnclosureCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
