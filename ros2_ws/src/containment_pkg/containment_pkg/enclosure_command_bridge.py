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
        # Meters per grid cell. Default 1.0 m/cell preserves the legacy
        # "1 cell == 1 m" encoding.  If your real scenario uses cells of a
        # different size (e.g. 0.5 m for a tighter map), set this to 0.5
        # so that world meters are mapped into the correct grid index.
        self.declare_parameter("resolution", 1.0)
        # When True, points that fall outside the [0, grid_size) grid are
        # dropped instead of being clamped to the boundary (which would
        # collapse every out-of-bounds target onto the same edge cell).
        self.declare_parameter("drop_out_of_bounds", True)

        command_topic = str(self.get_parameter("command_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._grid_size = max(1, int(self.get_parameter("grid_size").value))
        self._task_type = str(self.get_parameter("task_type").value)
        resolution = float(self.get_parameter("resolution").value)
        self._resolution = resolution if resolution > 0.0 else 1.0
        self._drop_out_of_bounds = bool(
            self.get_parameter("drop_out_of_bounds").value
        )
        # Maximum in-bounds coordinate (in world meters) for logging/diag.
        self._max_world_extent = self._grid_size * self._resolution

        self._sub = self.create_subscription(
            EnclosureCommandArray, command_topic, self.on_command, 10
        )
        self._pub = self.create_publisher(TaskAssignment, output_topic, 10)

        self.get_logger().info(
            f"enclosure_command_bridge: {command_topic} -> {output_topic} "
            f"(grid_size={self._grid_size}, resolution={self._resolution} m/cell, "
            f"drop_out_of_bounds={self._drop_out_of_bounds})"
        )

    def on_command(self, msg: EnclosureCommandArray) -> None:
        """Convert each enclosure command to a task assignment."""
        dropped = 0
        for cmd in msg.commands:
            if math.isnan(cmd.target_x) or math.isnan(cmd.target_y):
                continue
            task = TaskAssignment()
            task.drone_id = int(cmd.drone_id)
            # Map world meters -> grid indices using resolution.
            gx = int(round(cmd.target_x / self._resolution))
            gy = int(round(cmd.target_y / self._resolution))
            in_bounds = 0 <= gx < self._grid_size and 0 <= gy < self._grid_size
            if not in_bounds:
                if self._drop_out_of_bounds:
                    dropped += 1
                    continue
                # Fallback: clamp to the boundary grid cell.
                gx = max(0, min(self._grid_size - 1, gx))
                gy = max(0, min(self._grid_size - 1, gy))
            task.target_id = gx + gy * self._grid_size
            task.task_type = self._task_type
            self._pub.publish(task)
        if dropped:
            self.get_logger().warn(
                f"dropped {dropped} commands outside [0, "
                f"{self._grid_size}) grid "
                f"(world > {self._max_world_extent} m at resolution="
                f"{self._resolution} m/cell)"
            )

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
