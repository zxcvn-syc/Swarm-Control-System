"""grid_map_node — OccupancyGrid → UInt8MultiArray bridge.

Publishes to ``/grid_map`` (``std_msgs/UInt8MultiArray``) so that
``planner_node``'s ``/grid_map`` subscription receives a live feed.

Pipeline
--------
1. Subscribe to ``/grid_map_nav`` (``nav_msgs/OccupancyGrid``) — the
   grid published by ``planner_node`` itself.
2. Every 1 Hz, convert the latest ``OccupancyGrid`` into a flat
   ``UInt8MultiArray`` where each cell value is 0 (free) / 100 (occupied)
   / -1 (unknown).
3. Publish the ``UInt8MultiArray`` on ``/grid_map``.

If ``/grid_map_nav`` has never arrived, the node publishes a default
40×40 all-free ``UInt8MultiArray`` so the rest of the pipeline is never
blocked waiting for grid data.
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

try:
    from nav_msgs.msg import OccupancyGrid
    _HAS_NAV_MSGS = True
except ImportError:  # pragma: no cover
    _HAS_NAV_MSGS = False
    OccupancyGrid = None

from std_msgs.msg import MultiArrayDimension, UInt8MultiArray


_DEFAULT_SIZE = 40
_DEFAULT_DATA = [0] * (_DEFAULT_SIZE * _DEFAULT_SIZE)


class GridMapNode(Node):
    """Bridge from nav_msgs/OccupancyGrid to std_msgs/UInt8MultiArray."""

    def __init__(self) -> None:
        super().__init__("grid_map_node")

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._latest_grid: Optional[OccupancyGrid] = None

        if _HAS_NAV_MSGS:
            self.sub_grid_nav = self.create_subscription(
                OccupancyGrid,
                "/grid_map_nav",
                self._on_grid_nav,
                qos,
            )
        else:
            self.get_logger().warn(
                "nav_msgs not available; grid_map_node will only publish "
                "default grid"
            )
            self.sub_grid_nav = None

        self.pub_grid = self.create_publisher(
            UInt8MultiArray, "/grid_map", qos
        )

        self._timer = self.create_timer(1.0, self._publish_grid)

        self.get_logger().info(
            "grid_map_node up: subscribes /grid_map_nav, "
            "publishes /grid_map (UInt8MultiArray)"
        )

    def _on_grid_nav(self, msg: OccupancyGrid) -> None:
        """Cache the latest OccupancyGrid."""
        self._latest_grid = msg

    def _publish_grid(self) -> None:
        """Convert the cached OccupancyGrid to UInt8MultiArray and publish."""
        msg = UInt8MultiArray()
        msg.layout.data_offset = 0
        msg.layout.dim = [
            MultiArrayDimension(label="height", size=_DEFAULT_SIZE, stride=_DEFAULT_SIZE * _DEFAULT_SIZE),
            MultiArrayDimension(label="width", size=_DEFAULT_SIZE, stride=_DEFAULT_SIZE),
        ]

        if self._latest_grid is not None:
            grid: OccupancyGrid = self._latest_grid
            h = int(grid.info.height)
            w = int(grid.info.width)
            flat = list(grid.data)

            msg.layout.dim[0].size = h
            msg.layout.dim[0].stride = h * w
            msg.layout.dim[1].size = w
            msg.layout.dim[1].stride = w

            if len(flat) < h * w:
                self.get_logger().warn(
                    f"OccupancyGrid data truncated ({len(flat)} < {h * w})"
                )
                flat = flat + [0] * (h * w - len(flat))

            msg.data = flat[: h * w]
        else:
            msg.layout.dim[0].size = _DEFAULT_SIZE
            msg.layout.dim[0].stride = _DEFAULT_SIZE * _DEFAULT_SIZE
            msg.layout.dim[1].size = _DEFAULT_SIZE
            msg.layout.dim[1].stride = _DEFAULT_SIZE
            msg.data = list(_DEFAULT_DATA)

        self.pub_grid.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GridMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
