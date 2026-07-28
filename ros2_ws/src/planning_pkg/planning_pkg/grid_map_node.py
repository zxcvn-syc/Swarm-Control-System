"""grid_map_node — OccupancyGrid → UInt8MultiArray bridge.

Publishes to ``/grid_map`` (``std_msgs/UInt8MultiArray``) so that
``planner_node``'s ``/grid_map`` subscription receives a live feed.

Pipeline
--------
1. Subscribe to ``/grid_map_nav`` (``nav_msgs/OccupancyGrid``) — the
   grid published by ``planner_node`` itself.
2. Subscribe to ``/grid_obstacles`` (``std_msgs/UInt8MultiArray`` with a
   2-D layout, ``1`` = obstacle) — an externally injected obstacle
   mask that is OR-ed into the grid on every publish cycle so callers
   can drop new obstacles without rebuilding the whole pipeline.
3. Every 1 Hz, convert the latest ``OccupancyGrid`` into a flat
   ``UInt8MultiArray`` where each cell value is 0 (free) / 100 (occupied)
   / -1 (unknown), then OR in the obstacle mask and publish on
   ``/grid_map``.

If ``/grid_map_nav`` has never arrived, the node publishes a default
40×40 all-free ``UInt8MultiArray`` so the rest of the pipeline is never
blocked waiting for grid data.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

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
    """Bridge from nav_msgs/OccupancyGrid to std_msgs/UInt8MultiArray.

    Also subscribes to ``/grid_obstacles`` so the integration test (and
    future perception modules) can stamp an obstacle mask into the
    published grid without needing a full SLAM/OccupancyGrid producer.
    """

    def __init__(self) -> None:
        super().__init__("grid_map_node")

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # Internal obstacle mask: kept as a list-of-tuples so that the
        # memory cost stays proportional to the number of cells actually
        # flagged, and we can OR it into arbitrary /grid_map payloads
        # without copying whole numpy arrays.
        self._obstacle_cells: List[Tuple[int, int]] = []
        self._obstacle_h: int = _DEFAULT_SIZE
        self._obstacle_w: int = _DEFAULT_SIZE

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

        self.sub_obstacles = self.create_subscription(
            UInt8MultiArray,
            "/grid_obstacles",
            self._on_obstacles,
            qos,
        )

        self.pub_grid = self.create_publisher(
            UInt8MultiArray, "/grid_map", qos
        )

        self._timer = self.create_timer(1.0, self._publish_grid)

        self.get_logger().info(
            "grid_map_node up: subscribes /grid_map_nav + /grid_obstacles, "
            "publishes /grid_map (UInt8MultiArray)"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_grid_nav(self, msg: OccupancyGrid) -> None:
        """Cache the latest OccupancyGrid."""
        self._latest_grid = msg

    def _on_obstacles(self, msg: UInt8MultiArray) -> None:
        """Receive an externally injected obstacle mask.

        Layout conventions follow ``/grid_map``: ``dim[0]`` is height and
        ``dim[1]`` is width.  ``1`` marks an obstacle cell.  The mask is
        OR-ed into the published grid on the next tick.
        """
        try:
            if msg.layout is None or len(msg.layout.dim) < 2:
                self.get_logger().warn(
                    "/grid_obstacles: missing layout dims, ignored"
                )
                return
            h = int(msg.layout.dim[0].size)
            w = int(msg.layout.dim[1].size)
            data = list(msg.data)
            if len(data) < h * w:
                self.get_logger().warn(
                    f"/grid_obstacles truncated ({len(data)} < {h * w}); ignored"
                )
                return
            # Build the obstacle cell list from the mask.
            cells: List[Tuple[int, int]] = []
            flat = data[: h * w]
            for idx, v in enumerate(flat):
                if int(v) != 0:
                    cx = idx % w
                    cy = idx // w
                    cells.append((cx, cy))
            self._obstacle_h = h
            self._obstacle_w = w
            self._obstacle_cells = cells
            self.get_logger().info(
                f"/grid_obstacles: stored {len(cells)} obstacle cells "
                f"({w}x{h})"
            )
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"/grid_obstacles handler failed: {exc}")

    # ------------------------------------------------------------------
    # Publish loop
    # ------------------------------------------------------------------
    def _publish_grid(self) -> None:
        """Convert the cached OccupancyGrid to UInt8MultiArray and publish.

        The /grid_obstacles mask is OR-ed into the published data so
        downstream subscribers see obstacles from either source.
        """
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

        # OR-in the externally injected obstacle mask (only when shapes
        # agree; otherwise the mask targets a different world frame and
        # we skip it so we don't corrupt the grid).
        h_pub = int(msg.layout.dim[0].size)
        w_pub = int(msg.layout.dim[1].size)
        if (
            self._obstacle_cells
            and h_pub == self._obstacle_h
            and w_pub == self._obstacle_w
        ):
            for cx, cy in self._obstacle_cells:
                idx = cy * w_pub + cx
                if 0 <= idx < len(msg.data):
                    msg.data[idx] = 1

        self.pub_grid.publish(msg)

        # Observable metric: how many cells in the published payload
        # are blocked, and how many were just toggled by /grid_obstacles.
        n_blocked = sum(1 for v in msg.data if int(v) != 0)
        n_obstacle = len(self._obstacle_cells)
        self.get_logger().info(
            f"metric grid.obstacles={n_blocked}, "
            f"grid.injected_cells={n_obstacle}, "
            f"grid.size={w_pub}x{h_pub}"
        )


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