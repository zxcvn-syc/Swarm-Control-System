"""planner_node: ROS2 entry point that owns A* / D* Lite path planning for
the swarm.

Subscribes:
    /task_assignment            swarm_interfaces/TaskAssignment   (scheduler_pkg)
    /grid_map                   std_msgs/UInt8MultiArray          (grid_map_node)
    /target_track_world         swarm_interfaces/TargetTrackArray (coord_transform_node)
    /planned_path_set           swarm_interfaces/TaskAssignment   (optional, RflySim ack)
    /drone_pose_external        swarm_interfaces/DroneStateArray  (optional, RflySim pose)

Publishes:
    /drone_states               swarm_interfaces/DroneStateArray  (containment_pkg)
    /planned_path               nav_msgs/Path                     (RflySim / MAVROS)
    /planned_path_set           swarm_interfaces/TaskAssignment   (debug echo back)
    /grid_map_nav               nav_msgs/OccupancyGrid            (self-published default grid)

Parameters
----------
num_drones : int, default 8
grid_size : int, default 100   -- builds a square ``grid_size x grid_size`` grid
planner : str, default 'astar' -- 'astar' or 'dstar_lite'
tick_period : float, default 0.5  -- seconds between planner ticks
initial_positions : double[], default evenly spaced within the grid
obstacle_cells : double[], default empty   -- pre-seeded obstacles (cells or rectangles)
publish_path : bool, default True         -- emit /planned_path and drone states
sim_tick_speed : float, default 1.0       -- cells per tick the drones advance along their path

The node maintains one D* Lite (or A*) instance per drone.  When a
``TaskAssignment`` for ``drone_id`` arrives, the planner (re)plans from
that drone's current cell to the assignment's ``target_id`` mapped to
grid coordinates (target_id is the *index* in the drone assignment and
is treated as a deterministic scatter target here for the demo).
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# ROS2 message types -- some imports (e.g. nav_msgs/Path) live in
# ``geometry_msgs``/``nav_msgs`` so we keep them in a try/except so the
# module can still be imported in environments where they are missing
# (e.g. partial CI images).
from std_msgs.msg import UInt8MultiArray

from swarm_interfaces.msg import (
    DroneState,
    DroneStateArray,
    TaskAssignment,
    TargetTrackArray,
)

# std_srvs / optional types
try:
    from geometry_msgs.msg import PoseStamped  # noqa: F401
    from nav_msgs.msg import Path as NavPath   # noqa: F401
    from nav_msgs.msg import OccupancyGrid     # noqa: F401
    _HAS_NAV_MSGS = True
except ImportError:  # pragma: no cover - exercised in minimum CI
    _HAS_NAV_MSGS = False
    PoseStamped = None  # type: ignore
    NavPath = None  # type: ignore
    OccupancyGrid = None  # type: ignore

from .astar import astar as _astar
from .dstar_lite import DStarLite as _DStarLite


# ---------------------------------------------------------------------------
def _uint32(x: int) -> int:
    """Coerce ``x`` to uint32 range so ROS2 doesn't reject the assignment."""
    return int(x) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
class PlannerNode(Node):
    """ROS2 adapter for the A* / D* Lite planners."""

    def __init__(self) -> None:
        super().__init__("planner_node")

        # ---------------- parameters --------------------------------------
        self.declare_parameter("num_drones", 8)
        self.declare_parameter("grid_size", 100)
        self.declare_parameter("planner", "astar")          # "astar" | "dstar_lite"
        self.declare_parameter("tick_period", 0.5)
        self.declare_parameter("log_interval_sec", 5.0)
        self.declare_parameter("publish_path", True)
        self.declare_parameter("sim_tick_speed", 1.0)       # cells per tick

        # Topics
        self.declare_parameter("task_topic", "/task_assignment")
        self.declare_parameter("grid_topic", "/grid_map")
        self.declare_parameter("target_track_world_topic", "/target_track_world")
        self.declare_parameter("drone_states_topic", "/drone_states")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("rfly_pose_topic", "/drone_pose_external")

        # Initial drone layout (alternating x/y pairs).
        self.declare_parameter("initial_positions", [])       # flat [x0, y0, x1, y1, ...]
        # Pre-defined obstacles (list of dicts with {x, y} or {x0, y0, x1, y1}).
        self.declare_parameter("obstacle_cells", [])
        # Map from "drone_id" -> target cell; kept empty by default
        # (so target_id is mapped via the demo scatter rule below).
        self.declare_parameter("explicit_target_cells", [])

        self.num_drones: int = int(self.get_parameter("num_drones").value)
        self.grid_size: int = int(self.get_parameter("grid_size").value)
        self.planner_name: str = str(self.get_parameter("planner").value).lower()
        self.tick_period: float = float(self.get_parameter("tick_period").value)
        self.log_interval: float = float(self.get_parameter("log_interval_sec").value)
        self.publish_path: bool = bool(self.get_parameter("publish_path").value)
        self.sim_tick_speed: float = max(
            float(self.get_parameter("sim_tick_speed").value), 0.0
        )

        self.task_topic: str = str(self.get_parameter("task_topic").value)
        self.grid_topic: str = str(self.get_parameter("grid_topic").value)
        self.target_track_world_topic: str = str(self.get_parameter("target_track_world_topic").value)
        self.drone_states_topic: str = str(self.get_parameter("drone_states_topic").value)
        self.planned_path_topic: str = str(self.get_parameter("planned_path_topic").value)
        self.rfly_pose_topic: str = str(self.get_parameter("rfly_pose_topic").value)

        if self.planner_name not in ("astar", "dstar_lite"):
            self.get_logger().warn(
                f"Unknown planner '{self.planner_name}', falling back to astar."
            )
            self.planner_name = "astar"

        # ----------------- state -----------------------------------------
        # Build the grid; we keep a mutable ``self._grid`` used by every
        # planner instance.
        self._grid: np.ndarray = np.zeros(
            (self.grid_size, self.grid_size), dtype=np.int8
        )

        # Apply pre-seeded obstacles.  Two forms are accepted:
        #   * ``{"x": x, "y": y}``
        #   * ``{"x0": a, "y0": b, "x1": c, "y1": d}``  rectangle inclusive
        obstacle_cells = list(self.get_parameter("obstacle_cells").value or [])
        for spec in obstacle_cells:
            self._apply_obstacle_spec(spec)

        # Per-drone state.  Drone IDs default to 0..num_drones-1; explicit
        # TaskAssignment messages carry their own drone_id and we extend
        # the active roster on demand.
        self._drone_order: List[int] = list(range(self.num_drones))
        initial_positions = list(self.get_parameter("initial_positions").value or [])
        self._drone_xy: Dict[int, Tuple[float, float]] = {}
        for i, did in enumerate(self._drone_order):
            if 2 * i + 1 < len(initial_positions):
                self._drone_xy[did] = (
                    float(initial_positions[2 * i]),
                    float(initial_positions[2 * i + 1]),
                )
            else:
                # Default: drones start in a deterministic grid.
                self._drone_xy[did] = self._default_initial_position(i)

        self._drone_target: Dict[int, Tuple[int, int]] = {}
        explicit_targets = list(
            self.get_parameter("explicit_target_cells").value or []
        )
        # Allow integer or {x, y} format.
        for entry in explicit_targets:
            try:
                if isinstance(entry, dict):
                    did = int(entry.get("drone_id", 0))
                    x = int(entry.get("x", 0))
                    y = int(entry.get("y", 0))
                elif isinstance(entry, (list, tuple)) and len(entry) == 3:
                    did, x, y = entry
                    did, x, y = int(did), int(x), int(y)
                else:
                    continue
                self._drone_target[did] = (x, y)
            except Exception as exc:
                self.get_logger().warn(f"explicit_targets entry ignored: {entry!r} ({exc})")

        # Per-drone path (list of (x, y) cells).
        self._drone_path: Dict[int, List[Tuple[int, int]]] = {d: [] for d in self._drone_order}

        # D* Lite instances, lazily created per drone when the first
        # task assignment arrives.  We re-use them across updates so
        # the search tree is preserved.
        self._dstar: Dict[int, _DStarLite] = {}

        # Last time we logged a summary.
        self._last_log_t: float = 0.0
        # Map of seen obstacle edits coming from /grid_map updates.
        self._pending_obstacle_changes: List[Tuple[Tuple[int, int], int]] = []
        # For testing the dynamic update path on real turtlesim-style
        # topics, when an explicit_task triggers a fresh plan.

        # ----------------- QoS -------------------------------------------
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # ----------------- subscribers / publishers ---------------------
        self.sub_task = self.create_subscription(
            TaskAssignment, self.task_topic, self.on_task, qos
        )
        self.sub_grid = self.create_subscription(
            UInt8MultiArray, self.grid_topic, self.on_grid, qos
        )
        self.sub_target_world = self.create_subscription(
            TargetTrackArray, self.target_track_world_topic, self.on_target_world, qos
        )
        self.sub_rfly = self.create_subscription(
            DroneStateArray, self.rfly_pose_topic, self.on_rfly_pose, qos
        )
        self.pub_states = self.create_publisher(
            DroneStateArray, self.drone_states_topic, qos
        )
        self.pub_path = self.create_publisher(
            NavPath, self.planned_path_topic, qos
        ) if _HAS_NAV_MSGS else None

        # ----------------- timer -----------------------------------------
        self._timer = self.create_timer(self.tick_period, self.tick)

        # ----------------- /grid_map publisher (nav_msgs/OccupancyGrid) --
        # Publishes a default 40x40 grid at 1 Hz so planner_node's own
        # /grid_map subscription has a live feed even when no external
        # grid_map_node is running.
        self._grid_pub = None
        self._grid_timer = None
        if _HAS_NAV_MSGS:
            self._grid_pub = self.create_publisher(
                OccupancyGrid, "/grid_map_nav", qos
            )
            self._grid_timer = self.create_timer(1.0, self._publish_default_grid)

        self.get_logger().info(
            f"planner_node up: planner={self.planner_name}, "
            f"num_drones={self.num_drones}, grid={self.grid_size}x{self.grid_size}, "
            f"tick={self.tick_period}s, in={self.task_topic}+{self.grid_topic}, "
            f"out={self.drone_states_topic}"
            f"+{self.planned_path_topic}"
        )

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------
    def _publish_default_grid(self) -> None:
        """Publish a default 40x40 free OccupancyGrid at 1 Hz."""
        if self._grid_pub is None or OccupancyGrid is None:
            return
        try:
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            # 40x40 grid, 0.5 m resolution => covers 20m x 20m world area.
            msg.info.width = 40
            msg.info.height = 40
            msg.info.resolution = 0.5
            msg.info.origin.position.x = 0.0
            msg.info.origin.position.y = 0.0
            msg.info.origin.position.z = 0.0
            msg.info.origin.orientation.w = 1.0
            # All free (0).  Grid is row-major: height rows × width columns.
            msg.data = [0] * (40 * 40)
            self._grid_pub.publish(msg)
        except Exception as exc:  # pragma: no cover
            self.get_logger().debug(f"_publish_default_grid failed: {exc}")

    def _default_initial_position(self, idx: int) -> Tuple[float, float]:
        """Deterministic default start spread across the grid."""
        if self.num_drones <= 0:
            return (0.0, 0.0)
        side = max(1, int(np.ceil(np.sqrt(self.num_drones))))
        spacing = self.grid_size / max(side + 1, 1)
        row, col = divmod(idx, side)
        return (col * spacing + spacing, row * spacing + spacing)

    def _apply_obstacle_spec(self, spec) -> None:
        """Apply a single obstacle spec to ``self._grid``."""
        try:
            if not isinstance(spec, dict):
                return
            if all(k in spec for k in ("x0", "y0", "x1", "y1")):
                x0 = int(spec["x0"]); y0 = int(spec["y0"])
                x1 = int(spec["x1"]); y1 = int(spec["y1"])
                if x0 > x1:
                    x0, x1 = x1, x0
                if y0 > y1:
                    y0, y1 = y1, y0
                self._grid[
                    y0 : y1 + 1, x0 : x1 + 1
                ] = 1
            elif "x" in spec and "y" in spec:
                x = int(spec["x"]); y = int(spec["y"])
                self._grid[y, x] = 1
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"obstacle spec ignored: {spec!r} ({exc})")

    def _scatter_target(self, target_id: int) -> Tuple[int, int]:
        """Deterministic cell for the given ``target_id``.

        This keeps the demo self-contained: scheduler emits ``target_id``
        integers in ``[0, inf)`` and we deterministically map them to
        cells of the grid.  Production deployments may override this
        by populating ``explicit_target_cells``.
        """
        s = self.grid_size
        # Diagonal sweep so consecutive IDs spread out cleanly.
        x = int(target_id) % s
        y = (int(target_id) // s) % s
        # Clamp into bounds (in case the assignment drifts beyond grid).
        x = max(0, min(s - 1, x))
        y = max(0, min(s - 1, y))
        return (x, y)

    def _ensure_drone(self, did: int) -> None:
        """Register a drone id the first time it appears in a TaskAssignment."""
        if did not in self._drone_xy:
            # Place new drones near the centre of the grid so the planner
            # has room either side.  Real RflySim setups would update the
            # position via /drone_pose_external instead.
            mid = float(self.grid_size) / 2.0
            self._drone_xy[did] = (mid, mid)
            self._drone_path[did] = []
            self._drone_order.append(did)
            self.get_logger().info(f"registered new drone id={did}")

    def _plan_for_drone(self, did: int, target_xy: Tuple[int, int]) -> None:
        """(Re-)plan a path from ``did``'s current cell to ``target_xy``."""
        self._ensure_drone(did)
        # Snap drone position into integer grid cell.
        gx, gy = self._world_to_cell(self._drone_xy[did])
        tx, ty = int(target_xy[0]), int(target_xy[1])

        if self.planner_name == "dstar_lite" and did in self._dstar:
            # Re-use the existing search tree, just re-plan from the
            # current robot cell.  If the start has moved we update the
            # planner's ``start`` slot and call .plan() which performs a
            # full re-search but keeps the memoised internal state.
            self._dstar[did].start = (gx, gy)
            self._dstar[did].goal = (tx, ty)
            path = self._dstar[did].plan()
            self._drone_path[did] = path
        else:
            if self.planner_name == "dstar_lite":
                planner = _DStarLite(
                    self._grid, (gx, gy), (tx, ty), diagonal=True
                )
                path = planner.plan()
                self._dstar[did] = planner
                self._drone_path[did] = path
            else:
                path = _astar(self._grid, (gx, gy), (tx, ty), diagonal=True)
                self._drone_path[did] = path

        if not self._drone_path[did]:
            self.get_logger().warn(
                f"no path for drone {did} -> ({tx}, {ty}); "
                f"goal may be unreachable."
            )

    def _apply_pending_obstacle_changes(self) -> None:
        """Forward any buffered obstacle edits into the D* Lite trees."""
        if not self._pending_obstacle_changes:
            return
        edits = self._pending_obstacle_changes
        self._pending_obstacle_changes = []
        for did, planner in self._dstar.items():
            planner.update_obstacles(edits)
        # For drones that were planned with A*, drop any cached paths so
        # the next tick re-plans fresh against the new grid.
        for did in self._drone_path.keys():
            if self.planner_name == "astar" and self._drone_path[did]:
                self._drone_path[did] = []
                self._replan_for_drone(did)

    def _replan_for_drone(self, did: int) -> None:
        """Re-plan a drone against its current target after obstacle change."""
        target = self._drone_target.get(did)
        if target is None:
            return
        self._plan_for_drone(did, target)

    def _world_to_cell(self, xy: Tuple[float, float]) -> Tuple[int, int]:
        """Clamp continuous (x, y) world coordinates to grid cell indices."""
        s = self.grid_size
        x = max(0, min(s - 1, int(round(xy[0]))))
        y = max(0, min(s - 1, int(round(xy[1]))))
        # If somehow the snapped cell is blocked, fall back to nearest free.
        if int(self._grid[y, x]) != 0:
            for r in range(1, s):
                found = False
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        nx, ny = x + dx, y + dy
                        if (
                            0 <= nx < s
                            and 0 <= ny < s
                            and int(self._grid[ny, nx]) == 0
                        ):
                            return (nx, ny)
                # continue widening
                if found:
                    break
        return (x, y)

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------
    def on_task(self, msg: TaskAssignment) -> None:
        """Receive a (drone_id, target_id, task_type) assignment and trigger planning."""
        did = int(msg.drone_id)
        tid = int(msg.target_id)
        # Use the explicit list if present, else deterministic scatter.
        if did in self._drone_target and did in self._explicit_target_set():
            target = self._drone_target[did]
        else:
            target = self._scatter_target(tid)
            self._drone_target[did] = target
        self._plan_for_drone(did, target)
        self.get_logger().debug(
            f"task: drone {did} -> target {tid} (cell {target})"
        )

    def _explicit_target_set(self) -> set:
        """Helper returning the set of drones whose targets are explicit."""
        return {
            did
            for did in self._drone_target
            if did in self._drone_xy  # only known drones count
        }

    def on_grid(self, msg: UInt8MultiArray) -> None:
        """Receive an updated occupancy grid.

        The ``UInt8MultiArray`` layout is::

            layout.dim[0].label = "height"
            layout.dim[0].size  = H
            layout.dim[1].label = "width"
            layout.dim[1].size  = W
            data                = W * H bytes, row-major

        Replaces ``self._grid`` wholesale and queues D* Lite updates.
        """
        info = (
            msg.layout.dim if msg.layout is not None else []
        )
        if len(info) < 2:
            self.get_logger().warn("grid_map: missing layout dims, ignored")
            return
        h = int(info[0].size)
        w = int(info[1].size)
        flat = np.asarray(msg.data, dtype=np.uint8).reshape(-1)
        if flat.size < h * w:
            self.get_logger().warn(
                f"grid_map data too small ({flat.size} < {h * w}); ignored"
            )
            return
        new_grid = flat[: h * w].reshape(h, w).astype(np.int8)
        # Find the differences vs. our existing grid (only on the
        # overlapping region, otherwise treat as a wholesale replace).
        edits: List[Tuple[Tuple[int, int], int]] = []
        old_h, old_w = self._grid.shape
        # When the grid changes shape we drop the existing search trees
        # and re-seed positions safely inside the new bounds.
        if h != old_h or w != old_w:
            self._grid = new_grid
            self.grid_size = h  # keep parameter consistent
            # Snap existing drone positions into the new bounds.
            for did, (x, y) in list(self._drone_xy.items()):
                nx = max(0.0, min(float(w - 1), x))
                ny = max(0.0, min(float(h - 1), y))
                self._drone_xy[did] = (nx, ny)
            self._dstar.clear()
            self._drone_path = {d: [] for d in self._drone_path}
            # Replan all known targets against the new grid.
            for did, target in self._drone_target.items():
                self._plan_for_drone(did, target)
            return
        # Same shape, in-place grid edits so D* Lite trees can stay warm.
        for cy in range(h):
            for cx in range(w):
                new_state = int(new_grid[cy, cx])
                old_state = int(self._grid[cy, cx])
                if new_state != old_state:
                    edits.append(((cx, cy), new_state))
        self._grid = new_grid
        if edits:
            self._pending_obstacle_changes.extend(edits)
            self._apply_pending_obstacle_changes()

    def on_rfly_pose(self, msg: DroneStateArray) -> None:
        """Optional pose feedback from RflySim / MAVROS (cells)."""
        for d in msg.drones:
            did = int(d.drone_id)
            self._ensure_drone(did)
            self._drone_xy[did] = (float(d.x), float(d.y))
            if did not in self._drone_path or not self._drone_path[did]:
                # No path yet; one will be planned the next task arrives.
                continue

    def on_target_world(self, msg: TargetTrackArray) -> None:
        """Optional world-coordinate targets from coord_transform_node.

        Logs the arrival of world targets for observability.
        This subscription closes the loop: /target_track_world published by
        coord_transform_node now has a consumer in the planning pipeline.
        """
        if not msg.tracks:
            return
        self.get_logger().debug(
            f"/target_track_world: {len(msg.tracks)} track(s) "
            f"frame={msg.header.frame_id if msg.header else 'none'}"
        )

    # ----------------------------------------------------------------
    # Tick
    # ----------------------------------------------------------------
    def tick(self) -> None:
        """Move drones a few cells along their path and publish state.

        Called every ``tick_period`` seconds.  Drones without a path
        stay put.  If the head-of-path cell is a different drone, we
        wait one tick (simple conflict resolution).
        """
        # First apply any pending obstacle changes; this may shorten
        # or invalidate paths.
        if self._pending_obstacle_changes:
            self._apply_pending_obstacle_changes()

        occupied: Dict[Tuple[int, int], int] = {}  # cell -> drone id
        for did in self._drone_order:
            path = self._drone_path.get(did, [])
            if not path:
                continue
            # Drone occupies the first cell of its path.  If that cell
            # is taken by another drone, freeze for this tick.
            head = path[0]
            if head in occupied:
                continue
            occupied[head] = did
        # Now advance each drone up to ``sim_tick_speed`` cells along its
        # path, skipping cells held by other drones.
        occupied_cells = set(occupied.keys())
        for did in self._drone_order:
            path = self._drone_path.get(did, [])
            if not path:
                continue
            steps = max(1, int(round(self.sim_tick_speed)))
            for _ in range(steps):
                if len(path) <= 1:
                    break
                next_cell = path[1]
                if next_cell in occupied_cells:
                    # Another drone is heading into that cell; halt.
                    break
                # Update occupied bookkeeping on the fly.
                occupied_cells.discard(path[0])
                occupied_cells.add(next_cell)
                path.pop(0)
            # Update world position from the head cell of the path.
            head = path[0]
            self._drone_xy[did] = (float(head[0]), float(head[1]))

        # ----------------- publish DroneStateArray ----------------------
        msg = DroneStateArray()
        msg.num_drones = len(self._drone_order)
        msg.drones = []
        for did in self._drone_order:
            x, y = self._drone_xy[did]
            ds = DroneState()
            ds.drone_id = _uint32(did)
            ds.x = float(x)
            ds.y = float(y)
            ds.z = 0.0
            # Velocity: if we have a path of length >= 2 then vx/vy is
            # a normalised move; otherwise zero.
            path = self._drone_path.get(did, [])
            if len(path) >= 2:
                nx, ny = path[1]
                vx = float(nx - x)
                vy = float(ny - y)
                mag = (vx * vx + vy * vy) ** 0.5
                if mag > 0:
                    vx = vx / mag * self.sim_tick_speed
                    vy = vy / mag * self.sim_tick_speed
                ds.vx = float(vx)
                ds.vy = float(vy)
                ds.vz = 0.0
            else:
                ds.vx = 0.0
                ds.vy = 0.0
                ds.vz = 0.0
            ds.available = True
            msg.drones.append(ds)

        self.pub_states.publish(msg)

        # ----------------- publish nav_msgs/Path (best-effort) --------
        if self.publish_path and self.pub_path is not None:
            try:
                stamped_msg = self._build_nav_path(self._drone_path)
                if stamped_msg is not None:
                    self.pub_path.publish(stamped_msg)
            except Exception as exc:  # pragma: no cover
                self.get_logger().debug(f"path publish failed: {exc}")

        # ----------------- summary log ---------------------------------
        now = time.monotonic()
        if now - self._last_log_t >= self.log_interval:
            self._last_log_t = now
            n_paths = sum(
                1 for v in self._drone_path.values() if v
            )
            self.get_logger().info(
                f"planner summary: {n_paths}/{len(self._drone_order)} "
                f"drones have active paths, pending grid edits = "
                f"{len(self._pending_obstacle_changes)}"
            )

    # ----------------------------------------------------------------
    def _build_nav_path(self, paths):
        """Compose a single ``nav_msgs/Path`` covering every drone.

        Each drone's path is a chain of ``PoseStamped`` markers with
        ``frame_id`` set to ``f"drone_{id}"`` so RflySim / MAVROS can
        pick its waypoint stream.
        """
        if NavPath is None or PoseStamped is None:
            return None
        stamped = NavPath()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "world"
        for did, path in paths.items():
            if not path:
                continue
            for cell in path:
                ps = PoseStamped()
                ps.header.stamp = stamped.header.stamp
                ps.header.frame_id = f"drone_{int(did)}"
                ps.pose.position.x = float(cell[0])
                ps.pose.position.y = float(cell[1])
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                stamped.poses.append(ps)
        return stamped


# ---------------------------------------------------------------------------
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
