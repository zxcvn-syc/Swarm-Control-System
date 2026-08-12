"""planner_node: ROS2 entry point that owns A* / D* Lite path planning for
the swarm with kinematic constraints support (UAV vs UGV).

Subscribes:
    /task_assignment            swarm_interfaces/TaskAssignment   (scheduler_pkg)
    /grid_map                   std_msgs/UInt8MultiArray          (grid_map_node)
    /target_track_world         swarm_interfaces/TargetTrackArray (coord_transform_node)
    /drone_pose_external        swarm_interfaces/DroneStateArray  (optional, RflySim pose)

Publishes:
    /drone_states               swarm_interfaces/DroneStateArray  (containment_pkg)
    /planned_path               nav_msgs/Path                     (RflySim / MAVROS)
    /planned_path_set           swarm_interfaces/TaskAssignment   (debug echo back)
    /grid_map_nav               nav_msgs/OccupancyGrid            (self-published default grid)
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

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
except ImportError:  # pragma: no cover
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


def _uint8(x: int) -> int:
    """Coerce ``x`` to uint8 range."""
    return int(x) & 0xFF


# ---------------------------------------------------------------------------
class PlannerNode(Node):
    """ROS2 adapter for the A* / D* Lite planners with UAV/UGV Kinematic constraints."""

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

        # 运动学约束参数 (Kinematic Constraints)
        self.declare_parameter("platform_type", 0)          # 0: PLATFORM_DRONE, 1: PLATFORM_CAR
        self.declare_parameter("min_turning_radius", 2.0)   # 最小转向半径 (grid units)
        self.declare_parameter("max_speed_diff", 0.2)       # 最大速度差异限制
        self.declare_parameter("drone_z_default", 1.0)      # 无人机默认飞行高度

        # Topics
        self.declare_parameter("task_topic", "/task_assignment")
        self.declare_parameter("grid_topic", "/grid_map")
        self.declare_parameter("target_track_world_topic", "/target_track_world")
        self.declare_parameter("drone_states_topic", "/drone_states")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("rfly_pose_topic", "/drone_pose_external")

        # Initial drone layout
        self.declare_parameter("initial_positions", [])       # flat [x0, y0, x1, y1, ...]
        self.declare_parameter("obstacle_cells", [])
        self.declare_parameter("explicit_target_cells", [])

        # Parameter values
        self.num_drones: int = int(self.get_parameter("num_drones").value)
        self.grid_size: int = int(self.get_parameter("grid_size").value)
        self.planner_name: str = str(self.get_parameter("planner").value).lower()
        self.tick_period: float = float(self.get_parameter("tick_period").value)
        self.log_interval: float = float(self.get_parameter("log_interval_sec").value)
        self.publish_path: bool = bool(self.get_parameter("publish_path").value)
        self.sim_tick_speed: float = max(
            float(self.get_parameter("sim_tick_speed").value), 0.0
        )

        self.platform_type: int = int(self.get_parameter("platform_type").value)
        self.min_turning_radius: float = float(self.get_parameter("min_turning_radius").value)
        self.max_speed_diff: float = float(self.get_parameter("max_speed_diff").value)
        self.drone_z_default: float = float(self.get_parameter("drone_z_default").value)

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
        self._grid: np.ndarray = np.zeros(
            (self.grid_size, self.grid_size), dtype=np.int8
        )

        obstacle_cells = list(self.get_parameter("obstacle_cells").value or [])
        for spec in obstacle_cells:
            self._apply_obstacle_spec(spec)

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
                self._drone_xy[did] = self._default_initial_position(i)

        self._drone_target: Dict[int, Tuple[int, int]] = {}
        explicit_targets = list(
            self.get_parameter("explicit_target_cells").value or []
        )
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

        self._drone_path: Dict[int, List[Tuple[int, int]]] = {d: [] for d in self._drone_order}
        self._dstar: Dict[int, _DStarLite] = {}

        self._last_log_t: float = 0.0
        self._pending_obstacle_changes: List[Tuple[Tuple[int, int], int]] = []

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
        self._grid_pub = None
        self._grid_timer = None
        if _HAS_NAV_MSGS:
            self._grid_pub = self.create_publisher(
                OccupancyGrid, "/grid_map_nav", qos
            )
            self._grid_timer = self.create_timer(1.0, self._publish_default_grid)

        p_type_str = "CAR (UGV)" if self.platform_type == 1 else "DRONE (UAV)"
        self.get_logger().info(
            f"planner_node up: planner={self.planner_name}, platform={p_type_str}, "
            f"num_drones={self.num_drones}, grid={self.grid_size}x{self.grid_size}, "
            f"tick={self.tick_period}s, in={self.task_topic}+{self.grid_topic}, "
            f"out={self.drone_states_topic}+{self.planned_path_topic}"
        )

    # ----------------------------------------------------------------
    # 运动学约束处理 (Kinematic Constraints for UGV)
    # ----------------------------------------------------------------
    def _line_of_sight(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
        """Bresenham line algorithm to check if straight path between p1 and p2 is obstacle-free."""
        x0, y0 = p1
        x1, y1 = p2
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                if int(self._grid[y, x]) != 0:
                    return False
            else:
                return False
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True

    def _apply_ugv_kinematic_constraints(
        self, path: List[Tuple[int, int]]
    ) -> List[Tuple[int, int]]:
        """对无人车路径应用运动学约束：
        1. 消除悬停段（合并相邻重复坐标点）。
        2. 视线网格剪枝（Line-of-Sight Shortcutting），消除平移抖动。
        3. 限制转弯角度，符合转弯半径约束。
        """
        if len(path) <= 1:
            return path

        # Step 1: 消除悬停段（去重连续相同节点）
        dedup_path = [path[0]]
        for pt in path[1:]:
            if pt != dedup_path[-1]:
                dedup_path.append(pt)

        if len(dedup_path) <= 2:
            return dedup_path

        # Step 2: 视线快捷平滑 (Line-of-Sight shortcutting)
        shortcut_path = [dedup_path[0]]
        curr_idx = 0
        while curr_idx < len(dedup_path) - 1:
            next_idx = len(dedup_path) - 1
            while next_idx > curr_idx + 1:
                if self._line_of_sight(dedup_path[curr_idx], dedup_path[next_idx]):
                    break
                next_idx -= 1
            shortcut_path.append(dedup_path[next_idx])
            curr_idx = next_idx

        if len(shortcut_path) <= 2 or self.min_turning_radius <= 0.0:
            return shortcut_path

        # Step 3: 转向角度限制 (基于 min_turning_radius)
        smoothed = [shortcut_path[0]]
        max_allowable_turn_angle = (
            math.pi / 2.0 if self.min_turning_radius <= 1.0 else max(0.3, math.pi / self.min_turning_radius)
        )

        for i in range(1, len(shortcut_path) - 1):
            p_prev = smoothed[-1]
            p_curr = shortcut_path[i]
            p_next = shortcut_path[i + 1]

            v1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            v2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])

            len1 = math.hypot(v1[0], v1[1])
            len2 = math.hypot(v2[0], v2[1])

            if len1 == 0 or len2 == 0:
                continue

            dot = v1[0] * v2[0] + v1[1] * v2[1]
            cos_angle = max(-1.0, min(1.0, dot / (len1 * len2)))
            angle_diff = math.acos(cos_angle)

            # 如果弯折角度过大，进行圆角平滑插入中间过渡点
            if angle_diff > max_allowable_turn_angle:
                m1 = (int(round(0.5 * (p_prev[0] + p_curr[0]))), int(round(0.5 * (p_prev[1] + p_curr[1]))))
                m2 = (int(round(0.5 * (p_curr[0] + p_next[0]))), int(round(0.5 * (p_curr[1] + p_next[1]))))
                if m1 != smoothed[-1] and int(self._grid[m1[1], m1[0]]) == 0:
                    smoothed.append(m1)
                if m2 != smoothed[-1] and int(self._grid[m2[1], m2[0]]) == 0:
                    smoothed.append(m2)
            else:
                smoothed.append(p_curr)

        smoothed.append(shortcut_path[-1])

        # 再次确认无相邻重复点（保证无悬停段）
        final_path = [smoothed[0]]
        for pt in smoothed[1:]:
            if pt != final_path[-1]:
                final_path.append(pt)

        return final_path

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------
    def _publish_default_grid(self) -> None:
        if self._grid_pub is None or OccupancyGrid is None:
            return
        try:
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.info.width = 40
            msg.info.height = 40
            msg.info.resolution = 0.5
            msg.info.origin.position.x = 0.0
            msg.info.origin.position.y = 0.0
            msg.info.origin.position.z = 0.0
            msg.info.origin.orientation.w = 1.0
            msg.data = [0] * (40 * 40)
            self._grid_pub.publish(msg)
        except Exception as exc:  # pragma: no cover
            self.get_logger().debug(f"_publish_default_grid failed: {exc}")

    def _default_initial_position(self, idx: int) -> Tuple[float, float]:
        if self.num_drones <= 0:
            return (0.0, 0.0)
        side = max(1, int(np.ceil(np.sqrt(self.num_drones))))
        spacing = self.grid_size / max(side + 1, 1)
        row, col = divmod(idx, side)
        return (col * spacing + spacing, row * spacing + spacing)

    def _apply_obstacle_spec(self, spec) -> None:
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
                self._grid[y0 : y1 + 1, x0 : x1 + 1] = 1
            elif "x" in spec and "y" in spec:
                x = int(spec["x"]); y = int(spec["y"])
                self._grid[y, x] = 1
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"obstacle spec ignored: {spec!r} ({exc})")

    def _scatter_target(self, target_id: int) -> Tuple[int, int]:
        s = self.grid_size
        x = int(target_id) % s
        y = (int(target_id) // s) % s
        x = max(0, min(s - 1, x))
        y = max(0, min(s - 1, y))
        return (x, y)

    def _ensure_drone(self, did: int) -> None:
        if did not in self._drone_xy:
            mid = float(self.grid_size) / 2.0
            self._drone_xy[did] = (mid, mid)
            self._drone_path[did] = []
            if did not in self._drone_order:
                self._drone_order.append(did)
            self.get_logger().info(f"registered new drone id={did}")

    def _plan_for_drone(self, did: int, target_xy: Tuple[int, int]) -> None:
        self._ensure_drone(did)
        gx, gy = self._world_to_cell(self._drone_xy[did])
        tx, ty = int(target_xy[0]), int(target_xy[1])

        plan_start = time.monotonic()
        if self.planner_name == "dstar_lite" and did in self._dstar:
            self._dstar[did].start = (gx, gy)
            self._dstar[did].goal = (tx, ty)
            path = self._dstar[did].plan()
        else:
            if self.planner_name == "dstar_lite":
                planner = _DStarLite(
                    self._grid, (gx, gy), (tx, ty), diagonal=True
                )
                path = planner.plan()
                self._dstar[did] = planner
            else:
                path = _astar(self._grid, (gx, gy), (tx, ty), diagonal=True)

        # 当 platform_type == 1 (无人车) 时，执行运动学约束平滑处理
        if self.platform_type == 1 and path:
            path = self._apply_ugv_kinematic_constraints(path)

        self._drone_path[did] = path
        duration_ms = (time.monotonic() - plan_start) * 1000.0

        if not self._drone_path[did]:
            self.get_logger().warn(
                f"no path for drone {did} -> ({tx}, {ty}); goal may be unreachable."
            )

        self.get_logger().info(
            f"metric plan.duration_ms={duration_ms:.2f}, "
            f"drone={did}, path_len={len(self._drone_path[did])}, "
            f"platform={self.platform_type}, goal=({tx}, {ty})"
        )

    def _apply_pending_obstacle_changes(self) -> None:
        if not self._pending_obstacle_changes:
            return
        edits = self._pending_obstacle_changes
        self._pending_obstacle_changes = []
        for did, planner in self._dstar.items():
            planner.update_obstacles(edits)
        for did in self._drone_path.keys():
            if self.planner_name == "astar" and self._drone_path[did]:
                self._drone_path[did] = []
                self._replan_for_drone(did)

    def _replan_for_drone(self, did: int) -> None:
        target = self._drone_target.get(did)
        if target is None:
            return
        self._plan_for_drone(did, target)

    def _world_to_cell(self, xy: Tuple[float, float]) -> Tuple[int, int]:
        s = self.grid_size
        x = max(0, min(s - 1, int(round(xy[0]))))
        y = max(0, min(s - 1, int(round(xy[1]))))
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
                if found:
                    break
        return (x, y)

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------
    def on_task(self, msg: TaskAssignment) -> None:
        """Receive task assignment, plan path, and immediately trigger state broadcast."""
        did = int(msg.drone_id)
        tid = int(msg.target_id)
        if did in self._drone_target and did in self._explicit_target_set():
            target = self._drone_target[did]
        else:
            target = self._scatter_target(tid)
            self._drone_target[did] = target
        self._plan_for_drone(did, target)
        self.get_logger().debug(
            f"task: drone {did} -> target {tid} (cell {target})"
        )

        # 收到 TaskAssignment 后立即对外发布最新的非零真实 DroneStateArray
        self._publish_drone_states()

    def _explicit_target_set(self) -> set:
        return {
            did
            for did in self._drone_target
            if did in self._drone_xy
        }

    def on_grid(self, msg: UInt8MultiArray) -> None:
        info = msg.layout.dim if msg.layout is not None else []
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

        edits: List[Tuple[Tuple[int, int], int]] = []
        old_h, old_w = self._grid.shape

        if h != old_h or w != old_w:
            self._grid = new_grid
            self.grid_size = h
            for did, (x, y) in list(self._drone_xy.items()):
                nx = max(0.0, min(float(w - 1), x))
                ny = max(0.0, min(float(h - 1), y))
                self._drone_xy[did] = (nx, ny)
            self._dstar.clear()
            self._drone_path = {d: [] for d in self._drone_path}
            for did, target in self._drone_target.items():
                self._plan_for_drone(did, target)
            n_blocked = int((self._grid != 0).sum())
            self.get_logger().info(
                f"metric grid.obstacles={n_blocked}, "
                f"grid.changed_cells={h * w}, "
                f"grid.size={w}x{h} (resize)"
            )
            return

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

        n_blocked = int((self._grid != 0).sum())
        self.get_logger().info(
            f"metric grid.obstacles={n_blocked}, "
            f"grid.changed_cells={len(edits)}, "
            f"grid.size={w}x{h}"
        )

    def on_rfly_pose(self, msg: DroneStateArray) -> None:
        """Optional pose feedback from external simulator/hardware."""
        for d in msg.drones:
            did = int(d.drone_id)
            self._ensure_drone(did)
            self._drone_xy[did] = (float(d.x), float(d.y))

    def on_target_world(self, msg: TargetTrackArray) -> None:
        if not msg.tracks:
            return
        self.get_logger().debug(
            f"/target_track_world: {len(msg.tracks)} track(s) "
            f"frame={msg.header.frame_id if msg.header else 'none'}"
        )

    # ----------------------------------------------------------------
    # State Publishing
    # ----------------------------------------------------------------
    def _publish_drone_states(self) -> None:
        """构建并发布 DroneStateArray，包含真实的平台类型与位置坐标。"""
        msg = DroneStateArray()
        msg.num_drones = len(self._drone_order)
        msg.drones = []

        for did in self._drone_order:
            x, y = self._drone_xy[did]
            ds = DroneState()
            ds.drone_id = _uint32(did)
            ds.x = float(x)
            ds.y = float(y)

            # 设置平台类型与高程约束
            ds.platform_type = _uint8(self.platform_type)
            if self.platform_type == 1:
                # 无人车 (UGV): Z 轴严格锁死为 0.0
                ds.z = 0.0
            else:
                # 无人机 (UAV): 飞行高度
                ds.z = float(self.drone_z_default)

            path = self._drone_path.get(did, [])
            if len(path) >= 2:
                nx, ny = path[1]
                vx = float(nx - x)
                vy = float(ny - y)
                mag = (vx * vx + vy * vy) ** 0.5
                speed = min(self.sim_tick_speed, self.sim_tick_speed + self.max_speed_diff)
                if mag > 0:
                    vx = (vx / mag) * speed
                    vy = (vy / mag) * speed
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

    # ----------------------------------------------------------------
    # Timer Tick
    # ----------------------------------------------------------------
    def tick(self) -> None:
        """主循环：推进运动，更新非零实时坐标，并发布 DroneStateArray。"""
        if self._pending_obstacle_changes:
            self._apply_pending_obstacle_changes()

        occupied: Dict[Tuple[int, int], int] = {}
        for did in self._drone_order:
            path = self._drone_path.get(did, [])
            if not path:
                continue
            head = path[0]
            if head in occupied:
                continue
            occupied[head] = did

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
                    break
                occupied_cells.discard(path[0])
                occupied_cells.add(next_cell)
                path.pop(0)

            head = path[0]
            self._drone_xy[did] = (float(head[0]), float(head[1]))

        # 发布 /drone_states
        self._publish_drone_states()

        # 发布 nav_msgs/Path
        if self.publish_path and self.pub_path is not None:
            try:
                stamped_msg = self._build_nav_path(self._drone_path)
                if stamped_msg is not None:
                    self.pub_path.publish(stamped_msg)
            except Exception as exc:  # pragma: no cover
                self.get_logger().debug(f"path publish failed: {exc}")

        now = time.monotonic()
        if now - self._last_log_t >= self.log_interval:
            self._last_log_t = now
            n_paths = sum(1 for v in self._drone_path.values() if v)
            self.get_logger().info(
                f"planner summary: {n_paths}/{len(self._drone_order)} "
                f"drones active, platform_type={self.platform_type}, "
                f"pending grid edits = {len(self._pending_obstacle_changes)}"
            )

    # ----------------------------------------------------------------
    def _build_nav_path(self, paths):
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
                ps.pose.position.z = 0.0 if self.platform_type == 1 else self.drone_z_default
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
