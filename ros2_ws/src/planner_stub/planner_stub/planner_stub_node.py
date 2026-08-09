"""planner_stub_node — integration shim for the empty planning_pkg slot.

This node stands in for the path-planner that 程维好 will eventually
ship. It exists for one purpose: let the three-link integration test
run **today** without waiting for the real planner implementation.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from swarm_interfaces.msg import (
    DroneState,
    DroneStateArray,
    TaskAssignment,
    TargetTrack,
    TargetTrackArray,
)


def _step_towards(
    pos: Tuple[float, float],
    goal: Tuple[float, float],
    max_step: float,
) -> Tuple[float, float]:
    """Move ``pos`` toward ``goal`` by at most ``max_step`` meters."""
    dx = goal[0] - pos[0]
    dy = goal[1] - pos[1]
    dist = math.hypot(dx, dy)
    if dist <= 1e-6 or dist <= max_step:
        return goal
    scale = max_step / dist
    return (pos[0] + dx * scale, pos[1] + dy * scale)


class PlannerStubNode(Node):
    """Drone-state simulator that follows the latest ``TaskAssignment``."""

    LOG_TAG = "[planner_stub]"

    def __init__(self) -> None:
        super().__init__("planner_stub_node")

        # ----- parameters -----
        self.declare_parameter("num_drones", 8)
        self.declare_parameter("max_speed", 2.0)  # m/s
        self.declare_parameter("tick_period", 0.5)  # s
        self.declare_parameter("altitude", 5.0)  # m
        self.declare_parameter("min_sep", 3.0)  # m, drone-drone repulsion
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("seed_grid_spacing", 6.0)
        self.declare_parameter("assignment_topic", "/task_assignment")
        self.declare_parameter("target_topic", "/target_track")
        self.declare_parameter("drone_states_topic", "/drone_states")
        self.declare_parameter("drone_state_topic", "/drone_state")

        self.num_drones = int(self.get_parameter("num_drones").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.tick_period = float(self.get_parameter("tick_period").value)
        self.altitude = float(self.get_parameter("altitude").value)
        self.min_sep = float(self.get_parameter("min_sep").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        spacing = float(self.get_parameter("seed_grid_spacing").value)

        # ----- state -----
        side = max(1, math.ceil(math.sqrt(self.num_drones)))
        self._drones: Dict[int, Tuple[float, float, float, float]] = {}
        for i in range(self.num_drones):
            row, col = divmod(i, side)
            self._drones[i] = (
                (col + 1) * spacing,  # 非零初始坐标 (x)
                (row + 1) * spacing,  # 非零初始坐标 (y)
                0.0,
                0.0,  # vx, vy
            )

        self._targets: Dict[int, Tuple[float, float]] = {}
        self._assignment: Dict[int, int] = {}
        self._last_log_t = 0.0
        self._last_assign_event = ""

        # ----- QoS -----
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ----- subscriptions -----
        self.create_subscription(
            TaskAssignment,
            self.get_parameter("assignment_topic").value,
            self._on_assignment,
            qos,
        )
        self.create_subscription(
            TargetTrackArray,
            self.get_parameter("target_topic").value,
            self._on_target_track,
            qos,
        )

        # ----- publishers -----
        self._drone_states_pub = self.create_publisher(
            DroneStateArray,
            self.get_parameter("drone_states_topic").value,
            qos,
        )
        self._drone_state_pub = self.create_publisher(
            DroneState,
            self.get_parameter("drone_state_topic").value,
            qos,
        )

        # ----- timer -----
        self._timer = self.create_timer(self.tick_period, self._tick)
        self.get_logger().info(
            f"{self.LOG_TAG} ready: num_drones={self.num_drones} "
            f"max_speed={self.max_speed:.1f}m/s tick={self.tick_period:.2f}s "
            f"frame_id={self.frame_id}"
        )

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------
    def _on_assignment(self, msg: TaskAssignment) -> None:
        self._assignment[int(msg.drone_id)] = int(msg.target_id)
        if self._last_assign_event != (msg.drone_id, msg.target_id, msg.task_type):
            self._last_assign_event = (msg.drone_id, msg.target_id, msg.task_type)
            self.get_logger().info(
                f"{self.LOG_TAG} 收到 TaskAssignment: "
                f"drone_id={msg.drone_id} target_id={msg.target_id} "
                f"task_type='{msg.task_type}'"
            )

    def _on_target_track(self, msg: TargetTrackArray) -> None:
        new_targets: Dict[int, Tuple[float, float]] = {}
        for t in msg.tracks:
            new_targets[int(t.target_id)] = (float(t.x), float(t.y))
        self._targets = new_targets

    # ----------------------------------------------------------------
    # Tick
    # ----------------------------------------------------------------
    def _tick(self) -> None:
        dt = self.tick_period
        now = time.monotonic()

        # 1. move each drone toward its assigned target (if any)
        next_positions: Dict[int, Tuple[float, float, float, float]] = {}
        for did, (x, y, _vx, _vy) in self._drones.items():
            tgt_id = self._assignment.get(did)
            goal = self._targets.get(tgt_id) if tgt_id is not None else None
            if goal is None:
                next_positions[did] = (x, y, 0.0, 0.0)
                continue
            new_x, new_y = _step_towards((x, y), goal, self.max_speed * dt)
            next_positions[did] = (
                new_x,
                new_y,
                (new_x - x) / dt,
                (new_y - y) / dt,
            )

        # 2. pairwise repulsion
        ids = sorted(next_positions.keys())
        for i, ida in enumerate(ids):
            xa, ya, vxa, vya = next_positions[ida]
            for idb in ids[i + 1 :]:
                xb, yb, vxb, vyb = next_positions[idb]
                dx = xa - xb
                dy = ya - yb
                dist = math.hypot(dx, dy)
                if dist >= self.min_sep or dist < 1e-6:
                    continue
                push = (self.min_sep - dist) * 0.5
                nx, ny = dx / dist, dy / dist
                xa += nx * push
                ya += ny * push
                xb -= nx * push
                yb -= ny * push
                next_positions[ida] = (xa, ya, vxa, vya)
                next_positions[idb] = (xb, yb, vxb, vyb)

        self._drones = next_positions

        # 3. publish
        arr = DroneStateArray()
        for did in ids:
            x, y, vx, vy = self._drones[did]
            st = DroneState()
            st.drone_id = int(did)
            st.x = float(x)
            st.y = float(y)
            st.z = float(self.altitude)
            st.vx = float(vx)
            st.vy = float(vy)
            st.vz = 0.0
            st.available = True
            
            # 兼容新增的 platform_type 字段
            if hasattr(st, 'platform_type'):
                st.platform_type = getattr(DroneState, 'PLATFORM_DRONE', 0)

            arr.drones.append(st)

            # Per-drone topic
            self._drone_state_pub.publish(st)

        self._drone_states_pub.publish(arr)

        if now - self._last_log_t >= 5.0:
            self._last_log_t = now
            self.get_logger().info(
                f"{self.LOG_TAG} drone_states: n={len(ids)} "
                f"n_assigned={len(self._assignment)} n_targets={len(self._targets)}"
            )


def main(args: Optional[List[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    node = PlannerStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
