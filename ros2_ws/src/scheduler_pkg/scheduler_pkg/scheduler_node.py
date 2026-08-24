#!/usr/bin/env python3
from __future__ import annotations
import time
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple
import numpy as np
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from swarm_interfaces.msg import DroneStateArray, TargetTrackArray, TaskAssignment
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False
    rclpy = None
    Node = object
    ExternalShutdownException = Exception
    QoSProfile = object
    QoSReliabilityPolicy = object
    DroneStateArray = object
    TargetTrackArray = object
    TaskAssignment = object

from .assign import greedy_assign, hungarian_assign
from .auction_engine import AuctionEngine
from .agent import Agent
from .task import Task


def uint32(x: int) -> int:
    return int(x) & 0xFFFFFFFF


def normalize_strategy(strategy: str) -> str:
    if strategy in ("greedy", "hungarian", "auction"):
        return strategy
    return "greedy"


def target_priority(track) -> float:
    priority = float(np.clip(track.confidence, 0.0, 1.0))
    if bool(track.is_confirmed):
        priority = min(1.0, priority + 0.1)
    return priority


def parse_targets(msg: TargetTrackArray) -> Dict[int, Tuple[float, float, float]]:
    return {
        int(track.target_id): (
            float(track.x),
            float(track.y),
            target_priority(track),
        )
        for track in msg.tracks
    }


def parse_drones(msg: DroneStateArray) -> Dict[int, Tuple[float, float, int]]:
    """Parse DroneStateArray into a dict keyed by ``drone_id``.

    Returns ``(x, y, platform_type)`` per drone, where ``platform_type``
    follows the ``DroneState.msg`` convention:
      * ``0`` = aerial UAV (faster, default ``speed=2.0``)
      * ``1`` = ground UGV (slower, default ``speed=1.5``)

    Unavailable drones are filtered out.
    """
    result: Dict[int, Tuple[float, float, int]] = {}
    for drone in msg.drones:
        if not bool(drone.available):
            continue
        # ``platform_type`` was added to ``DroneState.msg`` for W9 P1-F;
        # fall back to 0 (UAV) when running against older message stubs
        # that don't carry the field (e.g. legacy unit tests).
        plat = int(getattr(drone, "platform_type", 0))
        result[int(drone.drone_id)] = (
            float(drone.x),
            float(drone.y),
            plat,
        )
    return result


class SchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__("scheduler_node")

        # --- parameters ---
        self.declare_parameter("num_drones", 8)
        self.declare_parameter("assignment_strategy", "greedy")
        self.declare_parameter("max_per_drone", 2)
        self.declare_parameter("tick_period", 0.5)
        self.declare_parameter("log_interval_sec", 5.0)
        self.declare_parameter("target_topic", "/target_track")
        self.declare_parameter("drone_topic", "/drone_states")
        self.declare_parameter("output_topic", "/task_assignment")
        self.declare_parameter("default_task_type", "track")

        self.num_drones = int(self.get_parameter("num_drones").value)
        self.strategy = normalize_strategy(str(self.get_parameter("assignment_strategy").value))
        self.max_per_drone = int(self.get_parameter("max_per_drone").value)
        self.tick_period = float(self.get_parameter("tick_period").value)
        self.log_interval = float(self.get_parameter("log_interval_sec").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.drone_topic = str(self.get_parameter("drone_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.default_task_type = str(self.get_parameter("default_task_type").value)

        # --- state caches ---
        self._targets: Dict[int, Tuple[float, float, float]] = {}
        # value is (x, y, platform_type); see parse_drones for the encoding
        self._drones: Dict[int, Tuple[float, float, int]] = {}

        # --- QoS ---
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # --- subscriptions ---
        self.sub_target = self.create_subscription(
            TargetTrackArray,
            self.target_topic,
            self.on_target,
            qos
        )
        self.sub_drone = self.create_subscription(
            DroneStateArray,
            self.drone_topic,
            self.on_drone,
            qos
        )

        # --- publisher ---
        self.pub_task = self.create_publisher(TaskAssignment, self.output_topic, 10)

        # --- timer ---
        self.timer = self.create_timer(self.tick_period, self.tick)

        self._last_log_t: float = 0.0
        self._last_assign_count: int = 0
        self._last_tick_wallclock: float = time.monotonic()
        self._empty_target_logged: bool = False
        # Latch so the 异构任务分发 banner only fires on the first tick that
        # actually dispatches a heterogeneous package.
        self._heterogeneous_log_emitted: bool = False

        self.get_logger().info(
            f"scheduler_node up: strategy={self.strategy}, "
            f"num_drones={self.num_drones}, max_per_drone={self.max_per_drone}, "
            f"tick={self.tick_period}s, "
            f"in={self.target_topic}+{self.drone_topic}, "
            f"out={self.output_topic}"
        )

    def on_target(self, msg: TargetTrackArray) -> None:
        self._targets = parse_targets(msg)
        self._empty_target_logged = False

    def on_drone(self, msg: DroneStateArray) -> None:
        self._drones = parse_drones(msg)

    def _seed_default_drones(self) -> None:
        # Default-seeded fleet is UAV-only (platform_type=0) so demo runs
        # continue to behave exactly as before W9 P1-F.
        n = max(1, self.num_drones)
        side = int(np.ceil(np.sqrt(n)))
        spacing = 5.0
        for i in range(n):
            row, col = divmod(i, side)
            self._drones[i] = (col * spacing, row * spacing, 0)

    def _maybe_log_summary(self, now: float, n_pairs: int, period: float) -> None:
        if now - self._last_log_t >= self.log_interval:
            self._last_log_t = now
            self.get_logger().info(
                f"scheduler summary: {n_pairs} assignments, "
                f"{len(self._targets)} active targets, "
                f"{len(self._drones)} active drones, "
                f"tick={period * 1000:.0f} ms"
            )
            self.get_logger().info(
                f"metric assignments.active={n_pairs}, "
                f"targets.active={len(self._targets)}, "
                f"drones.active={len(self._drones)}, "
                f"latency.ms={period * 1000:.0f}"
            )

    def _run_greedy_assign(self, drone_ids, drone_xy, target_ids, target_xy, target_priorities):
        return greedy_assign(
            drone_xy,
            target_xy,
            target_priorities,
            max_per_drone=self.max_per_drone,
        )

    # --- W9 P1-F: heterogeneous agent / task-type support ---
    # platform_type -> Agent(category, default speed). 0=UAV, 1=UGV.
    _PLATFORM_AGENT_PROFILE: Dict[int, Tuple[str, float]] = {
        0: ("UAV", 2.0),
        1: ("UGV", 1.5),
    }
    # platform_type -> heterogeneous task_type suffix. Anything outside
    # this map (or future platform codes) falls back to ``default_task_type``.
    _PLATFORM_TASK_TYPE: Dict[int, str] = {
        0: "track_aerial",
        1: "track_ground",
    }

    def _run_auction_assign(self, drone_ids, drone_xy, target_ids, target_xy, target_priorities):
        tasks = []
        for t_id, (x, y, priority) in self._targets.items():
            task = Task(
                tid=f"T{t_id:03d}",
                pos=[x, y],
                reward=50,
                priority=int(priority * 5),
                release_time=0,
                deadline=60,
                service_time=10
            )
            tasks.append(task)

        agents = []
        # Use the same sorted order as ``drone_ids`` in ``tick``.  ROS
        # messages need not arrive in ID order, so dictionary insertion
        # order is not a safe mapping from auction-agent index to drone ID.
        for d_id in drone_ids:
            x, y, platform_type = self._drones[d_id]
            category, speed = self._PLATFORM_AGENT_PROFILE.get(
                platform_type, ("UAV", 2.0)
            )
            agent = Agent(
                aid=f"{category}{d_id}",
                category=category,
                pos=[x, y],
                battery=100,
                max_load=self.max_per_drone,
                unit_cost=1.0,
                speed=speed,
            )
            agents.append(agent)

        engine = AuctionEngine(agents, tasks)
        result = engine.bid_allocation()

        pairs = []
        for task_id, agent_id in result.items():
            target_id = int(task_id[1:])
            drone_id = int(agent_id[3:])
            if target_id in target_ids and drone_id in drone_ids:
                d_idx = drone_ids.index(drone_id)
                t_idx = target_ids.index(target_id)
                pairs.append((d_idx, t_idx))

        return pairs

    def tick(self) -> None:
        now = time.monotonic()
        period = max(self.tick_period, 1e-3)
        self._last_tick_wallclock = now

        if not self._targets:
            if not self._empty_target_logged:
                self.get_logger().info("scheduler_node: no targets received yet, waiting...")
                self._empty_target_logged = True
            self._maybe_log_summary(now, 0, period)
            return

        self._empty_target_logged = False

        if not self._drones:
            self._seed_default_drones()

        drone_ids = sorted(self._drones.keys())[:self.num_drones]
        # ``self._drones`` now carries ``(x, y, platform_type)``; keep the
        # downstream numpy contract as a pure (N, 2) XY matrix.
        drone_xy = np.array(
            [(self._drones[d][0], self._drones[d][1]) for d in drone_ids],
            dtype=float,
        )
        drone_platforms = [int(self._drones[d][2]) for d in drone_ids]
        target_ids = sorted(self._targets.keys())
        target_xy = np.array([self._targets[t][:2] for t in target_ids], dtype=float)
        target_priorities = np.array([self._targets[t][2] for t in target_ids], dtype=float)

        if self.strategy == "auction":
            try:
                pairs = self._run_auction_assign(
                    drone_ids, drone_xy, target_ids, target_xy, target_priorities
                )
            except Exception as e:
                self.get_logger().warn(f"auction_assign failed ({e}); falling back to greedy.")
                pairs = self._run_greedy_assign(drone_ids, drone_xy, target_ids, target_xy, target_priorities)
        elif self.strategy == "hungarian":
            try:
                from .assign import hungarian_assign
                pairs = hungarian_assign(
                    drone_xy,
                    target_xy,
                    target_priorities,
                    max_per_drone=self.max_per_drone,
                )
            except RuntimeError as e:
                self.get_logger().warn(f"hungarian_assign failed ({e}); falling back to greedy.")
                pairs = self._run_greedy_assign(drone_ids, drone_xy, target_ids, target_xy, target_priorities)
        else:
            pairs = self._run_greedy_assign(drone_ids, drone_xy, target_ids, target_xy, target_priorities)

        # W9 P1-F: pick per-platform task_type so UAVs receive "track_aerial"
        # and UGVs receive "track_ground". Drones with an unknown platform
        # code (or the legacy UAV-only fleet) still get ``default_task_type``.
        task_type_by_platform = {
            int(plat): self._PLATFORM_TASK_TYPE.get(int(plat), self.default_task_type)
            for plat in set(drone_platforms)
        }
        uav_count = sum(1 for p in drone_platforms if p == 0)
        ugv_count = sum(1 for p in drone_platforms if p == 1)
        is_heterogeneous = ugv_count > 0 and uav_count > 0
        if is_heterogeneous and not self._heterogeneous_log_emitted:
            self.get_logger().info(
                f"异构任务分发: UAV={uav_count}, UGV={ugv_count}"
            )
            self._heterogeneous_log_emitted = True

        for d_idx, t_idx in pairs:
            plat = drone_platforms[d_idx] if d_idx < len(drone_platforms) else 0
            msg = TaskAssignment()
            msg.drone_id = uint32(drone_ids[d_idx])
            msg.target_id = uint32(target_ids[t_idx])
            msg.task_type = task_type_by_platform.get(plat, self.default_task_type)
            self.pub_task.publish(msg)

        self._last_assign_count = len(pairs)
        self._maybe_log_summary(now, len(pairs), period)


def main(args: Optional[List[str]] = None) -> None:
    if not _HAS_ROS:
        print("ERROR: rclpy not available. Please source ROS2 setup.bash first.")
        return
    rclpy.init(args=args)
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
