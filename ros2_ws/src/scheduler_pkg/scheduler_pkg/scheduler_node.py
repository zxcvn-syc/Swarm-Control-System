"""scheduler_node: ROS2 entry point that runs target -> drone assignment.

Subscribes:
    /target_track    swarm_interfaces/TargetTrackArray   (from perception_pkg)
    /drone_states    swarm_interfaces/DroneStateArray    (from simulator/state)

Publishes:
    /task_assignment swarm_interfaces/TaskAssignment     (one per drone target)

The actual assignment algorithm lives in :mod:`scheduler_pkg.assign`. This
node is intentionally thin: it caches the latest target / drone snapshots
and re-runs assignment every ``~tick_period`` seconds. If no targets have
arrived yet, ``tick`` simply logs once and returns.
"""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

import numpy as np

try:  # Keep pure parsing and unit tests usable without a sourced ROS2 setup.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from swarm_interfaces.msg import DroneStateArray, TargetTrackArray, TaskAssignment
    _HAS_ROS = True
except ImportError:  # pragma: no cover - exercised in non-ROS test environments
    _HAS_ROS = False

    class _FallbackLogger:
        def info(self, *args, **kwargs):
            return None

        def warn(self, *args, **kwargs):
            return None

    class Node:  # type: ignore[no-redef]
        def __init__(self, name: str) -> None:
            self._parameters = {}
            self._logger = _FallbackLogger()

        def declare_parameter(self, name, value):
            self._parameters.setdefault(name, value)

        def get_parameter(self, name):
            return SimpleNamespace(value=self._parameters[name])

        def get_logger(self):
            return self._logger

        def create_subscription(self, *args, **kwargs):
            return SimpleNamespace()

        def create_publisher(self, *args, **kwargs):
            return SimpleNamespace(publish=lambda msg: None)

        def create_timer(self, *args, **kwargs):
            return SimpleNamespace()

    class QoSProfile:
        def __init__(self, depth=10):
            self.depth = depth
            self.reliability = None

    class QoSReliabilityPolicy:
        RELIABLE = 1

    class TaskAssignment:  # type: ignore[no-redef]
        def __init__(self):
            self.drone_id = 0
            self.target_id = 0
            self.task_type = ""

    DroneStateArray = object  # type: ignore[assignment,misc]
    TargetTrackArray = object  # type: ignore[assignment,misc]
    rclpy = None  # type: ignore[assignment]

from .assign import greedy_assign, hungarian_assign


def uint32(x: int) -> int:
    """Coerce int to uint32 range so ROS2 does not raise on assignment."""
    return int(x) & 0xFFFFFFFF


def normalize_strategy(strategy: str) -> str:
    """Accept supported strategies and default unknown values to greedy."""
    return strategy if strategy in ("greedy", "hungarian") else "greedy"


def target_priority(track) -> float:
    """Convert a track confidence/confirmation pair into assignment priority."""
    priority = float(np.clip(track.confidence, 0.0, 1.0))
    if bool(track.is_confirmed):
        priority = min(1.0, priority + 0.1)
    return priority


def parse_targets(msg: TargetTrackArray) -> Dict[int, Tuple[float, float, float]]:
    """Return the latest target snapshot keyed by target ID."""
    return {
        int(track.target_id): (
            float(track.x),
            float(track.y),
            target_priority(track),
        )
        for track in msg.tracks
    }


def parse_drones(msg: DroneStateArray) -> Dict[int, Tuple[float, float]]:
    """Return available drones from the latest state snapshot."""
    return {
        int(drone.drone_id): (float(drone.x), float(drone.y))
        for drone in msg.drones
        if bool(drone.available)
    }


class SchedulerNode(Node):
    """Minimal but production-shape scheduler node."""

    def __init__(self) -> None:
        super().__init__("scheduler_node")

        # ----- parameters -----
        self.declare_parameter("num_drones", 8)
        self.declare_parameter("assignment_strategy", "greedy")  # greedy | hungarian
        self.declare_parameter("max_per_drone", 2)
        self.declare_parameter("tick_period", 0.5)
        self.declare_parameter("log_interval_sec", 5.0)
        self.declare_parameter("target_topic", "/target_track")
        self.declare_parameter("drone_topic", "/drone_states")
        self.declare_parameter("output_topic", "/task_assignment")
        self.declare_parameter("default_task_type", "track")

        self.num_drones: int = int(self.get_parameter("num_drones").value)
        self.strategy: str = str(self.get_parameter("assignment_strategy").value)
        self.max_per_drone: int = int(self.get_parameter("max_per_drone").value)
        self.tick_period: float = float(self.get_parameter("tick_period").value)
        self.log_interval: float = float(self.get_parameter("log_interval_sec").value)
        self.target_topic: str = str(self.get_parameter("target_topic").value)
        self.drone_topic: str = str(self.get_parameter("drone_topic").value)
        self.output_topic: str = str(self.get_parameter("output_topic").value)
        self.default_task_type: str = str(
            self.get_parameter("default_task_type").value
        )

        configured_strategy = self.strategy
        self.strategy = normalize_strategy(configured_strategy)
        if self.strategy != configured_strategy:
            self.get_logger().warn(
                f"Unknown strategy '{configured_strategy}', falling back to greedy."
            )

        # ----- state caches -----
        # target_id -> (x, y, priority). We keep the last-seen position per
        # target; priorities come from the speed/motion_mode heuristic.
        self._targets: Dict[int, Tuple[float, float, float]] = {}
        # drone_id -> (x, y)
        self._drones: Dict[int, Tuple[float, float]] = {}

        # ----- QoS: perception group publishes with RELIABLE in our tests,
        # but we keep the default profile to stay compatible.
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # ----- subscriptions -----
        self.sub_target = self.create_subscription(
            TargetTrackArray, self.target_topic, self.on_target, qos
        )
        self.sub_drone = self.create_subscription(
            DroneStateArray, self.drone_topic, self.on_drone, qos
        )

        # ----- publisher -----
        self.pub_task = self.create_publisher(TaskAssignment, self.output_topic, 10)

        # ----- timer -----
        self.timer = self.create_timer(self.tick_period, self.tick)
        self._last_log_t: float = 0.0
        self._last_assign_count: int = 0
        self._last_tick_wallclock: float = time.monotonic()
        self._empty_target_logged: bool = False

        self.get_logger().info(
            f"scheduler_node up: strategy={self.strategy}, "
            f"num_drones={self.num_drones}, max_per_drone={self.max_per_drone}, "
            f"tick={self.tick_period}s, "
            f"in={self.target_topic}+{self.drone_topic}, "
            f"out={self.output_topic}"
        )

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------
    def on_target(self, msg: TargetTrackArray) -> None:
        self._targets = parse_targets(msg)
        self._empty_target_logged = False

    def on_drone(self, msg: DroneStateArray) -> None:
        self._drones = parse_drones(msg)

    # ----------------------------------------------------------------
    # Tick
    # ----------------------------------------------------------------
    def tick(self) -> None:
        now = time.monotonic()
        period = max(self.tick_period, 1e-3)
        actual = now - self._last_tick_wallclock
        self._last_tick_wallclock = now

        # If no targets yet, log once and bail. Otherwise we'd starve the
        # logger with repeated "no targets" spam.
        if not self._targets:
            if not self._empty_target_logged:
                self.get_logger().info(
                    "scheduler_node: no targets received yet, waiting..."
                )
                self._empty_target_logged = True
            self._maybe_log_summary(now, 0, actual)
            return
        self._empty_target_logged = False

        # If we have no drone positions yet, synthesize a grid using
        # num_drones so the algorithm still has something to chew on. This
        # keeps the demo runnable before a real /drone_states publisher is
        # attached.
        if not self._drones:
            self._seed_default_drones()

        drone_ids = sorted(self._drones.keys())[: self.num_drones]
        drone_xy = np.array([self._drones[d] for d in drone_ids], dtype=float)
        target_ids = sorted(self._targets.keys())
        target_xy = np.array(
            [self._targets[t][:2] for t in target_ids], dtype=float
        )
        target_priorities = np.array(
            [self._targets[t][2] for t in target_ids], dtype=float
        )

        if self.strategy == "hungarian":
            try:
                pairs = hungarian_assign(
                    drone_xy,
                    target_xy,
                    target_priorities,
                    max_per_drone=self.max_per_drone,
                )
            except RuntimeError as e:
                self.get_logger().warn(
                    f"hungarian_assign failed ({e}); falling back to greedy."
                )
                pairs = greedy_assign(
                    drone_xy,
                    target_xy,
                    target_priorities,
                    max_per_drone=self.max_per_drone,
                )
        else:
            pairs = greedy_assign(
                drone_xy,
                target_xy,
                target_priorities,
                max_per_drone=self.max_per_drone,
            )

        # Publish one TaskAssignment per (drone_idx, target_idx) pair.
        task_type = self.default_task_type
        for d_idx, t_idx in pairs:
            msg = TaskAssignment()
            msg.drone_id = uint32(drone_ids[d_idx])
            msg.target_id = uint32(target_ids[t_idx])
            msg.task_type = task_type
            self.pub_task.publish(msg)

        self._last_assign_count = len(pairs)
        self._maybe_log_summary(now, len(pairs), actual)

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------
    def _seed_default_drones(self) -> None:
        """Make a deterministic grid of ``num_drones`` drones for demo runs."""
        n = max(1, self.num_drones)
        side = int(np.ceil(np.sqrt(n)))
        spacing = 5.0
        for i in range(n):
            row, col = divmod(i, side)
            self._drones[i] = (col * spacing, row * spacing)

    def _maybe_log_summary(self, now: float, n_pairs: int, period: float) -> None:
        if now - self._last_log_t >= self.log_interval:
            self._last_log_t = now
            self.get_logger().info(
                f"scheduler summary: {n_pairs} assignments, "
                f"{len(self._targets)} active targets, "
                f"{len(self._drones)} active drones, "
                f"tick={period * 1000:.0f} ms"
            )
            # Observable per-tick metric line in Prometheus-ish form:
            # ``metric assignments.active=A, targets.active=B, latency.ms=L``.
            self.get_logger().info(
                f"metric assignments.active={n_pairs}, "
                f"targets.active={len(self._targets)}, "
                f"drones.active={len(self._drones)}, "
                f"latency.ms={period * 1000:.0f}"
            )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
