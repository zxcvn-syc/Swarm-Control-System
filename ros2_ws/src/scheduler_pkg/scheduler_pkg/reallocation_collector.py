#!/usr/bin/env python3
"""Reallocation timing collector (Ma Ziyue, 2026-08-26).

Subscribes to ``/enclosure_targets`` (swarm_interfaces/EnclosureTargetArray,
published by escape_test_node during the 8.27 three-scene enclosure tests).
Every escape-target update triggers one AuctionEngine reallocation; the
per-event wall-clock cost is logged and appended to a CSV so the report can
quote "reallocation latency" numbers.

Usage (after ``colcon build --packages-select scheduler_pkg``)::

    ros2 run scheduler_pkg reallocation_collector \
        --ros-args -p log_dir:=~/ros2_ws/logs/reallocation

Notes
-----
* Platform set is fixed (2 UAV + 3 UGV) on purpose: this node measures the
  *algorithm-level* reallocation latency triggered by real escape events,
  consistent with the bench_auction methodology.
* CSV writing is best-effort: any failure only logs a warning and never
  blocks the timing collection.
"""
import csv
import os
import time

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import EnclosureTargetArray

from .agent import Agent
from .auction_engine import AuctionEngine
from .task import Task


class ReallocationCollector(Node):
    def __init__(self):
        super().__init__("reallocation_collector")

        # --- best-effort CSV event log --------------------------------
        self.declare_parameter("log_dir", "~/ros2_ws/logs/reallocation")
        self._csv_path = None
        try:
            log_dir = os.path.expanduser(
                str(self.get_parameter("log_dir").value))
            os.makedirs(log_dir, exist_ok=True)
            self._csv_path = os.path.join(
                log_dir,
                "reallocation_events_{}.csv".format(
                    time.strftime("%Y%m%d_%H%M%S")))
            with open(self._csv_path, "w", newline="",
                      encoding="utf-8") as f:
                csv.writer(f).writerow(
                    ["wall_time", "ros_time", "frame_idx",
                     "num_targets", "elapsed_ms", "allocation"])
        except Exception as exc:  # never block collection on CSV issues
            self.get_logger().warn(f"CSV event log disabled: {exc}")

        # --- subscription ---------------------------------------------
        self.subscription = self.create_subscription(
            EnclosureTargetArray,
            "/enclosure_targets",
            self.target_callback,
            10,
        )

        # --- fixed platform set (algorithm-level benchmark) ------------
        self.agents = [
            Agent("UAV0", "UAV", [0, 5], 100, 3, 1.0, 2.0),
            Agent("UAV1", "UAV", [10, 5], 100, 3, 1.0, 2.0),
            Agent("UGV0", "UGV", [20, 5], 80, 3, 0.6, 1.0),
            Agent("UGV1", "UGV", [30, 5], 80, 3, 0.6, 1.0),
            Agent("UGV2", "UGV", [40, 5], 80, 3, 0.6, 1.0),
        ]
        self.get_logger().info(
            "reallocation collector up: subscribed /enclosure_targets")

    def target_callback(self, msg: EnclosureTargetArray):
        tracks = msg.targets
        if not tracks:
            return
        try:
            start = time.perf_counter()

            tasks = [
                Task(
                    tid=f"T{t.target_id:03d}",
                    pos=[t.x, t.y],
                    reward=50,
                    priority=5,
                    release_time=0,
                    deadline=60,
                    service_time=10,
                )
                for t in tracks
            ]
            for ag in self.agents:
                ag.task_list = []
                ag.current_time = 0.0
                ag.battery = 100.0

            result = AuctionEngine(self.agents, tasks).bid_allocation()
            elapsed_ms = (time.perf_counter() - start) * 1000.0

            self.get_logger().info(
                f"[REALLOC] frame={msg.frame_idx} "
                f"targets={len(tracks)} "
                f"elapsed_ms={elapsed_ms:.3f} alloc={result}")

            if self._csv_path:
                try:
                    ros_time = self.get_clock().now().nanoseconds / 1e9
                    with open(self._csv_path, "a", newline="",
                              encoding="utf-8") as f:
                        csv.writer(f).writerow([
                            time.strftime("%Y-%m-%d %H:%M:%S"),
                            f"{ros_time:.3f}",
                            msg.frame_idx,
                            len(tracks),
                            f"{elapsed_ms:.3f}",
                            str(result),
                        ])
                except Exception as exc:
                    self.get_logger().warn(f"CSV append failed: {exc}")
        except Exception as exc:
            self.get_logger().error(f"reallocation handling failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = ReallocationCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
