#!/usr/bin/env python3
"""Drive and record the perception-to-containment process integration replay.

The script intentionally does not launch PX4, MAVROS, or an offboard bridge.
Start it through ``scripts/run_closed_loop_process_demo.sh`` so the real
scheduler, grid map, planner, and enclosure nodes are running first.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header, MultiArrayDimension, UInt8MultiArray
from swarm_interfaces.msg import (
    DroneStateArray,
    EnclosureCommandArray,
    EnclosureTarget,
    EnclosureTargetArray,
    TaskAssignment,
    TargetTrack,
    TargetTrackArray,
)


WARMUP_SECONDS = 1.5
OBSTACLE_COLUMNS = range(21, 27)
OBSTACLE_ROWS = range(12, 29)


class ClosedLoopDriver(Node):
    def __init__(self) -> None:
        super().__init__("closed_loop_demo_driver")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._track_pub = self.create_publisher(TargetTrackArray, "/target_track", qos)
        self._world_track_pub = self.create_publisher(
            TargetTrackArray, "/target_track_world", qos
        )
        self._enclosure_target_pub = self.create_publisher(
            EnclosureTargetArray, "/enclosure_targets", qos
        )
        self._obstacle_pub = self.create_publisher(
            UInt8MultiArray, "/grid_obstacles", qos
        )
        self.create_subscription(TaskAssignment, "/task_assignment", self._on_task, qos)
        self.create_subscription(DroneStateArray, "/drone_states", self._on_drones, qos)
        self.create_subscription(NavPath, "/planned_path", self._on_path, qos)
        self.create_subscription(
            EnclosureCommandArray, "/enclosure_command", self._on_enclosure, qos
        )
        self._started = time.monotonic()
        self._counts = {
            "target_track_published": 0,
            "target_track_world_published": 0,
            "enclosure_targets_published": 0,
            "task_assignment": 0,
            "drone_states": 0,
            "planned_path": 0,
            "enclosure_command": 0,
        }
        self._samples: dict[str, Any] = {}
        self._path_max_poses = 0
        self._obstacle_detour_observed = False
        self._obstacle_detour_path: list[dict[str, float]] = []
        self._timer = self.create_timer(0.2, self._publish)

    @staticmethod
    def _targets(elapsed: float) -> list[tuple[int, float, float, float, int, float, float]]:
        return [
            (101, 30.0 + 0.05 * elapsed, 20.0, 0.92, 2, 0.05, 0.0),
            (202, 30.0, 5.0 + 0.03 * elapsed, 0.86, 2, 0.0, 0.03),
        ]

    def _publish(self) -> None:
        elapsed = time.monotonic() - self._started
        self._obstacle_pub.publish(self._obstacle_mask())
        if elapsed < WARMUP_SECONDS:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"
        targets = self._targets(elapsed)

        tracks = TargetTrackArray()
        tracks.header = header
        tracks.frame_idx = self._counts["target_track_published"]
        world_tracks = TargetTrackArray()
        world_tracks.header = header
        world_tracks.frame_idx = tracks.frame_idx
        enclosure = EnclosureTargetArray()
        enclosure.header = header
        enclosure.frame_idx = tracks.frame_idx
        enclosure.num_drones = 4
        enclosure.enclosure_radius = 12.0
        enclosure.min_enclosure_dist = 4.0

        for target_id, x, y, confidence, motion_mode, vx, vy in targets:
            track = TargetTrack()
            track.target_id = target_id
            track.x, track.y, track.vx, track.vy = x, y, vx, vy
            track.confidence = confidence
            track.cls = 0
            track.is_confirmed = True
            track.speed = math.hypot(vx, vy)
            track.motion_mode = motion_mode
            track.pred_x = [x + vx * step for step in range(1, 6)]
            track.pred_y = [y + vy * step for step in range(1, 6)]
            track.pred_conf = [confidence - 0.03 * step for step in range(5)]
            tracks.tracks.append(track)
            world_tracks.tracks.append(track)

            item = EnclosureTarget()
            item.target_id = target_id
            item.x, item.y = x, y
            item.speed = track.speed
            item.motion_mode = motion_mode
            item.confidence = confidence
            item.box_x1, item.box_y1 = x - 1.0, y - 1.0
            item.box_x2, item.box_y2 = x + 1.0, y + 1.0
            item.pred_x, item.pred_y = track.pred_x, track.pred_y
            item.history_x, item.history_y = [x] * 10, [y] * 10
            enclosure.targets.append(item)

        self._track_pub.publish(tracks)
        self._world_track_pub.publish(world_tracks)
        self._enclosure_target_pub.publish(enclosure)
        self._counts["target_track_published"] += 1
        self._counts["target_track_world_published"] += 1
        self._counts["enclosure_targets_published"] += 1

    @staticmethod
    def _obstacle_mask() -> UInt8MultiArray:
        grid = UInt8MultiArray()
        grid.layout.dim = [
            MultiArrayDimension(label="height", size=40, stride=1600),
            MultiArrayDimension(label="width", size=40, stride=40),
        ]
        grid.data = [0] * 1600
        for row in range(12, 29):
            for column in range(21, 27):
                grid.data[row * 40 + column] = 1
        return grid

    def _on_task(self, message: TaskAssignment) -> None:
        self._counts["task_assignment"] += 1
        self._samples["task_assignment"] = {
            "drone_id": message.drone_id,
            "target_id": message.target_id,
            "task_type": message.task_type,
        }

    def _on_drones(self, message: DroneStateArray) -> None:
        self._counts["drone_states"] += 1
        self._samples["drone_states"] = [
            {
                "drone_id": item.drone_id,
                "x": round(item.x, 2),
                "y": round(item.y, 2),
                "available": item.available,
            }
            for item in message.drones[:4]
        ]

    def _on_path(self, message: NavPath) -> None:
        self._counts["planned_path"] += 1
        paths: dict[str, list[dict[str, float]]] = {}
        for item in message.poses:
            drone = item.header.frame_id or "unknown"
            paths.setdefault(drone, []).append(
                {
                    "x": round(item.pose.position.x, 2),
                    "y": round(item.pose.position.y, 2),
                }
            )
        self._samples["planned_path"] = paths
        pose_count = len(message.poses)
        if pose_count > self._path_max_poses:
            self._path_max_poses = pose_count
            self._samples["planned_path_longest"] = paths
        for path in paths.values():
            if self._path_detours_obstacle(path):
                self._obstacle_detour_observed = True
                self._obstacle_detour_path = path

    @staticmethod
    def _path_detours_obstacle(path: list[dict[str, float]]) -> bool:
        if len(path) < 2:
            return False
        left_of_barrier = any(point["x"] < min(OBSTACLE_COLUMNS) for point in path)
        right_of_barrier = any(point["x"] > max(OBSTACLE_COLUMNS) for point in path)
        starts_in_barrier_row = (
            min(OBSTACLE_ROWS) <= path[0]["y"] <= max(OBSTACLE_ROWS)
        )
        ends_in_barrier_row = (
            min(OBSTACLE_ROWS) <= path[-1]["y"] <= max(OBSTACLE_ROWS)
        )
        outside_barrier_rows = any(
            point["y"] < min(OBSTACLE_ROWS) or point["y"] > max(OBSTACLE_ROWS)
            for point in path
        )
        intersects_barrier = any(
            min(OBSTACLE_COLUMNS) <= point["x"] <= max(OBSTACLE_COLUMNS)
            and min(OBSTACLE_ROWS) <= point["y"] <= max(OBSTACLE_ROWS)
            for point in path
        )
        return (
            starts_in_barrier_row
            and ends_in_barrier_row
            and left_of_barrier
            and right_of_barrier
            and outside_barrier_rows
            and not intersects_barrier
        )

    def _on_enclosure(self, message: EnclosureCommandArray) -> None:
        self._counts["enclosure_command"] += 1
        self._samples["enclosure_command"] = [
            {
                "drone_id": item.drone_id,
                "x": round(item.target_x, 2),
                "y": round(item.target_y, 2),
                "radius": round(item.enclosure_radius, 2),
                "layer": item.layer,
            }
            for item in message.commands[:4]
        ]

    def report(self, duration: float) -> dict[str, Any]:
        required = ("task_assignment", "drone_states", "planned_path", "enclosure_command")
        nonempty_outputs = {
            "planned_path": self._path_max_poses >= 2,
            "enclosure_command": bool(self._samples.get("enclosure_command")),
        }
        obstacle_avoidance = {
            "warmup_seconds": WARMUP_SECONDS,
            "obstacle_columns": [min(OBSTACLE_COLUMNS), max(OBSTACLE_COLUMNS)],
            "obstacle_rows": [min(OBSTACLE_ROWS), max(OBSTACLE_ROWS)],
            "maximum_path_poses": self._path_max_poses,
            "detour_observed": self._obstacle_detour_observed,
            "detour_path": self._obstacle_detour_path,
        }
        return {
            "duration_s": duration,
            "counts": self._counts,
            "samples": self._samples,
            "nonempty_outputs": nonempty_outputs,
            "obstacle_avoidance": obstacle_avoidance,
            "passed": (
                all(self._counts[key] > 0 for key in required)
                and all(nonempty_outputs.values())
                and self._obstacle_detour_observed
            ),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    rclpy.init()
    driver = ClosedLoopDriver()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(driver, timeout_sec=0.1)
        report = driver.report(args.duration)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 1
    finally:
        driver.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
