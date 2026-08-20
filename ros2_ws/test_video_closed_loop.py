#!/usr/bin/env python3
"""Record a real video-to-decision ROS 2 replay without vehicle control.

The companion shell runner launches the real tracker, coordinate transform,
scheduler, grid map, planner, and containment nodes.  This recorder is only
an observer: it verifies and saves their topic output; it never publishes
target tracks, task assignments, paths, or enclosure commands.
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
from swarm_interfaces.msg import (
    DroneStateArray,
    EnclosureCommandArray,
    TaskAssignment,
    TargetTrackArray,
)


OBSTACLE_X_MIN = 21
OBSTACLE_X_MAX = 26
OBSTACLE_Y_MIN = 12
OBSTACLE_Y_MAX = 28


class VideoClosedLoopRecorder(Node):
    """Observe the replay pipeline and serialize a replayable evidence trace."""

    def __init__(self, source: dict[str, Any]) -> None:
        super().__init__("video_closed_loop_recorder")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self._started = time.monotonic()
        self._source = source
        self._counts = {
            "raw_track_messages": 0,
            "raw_track_frames_nonempty": 0,
            "world_track_messages": 0,
            "world_track_frames_nonempty": 0,
            "task_assignments": 0,
            "drone_states": 0,
            "planned_paths": 0,
            "enclosure_commands": 0,
        }
        self._timeline: dict[str, list[dict[str, Any]]] = {
            "raw_tracks": [],
            "world_tracks": [],
            "task_assignments": [],
            "planned_paths": [],
            "enclosure_commands": [],
        }
        self._first_raw_frame_idx: int | None = None
        self._path_max_poses = 0
        self._longest_path: dict[str, list[dict[str, float]]] = {}
        self._detour_path: list[dict[str, float]] = []
        self._detour_observed = False
        self.create_subscription(TargetTrackArray, "/target_track", self._on_raw, qos)
        self.create_subscription(
            TargetTrackArray, "/target_track_world", self._on_world, qos
        )
        self.create_subscription(TaskAssignment, "/task_assignment", self._on_task, qos)
        self.create_subscription(DroneStateArray, "/drone_states", self._on_drones, qos)
        self.create_subscription(NavPath, "/planned_path", self._on_path, qos)
        self.create_subscription(
            EnclosureCommandArray, "/enclosure_command", self._on_enclosure, qos
        )

    def _elapsed(self) -> float:
        return round(time.monotonic() - self._started, 3)

    @staticmethod
    def _tracks(message: TargetTrackArray) -> list[dict[str, Any]]:
        return [
            {
                "target_id": int(track.target_id),
                "x": round(float(track.x), 3),
                "y": round(float(track.y), 3),
                "vx": round(float(track.vx), 3),
                "vy": round(float(track.vy), 3),
                "confidence": round(float(track.confidence), 3),
                "confirmed": bool(track.is_confirmed),
            }
            for track in message.tracks
            if math.isfinite(float(track.x)) and math.isfinite(float(track.y))
        ]

    def _on_raw(self, message: TargetTrackArray) -> None:
        tracks = self._tracks(message)
        self._counts["raw_track_messages"] += 1
        self._counts["raw_track_frames_nonempty"] += bool(tracks)
        frame_idx = int(message.frame_idx)
        if self._first_raw_frame_idx is None:
            self._first_raw_frame_idx = frame_idx
        self._timeline["raw_tracks"].append(
            {"t": self._elapsed(), "frame_idx": frame_idx, "tracks": tracks}
        )

    def _on_world(self, message: TargetTrackArray) -> None:
        tracks = self._tracks(message)
        self._counts["world_track_messages"] += 1
        self._counts["world_track_frames_nonempty"] += bool(tracks)
        self._timeline["world_tracks"].append(
            {
                "t": self._elapsed(),
                "frame_idx": int(message.frame_idx),
                "frame_id": str(message.header.frame_id),
                "tracks": tracks,
            }
        )

    def _on_task(self, message: TaskAssignment) -> None:
        self._counts["task_assignments"] += 1
        self._timeline["task_assignments"].append(
            {
                "t": self._elapsed(),
                "drone_id": int(message.drone_id),
                "target_id": int(message.target_id),
                "task_type": str(message.task_type),
            }
        )

    def _on_drones(self, message: DroneStateArray) -> None:
        self._counts["drone_states"] += 1

    def _on_path(self, message: NavPath) -> None:
        self._counts["planned_paths"] += 1
        paths: dict[str, list[dict[str, float]]] = {}
        for pose in message.poses:
            drone_id = str(pose.header.frame_id or "unknown")
            paths.setdefault(drone_id, []).append(
                {
                    "x": round(float(pose.pose.position.x), 3),
                    "y": round(float(pose.pose.position.y), 3),
                }
            )
        self._timeline["planned_paths"].append(
            {"t": self._elapsed(), "paths": paths}
        )
        pose_count = sum(len(path) for path in paths.values())
        if pose_count > self._path_max_poses:
            self._path_max_poses = pose_count
            self._longest_path = paths
        for path in paths.values():
            if self._detours_obstacle(path):
                self._detour_observed = True
                self._detour_path = path

    def _on_enclosure(self, message: EnclosureCommandArray) -> None:
        self._counts["enclosure_commands"] += 1
        commands = [
            {
                "drone_id": int(command.drone_id),
                "x": round(float(command.target_x), 3),
                "y": round(float(command.target_y), 3),
                "radius": round(float(command.enclosure_radius), 3),
                "layer": int(command.layer),
            }
            for command in message.commands
        ]
        self._timeline["enclosure_commands"].append(
            {"t": self._elapsed(), "commands": commands}
        )

    @staticmethod
    def _detours_obstacle(path: list[dict[str, float]]) -> bool:
        if len(path) < 2:
            return False
        left = any(point["x"] < OBSTACLE_X_MIN for point in path)
        right = any(point["x"] > OBSTACLE_X_MAX for point in path)
        starts_in_row = OBSTACLE_Y_MIN <= path[0]["y"] <= OBSTACLE_Y_MAX
        ends_in_row = OBSTACLE_Y_MIN <= path[-1]["y"] <= OBSTACLE_Y_MAX
        exits_rows = any(
            point["y"] < OBSTACLE_Y_MIN or point["y"] > OBSTACLE_Y_MAX
            for point in path
        )
        intersects = any(
            OBSTACLE_X_MIN <= point["x"] <= OBSTACLE_X_MAX
            and OBSTACLE_Y_MIN <= point["y"] <= OBSTACLE_Y_MAX
            for point in path
        )
        return left and right and starts_in_row and ends_in_row and exits_rows and not intersects

    def report(self, duration: float) -> dict[str, Any]:
        frame_ids = {
            item["frame_id"]
            for item in self._timeline["world_tracks"]
            if item["tracks"]
        }
        required = (
            "raw_track_frames_nonempty",
            "world_track_frames_nonempty",
            "task_assignments",
            "planned_paths",
            "enclosure_commands",
        )
        passed = (
            all(self._counts[key] > 0 for key in required)
            and frame_ids == {"world"}
            and self._detour_observed
        )
        return {
            "duration_s": duration,
            "source": self._source,
            "counts": self._counts,
            "world_frame_ids": sorted(frame_ids),
            "first_raw_frame_idx": self._first_raw_frame_idx,
            "maximum_path_poses": self._path_max_poses,
            "longest_path": self._longest_path,
            "obstacle_avoidance": {
                "obstacle_columns": [OBSTACLE_X_MIN, OBSTACLE_X_MAX],
                "obstacle_rows": [OBSTACLE_Y_MIN, OBSTACLE_Y_MAX],
                "detour_observed": self._detour_observed,
                "detour_path": self._detour_path,
            },
            "timeline": self._timeline,
            "passed": passed,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-video", required=True)
    parser.add_argument("--source-width", type=int, required=True)
    parser.add_argument("--source-height", type=int, required=True)
    parser.add_argument("--source-fps", type=float, required=True)
    parser.add_argument("--tracker-rate", type=float, required=True)
    parser.add_argument("--detector-backend", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0 or args.source_width <= 0 or args.source_height <= 0:
        raise SystemExit("duration and source dimensions must be positive")
    source = {
        "video": args.source_video,
        "width": args.source_width,
        "height": args.source_height,
        "fps": args.source_fps,
        "tracker_rate": args.tracker_rate,
        "detector_backend": args.detector_backend,
        "target_source": "tracker_node video inference",
    }
    rclpy.init()
    recorder = VideoClosedLoopRecorder(source)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(recorder, timeout_sec=0.1)
        report = recorder.report(args.duration)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps({
            "passed": report["passed"],
            "counts": report["counts"],
            "obstacle_avoidance": report["obstacle_avoidance"],
        }, indent=2))
        return 0 if report["passed"] else 1
    finally:
        recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
