#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_yaml
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "task_assignment": ("/task_assignment", "swarm_interfaces/msg/TaskAssignment"),
    "planned_path": ("/planned_path", "nav_msgs/msg/Path"),
    "enclosure_command": ("/enclosure_command", "swarm_interfaces/msg/EnclosureCommandArray"),
    "target_track_world": ("/target_track_world", "swarm_interfaces/msg/TargetTrackArray"),
    "target_track_truth": ("/target_track_truth", "swarm_interfaces/msg/TargetTrackArray"),
    "drone_states": ("/drone_states", "swarm_interfaces/msg/DroneStateArray"),
    "ground_vehicle_states": ("/ground_vehicle_states", "swarm_interfaces/msg/DroneStateArray"),
}


def has_payload(message) -> bool:
    for field_name in ("commands", "tracks", "poses", "drones", "assignments", "tasks"):
        value = getattr(message, field_name, None)
        if value is not None:
            return bool(value)
    if hasattr(message, "target_id"):
        return int(message.target_id) > 0
    return True


class EvidenceRecorder(Node):
    def __init__(self, output_dir: Path) -> None:
        super().__init__("rfly_evidence_recorder")
        self.output_dir = output_dir
        self.pending = set(TOPICS)
        self.received: dict[str, int] = {name: 0 for name in TOPICS}
        self.first_payload_files: dict[str, str] = {}
        self._evidence_subscriptions = []
        for name, (topic, type_name) in TOPICS.items():
            message_type = get_message(type_name)
            self._evidence_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda message, key=name: self.on_message(key, message),
                    10,
                )
            )

    def on_message(self, name: str, message) -> None:
        self.received[name] += 1
        if name not in self.pending or not has_payload(message):
            return
        evidence_file = self.output_dir / f"{name}.yaml"
        evidence_file.write_text(
            message_to_yaml(message),
            encoding="utf-8",
        )
        self.first_payload_files[name] = evidence_file.name
        self.pending.remove(name)

    def manifest(self) -> dict:
        return {
            "schema": "cvtrack-rfly-evidence-v1",
            "topics": {
                name: {
                    "topic": topic,
                    "type": type_name,
                    "received": self.received[name],
                    "first_payload_file": self.first_payload_files.get(name),
                    "has_payload_evidence": name in self.first_payload_files,
                }
                for name, (topic, type_name) in TOPICS.items()
            },
            "pending": sorted(self.pending),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = EvidenceRecorder(args.output_dir)
    started = time.monotonic()
    completed = False
    try:
        while rclpy.ok() and node.pending and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
        completed = not node.pending
    finally:
        summary = {
            "duration_s": round(time.monotonic() - started, 3),
            "pending": sorted(node.pending),
            "received": node.received,
            "message_counts": node.received,
            "manifest": "evidence_manifest.json",
        }
        (args.output_dir / "evidence_manifest.json").write_text(
            json.dumps(node.manifest(), indent=2),
            encoding="utf-8",
        )
        (args.output_dir / "capture_summary.json").write_text(
            json.dumps(summary, indent=2),
            encoding="utf-8",
        )
        node.destroy_node()
        rclpy.shutdown()
    if not completed:
        raise SystemExit("required ROS payload evidence was incomplete")


if __name__ == "__main__":
    main()
