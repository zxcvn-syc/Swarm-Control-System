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
        (self.output_dir / f"{name}.yaml").write_text(
            message_to_yaml(message),
            encoding="utf-8",
        )
        self.pending.remove(name)


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
    try:
        while rclpy.ok() and node.pending and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        summary = {
            "duration_s": round(time.monotonic() - started, 3),
            "pending": sorted(node.pending),
            "received": node.received,
        }
        (args.output_dir / "capture_summary.json").write_text(
            json.dumps(summary, indent=2),
            encoding="utf-8",
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
