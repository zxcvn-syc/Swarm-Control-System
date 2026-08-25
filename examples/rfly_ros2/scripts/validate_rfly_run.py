#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


EXPECTED_EVIDENCE_TOPICS = frozenset({
    "task_assignment",
    "planned_path",
    "enclosure_command",
    "target_track_world",
    "target_track_truth",
    "drone_states",
    "ground_vehicle_states",
})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--telemetry", type=Path, required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--ros-summary", type=Path)
    parser.add_argument("--evidence-manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-physical-occlusion", action="store_true")
    parser.add_argument("--maximum-reacquisition-seconds", type=float, default=2.0)
    parser.add_argument("--physical-window-gap-seconds", type=float, default=0.55)
    parser.add_argument("--minimum-centered-track-ratio", type=float, default=0.0)
    parser.add_argument("--minimum-online-fps", type=float, default=0.0)
    return parser.parse_args()


def load_jsonl(path: Path) -> list[dict]:
    records = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(item, dict):
                records.append(item)
    return records


def video_metrics(path: Path) -> dict[str, float | int]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"could not open {path}")
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    sample_step = max(frame_count // 300, 1)
    previous = None
    frame_differences = []
    brightness = []
    sampled = 0
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % sample_step == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            brightness.append(float(gray.mean()))
            if previous is not None:
                frame_differences.append(float(cv2.absdiff(gray, previous).mean()))
            previous = gray
            sampled += 1
        frame_index += 1
    capture.release()
    return {
        "frame_count": frame_count,
        "fps": fps,
        "duration_seconds": frame_count / max(fps, 1e-6),
        "sampled_frames": sampled,
        "brightness_mean": float(np.mean(brightness)) if brightness else 0.0,
        "brightness_min": min(brightness, default=0.0),
        "mean_frame_difference": (
            float(np.mean(frame_differences)) if frame_differences else 0.0
        ),
        "moving_sample_ratio": (
            sum(value > 0.8 for value in frame_differences)
            / max(len(frame_differences), 1)
        ),
    }


def ros_message_counts(path: Path | None) -> dict[str, int]:
    if path is None or not path.exists():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    counts = data.get(
        "message_counts",
        data.get("topic_message_counts", data.get("received", {})),
    )
    return {
        str(topic): int(count)
        for topic, count in counts.items()
        if isinstance(count, (int, float))
    }


def evidence_manifest_is_complete(path: Path | None) -> bool:
    if path is None or not path.exists():
        return False
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return False
    topics = data.get("topics")
    if not isinstance(topics, dict) or set(topics) != EXPECTED_EVIDENCE_TOPICS:
        return False
    if data.get("pending"):
        return False
    for item in topics.values():
        if not isinstance(item, dict):
            return False
        evidence_file = item.get("first_payload_file")
        if (
            not item.get("has_payload_evidence")
            or not isinstance(evidence_file, str)
            or int(item.get("received", 0)) <= 0
        ):
            return False
        if not (path.parent / evidence_file).is_file():
            return False
    return True


def coalesce_physical_windows(
    windows: list[dict], gap_seconds: float
) -> list[dict]:
    ordered = sorted(
        (dict(window) for window in windows),
        key=lambda window: float(window.get("start_s", 0.0)),
    )
    merged: list[dict] = []
    for window in ordered:
        if not merged:
            merged.append(window)
            continue
        previous = merged[-1]
        gap = float(window.get("start_s", 0.0)) - float(previous.get("end_s", 0.0))
        if gap <= gap_seconds:
            previous["end_s"] = max(
                float(previous.get("end_s", 0.0)),
                float(window.get("end_s", 0.0)),
            )
            previous["target_lost"] = bool(
                previous.get("target_lost") or window.get("target_lost")
            )
            if previous.get("loss_started_at_s") is None:
                previous["loss_started_at_s"] = window.get("loss_started_at_s")
        else:
            merged.append(window)
    return merged


def main() -> None:
    args = parse_args()
    summary = json.loads(args.summary.read_text(encoding="utf-8"))
    telemetry = load_jsonl(args.telemetry)
    if not telemetry:
        raise RuntimeError(f"no telemetry in {args.telemetry}")
    video = video_metrics(args.video)
    overlap_max = max(int(item.get("vehicle_overlap_count", 0)) for item in telemetry)
    finite_clearances = [
        float(item["min_vehicle_clearance_m"])
        for item in telemetry
        if isinstance(item.get("min_vehicle_clearance_m"), (int, float))
        and math.isfinite(float(item["min_vehicle_clearance_m"]))
    ]
    clearance_min = min(finite_clearances, default=float("inf"))
    requested_records = sum(
        bool(item.get("physical_occlusion_requested")) for item in telemetry
    )
    engaged_records = sum(
        bool(item.get("physical_occlusion_engaged")) for item in telemetry
    )
    physical_windows = coalesce_physical_windows(
        summary.get("physical_occlusion_windows", []),
        args.physical_window_gap_seconds,
    )
    physical_windows = [window for window in physical_windows if bool(window.get("target_lost"))]
    target_loss_count = len(physical_windows)
    physical_events = [
        event
        for event in summary.get("reacquisition_events", [])
        if any(
            float(event["lost_at_s"]) >= float(window["start_s"]) - 0.6
            and float(event["lost_at_s"]) < float(window["end_s"])
            and float(event["reacquired_at_s"]) > float(window["start_s"])
            for window in physical_windows
        )
    ]
    physical_max_latency = max(
        (float(event["latency_s"]) for event in physical_events),
        default=None,
    )
    ros_counts = ros_message_counts(args.ros_summary)
    centering = dict(summary.get("tracking_centering", {}))
    centered_track_ratio = float(centering.get("centered_frame_ratio", 0.0))
    online_fps = float(summary.get("average_online_fps", 0.0))
    vision_packet_max = max(
        (int(item.get("vision_packet_count", 0)) for item in telemetry),
        default=0,
    )
    vision_stream_started = any(
        bool(item.get("vision_stream_started")) for item in telemetry
    )
    checks = {
        "video_decodes": video["frame_count"] > 0,
        "video_is_not_black": video["brightness_mean"] >= 15.0,
        "video_is_dynamic": video["moving_sample_ratio"] >= 0.15,
        "online_frames_present": int(summary.get("frames_processed", 0)) > 0,
        "vision_stream_received": vision_stream_started and vision_packet_max > 0,
        "confirmed_tracking_present": int(summary.get("confirmed_track_rows", 0)) > 0,
        "no_vehicle_overlap": overlap_max == 0,
        "separation_tolerance_met": clearance_min >= -0.051,
    }
    if args.minimum_centered_track_ratio > 0.0:
        checks["target_centered_while_locked"] = (
            centered_track_ratio >= args.minimum_centered_track_ratio
        )
    if args.minimum_online_fps > 0.0:
        checks["online_fps_met"] = online_fps >= args.minimum_online_fps
    if args.ros_summary is not None:
        checks["ros_topics_received"] = (
            set(ros_counts) == EXPECTED_EVIDENCE_TOPICS
            and all(ros_counts[topic] > 0 for topic in EXPECTED_EVIDENCE_TOPICS)
        )
    if args.evidence_manifest is not None:
        checks["ros_payload_evidence_complete"] = evidence_manifest_is_complete(
            args.evidence_manifest
        )
    if args.require_physical_occlusion:
        checks.update({
            "physical_occlusion_requested": requested_records > 0,
            "physical_occlusion_engaged": engaged_records > 0,
            "physical_occlusion_caused_target_loss": target_loss_count > 0,
            "all_physical_losses_reacquired": len(physical_events) >= target_loss_count,
        "physical_reacquisition_fast_enough": (
                physical_max_latency is not None
                and physical_max_latency <= args.maximum_reacquisition_seconds
            ),
        })
    result = {
        "passed": all(checks.values()),
        "scenario": summary.get("scenario"),
        "checks": checks,
        "video": video,
        "frames_processed": summary.get("frames_processed", 0),
        "confirmed_track_rows": summary.get("confirmed_track_rows", 0),
        "raw_tracker_fragment_count": summary.get(
            "raw_tracker_fragment_count", len(summary.get("raw_tracker_ids", []))
        ),
        "tracking_centering": {
            **centering,
            "required_centered_frame_ratio": args.minimum_centered_track_ratio,
        },
        "online_fps": {
            "measured": online_fps,
            "required": args.minimum_online_fps,
        },
        "reacquisition_count": summary.get("reacquisition_count", 0),
        "physical_occlusion": {
            "requested_telemetry_records": requested_records,
            "engaged_telemetry_records": engaged_records,
            "target_loss_count": target_loss_count,
            "reacquisition_count": len(physical_events),
            "maximum_reacquisition_latency_seconds": physical_max_latency,
            "coalesced_window_gap_seconds": args.physical_window_gap_seconds,
        },
        "vehicle_geometry": {
            "maximum_overlap_count": overlap_max,
            "minimum_clearance_meters": clearance_min,
        },
        "ros_message_counts": ros_counts,
        "vision_stream": {
            "started": vision_stream_started,
            "maximum_packet_count": vision_packet_max,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
