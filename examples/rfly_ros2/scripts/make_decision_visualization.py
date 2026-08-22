#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import json
from pathlib import Path

import cv2
import numpy as np


WORLD_SIZE = 220.0
WAYPOINTS = (
    (20.0, 30.0),
    (80.0, 25.0),
    (145.0, 35.0),
    (180.0, 80.0),
    (170.0, 145.0),
    (115.0, 185.0),
    (50.0, 170.0),
    (15.0, 120.0),
    (30.0, 70.0),
)
PARKED_CARS = (
    (95.0, 38.0),
    (108.0, 43.0),
    (161.0, 63.0),
    (170.0, 99.0),
    (151.0, 154.0),
    (112.0, 168.0),
    (43.0, 151.0),
    (29.0, 105.0),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--telemetry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenario", default="auto")
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument(
        "--telemetry-offset",
        type=float,
        default=0.0,
        help="scene telemetry time corresponding to video time zero",
    )
    return parser.parse_args()


def load_records(path: Path) -> list[dict]:
    records = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(item, dict) and isinstance(item.get("time_s"), (int, float)):
                records.append(item)
    return sorted(records, key=lambda item: float(item["time_s"]))


def value_text(value, fallback="-") -> str:
    if value is None:
        return fallback
    if isinstance(value, float):
        return f"{value:.2f}"
    return str(value)


def world_to_panel(x: float, y: float, panel_width: int, panel_height: int) -> tuple[int, int]:
    margin_x = 34
    margin_top = 118
    margin_bottom = 34
    scale = min(
        (panel_width - 2 * margin_x) / WORLD_SIZE,
        (panel_height - margin_top - margin_bottom) / WORLD_SIZE,
    )
    px = int(margin_x + x * scale)
    py = int(panel_height - margin_bottom - y * scale)
    return px, py


def draw_map(panel, record: dict) -> None:
    height, width = panel.shape[:2]
    cv2.rectangle(panel, (0, 0), (width, height), (18, 23, 28), -1)
    cv2.putText(
        panel,
        "CVTrack DECISION REPLAY",
        (24, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (240, 245, 248),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        "WORLD VIEW / DATA SOURCES",
        (25, 57),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.44,
        (142, 170, 184),
        1,
        cv2.LINE_AA,
    )
    for value in range(0, 221, 20):
        start = world_to_panel(value, 0.0, width, height)
        end = world_to_panel(value, WORLD_SIZE, width, height)
        cv2.line(panel, start, end, (34, 50, 57), 1)
        start = world_to_panel(0.0, value, width, height)
        end = world_to_panel(WORLD_SIZE, value, width, height)
        cv2.line(panel, start, end, (34, 50, 57), 1)

    route_points = [world_to_panel(x, y, width, height) for x, y in WAYPOINTS]
    for start, end in zip(route_points, route_points[1:] + route_points[:1]):
        cv2.line(panel, start, end, (58, 81, 86), 1, cv2.LINE_AA)

    for x, y in PARKED_CARS:
        cv2.circle(panel, world_to_panel(x, y, width, height), 6, (92, 98, 98), -1)

    for obstacle in record.get("dynamic_obstacles", []):
        try:
            point = world_to_panel(float(obstacle["x"]), float(obstacle["y"]), width, height)
        except (KeyError, TypeError, ValueError):
            continue
        cv2.circle(panel, point, 9, (160, 125, 62), -1)
        cv2.circle(panel, point, 12, (223, 175, 89), 1)

    uavs = record.get("uavs") or {}
    active_host = int(record.get("active_host", 1))
    for host_id, pose in uavs.items():
        if not isinstance(pose, (list, tuple)) or len(pose) < 3:
            continue
        point = world_to_panel(float(pose[0]), float(pose[1]), width, height)
        color = (54, 216, 240) if int(host_id) == active_host else (98, 145, 160)
        cv2.circle(panel, point, 7, color, -1)
        cv2.putText(
            panel,
            f"U{host_id}",
            (point[0] + 8, point[1] + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA,
        )

    target_truth = record.get("target_truth") or {}
    target_visual = record.get("target_visual") or {}
    target_control = record.get("target_control") or {}
    if target_truth:
        point = world_to_panel(float(target_truth["x"]), float(target_truth["y"]), width, height)
        cv2.drawMarker(panel, point, (66, 82, 235), cv2.MARKER_TILTED_CROSS, 22, 3)
    if target_visual:
        point = world_to_panel(float(target_visual["x"]), float(target_visual["y"]), width, height)
        cv2.circle(panel, point, 8, (68, 213, 237), 2)
    if target_control:
        control_point = world_to_panel(
            float(target_control["x"]),
            float(target_control["y"]),
            width,
            height,
        )
        cv2.circle(panel, control_point, 8, (73, 226, 116), -1)
        vx = float(target_control.get("vx", 0.0))
        vy = float(target_control.get("vy", 0.0))
        for step in (0.5, 1.0, 1.5, 2.0, 2.5):
            next_point = world_to_panel(
                float(target_control["x"]) + vx * step,
                float(target_control["y"]) + vy * step,
                width,
                height,
            )
            cv2.line(panel, control_point, next_point, (73, 226, 116), 1, cv2.LINE_AA)
            control_point = next_point

    ground_cars = record.get("ground_cars") or {}
    ground_goals = record.get("ground_goals") or {}
    for car_id, position in ground_cars.items():
        if not isinstance(position, (list, tuple)) or len(position) < 2:
            continue
        point = world_to_panel(float(position[0]), float(position[1]), width, height)
        cv2.circle(panel, point, 6, (175, 175, 175), -1)
        goal = ground_goals.get(str(car_id), ground_goals.get(car_id))
        if isinstance(goal, (list, tuple)) and len(goal) >= 2:
            goal_point = world_to_panel(float(goal[0]), float(goal[1]), width, height)
            cv2.circle(panel, goal_point, 9, (220, 180, 62), 1)
            cv2.line(panel, point, goal_point, (112, 129, 138), 1, cv2.LINE_AA)

    info_y = height - 82
    scenario = record.get("scenario", "unknown")
    weather = record.get("weather", "unknown")
    mode = record.get("mode", "unknown")
    source = record.get("target_control_source", "none")
    cv2.putText(panel, f"MODE {mode}  HOST U{active_host}", (24, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 245, 248), 1, cv2.LINE_AA)
    cv2.putText(panel, f"SCENE {scenario}  WEATHER {weather}", (24, info_y + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (176, 202, 211), 1, cv2.LINE_AA)
    cv2.putText(panel, f"CONTROL {source}  LEAD {value_text(record.get('prediction_lead_s'))} s", (24, info_y + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (176, 202, 211), 1, cv2.LINE_AA)
    cv2.putText(panel, "X truth  O visual  G control  gray UAV  gray car", (24, info_y + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (132, 150, 158), 1, cv2.LINE_AA)


def main() -> None:
    args = parse_args()
    records = load_records(args.telemetry)
    if not records:
        raise RuntimeError(f"no telemetry records found in {args.telemetry}")
    times = [float(item["time_s"]) for item in records]
    capture = cv2.VideoCapture(str(args.input))
    if not capture.isOpened():
        raise RuntimeError(f"could not open {args.input}")
    fps = capture.get(cv2.CAP_PROP_FPS) or 20.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    panel_width = max(420, int(width * 0.36))
    output_width = width + panel_width
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (output_width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open {args.output}")

    frame_count = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        video_time = frame_count / fps + args.telemetry_offset
        index = min(max(bisect.bisect_right(times, video_time) - 1, 0), len(records) - 1)
        record = records[index]
        panel = np.zeros((height, panel_width, 3), dtype=np.uint8)
        draw_map(panel, record)
        cv2.rectangle(frame, (0, 0), (width, 80), (12, 17, 21), -1)
        scenario = args.scenario if args.scenario != "auto" else record.get("scenario", "unknown")
        cv2.putText(frame, f"SCENARIO {scenario} | WEATHER {record.get('weather', 'unknown')} | MODE {record.get('mode', 'unknown')}", (18, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (244, 246, 248), 2, cv2.LINE_AA)
        cv2.putText(frame, f"target_control_source={record.get('target_control_source', 'none')} | t={video_time:06.1f}s", (18, 61), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (182, 204, 214), 1, cv2.LINE_AA)
        output_frame = np.hstack((frame, panel))
        writer.write(output_frame)
        frame_count += 1
        if args.max_frames > 0 and frame_count >= args.max_frames:
            break

    capture.release()
    writer.release()
    if args.summary:
        args.summary.write_text(
            json.dumps(
                {
                    "input": str(args.input),
                    "telemetry": str(args.telemetry),
                    "output": str(args.output),
                    "frames": frame_count,
                    "fps": fps,
                    "resolution": [output_width, height],
                    "telemetry_offset_s": args.telemetry_offset,
                    "visualization": ["aerial_view", "truth_visual_control_layers", "prediction_path", "uav_and_ground_vehicle_tasks"],
                },
                indent=2,
            ),
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
