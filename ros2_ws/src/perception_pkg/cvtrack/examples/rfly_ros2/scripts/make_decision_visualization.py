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
    (18.0, 24.0),
    (82.0, 18.0),
    (150.0, 22.0),
    (202.0, 62.0),
    (202.0, 128.0),
    (170.0, 190.0),
    (106.0, 205.0),
    (40.0, 190.0),
    (12.0, 140.0),
    (10.0, 74.0),
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
    parser.add_argument("--telemetry-offset", type=float, default=0.0)
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
    margin_bottom = 154
    scale = min(
        (panel_width - 2 * margin_x) / WORLD_SIZE,
        (panel_height - margin_top - margin_bottom) / WORLD_SIZE,
    )
    return int(margin_x + x * scale), int(panel_height - margin_bottom - y * scale)


def path_from_records(records: list[dict], collection: str, entity_id=None) -> list[tuple[float, float]]:
    points = []
    for item in records:
        data = item.get(collection) or {}
        if entity_id is not None:
            data = data.get(str(entity_id), data.get(entity_id))
        if isinstance(data, dict):
            try:
                points.append((float(data["x"]), float(data["y"])))
            except (KeyError, TypeError, ValueError):
                continue
        elif isinstance(data, (list, tuple)) and len(data) >= 2:
            try:
                points.append((float(data[0]), float(data[1])))
            except (TypeError, ValueError):
                continue
    return points


def draw_path(panel, points: list[tuple[float, float]], color, width: int = 2) -> None:
    pixels = [world_to_panel(x, y, panel.shape[1], panel.shape[0]) for x, y in points]
    for (start, end), (world_start, world_end) in zip(
        zip(pixels, pixels[1:]), zip(points, points[1:])
    ):
        if any(
            value < -2.0 or value > WORLD_SIZE + 2.0
            for value in (*world_start, *world_end)
        ):
            continue
        cv2.line(panel, start, end, color, width, cv2.LINE_AA)


def is_world_visible(x: float, y: float) -> bool:
    return 0.0 <= x <= WORLD_SIZE and 0.0 <= y <= WORLD_SIZE


def current_stage(record: dict) -> str:
    physical_occlusion = record.get("physical_occlusion_engaged", False)
    if isinstance(record.get("physical_occlusion"), dict):
        physical_occlusion = record["physical_occlusion"].get("engaged", False)
    if record.get("reacquisition_active"):
        return "REACQUIRE"
    if physical_occlusion and record.get("mode") in {"coast", "search"}:
        return "OCCLUDED"
    phase = str(record.get("phase", ""))
    if phase == "handoff" or record.get("host_changed"):
        return "HANDOFF"
    if phase == "contain" or record.get("ground_goals"):
        return "CONTAIN"
    if phase == "coast_recover" or record.get("mode") == "coast":
        return "COAST/RECOVER"
    if phase == "lock_predict" or record.get("mode") == "track":
        return "PREDICT"
    return "SEARCH"


def draw_stage_timeline(panel, record: dict, history: list[dict]) -> None:
    labels = ("SEARCH", "LOCK", "PREDICT", "OCCLUDED", "REACQUIRE", "HANDOFF", "CONTAIN")
    active = current_stage(record)
    seen = {
        "SEARCH": True,
        "LOCK": any(item.get("mode") == "track" for item in history),
        "PREDICT": any(item.get("target_control") for item in history),
        "OCCLUDED": any(item.get("physical_occlusion_engaged") for item in history),
        "REACQUIRE": any(item.get("reacquisition_active") for item in history),
        "HANDOFF": any(item.get("host_changed") for item in history),
        "CONTAIN": any(item.get("ground_goals") for item in history),
    }
    left_margin = 24
    top = 70
    box_width = (panel.shape[1] - 48) // len(labels)
    for index, label in enumerate(labels):
        left = left_margin + index * box_width
        right = left_margin + (index + 1) * box_width - 4
        color = (45, 71, 78)
        if seen[label]:
            color = (45, 105, 78)
        if label == active:
            color = (50, 128, 174)
        cv2.rectangle(panel, (left, top), (right, top + 24), color, -1)
        cv2.putText(
            panel,
            label,
            (left + 5, top + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.30 if len(label) > 8 else 0.36,
            (238, 245, 247),
            1,
            cv2.LINE_AA,
        )


def draw_map(panel, record: dict, history: list[dict]) -> None:
    height, width = panel.shape[:2]
    cv2.rectangle(panel, (0, 0), (width, height), (18, 23, 28), -1)
    cv2.putText(panel, "CVTrack DECISION REPLAY", (24, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (240, 245, 248), 2, cv2.LINE_AA)
    cv2.putText(panel, "WORLD VIEW / HISTORY + DATA SOURCES", (25, 57), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (142, 170, 184), 1, cv2.LINE_AA)
    draw_stage_timeline(panel, record, history)
    cv2.putText(panel, "RED truth | CYAN visual | GREEN control | GREY ground | ORANGE blocker", (25, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.26, (156, 181, 190), 1, cv2.LINE_AA)

    for value in range(0, 221, 20):
        cv2.line(panel, world_to_panel(value, 0.0, width, height), world_to_panel(value, WORLD_SIZE, width, height), (34, 50, 57), 1)
        cv2.line(panel, world_to_panel(0.0, value, width, height), world_to_panel(WORLD_SIZE, value, width, height), (34, 50, 57), 1)
    route_points = [world_to_panel(x, y, width, height) for x, y in WAYPOINTS]
    for start, end in zip(route_points, route_points[1:] + route_points[:1]):
        cv2.line(panel, start, end, (58, 81, 86), 1, cv2.LINE_AA)
    for x, y in PARKED_CARS:
        cv2.circle(panel, world_to_panel(x, y, width, height), 6, (92, 98, 98), -1)

    for obstacle in record.get("large_obstacles", []):
        try:
            point = world_to_panel(float(obstacle["x"]), float(obstacle["y"]), width, height)
            scale = max(1.0, float(obstacle.get("scale", 1.0)))
        except (KeyError, TypeError, ValueError):
            continue
        radius = min(13, 5 + int(scale * 2.0))
        cv2.rectangle(
            panel,
            (point[0] - radius, point[1] - radius),
            (point[0] + radius, point[1] + radius),
            (81, 118, 173),
            -1,
        )
        cv2.drawMarker(panel, point, (226, 192, 114), cv2.MARKER_CROSS, radius * 2, 1)

    draw_path(panel, path_from_records(history, "target_truth"), (235, 65, 45), 3)
    draw_path(panel, path_from_records(history, "target_visual"), (237, 213, 68), 2)
    draw_path(panel, path_from_records(history, "target_control"), (93, 225, 115), 2)
    for host_id in (1, 2, 3):
        draw_path(panel, path_from_records(history, "uavs", host_id), (70, 108, 128), 1)
    for car_id in (0, 1, 2):
        draw_path(panel, path_from_records(history, "ground_cars", car_id), (135, 135, 135), 1)

    for obstacle in record.get("dynamic_obstacles", []):
        try:
            point = world_to_panel(float(obstacle["x"]), float(obstacle["y"]), width, height)
        except (KeyError, TypeError, ValueError):
            continue
        cv2.circle(panel, point, 9, (160, 125, 62), -1)
        cv2.circle(panel, point, 12, (223, 175, 89), 1)

    active_host = int(record.get("active_host", 1))
    uavs = record.get("uavs") or {}
    for host_id, pose in uavs.items():
        if not isinstance(pose, (list, tuple)) or len(pose) < 3:
            continue
        point = world_to_panel(float(pose[0]), float(pose[1]), width, height)
        color = (54, 216, 240) if int(host_id) == active_host else (98, 145, 160)
        cv2.circle(panel, point, 7, color, -1)
        cv2.putText(panel, f"U{host_id}", (point[0] + 8, point[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    target_truth = record.get("target_truth") or {}
    target_visual = record.get("target_visual") or {}
    target_control = record.get("target_control") or {}
    if target_truth:
        point = world_to_panel(float(target_truth["x"]), float(target_truth["y"]), width, height)
        cv2.drawMarker(panel, point, (66, 82, 235), cv2.MARKER_TILTED_CROSS, 22, 3)
        cv2.putText(panel, "TARGET BLUE", (point[0] + 10, point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (66, 82, 235), 1, cv2.LINE_AA)
        if record.get("physical_occlusion_requested") or record.get("physical_occlusion_engaged"):
            active_pose = uavs.get(str(active_host), uavs.get(active_host))
            if isinstance(active_pose, (list, tuple)) and len(active_pose) >= 2:
                camera_point = world_to_panel(float(active_pose[0]), float(active_pose[1]), width, height)
                cv2.line(panel, camera_point, point, (120, 120, 220), 1, cv2.LINE_AA)
                cv2.putText(panel, "LOS / BLOCKER", (camera_point[0] + 5, camera_point[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (160, 160, 228), 1, cv2.LINE_AA)
    if target_visual:
        visual_x = float(target_visual["x"])
        visual_y = float(target_visual["y"])
        if is_world_visible(visual_x, visual_y):
            point = world_to_panel(visual_x, visual_y, width, height)
            cv2.circle(panel, point, 8, (68, 213, 237), 2)
    if target_control:
        control_x = float(target_control["x"])
        control_y = float(target_control["y"])
        if is_world_visible(control_x, control_y):
            control_point = world_to_panel(control_x, control_y, width, height)
            cv2.circle(panel, control_point, 8, (73, 226, 116), -1)
            vx = float(target_control.get("vx", 0.0))
            vy = float(target_control.get("vy", 0.0))
            for step in (0.5, 1.0, 1.5, 2.0, 2.5):
                next_x = control_x + vx * step
                next_y = control_y + vy * step
                if not is_world_visible(next_x, next_y):
                    break
                next_point = world_to_panel(next_x, next_y, width, height)
                cv2.line(panel, control_point, next_point, (73, 226, 116), 1, cv2.LINE_AA)
                control_point = next_point
            lead = float(record.get("prediction_lead_s", 1.0))
            predicted_x = control_x + vx * lead
            predicted_y = control_y + vy * lead
            if is_world_visible(predicted_x, predicted_y):
                predicted = world_to_panel(predicted_x, predicted_y, width, height)
                cv2.arrowedLine(panel, world_to_panel(control_x, control_y, width, height), predicted, (73, 226, 116), 3, cv2.LINE_AA, 0, 0.25)

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

    info_y = height - 142
    scenario = record.get("scenario", "unknown")
    weather = record.get("weather", "unknown")
    wind_speed = float(record.get("wind_speed_mps", 0.0))
    wind_direction = float(record.get("wind_direction_deg", 0.0))
    mode = record.get("mode", "unknown")
    source = record.get("target_control_source", "none")
    speed = value_text(record.get("target_speed_mps"), "-")
    heading = value_text(record.get("target_heading_deg"), "-")
    cv2.putText(panel, f"PHASE {current_stage(record)}  MODE {mode}  HOST U{active_host}", (24, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (240, 245, 248), 1, cv2.LINE_AA)
    rain_status = "RAIN ON" if "rain" in str(weather).lower() else "RAIN OFF"
    cv2.putText(panel, f"{scenario} | {rain_status} | WIND {wind_speed:.1f} m/s @ {wind_direction:.0f} deg", (24, info_y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (176, 202, 211), 1, cv2.LINE_AA)
    cv2.putText(panel, f"TARGET BLUE  SPEED {speed} m/s  HEADING {heading} deg  LEAD {value_text(record.get('prediction_lead_s'))} s", (24, info_y + 43), cv2.FONT_HERSHEY_SIMPLEX, 0.31, (176, 202, 211), 1, cv2.LINE_AA)
    cv2.putText(panel, f"ROS RGB -> CVTrack -> world -> planner -> enclosure | goals {record.get('ground_goal_update_count', 0)}", (24, info_y + 64), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (176, 202, 211), 1, cv2.LINE_AA)
    physical_status = "ENGAGED" if record.get("physical_occlusion_engaged") else "REQUESTED" if record.get("physical_occlusion_requested") else "OFF"
    min_distance = record.get("min_vehicle_distance_m")
    overlap_count = record.get("vehicle_overlap_count", 0)
    cv2.putText(panel, f"PHYSICAL LOS {physical_status} | MIN VEHICLE DIST {value_text(min_distance)} m | OVERLAP {overlap_count}", (24, info_y + 85), cv2.FONT_HERSHEY_SIMPLEX, 0.27, (132, 150, 158), 1, cv2.LINE_AA)


def draw_motion_inset(frame, record: dict, history: list[dict]) -> None:
    height, _ = frame.shape[:2]
    inset_width = 314
    inset_height = 164
    x0 = 18
    y0 = height - inset_height - 18
    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + inset_width, y0 + inset_height), (12, 18, 22), -1)
    cv2.addWeighted(overlay, 0.88, frame, 0.12, 0.0, frame)
    cv2.rectangle(frame, (x0, y0), (x0 + inset_width, y0 + inset_height), (90, 132, 145), 1)
    cv2.putText(frame, "WORLD MOTION / TARGET BLUE", (x0 + 10, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (235, 242, 245), 1, cv2.LINE_AA)
    plot_x, plot_y = x0 + 10, y0 + 30
    plot_w, plot_h = inset_width - 20, 92
    cv2.rectangle(frame, (plot_x, plot_y), (plot_x + plot_w, plot_y + plot_h), (23, 34, 40), -1)
    points = path_from_records(history, "target_truth")
    pixels = [
        (
            plot_x + int(max(0.0, min(WORLD_SIZE, x)) / WORLD_SIZE * plot_w),
            plot_y + plot_h - int(max(0.0, min(WORLD_SIZE, y)) / WORLD_SIZE * plot_h),
        )
        for x, y in points
    ]
    for start, end in zip(pixels, pixels[1:]):
        cv2.line(frame, start, end, (235, 65, 45), 2, cv2.LINE_AA)
    target = record.get("target_truth") or {}
    if target:
        current = (
            plot_x + int(max(0.0, min(WORLD_SIZE, float(target["x"]))) / WORLD_SIZE * plot_w),
            plot_y + plot_h - int(max(0.0, min(WORLD_SIZE, float(target["y"]))) / WORLD_SIZE * plot_h),
        )
        cv2.circle(frame, current, 6, (66, 82, 235), -1)
    cv2.putText(frame, f"x/y {value_text(target.get('x'))}/{value_text(target.get('y'))} m", (x0 + 10, y0 + 142), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (190, 210, 216), 1, cv2.LINE_AA)
    cv2.putText(frame, f"speed {value_text(record.get('target_speed_mps'))} m/s phase {current_stage(record)}", (x0 + 10, y0 + 157), cv2.FONT_HERSHEY_SIMPLEX, 0.31, (190, 210, 216), 1, cv2.LINE_AA)


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
    panel_width = max(500, int(width * 0.42))
    if (width + panel_width) % 2:
        panel_width += 1
    output_width = width + panel_width
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (output_width, height))
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
        history = records[max(0, index - 120):index + 1]
        panel = np.zeros((height, panel_width, 3), dtype=np.uint8)
        draw_map(panel, record, history)
        cv2.rectangle(frame, (0, 0, width, 80), (12, 17, 21), -1)
        scenario = args.scenario if args.scenario != "auto" else record.get("scenario", "unknown")
        cv2.putText(frame, f"SCENARIO {scenario} | WEATHER {record.get('weather', 'unknown')} | PHASE {current_stage(record)}", (18, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (244, 246, 248), 2, cv2.LINE_AA)
        cv2.putText(frame, f"RGB + CVTrack + ROS2 | source={record.get('target_control_source', 'none')} | t={video_time:06.1f}s", (18, 61), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (182, 204, 214), 1, cv2.LINE_AA)
        draw_motion_inset(frame, record, history)
        writer.write(np.hstack((frame, panel)))
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
                    "visualization": [
                        "aerial_view",
                        "target_truth_history",
                        "target_visual_history",
                        "control_prediction",
                        "uav_history",
                        "ground_vehicle_history",
                        "phase_timeline",
                        "rain_wind_occlusion_status",
                        "large_static_obstacles",
                        "world_motion_inset",
                    ],
                },
                indent=2,
            ),
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
