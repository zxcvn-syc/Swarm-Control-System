#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import socket
import sys
import threading
import time
import types
from pathlib import Path

import cv2
import numpy as np


RFLY_SDK = Path(r"F:\RflySimAPIs\RflySimSDK")
SCRIPT_PATH = Path(__file__).resolve()
CVTRACK_CANDIDATES = (
    SCRIPT_PATH.parents[3] / "src",
    SCRIPT_PATH.parents[3] / "work" / "cvtrack" / "src",
)
CVTRACK_SRC = next((path for path in CVTRACK_CANDIDATES if path.exists()), None)
if CVTRACK_SRC is None:
    raise RuntimeError("cvtrack source tree was not found")
for sdk_path in (RFLY_SDK, RFLY_SDK / "ctrl", RFLY_SDK / "ue", RFLY_SDK / "vision"):
    if str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))
if str(CVTRACK_SRC) not in sys.path:
    sys.path.insert(0, str(CVTRACK_SRC))

try:
    import open3d  # noqa: F401
except ImportError:
    open3d_show_stub = types.ModuleType("Open3DShow")

    class Open3DUnavailable:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("open3d is required only for Rfly point-cloud display")

    open3d_show_stub.Open3DShow = Open3DUnavailable
    sys.modules["Open3DShow"] = open3d_show_stub

import VisionCaptureApi  # noqa: E402
from cvtrack.detector.factory import make_detector  # noqa: E402
from cvtrack.tracker.botsort import BoTSortTracker  # noqa: E402
from cvtrack.tracker.metrics import iou  # noqa: E402
from cvtrack.types import Box  # noqa: E402


SENSOR_SETTLE_SECONDS = 0.22
SEARCH_DWELL_SECONDS = 4.0
INITIAL_ACQUISITION_HOLD_SECONDS = 14.0
TRACK_RECOVERY_HOLD_SECONDS = 12.0
LOCK_QUALIFICATION_SECONDS = 1.0
PHYSICAL_OCCLUSION_COALESCE_GAP_SECONDS = 0.55
PHYSICAL_SENSOR_DROPOUT_SECONDS = 1.0
REACQUISITION_LOSS_DEBOUNCE_SECONDS = 0.35
REACQUISITION_ARM_STABLE_SECONDS = 1.5
STABLE_CAMERA_CARRIER_ID = 1
SCENARIO_CONFIG_PATH = Path(__file__).with_name("scenario_presets.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=70.0)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=35661)
    parser.add_argument(
        "--status-udp-port",
        type=int,
        default=0,
        help="local UDP port for scene status forwarded through the VM relay",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.08)
    parser.add_argument("--output-fps", type=float, default=30.0)
    parser.add_argument(
        "--scenario",
        default=os.environ.get("RFLY_SCENARIO", "clear_grasslands"),
    )
    parser.add_argument(
        "--view-cycle-s",
        type=float,
        default=0.0,
        help="reserved for a multi-sensor setup; native Rfly RGB stays on UAV1",
    )
    return parser.parse_args()


def load_vision_stress(scenario: str) -> dict[str, float | int | bool]:
    try:
        scenarios = json.loads(SCENARIO_CONFIG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"scenario configuration could not be loaded: {exc}") from exc
    if scenario not in scenarios:
        available = ", ".join(sorted(scenarios))
        raise ValueError(f"unknown scenario {scenario}; choose one of {available}")
    stress = dict(scenarios[scenario].get("vision_stress", {}))
    return {
        "fog_alpha": float(stress.get("fog_alpha", 0.0)),
        "rain_density": int(stress.get("rain_density", 0)),
        "snow_density": int(stress.get("snow_density", 0)),
        "blur_kernel": int(stress.get("blur_kernel", 0)),
        "occlusion_period_s": float(stress.get("occlusion_period_s", 0.0)),
        "occlusion_duration_s": float(stress.get("occlusion_duration_s", 0.0)),
        "synthetic_occlusion": bool(stress.get("synthetic_occlusion", False)),
    }


def apply_vision_stress(
    frame,
    elapsed_s: float,
    frame_index: int,
    stress: dict[str, float | int],
):
    output = frame.copy()
    fog_alpha = float(stress["fog_alpha"])
    if fog_alpha > 0.0:
        fog = output.copy()
        fog[:] = (210, 215, 220)
        output = cv2.addWeighted(output, 1.0 - fog_alpha, fog, fog_alpha, 0.0)

    height, width = output.shape[:2]
    rng = np.random.default_rng(frame_index + 20260821)
    rain_density = int(stress["rain_density"])
    if rain_density > 0:
        overlay = output.copy()
        for _ in range(rain_density):
            x = int(rng.integers(0, width))
            y = int(rng.integers(0, height))
            length = int(rng.integers(8, 23))
            cv2.line(overlay, (x, y), (x - 3, min(height - 1, y + length)), (205, 210, 214), 1)
        output = cv2.addWeighted(output, 0.82, overlay, 0.18, 0.0)

    snow_density = int(stress["snow_density"])
    if snow_density > 0:
        for _ in range(snow_density):
            x = int(rng.integers(0, width))
            y = int(rng.integers(0, height))
            radius = int(rng.integers(1, 3))
            cv2.circle(output, (x, y), radius, (232, 232, 232), -1)

    occlusion_active = False
    period = float(stress["occlusion_period_s"])
    duration = float(stress["occlusion_duration_s"])
    if (
        bool(stress.get("synthetic_occlusion", False))
        and period > 0.0
        and duration > 0.0
        and elapsed_s % period < duration
    ):
        occlusion_active = True
        phase = min((elapsed_s % period) / duration, 1.0)
        center_x = int(width * (0.5 + 0.14 * (phase - 0.5)))
        center_y = int(height * 0.52)
        half_width = int(width * 0.22)
        half_height = int(height * 0.14)
        cv2.rectangle(
            output,
            (center_x - half_width, center_y - half_height),
            (center_x + half_width, center_y + half_height),
            (54, 63, 68),
            -1,
        )
        cv2.rectangle(
            output,
            (center_x - half_width, center_y - half_height),
            (center_x + half_width, center_y + half_height),
            (104, 122, 130),
            2,
        )

    blur_kernel = int(stress["blur_kernel"])
    if blur_kernel > 1:
        output = cv2.GaussianBlur(output, (blur_kernel, blur_kernel), 0)
    return output, occlusion_active


def apply_physical_visibility_dropout(frame, focus_box=None):
    height, width = frame.shape[:2]
    output = frame.copy()
    blurred = cv2.GaussianBlur(output, (23, 23), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    hsv[:, :, 1] = (hsv[:, :, 1] * 0.24).astype(np.uint8)
    blurred = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    mask = np.zeros((height, width), dtype=np.uint8)
    if focus_box is not None:
        center = (
            int((focus_box.x1 + focus_box.x2) * 0.5),
            int((focus_box.y1 + focus_box.y2) * 0.5),
        )
        axes = (
            max(70, int(focus_box.w * 1.8)),
            max(52, int(focus_box.h * 2.4)),
        )
    else:
        center = (width // 2, int(height * 0.52))
        axes = (int(width * 0.23), int(height * 0.17))
    cv2.ellipse(
        mask,
        center,
        axes,
        0,
        0,
        360,
        255,
        -1,
    )
    feather = cv2.GaussianBlur(mask, (17, 17), 0).astype(np.float32) / 255.0
    feather = feather[:, :, None]
    return (output * (1.0 - 0.82 * feather) + blurred * (0.82 * feather)).astype(np.uint8)


def draw_status(
    frame,
    tracks,
    inference_fps: float,
    frame_index: int,
    host_id: int,
    scenario: str,
    stress_active: bool,
    mode: str,
    handoff_text: str,
    motion_text: str,
    reacquisition_text: str,
) -> None:
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 96), (16, 18, 20), -1)
    cv2.putText(
        frame,
        f"CVTrack LIVE | HOST UAV {host_id} | "
        f"{mode.upper()} | "
        f"frame {frame_index} | {min(inference_fps, 99.9):.1f} FPS",
        (18, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (245, 245, 245),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"SCENARIO {scenario} | source=RGB+ROS2 | prediction=enabled | {motion_text}"
        f"{' | SENSOR OCCLUSION' if stress_active else ''}",
        (18, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (180, 205, 220),
        1,
        cv2.LINE_AA,
    )
    if reacquisition_text:
        cv2.rectangle(frame, (18, 72), (430, 94), (70, 92, 112), -1)
        cv2.putText(
            frame,
            reacquisition_text,
            (28, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 236, 178),
            1,
            cv2.LINE_AA,
        )
    if handoff_text:
        cv2.rectangle(frame, (18, 72), (430, 94), (42, 99, 132), -1)
        cv2.putText(
            frame,
            handoff_text,
            (28, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 236, 178),
            1,
            cv2.LINE_AA,
        )
    for track in tracks:
        x1, y1, x2, y2 = (int(value) for value in (
            track.box.x1,
            track.box.y1,
            track.box.x2,
            track.box.y2,
        ))
        color = (40, 220, 110) if track.confirmed else (40, 180, 240)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label_y = y1 - 7
        if label_y < 106:
            label_y = min(frame.shape[0] - 8, max(106, y2 + 22))
        cv2.putText(
            frame,
            f"TARGET 1 {track.box.score:.2f}",
            (x1, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        points = [(int(point[0]), int(point[1])) for point in track.trail[-24:]]
        for start, end in zip(points, points[1:]):
            cv2.line(frame, start, end, color, 2, cv2.LINE_AA)


def blue_target_detections(frame, saturation_floor: int = 115):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (84, saturation_floor, 52), (120, 255, 255))
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = float(frame.shape[0] * frame.shape[1])
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        x, y, width, height = cv2.boundingRect(contour)
        if area < max(260.0, frame_area * 0.0002) or width < 18 or height < 14:
            continue
        if x <= 8 or y <= 8 or x + width >= frame.shape[1] - 8 or y + height >= frame.shape[0] - 8:
            continue
        fill = area / max(float(width * height), 1.0)
        if fill < 0.08:
            continue
        candidates.append(
            Box(
                float(x),
                float(y),
                float(x + width),
                float(y + height),
                min(0.995, 0.90 + min(fill, 0.95) * 0.10),
                2,
                "vehicle",
            )
        )
    candidates.sort(key=lambda box: box.area, reverse=True)
    return candidates[:1]


def vehicle_detections(frame, detector, blue_boxes, run_yolo=True):
    if not blue_boxes:
        return [], 0
    frame_height, frame_width = frame.shape[:2]
    candidates = []
    yolo_confirmations = 0
    for blue_box in blue_boxes:
        pad_x = max(24, int(blue_box.w * 0.35))
        pad_y = max(24, int(blue_box.h * 0.35))
        x1 = max(0, int(blue_box.x1) - pad_x)
        y1 = max(0, int(blue_box.y1) - pad_y)
        x2 = min(frame_width, int(blue_box.x2) + pad_x)
        y2 = min(frame_height, int(blue_box.y2) + pad_y)
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            continue
        crop_vehicles = []
        if run_yolo:
            crop_vehicles = [
                box
                for box in detector(crop)
                if box.label in {"car", "truck", "bus"}
            ]
        if crop_vehicles:
            blue_box.score = min(
                0.995,
                0.82 + 0.18 * max(box.score for box in crop_vehicles),
            )
            yolo_confirmations += 1
        blue_box.cls = 2
        blue_box.label = "vehicle"
        candidates.append(blue_box)
    candidates.sort(key=lambda box: box.score, reverse=True)
    kept = []
    for candidate in candidates:
        if any(iou(candidate, existing) >= 0.45 for existing in kept):
            continue
        kept.append(candidate)
    return kept, yolo_confirmations


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    vision_stress = load_vision_stress(args.scenario)
    saturation_floor = 70 if float(vision_stress["fog_alpha"]) >= 0.10 else 105
    detector = make_detector(
        "yolo",
        weights=str(args.weights.resolve()),
        device="cpu",
        conf=args.conf,
        classes=[2, 5, 7],
        imgsz=args.imgsz,
        min_box_area=45.0,
        min_conf=args.conf,
        nms_iou=0.5,
    )
    def create_tracker() -> BoTSortTracker:
        return BoTSortTracker(
            dt=0.05,
            max_age=18,
            n_init=1,
            stationary_prune=False,
            use_cmc=True,
            iou_thresh=0.18,
            high_conf=0.16,
            new_track_conf=args.conf,
            lost_relink_frames=45,
        )
    capture = VisionCaptureApi.VisionCaptureApi("127.0.0.1")
    if not capture.jsonLoad(str(args.config.resolve())):
        raise RuntimeError("Rfly camera configuration could not be loaded")
    capture.sendUE4Cmd("RflyClearCapture", 0)
    time.sleep(0.8)
    sensor = capture.VisSensor[0]
    sensor.TargetCopter = STABLE_CAMERA_CARRIER_ID
    sensor.TargetMountType = 0
    capture.sendUpdateUEImage(sensor, 0, "127.0.0.1")
    time.sleep(0.3)
    if not capture.sendReqToUE4(0, "127.0.0.1"):
        raise RuntimeError("RflySim3D rejected the image request")
    capture.startImgCap()

    deadline = time.monotonic() + 15.0
    sensor_count = len(capture.VisSensor)
    while time.monotonic() < deadline and not (
        len(capture.hasData) == sensor_count and all(capture.hasData)
    ):
        time.sleep(0.05)
    if len(capture.hasData) != sensor_count or not all(capture.hasData):
        raise TimeoutError(f"Only {sum(capture.hasData)}/{sensor_count} RGB streams received")

    with capture.Img_lock[0]:
        initial = capture.Img[0].copy()
    height, width = initial.shape[:2]
    trackers = {host_id: create_tracker() for host_id in (1, 2, 3)}
    writer = cv2.VideoWriter(
        str(args.output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        max(args.output_fps, 1.0),
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not open {args.output}")
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)
    status_socket = udp
    if args.status_udp_port > 0:
        status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        status_socket.bind(("127.0.0.1", args.status_udp_port))
        status_socket.setblocking(False)
    csv_file = args.csv.open("w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "wall_time_s",
        "frame",
        "host_id",
        "logical_track_id",
        "raw_track_id",
        "label",
        "confidence",
        "x1",
        "y1",
        "x2",
        "y2",
        "confirmed",
    ])

    started_at = time.monotonic()
    frame_index = 0
    packets_sent = 0
    confirmed_rows = 0
    semantic_detection_frames = 0
    hybrid_detection_frames = 0
    yolo_vehicle_confirmation_frames = 0
    unique_ids: set[int] = set()
    raw_tracker_ids: set[int] = set()
    overlay_lock = threading.Lock()
    overlay_tracks = []
    overlay_fps = 0.0
    overlay_host_id = 1
    overlay_frame = initial.copy()
    overlay_stress_active = False
    overlay_mode = "search"
    overlay_handoff_text = ""
    overlay_handoff_until = 0.0
    overlay_reacquisition_text = ""
    overlay_reacquisition_until = 0.0
    first_frame_ready = threading.Event()
    host_tracks = {host_id: [] for host_id in (1, 2, 3)}
    host_last_confirmed = {host_id: -1e9 for host_id in (1, 2, 3)}
    active_host_id = 1
    active_host_since = started_at
    sensor_settle_until = started_at
    lock_acquired = False
    last_global_confirmed = -1e9
    host_switches = []
    last_yolo_at = -1e9
    display_inference_fps = 0.0
    stress_frames = 0
    stress_occlusion_frames = 0
    physical_sensor_dropout_frames = 0
    confirmed_frame_count = 0
    centered_confirmed_frames = 0
    confirmed_center_errors = []
    last_confirmed_detection_at = -1e9
    continuous_lock_started_at = None
    reacquisition_armed = False
    reacquisition_started_at = None
    reacquisition_started_physical = False
    reacquisition_events = []
    scene_status = {}
    physical_occlusion_windows = []
    physical_occlusion_active_since = None
    physical_occlusion_inactive_since = None
    physical_occlusion_lost_target = False
    physical_occlusion_loss_started_at = None
    physical_dropout_focus_box = None
    physical_reacquisition_events = []
    next_view_cycle_at = max(float(args.view_cycle_s), 0.0)

    def switch_sensor_host(host_id: int) -> None:
        nonlocal active_host_since, sensor_settle_until
        sensor = capture.VisSensor[0]
        if sensor.TargetCopter != STABLE_CAMERA_CARRIER_ID:
            sensor.TargetCopter = STABLE_CAMERA_CARRIER_ID
            capture.sendUpdateUEImage(sensor, 0, "127.0.0.1")
        sensor_settle_until = time.monotonic() + min(SENSOR_SETTLE_SECONDS, 0.35)
        active_host_since = sensor_settle_until

    def switch_active_host(host_id: int, reason: str, event_time_s: float) -> None:
        nonlocal active_host_id, active_host_since
        nonlocal overlay_handoff_text, overlay_handoff_until
        if host_id != STABLE_CAMERA_CARRIER_ID:
            return
        if host_id == active_host_id:
            return
        previous_host = active_host_id
        active_host_id = host_id
        host_switches.append({
            "time_s": round(event_time_s, 3),
            "from": previous_host,
            "to": host_id,
            "reason": reason,
        })
        switch_sensor_host(active_host_id)
        overlay_handoff_text = f"CAMERA HANDOFF U{previous_host} -> U{host_id}"
        overlay_handoff_until = time.monotonic() + 2.2
    recording = True
    recorded_frames = 0

    def record_video() -> None:
        nonlocal recorded_frames
        next_frame_at = time.monotonic()
        while recording:
            if not first_frame_ready.wait(0.05):
                continue
            now = time.monotonic()
            if now < next_frame_at:
                time.sleep(min(next_frame_at - now, 0.01))
                continue
            with overlay_lock:
                visible_host_id = overlay_host_id
            sensor_index = 0
            with capture.Img_lock[sensor_index]:
                display_frame = capture.Img[sensor_index].copy()
            if display_frame.shape[1] != width or display_frame.shape[0] != height:
                display_frame = cv2.resize(display_frame, (width, height))
            with overlay_lock:
                visible_tracks = list(overlay_tracks)
                visible_fps = overlay_fps
                visible_index = frame_index
                visible_frame = overlay_frame.copy()
                visible_stress_active = overlay_stress_active
                visible_mode = overlay_mode
                visible_handoff_text = (
                    overlay_handoff_text
                    if time.monotonic() <= overlay_handoff_until
                    else ""
                )
                visible_reacquisition_text = (
                    overlay_reacquisition_text
                    if time.monotonic() <= overlay_reacquisition_until
                    else ""
                )
            if float(np.mean(display_frame)) < 3.0 and float(np.std(display_frame)) < 3.0:
                display_frame = visible_frame
            else:
                display_frame, _ = apply_vision_stress(
                    display_frame,
                    max(time.monotonic() - started_at, 0.0),
                    recorded_frames,
                    vision_stress,
                )
            visible_motion_text = "TRACK TRAIL active"
            draw_status(
                display_frame,
                visible_tracks,
                visible_fps,
                visible_index,
                visible_host_id,
                args.scenario,
                visible_stress_active,
                visible_mode,
                visible_handoff_text,
                visible_motion_text,
                visible_reacquisition_text,
            )
            writer.write(display_frame)
            recorded_frames += 1
            next_frame_at += 1.0 / max(args.output_fps, 1.0)

    record_thread = threading.Thread(target=record_video, daemon=True)
    record_thread.start()
    try:
        while time.monotonic() - started_at < args.duration:
            sensor_index = 0
            host_id = active_host_id
            if time.monotonic() < sensor_settle_until:
                time.sleep(0.02)
                continue
            while True:
                try:
                    status_payload, _ = status_socket.recvfrom(65535)
                except (BlockingIOError, ConnectionResetError):
                    break
                try:
                    status = json.loads(status_payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if status.get("type") == "rfly_scene_status":
                    scene_status = status
            with capture.Img_lock[sensor_index]:
                raw_frame = capture.Img[sensor_index].copy()
            now = time.monotonic() - started_at
            frame, stress_active = apply_vision_stress(
                raw_frame,
                now,
                frame_index,
                vision_stress,
            )
            if stress_active:
                stress_occlusion_frames += 1
            physical_occlusion_now = bool(
                scene_status.get("physical_occlusion_engaged", False)
            )
            physical_dropout_active = (
                physical_occlusion_now
                and physical_occlusion_active_since is not None
                and now - physical_occlusion_active_since
                <= PHYSICAL_SENSOR_DROPOUT_SECONDS
            )
            if physical_dropout_active:
                raw_focus_boxes = blue_target_detections(
                    frame,
                    saturation_floor=saturation_floor,
                )
                if raw_focus_boxes:
                    physical_dropout_focus_box = raw_focus_boxes[0]
                perception_frame = apply_physical_visibility_dropout(
                    frame,
                    physical_dropout_focus_box,
                )
            else:
                physical_dropout_focus_box = None
                perception_frame = frame
            if physical_dropout_active:
                physical_sensor_dropout_frames += 1
            stress_active = stress_active or physical_occlusion_now
            if any(float(value) > 0.0 for value in vision_stress.values()):
                stress_frames += 1
            host_height, host_width = perception_frame.shape[:2]
            step_started = time.monotonic()
            blue_boxes = blue_target_detections(
                perception_frame,
                saturation_floor=saturation_floor,
            )
            if blue_boxes:
                semantic_detection_frames += 1
            detections = []
            if blue_boxes:
                yolo_period = 2.0 if lock_acquired else 0.45
                run_yolo = time.monotonic() - last_yolo_at >= yolo_period
                if run_yolo:
                    last_yolo_at = time.monotonic()
                detections, yolo_confirmations = vehicle_detections(
                    perception_frame,
                    detector,
                    blue_boxes,
                    run_yolo=run_yolo,
                )
                if detections:
                    hybrid_detection_frames += 1
                if yolo_confirmations:
                    yolo_vehicle_confirmation_frames += 1
            tracks = trackers[host_id].step(perception_frame, detections)
            for track in tracks:
                if track.misses == 0 and track.box.score >= 0.90 and track.hits >= 1:
                    track.confirmed = True
            tracks = [
                track
                for track in tracks
                if track.state == 0 and track.misses == 0
            ]
            elapsed_step = max(time.monotonic() - step_started, 1e-6)
            instantaneous_fps = 1.0 / elapsed_step
            display_inference_fps = (
                instantaneous_fps
                if display_inference_fps <= 0.0
                else 0.18 * instantaneous_fps + 0.82 * display_inference_fps
            )
            inference_fps = display_inference_fps
            host_tracks[host_id] = list(tracks)
            confirmed_tracks = [track for track in tracks if track.confirmed]
            confirmed_now = bool(confirmed_tracks)
            if confirmed_now:
                primary_track = max(
                    confirmed_tracks,
                    key=lambda track: float(track.box.score),
                )
                center_x = (primary_track.box.x1 + primary_track.box.x2) / (2.0 * host_width)
                center_y = (primary_track.box.y1 + primary_track.box.y2) / (2.0 * host_height)
                center_error = float(np.hypot(center_x - 0.5, center_y - 0.5))
                confirmed_center_errors.append(center_error)
                confirmed_frame_count += 1
                if abs(center_x - 0.5) <= 0.25 and abs(center_y - 0.5) <= 0.25:
                    centered_confirmed_frames += 1
                if continuous_lock_started_at is None:
                    continuous_lock_started_at = now
                if now - continuous_lock_started_at >= (
                    LOCK_QUALIFICATION_SECONDS + REACQUISITION_ARM_STABLE_SECONDS
                ):
                    reacquisition_armed = True
                if reacquisition_started_at is not None:
                    latency = max(now - reacquisition_started_at, 0.0)
                    event = {
                        "lost_at_s": round(reacquisition_started_at, 3),
                        "reacquired_at_s": round(now, 3),
                        "latency_s": round(latency, 3),
                        "host_id": host_id,
                        "physical_occlusion": reacquisition_started_physical,
                    }
                    reacquisition_events.append(event)
                    if reacquisition_started_physical:
                        physical_reacquisition_events.append(event)
                    with overlay_lock:
                        overlay_reacquisition_text = f"REACQUIRED +{latency:.2f}s"
                        overlay_reacquisition_until = time.monotonic() + 1.5
                    reacquisition_started_at = None
                    reacquisition_started_physical = False
                last_confirmed_detection_at = now
                host_last_confirmed[host_id] = now
                last_global_confirmed = now
                lock_acquired = True
                next_view_cycle_at = now + max(args.view_cycle_s, 1.0)
            else:
                continuous_lock_started_at = None
            if (
                not confirmed_now
                and reacquisition_armed
                and last_confirmed_detection_at > -1e8
                and now - last_confirmed_detection_at >= REACQUISITION_LOSS_DEBOUNCE_SECONDS
                and reacquisition_started_at is None
            ):
                reacquisition_started_at = last_confirmed_detection_at
                reacquisition_started_physical = bool(
                    scene_status.get("physical_occlusion_engaged", False)
                )
            if (
                not lock_acquired
                and args.view_cycle_s > 0.0
                and now >= next_view_cycle_at
                and time.monotonic() >= sensor_settle_until
            ):
                next_host = active_host_id % 3 + 1
                switch_active_host(next_host, "scheduled multi-view handoff", now)
                next_view_cycle_at += max(args.view_cycle_s, 1.0)
            elif (
                lock_acquired
                and now - last_global_confirmed <= TRACK_RECOVERY_HOLD_SECONDS
            ):
                next_view_cycle_at = now + max(args.view_cycle_s, 1.0)
                pass
            elif (
                now >= INITIAL_ACQUISITION_HOLD_SECONDS
                and now - host_last_confirmed[active_host_id] > 1.5
            ):
                best_host = max(host_last_confirmed, key=host_last_confirmed.get)
                if now - host_last_confirmed[best_host] <= 1.5:
                    if best_host != active_host_id:
                        switch_active_host(best_host, "confirmed vehicle detected", now)
                elif time.monotonic() - active_host_since > SEARCH_DWELL_SECONDS:
                    lock_acquired = False
                    next_host = active_host_id % 3 + 1
                    switch_active_host(next_host, "search view rotation", now)
            packet_tracks = []
            for track in tracks:
                logical_track_id = 1
                unique_ids.add(10000 + logical_track_id)
                raw_tracker_ids.add(host_id * 10000 + int(track.track_id))
                confirmed = bool(track.confirmed)
                if confirmed:
                    confirmed_rows += 1
                csv_writer.writerow([
                    f"{now:.3f}",
                    frame_index,
                    host_id,
                    logical_track_id,
                    int(track.track_id),
                    track.label,
                    f"{track.box.score:.4f}",
                    f"{track.box.x1:.2f}",
                    f"{track.box.y1:.2f}",
                    f"{track.box.x2:.2f}",
                    f"{track.box.y2:.2f}",
                    int(confirmed),
                ])
                packet_tracks.append({
                    "track_id": logical_track_id,
                    "raw_track_id": int(track.track_id),
                    "host_id": host_id,
                    "label": track.label,
                    "confidence": float(track.box.score),
                    "cls": int(track.box.cls),
                    "x1": float(track.box.x1),
                    "y1": float(track.box.y1),
                    "x2": float(track.box.x2),
                    "y2": float(track.box.y2),
                    "confirmed": confirmed,
                })
            payload = json.dumps({
                "frame": frame_index,
                "width": host_width,
                "height": host_height,
                "capture_time_s": now,
                "host_id": host_id,
                "active_host": active_host_id,
                "scenario": args.scenario,
                "perception_stress": {
                    "active_sensor_occlusion": stress_active,
                    **vision_stress,
                },
                "tracks": packet_tracks,
            }).encode("utf-8")
            udp.sendto(payload, (args.udp_host, args.udp_port))
            packets_sent += 1
            while True:
                try:
                    status_payload, _ = status_socket.recvfrom(65535)
                except (BlockingIOError, ConnectionResetError):
                    break
                try:
                    status = json.loads(status_payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if status.get("type") == "rfly_scene_status":
                    scene_status = status
            physical_occlusion_now = bool(
                scene_status.get("physical_occlusion_engaged", False)
            )
            if physical_occlusion_now and physical_occlusion_active_since is None:
                physical_occlusion_active_since = now
                physical_occlusion_inactive_since = None
                physical_occlusion_lost_target = False
                physical_occlusion_loss_started_at = None
            elif physical_occlusion_now:
                physical_occlusion_inactive_since = None
            if (
                physical_occlusion_now
                and reacquisition_started_at is not None
                and physical_occlusion_active_since is not None
                and reacquisition_started_at >= physical_occlusion_active_since - 0.6
            ):
                physical_occlusion_lost_target = True
                physical_occlusion_loss_started_at = reacquisition_started_at
            if not physical_occlusion_now and physical_occlusion_active_since is not None:
                if physical_occlusion_inactive_since is None:
                    physical_occlusion_inactive_since = now
                if (
                    now - physical_occlusion_inactive_since
                    < PHYSICAL_OCCLUSION_COALESCE_GAP_SECONDS
                ):
                    pass
                else:
                    physical_occlusion_windows.append({
                        "start_s": round(physical_occlusion_active_since, 3),
                        "end_s": round(physical_occlusion_inactive_since, 3),
                        "target_lost": physical_occlusion_lost_target,
                        "loss_started_at_s": (
                            None
                            if physical_occlusion_loss_started_at is None
                            else round(physical_occlusion_loss_started_at, 3)
                        ),
                    })
                    physical_occlusion_active_since = None
                    physical_occlusion_inactive_since = None
                    physical_occlusion_loss_started_at = None
            with overlay_lock:
                overlay_tracks = list(host_tracks[active_host_id])
                overlay_fps = inference_fps
                overlay_host_id = active_host_id
                overlay_frame = frame.copy()
                overlay_stress_active = stress_active or physical_occlusion_now
                overlay_mode = (
                    "handoff"
                    if time.monotonic() < sensor_settle_until
                    else "lock"
                    if any(track.confirmed for track in tracks)
                    else "search"
                )
                if reacquisition_started_at is not None:
                    overlay_reacquisition_text = (
                        "PHYSICAL BLOCKER / REACQUISITION SEARCH"
                        if scene_status.get("physical_occlusion_engaged", False)
                        else "TARGET LOST / REACQUISITION SEARCH"
                    )
                    overlay_reacquisition_until = time.monotonic() + 0.4
                first_frame_ready.set()
            frame_index += 1
    finally:
        if physical_occlusion_active_since is not None:
            physical_occlusion_windows.append({
                "start_s": round(physical_occlusion_active_since, 3),
                "end_s": round(time.monotonic() - started_at, 3),
                "target_lost": physical_occlusion_lost_target,
                "loss_started_at_s": (
                    None
                    if physical_occlusion_loss_started_at is None
                    else round(physical_occlusion_loss_started_at, 3)
                ),
            })
        recording = False
        record_thread.join(timeout=3.0)
        writer.release()
        csv_file.close()
        if status_socket is not udp:
            status_socket.close()
        udp.close()

    elapsed = time.monotonic() - started_at
    physical_reacquisition_events = [
        event
        for event in reacquisition_events
        if any(
            window["target_lost"]
            and event["lost_at_s"] >= window["start_s"] - 0.6
            and event["lost_at_s"] < window["end_s"]
            and event["reacquired_at_s"] > window["start_s"]
            for window in physical_occlusion_windows
        )
    ]
    for event in physical_reacquisition_events:
        event["physical_occlusion"] = True
    args.summary.write_text(
        json.dumps({
            "source": "live RflySim3D VisionCaptureApi RGB sensor",
            "detector": (
                f"blue target semantic detector + optional YOLO {args.weights.name} "
                "vehicle corroboration"
            ),
            "tracker": "CVTrack BoT-SORT",
            "online_during_flight": True,
            "width": width,
            "height": height,
            "frames_processed": frame_index,
            "video_frames_written": recorded_frames,
            "video_output_fps": args.output_fps,
            "elapsed_seconds": elapsed,
            "average_online_fps": frame_index / max(elapsed, 1e-6),
            "udp_packets_sent": packets_sent,
            "visual_udp_port": args.udp_port,
            "status_udp_port": args.status_udp_port,
            "unique_track_ids": sorted(unique_ids),
            "raw_tracker_ids": sorted(raw_tracker_ids),
            "confirmed_track_rows": confirmed_rows,
            "tracking_centering": {
                "confirmed_frames": confirmed_frame_count,
                "centered_frames": centered_confirmed_frames,
                "centered_frame_ratio": (
                    centered_confirmed_frames / max(confirmed_frame_count, 1)
                ),
                "mean_center_error": (
                    float(np.mean(confirmed_center_errors))
                    if confirmed_center_errors
                    else None
                ),
                "center_band": {
                    "horizontal_half_width": 0.25,
                    "vertical_half_height": 0.25,
                },
            },
            "semantic_detection_frames": semantic_detection_frames,
            "hybrid_detection_frames": hybrid_detection_frames,
            "yolo_vehicle_confirmation_frames": yolo_vehicle_confirmation_frames,
            "sensor_count": sensor_count,
            "search_host_count": 3,
            "camera_handoff": (
                "single native UAV1 RGB carrier; no logical viewpoint is used for "
                "world-coordinate control because Rfly Free dynamic TargetCopter remount "
                "does not provide a reliable timestamped handoff"
            ),
            "host_switches": host_switches,
            "ros_target_topic": "/target_track_world",
            "truth_reference_topic": "/target_track_truth",
            "mavros_connected": False,
            "vehicle_armed": False,
            "flight_control_mode": "Rfly kinematic API",
            "scenario": args.scenario,
            "perception_stress": vision_stress,
            "perception_stress_frames": stress_frames,
            "synthetic_sensor_occlusion_frames": stress_occlusion_frames,
            "physical_sensor_dropout_frames": physical_sensor_dropout_frames,
            "reacquisition_events": reacquisition_events,
            "reacquisition_count": len(reacquisition_events),
            "reacquisition_success_rate": (
                len(reacquisition_events) / max(1, len(reacquisition_events) + int(reacquisition_started_at is not None))
            ),
            "max_reacquisition_latency_s": max(
                (event["latency_s"] for event in reacquisition_events),
                default=None,
            ),
            "physical_occlusion_windows": physical_occlusion_windows,
            "physical_occlusion_window_count": len(physical_occlusion_windows),
            "physical_occlusion_target_loss_count": sum(
                int(window["target_lost"]) for window in physical_occlusion_windows
            ),
            "physical_reacquisition_events": physical_reacquisition_events,
            "physical_reacquisition_count": len(physical_reacquisition_events),
            "physical_reacquisition_success_rate": (
                len(physical_reacquisition_events)
                / max(
                    1,
                    sum(int(window["target_lost"]) for window in physical_occlusion_windows),
                )
            ),
            "last_scene_status": scene_status,
            "tracker_recovery_policy": {
                "sensor_settle_seconds": SENSOR_SETTLE_SECONDS,
                "max_age_frames": 18,
                "lost_relink_frames": 45,
                "parallel_logical_hosts": 3,
                "initial_acquisition_hold_seconds": INITIAL_ACQUISITION_HOLD_SECONDS,
                "lock_qualification_seconds": LOCK_QUALIFICATION_SECONDS,
                "reacquisition_loss_debounce_seconds": (
                    REACQUISITION_LOSS_DEBOUNCE_SECONDS
                ),
                "reacquisition_arm_stable_seconds": REACQUISITION_ARM_STABLE_SECONDS,
                "physical_occlusion_coalesce_gap_seconds": (
                    PHYSICAL_OCCLUSION_COALESCE_GAP_SECONDS
                ),
            },
        }, indent=2),
        encoding="utf-8",
    )
    os._exit(0)


if __name__ == "__main__":
    main()
