#!/usr/bin/env python3
"""Fit the Rfly fixed-world RGB sensor ground-plane mapping from known markers."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
import types
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np


RFLY_SDK = Path(os.environ.get("RFLY_SDK_ROOT", "F:/RflySimAPIs/RflySimSDK"))
for sdk_path in (RFLY_SDK, RFLY_SDK / "ctrl", RFLY_SDK / "ue", RFLY_SDK / "vision"):
    if str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))

try:
    import open3d  # noqa: F401
except ImportError:
    open3d_stub = types.ModuleType("Open3DShow")
    open3d_stub.Open3DShow = object
    sys.modules["Open3DShow"] = open3d_stub

import VisionCaptureApi  # noqa: E402
from ue.UE4CtrlAPI import UE4CtrlAPI  # noqa: E402


TARGET_ID = 101
TARGET_VEHICLE_TYPE = 50
CAMERA_FOV_DEG = 90.0


class UdpForwarder:
    def __init__(self, socket_obj: socket.socket, host: str, port: int) -> None:
        self.socket_obj = socket_obj
        self.host = host
        self.port = port

    def sendto(self, data: bytes, _address: tuple[str, int]) -> int:
        return self.socket_obj.sendto(data, (self.host, self.port))


@dataclass
class Sample:
    camera_x: float
    camera_y: float
    camera_z: float
    target_x: float
    target_y: float
    image_x: float | None
    image_y: float | None
    box_width: float | None
    box_height: float | None
    accepted: bool
    image: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--bridge-host", default="192.168.88.1")
    parser.add_argument("--bridge-port", type=int, default=30010)
    parser.add_argument("--settle-seconds", type=float, default=0.85)
    parser.add_argument(
        "--remote-marker-sweep",
        action="store_true",
        help="observe markers sent by tools/rfly_remote_marker_sweep.py on the VM",
    )
    parser.add_argument(
        "--initial-wait-seconds",
        type=float,
        default=0.0,
        help="wait after RGB initialization for the remote marker sweep to begin",
    )
    return parser.parse_args()


def find_blue_target(frame: np.ndarray) -> tuple[float, float, float, float] | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((92, 75, 40)), np.array((132, 255, 255)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        area = width * height
        if area < 80 or width < 8 or height < 5:
            continue
        candidates.append((area, x, y, width, height))
    if not candidates:
        return None
    _area, x, y, width, height = max(candidates)
    return x + width / 2.0, y + height / 2.0, float(width), float(height)


def feature_row(sample: Sample, width: int, height: int) -> list[float]:
    if sample.image_x is None or sample.image_y is None:
        raise ValueError("sample has no image detection")
    altitude = abs(sample.camera_z)
    focal = height / (2.0 * np.tan(np.radians(CAMERA_FOV_DEG) / 2.0))
    image_ground_x = (sample.image_x - width / 2.0) * altitude / focal
    image_ground_y = (sample.image_y - height / 2.0) * altitude / focal
    return [1.0, sample.camera_x, sample.camera_y, image_ground_x, image_ground_y]


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    capture = VisionCaptureApi.VisionCaptureApi("127.0.0.1")
    if not capture.jsonLoad(str(args.config.resolve())):
        raise RuntimeError("Rfly camera configuration could not be loaded")
    capture.sendUE4Cmd("RflyClearCapture", 0)
    time.sleep(0.8)
    sensor = capture.VisSensor[0]
    sensor.TargetMountType = 2
    sensor.SensorAngEular = [0.0, -90.0, 0.0]
    sensor.CameraFOV = CAMERA_FOV_DEG
    capture.sendUpdateUEImage(sensor, 0, "127.0.0.1")
    if not capture.sendReqToUE4(0, "127.0.0.1"):
        raise RuntimeError("RflySim3D rejected the RGB request")
    capture.startImgCap()
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and not all(capture.hasData):
        time.sleep(0.05)
    if not capture.hasData or not all(capture.hasData):
        raise TimeoutError("Rfly RGB sensor did not deliver an image")

    ue = None
    if not args.remote_marker_sweep:
        ue = UE4CtrlAPI("192.168.88.1")
        ue.udp_socket = UdpForwarder(ue.udp_socket, args.bridge_host, args.bridge_port)
        ue.sendUE4Destroy(TARGET_ID, 0)

    # The target offsets cover the usable image plane while all positions remain inside
    # the 220 m demo map.  Multiple camera locations make axis and translation errors
    # observable rather than baking them into a single frame origin.
    camera_poses = (
        (42.0, 42.0, -76.0),
        (112.0, 42.0, -76.0),
        (42.0, 112.0, -62.0),
        (112.0, 112.0, -62.0),
    )
    offsets = ((-30.0, -24.0), (0.0, -24.0), (30.0, -24.0), (-30.0, 22.0), (0.0, 22.0), (30.0, 22.0))
    samples: list[Sample] = []
    frame_shape: tuple[int, int] | None = None
    try:
        if args.initial_wait_seconds > 0.0:
            time.sleep(args.initial_wait_seconds)
        for camera_index, camera_pose in enumerate(camera_poses):
            sensor.SensorPosXYZ = list(camera_pose)
            sensor.SensorAngEular = [0.0, -90.0, 0.0]
            capture.sendUpdateUEImage(sensor, 0, "127.0.0.1")
            time.sleep(args.settle_seconds)
            for offset_index, (offset_x, offset_y) in enumerate(offsets):
                target_x = camera_pose[0] + offset_x
                target_y = camera_pose[1] + offset_y
                if ue is not None:
                    ue.sendUE4PosScale2Ground(
                        TARGET_ID,
                        TARGET_VEHICLE_TYPE,
                        0.0,
                        [target_x, target_y, 0.0],
                        [0.0, 0.0, 0.0],
                        [3.0, 3.0, 3.0],
                        windowID=0,
                    )
                time.sleep(args.settle_seconds)
                with capture.Img_lock[0]:
                    frame = capture.Img[0].copy()
                frame_shape = frame.shape[:2]
                detection = find_blue_target(frame)
                image_name = f"camera_{camera_index}_marker_{offset_index}.png"
                annotated = frame.copy()
                if detection is not None:
                    center_x, center_y, box_width, box_height = detection
                    cv2.rectangle(
                        annotated,
                        (int(center_x - box_width / 2), int(center_y - box_height / 2)),
                        (int(center_x + box_width / 2), int(center_y + box_height / 2)),
                        (0, 255, 0),
                        2,
                    )
                    cv2.circle(annotated, (int(center_x), int(center_y)), 4, (0, 255, 255), -1)
                cv2.putText(
                    annotated,
                    f"camera=({camera_pose[0]:.0f},{camera_pose[1]:.0f},{camera_pose[2]:.0f}) target=({target_x:.0f},{target_y:.0f})",
                    (18, 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.62,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imwrite(str(args.output_dir / image_name), annotated)
                samples.append(Sample(
                    camera_x=camera_pose[0],
                    camera_y=camera_pose[1],
                    camera_z=camera_pose[2],
                    target_x=target_x,
                    target_y=target_y,
                    image_x=None if detection is None else detection[0],
                    image_y=None if detection is None else detection[1],
                    box_width=None if detection is None else detection[2],
                    box_height=None if detection is None else detection[3],
                    accepted=detection is not None,
                    image=image_name,
                ))
                print(f"{image_name}: {'detected' if detection else 'missing'}", flush=True)
    finally:
        if ue is not None:
            ue.sendUE4Destroy(TARGET_ID, 0)

    if frame_shape is None:
        raise RuntimeError("no calibration frames were captured")
    height, width = frame_shape
    accepted = [sample for sample in samples if sample.accepted]
    if len(accepted) < 12:
        raise RuntimeError(f"only {len(accepted)} usable detections; need at least 12")
    matrix = np.asarray([feature_row(sample, width, height) for sample in accepted], dtype=float)
    targets_x = np.asarray([sample.target_x for sample in accepted], dtype=float)
    targets_y = np.asarray([sample.target_y for sample in accepted], dtype=float)
    coefficients_x, *_ = np.linalg.lstsq(matrix, targets_x, rcond=None)
    coefficients_y, *_ = np.linalg.lstsq(matrix, targets_y, rcond=None)
    predicted_x = matrix @ coefficients_x
    predicted_y = matrix @ coefficients_y
    residuals = np.hypot(predicted_x - targets_x, predicted_y - targets_y)
    calibration = {
        "schema": "rfly_world_sensor_affine_v1",
        "camera_fov_deg": CAMERA_FOV_DEG,
        "image_size_px": {"width": width, "height": height},
        "feature_order": ["bias", "sensor_x", "sensor_y", "image_ground_x", "image_ground_y"],
        "world_x_coefficients": [float(value) for value in coefficients_x],
        "world_y_coefficients": [float(value) for value in coefficients_y],
        "accepted_samples": len(accepted),
        "total_samples": len(samples),
        "reprojection_error_m": {
            "mean": float(np.mean(residuals)),
            "median": float(np.median(residuals)),
            "max": float(np.max(residuals)),
        },
        "samples": [asdict(sample) for sample in samples],
    }
    (args.output_dir / "samples.json").write_text(json.dumps(calibration, indent=2), encoding="utf-8")
    (args.output_dir / "camera_ground_calibration.json").write_text(json.dumps(calibration, indent=2), encoding="utf-8")
    print(json.dumps(calibration["reprojection_error_m"], indent=2), flush=True)


if __name__ == "__main__":
    main()
