#!/usr/bin/env python3
"""Capture native Rfly vehicle candidates from one overhead RGB view."""

from __future__ import annotations

import argparse
import json
import sys
import time
import types
from pathlib import Path

import cv2
import numpy as np


RFLY_SDK = Path(r"F:\RflySimAPIs\RflySimSDK")
for sdk_path in (RFLY_SDK, RFLY_SDK / "ctrl", RFLY_SDK / "ue", RFLY_SDK / "vision"):
    if str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))

try:
    import open3d  # noqa: F401
except ImportError:
    open3d_show_stub = types.ModuleType("Open3DShow")
    open3d_show_stub.Open3DShow = object
    sys.modules["Open3DShow"] = open3d_show_stub

import UE4CtrlAPI  # noqa: E402
import VisionCaptureApi  # noqa: E402


CANDIDATES = (
    (50, "standard_car_blue"),
    (10000444, "east_luv"),
    (10000814, "vision_qr_car"),
    (10000825, "match_car"),
    (10000888, "east_luv_large"),
    (51, "neutral_ground_car"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--map", default="3DDisplay")
    parser.add_argument("--warmup", type=float, default=4.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    ue = UE4CtrlAPI.UE4CtrlAPI("127.0.0.1")
    capture = VisionCaptureApi.VisionCaptureApi("127.0.0.1")
    ue.sendUE4Cmd(f"RflyChangeMapbyName {args.map}", 0)
    ue.sendUE4Cmd("r.setres 1280x720w", 0)
    ue.sendUE4Cmd("t.MaxFPS 45", 0)
    time.sleep(max(args.warmup, 1.0))
    ue.sendUE4Cmd("RflyClearCapture", 0)
    ue.sendUE4PosNew(
        1,
        3,
        [110.0, 110.0, -92.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [700.0] * 8,
        0.0,
        0,
    )
    ue.sendUE4Cmd("RflyCameraPosAng 110 110 -92 0 -90 0", 0)
    time.sleep(1.0)
    if not capture.jsonLoad(str(args.config.resolve())):
        raise RuntimeError(f"could not load {args.config}")
    sensor = capture.VisSensor[0]
    sensor.TargetCopter = 1
    sensor.TargetMountType = 0
    sensor.SensorPosXYZ = [0.0, 0.0, 0.0]
    sensor.SensorAngEular = [0.0, -90.0, 0.0]
    sensor.CameraFOV = 90.0
    capture.sendUpdateUEImage(sensor, 0, "127.0.0.1")
    time.sleep(0.3)
    if not capture.sendReqToUE4(0, "127.0.0.1"):
        raise RuntimeError("RflySim3D rejected the RGB request")
    capture.startImgCap()
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if capture.hasData and capture.hasData[0]:
            break
        time.sleep(0.05)
    if not capture.hasData or not capture.hasData[0]:
        raise TimeoutError("Rfly RGB frame did not arrive")
    time.sleep(1.0)
    with capture.Img_lock[0]:
        baseline = capture.Img[0].copy()
    cv2.imwrite(str(args.output / "baseline.png"), baseline)
    rows = []
    for index, (vehicle_type, label) in enumerate(CANDIDATES):
        entity_id = 900 + index
        ue.sendUE4PosScale2Ground(
            entity_id,
            vehicle_type,
            0.0,
            [110.0, 110.0, 0.0],
            [0.0, 0.0, 0.0],
            [3.2, 3.2, 3.2],
            0,
        )
        time.sleep(1.0)
        with capture.Img_lock[0]:
            frame = capture.Img[0].copy()
        difference = cv2.absdiff(frame, baseline)
        difference_gray = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)
        _, difference_mask = cv2.threshold(difference_gray, 24, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(
            difference_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if contours:
            contour = max(contours, key=cv2.contourArea)
            x, y, width, height = cv2.boundingRect(contour)
        else:
            x = y = width = height = 0
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, (84, 85, 42), (130, 255, 255))
        object_mask = cv2.bitwise_and(blue, difference_mask)
        blue_pixels = int(np.count_nonzero(object_mask))
        blue_coverage = blue_pixels / max(int(np.count_nonzero(difference_mask)), 1)
        image_path = args.output / f"{index:02d}_{label}.png"
        diff_path = args.output / f"{index:02d}_{label}_diff.png"
        cv2.putText(frame, f"{label} ({vehicle_type})", (24, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite(str(image_path), frame)
        cv2.imwrite(str(diff_path), difference)
        rows.append({
            "vehicle_type": vehicle_type,
            "label": label,
            "image": str(image_path),
            "diff_pixels": int(np.count_nonzero(difference_mask)),
            "diff_bbox": [x, y, width, height],
            "blue_pixels_in_vehicle": blue_pixels,
            "blue_vehicle_coverage": round(blue_coverage, 4),
        })
        ue.sendUE4Destroy(entity_id, 0)
        time.sleep(0.4)
    image_path = args.output / "vehicle_candidates.json"
    (args.output / "vehicle_candidates.json").write_text(
        json.dumps({"map": args.map, "candidates": rows}, indent=2),
        encoding="utf-8",
    )
    print(image_path)


if __name__ == "__main__":
    main()
