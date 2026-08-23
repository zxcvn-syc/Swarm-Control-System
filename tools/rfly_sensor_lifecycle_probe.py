#!/usr/bin/env python3
"""Verify that a live Rfly RGB sensor advances while its carrier moves."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
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

    class Open3DUnavailable:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("open3d is required only for point-cloud display")

    open3d_show_stub.Open3DShow = Open3DUnavailable
    sys.modules["Open3DShow"] = open3d_show_stub

import UE4CtrlAPI  # noqa: E402
import VisionCaptureApi  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    return parser.parse_args()


def wait_for_frame(capture: VisionCaptureApi.VisionCaptureApi, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if capture.hasData and capture.hasData[0]:
            return
        time.sleep(0.05)
    raise TimeoutError("Rfly RGB frame did not arrive")


def frame_snapshot(capture: VisionCaptureApi.VisionCaptureApi) -> tuple[np.ndarray, float]:
    with capture.Img_lock[0]:
        return capture.Img[0].copy(), float(capture.imgStmp[0])


def main() -> None:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    ue = UE4CtrlAPI.UE4CtrlAPI("127.0.0.1")
    capture = VisionCaptureApi.VisionCaptureApi("127.0.0.1")

    ue.sendUE4Cmd("RflyChangeMapbyName Grasslands", 0)
    ue.sendUE4Cmd("r.setres 1280x720w", 0)
    ue.sendUE4Cmd("t.MaxFPS 45", 0)
    time.sleep(args.warmup)
    ue.sendUE4Cmd("RflyClearCapture", 0)
    ue.sendUE4PosNew(
        1,
        3,
        [0.0, 0.0, -60.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [800.0] * 8,
        0.0,
        0,
    )
    time.sleep(0.8)

    if not capture.jsonLoad(str(args.config.resolve())):
        raise RuntimeError(f"could not load Rfly config: {args.config}")
    sensor = capture.VisSensor[0]
    sensor.TargetCopter = 1
    sensor.TargetMountType = 0
    capture.sendUpdateUEImage(sensor, 0, "127.0.0.1")
    time.sleep(0.2)
    if not capture.sendReqToUE4(0, "127.0.0.1"):
        raise RuntimeError("Rfly rejected the RGB request")
    capture.startImgCap()
    wait_for_frame(capture, 15.0)

    started_at = time.monotonic()
    samples: list[dict[str, float | str]] = []
    previous_gray: np.ndarray | None = None
    first_frame: np.ndarray | None = None
    last_frame: np.ndarray | None = None
    next_sample_at = started_at
    while time.monotonic() - started_at < args.duration:
        elapsed_s = time.monotonic() - started_at
        x = 18.0 * math.sin(0.55 * elapsed_s)
        y = 12.0 * math.sin(0.31 * elapsed_s)
        yaw = math.atan2(0.31 * 12.0 * math.cos(0.31 * elapsed_s), 0.55 * 18.0 * math.cos(0.55 * elapsed_s))
        ue.sendUE4PosNew(
            1,
            3,
            [x, y, -60.0 + 2.0 * math.sin(0.8 * elapsed_s)],
            [0.0, 0.0, yaw],
            [0.0, 0.0, 0.0],
            [800.0] * 8,
            elapsed_s,
            0,
        )
        if time.monotonic() < next_sample_at:
            time.sleep(0.01)
            continue
        frame, image_timestamp = frame_snapshot(capture)
        if first_frame is None:
            first_frame = frame.copy()
        last_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        difference = 0.0 if previous_gray is None else float(cv2.absdiff(gray, previous_gray).mean())
        samples.append({
            "elapsed_s": round(elapsed_s, 3),
            "image_timestamp": image_timestamp,
            "sha256": hashlib.sha256(frame.tobytes()).hexdigest(),
            "mean_difference": difference,
            "brightness": float(gray.mean()),
        })
        previous_gray = gray
        next_sample_at += 0.1

    if first_frame is not None:
        cv2.imwrite(str(args.output / "first_frame.jpg"), first_frame)
    if last_frame is not None:
        cv2.imwrite(str(args.output / "last_frame.jpg"), last_frame)
    differences = [float(sample["mean_difference"]) for sample in samples[1:]]
    hashes = {str(sample["sha256"]) for sample in samples}
    timestamps = [float(sample["image_timestamp"]) for sample in samples]
    result = {
        "sample_count": len(samples),
        "unique_frame_hashes": len(hashes),
        "image_timestamp_range": [min(timestamps, default=0.0), max(timestamps, default=0.0)],
        "mean_frame_difference": float(np.mean(differences)) if differences else 0.0,
        "moving_sample_ratio": (
            sum(value > 0.8 for value in differences) / max(len(differences), 1)
        ),
        "passed": len(hashes) >= 8 and any(value > 0.8 for value in differences),
        "samples": samples,
    }
    (args.output / "probe_summary.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
