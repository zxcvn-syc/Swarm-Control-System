#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2


SDK_ROOT = Path(r"F:\RflySimAPIs\RflySimSDK")
for sdk_path in (SDK_ROOT, SDK_ROOT / "ctrl", SDK_ROOT / "ue", SDK_ROOT / "vision"):
    if str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))

try:
    import open3d  # noqa: F401
except ImportError:
    import types

    module = types.ModuleType("Open3DShow")
    module.Open3DShow = object
    sys.modules["Open3DShow"] = module

import VisionCaptureApi  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--pitches",
        type=float,
        nargs="+",
        default=[-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0],
    )
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--mount-type", type=int)
    parser.add_argument(
        "--target-copter",
        type=int,
        help="Rfly target vehicle ID; ignored by fixed-ground mount type 2",
    )
    parser.add_argument("--position", type=float, nargs=3)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    capture = VisionCaptureApi.VisionCaptureApi("127.0.0.1")
    if not capture.jsonLoad(str(args.config.resolve())):
        raise RuntimeError("Rfly camera configuration could not be loaded")
    capture.sendUE4Cmd("RflyClearCapture", 0)
    time.sleep(0.8)
    if not capture.sendReqToUE4(0, "127.0.0.1"):
        raise RuntimeError("RflySim3D rejected the image request")
    capture.startImgCap()
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and not all(capture.hasData):
        time.sleep(0.05)
    if not capture.hasData or not all(capture.hasData):
        raise TimeoutError("Rfly RGB stream did not arrive")
    sensor = capture.VisSensor[0]
    if args.target_copter is not None:
        sensor.TargetCopter = args.target_copter
    if args.mount_type is not None:
        sensor.TargetMountType = args.mount_type
    if args.position is not None:
        sensor.SensorPosXYZ = list(args.position)
    for pitch in args.pitches:
        sensor.SensorAngEular = [0.0, float(pitch), 0.0]
        capture.sendUpdateUEImage(sensor, 0, "127.0.0.1")
        time.sleep(max(args.settle_seconds, 0.1))
        with capture.Img_lock[0]:
            frame = capture.Img[0].copy()
        output = args.output_dir / f"pitch_{pitch:+05.0f}.png"
        if not cv2.imwrite(str(output), frame):
            raise RuntimeError(f"could not write {output}")
        print(output, flush=True)


if __name__ == "__main__":
    main()
