"""Detector benchmark harness.

Repeatedly run the configured detector over a video and report per-frame
latency stats (mean, p50, p95, fps).  Useful for comparing YOLOv8s vs MOG2
on a given clip, or for measuring the cost of different image sizes.

Usage:
    python scripts/benchmark.py --source clip.mp4 --detector mog2 --frames 200
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Make src/ importable when running directly.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from cvtrack.detector.factory import build_detector  # noqa: E402


def _main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", required=True)
    ap.add_argument("--detector", default="mog2",
                    choices=("auto", "yolo", "mog2"))
    ap.add_argument("--weights", default=None)
    ap.add_argument("--imgsz", type=int, default=480)
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--warmup", type=int, default=10)
    args = ap.parse_args()

    logging.basicConfig(level=logging.WARNING, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    cap = cv2.VideoCapture(args.source)
    if not cap.isOpened():
        print(f"could not open {args.source}")
        return 1

    det = build_detector(
        backend=args.detector,
        weights=args.weights,
        imgsz=args.imgsz,
        conf=0.15,
        min_conf=0.05,
        min_box_area=60.0,
        classes=[0, 1, 2, 3, 5, 7],
    )

    # Warm-up: load weights, etc.
    ok, frame = cap.read()
    if not ok:
        print("empty video")
        return 1
    for _ in range(args.warmup):
        _ = det(frame)

    latencies: list[float] = []
    frames_seen = 0
    while frames_seen < args.frames:
        ok, frame = cap.read()
        if not ok:
            break
        t0 = time.perf_counter()
        _ = det(frame)
        latencies.append(time.perf_counter() - t0)
        frames_seen += 1

    if not latencies:
        print("no frames processed")
        return 1

    arr = np.array(latencies)
    fps = 1.0 / arr.mean()
    p50, p95 = np.percentile(arr, [50, 95])
    print(f"\nDetector: {args.detector}  weights={args.weights}  imgsz={args.imgsz}")
    print(f"Frames processed: {frames_seen}")
    print(f"Mean latency: {arr.mean()*1000:.1f} ms  ({fps:.2f} fps)")
    print(f"p50 latency : {p50*1000:.1f} ms")
    print(f"p95 latency : {p95*1000:.1f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
