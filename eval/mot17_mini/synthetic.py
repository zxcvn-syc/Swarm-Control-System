"""Synthetic ground truth generator for eval/mot17_mini/run_eval.py self-test.

Produces a 100-frame 640x360 video with 3 moving rectangles of known identity.
Used by `run_eval.py --synthetic` to verify the harness without external data.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


def make_synthetic(
    out_dir: Path,
    n_frames: int = 100,
    width: int = 640,
    height: int = 360,
) -> Tuple[Path, Path]:
    """Write a synthetic video and matching ground-truth file to ``out_dir``.

    Returns the (video_path, gt_path) pair.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    video_path = out_dir / "synthetic.mp4"
    gt_path = out_dir / "gt.txt"

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(video_path), fourcc, 30.0, (width, height))
    rng = np.random.default_rng(0)

    trajectories = [
        # (track_id, label, x0, y0, vx, vy, side)
        (1, "car", 50.0, 100.0, 3.0, 0.4, 24),
        (2, "car", 200.0, 60.0, -2.0, 0.6, 28),
        (3, "car", 100.0, 250.0, 1.5, -0.7, 22),
    ]

    gt_rows: List[str] = []
    for frame_idx in range(n_frames):
        img = np.full((height, width, 3), 64, dtype=np.uint8)
        noise = rng.integers(-2, 2, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        for tid, label, x0, y0, vx, vy, side in trajectories:
            cx = x0 + vx * frame_idx
            cy = y0 + vy * frame_idx
            x1, y1 = cx - side / 2, cy - side / 2
            x2, y2 = cx + side / 2, cy + side / 2
            cv2.rectangle(
                img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), -1
            )
            # MOT-style row: frame, track_id, x, y, w, h, conf, class, visibility
            gt_rows.append(
                f"{frame_idx+1},{tid},{x1:.2f},{y1:.2f},{side:.2f},{side:.2f},"
                f"1,{1 if label == 'pedestrian' else 1},1"
            )
        writer.write(img)
    writer.release()
    gt_path.write_text("\n".join(gt_rows) + "\n")
    return video_path, gt_path


def _main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", default="eval/mot17_mini/data/_synthetic")
    ap.add_argument("--frames", type=int, default=100)
    args = ap.parse_args()
    make_synthetic(Path(args.out_dir), n_frames=args.frames)
    print(f"wrote {args.out_dir}/synthetic.mp4 and gt.txt")


if __name__ == "__main__":
    _main()
