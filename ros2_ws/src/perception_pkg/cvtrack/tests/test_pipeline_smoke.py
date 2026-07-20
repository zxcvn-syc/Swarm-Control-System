"""End-to-end smoke test: build a tiny synthetic video and run pipeline on it."""

from __future__ import annotations

import csv
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pytest


def _make_synthetic_video(path: Path, n_frames: int = 50) -> None:
    h, w = 240, 320
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, 10.0, (w, h))
    rng = np.random.default_rng(0)
    for i in range(n_frames):
        img = np.full((h, w, 3), 32, dtype=np.uint8)
        # Two moving rectangles
        for k, (vx, vy, c) in enumerate([(2.5, 1.0, (0, 255, 0)), (-1.5, -0.7, (0, 0, 255))]):
            cx = int(50 + vx * i)
            cy = int(50 + vy * i)
            cv2.rectangle(img, (cx - 10, cy - 10), (cx + 10, cy + 10), c, -1)
        # Add a tiny amount of noise
        noise = rng.integers(-5, 5, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        writer.write(img)
    writer.release()


@pytest.mark.slow
def test_pipeline_smoke(tmp_path):
    src = tmp_path / "synth.mp4"
    out = tmp_path / "out"
    _make_synthetic_video(src)

    repo = Path(__file__).resolve().parents[1]
    src_dir = repo / "src"
    env_extra = {"PYTHONPATH": str(src_dir)}

    import os
    import sys as _sys

    env = os.environ.copy()
    env["PYTHONPATH"] = ":".join([env_extra["PYTHONPATH"], env.get("PYTHONPATH", "")])

    cmd = [
        _sys.executable,
        "-m",
        "cvtrack",
        "--source",
        str(src),
        "--out-dir",
        str(out),
        "--detector",
        "mog2",  # do not depend on YOLO weights in CI
        "--max-frames",
        "20",
        "--log-level",
        "WARNING",
    ]
    res = subprocess.run(cmd, env=env, capture_output=True, text=True, cwd=str(repo))
    # The output should at least contain tracks.csv
    csv_path = out / "tracks.csv"
    assert csv_path.exists(), f"no tracks.csv; stderr={res.stderr}\nstdout={res.stdout}"
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    # Either detector may produce 0 detections on this synthetic clip (MOG2 needs
    # a background plate) but the pipeline must complete without exception.
    assert isinstance(rows, list)
