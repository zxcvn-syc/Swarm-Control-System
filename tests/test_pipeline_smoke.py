"""End-to-end smoke test: build a tiny synthetic video and run pipeline on it."""

from __future__ import annotations

import csv
import os
import subprocess
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
        for vx, vy, color in [(2.5, 1.0, (0, 255, 0)), (-1.5, -0.7, (0, 0, 255))]:
            cx = int(50 + vx * i)
            cy = int(50 + vy * i)
            cv2.rectangle(img, (cx - 10, cy - 10), (cx + 10, cy + 10), color, -1)
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
    import sys as _sys

    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join([str(src_dir), env.get("PYTHONPATH", "")])

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
    assert res.returncode == 0, "stderr=" + res.stderr + " stdout=" + res.stdout
    # The output should at least contain tracks.csv
    csv_path = out / "tracks.csv"
    assert csv_path.exists(), f"no tracks.csv; stderr={res.stderr}\nstdout={res.stdout}"
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    # Either detector may produce 0 detections on this synthetic clip (MOG2 needs
    # a background plate) but the pipeline must complete without exception.
    assert isinstance(rows, list)


@pytest.mark.slow
def test_pipeline_respects_yaml_output_switches(tmp_path):
    src = tmp_path / "synth.mp4"
    out = tmp_path / "out"
    config = tmp_path / "output_config.yaml"
    _make_synthetic_video(src, n_frames=12)
    config.write_text(
        """extends: default
detector:
  backend: mog2
output:
  write_video: false
  write_csv: false
  write_smoothed_csv: true
  write_trails_json: true
  write_future_csv: true
viz:
  save_trail: false
  fps_overlay: false
pipeline:
  max_frames: 6
  predict_horizon: 3
"""
    )

    repo = Path(__file__).resolve().parents[1]
    import sys as _sys

    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join([str(repo / "src"), env.get("PYTHONPATH", "")])
    cmd = [
        _sys.executable,
        "-m",
        "cvtrack",
        "--config",
        str(config),
        "--source",
        str(src),
        "--out-dir",
        str(out),
        "--log-level",
        "WARNING",
    ]

    res = subprocess.run(cmd, env=env, capture_output=True, text=True, cwd=str(repo))

    assert res.returncode == 0, "stderr=" + res.stderr + " stdout=" + res.stdout
    assert not (out / "tracked.mp4").exists()
    assert not (out / "tracks.csv").exists()
    assert (out / "tracks_smoothed.csv").exists()
    assert (out / "tracks_trails.json").exists()
    assert (out / "tracks_future.csv").exists()


@pytest.mark.slow
def test_pipeline_writes_calibrated_world_csv(tmp_path):
    src = tmp_path / "synth.mp4"
    out = tmp_path / "out"
    calibration = tmp_path / "ground_plane.yaml"
    _make_synthetic_video(src, n_frames=20)
    calibration.write_text(
        """frame_id: test_map
units: m
image_points_px:
  - [0, 0]
  - [320, 0]
  - [320, 240]
  - [0, 240]
world_points_m:
  - [0, 0]
  - [32, 0]
  - [32, 24]
  - [0, 24]
"""
    )

    repo = Path(__file__).resolve().parents[1]
    import sys as _sys

    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join([str(repo / "src"), env.get("PYTHONPATH", "")])
    cmd = [
        _sys.executable,
        "-m",
        "cvtrack",
        "--source",
        str(src),
        "--out-dir",
        str(out),
        "--detector",
        "mog2",
        "--max-frames",
        "12",
        "--world-calibration",
        str(calibration),
        "--log-level",
        "WARNING",
    ]

    res = subprocess.run(cmd, env=env, capture_output=True, text=True, cwd=str(repo))
    assert res.returncode == 0, "stderr=" + res.stderr + " stdout=" + res.stdout
    world_csv = out / "tracks_world.csv"
    assert world_csv.exists()
    with world_csv.open() as f:
        reader = csv.DictReader(f)
        assert reader.fieldnames is not None
        assert "world_x_m" in reader.fieldnames
        assert "world_valid" in reader.fieldnames
        for row in reader:
            assert row["units"] == "m"
            assert row["frame_id"] == "test_map"
            if row["world_valid"] == "1":
                assert row["world_x_m"]
                assert row["world_y_m"]
