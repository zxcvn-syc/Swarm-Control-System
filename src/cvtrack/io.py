"""Video and CSV I/O helpers."""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class VideoInfo:
    path: str
    width: int
    height: int
    fps: float
    total_frames: int
    fourcc: str = "mp4v"


class VideoReader:
    """Thin wrapper around cv2.VideoCapture with explicit frame capping."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open video: {path}")
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 20.0
        self.fps = float(fps) if fps > 0 else 20.0
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0

    def info(self) -> VideoInfo:
        return VideoInfo(
            path=self.path,
            width=self.width,
            height=self.height,
            fps=self.fps,
            total_frames=self.total_frames,
        )

    def set_pos(self, frame_idx: int) -> None:
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, int(frame_idx)))

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        return self.cap.read()

    def close(self) -> None:
        self.cap.release()


class VideoWriter:
    """cv2.VideoWriter that prefers the broadly available mp4v codec."""

    def __init__(self, path: str, fps: float, size: Tuple[int, int]) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        w, h = size
        self.path = path
        self.fps = float(fps)
        for fourcc in ("mp4v", "avc1", "XVID"):
            w_ = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), self.fps, (w, h))
            if w_.isOpened():
                self.writer = w_
                self.fourcc = fourcc
                return
        raise RuntimeError(f"failed to create VideoWriter at {path}")

    def write(self, frame: np.ndarray) -> None:
        self.writer.write(frame)

    def close(self) -> None:
        self.writer.release()


class FutureTrailCsvWriter:
    """CSV writer for per-frame, multi-step Kalman future positions.

    Schema (v6):

        frame, track_id, future_step, future_frame,
        future_x, future_y, sigma_x, sigma_y

    ``sigma_x`` / ``sigma_y`` are populated when ``write_trail_with_cov`` is
    used (the position stddev along x/y from the projected KF covariance);
    ``write_trail`` leaves them blank for backward compatibility with the
    v5 schema (frame, track_id, future_step, future_frame, future_x,
    future_y).
    """

    HEADER = [
        "frame",
        "track_id",
        "future_step",
        "future_frame",
        "future_x",
        "future_y",
        "sigma_x",
        "sigma_y",
    ]

    def __init__(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(self.HEADER)

    def write_trail(
        self,
        frame: int,
        track_id: int,
        points: List[Tuple[float, float]],
    ) -> None:
        for step, (future_x, future_y) in enumerate(points, start=1):
            self.w.writerow([
                int(frame),
                int(track_id),
                step,
                int(frame) + step,
                f"{future_x:.2f}",
                f"{future_y:.2f}",
                "",
                "",
            ])

    def write_trail_with_cov(
        self,
        frame: int,
        track_id: int,
        points: List[Tuple[float, float, float, float]],
    ) -> None:
        """Like ``write_trail`` but accepts ``(x, y, sigma_x, sigma_y)`` tuples."""
        for step, (future_x, future_y, sigma_x, sigma_y) in enumerate(points, start=1):
            self.w.writerow([
                int(frame),
                int(track_id),
                step,
                int(frame) + step,
                f"{future_x:.2f}",
                f"{future_y:.2f}",
                f"{sigma_x:.2f}",
                f"{sigma_y:.2f}",
            ])

    def close(self) -> None:
        self.f.close()


class TrackCsvWriter:
    """Buffered CSV writer for per-frame track rows.

    Header is the same as the legacy v4 script so existing tools keep working.
    """

    HEADER = [
        "frame",
        "track_id",
        "label",
        "cx",
        "cy",
        "vx",
        "vy",
        "confirmed",
    ]

    def __init__(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(self.HEADER)

    def write_row(
        self,
        frame: int,
        track_id: int,
        label: str,
        cx: float,
        cy: float,
        vx: float,
        vy: float,
        confirmed: bool,
    ) -> None:
        self.w.writerow([
            int(frame), int(track_id), label,
            f"{cx:.2f}", f"{cy:.2f}",
            f"{vx:.3f}", f"{vy:.3f}",
            int(bool(confirmed)),
        ])

    def close(self) -> None:
        self.f.close()


class WorldTrackCsvWriter:
    """CSV writer for calibrated ground-plane tracks consumed by ROS adapters.

    Pixel-space ``tracks.csv`` remains unchanged for compatibility.  This
    separate file makes metre units and the coordinate frame explicit so a
    planner cannot mistake image coordinates for a physical position.
    """

    HEADER = [
        "frame",
        "timestamp_s",
        "track_id",
        "label",
        "image_x_px",
        "image_y_px",
        "world_x_m",
        "world_y_m",
        "world_vx_mps",
        "world_vy_mps",
        "world_valid",
        "frame_id",
        "units",
    ]

    def __init__(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(self.HEADER)

    def write_row(
        self,
        frame: int,
        timestamp_s: float,
        track_id: int,
        label: str,
        image_x_px: float,
        image_y_px: float,
        world_x_m: Optional[float],
        world_y_m: Optional[float],
        world_vx_mps: Optional[float],
        world_vy_mps: Optional[float],
        world_valid: bool,
        frame_id: str,
    ) -> None:
        self.w.writerow([
            int(frame),
            f"{timestamp_s:.6f}",
            int(track_id),
            label,
            f"{image_x_px:.2f}",
            f"{image_y_px:.2f}",
            "" if world_x_m is None else f"{world_x_m:.4f}",
            "" if world_y_m is None else f"{world_y_m:.4f}",
            "" if world_vx_mps is None else f"{world_vx_mps:.4f}",
            "" if world_vy_mps is None else f"{world_vy_mps:.4f}",
            int(bool(world_valid)),
            frame_id,
            "m",
        ])

    def close(self) -> None:
        self.f.close()
