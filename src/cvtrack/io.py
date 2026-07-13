"""Video and CSV I/O helpers."""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class VideoInfo:
    path: str
    width: int
    height: int
    fps: float
    total_frames: int
    fourcc: str = "avc1"


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
    """cv2.VideoWriter that falls back from avc1 -> mp4v automatically."""

    def __init__(self, path: str, fps: float, size: Tuple[int, int]) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        w, h = size
        self.path = path
        self.fps = float(fps)
        for fourcc in ("avc1", "mp4v", "XVID"):
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