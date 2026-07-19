"""Core data types shared across the package."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Box:
    """Axis-aligned bounding box in pixel coordinates."""

    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    cls: int
    label: str

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def wh(self) -> Tuple[float, float]:
        return (self.x2 - self.x1, self.y2 - self.y1)

    @property
    def w(self) -> float:
        return max(self.x2 - self.x1, 1.0)

    @property
    def h(self) -> float:
        return max(self.y2 - self.y1, 1.0)

    @property
    def area(self) -> float:
        w, h = self.wh
        return max(w, 0.0) * max(h, 0.0)

    @property
    def aspect(self) -> float:
        """height / width (>=1 if landscape)."""
        w, h = self.wh
        if w <= 0:
            return float("inf")
        return max(h, 1e-3) / max(w, 1e-3)

    def iou(self, other: "Box") -> float:
        ix1 = max(self.x1, other.x1)
        iy1 = max(self.y1, other.y1)
        ix2 = min(self.x2, other.x2)
        iy2 = min(self.y2, other.y2)
        iw = max(ix2 - ix1, 0.0)
        ih = max(iy2 - iy1, 0.0)
        inter = iw * ih
        if inter <= 0:
            return 0.0
        union = self.area + other.area - inter
        return inter / max(union, 1e-6)


@dataclass
class Detection:
    """A detection plus its optional ReID embedding."""

    box: Box
    embedding: Optional[np.ndarray] = None  # shape (D,), L2-normalised


@dataclass
class Track:
    """A single tracked object across frames.

    Works with both 4-state (cx, cy, vx, vy) and 8-state BoT-SORT KF
    (cx, cy, w, h, vx, vy, vw, vh). The KF dimensionality is detected via
    `getattr(kf, "STATE_DIM", 4)`.
    """

    track_id: int
    label: str
    mean: np.ndarray
    cov: np.ndarray
    box: Box
    hits: int = 1
    recent_hits: int = 1
    age: int = 1
    misses: int = 0
    confirmed: bool = False
    state: int = 0  # 0=tracked, 1=lost, 2=removed
    lost_age: int = 0
    relink_remaining: int = 0
    birth_frame: Optional[int] = None
    n_init: int = 3
    motion_score: float = 0.0
    trail: List[Tuple[float, float]] = field(default_factory=list)
    pred_trail: List[Tuple[float, float]] = field(default_factory=list)
    trail_scores: List[float] = field(default_factory=list)
    # Set per-step by the BoT-SORT tracker so the renderer can flash a box.
    was_lost_before_update: bool = False
    # Running mean of the ReID embedding for this track (set by Gallery).
    # ``None`` means "no embeddings yet" and disables ReID scoring for the track.
    embedding_mean: Optional[np.ndarray] = None

    @property
    def pos(self) -> Tuple[float, float]:
        return float(self.mean[0]), float(self.mean[1])

    # ------------------------------------------------------------------
    # Lifecycle methods (preserved from v4)
    # ------------------------------------------------------------------
    def predict(self, kf) -> None:
        self.mean, self.cov = kf.predict(self.mean, self.cov)
        cx, cy = self.pos
        if getattr(kf, "STATE_DIM", 4) == 8:
            w = max(float(self.mean[2]), 1.0)
            h = max(float(self.mean[3]), 1.0)
        else:
            w, h = self.box.wh
        self.box = Box(
            cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2,
            self.box.score, self.box.cls, self.box.label,
        )
        self.age += 1
        if self.relink_remaining > 0:
            self.relink_remaining -= 1

    def update(self, kf, box: Box) -> None:
        if getattr(kf, "STATE_DIM", 4) == 8:
            z = np.array([box.cx, box.cy, box.w, box.h], dtype=np.float64)
        else:
            z = np.array([box.cx, box.cy], dtype=np.float64)
        self.mean, self.cov = kf.update(self.mean, self.cov, z)
        self.box = box
        self.label = box.label
        self.hits += 1
        self.misses = 0
        self.lost_age = 0
        self.state = 0
        self.recent_hits = min(self.n_init, self.recent_hits + 1)
        self.trail.append((box.cx, box.cy))
        self.pred_trail.append((float(self.mean[0]), float(self.mean[1])))
        self.trail_scores.append(box.score)
        self._update_motion_stats(box)
        if self.recent_hits >= self.n_init:
            self.confirmed = True

    def mark_missed(self) -> None:
        self.misses += 1
        self.lost_age += 1
        if self.lost_age >= 1 and self.confirmed:
            self.state = 1
        if self.misses % 2 == 0:
            self.recent_hits = max(1, self.recent_hits - 1)
        self.motion_score *= 0.5

    def _update_motion_stats(self, box: Box) -> None:
        prev_pos = self.trail[-2] if len(self.trail) >= 2 else None
        if prev_pos is not None:
            dx = box.cx - prev_pos[0]
            dy = box.cy - prev_pos[1]
            d2 = dx * dx + dy * dy
            self.motion_score = 0.7 * self.motion_score + 0.3 * d2