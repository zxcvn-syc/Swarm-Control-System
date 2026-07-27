"""Core data types shared across the package."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

import numpy as np


# Constants for numerical stability
_EPS = 1e-9
_INF = float("inf")
_MIN_SPEED_THRESHOLD = 1e-6
_MIN_VARIANCE = 1e-9
_MAX_TRAJECTORY_LENGTH = 1000


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

    def __post_init__(self) -> None:
        """Validate and normalize box coordinates."""
        # Ensure coordinates are finite
        if not (math.isfinite(self.x1) and math.isfinite(self.y1) and
                math.isfinite(self.x2) and math.isfinite(self.y2)):
            raise ValueError(f"Box coordinates must be finite: x1={self.x1}, y1={self.y1}, x2={self.x2}, y2={self.y2}")
        if not math.isfinite(self.score):
            self.score = 0.0
        if not math.isfinite(self.cls):
            self.cls = 0
        # Normalize: ensure x1 <= x2, y1 <= y2
        if self.x1 > self.x2:
            self.x1, self.x2 = self.x2, self.x1
        if self.y1 > self.y2:
            self.y1, self.y2 = self.y2, self.y1

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
        return max(self.x2 - self.x1, _MIN_VARIANCE)

    @property
    def h(self) -> float:
        return max(self.y2 - self.y1, _MIN_VARIANCE)

    @property
    def area(self) -> float:
        w, h = self.wh
        return max(w, 0.0) * max(h, 0.0)

    @property
    def aspect(self) -> float:
        """height / width (>=1 if landscape)."""
        w, h = self.wh
        if w <= _MIN_VARIANCE:
            return _INF
        return max(h, _MIN_VARIANCE) / max(w, _MIN_VARIANCE)

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
        if union <= 0:
            return 0.0
        return inter / max(union, _MIN_VARIANCE)

    def clip_to_bounds(self, width: float, height: float) -> "Box":
        """Clip box coordinates to image boundaries.

        Args:
            width: Image width (maximum x coordinate + 1)
            height: Image height (maximum y coordinate + 1)

        Returns:
            New Box clipped to [0, width) x [0, height)
        """
        return Box(
            x1=max(0.0, min(self.x1, width)),
            y1=max(0.0, min(self.y1, height)),
            x2=max(0.0, min(self.x2, width)),
            y2=max(0.0, min(self.y2, height)),
            score=self.score,
            cls=self.cls,
            label=self.label,
        )

    def scale(self, scale_x: float, scale_y: float) -> "Box":
        """Scale box by given factors.

        Args:
            scale_x: Horizontal scale factor
            scale_y: Vertical scale factor

        Returns:
            New scaled Box
        """
        if scale_x <= 0 or scale_y <= 0:
            raise ValueError(f"Scale factors must be positive: scale_x={scale_x}, scale_y={scale_y}")
        return Box(
            x1=self.x1 * scale_x,
            y1=self.y1 * scale_y,
            x2=self.x2 * scale_x,
            y2=self.y2 * scale_y,
            score=self.score,
            cls=self.cls,
            label=self.label,
        )


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

    Enhanced with trajectory prediction capabilities.
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

    # Enhanced trajectory prediction fields
    predicted_future: List[Tuple[float, float, float, float]] = field(default_factory=list)
    prediction_confidence: float = 1.0
    motion_mode: str = "unknown"  # "stationary", "slow", "fast"
    is_anomaly: bool = False

    # Velocity history for acceleration computation
    _velocity_history: List[Tuple[float, float]] = field(default_factory=list)

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
        self._update_velocity_history()
        self._trim_trail_if_needed()
        if self.recent_hits >= self.n_init:
            self.confirmed = True

    def _trim_trail_if_needed(self) -> None:
        """Trim trajectory history if it exceeds maximum length."""
        if len(self.trail) > _MAX_TRAJECTORY_LENGTH:
            trim_count = len(self.trail) - _MAX_TRAJECTORY_LENGTH
            self.trail = self.trail[trim_count:]
            self.pred_trail = self.pred_trail[trim_count:]
            self.trail_scores = self.trail_scores[trim_count:]
        if len(self._velocity_history) > _MAX_TRAJECTORY_LENGTH:
            trim_count = len(self._velocity_history) - _MAX_TRAJECTORY_LENGTH
            self._velocity_history = self._velocity_history[trim_count:]

    def _update_velocity_history(self) -> None:
        """Update velocity history for acceleration computation."""
        vx, vy = self._velocity_xy()
        self._velocity_history.append((vx, vy))
        if len(self._velocity_history) > _MAX_TRAJECTORY_LENGTH:
            self._velocity_history = self._velocity_history[-_MAX_TRAJECTORY_LENGTH:]

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

    def update_trajectory_prediction(
        self,
        kf: Any,
        n_steps: int = 10,
        min_confidence: float = 0.1,
        confidence_decay: float = 0.9,
    ) -> None:
        """Update predicted future trajectory using Kalman filter.

        Args:
            kf: Kalman filter instance (must have predict_n_steps_with_uncertainty)
            n_steps: Number of future steps to predict
            min_confidence: Minimum confidence threshold
            confidence_decay: Confidence decay factor per step
        """
        self.predicted_future = []

        if n_steps <= 0:
            return

        cur_conf = 1.0
        mean = np.array(self.mean, dtype=np.float64, copy=True)
        cov = np.array(self.cov, dtype=np.float64, copy=True)

        for _ in range(n_steps):
            if cur_conf < min_confidence:
                break

            mean, cov = kf.predict(mean, cov)
            pred_x = float(mean[0])
            pred_y = float(mean[1])

            if len(mean) >= 4:
                std_x = float(np.sqrt(max(cov[0, 0], _MIN_VARIANCE)))
                std_y = float(np.sqrt(max(cov[1, 1], _MIN_VARIANCE)))
            else:
                std_x, std_y = 5.0, 5.0

            self.predicted_future.append((pred_x, pred_y, std_x, std_y))
            cur_conf *= confidence_decay

        self.prediction_confidence = cur_conf

    def _velocity_xy(self) -> Tuple[float, float]:
        """Return (vx, vy) honouring the KF state dimensionality.

        4-state KF  : mean = [cx, cy, vx, vy]
        8-state KF  : mean = [cx, cy, w, h, vx, vy, vw, vh]
        Falls back to zero velocity when neither shape applies.
        """
        if len(self.mean) >= 8:
            return float(self.mean[4]), float(self.mean[5])
        if len(self.mean) >= 4:
            return float(self.mean[2]), float(self.mean[3])
        return 0.0, 0.0

    def detect_motion_mode(self, speed_threshold_slow: float = 2.0,
                          speed_threshold_fast: float = 20.0,
                          smoothing_window: int = 5) -> str:
        """Detect current motion mode based on velocity with temporal smoothing.

        Uses a moving average of velocities over the smoothing window to avoid
        jitter from single-frame detection anomalies.

        Args:
            speed_threshold_slow: Speed below which is considered stationary
            speed_threshold_fast: Speed above which is considered fast
            smoothing_window: Number of recent velocity samples to average

        Returns:
            Motion mode: "stationary", "slow", "fast", or "unknown"
        """
        # Use velocity history for smoothing if available
        if len(self._velocity_history) >= smoothing_window:
            recent_velocities = self._velocity_history[-smoothing_window:]
            avg_vx = sum(v[0] for v in recent_velocities) / len(recent_velocities)
            avg_vy = sum(v[1] for v in recent_velocities) / len(recent_velocities)
            speed = float(np.sqrt(avg_vx * avg_vx + avg_vy * avg_vy))
        else:
            # Fall back to current velocity
            vx, vy = self._velocity_xy()
            speed = float(np.sqrt(vx * vx + vy * vy))

        # Apply thresholds with boundary checking
        if speed_threshold_slow <= 0:
            speed_threshold_slow = 2.0
        if speed_threshold_fast <= speed_threshold_slow:
            speed_threshold_fast = speed_threshold_slow * 10.0

        if speed < speed_threshold_slow:
            self.motion_mode = "stationary"
        elif speed > speed_threshold_fast:
            self.motion_mode = "fast"
        else:
            self.motion_mode = "slow"

        return self.motion_mode

    def get_speed(self) -> float:
        """Get current speed (velocity magnitude)."""
        vx, vy = self._velocity_xy()
        return float(np.sqrt(vx * vx + vy * vy))

    def get_position_uncertainty(self) -> Tuple[float, float]:
        """Get position uncertainty (std_x, std_y) from covariance matrix.

        Returns:
            Tuple of (std_x, std_y) position standard deviations.
            Returns (10.0, 10.0) as default when covariance is invalid.
        """
        try:
            if self.cov is None or len(self.cov) < 2:
                return 10.0, 10.0

            # Handle both 2D and multi-dimensional covariance
            cov_array = np.asarray(self.cov, dtype=np.float64)
            if cov_array.ndim != 2 or cov_array.shape[0] < 2 or cov_array.shape[1] < 2:
                return 10.0, 10.0

            var_x = float(cov_array[0, 0])
            var_y = float(cov_array[1, 1])

            # Check for valid variances
            if not math.isfinite(var_x) or var_x < 0:
                var_x = _MIN_VARIANCE
            if not math.isfinite(var_y) or var_y < 0:
                var_y = _MIN_VARIANCE

            std_x = float(np.sqrt(max(var_x, _MIN_VARIANCE)))
            std_y = float(np.sqrt(max(var_y, _MIN_VARIANCE)))

            return std_x, std_y
        except (IndexError, TypeError, ValueError):
            return 10.0, 10.0

    def get_acceleration(self) -> Tuple[float, float]:
        """Compute acceleration (ax, ay) from velocity history.

        Uses finite difference approximation. Requires at least 2 velocity
        samples in history.

        Returns:
            Tuple of (ax, ay) acceleration components, or (0.0, 0.0) if
            insufficient data.
        """
        if len(self._velocity_history) < 2:
            return 0.0, 0.0

        # Get last two velocities
        v1_x, v1_y = self._velocity_history[-2]
        v2_x, v2_y = self._velocity_history[-1]

        # Assume dt = 1 frame for acceleration computation
        ax = v2_x - v1_x
        ay = v2_y - v1_y

        return float(ax), float(ay)

    def get_trajectory_direction(self, lookback: int = 5) -> Tuple[float, float]:
        """Compute average trajectory direction vector.

        Args:
            lookback: Number of past positions to consider for direction.

        Returns:
            Tuple of (dx, dy) normalized direction vector, or (0.0, 0.0)
            if trajectory is too short or stationary.
        """
        min_lookback = 2
        actual_lookback = min(lookback, len(self.trail))

        if actual_lookback < min_lookback:
            return 0.0, 0.0

        # Get positions at start and end of lookback window
        start_idx = len(self.trail) - actual_lookback
        end_idx = len(self.trail) - 1

        x1, y1 = self.trail[start_idx]
        x2, y2 = self.trail[end_idx]

        dx = x2 - x1
        dy = y2 - y1
        magnitude = float(np.sqrt(dx * dx + dy * dy))

        if magnitude < _MIN_SPEED_THRESHOLD:
            return 0.0, 0.0

        # Normalize
        return dx / magnitude, dy / magnitude

    def is_stable(self, stability_threshold: float = 0.5,
                  min_hits: int = 5) -> bool:
        """Determine if track trajectory is stable.

        A track is considered stable if:
        1. It has at least min_hits confirmed updates
        2. Position uncertainty is below stability_threshold
        3. Motion score is relatively consistent (low variance)

        Args:
            stability_threshold: Maximum position stddev for stability (pixels)
            min_hits: Minimum number of hits to consider for stability

        Returns:
            True if track is stable, False otherwise.
        """
        # Must have sufficient hits
        if self.hits < min_hits:
            return False

        # Check position uncertainty
        std_x, std_y = self.get_position_uncertainty()
        if std_x > stability_threshold or std_y > stability_threshold:
            return False

        # Check motion consistency using trajectory variance
        if len(self.trail) >= 3:
            positions = np.array(self.trail[-min_hits:])
            variance_x = float(np.var(positions[:, 0])) if len(positions) > 1 else 0.0
            variance_y = float(np.var(positions[:, 1])) if len(positions) > 1 else 0.0

            # High variance indicates unstable trajectory
            if variance_x + variance_y > 1000.0:
                return False

        return True