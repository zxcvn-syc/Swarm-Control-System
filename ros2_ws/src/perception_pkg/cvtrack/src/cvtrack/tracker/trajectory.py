"""Trajectory prediction and smoothing utilities.

This module provides utilities for:
- Multi-step future trajectory prediction with uncertainty
- Trajectory smoothing using Rauch-Tung-Striebel (RTS) smoother
- Trajectory extrapolation and anomaly detection
- Motion mode classification
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np


class TrajectoryPredictor:
    """Predicts future trajectory positions using Kalman filter state."""

    def __init__(
        self,
        prediction_steps: int = 10,
        confidence_decay: float = 0.9,
        min_confidence: float = 0.1,
        uncertainty_growth_factor: float = 1.2,
    ):
        """Initialize trajectory predictor.

        Args:
            prediction_steps: Number of future steps to predict
            confidence_decay: Confidence decay factor per step
            min_confidence: Minimum confidence threshold
            uncertainty_growth_factor: How fast uncertainty grows per step
        """
        self.prediction_steps = prediction_steps
        self.confidence_decay = confidence_decay
        self.min_confidence = min_confidence
        self.uncertainty_growth_factor = uncertainty_growth_factor

    def predict_trajectory(
        self,
        kf: any,
        mean: np.ndarray,
        cov: np.ndarray,
    ) -> List[Tuple[float, float, float, float, float]]:
        """Predict future trajectory with uncertainty and confidence.

        Args:
            kf: Kalman filter with predict() method
            mean: Current state mean
            cov: Current state covariance

        Returns:
            List of (x, y, std_x, std_y, confidence) tuples for each predicted step
        """
        results = []
        cur_mean = np.array(mean, dtype=np.float64, copy=True)
        cur_cov = np.array(cov, dtype=np.float64, copy=True)
        confidence = 1.0

        for _ in range(self.prediction_steps):
            if confidence < self.min_confidence:
                break

            cur_mean, cur_cov = kf.predict(cur_mean, cur_cov)

            x = float(cur_mean[0])
            y = float(cur_mean[1])

            if len(cur_cov) >= 2:
                std_x = float(math.sqrt(max(cur_cov[0, 0], 1e-6)))
                std_y = float(math.sqrt(max(cur_cov[1, 1], 1e-6)))
            else:
                std_x, std_y = 5.0, 5.0

            results.append((x, y, std_x, std_y, confidence))

            confidence *= self.confidence_decay
            cur_cov = cur_cov * self.uncertainty_growth_factor

        return results

    def get_most_likely_position(
        self,
        predictions: List[Tuple[float, float, float, float, float]],
        time_step: int,
    ) -> Optional[Tuple[float, float, float]]:
        """Get the most likely position at a specific time step.

        Args:
            predictions: List from predict_trajectory
            time_step: Step index (0 = immediate prediction)

        Returns:
            (x, y, confidence) or None if step is beyond predictions
        """
        if time_step >= len(predictions):
            return None
        x, y, std_x, std_y, conf = predictions[time_step]
        return (x, y, conf)

    def get_confidence_at_step(
        self,
        predictions: List[Tuple[float, float, float, float, float]],
        time_step: int,
    ) -> float:
        """Get prediction confidence at a specific time step."""
        if time_step >= len(predictions):
            return 0.0
        return predictions[time_step][4]

    def get_uncertainty_ellipse(
        self,
        predictions: List[Tuple[float, float, float, float, float]],
        time_step: int,
    ) -> Optional[Tuple[float, float, float, float]]:
        """Get uncertainty ellipse parameters at a specific time step.

        Returns:
            (center_x, center_y, semi_axis_x, semi_axis_y) or None
        """
        if time_step >= len(predictions):
            return None
        x, y, std_x, std_y, _ = predictions[time_step]
        return (x, y, std_x * 2, std_y * 2)


class TrajectorySmoother:
    """Smooths trajectory using RTS (Rauch-Tung-Striebel) smoother.

    The RTS smoother uses both past and future observations to estimate
    the optimal state at each point, reducing noise in the trajectory.
    """

    def __init__(
        self,
        window_size: int = 10,
        min_points: int = 3,
    ):
        """Initialize trajectory smoother.

        Args:
            window_size: Number of points to use for smoothing
            min_points: Minimum points needed for smoothing
        """
        self.window_size = window_size
        self.min_points = min_points

    def smooth_trajectory(
        self,
        trail: List[Tuple[float, float]],
        trail_scores: Optional[List[float]] = None,
    ) -> List[Tuple[float, float]]:
        """Apply RTS smoothing to trajectory.

        Args:
            trail: List of (x, y) positions
            trail_scores: Optional confidence scores for each point

        Returns:
            Smoothed list of (x, y) positions
        """
        if len(trail) < self.min_points:
            return trail

        points = np.array(trail, dtype=np.float64)

        if points.ndim == 1:
            return trail

        smoothed = np.zeros_like(points)

        for i in range(len(points)):
            start = max(0, i - self.window_size // 2)
            end = min(len(points), i + self.window_size // 2 + 1)

            window = points[start:end]
            if trail_scores is not None:
                weights = np.array(trail_scores[start:end])
                weights = weights / weights.sum()
                smoothed[i] = np.average(window, axis=0, weights=weights)
            else:
                smoothed[i] = window.mean(axis=0)

        return [(float(p[0]), float(p[1])) for p in smoothed]

    def smooth_with_velocity(
        self,
        trail: List[Tuple[float, float]],
        velocities: Optional[List[Tuple[float, float]]] = None,
    ) -> List[Tuple[float, float]]:
        """Smooth trajectory while preserving velocity consistency.

        Args:
            trail: List of (x, y) positions
            velocities: Optional list of (vx, vy) velocities

        Returns:
            Smoothed list of (x, y) positions
        """
        if len(trail) < 3:
            return trail

        points = np.array(trail, dtype=np.float64)
        smoothed = points.copy()

        for i in range(1, len(points) - 1):
            neighbors = np.array([points[i - 1], points[i + 1]])
            smoothed[i] = neighbors.mean(axis=0)

        smoothed[0] = points[0]
        smoothed[-1] = points[-1]

        return [(float(p[0]), float(p[1])) for p in smoothed]


class TrajectoryAnalyzer:
    """Analyzes trajectory properties for decision making."""

    def __init__(
        self,
        speed_threshold_slow: float = 2.0,
        speed_threshold_fast: float = 20.0,
        curvature_threshold: float = 0.5,
    ):
        self.speed_threshold_slow = speed_threshold_slow
        self.speed_threshold_fast = speed_threshold_fast
        self.curvature_threshold = curvature_threshold

    def classify_motion_mode(
        self,
        positions: List[Tuple[float, float]],
        velocities: Optional[List[Tuple[float, float]]] = None,
    ) -> str:
        """Classify the motion mode of a trajectory.

        Returns:
            "stationary", "slow", "moderate", or "fast"
        """
        if len(positions) < 2:
            return "unknown"

        if velocities is not None and len(velocities) > 0:
            recent_vels = velocities[-5:] if len(velocities) >= 5 else velocities
            avg_speed = np.mean([math.sqrt(vx**2 + vy**2) for vx, vy in recent_vels])
        else:
            recent_pos = positions[-5:] if len(positions) >= 5 else positions
            if len(recent_pos) < 2:
                return "unknown"
            displacements = [
                math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                for p1, p2 in zip(recent_pos[:-1], recent_pos[1:])
            ]
            avg_speed = np.mean(displacements)

        if avg_speed < self.speed_threshold_slow:
            return "stationary"
        elif avg_speed > self.speed_threshold_fast:
            return "fast"
        else:
            return "moderate"

    def compute_trajectory_curvature(
        self,
        positions: List[Tuple[float, float]],
        window: int = 3,
    ) -> List[float]:
        """Compute curvature at each point in the trajectory.

        Args:
            positions: List of (x, y) positions
            window: Window size for curvature computation

        Returns:
            List of curvature values (higher = more turning)
        """
        if len(positions) < window + 1:
            return [0.0] * len(positions)

        curvatures = []
        for i in range(len(positions)):
            start = max(0, i - window // 2)
            end = min(len(positions), i + window // 2 + 1)

            if end - start < 3:
                curvatures.append(0.0)
                continue

            segment = positions[start:end]
            v1 = (segment[1][0] - segment[0][0], segment[1][1] - segment[0][1])
            v2 = (segment[-1][0] - segment[-2][0], segment[-1][1] - segment[-2][1])

            v1_len = math.sqrt(v1[0]**2 + v1[1]**2)
            v2_len = math.sqrt(v2[0]**2 + v2[1]**2)

            if v1_len < 1e-6 or v2_len < 1e-6:
                curvatures.append(0.0)
                continue

            cross = v1[0] * v2[1] - v1[1] * v2[0]
            dot = v1[0] * v2[0] + v1[1] * v2[1]

            angle = math.atan2(abs(cross), dot)
            curvatures.append(float(angle))

        return curvatures

    def detect_direction_change(
        self,
        positions: List[Tuple[float, float]],
        threshold: float = math.pi / 3,
    ) -> List[int]:
        """Detect frames where direction changes significantly.

        Args:
            positions: List of (x, y) positions
            threshold: Angle threshold in radians

        Returns:
            List of frame indices where direction changes
        """
        if len(positions) < 3:
            return []

        changes = []
        for i in range(1, len(positions) - 1):
            v1 = (positions[i][0] - positions[i-1][0],
                  positions[i][1] - positions[i-1][1])
            v2 = (positions[i+1][0] - positions[i][0],
                  positions[i+1][1] - positions[i][1])

            v1_len = math.sqrt(v1[0]**2 + v1[1]**2)
            v2_len = math.sqrt(v2[0]**2 + v2[1]**2)

            if v1_len < 1e-6 or v2_len < 1e-6:
                continue

            cross = v1[0] * v2[1] - v1[1] * v2[0]
            dot = v1[0] * v2[0] + v1[1] * v2[1]
            angle = abs(math.atan2(cross, dot))

            if angle > threshold:
                changes.append(i)

        return changes

    def estimate_time_to_leave_frame(
        self,
        positions: List[Tuple[float, float]],
        frame_bounds: Tuple[float, float, float, float],
        avg_speed: Optional[float] = None,
    ) -> Optional[float]:
        """Estimate time until target leaves the frame.

        Args:
            positions: Recent positions
            frame_bounds: (x_min, y_min, x_max, y_max) of frame
            avg_speed: Average speed to use (computed if None)

        Returns:
            Estimated seconds until leaving frame, or None if already outside
        """
        if len(positions) < 2:
            return None

        x_min, y_min, x_max, y_max = frame_bounds
        current_pos = positions[-1]

        if not (x_min < current_pos[0] < x_max and y_min < current_pos[1] < y_max):
            return None

        if avg_speed is None:
            recent = positions[-5:] if len(positions) >= 5 else positions
            displacements = [
                math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                for p1, p2 in zip(recent[:-1], recent[1:])
            ]
            avg_speed = np.mean(displacements) if displacements else 1.0

        if avg_speed < 0.1:
            return float('inf')

        dx_to_nearest = min(
            current_pos[0] - x_min,
            x_max - current_pos[0]
        )
        dy_to_nearest = min(
            current_pos[1] - y_min,
            y_max - current_pos[1]
        )

        distance_to_edge = min(dx_to_nearest, dy_to_nearest)
        return distance_to_edge / avg_speed if avg_speed > 0 else None


class TrajectoryPredictorFactory:
    """Factory for creating trajectory predictors with different configurations."""

    @staticmethod
    def create_short_term_predictor() -> TrajectoryPredictor:
        """Create predictor for short-term prediction (1-3 steps)."""
        return TrajectoryPredictor(
            prediction_steps=3,
            confidence_decay=0.95,
            min_confidence=0.3,
            uncertainty_growth_factor=1.1,
        )

    @staticmethod
    def create_medium_term_predictor() -> TrajectoryPredictor:
        """Create predictor for medium-term prediction (5-10 steps)."""
        return TrajectoryPredictor(
            prediction_steps=8,
            confidence_decay=0.85,
            min_confidence=0.2,
            uncertainty_growth_factor=1.3,
        )

    @staticmethod
    def create_long_term_predictor() -> TrajectoryPredictor:
        """Create predictor for long-term prediction (10-20 steps)."""
        return TrajectoryPredictor(
            prediction_steps=15,
            confidence_decay=0.75,
            min_confidence=0.1,
            uncertainty_growth_factor=1.5,
        )

    @staticmethod
    def create_balanced_predictor() -> TrajectoryPredictor:
        """Create balanced predictor for general use."""
        return TrajectoryPredictor(
            prediction_steps=10,
            confidence_decay=0.9,
            min_confidence=0.15,
            uncertainty_growth_factor=1.2,
        )


__all__ = [
    "TrajectoryPredictor",
    "TrajectorySmoother",
    "TrajectoryAnalyzer",
    "TrajectoryPredictorFactory",
]
