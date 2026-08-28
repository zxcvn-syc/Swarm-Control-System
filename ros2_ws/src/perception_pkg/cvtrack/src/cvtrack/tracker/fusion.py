"""Multi-source trajectory association and confidence-aware fusion.

The module operates on :class:`cvtrack.types.Track` objects so it can be used
both by the standalone tracker package and by the ROS2 bridge.  Coordinates
must already share one frame; camera calibration and world-frame projection
belong upstream of this module.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Sequence

import numpy as np

from cvtrack.types import Box, Track


_EPS = 1e-9


def _confidence(value: Optional[float]) -> float:
    """Return a finite non-negative confidence, defaulting missing values."""
    if value is None:
        return 1.0
    try:
        result = float(value)
    except (TypeError, ValueError):
        return 1.0
    if not math.isfinite(result):
        return 1.0
    return max(0.0, min(1.0, result))


def _covariance_diagonal(
    covariance: Sequence[float] | np.ndarray,
) -> tuple[float, float]:
    """Extract x/y variances from a flattened 2x2 or full covariance."""
    array = np.asarray(covariance, dtype=np.float64)
    if array.ndim == 2 and array.shape[0] >= 2 and array.shape[1] >= 2:
        return max(float(array[0, 0]), _EPS), max(float(array[1, 1]), _EPS)
    flat = array.reshape(-1)
    if flat.size >= 4:
        return max(float(flat[0]), _EPS), max(float(flat[3]), _EPS)
    if flat.size >= 2:
        return max(float(flat[0]), _EPS), max(float(flat[1]), _EPS)
    raise ValueError("each covariance must contain at least two variances")


def _is_outlier(
    value: float,
    median_value: float,
    threshold: float,
    mad: float,
) -> bool:
    """Determine if a value is an outlier using robust statistics.

    Args:
        value: Value to test
        median_value: Median of reference distribution
        threshold: Outlier threshold multiplier
        mad: Median absolute deviation

    Returns:
        True if value is an outlier, False otherwise
    """
    if not math.isfinite(value):
        return True

    deviation = abs(value - median_value)
    if mad > _EPS:
        # MAD-based detection
        robust_sigma = 1.4826 * mad
        return deviation > threshold * robust_sigma
    else:
        # Fall back to relative threshold
        return deviation > threshold * max(abs(median_value) * 0.1, 1.0)


def weighted_fuse(
    positions: list[tuple[float, float, Optional[float]]],
    covariances: Optional[
        list[tuple[float, float, float, float] | np.ndarray]
    ] = None,
    outlier_sigma: float = 3.0,
    outlier_check: bool = True,
) -> tuple[float, float, float]:
    """Fuse ``(x, y, confidence)`` observations using uncertainty weights.

    Confidence supplies the base weight.  When covariance is available, the
    x and y estimates additionally use the inverse variance on the respective
    axis.  Missing confidence is treated as neutral confidence ``1.0``.  The
    returned confidence is the probability that at least one independent
    observation is correct, ``1 - product(1 - confidence)``.

    Outlier observations are detected using median absolute deviation (MAD)
    and their weights are reduced to avoid polluting the fused estimate.

    Args:
        positions: List of (x, y, confidence) tuples
        covariances: Optional list of covariance matrices matching positions
        outlier_sigma: Threshold for outlier detection (default 3.0)
        outlier_check: Whether to perform outlier detection (default True)
    """
    if not positions:
        raise ValueError("positions must contain at least one observation")
    if covariances is not None and len(covariances) != len(positions):
        raise ValueError("covariances must have the same length as positions")

    # Validate inputs
    valid_positions = []
    valid_covariances = []
    for idx, (x, y, conf) in enumerate(positions):
        try:
            x_val = float(x)
            y_val = float(y)
            if not math.isfinite(x_val) or not math.isfinite(y_val):
                continue
            valid_positions.append((x_val, y_val, conf))
            if covariances is not None:
                valid_covariances.append(covariances[idx])
        except (TypeError, ValueError):
            continue

    if not valid_positions:
        raise ValueError("no valid (finite) positions provided")
    if covariances is not None and len(valid_covariances) != len(valid_positions):
        raise ValueError("valid covariances must match valid positions")

    positions = valid_positions
    if covariances is not None:
        covariances = valid_covariances

    # Outlier detection using MAD
    outlier_mask = [False] * len(positions)
    if outlier_check and len(positions) >= 3:
        x_values = np.array([p[0] for p in positions], dtype=np.float64)
        y_values = np.array([p[1] for p in positions], dtype=np.float64)

        median_x = float(np.median(x_values))
        median_y = float(np.median(y_values))
        mad_x = float(np.median(np.abs(x_values - median_x)))
        mad_y = float(np.median(np.abs(y_values - median_y)))

        for idx, (x_val, y_val, _) in enumerate(positions):
            if _is_outlier(x_val, median_x, outlier_sigma, mad_x):
                outlier_mask[idx] = True
            elif _is_outlier(y_val, median_y, outlier_sigma, mad_y):
                outlier_mask[idx] = True

    x_numerator = 0.0
    y_numerator = 0.0
    x_denominator = 0.0
    y_denominator = 0.0
    confidences: list[float] = []

    for index, (x, y, confidence) in enumerate(positions):
        conf = _confidence(confidence)
        confidences.append(conf)
        # Reduce weight for outliers
        outlier_factor = 0.05 if outlier_mask[index] else 1.0
        base_weight = max(conf * outlier_factor, _EPS)
        if covariances is None:
            x_weight = y_weight = base_weight
        else:
            var_x, var_y = _covariance_diagonal(covariances[index])
            x_weight = base_weight / var_x
            y_weight = base_weight / var_y
        x_numerator += float(x) * x_weight
        y_numerator += float(y) * y_weight
        x_denominator += x_weight
        y_denominator += y_weight

    if x_denominator <= _EPS or y_denominator <= _EPS:
        # Fallback to simple average if weighting failed
        x_fused = float(np.mean([p[0] for p in positions]))
        y_fused = float(np.mean([p[1] for p in positions]))
    else:
        x_fused = x_numerator / x_denominator
        y_fused = y_numerator / y_denominator

    # Calculate confidence: use OR combination, but downweight for outliers
    non_outlier_confs = [
        conf for idx, conf in enumerate(confidences) if not outlier_mask[idx]
    ]
    if not non_outlier_confs:
        non_outlier_confs = confidences
    fused_confidence = 1.0 - math.prod(1.0 - value for value in non_outlier_confs)
    return (
        x_fused,
        y_fused,
        max(0.0, min(1.0, fused_confidence)),
    )


class TrajectoryGraph:
    """Undirected cross-sensor association graph."""

    def __init__(self) -> None:
        self._adjacency: dict[tuple[str, int], set[tuple[str, int]]] = {}

    def add_track(self, source: str, track_id: int) -> None:
        self._adjacency.setdefault((str(source), int(track_id)), set())

    def link(
        self,
        source_a: str,
        id_a: int,
        source_b: str,
        id_b: int,
    ) -> None:
        node_a = (str(source_a), int(id_a))
        node_b = (str(source_b), int(id_b))
        self.add_track(*node_a)
        self.add_track(*node_b)
        if node_a == node_b:
            return
        self._adjacency[node_a].add(node_b)
        self._adjacency[node_b].add(node_a)

    def neighbors(self, source: str, track_id: int) -> list[tuple[str, int]]:
        node = (str(source), int(track_id))
        return sorted(self._adjacency.get(node, set()))

    def components(self) -> list[set[tuple[str, int]]]:
        result: list[set[tuple[str, int]]] = []
        unseen = set(self._adjacency)
        while unseen:
            start = min(unseen)
            component: set[tuple[str, int]] = set()
            stack = [start]
            while stack:
                node = stack.pop()
                if node in component:
                    continue
                component.add(node)
                unseen.discard(node)
                stack.extend(self._adjacency[node] - component)
            result.append(component)
        return result


class ConsistencyGuard:
    """Robust one- and two-dimensional sigma clipping.

    Enhanced with multiple sigma-clipping methods:
    - Median Absolute Deviation (MAD)
    - Interquartile Range (IQR)
    - Standard deviation (for normal distributions)
    """

    def __init__(
        self,
        sigma_threshold: float = 3.0,
        use_iqr_fallback: bool = True,
    ) -> None:
        if sigma_threshold <= 0:
            raise ValueError("sigma_threshold must be positive")
        self.sigma_threshold = float(sigma_threshold)
        self.use_iqr_fallback = use_iqr_fallback

    def _mask(self, samples: Sequence[float]) -> list[bool]:
        if not samples:
            return []

        # Filter to finite values
        finite_pairs = [
            (i, float(v)) for i, v in enumerate(samples)
            if math.isfinite(v)
        ]
        finite_count = len(finite_pairs)

        if finite_count == 0:
            return [False] * len(samples)
        if finite_count <= 2:
            # Too few samples to reject outliers
            return [math.isfinite(samples[i]) for i in range(len(samples))]

        values = np.array([v for _, v in finite_pairs], dtype=np.float64)

        # Detect outliers using multiple robust methods
        median = float(np.median(values))
        deviations = np.abs(values - median)
        mad = float(np.median(deviations))

        outlier_mask = np.zeros(len(values), dtype=bool)

        if mad > _EPS:
            # MAD-based detection (most robust)
            robust_sigma = 1.4826 * mad
            outlier_mask = deviations > self.sigma_threshold * robust_sigma
        elif self.use_iqr_fallback and len(values) >= 4:
            # IQR-based fallback for tight distributions
            q75, q25 = np.percentile(values, [75, 25])
            iqr = q75 - q25
            if iqr > _EPS:
                outlier_mask = (values < (q25 - self.sigma_threshold * iqr)) | \
                               (values > (q75 + self.sigma_threshold * iqr))
            else:
                # All values nearly identical, use std-based detection
                std = float(np.std(values))
                if std > _EPS:
                    outlier_mask = deviations > self.sigma_threshold * std
        else:
            # Fall back to std-based detection
            std = float(np.std(values))
            if std > _EPS:
                outlier_mask = deviations > self.sigma_threshold * std

        # Reconstruct full mask
        result: list[bool] = [False] * len(samples)
        for (orig_idx, _), is_outlier in zip(finite_pairs, outlier_mask):
            result[orig_idx] = not bool(is_outlier)

        return result

    def filter(self, samples: list[float]) -> list[float]:
        return [value for value, keep in zip(samples, self._mask(samples)) if keep]

    def filter_2d(
        self,
        samples: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        if not samples:
            return []
        x_mask = self._mask([sample[0] for sample in samples])
        y_mask = self._mask([sample[1] for sample in samples])
        return [
            sample
            for sample, keep_x, keep_y in zip(samples, x_mask, y_mask)
            if keep_x and keep_y
        ]


# Reused _Observation and _FusedState below (kept as is from the original)
@dataclass
class _Observation:
    source: str
    track: Track
    updated_at: float
    revision: int

    @property
    def key(self) -> tuple[str, int]:
        return self.source, int(self.track.track_id)


@dataclass
class _FusedState:
    track: Track
    signature: tuple[tuple[str, int, float], ...]


class TrackFusion:
    """Multi-source trajectory fusion for cameras or UAV observers.

    ``update(source, tracks)`` treats a list as the source's latest snapshot.
    Passing one :class:`Track` is also supported and acts as an upsert.  Local
    IDs are associated to stable global IDs using distance and box overlap;
    measurements for each global target are then sigma-clipped and fused.

    Enhanced with:
    - Robust state reset mechanism
    - Numerically stable covariance fusion
    - Empty input handling
    - Trajectory history length limits
    """

    MAX_TRAIL_LENGTH = 200  # Maximum number of positions to keep per fused track

    def __init__(
        self,
        *,
        max_idle_seconds: float = 1.0,
        outlier_sigma: float = 3.0,
        max_distance_px: float = 100.0,
        iou_threshold: float = 0.3,
        max_history_per_track: int = MAX_TRAIL_LENGTH,
    ) -> None:
        if max_idle_seconds <= 0:
            raise ValueError("max_idle_seconds must be positive")
        if max_distance_px <= 0:
            raise ValueError("max_distance_px must be positive")
        if not 0.0 <= iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be between zero and one")
        if max_history_per_track < 0:
            raise ValueError("max_history_per_track must be non-negative")
        self.max_idle_seconds = float(max_idle_seconds)
        self.max_distance_px = float(max_distance_px)
        self.iou_threshold = float(iou_threshold)
        self.max_history_per_track = int(max_history_per_track)
        self.graph = TrajectoryGraph()
        self.guard = ConsistencyGuard(outlier_sigma)
        self._source_order: list[str] = []
        self._observations: dict[tuple[str, int], _Observation] = {}
        self._current_keys: dict[str, set[tuple[str, int]]] = {}
        self._key_to_global: dict[tuple[str, int], int] = {}
        self._groups: dict[int, set[tuple[str, int]]] = {}
        self._states: dict[int, _FusedState] = {}
        self._next_global_id = 1
        self._next_observation_revision = 1
        self._clock: Callable[[], float] = time.monotonic
        self._lock = threading.RLock()

    def register_source(self, name: str) -> None:
        source = str(name).strip()
        if not source:
            raise ValueError("source name must not be empty")
        with self._lock:
            if source not in self._current_keys:
                self._source_order.append(source)
                self._current_keys[source] = set()

    def sources(self) -> list[str]:
        with self._lock:
            return list(self._source_order)

    def update(self, source: str, tracks: Track | Iterable[Track]) -> None:
        self.register_source(source)
        is_snapshot = not isinstance(tracks, Track)
        track_list = list(tracks) if is_snapshot else [tracks]
        if any(not isinstance(track, Track) for track in track_list):
            raise TypeError("tracks must be a Track or an iterable of Track objects")

        # Handle empty input gracefully
        if not track_list:
            self.register_source(source)
            with self._lock:
                self._current_keys[source] = set()
            return

        now = self._clock()
        keys = {(source, int(track.track_id)) for track in track_list}
        with self._lock:
            self._expire(now)
            if is_snapshot:
                self._current_keys[source] = keys
            else:
                self._current_keys[source].update(keys)
            for track in track_list:
                key = (source, int(track.track_id))
                self.graph.add_track(*key)
                global_id = self._key_to_global.get(key)
                if global_id is None:
                    global_id = self._associate(source, track)
                    if global_id is None:
                        global_id = self._allocate_global_id()
                    self._key_to_global[key] = global_id
                    self._groups.setdefault(global_id, set()).add(key)
                    self._link_group(key, global_id)
                # Clock resolution is not a reliable cache invalidator: rapid
                # updates can share one monotonic timestamp on some platforms.
                # Keep expiry time separately and use a per-update revision for
                # the fused-state signature below.
                self._observations[key] = _Observation(
                    source,
                    track,
                    now,
                    self._next_observation_revision,
                )
                self._next_observation_revision += 1

    def fused_tracks(self) -> list[Track]:
        now = self._clock()
        with self._lock:
            self._expire(now)
            result: list[Track] = []
            for global_id in sorted(self._groups):
                observations = self._active_observations(global_id)
                if not observations:
                    continue
                result.append(self._fuse_group(global_id, observations))
            return result

    def get_track(self, global_id: int) -> Optional[Track]:
        wanted = int(global_id)
        for track in self.fused_tracks():
            if track.track_id == wanted:
                return track
        return None

    def reset(self, keep_groups: bool = False) -> None:
        """Reset the TrackFusion state.

        Args:
            keep_groups: If True, preserve group associations but clear
                observations.  If False (default), fully reset state.
        """
        with self._lock:
            if keep_groups:
                # Clear observations but keep group structure
                self._observations.clear()
                for key in list(self._current_keys):
                    self._current_keys[key].clear()
            else:
                # Full reset
                self._observations.clear()
                self._current_keys.clear()
                self._key_to_global.clear()
                self._groups.clear()
                self._states.clear()
                self._next_global_id = 1
                self._next_observation_revision = 1
                self._source_order.clear()
                self.graph = TrajectoryGraph()

    def _allocate_global_id(self) -> int:
        global_id = self._next_global_id
        self._next_global_id += 1
        return global_id

    def _associate(self, source: str, track: Track) -> Optional[int]:
        best_global_id: Optional[int] = None
        best_cost = float("inf")
        x, y = track.pos
        for global_id in self._groups:
            observations = self._active_observations(global_id)
            if not observations:
                continue
            if any(obs.key in self._current_keys[source] for obs in observations if obs.source == source):
                continue
            reference_x, reference_y, _ = weighted_fuse(
                [(obs.track.pos[0], obs.track.pos[1], obs.track.box.score) for obs in observations],
                [obs.track.cov for obs in observations],
            )
            distance = math.hypot(x - reference_x, y - reference_y)
            best_iou = max(track.box.iou(obs.track.box) for obs in observations)
            close_centroid = distance <= self.max_distance_px * 0.5
            if distance > self.max_distance_px:
                continue
            if best_iou < self.iou_threshold and not close_centroid:
                continue
            cost = distance / self.max_distance_px + (1.0 - best_iou)
            if cost < best_cost:
                best_cost = cost
                best_global_id = global_id
        return best_global_id

    def _link_group(self, key: tuple[str, int], global_id: int) -> None:
        for other in self._groups.get(global_id, set()):
            if other != key and other[0] != key[0]:
                self.graph.link(*key, *other)

    def _active_observations(self, global_id: int) -> list[_Observation]:
        latest_by_source: dict[str, _Observation] = {}
        for key in self._groups.get(global_id, set()):
            observation = self._observations.get(key)
            if observation is None:
                continue
            previous = latest_by_source.get(observation.source)
            if previous is None or observation.updated_at > previous.updated_at:
                latest_by_source[observation.source] = observation
        return list(latest_by_source.values())

    def _inlier_observations(
        self,
        observations: list[_Observation],
    ) -> list[_Observation]:
        if len(observations) <= 2:
            return observations
        positions = [observation.track.pos for observation in observations]
        x_mask = self.guard._mask([position[0] for position in positions])
        y_mask = self.guard._mask([position[1] for position in positions])
        filtered = [
            observation
            for observation, keep_x, keep_y in zip(observations, x_mask, y_mask)
            if keep_x and keep_y
        ]
        return filtered or observations

    def _fuse_group(
        self,
        global_id: int,
        observations: list[_Observation],
    ) -> Track:
        observations = self._inlier_observations(observations)
        signature = tuple(sorted(
            (obs.source, int(obs.track.track_id), obs.revision)
            for obs in observations
        ))
        previous = self._states.get(global_id)
        if previous is not None and previous.signature == signature:
            return previous.track

        best = max(observations, key=lambda obs: _confidence(obs.track.box.score)).track
        fused = copy.deepcopy(best)
        positions = [
            (obs.track.pos[0], obs.track.pos[1], obs.track.box.score)
            for obs in observations
        ]
        covariances = [obs.track.cov for obs in observations]
        x, y, confidence = weighted_fuse(positions, covariances)
        if previous is not None:
            alpha = 0.65
            x = alpha * x + (1.0 - alpha) * previous.track.pos[0]
            y = alpha * y + (1.0 - alpha) * previous.track.pos[1]

        # Determine common dimension safely
        try:
            common_mean_size = min(obs.track.mean.size for obs in observations)
        except (TypeError, ValueError):
            common_mean_size = 2  # Fallback to position-only

        for index in range(common_mean_size):
            if index in (0, 1):
                continue
            try:
                values = [float(obs.track.mean[index]) for obs in observations]
                weights = np.array(
                    [_confidence(obs.track.box.score) for obs in observations],
                    dtype=np.float64,
                )
                # Guard against non-finite values
                finite_mask = np.isfinite(values)
                if not finite_mask.all():
                    values_arr = np.array(values)
                    values_arr = values_arr[np.isfinite(values_arr)]
                    if len(values_arr) == 0:
                        continue
                    fused.mean[index] = float(np.average(
                        values_arr,
                        weights=weights[np.isfinite(weights)] if np.isfinite(weights).any() else None,
                    ))
                else:
                    fused.mean[index] = float(np.average(
                        values,
                        weights=np.maximum(weights, _EPS),
                    ))
            except (IndexError, ValueError, TypeError):
                continue

        fused.mean[0] = x
        fused.mean[1] = y
        fused.cov = self._fused_covariance(observations, fused.cov)

        # Aggregate widths and heights
        widths = [obs.track.box.w for obs in observations]
        heights = [obs.track.box.h for obs in observations]
        weights = np.maximum(
            [_confidence(obs.track.box.score) for obs in observations], _EPS
        )
        width = float(np.average(widths, weights=weights))
        height = float(np.average(heights, weights=weights))
        fused.box = Box(
            x - width / 2.0,
            y - height / 2.0,
            x + width / 2.0,
            y + height / 2.0,
            confidence,
            best.box.cls,
            best.box.label,
        )
        fused.track_id = global_id
        fused.label = best.label
        fused.hits = max(obs.track.hits for obs in observations)
        fused.age = max(obs.track.age for obs in observations)
        fused.misses = min(obs.track.misses for obs in observations)
        fused.confirmed = any(obs.track.confirmed for obs in observations)
        fused.state = 0

        # Manage trail with size limits
        fused.trail = list(previous.track.trail) if previous is not None else []
        if not fused.trail or fused.trail[-1] != (x, y):
            fused.trail.append((x, y))

        # Enforce trail length limit
        if self.max_history_per_track > 0 and len(fused.trail) > self.max_history_per_track:
            fused.trail = fused.trail[-self.max_history_per_track:]

        setattr(fused, "global_id", global_id)
        self._states[global_id] = _FusedState(fused, signature)
        return fused

    @staticmethod
    def _fused_covariance(
        observations: list[_Observation],
        template: np.ndarray,
    ) -> np.ndarray:
        """Numerically stable covariance fusion using harmonic mean.

        For each diagonal element, the fused variance is computed using
        the harmonic mean of individual variances (i.e., inverse-variance
        weighting).  Off-diagonal terms use the best observation's values
        (a common simplification for fusion).
        """
        template_arr = np.asarray(template, dtype=np.float64)
        covariance = np.array(template_arr, dtype=np.float64, copy=True)

        # Determine matrix size
        min_row = covariance.shape[0] if covariance.ndim >= 1 else 0
        min_col = covariance.shape[1] if covariance.ndim >= 2 else 0
        for obs in observations:
            try:
                obs_cov = np.asarray(obs.track.cov, dtype=np.float64)
                if obs_cov.ndim >= 2:
                    min_row = min(min_row, obs_cov.shape[0])
                    min_col = min(min_col, obs_cov.shape[1])
            except (TypeError, ValueError):
                continue

        size = max(0, min(min_row, min_col))

        for index in range(size):
            variances: list[float] = []
            for obs in observations:
                try:
                    obs_cov = np.asarray(obs.track.cov, dtype=np.float64)
                    if obs_cov.ndim >= 2 and obs_cov.shape[0] > index and obs_cov.shape[1] > index:
                        var = float(obs_cov[index, index])
                        if math.isfinite(var) and var > 0:
                            variances.append(var)
                except (TypeError, ValueError, IndexError):
                    continue

            if not variances:
                continue

            # Harmonic mean for numerical stability
            valid_vars = [max(v, _EPS) for v in variances]
            inv_sum = sum(1.0 / v for v in valid_vars)
            if inv_sum > _EPS:
                fused_var = 1.0 / inv_sum
                covariance[index, index] = max(fused_var, _EPS)
            else:
                covariance[index, index] = max(float(covariance[index, index]), _EPS)

        return covariance

    def _expire(self, now: float) -> None:
        expired = [
            key
            for key, observation in self._observations.items()
            if now - observation.updated_at > self.max_idle_seconds
        ]
        for key in expired:
            self._observations.pop(key, None)
            global_id = self._key_to_global.pop(key, None)
            self._current_keys.get(key[0], set()).discard(key)
            if global_id is not None:
                group = self._groups.get(global_id)
                if group is not None:
                    group.discard(key)
                    if not group:
                        self._groups.pop(global_id, None)
                        self._states.pop(global_id, None)


__all__ = [
    "ConsistencyGuard",
    "TrackFusion",
    "TrajectoryGraph",
    "weighted_fuse",
]
