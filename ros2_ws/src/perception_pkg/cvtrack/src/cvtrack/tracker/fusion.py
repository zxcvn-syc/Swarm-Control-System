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


def weighted_fuse(
    positions: list[tuple[float, float, Optional[float]]],
    covariances: Optional[
        list[tuple[float, float, float, float] | np.ndarray]
    ] = None,
) -> tuple[float, float, float]:
    """Fuse ``(x, y, confidence)`` observations using uncertainty weights.

    Confidence supplies the base weight.  When covariance is available, the
    x and y estimates additionally use the inverse variance on the respective
    axis.  Missing confidence is treated as neutral confidence ``1.0``.  The
    returned confidence is the probability that at least one independent
    observation is correct, ``1 - product(1 - confidence)``.
    """
    if not positions:
        raise ValueError("positions must contain at least one observation")
    if covariances is not None and len(covariances) != len(positions):
        raise ValueError("covariances must have the same length as positions")

    x_numerator = 0.0
    y_numerator = 0.0
    x_denominator = 0.0
    y_denominator = 0.0
    confidences: list[float] = []

    for index, (x, y, confidence) in enumerate(positions):
        x_value = float(x)
        y_value = float(y)
        if not math.isfinite(x_value) or not math.isfinite(y_value):
            raise ValueError("positions must contain only finite coordinates")
        conf = _confidence(confidence)
        confidences.append(conf)
        base_weight = max(conf, _EPS)
        if covariances is None:
            x_weight = y_weight = base_weight
        else:
            var_x, var_y = _covariance_diagonal(covariances[index])
            x_weight = base_weight / var_x
            y_weight = base_weight / var_y
        x_numerator += x_value * x_weight
        y_numerator += y_value * y_weight
        x_denominator += x_weight
        y_denominator += y_weight

    fused_confidence = 1.0 - math.prod(1.0 - value for value in confidences)
    return (
        x_numerator / max(x_denominator, _EPS),
        y_numerator / max(y_denominator, _EPS),
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
    """Robust one- and two-dimensional sigma clipping."""

    def __init__(self, sigma_threshold: float = 3.0) -> None:
        if sigma_threshold <= 0:
            raise ValueError("sigma_threshold must be positive")
        self.sigma_threshold = float(sigma_threshold)

    def _mask(self, samples: Sequence[float]) -> list[bool]:
        if not samples:
            return []
        values = np.asarray(samples, dtype=np.float64)
        finite = np.isfinite(values)
        finite_values = values[finite]
        if finite_values.size <= 2:
            return [bool(value) for value in finite]

        median = float(np.median(finite_values))
        deviations = np.abs(finite_values - median)
        mad = float(np.median(deviations))
        if mad <= _EPS:
            median_count = int(np.count_nonzero(deviations <= _EPS))
            if median_count > finite_values.size / 2.0:
                finite_mask = deviations <= _EPS
            else:
                std = float(np.std(finite_values))
                finite_mask = deviations <= self.sigma_threshold * max(std, _EPS)
        else:
            robust_sigma = 1.4826 * mad
            finite_mask = deviations <= self.sigma_threshold * robust_sigma

        result: list[bool] = []
        finite_index = 0
        for is_finite in finite:
            if not is_finite:
                result.append(False)
            else:
                result.append(bool(finite_mask[finite_index]))
                finite_index += 1
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


@dataclass
class _Observation:
    source: str
    track: Track
    updated_at: float

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
    """

    def __init__(
        self,
        *,
        max_idle_seconds: float = 1.0,
        outlier_sigma: float = 3.0,
        max_distance_px: float = 100.0,
        iou_threshold: float = 0.3,
    ) -> None:
        if max_idle_seconds <= 0:
            raise ValueError("max_idle_seconds must be positive")
        if max_distance_px <= 0:
            raise ValueError("max_distance_px must be positive")
        if not 0.0 <= iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be between zero and one")
        self.max_idle_seconds = float(max_idle_seconds)
        self.max_distance_px = float(max_distance_px)
        self.iou_threshold = float(iou_threshold)
        self.graph = TrajectoryGraph()
        self.guard = ConsistencyGuard(outlier_sigma)
        self._source_order: list[str] = []
        self._observations: dict[tuple[str, int], _Observation] = {}
        self._current_keys: dict[str, set[tuple[str, int]]] = {}
        self._key_to_global: dict[tuple[str, int], int] = {}
        self._groups: dict[int, set[tuple[str, int]]] = {}
        self._states: dict[int, _FusedState] = {}
        self._next_global_id = 1
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
                self._observations[key] = _Observation(source, track, now)

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
            (obs.source, int(obs.track.track_id), obs.updated_at)
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

        common_mean_size = min(obs.track.mean.size for obs in observations)
        for index in range(common_mean_size):
            if index in (0, 1):
                continue
            values = [float(obs.track.mean[index]) for obs in observations]
            weights = [_confidence(obs.track.box.score) for obs in observations]
            fused.mean[index] = float(np.average(values, weights=np.maximum(weights, _EPS)))
        fused.mean[0] = x
        fused.mean[1] = y
        fused.cov = self._fused_covariance(observations, fused.cov)

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
        fused.trail = list(previous.track.trail) if previous is not None else []
        if not fused.trail or fused.trail[-1] != (x, y):
            fused.trail.append((x, y))
        setattr(fused, "global_id", global_id)
        self._states[global_id] = _FusedState(fused, signature)
        return fused

    @staticmethod
    def _fused_covariance(
        observations: list[_Observation],
        template: np.ndarray,
    ) -> np.ndarray:
        covariance = np.array(template, dtype=np.float64, copy=True)
        size = min(
            [covariance.shape[0], covariance.shape[1]]
            + [min(obs.track.cov.shape[:2]) for obs in observations]
        )
        for index in range(size):
            variances = [max(float(obs.track.cov[index, index]), _EPS) for obs in observations]
            covariance[index, index] = 1.0 / sum(1.0 / value for value in variances)
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
