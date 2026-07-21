"""Tracking stability enhancement module.

This module provides enhancements to improve tracking stability:
- Identity preservation during occlusions
- Anti-fragmentation mechanisms
- Enhanced appearance matching with temporal consistency
- Confidence-weighted association
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from cvtrack.types import Box, Track


log = logging.getLogger(__name__)


class IdentityManager:
    """Manages track identities to prevent unnecessary ID switches.

    Key features:
    - Tracks recently lost IDs for reactivation
    - Prevents ID conflicts
    - Maintains ID history for consistency
    """

    def __init__(
        self,
        max_lost_ids: int = 20,
        max_idle_frames: int = 30,
        reactivation_threshold: float = 0.7,
    ):
        self.max_lost_ids = max_lost_ids
        self.max_idle_frames = max_idle_frames
        self.reactivation_threshold = reactivation_threshold

        self._active_ids: Set[int] = set()
        self._lost_ids: Dict[int, _LostTrackInfo] = {}
        self._id_counter = 0

    def generate_id(self) -> int:
        """Generate a new unique track ID."""
        self._id_counter += 1
        while self._id_counter in self._active_ids:
            self._id_counter += 1
        return self._id_counter

    def register_active(self, track_id: int) -> None:
        """Register a track as active."""
        self._active_ids.add(track_id)
        if track_id in self._lost_ids:
            del self._lost_ids[track_id]

    def mark_lost(
        self,
        track_id: int,
        last_box: Box,
        last_embedding: Optional[np.ndarray],
        confidence: float,
    ) -> None:
        """Mark a track as lost but keep it for potential reactivation."""
        if len(self._lost_ids) >= self.max_lost_ids:
            oldest = min(self._lost_ids.keys(), key=lambda k: self._lost_ids[k].lost_time)
            del self._lost_ids[oldest]

        self._lost_ids[track_id] = _LostTrackInfo(
            track_id=track_id,
            last_box=last_box,
            last_embedding=last_embedding,
            confidence=confidence,
            lost_time=0,
        )
        self._active_ids.discard(track_id)

    def find_reactivation_candidate(
        self,
        box: Box,
        embedding: Optional[np.ndarray],
        appearance_threshold: float = 0.3,
    ) -> Optional[int]:
        """Find a lost track that might match the given detection.

        Returns the track ID if a match is found, None otherwise.
        """
        if embedding is None:
            return None

        best_match = None
        best_score = self.reactivation_threshold

        for track_id, info in self._lost_ids.items():
            if info.lost_time > self.max_idle_frames:
                continue

            if info.last_embedding is None:
                continue

            score = self._compute_similarity(embedding, info.last_embedding)

            if score > best_score:
                best_score = score
                best_match = track_id

        return best_match

    def _compute_similarity(
        self, emb1: np.ndarray, emb2: np.ndarray
    ) -> float:
        """Compute cosine similarity between embeddings."""
        norm1 = np.linalg.norm(emb1)
        norm2 = np.linalg.norm(emb2)
        if norm1 < 1e-6 or norm2 < 1e-6:
            return 0.0
        return float(np.dot(emb1, emb2) / (norm1 * norm2))

    def tick(self) -> None:
        """Increment lost time for all lost tracks."""
        for info in self._lost_ids.values():
            info.lost_time += 1

    def get_lost_track_info(self, track_id: int) -> Optional[_LostTrackInfo]:
        """Get information about a lost track."""
        return self._lost_ids.get(track_id)


class _LostTrackInfo:
    """Information about a lost track for potential reactivation."""

    def __init__(
        self,
        track_id: int,
        last_box: Box,
        last_embedding: Optional[np.ndarray],
        confidence: float,
        lost_time: int,
    ):
        self.track_id = track_id
        self.last_box = last_box
        self.last_embedding = last_embedding
        self.confidence = confidence
        self.lost_time = lost_time


class OcclusionHandler:
    """Handles occlusions to maintain tracking stability.

    Features:
    - Occlusion detection based on box overlap
    - Adaptive gating during occlusions
    - Prediction enhancement for occluded tracks
    """

    def __init__(
        self,
        overlap_threshold: float = 0.5,
        occlusion_max_frames: int = 15,
        prediction_boost: float = 1.5,
    ):
        self.overlap_threshold = overlap_threshold
        self.occlusion_max_frames = occlusion_max_frames
        self.prediction_boost = prediction_boost

        self._occluded_tracks: Dict[int, int] = {}
        self._occlusion_history: List[Tuple[int, int]] = []

    def detect_occlusions(
        self,
        tracks: List[Track],
        detections: List[Box],
    ) -> Dict[int, bool]:
        """Detect which tracks are occluded.

        Returns:
            Dict mapping track_id to occlusion status (True = occluded)
        """
        occlusion_status = {}

        for i, tr in enumerate(tracks):
            is_occluded = False

            for j, det in enumerate(detections):
                iou = tr.box.iou(det)
                if iou > self.overlap_threshold:
                    is_occluded = True
                    break

            occlusion_status[tr.track_id] = is_occluded

            if is_occluded:
                self._occluded_tracks[tr.track_id] = (
                    self._occluded_tracks.get(tr.track_id, 0) + 1
                )
            else:
                if tr.track_id in self._occluded_tracks:
                    del self._occluded_tracks[tr.track_id]

        return occlusion_status

    def is_track_permanently_occluded(self, track_id: int) -> bool:
        """Check if a track has been occluded for too long."""
        return self._occluded_tracks.get(track_id, 0) > self.occlusion_max_frames

    def get_occlusion_confidence(self, track_id: int) -> float:
        """Get confidence factor based on occlusion duration.

        Returns a value 0-1, where lower means more likely to be lost.
        """
        frames = self._occluded_tracks.get(track_id, 0)
        if frames == 0:
            return 1.0
        return max(0.0, 1.0 - frames / self.occlusion_max_frames)

    def get_gating_adjustment(self, track_id: int) -> float:
        """Get gating threshold adjustment for occluded tracks.

        Returns a multiplier for the Mahalanobis gate (higher = more permissive).
        """
        if track_id not in self._occluded_tracks:
            return 1.0

        frames = self._occluded_tracks[track_id]
        if frames == 0:
            return 1.0

        boost = min(self.prediction_boost, 1.0 + frames * 0.1)
        return boost


class AppearanceMemory:
    """Manages appearance features with temporal consistency.

    Features:
    - EMA-based feature averaging
    - Temporal consistency constraints
    - Feature quality scoring
    """

    def __init__(
        self,
        memory_size: int = 30,
        ema_alpha: float = 0.1,
        quality_threshold: float = 0.5,
        min_samples: int = 3,
    ):
        self.memory_size = memory_size
        self.ema_alpha = ema_alpha
        self.quality_threshold = quality_threshold
        self.min_samples = min_samples

        self._track_features: Dict[int, _AppearanceMemoryEntry] = {}

    def update(
        self,
        track_id: int,
        embedding: np.ndarray,
        box: Box,
        is_occluded: bool = False,
    ) -> None:
        """Update appearance memory for a track."""
        if track_id not in self._track_features:
            self._track_features[track_id] = _AppearanceMemoryEntry(
                memory_size=self.memory_size,
                ema_alpha=self.ema_alpha,
            )

        entry = self._track_features[track_id]
        entry.add(embedding, box, is_occluded)

        if len(entry.samples) > self.memory_size:
            entry.samples.pop(0)

    def get_mean_embedding(self, track_id: int) -> Optional[np.ndarray]:
        """Get the mean appearance embedding for a track."""
        if track_id not in self._track_features:
            return None
        entry = self._track_features[track_id]
        return entry.get_mean_embedding()

    def get_quality_score(self, track_id: int) -> float:
        """Get quality score for track's appearance (0-1)."""
        if track_id not in self._track_features:
            return 0.0
        return self._track_features[track_id].get_quality_score()

    def is_consistent(
        self,
        track_id: int,
        embedding: np.ndarray,
        threshold: float = 0.7,
    ) -> bool:
        """Check if new embedding is consistent with track's history."""
        mean_emb = self.get_mean_embedding(track_id)
        if mean_emb is None:
            return True

        quality = self.get_quality_score(track_id)
        if quality < self.quality_threshold:
            return True

        similarity = self._cosine_similarity(embedding, mean_emb)
        return similarity > threshold

    def _cosine_similarity(self, a: np.ndarray, b: np.ndarray) -> float:
        """Compute cosine similarity between two embeddings."""
        norm_a = np.linalg.norm(a)
        norm_b = np.linalg.norm(b)
        if norm_a < 1e-6 or norm_b < 1e-6:
            return 0.0
        return float(np.dot(a, b) / (norm_a * norm_b))

    def remove_track(self, track_id: int) -> None:
        """Remove track from memory."""
        if track_id in self._track_features:
            del self._track_features[track_id]

    def prune_low_quality(self) -> List[int]:
        """Remove tracks with low quality appearance features.

        Returns list of removed track IDs.
        """
        removed = []
        for track_id in list(self._track_features.keys()):
            if self.get_quality_score(track_id) < self.quality_threshold * 0.5:
                self.remove_track(track_id)
                removed.append(track_id)
        return removed


class _AppearanceMemoryEntry:
    """Internal storage for appearance memory entries."""

    def __init__(self, memory_size: int, ema_alpha: float):
        self.memory_size = memory_size
        self.ema_alpha = ema_alpha
        self.samples: List[Tuple[np.ndarray, Box, bool]] = []
        self._ema_embedding: Optional[np.ndarray] = None
        self._consistency_scores: List[float] = []

    def add(
        self,
        embedding: np.ndarray,
        box: Box,
        is_occluded: bool,
    ) -> None:
        """Add a new sample to the memory."""
        self.samples.append((embedding.copy(), box, is_occluded))

        if self._ema_embedding is None:
            self._ema_embedding = embedding.copy()
        else:
            self._ema_embedding = (
                self.ema_alpha * embedding
                + (1 - self.ema_alpha) * self._ema_embedding
            )

        if len(self.samples) >= 2:
            prev_emb = self.samples[-2][0]
            score = float(np.dot(embedding, prev_emb) / (
                np.linalg.norm(embedding) * np.linalg.norm(prev_emb)
            ))
            self._consistency_scores.append(score)

    def get_mean_embedding(self) -> Optional[np.ndarray]:
        """Get the EMA-averaged embedding."""
        return self._ema_embedding.copy() if self._ema_embedding is not None else None

    def get_quality_score(self) -> float:
        """Compute quality score based on sample consistency and coverage."""
        if len(self.samples) < self.min_samples:
            return len(self.samples) / self.min_samples

        quality = 0.5

        if len(self._consistency_scores) > 0:
            avg_consistency = np.mean(self._consistency_scores)
            quality = 0.3 + 0.4 * avg_consistency

        coverage = min(1.0, len(self.samples) / 10.0)
        quality *= 0.5 + 0.5 * coverage

        non_occluded = sum(1 for s in self.samples if not s[2])
        occlusion_ratio = non_occluded / len(self.samples) if self.samples else 0
        quality *= 0.7 + 0.3 * occlusion_ratio

        return float(np.clip(quality, 0.0, 1.0))


class StabilityMetrics:
    """Computes and tracks stability metrics for tracking performance."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        """Reset all metrics."""
        self.id_switches = 0
        self.fragments = 0
        self.misses = 0
        self.false_positives = 0
        self.total_updates = 0
        self._previous_ids: Dict[int, int] = {}

    def record_association(
        self,
        track_id: int,
        previous_id: Optional[int],
    ) -> None:
        """Record an association event for metrics."""
        if previous_id is not None and previous_id != track_id:
            self.id_switches += 1

        self._previous_ids[track_id] = track_id
        self.total_updates += 1

    def record_miss(self) -> None:
        """Record a missed detection."""
        self.misses += 1

    def record_fragment(self, track_id: int) -> None:
        """Record a track fragment (brief loss of tracking)."""
        if track_id in self._previous_ids:
            self.fragments += 1

    def get_metrics(self) -> Dict[str, float]:
        """Get current metrics as a dictionary."""
        total = max(1, self.total_updates)
        return {
            "id_switches": self.id_switches,
            "id_switch_rate": self.id_switches / total,
            "fragments": self.fragments,
            "fragment_rate": self.fragments / total,
            "misses": self.misses,
            "miss_rate": self.misses / total,
        }

    def print_summary(self) -> None:
        """Print a summary of stability metrics."""
        metrics = self.get_metrics()
        print("Tracking Stability Metrics:")
        print(f"  ID Switches: {metrics['id_switches']} ({metrics['id_switch_rate']:.2%})")
        print(f"  Fragments: {metrics['fragments']} ({metrics['fragment_rate']:.2%})")
        print(f"  Misses: {metrics['misses']} ({metrics['miss_rate']:.2%})")


__all__ = [
    "IdentityManager",
    "OcclusionHandler",
    "AppearanceMemory",
    "StabilityMetrics",
]
