"""Target-lock state machine independent from ROS message types.

The tracker owns multi-object association.  This module owns the narrower
question of whether a human-selected target is reliable enough to expose to a
control consumer.  It deliberately never emits vehicle commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Sequence

import numpy as np


class LockState(str, Enum):
    SEARCHING = "searching"
    ACQUIRING = "acquiring"
    LOCKED = "locked"
    SUSPECT = "suspect"
    LOST = "lost"


@dataclass(frozen=True)
class LockObservation:
    """One tracker result expressed in image coordinates.

    ``measured`` must be false for prediction-only tracks.  The manager never
    considers a prediction-only result command eligible.
    """

    target_id: int
    x: float
    y: float
    vx: float
    vy: float
    confidence: float
    confirmed: bool
    measured: bool
    covariance: tuple[float, float, float] = (1.0, 0.0, 1.0)
    embedding: Optional[Sequence[float]] = None


@dataclass(frozen=True)
class TargetLockConfig:
    acquire_frames: int = 3
    reacquire_frames: int = 3
    min_confidence: float = 0.35
    max_mahalanobis: float = 9.21
    lock_missed_frames: int = 2
    suspect_timeout_frames: int = 15
    min_reid_similarity: float = 0.70
    covariance_growth_per_s: float = 25.0

    def __post_init__(self) -> None:
        if self.acquire_frames < 1 or self.reacquire_frames < 1:
            raise ValueError("acquire and reacquire frame counts must be positive")
        if self.lock_missed_frames < 0 or self.suspect_timeout_frames < 1:
            raise ValueError("invalid lock timeout")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if not -1.0 <= self.min_reid_similarity <= 1.0:
            raise ValueError("min_reid_similarity must be in [-1, 1]")
        if self.max_mahalanobis <= 0.0 or self.covariance_growth_per_s < 0.0:
            raise ValueError("invalid motion-gating configuration")


@dataclass(frozen=True)
class LockDecision:
    state: LockState
    selected_target_id: Optional[int]
    command_eligible: bool
    observation: Optional[LockObservation]
    reason: str


class TargetLockManager:
    """Gate a manually selected image-space track through a strict lifecycle."""

    def __init__(self, config: TargetLockConfig | None = None) -> None:
        self.config = config or TargetLockConfig()
        self.reset()

    def reset(self) -> None:
        self.state = LockState.SEARCHING
        self.selected_target_id: Optional[int] = None
        self._candidate_id: Optional[int] = None
        self._candidate_frames = 0
        self._missed_frames = 0
        self._suspect_frames = 0
        self._last_observation: Optional[LockObservation] = None
        self._last_stamp_s: Optional[float] = None
        self._reference_embedding: Optional[np.ndarray] = None

    def select_target(self, target_id: Optional[int]) -> None:
        """Start a fresh acquisition after an explicit operator selection."""
        self.reset()
        if target_id is not None and int(target_id) >= 0:
            self.selected_target_id = int(target_id)

    def update(
        self,
        observations: Sequence[LockObservation],
        stamp_s: float,
    ) -> LockDecision:
        """Advance the lifecycle for a monotonically increasing frame stamp."""
        if not math.isfinite(stamp_s):
            return self._decision(None, "invalid_timestamp")
        if self._last_stamp_s is not None and stamp_s < self._last_stamp_s:
            return self._decision(None, "stale_timestamp")
        if self.selected_target_id is None:
            return self._decision(None, "no_target_selected")

        primary = next(
            (obs for obs in observations if obs.target_id == self.selected_target_id),
            None,
        )
        if self.state in (LockState.SEARCHING, LockState.ACQUIRING):
            if self._eligible_measurement(primary, stamp_s, require_reid=False):
                return self._advance_acquisition(primary, stamp_s, "acquiring")
            self.state = LockState.SEARCHING
            self._candidate_id = None
            self._candidate_frames = 0
            return self._decision(None, "selected_track_not_eligible")

        if self.state is LockState.LOCKED:
            if self._eligible_measurement(primary, stamp_s, require_reid=False):
                self._accept(primary, stamp_s)
                self._missed_frames = 0
                return self._decision(primary, "locked_measurement")
            self._missed_frames += 1
            if self._missed_frames <= self.config.lock_missed_frames:
                return self._decision(None, "measurement_missing_grace")
            self.state = LockState.SUSPECT
            self._suspect_frames = 0
            self._candidate_id = None
            self._candidate_frames = 0
            return self._decision(None, "lock_became_suspect")

        candidate = self._find_reacquisition(observations, stamp_s)
        if candidate is not None:
            return self._advance_reacquisition(candidate, stamp_s)

        self._suspect_frames += 1
        if self._suspect_frames >= self.config.suspect_timeout_frames:
            self.state = LockState.LOST
            self._candidate_id = None
            self._candidate_frames = 0
            return self._decision(None, "reacquisition_timeout")
        return self._decision(None, "awaiting_reacquisition")

    def _advance_acquisition(
        self, observation: LockObservation, stamp_s: float, reason: str,
    ) -> LockDecision:
        self.state = LockState.ACQUIRING
        if self._candidate_id != observation.target_id:
            self._candidate_id = observation.target_id
            self._candidate_frames = 0
        self._candidate_frames += 1
        self._accept(observation, stamp_s)
        if self._candidate_frames < self.config.acquire_frames:
            return self._decision(None, reason)
        self.state = LockState.LOCKED
        self._missed_frames = 0
        self._candidate_id = None
        self._candidate_frames = 0
        return self._decision(observation, "lock_acquired")

    def _advance_reacquisition(
        self, observation: LockObservation, stamp_s: float,
    ) -> LockDecision:
        self.state = LockState.SUSPECT
        if self._candidate_id != observation.target_id:
            self._candidate_id = observation.target_id
            self._candidate_frames = 0
        self._candidate_frames += 1
        self._accept(observation, stamp_s, update_reference=False)
        self._suspect_frames = 0
        if self._candidate_frames < self.config.reacquire_frames:
            return self._decision(None, "reacquiring")
        self.selected_target_id = observation.target_id
        self.state = LockState.LOCKED
        self._missed_frames = 0
        self._candidate_id = None
        self._candidate_frames = 0
        return self._decision(observation, "lock_reacquired")

    def _eligible_measurement(
        self,
        observation: Optional[LockObservation],
        stamp_s: float,
        *,
        require_reid: bool,
    ) -> bool:
        if observation is None or not observation.confirmed or not observation.measured:
            return False
        if not self._finite_observation(observation):
            return False
        if observation.confidence < self.config.min_confidence:
            return False
        if not self._motion_match(observation, stamp_s):
            return False
        return not require_reid or self._reid_match(observation)

    def _find_reacquisition(
        self, observations: Sequence[LockObservation], stamp_s: float,
    ) -> Optional[LockObservation]:
        matches = [
            obs for obs in observations
            if self._eligible_measurement(obs, stamp_s, require_reid=True)
        ]
        if not matches:
            return None
        return max(matches, key=self._reid_similarity)

    def _motion_match(self, observation: LockObservation, stamp_s: float) -> bool:
        if self._last_observation is None or self._last_stamp_s is None:
            return True
        elapsed = max(0.0, stamp_s - self._last_stamp_s)
        expected = np.array([
            self._last_observation.x + self._last_observation.vx * elapsed,
            self._last_observation.y + self._last_observation.vy * elapsed,
        ])
        residual = np.array([observation.x, observation.y]) - expected
        xx, xy, yy = observation.covariance
        covariance = np.array([[xx, xy], [xy, yy]], dtype=float)
        covariance += np.eye(2) * self.config.covariance_growth_per_s * elapsed
        try:
            distance = float(residual.T @ np.linalg.inv(covariance) @ residual)
        except np.linalg.LinAlgError:
            return False
        return math.isfinite(distance) and distance <= self.config.max_mahalanobis

    def _reid_match(self, observation: LockObservation) -> bool:
        if self._reference_embedding is None:
            return False
        return self._reid_similarity(observation) >= self.config.min_reid_similarity

    def _reid_similarity(self, observation: LockObservation) -> float:
        if self._reference_embedding is None or observation.embedding is None:
            return -1.0
        vector = np.asarray(observation.embedding, dtype=float).reshape(-1)
        if vector.size != self._reference_embedding.size:
            return -1.0
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-9 or not np.all(np.isfinite(vector)):
            return -1.0
        return float(np.dot(self._reference_embedding, vector / norm))

    def _accept(
        self,
        observation: LockObservation,
        stamp_s: float,
        *,
        update_reference: bool = True,
    ) -> None:
        self._last_observation = observation
        self._last_stamp_s = stamp_s
        if update_reference and observation.embedding is not None:
            vector = np.asarray(observation.embedding, dtype=float).reshape(-1)
            norm = float(np.linalg.norm(vector))
            if norm > 1e-9 and np.all(np.isfinite(vector)):
                normalized = vector / norm
                if self._reference_embedding is None:
                    self._reference_embedding = normalized
                elif normalized.size == self._reference_embedding.size:
                    blended = 0.9 * self._reference_embedding + 0.1 * normalized
                    self._reference_embedding = blended / np.linalg.norm(blended)

    @staticmethod
    def _finite_observation(observation: LockObservation) -> bool:
        values = [
            observation.x, observation.y, observation.vx, observation.vy,
            observation.confidence, *observation.covariance,
        ]
        return all(math.isfinite(float(value)) for value in values)

    def _decision(
        self, observation: Optional[LockObservation], reason: str,
    ) -> LockDecision:
        eligible = self.state is LockState.LOCKED and observation is not None
        return LockDecision(
            state=self.state,
            selected_target_id=self.selected_target_id,
            command_eligible=eligible,
            observation=observation if eligible else None,
            reason=reason,
        )
