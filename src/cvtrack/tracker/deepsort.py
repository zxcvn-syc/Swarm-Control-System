"""Legacy DeepSORT-style tracker (4-state KF + Mahalanobis Hungarian).

This is preserved for users that pass ``--tracker deepsort`` and as a
reference for the simpler-than-BoT-SORT path.  The behaviour matches v4.
"""

from __future__ import annotations

import logging
from typing import List

import numpy as np
from scipy.optimize import linear_sum_assignment

from cvtrack.tracker.kalman import CHI2_THRESHOLD, KalmanCV2D
from cvtrack.types import Box, Track


log = logging.getLogger(__name__)


class DeepSortLite:
    """DeepSORT-style tracker with hand-written Hungarian + KF gating."""

    def __init__(
        self,
        dt: float = 0.05,
        max_age: int = 20,
        n_init: int = 3,
        stationary_prune: bool = True,
    ) -> None:
        self.kf = KalmanCV2D(dt=dt)
        self.dt = dt
        self.max_age = max_age
        self.n_init = n_init
        self.stationary_prune = stationary_prune
        self.tracks: List[Track] = []
        DeepSortLite._id_seq += 1
        self._next_id_start = DeepSortLite._id_seq

    _id_seq = 0

    def _next_id(self) -> int:
        DeepSortLite._id_seq += 1
        return DeepSortLite._id_seq

    def step(self, detections: List[Box]) -> List[Track]:
        # 1. predict
        for t in self.tracks:
            t.predict(self.kf)

        if len(detections) == 0:
            for t in self.tracks:
                t.mark_missed()
            self._prune()
            return [t for t in self.tracks if t.confirmed]

        # 2. squared Mahalanobis cost, with class penalty
        cost = np.full((len(self.tracks), len(detections)), 1e5, dtype=np.float64)
        for i, tr in enumerate(self.tracks):
            for j, det in enumerate(detections):
                d = self.kf.mahalanobis(tr.mean, tr.cov,
                                        np.array([det.cx, det.cy], dtype=np.float64))
                if det.label != tr.label:
                    d += 50.0
                cost[i, j] = d

        row_ind, col_ind = linear_sum_assignment(cost)
        matched, unmatched_dets = [], set(range(len(detections)))
        for r, c in zip(row_ind, col_ind):
            if cost[r, c] > CHI2_THRESHOLD:
                continue
            self.tracks[r].update(self.kf, detections[c])
            matched.append((r, c))
            unmatched_dets.discard(c)

        for i, tr in enumerate(self.tracks):
            if all(r != i for r, _ in matched):
                tr.mark_missed()

        # 3. spawn new tentative tracks
        for j in unmatched_dets:
            z = np.array([detections[j].cx, detections[j].cy], dtype=np.float64)
            mean, cov = self.kf.initiate(z)
            self.tracks.append(Track(
                track_id=self._next_id(),
                label=detections[j].label,
                mean=mean, cov=cov,
                box=detections[j],
                n_init=self.n_init,
                trail=[(detections[j].cx, detections[j].cy)],
                pred_trail=[(float(mean[0]), float(mean[1]))],
                trail_scores=[detections[j].score],
            ))

        self._prune()
        return [t for t in self.tracks if t.confirmed]

    def _prune(self) -> None:
        alive: List[Track] = []
        for t in self.tracks:
            stationary = (
                self.stationary_prune
                and t.confirmed
                and t.hits >= 6
                and t.misses == 0
                and t.motion_score < 1.5
            )
            delete = (
                (not t.confirmed and t.hits < 1)
                or (t.confirmed and t.misses > self.max_age)
                or (not t.confirmed and t.misses > self.n_init)
                or stationary
            )
            if not delete:
                alive.append(t)
        self.tracks = alive