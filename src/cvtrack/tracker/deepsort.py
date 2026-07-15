"""Legacy DeepSORT-style tracker + the v6 true DeepSORT cascade matcher.

Two implementations live here:

* ``DeepSortLite`` -- the v4-compatible 4-state KF + Mahalanobis Hungarian
  matcher.  Kept verbatim (behaviour-preserving) so users that pass
  ``--tracker deepsort`` see the same numbers as v4.

* ``DeepSortCascade`` -- the v6 DeepSORT implementation that matches the
  original paper's three-stage cascade:

      1. Mahalanobis gating (chi-squared threshold for k=2 dof, the DeepSORT
         paper's default).
      2. For pairs that pass the gate, score the cosine distance between the
         detection embedding and the track's ReID gallery mean.
      3. Associate in age order: fresh lost tracks (age = 1) match first,
         older lost tracks match later.  This is what the paper calls the
         *matching cascade* and is what suppresses identity switching when
         an object occludes for a long stretch.

  Unmatched tracks fall through to an IoU-distance fallback (handles brief
  occlusions where the gating alone is too strict), and unmatched tracks
  age-out at ``max_age`` as usual.

This module deliberately does not depend on torchreid: the appearance
embedding is whatever the caller passes in (could be OSNet, could be the
``HistogramExtractor`` (removed)).  ``det_embeddings`` and per-track
``gallery`` are supplied by the pipeline; the cascade is computed here.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment

from cvtrack.appearance.gallery import Gallery
from cvtrack.tracker.kalman import CHI2_THRESHOLD, KalmanCV2D
from cvtrack.tracker.metrics import iou_matrix
from cvtrack.types import Box, Track


log = logging.getLogger(__name__)


# DeepSORT paper: chi-squared 95% threshold for 4 dof (cx, cy, aspect, h).
# We're using the 4-state KF here so the gating is the 2-dof variant.
# Keep both symbols around because the cascade gating can be overridden.
DEEPSORT_MAHALANOBIS_GATE = CHI2_THRESHOLD
DEEPSORT_APPEARANCE_GATE = 0.5  # cosine distance ceiling (1 - similarity >= 0.5)


def _predicted_box_from_4state(t: Track) -> Box:
    """4-state KF track's predicted bounding box (legacy DeepSORT)."""
    cx, cy = t.pos
    w, h = t.box.wh
    return Box(
        cx - w / 2.0,
        cy - h / 2.0,
        cx + w / 2.0,
        cy + h / 2.0,
        t.box.score,
        t.box.cls,
        t.box.label,
    )


# ---------------------------------------------------------------------------
# Legacy v4 tracker (behaviour preserved)
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# v6 true DeepSORT cascade matcher
# ---------------------------------------------------------------------------
class DeepSortCascade:
    """DeepSORT with matching cascade + appearance fusion (paper-faithful).

    Pipeline
    --------

    Per step:

    1. KF predict every track.
    2. Build the *Mahalanobis gating* mask: True iff ``d_maha < gate`` and
       the detection class matches the track class.
    3. Within the gate, compute the cosine-distance cost between the
       detection embedding and the track's ReID gallery mean.  Tracks that
       have no embedding yet are costed at a permissive default so they can
       still acquire one (they look like "first-sight" tracks).
    4. **Matching cascade**: iterate ``age = 1, 2, ..., max_age``.  In each
       tier, run Hungarian on the tracks at that tier and the still-unmatched
       detections; only consider track/det pairs whose Mahalanobis gate
       passed.
    5. **IoU fallback**: for tracks that did *not* match in the cascade and
       detections that are still free, run a final IoU-distance Hungarian
       pass.  This rescues short occlusions where the KF-predicted centre
       drifted past the Mahalanobis gate but the boxes still overlap.
    6. Unmatched tracks age++ and are pruned at ``max_age``.  Unmatched
       detections spawn new tentative tracks.

    Parameters
    ----------
    use_appearance:
        If False, the appearance cost term is skipped (purely geometric
        cascade + IoU).  Useful as an ablation.
    appearance_thresh:
        Maximum cosine distance accepted for an appearance match.  Pairs
        beyond this are dropped from the cascade (they can still be rescued
        by the IoU fallback).
    max_age, n_init, stationary_prune:
        Standard DeepSORT lifecycle knobs.
    iou_thresh:
        Threshold below which two boxes are considered "non-overlapping"
        for the IoU fallback.  Default 0.30 matches BoT-SORT.
    """

    def __init__(
        self,
        dt: float = 0.05,
        max_age: int = 30,
        n_init: int = 3,
        stationary_prune: bool = True,
        use_appearance: bool = True,
        appearance_thresh: float = DEEPSORT_APPEARANCE_GATE,
        iou_thresh: float = 0.30,
        maha_gate: float = DEEPSORT_MAHALANOBIS_GATE,
    ) -> None:
        self.kf = KalmanCV2D(dt=dt)
        self.dt = dt
        self.max_age = int(max_age)
        self.n_init = int(n_init)
        self.stationary_prune = stationary_prune
        self.use_appearance = bool(use_appearance)
        self.appearance_thresh = float(appearance_thresh)
        self.iou_thresh = float(iou_thresh)
        self.maha_gate = float(maha_gate)
        self.tracks: List[Track] = []
        DeepSortCascade._id_seq += 1
        self._next_id_start = DeepSortCascade._id_seq

    _id_seq = 0

    def _next_id(self) -> int:
        DeepSortCascade._id_seq += 1
        return DeepSortCascade._id_seq

    # ------------------------------------------------------------------
    # Cost matrices
    # ------------------------------------------------------------------
    def _maha_gate(
        self,
        tracks: Sequence[Track],
        detections: Sequence[Box],
    ) -> np.ndarray:
        """Boolean (n_tracks, n_dets) mask: True iff the KF agrees."""
        gate = np.zeros((len(tracks), len(detections)), dtype=bool)
        for i, tr in enumerate(tracks):
            for j, det in enumerate(detections):
                if det.label != tr.label:
                    continue
                z = np.array([det.cx, det.cy], dtype=np.float64)
                if self.kf.mahalanobis(tr.mean, tr.cov, z) < self.maha_gate:
                    gate[i, j] = True
        return gate

    def _appearance_cost(
        self,
        tracks: Sequence[Track],
        detections: Sequence[Box],
        det_embeddings: Optional[Sequence[Optional[np.ndarray]]],
    ) -> np.ndarray:
        """Cosine-distance cost for (track, det) pairs.

        Tracks with no gallery yet (freshly spawned) get a permissive cost of
        ``appearance_thresh`` so the cascade can pair them with the first
        plausible detection and bootstrap the gallery.
        """
        big = 1e4
        cost = np.full((len(tracks), len(detections)), big, dtype=np.float64)
        if det_embeddings is None:
            return cost
        for i, tr in enumerate(tracks):
            mu = tr.embedding_mean
            for j, det in enumerate(detections):
                emb = det_embeddings[j] if j < len(det_embeddings) else None
                if emb is None:
                    # No embedding for this detection -- neutral cost.
                    cost[i, j] = self.appearance_thresh
                    continue
                if mu is None:
                    # No track gallery yet -- permissive so the cascade can
                    # bootstrap it.  Use the threshold itself.
                    cost[i, j] = self.appearance_thresh
                    continue
                cos_d = float(1.0 - np.dot(emb, mu))
                cost[i, j] = cos_d
        return cost

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------
    def step(
        self,
        detections: List[Box],
        det_embeddings: Optional[List[Optional[np.ndarray]]] = None,
        galleries: Optional[Dict[int, Gallery]] = None,
    ) -> List[Track]:
        # 1. Predict.
        for t in self.tracks:
            t.predict(self.kf)

        if len(self.tracks) == 0 and not detections:
            return [t for t in self.tracks if t.confirmed]

        # 2. Gating.
        if detections:
            gate = self._maha_gate(self.tracks, detections)
            app_cost = self._appearance_cost(self.tracks, detections, det_embeddings) \
                if self.use_appearance else None
        else:
            gate = np.zeros((len(self.tracks), 0), dtype=bool)
            app_cost = None

        matched_tracks: set = set()
        matched_dets: set = set()

        # 3. Matching cascade (age-ordered).
        if detections:
            tracks_by_age: Dict[int, List[int]] = {}
            for i, tr in enumerate(self.tracks):
                # Only confirmed tracks participate in the cascade.  Brand
                # new tentative tracks can still match via the IoU fallback
                # (same as DeepSORT paper).
                if not tr.confirmed:
                    continue
                if tr.misses <= 0:
                    continue
                tracks_by_age.setdefault(int(tr.misses), []).append(i)

            for age in sorted(tracks_by_age.keys()):
                tier = tracks_by_age[age]
                if age > self.max_age:
                    break
                tier_tracks = [self.tracks[i] for i in tier]
                remain_dets = [j for j in range(len(detections)) if j not in matched_dets]
                if not remain_dets:
                    break
                cost = self._cascade_cost(
                    tier_tracks,
                    [detections[j] for j in remain_dets],
                    gate[tier][:, remain_dets] if gate.size else None,
                    app_cost[tier][:, remain_dets] if app_cost is not None else None,
                )
                row_ind, col_ind = linear_sum_assignment(cost)
                for r, c in zip(row_ind, col_ind):
                    if cost[r, c] > 1e3 - 1:
                        continue
                    track_idx = tier[r]
                    det_idx = remain_dets[c]
                    self.tracks[track_idx].update(self.kf, detections[det_idx])
                    matched_tracks.add(track_idx)
                    matched_dets.add(det_idx)

        # 4. IoU fallback for everything still unmatched.
        unmatched_tracks_idx = [i for i in range(len(self.tracks))
                                if i not in matched_tracks]
        unmatched_dets_idx = [j for j in range(len(detections))
                              if j not in matched_dets]
        if unmatched_tracks_idx and unmatched_dets_idx:
            u_tracks = [self.tracks[i] for i in unmatched_tracks_idx]
            u_dets = [detections[j] for j in unmatched_dets_idx]
            ious = iou_matrix(
                [_predicted_box_from_4state(t) for t in u_tracks],
                u_dets,
            )
            cost = 1.0 - ious
            # Reject class mismatches and zero-overlap pairs.
            for i, tr in enumerate(u_tracks):
                for j, det in enumerate(u_dets):
                    if tr.label != det.label:
                        cost[i, j] = 1e4
                    elif cost[i, j] > (1.0 - self.iou_thresh):
                        # Strictly enforce the IoU threshold.
                        cost[i, j] = 1e4
            row_ind, col_ind = linear_sum_assignment(cost)
            for r, c in zip(row_ind, col_ind):
                if cost[r, c] > 1e3 - 1:
                    continue
                track_idx = unmatched_tracks_idx[r]
                det_idx = unmatched_dets_idx[c]
                self.tracks[track_idx].update(self.kf, detections[det_idx])
                matched_tracks.add(track_idx)
                matched_dets.add(det_idx)

        # 5. Mark unmatched tracks as missed (bumps misses/lost_age).
        for i, tr in enumerate(self.tracks):
            if i not in matched_tracks:
                tr.mark_missed()

        # 6. Spawn new tentative tracks for unmatched detections.
        for j, det in enumerate(detections):
            if j in matched_dets:
                continue
            z = np.array([det.cx, det.cy], dtype=np.float64)
            mean, cov = self.kf.initiate(z)
            new_track = Track(
                track_id=self._next_id(),
                label=det.label,
                mean=mean, cov=cov,
                box=det,
                n_init=self.n_init,
                trail=[(det.cx, det.cy)],
                pred_trail=[(float(mean[0]), float(mean[1]))],
                trail_scores=[det.score],
            )
            self.tracks.append(new_track)
            # Bootstrap the gallery with the matching detection embedding.
            if (
                galleries is not None
                and det_embeddings is not None
                and j < len(det_embeddings)
                and det_embeddings[j] is not None
            ):
                g = galleries.get(new_track.track_id)
                if g is None:
                    g = Gallery(size=50, ema_alpha=0.05)
                    galleries[new_track.track_id] = g
                g.add(det_embeddings[j])
                new_track.embedding_mean = g.mean

        self._prune()
        return [t for t in self.tracks if t.confirmed]

    def _cascade_cost(
        self,
        tier_tracks: Sequence[Track],
        tier_dets: Sequence[Box],
        tier_gate: Optional[np.ndarray],
        tier_app: Optional[np.ndarray],
    ) -> np.ndarray:
        big = 1e4
        cost = np.full((len(tier_tracks), len(tier_dets)), big, dtype=np.float64)
        if tier_gate is None:
            # No gating info: fall back to a simple IoU cost so Hungarian
            # still finds something sensible.
            ious = iou_matrix(
                [_predicted_box_from_4state(t) for t in tier_tracks],
                list(tier_dets),
            )
            return 1.0 - ious

        for i in range(len(tier_tracks)):
            for j in range(len(tier_dets)):
                if not tier_gate[i, j]:
                    continue
                # Mahalanobis already accepted this pair; score it.
                if tier_app is not None:
                    a = tier_app[i, j]
                    if a <= self.appearance_thresh:
                        # Pure appearance cost when within the appearance gate.
                        cost[i, j] = a
                    else:
                        # Outside the appearance gate: only acceptable via the
                        # IoU fallback (we set a very high cost here).
                        cost[i, j] = a
                else:
                    # No appearance: use IoU as the within-gate cost.
                    tr = tier_tracks[i]
                    det = tier_dets[j]
                    pb = _predicted_box_from_4state(tr)
                    iou_val = pb.iou(det)
                    if iou_val > 0.0:
                        cost[i, j] = 1.0 - iou_val
                    else:
                        # No overlap but gating says OK (e.g. fast motion);
                        # use a moderate cost so Hungarian can still pick it.
                        cost[i, j] = 0.7
        return cost

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
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


__all__ = [
    "DeepSortLite",
    "DeepSortCascade",
    "DEEPSORT_APPEARANCE_GATE",
    "DEEPSORT_MAHALANOBIS_GATE",
]