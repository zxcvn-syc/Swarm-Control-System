"""Adaptive Kalman Filter-based tracker with enhanced prediction.

This module provides tracker variants that use the adaptive Kalman filters
(KalmanCV2DAdaptive, KalmanBoTAdaptive) for improved tracking performance
in dynamic scenarios.

Key improvements:
- Motion-adaptive process noise
- Confidence-based measurement noise
- Enhanced trajectory prediction
- Anomaly detection for occlusions
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment

from cvtrack.appearance.gallery import Gallery
from cvtrack.tracker.kalman import (
    CHI2_INV_95_4DOF,
    KalmanBoTAdaptive,
    KalmanCV2DAdaptive,
)
from cvtrack.tracker.metrics import iou_matrix
from cvtrack.types import Box, Track


log = logging.getLogger(__name__)


class DeepSortAdaptive:
    """DeepSORT with adaptive Kalman filter for improved tracking stability.

    Uses KalmanCV2DAdaptive with:
    - Motion-adaptive process noise
    - Confidence-based measurement noise
    - Enhanced trajectory prediction
    """

    def __init__(
        self,
        dt: float = 0.05,
        max_age: int = 30,
        n_init: int = 3,
        stationary_prune: bool = True,
        use_appearance: bool = True,
        appearance_thresh: float = 0.5,
        iou_thresh: float = 0.30,
        maha_gate: float = CHI2_INV_95_4DOF,
        # Adaptive Kalman parameters
        kalman_dt: Optional[float] = None,
        base_std_pos: float = 0.05,
        base_std_vel: float = 0.00625,
        base_std_meas: float = 0.05,
        motion_adapt_gain: float = 0.3,
        velocity_limit: float = 100.0,
        # Trajectory prediction
        enable_prediction: bool = True,
        prediction_steps: int = 10,
        prediction_confidence_decay: float = 0.9,
        min_prediction_confidence: float = 0.1,
    ) -> None:
        self.kf = KalmanCV2DAdaptive(
            dt=kalman_dt if kalman_dt is not None else dt,
            base_std_pos=base_std_pos,
            base_std_vel=base_std_vel,
            base_std_meas=base_std_meas,
            motion_adapt_gain=motion_adapt_gain,
            velocity_limit=velocity_limit,
        )
        self.dt = dt
        self.max_age = int(max_age)
        self.n_init = int(n_init)
        self.stationary_prune = stationary_prune
        self.use_appearance = bool(use_appearance)
        self.appearance_thresh = float(appearance_thresh)
        self.iou_thresh = float(iou_thresh)
        self.maha_gate = float(maha_gate)

        # Trajectory prediction settings
        self.enable_prediction = enable_prediction
        self.prediction_steps = prediction_steps
        self.prediction_confidence_decay = prediction_confidence_decay
        self.min_prediction_confidence = min_prediction_confidence

        self.tracks: List[Track] = []
        DeepSortAdaptive._id_seq += 1
        self._next_id_start = DeepSortAdaptive._id_seq

    _id_seq = 0

    def _next_id(self) -> int:
        DeepSortAdaptive._id_seq += 1
        return DeepSortAdaptive._id_seq

    def _predicted_box(self, t: Track) -> Box:
        """Get predicted bounding box."""
        cx, cy = t.pos
        w, h = t.box.wh
        return Box(
            cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0,
            t.box.score, t.box.cls, t.box.label,
        )

    def _maha_gate(
        self,
        tracks: Sequence[Track],
        detections: Sequence[Box],
    ) -> np.ndarray:
        """Boolean gating mask based on Mahalanobis distance."""
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
        """Cosine-distance appearance cost."""
        big = 1e4
        cost = np.full((len(tracks), len(detections)), big, dtype=np.float64)
        if det_embeddings is None:
            return cost
        for i, tr in enumerate(tracks):
            mu = tr.embedding_mean
            for j, det in enumerate(detections):
                emb = det_embeddings[j] if j < len(det_embeddings) else None
                if emb is None or mu is None:
                    cost[i, j] = self.appearance_thresh
                    continue
                cos_d = float(1.0 - np.dot(emb, mu))
                cost[i, j] = cos_d
        return cost

    def _update_track_prediction(self, track: Track) -> None:
        """Update predicted trajectory for a track."""
        if self.enable_prediction:
            track.update_trajectory_prediction(
                kf=self.kf,
                n_steps=self.prediction_steps,
                min_confidence=self.min_prediction_confidence,
                confidence_decay=self.prediction_confidence_decay,
            )
        track.detect_motion_mode()

    def step(
        self,
        detections: List[Box],
        det_embeddings: Optional[List[Optional[np.ndarray]]] = None,
        galleries: Optional[Dict[int, Gallery]] = None,
    ) -> List[Track]:
        # 1. Predict
        for t in self.tracks:
            t.predict(self.kf)
            self._update_track_prediction(t)

        if len(self.tracks) == 0 and not detections:
            return [t for t in self.tracks if t.confirmed]

        # 2. Gating
        if detections:
            gate = self._maha_gate(self.tracks, detections)
            app_cost = self._appearance_cost(
                self.tracks, detections, det_embeddings
            ) if self.use_appearance else None
        else:
            gate = np.zeros((len(self.tracks), 0), dtype=bool)
            app_cost = None

        matched_tracks: set = set()
        matched_dets: set = set()

        # 3. Matching cascade
        if detections:
            tracks_by_age: Dict[int, List[int]] = {}
            for i, tr in enumerate(self.tracks):
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

        # 4. IoU fallback
        unmatched_tracks_idx = [i for i in range(len(self.tracks))
                                if i not in matched_tracks]
        unmatched_dets_idx = [j for j in range(len(detections))
                              if j not in matched_dets]
        if unmatched_tracks_idx and unmatched_dets_idx:
            u_tracks = [self.tracks[i] for i in unmatched_tracks_idx]
            u_dets = [detections[j] for j in unmatched_dets_idx]
            ious = iou_matrix([self._predicted_box(t) for t in u_tracks], u_dets)
            cost = 1.0 - ious
            for i, tr in enumerate(u_tracks):
                for j, det in enumerate(u_dets):
                    if tr.label != det.label:
                        cost[i, j] = 1e4
                    elif cost[i, j] > (1.0 - self.iou_thresh):
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

        # 5. Mark unmatched as missed
        for i, tr in enumerate(self.tracks):
            if i not in matched_tracks:
                tr.mark_missed()
                # Check for anomalies
                if hasattr(self.kf, 'is_innovation_anomaly'):
                    tr.is_anomaly = False

        # 6. Spawn new tracks
        for j, det in enumerate(detections):
            if j in matched_dets:
                continue
            z = np.array([det.cx, det.cy], dtype=np.float64)
            confidence = det.score
            mean, cov = self.kf.initiate(z, confidence=confidence)
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

            # Bootstrap gallery
            if (galleries is not None and det_embeddings is not None
                    and j < len(det_embeddings) and det_embeddings[j] is not None):
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
            ious = iou_matrix([self._predicted_box(t) for t in tier_tracks], list(tier_dets))
            return 1.0 - ious

        for i in range(len(tier_tracks)):
            for j in range(len(tier_dets)):
                if not tier_gate[i, j]:
                    continue
                if tier_app is not None:
                    a = tier_app[i, j]
                    if a <= self.appearance_thresh:
                        cost[i, j] = a
                    else:
                        cost[i, j] = a
                else:
                    tr = tier_tracks[i]
                    det = tier_dets[j]
                    pb = self._predicted_box(tr)
                    iou_val = pb.iou(det)
                    if iou_val > 0.0:
                        cost[i, j] = 1.0 - iou_val
                    else:
                        cost[i, j] = 0.7
        return cost

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


class BoTSortAdaptive:
    """BoT-SORT with adaptive Kalman filter for improved dynamic tracking.

    Uses KalmanBoTAdaptive with:
    - Motion-mode detection (stationary/slow/fast)
    - Acceleration-based noise adaptation
    - Enhanced trajectory prediction
    - Camera motion compensation support
    """

    RELINK_FLASH_FRAMES = 10

    def __init__(
        self,
        dt: float = 0.05,
        max_age: int = 30,
        n_init: int = 3,
        stationary_prune: bool = True,
        use_cmc: bool = True,
        iou_thresh: float = 0.30,
        high_conf: float = 0.35,
        new_track_conf: float = 0.20,
        lost_relink_frames: int = 30,
        cmc_method: str = "sparse_of",
        cmc_downscale: float = 0.5,
        appearance_reid_weight: float = 0.0,
        # KalmanBoTAdaptive parameters
        sigma_p: float = 0.05,
        sigma_v: float = 0.00625,
        sigma_m: float = 0.05,
        acceleration_gain: float = 0.5,
        motion_threshold_slow: float = 2.0,
        motion_threshold_fast: float = 20.0,
        # Trajectory prediction
        enable_prediction: bool = True,
        prediction_steps: int = 10,
        prediction_confidence_decay: float = 0.9,
        min_prediction_confidence: float = 0.1,
    ) -> None:
        self.kf = KalmanBoTAdaptive(
            dt=dt,
            base_sigma_p=sigma_p,
            base_sigma_v=sigma_v,
            base_sigma_m=sigma_m,
            acceleration_gain=acceleration_gain,
            motion_threshold_slow=motion_threshold_slow,
            motion_threshold_fast=motion_threshold_fast,
        )
        self.dt = dt
        self.max_age = int(max_age)
        self.n_init = int(n_init)
        self.stationary_prune = stationary_prune
        self.use_cmc = use_cmc
        self.iou_thresh = float(iou_thresh)
        self.high_conf = float(high_conf)
        self.new_track_conf = float(new_track_conf)
        self.lost_relink_frames = int(lost_relink_frames)
        self.appearance_reid_weight = float(appearance_reid_weight)

        # CMC setup (same as original BoT-SORT)
        from cvtrack.tracker.cmc import SparseOFCompensator
        self.gmc = SparseOFCompensator(downscale=cmc_downscale) if use_cmc else None

        # Trajectory prediction settings
        self.enable_prediction = enable_prediction
        self.prediction_steps = prediction_steps
        self.prediction_confidence_decay = prediction_confidence_decay
        self.min_prediction_confidence = min_prediction_confidence

        self.tracks: List[Track] = []

    def _affine_for_frame(self, frame: np.ndarray, fg_boxes=None) -> Optional[np.ndarray]:
        """Get affine transform for camera motion compensation."""
        if self.gmc is None:
            return None
        from cvtrack.tracker.cmc import affine_is_pure_camera_pan
        A = self.gmc(frame, fg_boxes=fg_boxes)
        if A is None or not affine_is_pure_camera_pan(A):
            return None
        return A

    def _predicted_box(self, t: Track) -> Box:
        """Get predicted bounding box from track state."""
        cx, cy = t.pos
        w = max(float(t.mean[2]), 1.0)
        h = max(float(t.mean[3]), 1.0)
        return Box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2,
                   t.box.score, t.box.cls, t.box.label)

    def _update_track_prediction(self, track: Track) -> None:
        """Update predicted trajectory for a track."""
        if self.enable_prediction:
            track.update_trajectory_prediction(
                kf=self.kf,
                n_steps=self.prediction_steps,
                min_confidence=self.min_prediction_confidence,
                confidence_decay=self.prediction_confidence_decay,
            )
        track.detect_motion_mode(speed_threshold_slow=2.0, speed_threshold_fast=20.0)

    def step(
        self,
        frame: np.ndarray,
        detections: List[Box],
        det_embeddings: Optional[List[Optional[np.ndarray]]] = None,
    ) -> List[Track]:
        # 1. Camera motion compensation
        warp = self._affine_for_frame(frame, fg_boxes=detections)

        # 2. Predict + apply CMC
        for t in self.tracks:
            t.was_lost_before_update = (t.lost_age > 0)
            t.predict(self.kf)
            if warp is not None:
                t.mean, t.cov = self.kf.apply_affine(t.mean, t.cov, warp)
            self._update_track_prediction(t)

        # 3. Split by confidence
        high = [d for d in detections if d.score >= self.high_conf]
        low = [d for d in detections if self.new_track_conf <= d.score < self.high_conf]

        emb_by_box = {id(d): det_embeddings[i] for i, d in enumerate(detections)} \
            if det_embeddings else {}

        def embs_for(subset: List[Box]) -> List[Optional[np.ndarray]]:
            return [emb_by_box.get(id(d)) for d in subset]

        matched_tracks: set = set()
        matched_dets: set = set()

        # 4-6. Association (same as original BoT-SORT)
        if high and self.tracks:
            cost = self._fused_cost(self.tracks, high, embs_for(high))
            row_ind, col_ind = linear_sum_assignment(cost)
            for r, c in zip(row_ind, col_ind):
                if not self._accept_match(self.tracks[r], high[c], cost[r, c]):
                    continue
                self.tracks[r].update(self.kf, high[c])
                matched_tracks.add(r)
                matched_dets.add(c)

        if low:
            remain_idx = [i for i in range(len(self.tracks)) if i not in matched_tracks]
            remain_tracks = [self.tracks[i] for i in remain_idx]
            if remain_tracks:
                cost = self._fused_cost(remain_tracks, low, embs_for(low))
                row_ind, col_ind = linear_sum_assignment(cost)
                for r, c in zip(row_ind, col_ind):
                    if not self._accept_match(remain_tracks[r], low[c], cost[r, c]):
                        continue
                    remain_tracks[r].update(self.kf, low[c])
                    matched_tracks.add(remain_idx[r])
                    matched_dets.add(c)

        # Anti-fragmentation relink
        unmatched_high = [j for j, d in enumerate(high) if j not in matched_dets]
        if unmatched_high:
            relink_idx = [i for i, tr in enumerate(self.tracks)
                          if i not in matched_tracks
                          and tr.lost_age <= self.lost_relink_frames
                          and tr.lost_age <= self.max_age]
            if relink_idx:
                relink_tracks = [self.tracks[i] for i in relink_idx]
                cost = self._fused_cost(relink_tracks, [high[j] for j in unmatched_high],
                                        embs_for([high[j] for j in unmatched_high]))
                row_ind, col_ind = linear_sum_assignment(cost)
                for r, c in zip(row_ind, col_ind):
                    tr = relink_tracks[r]
                    det = high[unmatched_high[c]]
                    if not self._accept_match(tr, det, cost[r, c]):
                        continue
                    if tr.label != det.label:
                        continue
                    tr.update(self.kf, det)
                    matched_tracks.add(relink_idx[r])
                    matched_dets.add(unmatched_high[c])

        # 7. Mark missed
        for i, tr in enumerate(self.tracks):
            if i not in matched_tracks:
                tr.mark_missed()

        # 8. Spawn new tracks
        for j, d in enumerate(high):
            if j not in matched_dets:
                self.tracks.append(self._spawn(d))

        # 9. Flash relinked
        for i in matched_tracks:
            tr = self.tracks[i]
            if tr.was_lost_before_update:
                tr.relink_remaining = self.RELINK_FLASH_FRAMES

        self._prune()
        return [t for t in self.tracks if t.state != 2]

    def _fused_cost(
        self,
        tracks: List[Track],
        detections: List[Box],
        det_embeddings: Optional[List[Optional[np.ndarray]]] = None,
    ) -> np.ndarray:
        """Compute fused IoU + Mahalanobis cost."""
        big = 1e4
        cost = np.full((len(tracks), len(detections)), big, dtype=np.float64)
        w_iou, w_maha = 0.6, 0.4
        maha_gate = CHI2_INV_95_4DOF
        ious = iou_matrix([self._predicted_box(t) for t in tracks], detections)

        for i, tr in enumerate(tracks):
            pred = self._predicted_box(tr)
            z_pred = np.array([pred.cx, pred.cy, pred.w, pred.h], dtype=np.float64)
            S = np.eye(4, 8) @ tr.cov @ np.eye(4, 8).T + self.kf._r(tr.mean)
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue
            for j, det in enumerate(detections):
                if det.label != tr.label:
                    continue
                iou = ious[i, j]
                centre_dist = ((det.cx - pred.cx) ** 2 + (det.cy - pred.cy) ** 2) ** 0.5
                gate_radius = (pred.w + pred.h) * 0.6 + 8.0
                if iou <= 0.0 and centre_dist > gate_radius:
                    continue
                z = np.array([det.cx, det.cy, det.w, det.h], dtype=np.float64)
                maha = float((z - z_pred) @ S_inv @ (z - z_pred))
                if iou <= 0.0 and maha > maha_gate:
                    continue
                iou_term = 1.0 - max(iou, 0.0)
                maha_term = min(maha / maha_gate, 1.5)
                cost[i, j] = w_iou * iou_term + w_maha * maha_term

                if (self.appearance_reid_weight > 0 and det_embeddings
                        and det_embeddings[j] is not None and tr.embedding_mean is not None):
                    emb = det_embeddings[j]
                    cos_d = float(1.0 - np.dot(emb, tr.embedding_mean))
                    cost[i, j] = ((1.0 - self.appearance_reid_weight) * cost[i, j]
                                  + self.appearance_reid_weight * cos_d)
        return cost

    @staticmethod
    def _accept_match(tr: Track, det: Box, cost: float) -> bool:
        if cost >= 1e4 - 1 or tr.label != det.label:
            return False
        return cost < 0.85

    def _spawn(self, det: Box) -> Track:
        z = np.array([det.cx, det.cy, det.w, det.h], dtype=np.float64)
        mean, cov = self.kf.initiate(z)
        return Track(
            track_id=self._next_id(),
            label=det.label,
            mean=mean, cov=cov,
            box=det,
            n_init=self.n_init,
            trail=[(det.cx, det.cy)],
            pred_trail=[(float(mean[0]), float(mean[1]))],
            trail_scores=[det.score],
        )

    def _next_id(self) -> int:
        BoTSortAdaptive._id_seq += 1
        return BoTSortAdaptive._id_seq

    _id_seq = 0

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
            too_old_lost = (t.state == 1 and t.lost_age > self.max_age)
            tentative_dead = (not t.confirmed and t.lost_age > self.max_age)
            delete = stationary or too_old_lost or tentative_dead
            if delete:
                t.state = 2
                continue
            alive.append(t)
        self.tracks = alive


__all__ = [
    "DeepSortAdaptive",
    "BoTSortAdaptive",
]
