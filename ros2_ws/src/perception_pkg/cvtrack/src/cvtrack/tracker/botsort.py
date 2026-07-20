"""BoT-SORT tracker (paper sec. 3, Algorithm 1 in Appendix A).

Behavioural contract (preserved from v4):

* 8-state KF (cx, cy, w, h, vx, vy, vw, vh).
* Camera motion compensation via sparse optical flow + RANSAC affine.
* Two-stage association: high-conf IoU, then low-conf IoU.
* Class-aware gating (different classes never match).
* Recently-lost tracks can be re-activated within ``lost_relink_frames``.
* New tracks only spawned for detections whose score >= ``new_track_conf``.
* Optional ReID second-stage fusion (see ``appearance_reid_weight``).
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from cvtrack.tracker.cmc import (
    CameraMotionCompensator,
    SparseOFCompensator,
    affine_is_pure_camera_pan,
)
from cvtrack.tracker.kalman import (
    BOTSORT_HIGH_CONF,
    BOTSORT_IOU_THRESH,
    BOTSORT_LOST_RELINK_FRAMES,
    BOTSORT_NEW_TRACK_CONF,
    KalmanBoT,
)
from cvtrack.tracker.metrics import CHI2_INV_95_4DOF, iou_matrix
from cvtrack.types import Box, Track


log = logging.getLogger(__name__)


def _predicted_box(t: Track) -> Box:
    """Box implied by the current track state (8-state KF)."""
    cx, cy = t.pos
    w = max(float(t.mean[2]), 1.0)
    h = max(float(t.mean[3]), 1.0)
    return Box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2,
               t.box.score, t.box.cls, t.box.label)


class BoTSortTracker:
    """BoT-SORT tracker with optional ReID second-stage fusion."""

    RELINK_FLASH_FRAMES = 10

    def __init__(
        self,
        dt: float = 0.05,
        max_age: int = 30,
        n_init: int = 3,
        stationary_prune: bool = True,
        use_cmc: bool = True,
        iou_thresh: float = BOTSORT_IOU_THRESH,
        high_conf: float = BOTSORT_HIGH_CONF,
        new_track_conf: float = BOTSORT_NEW_TRACK_CONF,
        lost_relink_frames: int = BOTSORT_LOST_RELINK_FRAMES,
        cmc_method: str = "sparse_of",
        cmc_downscale: float = 0.5,
        # ReID second-stage fusion.  ``appearance_reid_weight=0`` disables it.
        appearance_reid_weight: float = 0.0,
    ) -> None:
        self.kf = KalmanBoT(dt=dt)
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
        self.gmc: Optional[CameraMotionCompensator] = (
            SparseOFCompensator(downscale=cmc_downscale) if use_cmc else None
        )
        self.tracks: List[Track] = []

    # ------------------------------------------------------------------
    # Camera-motion compensation
    # ------------------------------------------------------------------
    def _affine_for_frame(self, frame: np.ndarray, fg_boxes=None) -> Optional[np.ndarray]:
        if self.gmc is None:
            return None
        A = self.gmc(frame, fg_boxes=fg_boxes)
        if A is None:
            return None
        if not affine_is_pure_camera_pan(A):
            return None
        return A

    # ------------------------------------------------------------------
    # Association cost: fused IoU + Mahalanobis (+ optional ReID)
    # ------------------------------------------------------------------
    def _fused_cost(
        self,
        tracks: List[Track],
        detections: List[Box],
        det_embeddings: Optional[List[Optional[np.ndarray]]] = None,
    ) -> np.ndarray:
        big = 1e4
        cost = np.full((len(tracks), len(detections)), big, dtype=np.float64)
        w_iou, w_maha = 0.6, 0.4
        maha_gate = CHI2_INV_95_4DOF
        ious = iou_matrix([_predicted_box(t) for t in tracks], detections)
        for i, tr in enumerate(tracks):
            pred = _predicted_box(tr)
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
                # Optional ReID second-stage term.  Add a ReID distance term that
                # pushes matched-looking embeddings toward low cost.
                if (
                    self.appearance_reid_weight > 0
                    and det_embeddings is not None
                    and det_embeddings[j] is not None
                    and tr.embedding_mean is not None
                ):
                    emb = det_embeddings[j]
                    cos_d = float(1.0 - np.dot(emb, tr.embedding_mean))
                    cost[i, j] = (
                        (1.0 - self.appearance_reid_weight) * cost[i, j]
                        + self.appearance_reid_weight * cos_d
                    )
        return cost

    @staticmethod
    def _accept_match(tr: Track, det: Box, cost: float) -> bool:
        if cost >= 1e4 - 1:
            return False
        if tr.label != det.label:
            return False
        return cost < 0.85

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------
    def step(
        self,
        frame: np.ndarray,
        detections: List[Box],
        det_embeddings: Optional[List[Optional[np.ndarray]]] = None,
    ) -> List[Track]:
        # 1. CMC warp from k-1 -> k.
        warp = self._affine_for_frame(frame, fg_boxes=detections)

        # 2. Predict + apply CMC.
        for t in self.tracks:
            t.was_lost_before_update = (t.lost_age > 0)
            t.predict(self.kf)
            if warp is not None:
                t.mean, t.cov = self.kf.apply_affine(t.mean, t.cov, warp)

        # 3. Split detections by score.
        high = [d for d in detections if d.score >= self.high_conf]
        low = [d for d in detections if self.new_track_conf <= d.score < self.high_conf]

        if det_embeddings is None:
            # Build per-detection embedding list aligned with `detections`.
            emb_by_box: Dict[int, Optional[np.ndarray]] = {}
        else:
            emb_by_box = {id(d): det_embeddings[i] for i, d in enumerate(detections)}

        def embs_for(subset: List[Box]) -> List[Optional[np.ndarray]]:
            return [emb_by_box.get(id(d)) for d in subset]

        matched_tracks: set = set()
        matched_dets: set = set()

        # 4. First association: high-conf vs ALL tracks.
        if high and self.tracks:
            cost = self._fused_cost(self.tracks, high, embs_for(high))
            row_ind, col_ind = linear_sum_assignment(cost)
            for r, c in zip(row_ind, col_ind):
                if not self._accept_match(self.tracks[r], high[c], cost[r, c]):
                    continue
                self.tracks[r].update(self.kf, high[c])
                matched_tracks.add(r)
                matched_dets.add(c)

        # 5. Second association: low-conf vs remaining tracks.
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

        # 6. Anti-fragmentation relink: unmatched high-conf vs recently-lost.
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

        # 7. Mark unmatched tracks as missed.
        for i, tr in enumerate(self.tracks):
            if i not in matched_tracks:
                tr.mark_missed()

        # 8. Spawn new tracks for unmatched high-conf detections.
        for j, d in enumerate(high):
            if j not in matched_dets:
                self.tracks.append(self._spawn(d))

        # 9. Flash relinked tracks.
        for i in matched_tracks:
            tr = self.tracks[i]
            if tr.was_lost_before_update:
                tr.relink_remaining = self.RELINK_FLASH_FRAMES

        self._prune()
        return [t for t in self.tracks if t.state != 2]

    def _spawn(self, det: Box) -> Track:
        z = np.array([det.cx, det.cy, det.w, det.h], dtype=np.float64)
        mean, cov = self.kf.initiate(z)
        return Track(
            track_id=self._next_id(),
            label=det.label,
            mean=mean,
            cov=cov,
            box=det,
            n_init=self.n_init,
            trail=[(det.cx, det.cy)],
            pred_trail=[(float(mean[0]), float(mean[1]))],
            trail_scores=[det.score],
        )

    def _next_id(self) -> int:
        # Tracks are numbered globally so consumers don't have to deal with
        # per-step renumbering when tracks are pruned.
        BoTSortTracker._id_seq += 1
        return BoTSortTracker._id_seq

    _id_seq = 0

    # ------------------------------------------------------------------
    # Lifecycle / pruning
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
            too_old_lost = (t.state == 1 and t.lost_age > self.max_age)
            tentative_dead = (not t.confirmed and t.lost_age > self.max_age)
            delete = stationary or too_old_lost or tentative_dead
            if delete:
                t.state = 2
                continue
            alive.append(t)
        self.tracks = alive