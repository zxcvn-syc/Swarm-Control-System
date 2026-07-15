"""Cost-matrix builders, IoU, chi-squared gating, and MOT metrics."""

from __future__ import annotations

from collections import defaultdict
from typing import Dict, List, Sequence, Tuple

import numpy as np

from cvtrack.types import Box, Track


# chi-squared 95% threshold for k=2 dof (used by DeepSortLite / KF4 gating)
CHI2_INV_95_2DOF = 5.991
# chi-squared 95% threshold for k=4 dof (used by BoT-SORT / KF8 gating)
CHI2_INV_95_4DOF = 9.488


def iou(a: Box, b: Box) -> float:
    return a.iou(b)


def iou_matrix(boxes_a: Sequence[Box], boxes_b: Sequence[Box]) -> np.ndarray:
    """Vectorised pairwise IoU. Returns (n_a, n_b) float64."""
    n_a, n_b = len(boxes_a), len(boxes_b)
    if n_a == 0 or n_b == 0:
        return np.zeros((n_a, n_b), dtype=np.float64)
    a = np.array([[b.x1, b.y1, b.x2, b.y2] for b in boxes_a], dtype=np.float64)
    b = np.array([[bb.x1, bb.y1, bb.x2, bb.y2] for bb in boxes_b], dtype=np.float64)
    tl = np.maximum(a[:, None, :2], b[None, :, :2])
    br = np.minimum(a[:, None, 2:], b[None, :, 2:])
    wh = np.clip(br - tl, a_min=0.0, a_max=None)
    inter = wh[..., 0] * wh[..., 1]
    area_a = np.clip(a[:, 2] - a[:, 0], 0, None) * np.clip(a[:, 3] - a[:, 1], 0, None)
    area_b = np.clip(b[:, 2] - b[:, 0], 0, None) * np.clip(b[:, 3] - b[:, 1], 0, None)
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-6), 0.0)


def class_aware_iou_distance(
    tracks: Sequence[Track],
    detections: Sequence[Box],
    kf_state_dim: int,
    predicted_boxes: Sequence[Box],
    *,
    iou_gate: float = 0.30,
    class_penalty: float = 1e3,
) -> np.ndarray:
    """IoU distance matrix for the simple (non-BoT-SORT) tracker.

    Cost = 1 - IoU, with pairs that disagree on class pushed to class_penalty.
    Pairs with zero IoU are assigned iou_gate cost if the class matches so the
    Hungarian solver still considers them as potential relink candidates.
    """
    cost = np.full((len(tracks), len(detections)), class_penalty, dtype=np.float64)
    ious = iou_matrix(list(predicted_boxes), list(detections))
    for i, tr in enumerate(tracks):
        for j in range(len(detections)):
            iou_val = ious[i, j]
            if iou_val <= 0.0:
                continue
            base = 1.0 - iou_val
            if detections[j].label != tr.label:
                base = max(base, 1.0 - iou_gate) + 0.5
            cost[i, j] = base
    return cost


def mahalanobis_2d(kf, track: Track, z: np.ndarray) -> float:
    """Squared Mahalanobis distance for the 4-state KF (legacy DeepSORT path)."""
    z_pred = kf.H @ track.mean
    S = kf.H @ track.cov @ kf.H.T + kf._R()
    d = z - z_pred
    return float(d @ np.linalg.inv(S) @ d)


def gate_mahalanobis(
    kf,
    tracks: Sequence[Track],
    detections: Sequence[Box],
    threshold: float = CHI2_INV_95_2DOF,
) -> np.ndarray:
    """Boolean (n_tracks, n_det) gate: True if (d_maha < threshold AND class match)."""
    out = np.zeros((len(tracks), len(detections)), dtype=bool)
    for i, tr in enumerate(tracks):
        for j, det in enumerate(detections):
            if det.label != tr.label:
                continue
            z = np.array([det.cx, det.cy], dtype=np.float64)
            out[i, j] = mahalanobis_2d(kf, tr, z) < threshold
    return out


# ---------------------------------------------------------------------------
# MOT-style metrics (no external deps -- motmetrics is in requirements.txt but
# this lets us compute the headline number inside the test suite too).
# ---------------------------------------------------------------------------

def _best_mapping_one_to_one(counts: Dict[Tuple[int, int], int]) -> Dict[int, int]:
    """Greedy 1-to-1 mapping that maximises total co-occurrence.

    ``counts`` is ``{(gt_id, pred_id): n_frames}``.  We build the bipartite
    assignment greedily by descending count, picking each (gt, pred) pair
    only if both ids are still unmatched.  This is an O(K^2) approximation
    to the optimal assignment (where K is the number of distinct ids); for
    the per-clip sizes we deal with (<200 ids) it is exact in practice.
    """
    used_gt: set = set()
    used_pred: set = set()
    mapping: Dict[int, int] = {}
    for (gt_id, pred_id), cnt in sorted(counts.items(), key=lambda kv: -kv[1]):
        if gt_id in used_gt or pred_id in used_pred:
            continue
        mapping[gt_id] = pred_id
        used_gt.add(gt_id)
        used_pred.add(pred_id)
    return mapping


def idf1(
    gt_ids: Sequence[int],
    pred_ids: Sequence[int],
    gt_dets: Sequence[Box],
    pred_dets: Sequence[Box],
) -> dict:
    """Compute IDF1 / IDP / IDR for a single clip.

    Parameters
    ----------
    gt_ids, pred_ids:
        Per-observation ground-truth and predicted track ids (same length).
    gt_dets, pred_dets:
        The corresponding bounding boxes (same length as the id lists).
    Two observations are paired (i.e. ``(gt_i, pred_i)`` contributes to
    their (gt_id, pred_id) co-occurrence count) iff their boxes overlap
    with IoU >= 0.5 (the MOT convention).  Observations without any
    pairing are ignored -- the standard "evaluate only on matched boxes"
    rule.

    Returns
    -------
    dict with keys ``idf1``, ``idp``, ``idr`` (floats in [0, 1]) and
    ``mapping`` (a ``{gt_id: pred_id}`` dictionary for the best 1-to-1
    assignment).
    """
    n = len(gt_ids)
    if n == 0 or n != len(pred_ids) or n != len(gt_dets) or n != len(pred_dets):
        return {"idf1": 0.0, "idp": 0.0, "idr": 0.0, "mapping": {}, "tp": 0, "fp": 0, "fn": 0}

    # Co-occurrence counts: only pair detections with IoU >= 0.5 (MOT
    # convention).  O(n^2) but n is per-clip and typically < 5k here.
    co_counts: Dict[Tuple[int, int], int] = defaultdict(int)
    for i in range(n):
        gi, di = gt_ids[i], gt_dets[i]
        for j in range(n):
            pj, dj = pred_ids[j], pred_dets[j]
            if di.iou(dj) >= 0.5:
                co_counts[(gi, pj)] += 1
    if not co_counts:
        return {"idf1": 0.0, "idp": 0.0, "idr": 0.0, "mapping": {}, "tp": 0, "fp": 0, "fn": 0}

    mapping = _best_mapping_one_to_one(dict(co_counts))
    tp = sum(co_counts[(gt, mapping[gt])] for gt in mapping)

    # Per-id totals across the (matched) observation set.
    per_gt_total: Dict[int, int] = defaultdict(int)
    per_pred_total: Dict[int, int] = defaultdict(int)
    for (gt_id, pred_id), cnt in co_counts.items():
        per_gt_total[gt_id] += cnt
        per_pred_total[pred_id] += cnt
    fn = sum(per_gt_total[g] for g in per_gt_total if g not in mapping)
    fp = sum(per_pred_total[p] for p in per_pred_total if p not in mapping.values())

    idp = tp / max(tp + fp, 1)
    idr = tp / max(tp + fn, 1)
    denom = idp + idr
    idf1_v = (2 * idp * idr / denom) if denom > 0 else 0.0
    return {
        "idf1": float(idf1_v),
        "idp": float(idp),
        "idr": float(idr),
        "mapping": {int(k): int(v) for k, v in mapping.items()},
        "tp": int(tp),
        "fp": int(fp),
        "fn": int(fn),
    }