"""Cost-matrix builders, IoU, and chi-squared gating."""

from __future__ import annotations

from typing import List, Sequence

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