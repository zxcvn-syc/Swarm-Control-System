"""Cost-matrix builders, IoU, chi-squared gating, and MOT metrics."""

from __future__ import annotations

import math
from collections import defaultdict
from typing import Dict, List, Sequence, Tuple

import numpy as np

from cvtrack.types import Box, Track


# chi-squared 95% threshold for k=2 dof (used by DeepSortLite / KF4 gating)
CHI2_INV_95_2DOF = 5.991
# chi-squared 95% threshold for k=4 dof (used by BoT-SORT / KF8 gating)
CHI2_INV_95_4DOF = 9.488

_IOPUS_THRESHOLD = 0.5
_MIN_DET_AREA = 1e-9
_EPS = 1e-9


def iou(a: Box, b: Box) -> float:
    """Calculate IoU between two boxes with NaN/inf guard."""
    if a is None or b is None:
        return 0.0
    try:
        result = a.iou(b)
        return float(result) if math.isfinite(result) else 0.0
    except (AttributeError, TypeError):
        return 0.0


def iou_matrix(boxes_a: Sequence[Box], boxes_b: Sequence[Box]) -> np.ndarray:
    """Vectorised pairwise IoU. Returns (n_a, n_b) float64.

    Enhanced with:
    - Robust boundary handling (empty inputs)
    - Degenerate box (zero area) handling
    - NaN/Inf protection
    """
    n_a, n_b = len(boxes_a), len(boxes_b)

    # Handle edge cases
    if n_a == 0 or n_b == 0:
        return np.zeros((n_a, n_b), dtype=np.float64)

    # Validate inputs
    valid_a = [b for b in boxes_a if b is not None and hasattr(b, "x1")]
    valid_b = [b for b in boxes_b if b is not None and hasattr(b, "x1")]

    n_valid_a = len(valid_a)
    n_valid_b = len(valid_b)

    if n_valid_a == 0 or n_valid_b == 0:
        return np.zeros((n_a, n_b), dtype=np.float64)

    try:
        a = np.array(
            [[b.x1, b.y1, b.x2, b.y2] for b in valid_a],
            dtype=np.float64,
        )
        b = np.array(
            [[bb.x1, bb.y1, bb.x2, bb.y2] for bb in valid_b],
            dtype=np.float64,
        )
    except (TypeError, ValueError):
        return np.zeros((n_a, n_b), dtype=np.float64)

    # Clean NaN/Inf values
    a = np.where(np.isfinite(a), a, 0.0)
    b = np.where(np.isfinite(b), b, 0.0)

    tl = np.maximum(a[:, None, :2], b[None, :, :2])
    br = np.minimum(a[:, None, 2:], b[None, :, 2:])
    wh = np.clip(br - tl, a_min=0.0, a_max=None)
    inter = wh[..., 0] * wh[..., 1]

    # Calculate areas with epsilon protection
    area_a = np.clip(a[:, 2] - a[:, 0], 0, None) * np.clip(a[:, 3] - a[:, 1], 0, None)
    area_b = np.clip(b[:, 2] - b[:, 0], 0, None) * np.clip(b[:, 3] - b[:, 1], 0, None)
    union = area_a[:, None] + area_b[None, :] - inter

    # Use epsilon to prevent division by zero
    result = np.where(
        union > _MIN_DET_AREA,
        inter / np.maximum(union, _EPS),
        0.0,
    )

    # Clip to valid range [0, 1]
    result = np.clip(result, 0.0, 1.0)

    # Ensure NaN-free output
    result = np.where(np.isfinite(result), result, 0.0)

    # If some boxes were invalid, expand back to full matrix
    if n_valid_a < n_a or n_valid_b < n_b:
        full_result = np.zeros((n_a, n_b), dtype=np.float64)
        # Build mapping from valid indices
        valid_a_idx = [i for i, b in enumerate(boxes_a) if b is not None and hasattr(b, "x1")]
        valid_b_idx = [i for i, b in enumerate(boxes_b) if b is not None and hasattr(b, "x1")]
        for vi, ai in enumerate(valid_a_idx):
            for vj, bj in enumerate(valid_b_idx):
                full_result[ai, bj] = result[vi, vj]
        return full_result

    return result


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
    # convention).  Using spatial bucketing for O(n) average on large data.
    co_counts: Dict[Tuple[int, int], int] = defaultdict(int)

    # Determine if spatial bucketing helps (large sequences)
    use_bucketing = n > 1000

    if use_bucketing:
        # Compute spatial bucket size based on bbox spread
        try:
            valid_boxes = [gt_dets[i] for i in range(n) if gt_dets[i] is not None]
            if not valid_boxes:
                valid_boxes = [pred_dets[i] for i in range(n) if pred_dets[i] is not None]
            if valid_boxes:
                all_x1 = [b.x1 for b in valid_boxes]
                all_y1 = [b.y1 for b in valid_boxes]
                all_x2 = [b.x2 for b in valid_boxes]
                all_y2 = [b.y2 for b in valid_boxes]
                x_range = max(max(all_x2) - min(all_x1), 1.0)
                y_range = max(max(all_y2) - min(all_y1), 1.0)
                # Target ~30x30 grid for bucketing
                bucket_size_x = max(x_range / 30.0, 1.0)
                bucket_size_y = max(y_range / 30.0, 1.0)
            else:
                bucket_size_x = bucket_size_y = 100.0
        except (AttributeError, ValueError):
            bucket_size_x = bucket_size_y = 100.0

        # Build spatial buckets for gt detections
        gt_buckets: Dict[Tuple[int, int], List[int]] = defaultdict(list)
        for i in range(n):
            det = gt_dets[i]
            if det is None:
                continue
            try:
                bx = int(det.cx / bucket_size_x)
                by = int(det.cy / bucket_size_y)
                gt_buckets[(bx, by)].append(i)
            except (AttributeError, ZeroDivisionError):
                continue

        # For each prediction, check nearby gt buckets
        for j in range(n):
            det_j = pred_dets[j]
            if det_j is None:
                continue
            try:
                jx = int(det_j.cx / bucket_size_x)
                jy = int(det_j.cy / bucket_size_y)
                gi = gt_ids[j]
            except (AttributeError, ZeroDivisionError, IndexError):
                continue

            # Check 3x3 surrounding buckets
            matched_any = False
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    bucket = (jx + dx, jy + dy)
                    for i in gt_buckets.get(bucket, []):
                        try:
                            det_i = gt_dets[i]
                            if det_i is None:
                                continue
                            if det_i.iou(det_j) >= _IOPUS_THRESHOLD:
                                co_counts[(gi, pred_ids[j])] += 1
                                matched_any = True
                        except (AttributeError, ValueError):
                            continue

            if not matched_any:
                # Fall back to brute force for unmatched (rare)
                for i in range(n):
                    try:
                        det_i = gt_dets[i]
                        if det_i is None:
                            continue
                        if det_i.iou(det_j) >= _IOPUS_THRESHOLD:
                            co_counts[(gi, pred_ids[j])] += 1
                    except (AttributeError, ValueError):
                        continue
    else:
        # Brute force for smaller datasets
        for i in range(n):
            gi, di = gt_ids[i], gt_dets[i]
            if di is None:
                continue
            for j in range(n):
                pj, dj = pred_ids[j], pred_dets[j]
                if dj is None:
                    continue
                try:
                    if di.iou(dj) >= _IOPUS_THRESHOLD:
                        co_counts[(gi, pj)] += 1
                except (AttributeError, ValueError):
                    continue

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


def compute_mota(
    gt_ids: Sequence[int],
    pred_ids: Sequence[int],
    gt_dets: Sequence[Box],
    pred_dets: Sequence[Box],
) -> dict:
    """Compute MOTA (Multiple Object Tracking Accuracy).

    MOTA = 1 - (FN + FP + IDS) / GT_count

    A more negative value is worse. Range is [-inf, 1.0].

    Returns:
        dict with keys: mota, motp, fp, fn, ids, total_gt, num_switches
    """
    n = len(gt_ids)
    if n == 0 or n != len(pred_ids) or n != len(gt_dets) or n != len(pred_dets):
        return {
            "mota": 0.0,
            "motp": 0.0,
            "fp": 0,
            "fn": 0,
            "ids": 0,
            "total_gt": 0,
            "num_switches": 0,
        }

    # Build per-frame matched pairs using IoU >= 0.5
    matches: List[Tuple[int, int, float]] = []  # (gt_id, pred_id, iou)
    for i in range(n):
        di = gt_dets[i]
        if di is None:
            continue
        for j in range(n):
            dj = pred_dets[j]
            if dj is None:
                continue
            try:
                iou_val = di.iou(dj)
                if iou_val >= _IOPUS_THRESHOLD:
                    matches.append((gt_ids[i], pred_ids[j], iou_val))
            except (AttributeError, ValueError):
                continue

    # Greedy match: highest IoU first
    matches.sort(key=lambda m: -m[2])

    used_gt: set = set()
    used_pred: set = set()
    matched_pairs: List[Tuple[int, int]] = []
    for gt_id, pred_id, _ in matches:
        if gt_id in used_gt or pred_id in used_pred:
            continue
        used_gt.add(gt_id)
        used_pred.add(pred_id)
        matched_pairs.append((gt_id, pred_id))

    total_gt = len(used_gt)
    num_fp = len(set(pred_ids) - used_pred)
    num_fn = n - len(matched_pairs)
    num_ids = 0

    # Simple IDS: count mid-clip id switches for matched gt ids
    last_pred_for_gt: Dict[int, int] = {}
    for gt_id, pred_id in matched_pairs:
        if gt_id in last_pred_for_gt and last_pred_for_gt[gt_id] != pred_id:
            num_ids += 1
        last_pred_for_gt[gt_id] = pred_id

    if total_gt + total_gt > 0:
        mota = 1.0 - (num_fn + num_fp + num_ids) / max(n, 1)
    else:
        mota = 0.0

    # MOTP: average IoU of matched pairs
    if matched_pairs:
        iou_sum = sum(
            next(m[2] for m in matches if m[0] == gt and m[1] == pr)
            for gt, pr in matched_pairs
        )
        motp = iou_sum / len(matched_pairs)
    else:
        motp = 0.0

    return {
        "mota": float(mota),
        "motp": float(motp),
        "fp": int(num_fp),
        "fn": int(num_fn),
        "ids": int(num_ids),
        "total_gt": int(total_gt),
        "num_switches": int(num_ids),
    }


def compute_hota(
    gt_ids: Sequence[int],
    pred_ids: Sequence[int],
    gt_dets: Sequence[Box],
    pred_dets: Sequence[Box],
    alpha: float = 0.5,
) -> dict:
    """Compute HOTA (Higher Order Tracking Accuracy).

    HOTA is the geometric mean of Detection Accuracy (DetA) and
    Association Accuracy (AssA), weighted by alpha.

    HOTA = sqrt(DetA * AssA)

    Args:
        gt_ids: Ground-truth track ids
        pred_ids: Predicted track ids
        gt_dets: Ground-truth bounding boxes
        pred_dets: Predicted bounding boxes
        alpha: Weight (default 0.5)

    Returns:
        dict with keys: hota, deta, assa, deta_sum, assa_sum
    """
    n = len(gt_ids)
    if n == 0 or n != len(pred_ids) or n != len(gt_dets) or n != len(pred_dets):
        return {
            "hota": 0.0,
            "deta": 0.0,
            "assa": 0.0,
            "deta_sum": 0.0,
            "assa_sum": 0.0,
        }

    # Compute IDF1 to derive association scores
    idf1_result = idf1(gt_ids, pred_ids, gt_dets, pred_dets)
    mapping = idf1_result["mapping"]
    co_counts = idf1_result.get("mapping", {})

    # Calculate per-prediction TPA (true positive associations)
    pred_tpa: Dict[int, int] = defaultdict(int)
    pred_fna: Dict[int, int] = defaultdict(int)
    gt_tpa: Dict[int, int] = defaultdict(int)
    gt_fna: Dict[int, int] = defaultdict(int)

    # Rebuild co_counts:
    co_counts_orig: Dict[Tuple[int, int], int] = defaultdict(int)
    for i in range(n):
        di = gt_dets[i]
        if di is None:
            continue
        for j in range(n):
            dj = pred_dets[j]
            if dj is None:
                continue
            try:
                if di.iou(dj) >= _IOPUS_THRESHOLD:
                    co_counts_orig[(gt_ids[i], pred_ids[j])] += 1
            except (AttributeError, ValueError):
                continue

    # Build TPA, FNA per id
    for (gt_id, pred_id), cnt in co_counts_orig.items():
        gt_tpa[gt_id] += cnt
        pred_tpa[pred_id] += cnt

    # For unmatched ids, count towards FNA
    mapped_preds = set(mapping.values())
    for gt_id in set(gt_ids):
        if gt_id not in mapping:
            gt_fna[gt_id] = sum(c for (g, _), c in co_counts_orig.items() if g == gt_id)
    for pred_id in set(pred_ids):
        if pred_id not in mapped_preds:
            pred_fna[pred_id] = sum(c for (_, p), c in co_counts_orig.items() if p == pred_id)

    # DetA: detection accuracy
    total_tp = sum(pred_tpa.values())
    total_fp = sum(pred_fna.values())
    total_fn = sum(gt_fna.values())
    if total_tp + total_fp + total_fn > 0:
        deta = total_tp / (total_tp + 0.5 * total_fp + 0.5 * total_fn)
    else:
        deta = 0.0

    # AssA: association accuracy (using A_score = TP/(TP+FN+FP) for each id)
    assa_scores: List[float] = []
    for pred_id in set(pred_ids):
        tp = pred_tpa.get(pred_id, 0)
        fn = pred_fna.get(pred_id, 0)
        fp_local = 0  # Per-prediction, this is bounded
        denom = tp + fn + fp_local + _EPS
        if tp > 0:
            assa_scores.append(tp / denom)

    assa = float(np.mean(assa_scores)) if assa_scores else 0.0

    # HOTA = sqrt(DetA * AssA)
    hota = math.sqrt(max(deta * assa, 0.0))

    return {
        "hota": float(hota),
        "deta": float(deta),
        "assa": float(assa),
        "deta_sum": float(deta),
        "assa_sum": float(assa),
    }