#!/usr/bin/env python3
"""Compute simple headline metrics from a tracker run directory."""

from __future__ import annotations

import csv
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

# Add src to path
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

import numpy as np


def _load_boxes(csv_path: Path):
    """Yield (frame, track_id, Box) tuples from a tracks.csv file."""
    from cvtrack.types import Box
    rows = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append((
                    int(r["frame"]),
                    int(r["track_id"]),
                    Box(
                        x1=float(r["cx"]) - 10, y1=float(r["cy"]) - 10,
                        x2=float(r["cx"]) + 10, y2=float(r["cy"]) + 10,
                        score=1.0, cls=0,
                        label=r.get("label", "obj"),
                    ),
                ))
            except (KeyError, ValueError):
                continue
    return rows


def summarise(run_dir: Path, gt_run_dir: Path | None = None) -> dict:
    csv_path = run_dir / "tracks.csv"
    if not csv_path.exists():
        return {"run_dir": str(run_dir), "error": "no tracks.csv"}

    obs = _load_boxes(csv_path)
    if not obs:
        return {"run_dir": str(run_dir), "n_obs": 0}

    # Per-track length.
    per_track_frames: dict[int, set] = defaultdict(set)
    n_confirmed = 0
    for frame, tid, _ in obs:
        per_track_frames[tid].add(frame)

    track_lengths = [len(s) for s in per_track_frames.values()]
    n_tracks = len(per_track_frames)
    mean_len = float(np.mean(track_lengths)) if track_lengths else 0.0
    median_len = float(np.median(track_lengths)) if track_lengths else 0.0

    n_frames = max(frame for frame, _, _ in obs) + 1 if obs else 0

    out = {
        "run_dir": str(run_dir),
        "n_obs": len(obs),
        "n_tracks": n_tracks,
        "mean_track_length": mean_len,
        "median_track_length": median_len,
        "n_frames": n_frames,
    }

    # Optional IDF1 against a ground-truth run directory (same scheme).
    if gt_run_dir is not None:
        gt_csv = gt_run_dir / "tracks.csv"
        if gt_csv.exists():
            from cvtrack.tracker.metrics import idf1
            gt_obs = _load_boxes(gt_csv)
            gt_ids = [t for _, t, _ in gt_obs]
            gt_boxes = [b for _, _, b in gt_obs]
            pr_ids = [t for _, t, _ in obs]
            pr_boxes = [b for _, _, b in obs]
            # Re-index gt ids so they start at 1 (in case the same physical
            # track was numbered differently across runs).
            remap: dict[int, int] = {}
            next_id = 1
            for g in gt_ids:
                if g not in remap:
                    remap[g] = next_id
                    next_id += 1
            gt_ids = [remap[g] for g in gt_ids]
            metric = idf1(gt_ids, pr_ids, gt_boxes, pr_boxes)
            out["idf1"] = metric["idf1"]
            out["idp"] = metric["idp"]
            out["idr"] = metric["idr"]
            out["idf1_tp"] = metric["tp"]
            out["idf1_fp"] = metric["fp"]
            out["idf1_fn"] = metric["fn"]
    return out


def main():
    runs = [
        Path("weights/run_deepsort_legacy"),
        Path("weights/run_deepsort_cascade"),
        Path("weights/run_botsort"),
    ]
    results = []
    for r in runs:
        if r.exists():
            results.append(summarise(r))
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()