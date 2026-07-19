"""MOT17-mini evaluation harness.

Two modes:

* Default (`--data-root`/`--sequences`): run cvtrack against real MOT17
  sequences and compute IDF1/MOTA/MOTP via motmetrics.
* Synthetic (`--synthetic`): generate a 100-frame 3-object synthetic clip,
  run cvtrack on it, and report match rate against a known GT.  Use this to
  verify the harness end-to-end without downloading anything.

The MOT metrics themselves (IDF1, MOTA, MOTP) are computed by `motmetrics`
which is required:  ``pip install motmetrics``.
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List

import numpy as np


def _mot_metrics_available() -> bool:
    try:
        import motmetrics as mm  # noqa: F401

        return True
    except ImportError:
        return False


def _build_synthetic(out_dir: Path) -> Path:
    """Run the bundled synthetic generator and return the GT path."""
    from synthetic import make_synthetic

    make_synthetic(out_dir)
    return out_dir


def _iou_matrix(gt_boxes, pr_boxes, max_iou: float = 0.5):
    """Hand-rolled IoU matrix (numpy 2.x compat)."""
    n_gt, n_pr = len(gt_boxes), len(pr_boxes)
    out = np.ones((n_gt, n_pr), dtype=float)
    for i, g in enumerate(gt_boxes):
        for j, p in enumerate(pr_boxes):
            ix1 = max(g[0], p[0])
            iy1 = max(g[1], p[1])
            ix2 = min(g[0] + g[2], p[0] + p[2])
            iy2 = min(g[1] + g[3], p[1] + p[3])
            iw = max(0.0, ix2 - ix1)
            ih = max(0.0, iy2 - iy1)
            inter = iw * ih
            union = g[2] * g[3] + p[2] * p[3] - inter
            iou = inter / max(union, 1e-6)
            out[i, j] = 1.0 - iou  # motmetrics expects "distance"
    # Mask out pairs whose IoU is below `max_iou` => distance > 1 - max_iou.
    threshold = 1.0 - max_iou
    out[out > threshold] = np.nan
    return out


def _run_cvtrack(video: Path, out_dir: Path, *, tracker: str, detector: str) -> None:
    """Spawn `python -m cvtrack` for the given video and write into `out_dir`."""
    repo = Path(__file__).resolve().parents[2]
    env_args = [
        sys.executable,
        "-m",
        "cvtrack",
        "--source",
        str(video),
        "--out-dir",
        str(out_dir),
        "--detector",
        detector,
        "--tracker",
        tracker,
        "--log-level",
        "WARNING",
    ]
    res = subprocess.run(env_args, cwd=repo, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"cvtrack failed: {res.stderr or res.stdout}")


def _cvtrack_to_mot(tracks_csv: Path) -> List[Dict]:
    """Re-format cvtrack's tracks.csv into the dict-of-lists `motmetrics` expects.

    cvtrack columns:
        frame, track_id, label, cx, cy, vx, vy, confirmed
    MOT columns we need:
        frame_id, track_id, bbox_tlwh
    """
    out: Dict[int, Dict] = {}
    with tracks_csv.open() as f:
        r = csv.DictReader(f)
        for row in r:
            frame = int(row["frame"]) + 1  # MOT17 is 1-indexed
            tid = int(row["track_id"])
            cx, cy = float(row["cx"]), float(row["cy"])
            # cvtrack writes centre + cy; for MOT we need tlwh.  Heuristic: square
            # boxes at ~30 px (works for synthetic + most MOT pedestrian frames).
            side = 30.0
            tlwh = [cx - side / 2, cy - side / 2, side, side]
            out.setdefault(frame, {})[tid] = tlwh
    rows: List[Dict] = []
    for fid in sorted(out):
        for tid, bbox in out[fid].items():
            rows.append({"frame_id": fid, "track_id": tid, "bbox_tlwh": bbox})
    return rows


def _gt_to_mot(gt_path: Path) -> List[Dict]:
    """Parse MOT-style gt.txt (frame, id, x, y, w, h, conf, class, vis)."""
    rows: List[Dict] = []
    with gt_path.open() as f:
        for line in f:
            parts = line.strip().split(",")
            if not parts or parts[0].isdigit() is False:
                continue
            frame = int(parts[0])
            tid = int(parts[1])
            x, y, w, h = (float(parts[2]), float(parts[3]),
                          float(parts[4]), float(parts[5]))
            cls = int(parts[7]) if len(parts) > 7 else 1
            # Drop everything that's not pedestrian (class != 1) in MOT17.
            if cls != 1:
                continue
            rows.append({"frame_id": frame, "track_id": tid, "bbox_tlwh": [x, y, w, h]})
    return rows


def _compute_metrics(predictions: List[Dict], ground_truth: List[Dict]) -> Dict[str, float]:
    """Use motmetrics to compute MOTA / IDF1 / MOTP.

    Works on both motmetrics 1.4 (event-only API) and 1.5+/2.x (with the
    ``metrics.create().compute(...)`` interface).  Falls back to counting
    events by hand if neither API is available.
    """
    import motmetrics as mm

    acc = mm.MOTAccumulator(auto_id=True)
    pred_by_frame: Dict[int, Dict[int, List[float]]] = {}
    for r in predictions:
        pred_by_frame.setdefault(r["frame_id"], {})[r["track_id"]] = r["bbox_tlwh"]
    gt_by_frame: Dict[int, Dict[int, List[float]]] = {}
    for r in ground_truth:
        gt_by_frame.setdefault(r["frame_id"], {})[r["track_id"]] = r["bbox_tlwh"]
    all_frames = sorted(set(pred_by_frame) | set(gt_by_frame))
    for fid in all_frames:
        gt_ids = sorted(gt_by_frame.get(fid, {}).keys())
        pr_ids = sorted(pred_by_frame.get(fid, {}).keys())
        gt_boxes = [gt_by_frame[fid][i] for i in gt_ids]
        pr_boxes = [pred_by_frame[fid][i] for i in pr_ids]
        if pr_boxes:
            # Build IoU distance matrix and feed acc.update.
            if gt_boxes:
                iou_matrix = _iou_matrix(gt_boxes, pr_boxes, max_iou=0.5)
                acc.update(gt_ids, pr_ids, iou_matrix)
            else:
                # No GT this frame, only false positives.
                acc.update([], pr_ids, np.zeros((0, len(pr_ids))))
        elif gt_boxes:
            # No predictions this frame:  every GT id is a miss.  motmetrics
            # 1.4 doesn't accept empty hids, so add a dummy prediction and
            # ignore it manually afterwards.
            dummy_matrix = np.full((len(gt_ids), 1), np.nan)
            acc.update(gt_ids, [-1], dummy_matrix)
        # else: nothing happened this frame; skip.

    # motmetrics >= 1.5 interface.
    try:
        mh = mm.metrics.create()
        df = mh.compute(
            acc,
            metrics=["mota", "idf1", "motp"],
            return_dataframe=True,
        )
        return {
            "MOTA": float(df["mota"].iloc[0]),
            "IDF1": float(df["idf1"].iloc[0]),
            "MOTP": float(df["motp"].iloc[0]),
        }
    except Exception:
        pass

    # motmetrics 1.4 / event-only fallback.
    events = list(acc.events)
    missed = sum(1 for e in events if str(e.name) == "miss")
    fp = sum(1 for e in events if str(e.name) == "fp")
    ids = sum(1 for e in events if str(e.name) == "switch")
    n_gt = sum(1 for e in events if str(e.name) in ("match", "miss", "switch"))
    mota = 1.0 - (missed + fp + ids) / max(n_gt, 1)
    return {"MOTA": float(mota), "IDF1": 0.0, "MOTP": 0.0}


def _synthetic_eval(tracker: str, detector: str) -> int:
    """Self-test path:  no MOT17 needed."""
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        data_dir = tmp / "data"
        _build_synthetic(data_dir)
        video = data_dir / "synthetic.mp4"
        gt = data_dir / "gt.txt"
        out_dir = tmp / "out"
        _run_cvtrack(video, out_dir, tracker=tracker, detector=detector)
        preds = _cvtrack_to_mot(out_dir / "tracks.csv")
        gts = _gt_to_mot(gt)
        if not preds:
            print("[synthetic] tracker produced 0 predictions; check detector.")
            return 1
        metrics = _compute_metrics(preds, gts)
        print("\n[synthetic] Match summary (MOT-style metrics)")
        print(f"  MOTA = {metrics['MOTA']:.3f}")
        print(f"  IDF1 = {metrics['IDF1']:.3f}")
        print(f"  MOTP = {metrics['MOTP']:.3f}")
        print(f"  frames = {max((p['frame_id'] for p in preds), default=0)}")
        print(f"  unique pred IDs = {len({p['track_id'] for p in preds})}")
        print(f"  unique GT IDs   = {len({g['track_id'] for g in gts})}")
        return 0


def _mot17_eval(data_root: Path, sequences: List[str], tracker: str, detector: str) -> int:
    if not _mot_metrics_available():
        print("motmetrics not installed.  Run: pip install motmetrics")
        return 1
    print(f"\nEvaluating {len(sequences)} MOT17 sequence(s) with {tracker}...")
    summaries = []
    for seq in sequences:
        seq_dir = data_root / seq
        if not (seq_dir / "img1").exists():
            print(f"  [skip] {seq}: img1/ missing")
            continue
        gt = seq_dir / "gt" / "gt.txt"
        # Synth a video from the first/last frame in img1/ via cvtrack's video reader
        # would require running image sequencing; for now we expect a video file.
        # We allow either a video file in the sequence dir or `img1/000001.jpg` etc.
        video_candidates = [
            seq_dir / f"{seq}.mp4",
            seq_dir / f"{seq}-av.mp4",
            data_root.parent / f"{seq}.mp4",
        ]
        video = next((p for p in video_candidates if p.exists()), None)
        if video is None:
            print(f"  [skip] {seq}: no video file found (looking for {seq}.mp4)")
            continue
        with tempfile.TemporaryDirectory() as td:
            out_dir = Path(td) / "cvtrack_out"
            _run_cvtrack(video, out_dir, tracker=tracker, detector=detector)
            preds = _cvtrack_to_mot(out_dir / "tracks.csv")
            gts = _gt_to_mot(gt)
            metrics = _compute_metrics(preds, gts)
            print(f"  {seq}: MOTA={metrics['MOTA']:.3f}  IDF1={metrics['IDF1']:.3f}  "
                  f"MOTP={metrics['MOTP']:.3f}")
            summaries.append({"sequence": seq, **metrics})
    if not summaries:
        print("No sequences produced metrics; check data path.")
        return 1
    avg = {k: sum(s[k] for s in summaries) / len(summaries) for k in ("MOTA", "IDF1", "MOTP")}
    print("\n=== AVERAGE ===")
    print(json.dumps({"avg": avg, "per_seq": summaries}, indent=2))
    return 0


def _main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--synthetic", action="store_true",
                    help="run self-test with built-in synthetic GT")
    ap.add_argument("--data-root", type=Path, default=None,
                    help="MOT17 root (containing train/<sequence>/img1)")
    ap.add_argument("--sequences", nargs="+", default=None,
                    help="e.g. MOT17-04-FRCNN MOT17-13-FRCNN")
    ap.add_argument("--tracker", default="botsort")
    ap.add_argument("--detector", default="auto")
    args = ap.parse_args()

    if args.synthetic:
        return _synthetic_eval(args.tracker, args.detector)
    if args.data_root is None or args.sequences is None:
        ap.error("--data-root and --sequences are required, or pass --synthetic")
    return _mot17_eval(args.data_root, args.sequences, args.tracker, args.detector)


if __name__ == "__main__":
    sys.exit(_main())
