"""Post-process a tracks.csv into trajectories.png, summary, etc.

CLI:

    python -m cvtrack.postprocess --tracks out/tracks.csv --gt sample_gt.csv \
        --out-dir out/post
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load_tracks_csv(path: str) -> List[dict]:
    rows: List[dict] = []
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {h: i for i, h in enumerate(header)}
        for r in reader:
            rows.append({
                "frame": int(r[idx["frame"]]),
                "track_id": int(r[idx["track_id"]]),
                "label": r[idx["label"]],
                "cx": float(r[idx["cx"]]),
                "cy": float(r[idx["cy"]]),
                "vx": float(r[idx["vx"]]),
                "vy": float(r[idx["vy"]]),
                "confirmed": bool(int(r[idx["confirmed"]])),
            })
    return rows


def summarize(rows: List[dict]) -> tuple:
    by_id: Dict[int, List[dict]] = defaultdict(list)
    for r in rows:
        by_id[r["track_id"]].append(r)
    summary: List[dict] = []
    for tid, rlist in sorted(by_id.items()):
        rlist.sort(key=lambda x: x["frame"])
        n = len(rlist)
        dist = 0.0
        for a, b in zip(rlist[:-1], rlist[1:]):
            dx = b["cx"] - a["cx"]
            dy = b["cy"] - a["cy"]
            dist += (dx * dx + dy * dy) ** 0.5
        avg_speed = dist / max(n - 1, 1)
        summary.append({
            "track_id": tid,
            "label": rlist[0]["label"],
            "start_frame": rlist[0]["frame"],
            "end_frame": rlist[-1]["frame"],
            "length_pts": n,
            "path_length_px": round(dist, 2),
            "avg_speed_px_per_frame": round(avg_speed, 3),
        })
    return summary, by_id


def plot_trajectories(by_id: Dict[int, List[dict]], gt_path: Optional[str], out_png: str) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    cmap = plt.get_cmap("tab10")
    for tid, rlist in sorted(by_id.items()):
        xs = [r["cx"] for r in rlist]
        ys = [r["cy"] for r in rlist]
        ax.plot(xs, ys, "-o", markersize=2, linewidth=1.2,
                color=cmap(tid % 10), label=f"id {tid} ({rlist[0]['label']})")
    ax.set_title("Tracker trajectories")
    ax.set_xlabel("cx (px)")
    ax.set_ylabel("cy (px)")
    ax.invert_yaxis()
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, linestyle=":", alpha=0.5)

    if gt_path and os.path.exists(gt_path):
        gt: Dict[int, list] = defaultdict(list)
        with open(gt_path) as f:
            rdr = csv.DictReader(f)
            for r in rdr:
                gt[int(r["obj_id"])].append((int(r["frame"]),
                                              float(r["cx"]),
                                              float(r["cy"]),
                                              r["label"]))
        for obj_id, pts in gt.items():
            xs = [p[1] for p in pts]
            ys = [p[2] for p in pts]
            ax.plot(xs, ys, "x", markersize=4, alpha=0.5,
                    color="black", label=f"GT #{obj_id}" if obj_id == 0 else None)

    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="cvtrack.postprocess")
    ap.add_argument("--tracks", required=True, help="path to tracks.csv")
    ap.add_argument("--gt", default=None, help="optional GT csv to overlay")
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args(argv)

    rows = load_tracks_csv(args.tracks)
    summary, by_id = summarize(rows)
    os.makedirs(args.out_dir, exist_ok=True)

    long_path = os.path.join(args.out_dir, "tracks_per_id.csv")
    with open(long_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["track_id", "frame", "label", "cx", "cy", "vx", "vy", "confirmed"])
        for r in rows:
            w.writerow([r["track_id"], r["frame"], r["label"],
                        r["cx"], r["cy"], r["vx"], r["vy"], int(r["confirmed"])])

    summary_path = os.path.join(args.out_dir, "tracks_summary.csv")
    if summary:
        with open(summary_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
            w.writeheader()
            w.writerows(summary)
    else:
        with open(summary_path, "w", newline="") as f:
            f.write("track_id,label,start_frame,end_frame,length_pts,path_length_px,avg_speed_px_per_frame\n")
        by_id = {}

    png_path = os.path.join(args.out_dir, "trajectories.png")
    if by_id:
        plot_trajectories(by_id, args.gt, png_path)
    else:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.text(0.5, 0.5, "no tracks produced", ha="center", va="center", fontsize=14)
        ax.set_axis_off()
        fig.tight_layout()
        fig.savefig(png_path, dpi=120)
        plt.close(fig)

    print(f"[post] wrote {long_path}")
    print(f"[post] wrote {summary_path}")
    print(f"[post] wrote {png_path}")
    if summary:
        print(f"[post] {len(summary)} tracks, longest has "
              f"{max(s['length_pts'] for s in summary)} pts")
    else:
        print("[post] no tracks to summarise (empty input CSV)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))