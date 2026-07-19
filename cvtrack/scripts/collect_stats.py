"""Multi-video stats collector.

Run cvtrack on every <source> listed in --sources and summarise the resulting
`tracks.csv` (n tracks, mean track length, fps, explosion warnings).

Usage:
    python scripts/collect_stats.py --sources clip1.mp4 clip2.mp4 --out-dir /tmp/x
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path
from typing import List


def _run(cvtrack_args: List[str], cwd: Path) -> dict:
    t0 = time.perf_counter()
    res = subprocess.run(cvtrack_args, cwd=cwd, capture_output=True, text=True)
    dt = time.perf_counter() - t0
    return {"exit": res.returncode, "elapsed": dt, "stdout": res.stdout, "stderr": res.stderr}


def _summarise(tracks_csv: Path) -> dict:
    counts: Counter[int] = Counter()
    max_frame = 0
    with tracks_csv.open() as f:
        for row in csv.DictReader(f):
            tid = int(row["track_id"])
            counts[tid] += 1
            max_frame = max(max_frame, int(row["frame"]))
    if not counts:
        return {"n_tracks": 0, "mean_track_length": 0.0, "last_frame": 0}
    lengths = list(counts.values())
    return {
        "n_tracks": len(lengths),
        "mean_track_length": sum(lengths) / len(lengths),
        "last_frame": max_frame,
    }


def _main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sources", nargs="+", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--config", default="drone",
                    help="config preset name or path (default: drone)")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[1]
    results = []
    for source in args.sources:
        run_dir = Path(args.out_dir) / Path(source).stem
        run_dir.mkdir(parents=True, exist_ok=True)
        cmd = [
            sys.executable,
            "-m",
            "cvtrack",
            "--config",
            args.config,
            "--source",
            source,
            "--out-dir",
            str(run_dir),
            "--log-level",
            "WARNING",
        ]
        if args.max_frames > 0:
            cmd.extend(["--max-frames", str(args.max_frames)])
        t0 = time.perf_counter()
        env = {**__import__("os").environ,
               "PYTHONPATH": "/tmp/cvfix:" + str(repo / "src") + ":" + __import__("os").environ.get("PYTHONPATH", "")}
        rc = subprocess.run(cmd, cwd=repo, capture_output=True, text=True, env=env)
        elapsed = time.perf_counter() - t0

        tracks_csv = run_dir / "tracks.csv"
        stats = _summarise(tracks_csv) if tracks_csv.exists() else {}
        # Pull fps from the cvtrack log if available.
        fps_match = None
        for line in rc.stdout.splitlines() + rc.stderr.splitlines():
            if "fps" in line.lower():
                fps_match = line.strip()
        summary = {
            "source": source,
            "elapsed_s": round(elapsed, 2),
            "log_fps_line": fps_match,
            **stats,
        }
        results.append(summary)
        print(json.dumps(summary, indent=2))

    out_json = Path(args.out_dir) / "stats_summary.json"
    out_json.write_text(json.dumps(results, indent=2))
    print(f"\nWrote {out_json}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
