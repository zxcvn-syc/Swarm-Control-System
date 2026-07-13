#!/usr/bin/env python3
"""VisDrone + ReID head-to-head comparison runner.

Runs 4 cvtrack configurations on the same drone clip and produces a unified
report.

Usage:
    python3 scripts/visdrone_compare.py --source pexels_aerial_2034115.mp4 \
        --frames 200 --out-dir weights
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SRC = ROOT / "src"

WEIGHTS = ROOT / "weights"
WEIGHTS.mkdir(exist_ok=True)

# hf-mirror.com is a CN-friendly mirror of huggingface.co
_VISDRONE_URLS = [
    "https://hf-mirror.com/dronefreak/visdrone-yolov8s/resolve/main/best.pt",
    "https://huggingface.co/dronefreak/visdrone-yolov8s/resolve/main/best.pt",
]
_OSNET_URLS = [
    "https://hf-mirror.com/kaiyangzhou/osnet/resolve/main/osnet_x0_25_msmt17_combineall_256x128_amsgrad_ep150_stp60_lr0.0015_b64_fb10_softmax_labelsmooth_flip_jitter.pth",
    "https://huggingface.co/kaiyangzhou/osnet/resolve/main/osnet_x0_25_msmt17_combineall_256x128_amsgrad_ep150_stp60_lr0.0015_b64_fb10_softmax_labelsmooth_flip_jitter.pth",
]


def _http_download(url: str, dest: Path) -> None:
    from urllib.request import Request, urlopen
    req = Request(url, headers={"User-Agent": "curl/8.0"})
    with urlopen(req, timeout=180) as r, open(dest, "wb") as f:
        shutil.copyfileobj(r, f, length=64 * 1024)


def _dl_with_fallback(target: Path, urls: list[str], label: str) -> Path | None:
    for url in urls:
        try:
            print(f"  try: {url}")
            _http_download(url, target)
            size = target.stat().st_size
            print(f"  OK  {label}: {size / 1024 / 1024:.1f} MB -> {target}")
            return target
        except Exception as e:
            print(f"  fail: {type(e).__name__}: {e}")
            if target.exists():
                target.unlink()
    return None


def ensure_visdrone_weights() -> Path | None:
    target = WEIGHTS / "visdrone_yolov8s.pt"
    if target.exists() and target.stat().st_size > 1_000_000:
        return target
    print("  downloading dronefreak/visdrone-yolov8s ...")
    p = _dl_with_fallback(target, _VISDRONE_URLS, "visdrone")
    if p is None:
        print("  [skip VisDrone] all mirrors failed")
    return p


def ensure_osnet_weights() -> Path | None:
    target = WEIGHTS / "osnet_x0_25_msmt17.pth.tar"
    if target.exists() and target.stat().st_size > 100_000:
        return target
    print("  downloading kaiyangzhou/osnet ...")
    p = _dl_with_fallback(target, _OSNET_URLS, "osnet")
    if p is None:
        print("  [skip ReID] all mirrors failed")
    return p


def run_cvtrack(extra_args: list[str], out_dir: Path, label: str) -> bool:
    out_dir.mkdir(parents=True, exist_ok=True)
    # --out-dir is placed LAST so it overrides any YAML-level output path.
    cmd = [sys.executable, "-m", "cvtrack", *extra_args, "--out-dir", str(out_dir)]
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{SRC}:{env.get('PYTHONPATH', '')}"
    print(f"\n=== {label} ===")
    print("  $", " ".join(cmd))
    rc = subprocess.call(cmd, env=env)
    found_csv = list(out_dir.glob("tracks.csv"))
    found_mp4 = list(out_dir.glob("tracked.mp4"))
    print(f"  -> cvtrack wrote: csv={found_csv} mp4={found_mp4}")
    return rc == 0


def stats_from_csv(path: Path) -> dict | None:
    if not path.exists():
        return None
    rows = [r for r in csv.DictReader(open(path)) if r.get("track_id")]
    if not rows:
        return {"ids": 0, "mean_len": 0.0, "total_rows": 0,
                "median_dets_per_frame": 0, "max_frame": 0}
    ids = {r["track_id"] for r in rows}
    lens = Counter(r["track_id"] for r in rows)
    mean_len = sum(lens.values()) / max(1, len(lens))
    frames = Counter(int(r["frame"]) for r in rows)
    frames_sorted = sorted(frames.keys())
    dets_per_frame = sorted(frames.values())
    median_dets = dets_per_frame[len(dets_per_frame) // 2] if dets_per_frame else 0
    return {
        "ids": len(ids),
        "mean_len": mean_len,
        "total_rows": len(rows),
        "median_dets_per_frame": median_dets,
        "max_frame": frames_sorted[-1] if frames_sorted else 0,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--out-dir", default="weights", help="parent for per-config subdirs")
    ap.add_argument("--skip-download", action="store_true",
                    help="don't try to fetch VisDrone / OSNet weights")
    args = ap.parse_args()

    out_root = (HERE.parent / args.out_dir).resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    coco_s = "/home/hhh/CascadeProjects/multi_sensor_perception_ws/models/yolov8s.pt"

    visdrone_p = None
    osnet_p = None
    if args.skip_download:
        # Reuse the cached weights if present.
        vp = WEIGHTS / "visdrone_yolov8s.pt"
        op = WEIGHTS / "osnet_x0_25_msmt17.pth.tar"
        if vp.exists() and vp.stat().st_size > 1_000_000:
            visdrone_p = vp
        if op.exists() and op.stat().st_size > 100_000:
            osnet_p = op
        print(f"[1/3] --skip-download: visdrone={visdrone_p}  osnet={osnet_p}")
    else:
        print("[1/3] preparing weights")
        visdrone_p = ensure_visdrone_weights()
        osnet_p = ensure_osnet_weights()

    runs: list[tuple[str, list[str]]] = []

    # 2a MOG2 (v4 fallback) -- drone.yaml sets backend=mog2 intentionally.
    runs.append(("MOG2 (v4 fallback)",
                 ["--config", "drone", "--source", args.source,
                  "--max-frames", str(args.frames), "--no-video"]))

    # 2b COCO yolov8s no ReID.
    # NOTE: --drone loads drone.yaml which sets backend=mog2.  We must pass
    # --detector yolo explicitly to override.  Also place --drone LAST so
    # its drone_overrides print the correct weights name.
    runs.append(("COCO yolov8s (no ReID)",
                 ["--source", args.source,
                  "--max-frames", str(args.frames), "--no-video",
                  "--weights", coco_s, "--imgsz", "480", "--conf", "0.12",
                  "--detector", "yolo",
                  "--classes", "0,1,2,3,4,5,7,8",
                  "--min-box-area", "60",
                  "--high-conf", "0.22", "--new-track-conf", "0.07",
                  "--iou-thresh", "0.20", "--lost-relink-frames", "45",
                  "--drone"]))

    if visdrone_p:
        runs.append(("VisDrone yolov8s (no ReID)",
                     ["--source", args.source,
                      "--max-frames", str(args.frames), "--no-video",
                      "--weights", str(visdrone_p), "--imgsz", "960",
                      "--conf", "0.10",
                      "--detector", "yolo",
                      "--min-box-area", "60",
                      "--high-conf", "0.22", "--new-track-conf", "0.07",
                      "--iou-thresh", "0.20", "--lost-relink-frames", "45",
                      "--drone"]))

    if visdrone_p and osnet_p:
        runs.append(("VisDrone yolov8s + ReID(OSNet MSMT17)",
                     ["--source", args.source,
                      "--max-frames", str(args.frames), "--no-video",
                      "--weights", str(visdrone_p), "--imgsz", "960",
                      "--conf", "0.10",
                      "--detector", "yolo",
                      "--min-box-area", "60",
                      "--high-conf", "0.22", "--new-track-conf", "0.07",
                      "--iou-thresh", "0.20", "--lost-relink-frames", "45",
                      "--reid",
                      "--reid-weights", str(osnet_p),
                      "--reid-model", "osnet_x0_25",
                      "--reid-weight", "0.5",
                      "--drone"]))

    print(f"\n[2/3] running {len(runs)} configuration(s)")
    results: list[tuple[str, dict | None]] = []
    for i, (label, argv) in enumerate(runs):
        out_dir = out_root / f"run_{i:02d}"
        run_cvtrack(argv, out_dir, label)
        st = stats_from_csv(out_dir / "tracks.csv")
        results.append((label, st))

    print(f"\n[3/3] writing report -> {out_root / 'COMPARE_REPORT.md'}")
    lines: list[str] = []
    lines.append("# cvtrack head-to-head: drone video, 4 configurations\n")
    lines.append(f"- source: `{args.source}`  frames: {args.frames}")
    lines.append(f"- visdrone weights: `{visdrone_p}`")
    lines.append(f"- osnet weights: `{osnet_p}`\n")
    lines.append("| configuration | IDs | mean_len | total rows | med dets/frame | max_frame |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for label, st in results:
        if st is None:
            lines.append(f"| {label} | (no tracks.csv) | | | | |")
        else:
            lines.append(
                f"| {label} | {st['ids']} | {st['mean_len']:.1f} | {st['total_rows']} | "
                f"{st['median_dets_per_frame']} | {st['max_frame']} |"
            )
    lines.append("")
    if visdrone_p is None:
        lines.append("> VisDrone configuration skipped - weights could not be downloaded.\n")
    if osnet_p is None:
        lines.append("> ReID configuration skipped - OSNet MSMT17 weights could not be downloaded.\n")
    (out_root / "COMPARE_REPORT.md").write_text("\n".join(lines))

    summary = [{"label": l, "stats": s} for l, s in results]
    (out_root / "summary.json").write_text(json.dumps(summary, indent=2))

    print("\n" + "\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
