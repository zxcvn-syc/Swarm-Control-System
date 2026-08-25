#!/usr/bin/env python3
"""Aggregate the three-scene enclosure test CSV into a success-rate report.

Reads the CSV written by ``containment_evaluator`` (one row per run) and prints
a per-scene + overall success-rate table, then writes a Markdown summary next
to the CSV.

Usage:
  python3 analyze_results.py <eval_results.csv> [--markdown]

The CSV columns (from containment_evaluator) are:
  timestamp, scene, direction, trajectory, outcome, reason,
  duration_s, max_r, min_r, num_responded
"""
import argparse
import csv
import os
import sys
from collections import defaultdict


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def summarize(rows):
    scenes = defaultdict(lambda: {"total": 0, "success": 0,
                                  "reasons": defaultdict(int),
                                  "traj": defaultdict(int)})
    for r in rows:
        s = r.get("scene", "?")
        d = scenes[s]
        d["total"] += 1
        if str(r.get("outcome", "")).upper() == "SUCCESS":
            d["success"] += 1
        d["reasons"][r.get("reason", "?")] += 1
        d["traj"][r.get("trajectory", "?")] += 1
    return scenes


def print_table(scenes):
    tot = sum = 0
    print(f"{'scene':<10} {'runs':>5} {'success':>8} {'rate':>7}  trajectory mix")
    print("-" * 64)
    for s, d in sorted(scenes.items()):
        rate = (d["success"] / d["total"] * 100) if d["total"] else 0.0
        mix = ", ".join(f"{k}={v}" for k, v in sorted(d["traj"].items()))
        print(f"{s:<10} {d['total']:>5} {d['success']:>8} {rate:>6.1f}%  {mix}")
        tot += d["total"]
        sum += d["success"]
    overall = (sum / tot * 100) if tot else 0.0
    print("-" * 64)
    print(f"{'OVERALL':<10} {tot:>5} {sum:>8} {overall:>6.1f}%")
    return overall


def write_markdown(path, scenes, overall):
    out = os.path.splitext(path)[0] + "_summary.md"
    lines = ["# 三场景封控成功率测试统计", "",
             f"- 总测试次数：{sum(d['total'] for d in scenes.values())}",
             f"- 综合成功率：**{overall:.1f}%** （目标 ≥85%）", ""]
    lines.append("| 场景 | 次数 | 成功 | 成功率 | 轨迹分布 |")
    lines.append("|------|------|------|--------|----------|")
    for s, d in sorted(scenes.items()):
        rate = (d["success"] / d["total"] * 100) if d["total"] else 0.0
        mix = ", ".join(f"{k}={v}" for k, v in sorted(d["traj"].items()))
        lines.append(f"| {s} | {d['total']} | {d['success']} | {rate:.1f}% | {mix} |")
    lines.append("")
    lines.append("## 失败原因分布")
    lines.append("")
    lines.append("| 场景 | 原因 | 次数 |")
    lines.append("|------|------|------|")
    for s, d in sorted(scenes.items()):
        for reason, n in sorted(d["reasons"].items()):
            lines.append(f"| {s} | {reason} | {n} |")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[written] {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--markdown", action="store_true",
                    help="also write a Markdown summary next to the CSV")
    args = ap.parse_args()
    if not os.path.exists(args.csv):
        print(f"CSV not found: {args.csv}", file=sys.stderr)
        return 2
    rows = load_rows(args.csv)
    if not rows:
        print("CSV is empty.", file=sys.stderr)
        return 1
    scenes = summarize(rows)
    overall = print_table(scenes)
    if args.markdown:
        write_markdown(args.csv, scenes, overall)
    return 0


if __name__ == "__main__":
    sys.exit(main())
