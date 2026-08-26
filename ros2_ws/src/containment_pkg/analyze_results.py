#!/usr/bin/env python3
"""Aggregate the three-scene enclosure test CSV into a success-rate report.

Reads the CSV written by ``containment_evaluator`` (one row per run) and prints
a per-scene + overall table, then writes a Markdown summary next to the CSV.

8.26 verdict model
------------------
Each run is one of three outcomes:

  * SUCCESS  : target contained AND a platform came within ``intercept_radius``
  * FAIL     : target escaped the monitor ring
  * INVALID  : target contained BUT no platform ever engaged -> excluded from
               the success rate (counted neither as success nor as failure)

The reported containment success rate is therefore computed over *valid* runs
only::

    success_rate = SUCCESS / (SUCCESS + FAIL)

INVALID runs are listed separately so the operator can see how many runs the
gate rejected (e.g. because the platform layer was absent or too far).

Usage:
  python3 analyze_results.py <eval_results.csv> [--markdown]
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
    scenes = defaultdict(lambda: {
        "total": 0, "success": 0, "fail": 0, "invalid": 0,
        "reasons": defaultdict(int),
        "traj": defaultdict(int),
    })
    for r in rows:
        s = r.get("scene", "?")
        d = scenes[s]
        d["total"] += 1
        outcome = str(r.get("outcome", "")).upper()
        if outcome == "SUCCESS":
            d["success"] += 1
        elif outcome == "FAIL":
            d["fail"] += 1
        elif outcome == "INVALID":
            d["invalid"] += 1
        else:
            d["invalid"] += 1  # unknown outcome -> treat as excluded
        d["reasons"][r.get("reason", "?")] += 1
        d["traj"][r.get("trajectory", "?")] += 1
    return scenes


def print_table(scenes):
    tot_s = tot_f = tot_i = 0
    print(f"{'scene':<10} {'runs':>5} {'succ':>5} {'fail':>5} {'inv':>5} "
          f"{'rate':>7}  trajectory mix")
    print("-" * 70)
    for s, d in sorted(scenes.items()):
        valid = d["success"] + d["fail"]
        rate = (d["success"] / valid * 100) if valid else 0.0
        mix = ", ".join(f"{k}={v}" for k, v in sorted(d["traj"].items()))
        print(f"{s:<10} {d['total']:>5} {d['success']:>5} {d['fail']:>5} "
              f"{d['invalid']:>5} {rate:>6.1f}%  {mix}")
        tot_s += d["success"]
        tot_f += d["fail"]
        tot_i += d["invalid"]
    overall_valid = tot_s + tot_f
    overall = (tot_s / overall_valid * 100) if overall_valid else 0.0
    print("-" * 70)
    print(f"{'OVERALL':<10} {tot_s + tot_f + tot_i:>5} {tot_s:>5} {tot_f:>5} "
          f"{tot_i:>5} {overall:>6.1f}%  (valid = success+fail)")
    return tot_s, tot_f, tot_i, overall


def write_markdown(path, scenes, agg):
    tot_s, tot_f, tot_i, overall = agg
    out = os.path.splitext(path)[0] + "_summary.md"
    total = tot_s + tot_f + tot_i
    lines = [
        "# 三场景封控成功率测试统计（8.26 响应证据门槛）",
        "",
        f"- 总测试次数：{total}",
        f"- 有效判定（成功+失败）：{tot_s + tot_f}",
        f"- 无效（INVALID，无响应证据）：{tot_i}",
        f"- **封控成功率（有效样本）：{overall:.1f}%** （指标 6 目标 ≥85%）",
        "",
        "> 说明：INVALID 指目标被回收/守住，但全程无平台进入 intercept_radius"
        "（默认 5 m），即封控无法归因于系统，故单列、不计入成功率分母。",
        "",
        "## 各场景明细",
        "",
        "| 场景 | 次数 | 成功 | 失败 | INVALID | 成功率 | 轨迹分布 |",
        "|------|------|------|------|---------|--------|----------|",
    ]
    for s, d in sorted(scenes.items()):
        valid = d["success"] + d["fail"]
        rate = (d["success"] / valid * 100) if valid else 0.0
        mix = ", ".join(f"{k}={v}" for k, v in sorted(d["traj"].items()))
        lines.append(
            f"| {s} | {d['total']} | {d['success']} | {d['fail']} | "
            f"{d['invalid']} | {rate:.1f}% | {mix} |"
        )
    lines += [
        "",
        "## 失败 / 无效原因分布",
        "",
        "| 场景 | 结果 | 原因 | 次数 |",
        "|------|------|------|------|",
    ]
    for s, d in sorted(scenes.items()):
        for reason, n in sorted(d["reasons"].items()):
            lines.append(f"| {s} | {reason} | {reason} | {n} |")
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
    agg = print_table(scenes)
    if args.markdown:
        write_markdown(args.csv, scenes, agg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
