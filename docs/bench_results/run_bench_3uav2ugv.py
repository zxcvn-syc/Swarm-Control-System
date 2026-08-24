# -*- coding: utf-8 -*-
"""3UAV+2UGV 配置下三模式任务分配对比基准（对应冲刺计划 8.24 任务）。

复现方法::

    python3 docs/bench_results/run_bench_3uav2ugv.py

直接复用 scheduler_pkg/bench_auction.py 的官方场景生成与计时函数
（_make_agents/_make_tasks/_time_greedy/_time_hungarian/_time_auction），
保证口径与 bench_auction 完全一致。
口径：5 平台（3UAV+2UGV）× 10 任务 × 10 随机场景（seed 2026-2035）× 每场景重复 5 次取中位数。
"""
import csv
import logging
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "ros2_ws" / "src" / "scheduler_pkg"))

from scheduler_pkg.bench_auction import (  # noqa: E402
    _make_agents,
    _make_tasks,
    _time_greedy,
    _time_hungarian,
    _time_auction,
)

logging.getLogger("AuctionDemo").setLevel(logging.WARNING)

N_AGENTS, N_TASKS, SCENARIOS, REPEATS = 5, 10, 10, 5
STRATEGIES = (("greedy", _time_greedy), ("hungarian", _time_hungarian), ("auction", _time_auction))

rows = []
for s in range(SCENARIOS):
    seed = 2026 + s
    # 校验场景确为 3UAV+2UGV（官方 _make_agents 交替生成，i%2==0 为 UAV）
    import random as _random
    rng = _random.Random(seed)
    agents = _make_agents(N_AGENTS, rng)
    uav = sum(1 for a in agents if a.category == "UAV")
    assert uav == 3 and len(agents) - uav == 2, f"seed {seed}: {uav}UAV+{len(agents)-uav}UGV"
    tasks = _make_tasks(N_TASKS, _random.Random(seed + 1000))
    rec = {"seed": seed, "platforms": f"{uav}UAV+{len(agents)-uav}UGV", "tasks": len(tasks)}
    for name, fn in STRATEGIES:
        samples = [fn(N_AGENTS, N_TASKS, seed) for _ in range(REPEATS)]
        rec[name + "_median_ms"] = round(statistics.median(samples), 4)
        rec[name + "_all_ms"] = ";".join(f"{x:.3f}" for x in samples)
    rows.append(rec)

summary = {}
for name, _ in STRATEGIES:
    m = [r[name + "_median_ms"] for r in rows]
    x = [float(v) for r in rows for v in r[name + "_all_ms"].split(";")]
    summary[name] = {"median": round(statistics.median(m), 4),
                     "mean": round(statistics.mean(x), 4),
                     "worst": round(max(x), 4)}

out_dir = HERE

with open(out_dir / "bench_3uav2ugv_three_mode.csv", "w", newline="", encoding="utf-8-sig") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

lines = [
    "# 3UAV+2UGV 配置三模式任务分配耗时对比",
    "",
    f"口径: {N_AGENTS} 平台（3UAV+2UGV）× {N_TASKS} 随机任务 × {SCENARIOS} 个随机场景 × 每场景重复 {REPEATS} 次取中位数",
    f"seed: 2026-2035 | Python {sys.version.split()[0]} | 生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}",
    "场景生成与计时函数直接调用 scheduler_pkg/bench_auction.py 官方实现（_time_greedy/_time_hungarian/_time_auction）",
    "复现: python3 docs/bench_results/run_bench_3uav2ugv.py",
    "",
    "## 各场景中位数（ms）",
    "",
    "| seed | 平台 | 任务数 | greedy | hungarian | auction |",
    "|---|---|---|---|---|---|",
]
for r in rows:
    lines.append(f"| {r['seed']} | {r['platforms']} | {r['tasks']} | "
                 f"{r['greedy_median_ms']:.3f} | {r['hungarian_median_ms']:.3f} | {r['auction_median_ms']:.3f} |")
lines += [
    "",
    "## 三模式汇总（跨全部 50 个样本/模式）",
    "",
    "| 模式 | 中位数 (ms) | 均值 (ms) | 最差样本 (ms) |",
    "|---|---|---|---|",
]
for name, _ in STRATEGIES:
    v = summary[name]
    lines.append(f"| {name} | {v['median']:.3f} | {v['mean']:.3f} | {v['worst']:.3f} |")
lines += [
    "",
    "结论: 三模式在 3UAV+2UGV × 10 任务规模下均为毫秒级，远低于指标 2（全任务分配平均耗时 ≤3s）要求。",
    "",
    "> 注: 与既有 bench_auction_result.md（8/16/32 agents × 10/20/50 tasks 规模扫描，seed 42）口径互补，"
    "本表针对系统实际部署配置（3机2车）。",
]
(out_dir / "bench_3uav2ugv_three_mode.md").write_text("\n".join(lines), encoding="utf-8")

print("=== 三模式汇总（3UAV+2UGV × 10任务 × 10场景 × 5次，官方口径）===")
for name, _ in STRATEGIES:
    v = summary[name]
    print(f"{name:10s} median={v['median']:.3f}ms  mean={v['mean']:.3f}ms  worst={v['worst']:.3f}ms")
print("\n输出:", out_dir / "bench_3uav2ugv_three_mode.md")
