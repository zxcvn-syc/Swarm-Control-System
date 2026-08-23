"""Auction engine timing benchmark (P1-G).

Standalone script — does not depend on ROS2.  Imports the scheduler_pkg
modules directly via ``sys.path`` so the benchmark can be reproduced on
any developer machine.

What it measures
----------------
For each (agents, tasks) cell in a 3x3 grid we build a fresh batch of
random agents/tasks, run all three assignment strategies (greedy /
hungarian / auction) ``--repeats`` times each, and report the median
wall-clock cost.  Output is written as both Markdown and CSV to
``output/bench_auction_result.{md,csv}``.

Usage
-----
    python3 bench_auction.py                # full 3x3 sweep
    python3 bench_auction.py --quick        # one cell only (smoke test)
    python3 bench_auction.py --seed 7       # override RNG seed
    python3 bench_auction.py --out-dir /tmp # change output location
"""

from __future__ import annotations

import argparse
import csv
import os
import platform
import random
import statistics
import sys
import time
from pathlib import Path
from typing import List, Tuple

# ---------------------------------------------------------------------------
# Locate the scheduler_pkg source tree and import its modules by file path.
# This keeps the benchmark decoupled from any ROS2 install / ament overlay.
# ---------------------------------------------------------------------------
_HERE = Path(__file__).resolve().parent
_SCHEDULER_PKG_DIR = _HERE  # this script lives next to agent.py etc.
# Make the script work whether invoked as `python3 bench_auction.py` from any
# cwd, or via `python3 -m scheduler_pkg.bench_auction`.
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))  # lets us `import scheduler_pkg.*`

# pylint: disable=wrong-import-position
from scheduler_pkg.agent import Agent  # noqa: E402
from scheduler_pkg.task import Task  # noqa: E402
from scheduler_pkg.auction_engine import AuctionEngine  # noqa: E402
from scheduler_pkg import assign as assign_mod  # noqa: E402
import logging  # noqa: E402  (used to silence noisy library loggers)

# AuctionEngine.generate_utility_matrix() prints the whole N x M matrix via
# logger.info(...) which is real wall-clock I/O that would skew the timing
# measurement.  Silence it for the duration of the benchmark.
logging.getLogger("AuctionDemo").setLevel(logging.WARNING)


# ---------------------------------------------------------------------------
# Scenario generation
# ---------------------------------------------------------------------------
def _make_agents(n: int, rng: random.Random) -> List[Agent]:
    """Build *n* heterogeneous agents (mix of UAV / UGV) on a 50x50 grid."""
    agents: List[Agent] = []
    for i in range(n):
        # Alternate UAV/UGV so the auction engine exercises both branches.
        category = "UAV" if i % 2 == 0 else "UGV"
        if category == "UAV":
            battery, max_load, unit_cost, speed = 100.0, 3, 1.0, 2.0
        else:
            battery, max_load, unit_cost, speed = 80.0, 4, 0.6, 1.0
        pos = [rng.uniform(0.0, 50.0), rng.uniform(0.0, 50.0)]
        agents.append(Agent(f"A{i:02d}", category, pos, battery,
                            max_load, unit_cost, speed))
    return agents


def _make_tasks(m: int, rng: random.Random) -> List[Task]:
    """Build *m* tasks on the same 50x50 grid with generous time windows."""
    tasks: List[Task] = []
    for j in range(m):
        pos = [rng.uniform(0.0, 50.0), rng.uniform(0.0, 50.0)]
        reward = rng.uniform(10.0, 100.0)
        priority = rng.randint(1, 5)
        release_time = rng.uniform(0.0, 5.0)
        # Generous deadlines so every task is feasible for some agent —
        # we want to time the algorithm, not measure feasibility filtering.
        deadline = release_time + 1000.0
        service_time = 10.0
        tasks.append(Task(f"T{j:03d}", pos, reward, priority,
                          release_time, deadline, service_time))
    return tasks


def _fresh_scenario(n_agents: int, n_tasks: int, seed: int
                    ) -> Tuple[List[Agent], List[Task], List[List[float]],
                               List[List[float]]]:
    """Create an independent scenario and the numpy views used by assign.py.

    Returns
    -------
    agents, tasks, drones_xy, targets_xy
    The numpy arrays are needed by ``assign.greedy_assign`` /
    ``assign.hungarian_assign`` which take plain (N,2) / (M,2) matrices.
    """
    rng = random.Random(seed)
    agents = _make_agents(n_agents, rng)
    tasks = _make_tasks(n_tasks, rng)
    drones_xy = [[a.pos[0], a.pos[1]] for a in agents]
    targets_xy = [[t.pos[0], t.pos[1]] for t in tasks]
    return agents, tasks, drones_xy, targets_xy


# ---------------------------------------------------------------------------
# Timed runners — one per strategy.  Each builds a fresh scenario so we
# measure the algorithm's cost in isolation (no warm caches / dirty state).
# ---------------------------------------------------------------------------
def _time_greedy(n_agents: int, n_tasks: int, seed: int) -> float:
    _, _, drones, targets = _fresh_scenario(n_agents, n_tasks, seed)
    t0 = time.perf_counter()
    assign_mod.greedy_assign(drones, targets, max_per_drone=4)
    return (time.perf_counter() - t0) * 1000.0  # ms


def _time_hungarian(n_agents: int, n_tasks: int, seed: int) -> float:
    _, _, drones, targets = _fresh_scenario(n_agents, n_tasks, seed)
    t0 = time.perf_counter()
    assign_mod.hungarian_assign(drones, targets, max_per_drone=1)
    return (time.perf_counter() - t0) * 1000.0


def _time_auction(n_agents: int, n_tasks: int, seed: int) -> float:
    agents, tasks, _, _ = _fresh_scenario(n_agents, n_tasks, seed)
    t0 = time.perf_counter()
    engine = AuctionEngine(agents, tasks)
    engine.generate_utility_matrix()
    engine.bid_allocation()
    return (time.perf_counter() - t0) * 1000.0


_STRATEGIES = (
    ("greedy", _time_greedy),
    ("hungarian", _time_hungarian),
    ("auction", _time_auction),
)


def _median(samples: List[float]) -> float:
    return statistics.median(samples) if samples else float("nan")


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def _format_ms(v: float) -> str:
    if v != v:  # NaN check
        return "  -  "
    return f"{v:5.2f} ms"


def _render_markdown(grid_agents, grid_tasks, results, meta) -> str:
    lines: List[str] = []
    lines.append("# Auction 引擎耗时基准 (agents × tasks)")
    lines.append("")
    lines.append(
        f"基线: {meta['baseline']} | seed: {meta['seed']} | "
        f"平台: {meta['platform']} | 时间: {meta['timestamp']}"
    )
    lines.append("")
    lines.append(
        f"重复次数: {meta['repeats']} 次取中位数 | "
        f"Python: {meta['python']}"
    )
    lines.append("")

    header_tasks = " | ".join(f"{t}" for t in grid_tasks)
    for strategy, _ in _STRATEGIES:
        title = {"greedy": "Greedy Assign",
                 "hungarian": "Hungarian Assign",
                 "auction": "Auction Engine"}[strategy]
        lines.append(f"## {title}")
        lines.append("")
        lines.append(f"| Agents \\ Tasks | {header_tasks} |")
        lines.append("|" + "---|" * (len(grid_tasks) + 1))
        for n_a in grid_agents:
            row_vals = " | ".join(
                _format_ms(results[strategy][n_a][n_t])
                for n_t in grid_tasks
            )
            lines.append(f"| {n_a} | {row_vals} |")
        lines.append("")
    return "\n".join(lines)


def _render_csv(grid_agents, grid_tasks, results) -> List[List[str]]:
    """Build a flat CSV: one row per (strategy, agents, tasks) triple."""
    rows: List[List[str]] = [["strategy", "agents", "tasks", "median_ms"]]
    for strategy, _ in _STRATEGIES:
        for n_a in grid_agents:
            for n_t in grid_tasks:
                v = results[strategy][n_a][n_t]
                rows.append([strategy, str(n_a), str(n_t),
                             "" if v != v else f"{v:.4f}"])
    return rows


def _write_outputs(out_dir: Path, md_text: str, csv_rows: List[List[str]]
                   ) -> Tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    md_path = out_dir / "bench_auction_result.md"
    csv_path = out_dir / "bench_auction_result.csv"
    md_path.write_text(md_text, encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(csv_rows)
    return md_path, csv_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _baseline_commit() -> str:
    """Best-effort lookup of HEAD commit short-hash; falls back to 'unknown'."""
    try:
        import subprocess
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(_HERE.parents[3]),  # .../scheduler_pkg -> repo root
            stderr=subprocess.DEVNULL,
        )
        return out.decode().strip() or "unknown"
    except Exception:
        return "unknown"


def _configure_utf8_stdout() -> None:
    """Keep the Chinese Markdown report printable on Windows terminals."""
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


def main(argv=None) -> int:
    _configure_utf8_stdout()
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--quick", action="store_true",
                        help="Run a single (16 agents, 20 tasks) cell only.")
    parser.add_argument("--seed", type=int, default=42,
                        help="Base RNG seed (cell-specific seeds are derived).")
    parser.add_argument("--repeats", type=int, default=5,
                        help="Number of repeats per cell (median).")
    parser.add_argument("--out-dir", type=Path, default=None,
                        help="Output directory (default: <repo>/output).")
    args = parser.parse_args(argv)

    grid_agents = [8, 16, 32]
    grid_tasks = [10, 20, 50]
    if args.quick:
        grid_agents = [16]
        grid_tasks = [20]

    out_dir = args.out_dir or _HERE.parents[3] / "output"
    # If the auto-detected repo root isn't writable fall back to CWD.
    try:
        out_dir.mkdir(parents=True, exist_ok=True)
    except OSError:
        out_dir = Path.cwd() / "output"
        out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[bench] sweep agents={grid_agents} tasks={grid_tasks} "
          f"repeats={args.repeats} seed={args.seed}")
    print(f"[bench] output dir: {out_dir}")

    results = {
        name: {n_a: {n_t: float("nan") for n_t in grid_tasks}
               for n_a in grid_agents}
        for name, _ in _STRATEGIES
    }

    for n_a in grid_agents:
        for n_t in grid_tasks:
            for strategy, runner in _STRATEGIES:
                samples: List[float] = []
                for r in range(args.repeats):
                    # Per-cell, per-repeat seed = base * 1000003 + offsets
                    seed = (args.seed * 1_000_003
                            + n_a * 10_007
                            + n_t * 101
                            + r * 31
                            + hash(strategy) % 9973) & 0x7FFFFFFF
                    try:
                        samples.append(runner(n_a, n_t, seed))
                    except Exception as exc:  # pragma: no cover - defensive
                        print(f"[bench] {strategy} {n_a}x{n_t} rep{r} "
                              f"FAILED: {exc!r}")
                med = _median(samples)
                results[strategy][n_a][n_t] = med
                if samples:
                    print(f"[bench] {strategy:9s} | {n_a:>2} agents x {n_t:>2} "
                          f"tasks | median = {med:7.3f} ms "
                          f"(min {min(samples):.3f} / max {max(samples):.3f})")
                else:
                    print(f"[bench] {strategy:9s} | {n_a:>2} agents x {n_t:>2} "
                          "tasks | no successful samples")

    meta = {
        "baseline": _baseline_commit(),
        "seed": args.seed,
        "platform": platform.processor() or platform.machine() or "unknown",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "repeats": args.repeats,
        "python": platform.python_version(),
    }
    md_text = _render_markdown(grid_agents, grid_tasks, results, meta)
    csv_rows = _render_csv(grid_agents, grid_tasks, results)
    md_path, csv_path = _write_outputs(out_dir, md_text, csv_rows)

    print()
    print(md_text)
    print()
    print(f"[bench] wrote {md_path}")
    print(f"[bench] wrote {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
