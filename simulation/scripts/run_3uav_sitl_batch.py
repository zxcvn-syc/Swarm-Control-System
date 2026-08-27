#!/usr/bin/env python3
"""Run bounded three-UAV PX4/Gazebo SITL process-stability trials.

The launcher deliberately remains below MAVROS and flight control: this tool
only proves that Gazebo Classic and three PX4 processes start and stay alive.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


SIMULATION_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_DIR = SIMULATION_DIR.parent
DEFAULT_LAUNCHER = SIMULATION_DIR / "px4_sitl_3uav" / "start_3uav_sitl.sh"


def utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="seconds")


def positive_integer(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def nonnegative_integer(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_revision(repository_dir: Path) -> str | None:
    result = subprocess.run(
        ["git", "-C", str(repository_dir), "rev-parse", "HEAD"],
        capture_output=True,
        check=False,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def build_launcher_command(
    launcher: Path,
    px4_sitl_root: Path,
    world: str,
    duration_seconds: int,
    startup_timeout_seconds: int,
    output_dir: Path,
    run_id: str,
    cleanup_leftovers: bool,
) -> list[str]:
    """Build the launcher invocation without running a simulator."""
    launcher_path = launcher.as_posix()
    px4_root_path = px4_sitl_root.as_posix()
    output_path = output_dir.as_posix()
    command = [
        "bash",
        launcher_path,
        "--px4-sitl-root",
        px4_root_path,
        "--world",
        world,
        "--duration",
        str(duration_seconds),
        "--startup-timeout",
        str(startup_timeout_seconds),
        "--output-dir",
        output_path,
        "--run-id",
        run_id,
    ]
    if cleanup_leftovers:
        command.append("--cleanup-leftovers")
    return command


def load_result(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def run_attempt(command: list[str], attempt_dir: Path) -> dict[str, Any]:
    attempt_dir.mkdir(parents=True, exist_ok=True)
    launcher_log = attempt_dir / "launcher.log"
    started_at = utc_now()
    with launcher_log.open("w", encoding="utf-8") as log_file:
        completed = subprocess.run(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            check=False,
            text=True,
        )
    result = load_result(attempt_dir / "result.json")
    passed = completed.returncode == 0 and result is not None and result.get("status") == "passed"
    return {
        "attempt": attempt_dir.name,
        "started_at": started_at,
        "finished_at": utc_now(),
        "command": command,
        "return_code": completed.returncode,
        "launcher_log": str(launcher_log),
        "result_file": str(attempt_dir / "result.json"),
        "launcher_result": result,
        "passed": passed,
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(
        description="Run reproducible three-UAV PX4/Gazebo SITL process-stability trials."
    )
    parser.add_argument("--runs", type=positive_integer, default=20)
    parser.add_argument("--duration", type=positive_integer, default=60)
    parser.add_argument("--startup-timeout", type=positive_integer, default=60)
    parser.add_argument("--retries", type=nonnegative_integer, default=0)
    parser.add_argument("--world", default="empty")
    parser.add_argument(
        "--px4-sitl-root",
        type=Path,
        default=Path.home() / "src" / "PX4-Autopilot",
    )
    parser.add_argument("--launcher", type=Path, default=DEFAULT_LAUNCHER)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=SIMULATION_DIR / "results" / f"three_uav_sitl_batch_{timestamp}",
    )
    parser.add_argument(
        "--cleanup-leftovers",
        action="store_true",
        help="Pass the launcher's explicit old PX4/Gazebo cleanup option for every attempt.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    launcher = args.launcher.resolve()
    if not launcher.is_file():
        print(f"error: launcher does not exist: {launcher}", file=sys.stderr)
        return 2

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "created_at": utc_now(),
        "runs": args.runs,
        "duration_seconds": args.duration,
        "startup_timeout_seconds": args.startup_timeout,
        "retries": args.retries,
        "world": args.world,
        "px4_sitl_root": str(args.px4_sitl_root.resolve()),
        "launcher": str(launcher),
        "launcher_sha256": sha256(launcher),
        "source_revision": source_revision(REPOSITORY_DIR),
        "scope": "PX4/Gazebo SITL process stability only; MAVROS/arming/mode/setpoints excluded",
        "trials": [],
    }
    (output_dir / "batch_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )

    for trial_number in range(1, args.runs + 1):
        trial_dir = output_dir / f"trial_{trial_number:02d}"
        trial_record: dict[str, Any] = {"trial": trial_number, "attempts": [], "passed": False}
        for attempt_number in range(1, args.retries + 2):
            attempt_dir = trial_dir / f"attempt_{attempt_number:02d}"
            command = build_launcher_command(
                launcher=launcher,
                px4_sitl_root=args.px4_sitl_root.resolve(),
                world=args.world,
                duration_seconds=args.duration,
                startup_timeout_seconds=args.startup_timeout,
                output_dir=attempt_dir,
                run_id=f"trial-{trial_number:02d}-attempt-{attempt_number:02d}",
                cleanup_leftovers=args.cleanup_leftovers,
            )
            print(f"[batch] trial {trial_number:02d} attempt {attempt_number:02d}/{args.retries + 1}")
            attempt_record = run_attempt(command, attempt_dir)
            trial_record["attempts"].append(attempt_record)
            if attempt_record["passed"]:
                trial_record["passed"] = True
                break
        manifest["trials"].append(trial_record)
        (output_dir / "batch_manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )

    success_count = sum(bool(trial["passed"]) for trial in manifest["trials"])
    summary = {
        "created_at": utc_now(),
        "runs": args.runs,
        "duration_seconds": args.duration,
        "retries": args.retries,
        "success_count": success_count,
        "success_rate": success_count / args.runs,
        "passed": success_count == args.runs,
        "failed_trials": [
            trial["trial"] for trial in manifest["trials"] if not trial["passed"]
        ],
        "manifest": str(output_dir / "batch_manifest.json"),
        "scope": manifest["scope"],
    }
    (output_dir / "batch_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
