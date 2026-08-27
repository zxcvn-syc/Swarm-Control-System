import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "run_3uav_sitl_batch.py"
SPEC = importlib.util.spec_from_file_location("three_uav_sitl_batch", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_build_launcher_command_has_bounded_evidence_arguments() -> None:
    command = MODULE.build_launcher_command(
        launcher=Path("/workspace/simulation/px4_sitl_3uav/start_3uav_sitl.sh"),
        px4_sitl_root=Path("/opt/PX4-Autopilot"),
        world="swarm_field",
        duration_seconds=60,
        startup_timeout_seconds=45,
        output_dir=Path("/tmp/trial_01/attempt_01"),
        run_id="trial-01-attempt-01",
        cleanup_leftovers=False,
    )

    assert command == [
        "bash",
        "/workspace/simulation/px4_sitl_3uav/start_3uav_sitl.sh",
        "--px4-sitl-root",
        "/opt/PX4-Autopilot",
        "--world",
        "swarm_field",
        "--duration",
        "60",
        "--startup-timeout",
        "45",
        "--output-dir",
        "/tmp/trial_01/attempt_01",
        "--run-id",
        "trial-01-attempt-01",
    ]


def test_build_launcher_command_requires_explicit_leftover_cleanup() -> None:
    command = MODULE.build_launcher_command(
        launcher=Path("/launcher.sh"),
        px4_sitl_root=Path("/px4"),
        world="empty",
        duration_seconds=1,
        startup_timeout_seconds=1,
        output_dir=Path("/tmp/run"),
        run_id="run",
        cleanup_leftovers=True,
    )

    assert command[-1] == "--cleanup-leftovers"
