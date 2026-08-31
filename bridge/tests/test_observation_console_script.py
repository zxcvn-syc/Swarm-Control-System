from pathlib import Path
import shutil
import subprocess

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "start_ros2_observation_console.sh"


def test_observation_console_has_valid_bash_syntax() -> None:
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash is not available")

    subprocess.run([bash, "-n"], input=SCRIPT.read_bytes(), check=True)


def test_observation_console_starts_and_cleans_up_real_perception() -> None:
    script = SCRIPT.read_text(encoding="utf-8")

    assert ': "${ENABLE_PERCEPTION:=true}"' in script
    assert "ros2 run perception_pkg tracker_node" in script
    assert 'kill -INT "$TRACKER_PID"' in script
    assert '-p perception_topic:="$PERCEPTION_TOPIC"' in script
    assert "-p enable_pilot_commands:=false" in script
    assert script.index("ros2 run perception_pkg tracker_node") < script.index(
        "ros2 run planning_pkg flight_safety_dashboard"
    )
