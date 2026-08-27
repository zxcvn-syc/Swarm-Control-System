from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "px4_control_gate.py"
SPEC = importlib.util.spec_from_file_location("px4_control_gate", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
px4_control_gate = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = px4_control_gate
SPEC.loader.exec_module(px4_control_gate)


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data), encoding="utf-8")


def make_evidence_dir(path: Path) -> Path:
    path.mkdir()
    write_json(path / "validation.json", {"passed": True})
    write_json(path / "capture_summary.json", {"pending": [], "received": {"x": 1}})
    write_json(
        path / "evidence_manifest.json",
        {
            "topics": {
                "target_track_world": {"has_payload_evidence": True},
                "enclosure_command": {"has_payload_evidence": True},
            }
        },
    )
    return path


def test_control_gate_requires_explicit_intent(tmp_path: Path, monkeypatch) -> None:
    evidence = make_evidence_dir(tmp_path / "evidence")
    calibration = tmp_path / "calibration.json"
    mavros = tmp_path / "mavros.json"
    approval = tmp_path / "approval.json"
    write_json(
        calibration,
        {
            "frame_id": "world",
            "units": "m",
            "points": [[0, 0], [1, 0], [1, 1], [0, 1]],
        },
    )
    write_json(mavros, {"connected": True, "armed": False, "mode": "POSCTL", "age_s": 0.2})
    write_json(
        approval,
        {
            "mission_id": "field-001",
            "approved_by": "operator",
            "safety_pilot_present": True,
            "kill_switch_tested": True,
            "geofence_checked": True,
            "battery_checked": True,
            "propeller_area_clear": True,
            "manual_takeover_ready": True,
        },
    )
    monkeypatch.delenv(px4_control_gate.CONTROL_ENV_NAME, raising=False)

    checks = [
        px4_control_gate.check_evidence_dir(evidence),
        px4_control_gate.check_world_calibration(calibration),
        px4_control_gate.check_mavros_state(mavros, 2.0, False),
        px4_control_gate.check_operator_approval(approval),
        px4_control_gate.check_control_intent(True),
    ]

    assert [check.passed for check in checks] == [True, True, True, True, False]


def test_control_gate_accepts_complete_audit_inputs(tmp_path: Path, monkeypatch) -> None:
    evidence = make_evidence_dir(tmp_path / "evidence")
    calibration = tmp_path / "calibration.json"
    mavros = tmp_path / "mavros.json"
    approval = tmp_path / "approval.json"
    write_json(
        calibration,
        {
            "world_frame": "map",
            "unit": "meters",
            "correspondences": [[0, 0], [1, 0], [1, 1], [0, 1]],
        },
    )
    write_json(mavros, {"connected": True, "armed": False, "mode": "POSCTL", "age_s": 0.1})
    write_json(
        approval,
        {
            "mission_id": "field-002",
            "approved_by": "operator",
            "safety_pilot_present": True,
            "kill_switch_tested": True,
            "geofence_checked": True,
            "battery_checked": True,
            "propeller_area_clear": True,
            "manual_takeover_ready": True,
        },
    )
    monkeypatch.setenv(
        px4_control_gate.CONTROL_ENV_NAME,
        px4_control_gate.CONTROL_ENV_VALUE,
    )

    checks = [
        px4_control_gate.check_evidence_dir(evidence),
        px4_control_gate.check_world_calibration(calibration),
        px4_control_gate.check_mavros_state(mavros, 2.0, False),
        px4_control_gate.check_operator_approval(approval),
        px4_control_gate.check_control_intent(True),
    ]

    assert all(check.passed for check in checks)
