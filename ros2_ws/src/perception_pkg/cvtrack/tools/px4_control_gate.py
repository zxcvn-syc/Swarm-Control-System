#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML is a project dependency.
    yaml = None


CONTROL_ENV_NAME = "CVTRACK_PX4_CONTROL_ALLOWED"
CONTROL_ENV_VALUE = "YES_I_ACCEPT_REAL_VEHICLE_RISK"


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    detail: str


def load_structured(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        if yaml is None:
            raise RuntimeError("PyYAML is required to read YAML gate inputs")
        data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping/object")
    return data


def truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y", "on"}
    return False


def check_evidence_dir(path: Path | None) -> Check:
    if path is None:
        return Check("closed_loop_evidence_dir", False, "missing --evidence-dir")
    if not path.exists():
        return Check("closed_loop_evidence_dir", False, f"{path} does not exist")
    required_files = ["validation.json", "capture_summary.json", "evidence_manifest.json"]
    missing = [name for name in required_files if not (path / name).exists()]
    if missing:
        return Check("closed_loop_evidence_dir", False, f"missing {', '.join(missing)}")
    try:
        validation = load_structured(path / "validation.json")
        summary = load_structured(path / "capture_summary.json")
        manifest = load_structured(path / "evidence_manifest.json")
    except Exception as exc:
        return Check("closed_loop_evidence_dir", False, f"could not parse evidence: {exc}")
    if validation.get("passed") is not True:
        return Check("closed_loop_evidence_dir", False, "validation.json did not pass")
    pending = summary.get("pending", [])
    if pending:
        return Check("closed_loop_evidence_dir", False, f"pending ROS evidence: {pending}")
    topics = manifest.get("topics", {})
    if not isinstance(topics, dict) or not topics:
        return Check("closed_loop_evidence_dir", False, "evidence_manifest.json has no topics")
    missing_payload = [
        name for name, topic in topics.items()
        if not isinstance(topic, dict) or not truthy(topic.get("has_payload_evidence"))
    ]
    if missing_payload:
        return Check(
            "closed_loop_evidence_dir",
            False,
            f"topics without first payload evidence: {', '.join(missing_payload)}",
        )
    return Check("closed_loop_evidence_dir", True, f"validated evidence in {path}")


def calibration_point_count(data: dict[str, Any]) -> int:
    for key in ("points", "correspondences", "ground_points", "image_points"):
        value = data.get(key)
        if isinstance(value, list):
            return len(value)
    if isinstance(data.get("image_to_world_homography"), list):
        return 4
    return 0


def check_world_calibration(path: Path | None) -> Check:
    if path is None:
        return Check("world_calibration", False, "missing --world-calibration")
    if not path.exists():
        return Check("world_calibration", False, f"{path} does not exist")
    try:
        data = load_structured(path)
    except Exception as exc:
        return Check("world_calibration", False, f"could not parse calibration: {exc}")
    frame_id = str(data.get("frame_id", data.get("world_frame", ""))).strip()
    units = str(data.get("units", data.get("unit", ""))).strip().lower()
    points = calibration_point_count(data)
    if frame_id not in {"world", "map", "local_origin"}:
        return Check("world_calibration", False, "frame_id/world_frame must be world/map/local_origin")
    if units not in {"m", "meter", "meters"}:
        return Check("world_calibration", False, "units must be meters")
    if points < 4:
        return Check("world_calibration", False, "need at least four ground correspondences")
    return Check("world_calibration", True, f"{points} calibration correspondences in {frame_id}")


def check_mavros_state(path: Path | None, max_age: float, allow_armed_state: bool) -> Check:
    if path is None:
        return Check("fresh_mavros_state", False, "missing --mavros-state-file")
    if not path.exists():
        return Check("fresh_mavros_state", False, f"{path} does not exist")
    try:
        data = load_structured(path)
    except Exception as exc:
        return Check("fresh_mavros_state", False, f"could not parse MAVROS state: {exc}")
    connected = truthy(data.get("connected"))
    armed = truthy(data.get("armed"))
    mode = str(data.get("mode", "")).strip()
    age = data.get("age_s", data.get("observed_age_s", data.get("freshness_s")))
    try:
        age_s = float(age)
    except (TypeError, ValueError):
        return Check("fresh_mavros_state", False, "state must include age_s/observed_age_s")
    if not connected:
        return Check("fresh_mavros_state", False, "MAVROS is not connected")
    if age_s > max_age:
        return Check("fresh_mavros_state", False, f"MAVROS state age {age_s:.3f}s > {max_age:.3f}s")
    if armed and not allow_armed_state:
        return Check("fresh_mavros_state", False, "vehicle is already armed")
    if not mode:
        return Check("fresh_mavros_state", False, "MAVROS mode is empty")
    return Check("fresh_mavros_state", True, f"connected mode={mode} age={age_s:.3f}s")


def check_operator_approval(path: Path | None) -> Check:
    if path is None:
        return Check("operator_approval", False, "missing --operator-approval")
    if not path.exists():
        return Check("operator_approval", False, f"{path} does not exist")
    try:
        data = load_structured(path)
    except Exception as exc:
        return Check("operator_approval", False, f"could not parse approval: {exc}")
    required_booleans = [
        "safety_pilot_present",
        "kill_switch_tested",
        "geofence_checked",
        "battery_checked",
        "propeller_area_clear",
        "manual_takeover_ready",
    ]
    missing = [name for name in required_booleans if not truthy(data.get(name))]
    text_fields = ["mission_id", "approved_by"]
    missing.extend(name for name in text_fields if not str(data.get(name, "")).strip())
    if missing:
        return Check("operator_approval", False, f"missing/false fields: {', '.join(missing)}")
    return Check("operator_approval", True, f"mission {data['mission_id']} approved by {data['approved_by']}")


def check_control_intent(flag: bool) -> Check:
    env_value = os.environ.get(CONTROL_ENV_NAME, "")
    if not flag:
        return Check("explicit_control_intent", False, "missing --allow-px4-control")
    if env_value != CONTROL_ENV_VALUE:
        return Check(
            "explicit_control_intent",
            False,
            f"{CONTROL_ENV_NAME} must equal {CONTROL_ENV_VALUE}",
        )
    return Check("explicit_control_intent", True, "explicit operator intent present")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fail-closed safety gate before enabling any CVTrack PX4 control path."
    )
    parser.add_argument("--mode", choices=["audit", "control"], default="audit")
    parser.add_argument("--evidence-dir", type=Path)
    parser.add_argument("--world-calibration", type=Path)
    parser.add_argument("--mavros-state-file", type=Path)
    parser.add_argument("--operator-approval", type=Path)
    parser.add_argument("--max-state-age-seconds", type=float, default=2.0)
    parser.add_argument("--allow-armed-state", action="store_true")
    parser.add_argument("--allow-px4-control", action="store_true")
    parser.add_argument("--write-decision", type=Path)
    parser.add_argument("--fail-on-deny", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    checks = [
        check_evidence_dir(args.evidence_dir),
        check_world_calibration(args.world_calibration),
        check_mavros_state(args.mavros_state_file, args.max_state_age_seconds, args.allow_armed_state),
        check_operator_approval(args.operator_approval),
        check_control_intent(args.allow_px4_control),
    ]
    authorized = args.mode == "control" and all(check.passed for check in checks)
    decision = {
        "schema": "cvtrack-px4-control-gate-v1",
        "mode": args.mode,
        "authorized": authorized,
        "checks": [
            {"name": check.name, "passed": check.passed, "detail": check.detail}
            for check in checks
        ],
        "required_env": {
            "name": CONTROL_ENV_NAME,
            "value": CONTROL_ENV_VALUE,
        },
    }
    rendered = json.dumps(decision, indent=2, ensure_ascii=False)
    if args.write_decision:
        args.write_decision.parent.mkdir(parents=True, exist_ok=True)
        args.write_decision.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    if not authorized and (args.mode == "control" or args.fail_on_deny):
        raise SystemExit(3)


if __name__ == "__main__":
    main()
