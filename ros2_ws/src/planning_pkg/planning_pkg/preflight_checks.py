"""Pure checks shared by the read-only real-UAV preflight monitor."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Any, Mapping

import yaml


_PLACEHOLDERS = {"", "todo", "tbd", "replace", "replace_me", "null", "none", "n/a"}


@dataclass(frozen=True)
class CheckResult:
    """One fail-closed preflight observation."""

    name: str
    passed: bool
    detail: str
    observation: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def is_present(value: object) -> bool:
    """Return true only for non-placeholder scalar values."""

    if value is None:
        return False
    if isinstance(value, str):
        normalized = value.strip().casefold()
        return (
            normalized not in _PLACEHOLDERS
            and not normalized.startswith(("replace_", "your_", "example_"))
        )
    return True


def finite_number(value: object) -> float | None:
    """Convert a finite numeric value, returning ``None`` for invalid input."""

    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def load_yaml_mapping(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    """Load a YAML mapping without treating an unreadable record as valid."""

    if not path.is_file():
        return None, "file_missing"
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        return None, f"yaml_unreadable:{error.__class__.__name__}"
    if not isinstance(loaded, dict):
        return None, "yaml_mapping_required"
    return loaded, None


def nested(mapping: Mapping[str, Any], *keys: str) -> Any:
    """Read a nested mapping key, returning ``None`` when a level is missing."""

    value: Any = mapping
    for key in keys:
        if not isinstance(value, Mapping):
            return None
        value = value.get(key)
    return value


def _artifact_check(manifest_path: Path, value: object, field: str) -> str | None:
    if not is_present(value):
        return f"{field}_missing"
    artifact = Path(str(value)).expanduser()
    if not artifact.is_absolute():
        artifact = manifest_path.parent / artifact
    return None if artifact.is_file() else f"{field}_not_found"


def validate_calibration_manifest(
    path: Path,
    *,
    max_reprojection_error_px: float,
) -> CheckResult:
    """Validate a flight-specific calibration record and referenced artifacts.

    This validates that a calibration record exists and is complete. It cannot
    determine whether its measurements were performed correctly, which remains
    a field responsibility recorded in the operator checklist.
    """

    manifest, load_error = load_yaml_mapping(path)
    if manifest is None:
        return CheckResult(
            "calibration_manifest",
            False,
            load_error or "manifest_invalid",
            {"path": str(path)},
        )

    errors: list[str] = []
    for field, value in {
        "vehicle.airframe": nested(manifest, "vehicle", "airframe"),
        "vehicle.flight_controller_serial": nested(
            manifest, "vehicle", "flight_controller_serial"
        ),
        "camera.serial": nested(manifest, "camera", "serial"),
        "camera.frame_id": nested(manifest, "camera", "frame_id"),
        "intrinsics.calibrated_at": nested(manifest, "intrinsics", "calibrated_at"),
        "extrinsics.body_frame": nested(manifest, "extrinsics", "body_frame"),
        "extrinsics.camera_frame": nested(manifest, "extrinsics", "camera_frame"),
        "extrinsics.verified_at": nested(manifest, "extrinsics", "verified_at"),
        "local_origin.frame_id": nested(manifest, "local_origin", "frame_id"),
        "local_origin.verified_at": nested(manifest, "local_origin", "verified_at"),
        "ground_reference.verified_at": nested(manifest, "ground_reference", "verified_at"),
    }.items():
        if not is_present(value):
            errors.append(f"{field}_missing")

    for field, value in {
        "intrinsics.artifact": nested(manifest, "intrinsics", "artifact"),
        "extrinsics.artifact": nested(manifest, "extrinsics", "artifact"),
    }.items():
        issue = _artifact_check(path, value, field)
        if issue:
            errors.append(issue)

    reprojection_error = finite_number(nested(manifest, "intrinsics", "reprojection_error_px"))
    if reprojection_error is None:
        errors.append("intrinsics.reprojection_error_px_invalid")
    elif reprojection_error > max_reprojection_error_px:
        errors.append("intrinsics.reprojection_error_px_exceeds_limit")

    camera_frame = nested(manifest, "camera", "frame_id")
    extrinsics_frame = nested(manifest, "extrinsics", "camera_frame")
    if (
        is_present(camera_frame)
        and is_present(extrinsics_frame)
        and camera_frame != extrinsics_frame
    ):
        errors.append("camera_frame_extrinsics_mismatch")

    detail = "calibration_record_complete" if not errors else ";".join(errors)
    return CheckResult(
        "calibration_manifest",
        not errors,
        detail,
        {
            "path": str(path),
            "camera_frame": camera_frame,
            "local_frame": nested(manifest, "local_origin", "frame_id"),
            "reprojection_error_px": reprojection_error,
            "max_reprojection_error_px": max_reprojection_error_px,
        },
    )


_OPERATOR_CHECKS = (
    "airspace_authorized",
    "weather_acceptable",
    "physical_emergency_stop_tested",
    "rc_link_and_failsafe_verified",
    "px4_failsafes_reviewed",
    "battery_and_propellers_inspected",
    "observer_briefed",
    "flight_area_secured",
    "operator_has_manual_control",
    "safety_gate_locked_at_start",
)


def validate_operator_checklist(path: Path) -> CheckResult:
    """Validate a filled, explicit field-safety attestation record."""

    checklist, load_error = load_yaml_mapping(path)
    if checklist is None:
        return CheckResult(
            "operator_checklist",
            False,
            load_error or "checklist_invalid",
            {"path": str(path)},
        )

    errors: list[str] = []
    for field in ("flight_director", "safety_pilot", "recorded_at"):
        if not is_present(checklist.get(field)):
            errors.append(f"{field}_missing")
    checks = checklist.get("checks")
    if not isinstance(checks, Mapping):
        errors.append("checks_mapping_required")
        checks = {}
    unchecked = [name for name in _OPERATOR_CHECKS if checks.get(name) is not True]
    if unchecked:
        errors.append("unchecked:" + ",".join(unchecked))

    detail = "operator_attestation_complete" if not errors else ";".join(errors)
    return CheckResult(
        "operator_checklist",
        not errors,
        detail,
        {
            "path": str(path),
            "flight_director": checklist.get("flight_director"),
            "safety_pilot": checklist.get("safety_pilot"),
            "recorded_at": checklist.get("recorded_at"),
            "unchecked": unchecked,
        },
    )


def fresh_topic_check(
    name: str,
    *,
    count: int,
    receive_age_seconds: float | None,
    source_age_seconds: float | None,
    max_age_seconds: float,
    require_source_stamp: bool,
    rate_hz: float | None = None,
    min_rate_hz: float | None = None,
    observation: dict[str, Any] | None = None,
) -> CheckResult:
    """Evaluate topic liveness using receive time and optional ROS stamps."""

    details: list[str] = []
    if count < 1:
        details.append("no_messages")
    if receive_age_seconds is None or receive_age_seconds > max_age_seconds:
        details.append("receive_stale")
    if require_source_stamp and (
        source_age_seconds is None or abs(source_age_seconds) > max_age_seconds
    ):
        details.append("source_stamp_stale_or_missing")
    if min_rate_hz is not None and (rate_hz is None or rate_hz < min_rate_hz):
        details.append("rate_below_minimum")
    collected = {
        "count": count,
        "receive_age_seconds": receive_age_seconds,
        "source_age_seconds": source_age_seconds,
        "rate_hz": rate_hz,
        "max_age_seconds": max_age_seconds,
    }
    if observation:
        collected.update(observation)
    return CheckResult(name, not details, "fresh" if not details else ";".join(details), collected)


def utc_now() -> str:
    """Return an ISO-8601 timestamp suitable for a JSON evidence file."""

    return datetime.now(timezone.utc).isoformat()
