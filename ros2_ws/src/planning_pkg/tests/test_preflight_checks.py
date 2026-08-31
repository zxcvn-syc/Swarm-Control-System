from pathlib import Path

from planning_pkg.preflight_checks import (
    fresh_topic_check,
    validate_calibration_manifest,
    validate_operator_checklist,
)


def _write(path: Path, content: str) -> Path:
    path.write_text(content, encoding="utf-8")
    return path


def test_complete_calibration_manifest_requires_real_artifacts(tmp_path):
    intrinsics = _write(tmp_path / "camera.yaml", "camera_matrix: []\n")
    extrinsics = _write(tmp_path / "camera_to_body.yaml", "transform: []\n")
    manifest = _write(
        tmp_path / "calibration.yaml",
        f"""vehicle:
  airframe: quadrotor
  flight_controller_serial: FCU-123
camera:
  serial: CAM-123
  frame_id: camera_optical_frame
intrinsics:
  artifact: {intrinsics.name}
  calibrated_at: 2026-08-30T08:00:00Z
  reprojection_error_px: 0.4
extrinsics:
  artifact: {extrinsics.name}
  body_frame: base_link
  camera_frame: camera_optical_frame
  verified_at: 2026-08-30T08:05:00Z
local_origin:
  frame_id: map
  verified_at: 2026-08-30T08:10:00Z
ground_reference:
  verified_at: 2026-08-30T08:15:00Z
""",
    )

    result = validate_calibration_manifest(manifest, max_reprojection_error_px=1.0)

    assert result.passed
    assert result.detail == "calibration_record_complete"


def test_calibration_manifest_rejects_placeholder_and_high_error(tmp_path):
    manifest = _write(
        tmp_path / "calibration.yaml",
        """vehicle: {}
camera:
  serial: REPLACE_WITH_CAMERA_SERIAL
  frame_id: camera_optical_frame
intrinsics:
  artifact: missing.yaml
  reprojection_error_px: 3.0
extrinsics:
  artifact: missing_tf.yaml
  camera_frame: other_camera_frame
local_origin: {}
ground_reference: {}
""",
    )

    result = validate_calibration_manifest(manifest, max_reprojection_error_px=1.0)

    assert not result.passed
    assert "camera.serial_missing" in result.detail
    assert "intrinsics.reprojection_error_px_exceeds_limit" in result.detail
    assert "camera_frame_extrinsics_mismatch" in result.detail


def test_operator_checklist_fails_closed_until_every_required_check_is_true(tmp_path):
    checklist = _write(
        tmp_path / "checklist.yaml",
        """flight_director: director
safety_pilot: pilot
recorded_at: 2026-08-30T08:00:00Z
checks:
  airspace_authorized: true
  weather_acceptable: false
""",
    )

    result = validate_operator_checklist(checklist)

    assert not result.passed
    assert "unchecked:weather_acceptable" in result.detail
    assert "physical_emergency_stop_tested" in result.detail


def test_fresh_topic_check_requires_message_age_stamp_and_rate():
    missing = fresh_topic_check(
        "camera_image",
        count=0,
        receive_age_seconds=None,
        source_age_seconds=None,
        max_age_seconds=1.0,
        require_source_stamp=True,
        rate_hz=None,
        min_rate_hz=10.0,
    )
    healthy = fresh_topic_check(
        "camera_image",
        count=30,
        receive_age_seconds=0.05,
        source_age_seconds=0.03,
        max_age_seconds=1.0,
        require_source_stamp=True,
        rate_hz=29.0,
        min_rate_hz=10.0,
    )

    assert not missing.passed
    assert "no_messages" in missing.detail
    assert healthy.passed


def test_shipped_field_templates_are_intentionally_no_go():
    package_root = Path(__file__).parents[1]

    calibration = validate_calibration_manifest(
        package_root / "config" / "real_uav_calibration.template.yaml",
        max_reprojection_error_px=1.0,
    )
    checklist = validate_operator_checklist(
        package_root / "config" / "real_uav_operator_checklist.template.yaml"
    )

    assert not calibration.passed
    assert not checklist.passed
