"""Tests for calibrated image-to-ground-plane projection."""

from __future__ import annotations

import numpy as np
import pytest

from cvtrack.geometry import CalibrationError, GroundPlaneProjector


def _calibration() -> dict:
    return {
        "frame_id": "site_map",
        "units": "m",
        "image_points_px": [[0, 0], [100, 0], [100, 50], [0, 50]],
        "world_points_m": [[10, 20], [20, 20], [20, 30], [10, 30]],
        "max_reprojection_error_m": 1e-6,
    }


def test_projects_known_point_in_metres() -> None:
    projector = GroundPlaneProjector.from_dict(_calibration())

    point = projector.project(50, 25)

    assert point.frame_id == "site_map"
    assert point.x_m == pytest.approx(15.0)
    assert point.y_m == pytest.approx(25.0)
    assert projector.reprojection_rmse_m == pytest.approx(0.0, abs=1e-9)


def test_rejects_collinear_calibration_points() -> None:
    calibration = _calibration()
    calibration["image_points_px"] = [[0, 0], [1, 0], [2, 0], [3, 0]]

    with pytest.raises(CalibrationError, match="collinear"):
        GroundPlaneProjector.from_dict(calibration)


def test_rejects_duplicate_calibration_points() -> None:
    calibration = _calibration()
    calibration["image_points_px"] = [[0, 0], [100, 0], [100, 0], [0, 50]]

    with pytest.raises(CalibrationError, match="duplicate"):
        GroundPlaneProjector.from_dict(calibration)


def test_rejects_homography_horizon() -> None:
    projector = GroundPlaneProjector(
        homography=np.array([[1, 0, 0], [0, 1, 0], [0, 1, 0]], dtype=np.float64),
        frame_id="site_map",
        source="test",
    )

    with pytest.raises(CalibrationError, match="horizon"):
        projector.project(4, 0)
