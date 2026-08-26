"""Calibrated image-to-ground-plane coordinate projection.

The tracker intentionally stays in pixel coordinates.  This module provides
the explicit boundary where a detected object's ground-contact pixel is
projected into a local metric frame using a calibrated planar homography.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import yaml


class CalibrationError(ValueError):
    """Raised when a ground-plane calibration is malformed or unusable."""


@dataclass(frozen=True)
class ProjectedPoint:
    """A valid point in the calibrated local ground frame, expressed in metres."""

    x_m: float
    y_m: float
    frame_id: str


@dataclass(frozen=True)
class GroundPlaneProjector:
    """Validated homography from image pixels to a local ground-plane frame."""

    homography: np.ndarray
    frame_id: str
    source: str
    reprojection_rmse_m: float = 0.0

    @classmethod
    def from_file(cls, path: str) -> "GroundPlaneProjector":
        calibration_path = Path(path).expanduser()
        if not calibration_path.is_file():
            raise CalibrationError(f"calibration file does not exist: {calibration_path}")
        try:
            with calibration_path.open("r", encoding="utf-8") as calibration_file:
                data = yaml.safe_load(calibration_file)
        except (OSError, yaml.YAMLError) as exc:
            raise CalibrationError(
                f"cannot read calibration file {calibration_path}: {exc}"
            ) from exc
        return cls.from_dict(data, source=str(calibration_path))

    @classmethod
    def from_dict(
        cls,
        data: Mapping[str, Any] | None,
        *,
        source: str = "<inline calibration>",
    ) -> "GroundPlaneProjector":
        if not isinstance(data, Mapping):
            raise CalibrationError(f"{source}: calibration must be a mapping")
        if data.get("units") != "m":
            raise CalibrationError(f"{source}: calibration units must be exactly 'm'")
        frame_id = data.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise CalibrationError(f"{source}: frame_id must be a non-empty string")

        image_points = _points(data.get("image_points_px"), "image_points_px", source)
        world_points = _points(data.get("world_points_m"), "world_points_m", source)
        if len(image_points) != len(world_points):
            raise CalibrationError(
                f"{source}: image_points_px and world_points_m must have equal length"
            )
        if len(image_points) < 4:
            raise CalibrationError(f"{source}: at least four point correspondences are required")
        if len(np.unique(image_points, axis=0)) < 4:
            raise CalibrationError(f"{source}: image_points_px contain duplicate points")
        if len(np.unique(world_points, axis=0)) < 4:
            raise CalibrationError(f"{source}: world_points_m contain duplicate points")
        if _is_collinear(image_points):
            raise CalibrationError(f"{source}: image_points_px are collinear or repeated")
        if _is_collinear(world_points):
            raise CalibrationError(f"{source}: world_points_m are collinear or repeated")

        homography, _ = cv2.findHomography(image_points, world_points, method=0)
        if homography is None or homography.shape != (3, 3):
            raise CalibrationError(f"{source}: could not solve image-to-world homography")
        if not np.isfinite(homography).all() or np.linalg.matrix_rank(homography) != 3:
            raise CalibrationError(f"{source}: solved homography is singular or non-finite")

        projected = _apply_homography(homography, image_points, source)
        residuals = np.linalg.norm(projected - world_points, axis=1)
        rmse = float(np.sqrt(np.mean(residuals**2)))
        max_error = data.get("max_reprojection_error_m")
        if max_error is not None:
            if not isinstance(max_error, (int, float)) or not np.isfinite(max_error) or max_error < 0:
                raise CalibrationError(
                    f"{source}: max_reprojection_error_m must be a finite non-negative number"
                )
            if float(np.max(residuals)) > float(max_error):
                raise CalibrationError(
                    f"{source}: calibration max reprojection error "
                    f"{float(np.max(residuals)):.3f} m exceeds {float(max_error):.3f} m"
                )

        return cls(
            homography=np.asarray(homography, dtype=np.float64),
            frame_id=frame_id.strip(),
            source=source,
            reprojection_rmse_m=rmse,
        )

    def project(self, image_x_px: float, image_y_px: float) -> ProjectedPoint:
        """Project one finite image point into the calibrated local metric frame."""
        point = np.asarray([image_x_px, image_y_px], dtype=np.float64)
        if point.shape != (2,) or not np.isfinite(point).all():
            raise CalibrationError("image point must contain two finite pixel coordinates")
        world = _apply_homography(self.homography, point.reshape(1, 2), self.source)[0]
        return ProjectedPoint(float(world[0]), float(world[1]), self.frame_id)


def _points(value: Any, field_name: str, source: str) -> np.ndarray:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise CalibrationError(f"{source}: {field_name} must be a list of [x, y] pairs")
    try:
        points = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"{source}: {field_name} must contain numeric pairs") from exc
    if points.ndim != 2 or points.shape[1:] != (2,):
        raise CalibrationError(f"{source}: {field_name} must have shape N x 2")
    if not np.isfinite(points).all():
        raise CalibrationError(f"{source}: {field_name} contains non-finite values")
    return points


def _is_collinear(points: np.ndarray) -> bool:
    centered = points - np.mean(points, axis=0)
    return np.linalg.matrix_rank(centered) < 2


def _apply_homography(homography: np.ndarray, points: np.ndarray, source: str) -> np.ndarray:
    homogeneous = np.column_stack((points, np.ones(len(points), dtype=np.float64)))
    projected = homogeneous @ np.asarray(homography, dtype=np.float64).T
    denominator = projected[:, 2]
    if not np.isfinite(denominator).all() or np.any(np.abs(denominator) < 1e-12):
        raise CalibrationError(f"{source}: projection reached an invalid homography horizon")
    output = projected[:, :2] / denominator[:, None]
    if not np.isfinite(output).all():
        raise CalibrationError(f"{source}: projection produced non-finite world coordinates")
    return output


__all__ = ["CalibrationError", "GroundPlaneProjector", "ProjectedPoint"]
