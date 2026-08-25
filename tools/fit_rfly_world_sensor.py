#!/usr/bin/env python3
"""Fit and hold out-validate a flat-ground Rfly RGB sensor projection."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--observations", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-samples", type=int, default=30)
    return parser.parse_args()


def read_observations(path: Path) -> list[dict]:
    observations = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            item = json.loads(line)
            pose = item["camera_pose"]
            truth = item["target_truth"]
            if len(pose) >= 3 and all(
                math.isfinite(float(value))
                for value in (item["image_x"], item["image_y"], pose[0], pose[1], pose[2], truth["x"], truth["y"])
            ):
                observations.append(item)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
    return observations


def feature_matrix(observations: list[dict]) -> np.ndarray:
    rows = []
    for item in observations:
        width = float(item["image_width"])
        height = float(item["image_height"])
        fov = float(item.get("camera_fov_deg") or 90.0)
        focal = height / (2.0 * math.tan(math.radians(fov) / 2.0))
        altitude = abs(float(item["camera_pose"][2]))
        image_ground_x = (float(item["image_x"]) - width / 2.0) * altitude / focal
        image_ground_y = (float(item["image_y"]) - height / 2.0) * altitude / focal
        rows.append([
            1.0,
            float(item["camera_pose"][0]),
            float(item["camera_pose"][1]),
            image_ground_x,
            image_ground_y,
        ])
    return np.asarray(rows, dtype=float)


def fit(matrix: np.ndarray, observations: list[dict], indices: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x_target = np.asarray([float(item["target_truth"]["x"]) for item in observations])
    y_target = np.asarray([float(item["target_truth"]["y"]) for item in observations])
    x_coefficients, *_ = np.linalg.lstsq(matrix[indices], x_target[indices], rcond=None)
    y_coefficients, *_ = np.linalg.lstsq(matrix[indices], y_target[indices], rcond=None)
    residuals = np.hypot(matrix @ x_coefficients - x_target, matrix @ y_coefficients - y_target)
    return x_coefficients, y_coefficients, residuals


def metrics(values: np.ndarray) -> dict[str, float]:
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def coverage(values: np.ndarray) -> dict[str, float]:
    return {
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "range": float(np.ptp(values)),
        "standard_deviation": float(np.std(values)),
    }


def main() -> None:
    args = parse_args()
    observations = read_observations(args.observations)
    if len(observations) < args.minimum_samples:
        raise RuntimeError(
            f"only {len(observations)} valid observations; need {args.minimum_samples}"
        )
    matrix = feature_matrix(observations)
    all_indices = np.arange(len(observations))
    initial_x, initial_y, initial_residuals = fit(matrix, observations, all_indices)
    median = float(np.median(initial_residuals))
    mad = float(np.median(np.abs(initial_residuals - median)))
    threshold = max(median + 3.0 * max(mad, 0.25), 2.0)
    inliers = np.flatnonzero(initial_residuals <= threshold)
    if len(inliers) < args.minimum_samples:
        raise RuntimeError(f"only {len(inliers)} inliers after robust filtering")
    train_indices = inliers[inliers % 5 != 0]
    holdout_indices = inliers[inliers % 5 == 0]
    if len(holdout_indices) < 5:
        holdout_indices = inliers[-max(5, len(inliers) // 5):]
        train_indices = np.setdiff1d(inliers, holdout_indices)
    x_coefficients, y_coefficients, final_residuals = fit(
        matrix, observations, train_indices
    )
    holdout_x = np.asarray([float(item["target_truth"]["x"]) for item in observations])[holdout_indices]
    holdout_y = np.asarray([float(item["target_truth"]["y"]) for item in observations])[holdout_indices]
    holdout_residuals = np.hypot(
        matrix[holdout_indices] @ x_coefficients - holdout_x,
        matrix[holdout_indices] @ y_coefficients - holdout_y,
    )
    image_ground_x_coverage = coverage(matrix[inliers, 3])
    image_ground_y_coverage = coverage(matrix[inliers, 4])
    sufficient_coverage = (
        image_ground_x_coverage["range"] >= 35.0
        and image_ground_y_coverage["range"] >= 35.0
    )
    result = {
        "schema": "rfly_world_sensor_affine_v1",
        "feature_order": ["bias", "sensor_x", "sensor_y", "image_ground_x", "image_ground_y"],
        "world_x_coefficients": [float(value) for value in x_coefficients],
        "world_y_coefficients": [float(value) for value in y_coefficients],
        "observation_count": len(observations),
        "inlier_count": len(inliers),
        "training_error_m": metrics(final_residuals[train_indices]),
        "holdout_error_m": metrics(holdout_residuals),
        "image_ground_coverage_m": {
            "x": image_ground_x_coverage,
            "y": image_ground_y_coverage,
        },
        "acceptance": {
            "maximum_holdout_median_error": 5.0,
            "maximum_holdout_p95_error": 12.0,
            "minimum_image_ground_range_m": 35.0,
            "passed": bool(
                np.median(holdout_residuals) <= 5.0
                and np.percentile(holdout_residuals, 95) <= 12.0
                and sufficient_coverage
            ),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
