"""Tests for the standalone, non-destructive delivery tracker runner."""

import csv
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "eval"
    / "uav_detection_delivery"
    / "04_code"
    / "run_tracker_eval.py"
)
SPEC = importlib.util.spec_from_file_location("run_tracker_eval_under_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
tracker_eval = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = tracker_eval
SPEC.loader.exec_module(tracker_eval)


def _write_detections(path: Path) -> None:
    """Use a growing frame gap that needs prediction to preserve an ID."""
    rows = [
        {
            "frame_index": 0,
            "video_time": 0.0,
            "class_id": 0,
            "class_name": "car",
            "confidence": 0.9,
            "x1": 0.0,
            "y1": 0.0,
            "x2": 10.0,
            "y2": 10.0,
        },
        {
            "frame_index": 1,
            "video_time": 0.1,
            "class_id": 0,
            "class_name": "car",
            "confidence": 0.9,
            "x1": 4.0,
            "y1": 0.0,
            "x2": 14.0,
            "y2": 10.0,
        },
        {
            "frame_index": 3,
            "video_time": 0.3,
            "class_id": 0,
            "class_name": "car",
            "confidence": 0.9,
            "x1": 12.0,
            "y1": 0.0,
            "x2": 22.0,
            "y2": 10.0,
        },
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=sorted(tracker_eval.INPUT_FIELDS))
        writer.writeheader()
        writer.writerows(rows)


def _track_ids(path: Path) -> tuple[list[int], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    return [int(row["track_id"]) for row in rows], rows


def test_default_paths_are_repository_relative_and_non_destructive():
    args = tracker_eval.parse_args([])

    assert args.input_dir == tracker_eval.DELIVERY_DIR / "01_detection"
    assert args.output_dir == tracker_eval.DELIVERY_DIR / "04_code" / "outputs" / "tracker"
    assert "02_tracking" not in args.output_dir.parts


def test_motion_prediction_preserves_id_when_legacy_iou_cannot(tmp_path: Path):
    input_path = tmp_path / "detections_eval_park.csv"
    legacy_path = tmp_path / "legacy.csv"
    motion_path = tmp_path / "motion.csv"
    _write_detections(input_path)

    tracker_eval.run_scene("park", input_path, legacy_path, association="legacy")
    tracker_eval.run_scene("park", input_path, motion_path, association="motion")

    legacy_ids, legacy_rows = _track_ids(legacy_path)
    motion_ids, motion_rows = _track_ids(motion_path)
    assert legacy_ids == [1, 1, 2]
    assert motion_ids == [1, 1, 1]
    assert float(legacy_rows[-1]["match_iou"]) == pytest.approx(0.1111)
    assert float(motion_rows[-1]["match_iou"]) == pytest.approx(1.0)
    assert tuple(motion_rows[0]) == tracker_eval.OUTPUT_FIELDS


def test_motion_assignment_prioritizes_number_of_threshold_passing_links():
    """A rejected candidate must not displace two accepted associations."""
    tracks = {
        1: tracker_eval.TrackState(np.array([0.0, 0.0, 10.0, 10.0]), 0, "car", 0, np.zeros(4)),
        2: tracker_eval.TrackState(
            np.array([-5.2, 0.0, 4.8, 10.0]), 0, "car", 0, np.zeros(4)
        ),
    }
    detections = [
        tracker_eval.Detection(1, 0.1, 0, "car", 0.9, np.array([0.0, 0.0, 10.0, 10.0])),
        tracker_eval.Detection(1, 0.1, 0, "car", 0.9, np.array([5.2, 0.0, 15.2, 10.0])),
    ]

    matches, _ = tracker_eval._motion_matches(tracks, detections, frame_index=1, iou_threshold=0.3)

    assert {detection_index: track_id for detection_index, (track_id, _) in matches.items()} == {
        0: 2,
        1: 1,
    }
