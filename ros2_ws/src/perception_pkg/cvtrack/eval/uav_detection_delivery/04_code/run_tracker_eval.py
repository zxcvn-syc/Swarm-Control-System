"""Run a reproducible IoU tracker over the frozen detection export.

The delivery's CSV evidence is intentionally immutable. By default this
script reads ``01_detection`` and writes a separate experimental result under
``04_code/outputs/tracker``. Passing another output directory is explicit
opt-in, which permits comparisons against a copied or external data set.

``legacy`` exactly preserves the original greedy association rule so its
results can be compared with the frozen tracker CSVs. ``motion`` is an
experimental constant-velocity/Hungarian association; it must be evaluated
and reported separately from the frozen baseline.
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment


SCENES = ("park", "security", "border")
IOU_THRESHOLD = 0.30
MAX_AGE = 10
MATCH_SAME_CLASS = True

DELIVERY_DIR = Path(__file__).resolve().parent.parent
DEFAULT_INPUT_DIR = DELIVERY_DIR / "01_detection"
DEFAULT_OUTPUT_DIR = DELIVERY_DIR / "04_code" / "outputs" / "tracker"

INPUT_FIELDS = {
    "frame_index",
    "video_time",
    "class_id",
    "class_name",
    "confidence",
    "x1",
    "y1",
    "x2",
    "y2",
}
OUTPUT_FIELDS = (
    "scene",
    "frame_index",
    "video_time",
    "track_id",
    "class_id",
    "class_name",
    "confidence",
    "center_x",
    "center_y",
    "x1",
    "y1",
    "x2",
    "y2",
    "match_iou",
)


@dataclass(frozen=True)
class Detection:
    """One validated row from a GT-aligned detector export."""

    frame_index: int
    video_time: float
    class_id: int
    class_name: str
    confidence: float
    bbox: np.ndarray


@dataclass
class TrackState:
    """State needed by either association mode for a single local track."""

    bbox: np.ndarray
    class_id: int
    class_name: str
    last_frame: int
    velocity: np.ndarray


@dataclass(frozen=True)
class SceneSummary:
    """Counts emitted after writing one scene's tracking CSV."""

    scene: str
    input_path: Path
    output_path: Path
    frame_count: int
    detection_count: int
    track_count: int


def calculate_iou(box1: Sequence[float], box2: Sequence[float]) -> float:
    """Return intersection-over-union for two ``xyxy`` boxes."""

    x1 = max(float(box1[0]), float(box2[0]))
    y1 = max(float(box1[1]), float(box2[1]))
    x2 = min(float(box1[2]), float(box2[2]))
    y2 = min(float(box1[3]), float(box2[3]))

    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area1 = max(0.0, float(box1[2]) - float(box1[0])) * max(
        0.0, float(box1[3]) - float(box1[1])
    )
    area2 = max(0.0, float(box2[2]) - float(box2[0])) * max(
        0.0, float(box2[3]) - float(box2[1])
    )
    union = area1 + area2 - intersection
    return intersection / union if union > 0.0 else 0.0


def _parse_int(raw: Any, field: str, path: Path, row_number: int) -> int:
    try:
        value = int(str(raw).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path}: row {row_number} has invalid {field!r}: {raw!r}") from exc
    return value


def _parse_float(raw: Any, field: str, path: Path, row_number: int) -> float:
    try:
        value = float(str(raw).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path}: row {row_number} has invalid {field!r}: {raw!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"{path}: row {row_number} has non-finite {field!r}: {raw!r}")
    return value


def load_detections(path: Path) -> dict[int, list[Detection]]:
    """Load and validate a detector CSV keyed by source frame index."""

    if not path.is_file():
        raise FileNotFoundError(f"Detection CSV does not exist: {path}")

    frames: dict[int, list[Detection]] = defaultdict(list)
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        actual_fields = set(reader.fieldnames or ())
        missing = INPUT_FIELDS - actual_fields
        if missing:
            raise ValueError(
                f"{path}: missing required fields {sorted(missing)}; "
                f"found {reader.fieldnames}"
            )

        for row_number, row in enumerate(reader, start=2):
            frame_index = _parse_int(row.get("frame_index"), "frame_index", path, row_number)
            class_id = _parse_int(row.get("class_id"), "class_id", path, row_number)
            video_time = _parse_float(row.get("video_time"), "video_time", path, row_number)
            confidence = _parse_float(row.get("confidence"), "confidence", path, row_number)
            bbox = np.asarray(
                [
                    _parse_float(row.get("x1"), "x1", path, row_number),
                    _parse_float(row.get("y1"), "y1", path, row_number),
                    _parse_float(row.get("x2"), "x2", path, row_number),
                    _parse_float(row.get("y2"), "y2", path, row_number),
                ],
                dtype=np.float64,
            )
            class_name = str(row.get("class_name") or "").strip()

            if frame_index < 0:
                raise ValueError(f"{path}: row {row_number} has a negative frame_index")
            if class_id < 0:
                raise ValueError(f"{path}: row {row_number} has a negative class_id")
            if not class_name:
                raise ValueError(f"{path}: row {row_number} has an empty class_name")
            if not 0.0 <= confidence <= 1.0:
                raise ValueError(f"{path}: row {row_number} confidence must be within [0, 1]")
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                raise ValueError(f"{path}: row {row_number} has an invalid xyxy bounding box")

            frames[frame_index].append(
                Detection(
                    frame_index=frame_index,
                    video_time=video_time,
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    bbox=bbox,
                )
            )

    return dict(frames)


def _expire_tracks(tracks: dict[int, TrackState], frame_index: int, max_age: int) -> None:
    expired_ids = [
        track_id
        for track_id, track in tracks.items()
        if frame_index - track.last_frame > max_age
    ]
    for track_id in expired_ids:
        del tracks[track_id]


def _legacy_matches(
    tracks: dict[int, TrackState], detections: Sequence[Detection], iou_threshold: float
) -> tuple[dict[int, tuple[int, float]], dict[int, float]]:
    """Replicate the original detection-order greedy IoU association."""

    matches: dict[int, tuple[int, float]] = {}
    best_ious: dict[int, float] = {}
    used_track_ids: set[int] = set()
    for detection_index, detection in enumerate(detections):
        best_track_id: int | None = None
        best_iou = 0.0
        for track_id, track in tracks.items():
            if track_id in used_track_ids:
                continue
            if MATCH_SAME_CLASS and track.class_id != detection.class_id:
                continue
            score = calculate_iou(detection.bbox, track.bbox)
            if score > best_iou:
                best_iou = score
                best_track_id = track_id

        # The original delivery records the best IoU even when it is below the
        # threshold and a new ID is created, so retain it for legacy output.
        best_ious[detection_index] = best_iou
        if best_track_id is not None and best_iou >= iou_threshold:
            matches[detection_index] = (best_track_id, best_iou)
            used_track_ids.add(best_track_id)
    return matches, best_ious


def _predicted_box(track: TrackState, frame_index: int) -> np.ndarray:
    frame_gap = frame_index - track.last_frame
    return track.bbox + track.velocity * max(frame_gap, 0)


def _motion_matches(
    tracks: dict[int, TrackState],
    detections: Sequence[Detection],
    frame_index: int,
    iou_threshold: float,
) -> tuple[dict[int, tuple[int, float]], dict[int, float]]:
    """Associate predicted boxes globally with class-gated Hungarian matching."""

    if not tracks or not detections:
        return {}, {}

    track_ids = list(tracks)
    scores = np.full((len(track_ids), len(detections)), -1.0, dtype=np.float64)
    for row, track_id in enumerate(track_ids):
        track = tracks[track_id]
        predicted_box = _predicted_box(track, frame_index)
        for column, detection in enumerate(detections):
            if MATCH_SAME_CLASS and track.class_id != detection.class_id:
                continue
            scores[row, column] = calculate_iou(predicted_box, detection.bbox)

    # Pairs below the acceptance threshold must be invalid before assignment.
    # Otherwise one excellent + one rejected pair can beat two accepted pairs,
    # reducing the number of valid links after post-assignment filtering.
    accepted = scores >= iou_threshold
    cost = np.where(accepted, 1.0 - scores, 2.0)
    rows, columns = linear_sum_assignment(cost)
    matches: dict[int, tuple[int, float]] = {}
    best_ious = {
        column: max(0.0, float(scores[:, column].max()))
        for column in range(len(detections))
    }
    for row, column in zip(rows.tolist(), columns.tolist()):
        score = float(scores[row, column])
        if accepted[row, column]:
            matches[column] = (track_ids[row], score)
    return matches, best_ious


def _update_track(
    tracks: dict[int, TrackState], track_id: int, detection: Detection, previous: TrackState | None
) -> None:
    if previous is None:
        velocity = np.zeros(4, dtype=np.float64)
    else:
        frame_gap = detection.frame_index - previous.last_frame
        velocity = (detection.bbox - previous.bbox) / frame_gap if frame_gap > 0 else previous.velocity

    tracks[track_id] = TrackState(
        bbox=detection.bbox.copy(),
        class_id=detection.class_id,
        class_name=detection.class_name,
        last_frame=detection.frame_index,
        velocity=velocity,
    )


def _output_row(scene: str, detection: Detection, track_id: int, match_iou: float) -> dict[str, Any]:
    x1, y1, x2, y2 = detection.bbox.tolist()
    return {
        "scene": scene,
        "frame_index": detection.frame_index,
        "video_time": detection.video_time,
        "track_id": track_id,
        "class_id": detection.class_id,
        "class_name": detection.class_name,
        "confidence": detection.confidence,
        "center_x": round((x1 + x2) / 2.0, 2),
        "center_y": round((y1 + y2) / 2.0, 2),
        "x1": round(x1, 2),
        "y1": round(y1, 2),
        "x2": round(x2, 2),
        "y2": round(y2, 2),
        "match_iou": round(match_iou, 4),
    }


def run_scene(
    scene: str,
    input_path: Path,
    output_path: Path,
    *,
    association: str = "legacy",
    iou_threshold: float = IOU_THRESHOLD,
    max_age: int = MAX_AGE,
) -> SceneSummary:
    """Track one scene and write a CSV compatible with ``tracking_success_eval.py``."""

    if association not in {"legacy", "motion"}:
        raise ValueError(f"Unsupported association mode: {association}")
    if not 0.0 <= iou_threshold <= 1.0:
        raise ValueError("iou_threshold must be within [0, 1]")
    if max_age < 0:
        raise ValueError("max_age must be non-negative")

    frames = load_detections(input_path)
    if not frames:
        raise ValueError(f"Detection CSV is empty: {input_path}")

    tracks: dict[int, TrackState] = {}
    next_track_id = 1
    output_rows: list[dict[str, Any]] = []

    for frame_index in sorted(frames):
        detections = frames[frame_index]
        _expire_tracks(tracks, frame_index, max_age)
        if association == "legacy":
            matches, association_ious = _legacy_matches(tracks, detections, iou_threshold)
        else:
            matches, association_ious = _motion_matches(
                tracks, detections, frame_index, iou_threshold
            )

        for detection_index, detection in enumerate(detections):
            match = matches.get(detection_index)
            if match is None:
                track_id = next_track_id
                next_track_id += 1
                match_iou = association_ious.get(detection_index, 0.0)
                previous = None
            else:
                track_id, match_iou = match
                previous = tracks[track_id]

            _update_track(tracks, track_id, detection, previous)
            output_rows.append(_output_row(scene, detection, track_id, match_iou))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(output_rows)

    return SceneSummary(
        scene=scene,
        input_path=input_path,
        output_path=output_path,
        frame_count=len(frames),
        detection_count=len(output_rows),
        track_count=next_track_id - 1,
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help=f"Directory containing detections_eval_<scene>.csv (default: {DEFAULT_INPUT_DIR})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Directory for generated tracker CSVs (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--scenes",
        nargs="+",
        choices=SCENES,
        default=list(SCENES),
        help="Scenes to process (default: all delivery scenes)",
    )
    parser.add_argument(
        "--association",
        choices=("legacy", "motion"),
        default="legacy",
        help="legacy reproduces delivery logic; motion is experimental",
    )
    parser.add_argument("--iou-threshold", type=float, default=IOU_THRESHOLD)
    parser.add_argument("--max-age", type=int, default=MAX_AGE)
    args = parser.parse_args(argv)
    if not 0.0 <= args.iou_threshold <= 1.0:
        parser.error("--iou-threshold must be within [0, 1]")
    if args.max_age < 0:
        parser.error("--max-age must be non-negative")
    return args


def _print_summary(summary: SceneSummary) -> None:
    print(f"Scene: {summary.scene}")
    print(f"  Detection frames: {summary.frame_count}")
    print(f"  Detections: {summary.detection_count}")
    print(f"  Generated track IDs: {summary.track_count}")
    print(f"  Output: {summary.output_path}")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    print("GT-aligned detections -> IoU tracker")
    print(f"Association: {args.association}")
    print(f"IoU threshold: {args.iou_threshold}")
    print(f"Max age: {args.max_age}")
    print("Outputs are experimental and do not replace frozen delivery evidence.")

    try:
        summaries = [
            run_scene(
                scene,
                args.input_dir / f"detections_eval_{scene}.csv",
                args.output_dir / f"tracking_eval_{scene}.csv",
                association=args.association,
                iou_threshold=args.iou_threshold,
                max_age=args.max_age,
            )
            for scene in args.scenes
        ]
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2

    for summary in summaries:
        _print_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
