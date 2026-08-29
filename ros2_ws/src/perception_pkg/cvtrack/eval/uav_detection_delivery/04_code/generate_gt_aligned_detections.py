"""Generate GT-aligned detections without overwriting frozen delivery CSVs.

Video files and trained weights are external inputs, so callers must provide
``--model-path`` and may supply ``--video-dir``. New detections are written to
``04_code/outputs/detections`` by default and are experimental until evaluated.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any, Sequence


SCENES = ("park", "security", "border")
CONF = 0.50
DELIVERY_DIR = Path(__file__).resolve().parent.parent
DEFAULT_GT_DIR = DELIVERY_DIR / "03_tracking_gt"
DEFAULT_VIDEO_DIR = DELIVERY_DIR / "videos"
DEFAULT_OUTPUT_DIR = DELIVERY_DIR / "04_code" / "outputs" / "detections"
FIELDS = (
    "frame_index",
    "video_time",
    "class_id",
    "class_name",
    "confidence",
    "x1",
    "y1",
    "x2",
    "y2",
)


def get_gt_frames(gt_path: Path) -> list[int]:
    """Return validated, sorted frame indices represented by the GT export."""

    if not gt_path.is_file():
        raise FileNotFoundError(f"Tracking GT does not exist: {gt_path}")

    frames: set[int] = set()
    with gt_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if "frame_index" not in set(reader.fieldnames or ()):
            raise ValueError(f"{gt_path}: missing required field 'frame_index'")
        for row_number, row in enumerate(reader, start=2):
            raw_frame = row.get("frame_index")
            try:
                frame_index = int(str(raw_frame).strip())
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"{gt_path}: row {row_number} has invalid frame_index: {raw_frame!r}"
                ) from exc
            if frame_index < 0:
                raise ValueError(f"{gt_path}: row {row_number} has a negative frame_index")
            frames.add(frame_index)

    if not frames:
        raise ValueError(f"Tracking GT is empty: {gt_path}")
    return sorted(frames)


def _finite_box(box: Sequence[float], scene: str, frame_index: int) -> tuple[float, float, float, float]:
    values = tuple(float(value) for value in box)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{scene} frame {frame_index}: detector returned a non-finite box")
    x1, y1, x2, y2 = values
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f"{scene} frame {frame_index}: detector returned an invalid xyxy box")
    return values


def process_scene(
    model: Any,
    scene: str,
    gt_dir: Path,
    video_dir: Path,
    output_dir: Path,
    confidence_threshold: float,
) -> tuple[int, Path]:
    """Run detector inference on exactly the frames represented in tracking GT."""

    import cv2

    gt_path = gt_dir / f"tracking_gt_{scene}.csv"
    video_path = video_dir / f"{scene}.mp4"
    output_path = output_dir / f"detections_eval_{scene}.csv"
    frames = get_gt_frames(gt_path)
    if not video_path.is_file():
        raise FileNotFoundError(f"Video does not exist: {video_path}")

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"Could not open video: {video_path}")

    fps = float(cap.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(fps) or fps <= 0.0:
        cap.release()
        raise ValueError(f"{video_path}: invalid FPS reported by OpenCV: {fps!r}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    detection_count = 0
    try:
        with output_path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            for frame_index in frames:
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f"{scene}: could not read source frame {frame_index}")

                results = model.predict(frame, conf=confidence_threshold, verbose=False)
                result = results[0]
                if result.boxes is None:
                    continue

                for box in result.boxes:
                    class_id = int(box.cls[0].item())
                    score = float(box.conf[0].item())
                    if not math.isfinite(score) or not 0.0 <= score <= 1.0:
                        raise ValueError(
                            f"{scene} frame {frame_index}: detector returned invalid confidence {score!r}"
                        )
                    x1, y1, x2, y2 = _finite_box(
                        box.xyxy[0].cpu().tolist(), scene, frame_index
                    )
                    class_name = str(model.names[class_id]).strip()
                    if not class_name:
                        raise ValueError(
                            f"{scene} frame {frame_index}: detector returned an empty class name"
                        )
                    writer.writerow(
                        {
                            "frame_index": frame_index,
                            "video_time": round(frame_index / fps, 4),
                            "class_id": class_id,
                            "class_name": class_name,
                            "confidence": round(score, 4),
                            "x1": round(x1, 2),
                            "y1": round(y1, 2),
                            "x2": round(x2, 2),
                            "y2": round(y2, 2),
                        }
                    )
                    detection_count += 1
    finally:
        cap.release()

    print(f"Scene: {scene}")
    print(f"  GT frames: {len(frames)} ({frames[0]} to {frames[-1]})")
    print(f"  Detections: {detection_count}")
    print(f"  Output: {output_path}")
    return detection_count, output_path


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-path", type=Path, required=True, help="YOLO weights file")
    parser.add_argument("--gt-dir", type=Path, default=DEFAULT_GT_DIR)
    parser.add_argument("--video-dir", type=Path, default=DEFAULT_VIDEO_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--conf", type=float, default=CONF)
    parser.add_argument("--scenes", nargs="+", choices=SCENES, default=list(SCENES))
    args = parser.parse_args(argv)
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf must be within [0, 1]")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.model_path.is_file():
        print(f"ERROR: YOLO weights do not exist: {args.model_path}")
        return 2

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics is required; install the cvtrack dependencies first.")
        return 2

    print("Tracking-GT-aligned YOLO detections")
    print(f"Confidence threshold: {args.conf}")
    print("Outputs are experimental and do not replace frozen delivery evidence.")
    model = YOLO(str(args.model_path))
    try:
        for scene in args.scenes:
            process_scene(model, scene, args.gt_dir, args.video_dir, args.output_dir, args.conf)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
