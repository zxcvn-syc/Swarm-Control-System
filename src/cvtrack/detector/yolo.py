"""YOLOv8 detector via Ultralytics.

Preserves the v4 geometric post-filters:

* ``min_box_area`` drop
* class-specific aspect-ratio gate
* visibility (in-frame) gate
* class-aware NMS at IoU=0.5
"""

from __future__ import annotations

import logging
import os
from typing import List, Optional, Sequence

import numpy as np

from cvtrack.detector.base import class_aware_nms
from cvtrack.types import Box


log = logging.getLogger(__name__)


# COCO class id -> (name, min aspect ratio h/w, max aspect ratio h/w).
# Values come from the COCO bbox priors for 640x640 input (Ultralytics
# default data.yaml).  They are tuned to reject sky / crosswalk stripes /
# shadow blobs.
_ASPECT_PRIORS = {
    0:  ("person",     0.25, 7.0),
    1:  ("bicycle",    0.3,  5.0),
    2:  ("car",        0.20, 5.0),
    3:  ("motorcycle", 0.3,  5.0),
    5:  ("bus",        0.15, 4.0),
    7:  ("truck",      0.15, 4.0),
    16: ("dog",        0.30, 4.0),
    24: ("backpack",   0.4,  3.5),
    28: ("suitcase",   0.25, 4.0),
    39: ("bottle",     0.35, 8.0),
}


class YoloDetector:
    """Ultralytics YOLO wrapper with v4's geometric post-filters."""

    def __init__(
        self,
        weights: str,
        device: str = "cpu",
        conf: float = 0.15,
        classes: Optional[Sequence[int]] = None,
        imgsz: int = 320,
        min_box_area: float = 200.0,
        min_conf: float = 0.0,
        edge_margin: int = 3,
        nms_iou: float = 0.50,
    ) -> None:
        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as exc:
            raise RuntimeError(f"ultralytics unavailable: {exc}")
        if not os.path.isfile(weights):
            raise FileNotFoundError(f"YOLO weights not found: {weights}")
        self.model = YOLO(weights)
        self.device = device
        self.conf = conf
        self.classes = list(classes) if classes else None
        self.imgsz = int(imgsz)
        self.min_box_area = float(min_box_area)
        self.min_conf = float(min_conf)
        self.edge_margin = int(edge_margin)
        self.nms_iou = float(nms_iou)
        self.model_name = f"yolov8 {os.path.basename(weights)}"

    def __call__(self, frame: np.ndarray) -> List[Box]:
        r = self.model.predict(
            frame,
            device=self.device,
            conf=self.conf,
            classes=self.classes,
            imgsz=self.imgsz,
            verbose=False,
        )[0]
        boxes: List[Box] = []
        if r.boxes is None:
            return boxes
        names = r.names
        H, W = frame.shape[:2]
        for b in r.boxes:
            x1, y1, x2, y2 = b.xyxy[0].cpu().numpy().tolist()
            score = float(b.conf[0].cpu().numpy())
            cls = int(b.cls[0].cpu().numpy())
            if self.min_conf > 0 and score < self.min_conf:
                continue
            box = Box(x1, y1, x2, y2, score, cls, names[cls])
            if box.area < self.min_box_area:
                continue
            prior = _ASPECT_PRIORS.get(cls)
            if prior is not None:
                aspect_min, aspect_max = prior[1], prior[2]
                if not (aspect_min <= box.aspect <= aspect_max):
                    continue
            vis = max(0.0, min(x2, W - 1) - max(x1, 0)) * max(
                0.0, min(y2, H - 1) - max(y1, 0)
            )
            if box.area > 0 and (vis / box.area) < 0.10:
                continue
            boxes.append(box)
        return class_aware_nms(boxes, iou_thresh=self.nms_iou)


class MOG2Detector:
    """Fallback detector: MOG2 background subtraction for synthetic videos.

    Behaves like the v4 SyntheticDetector so users without a GPU can still
    run the demo on simple inputs.
    """

    def __init__(self, frame_size=None) -> None:
        import cv2
        self.size = frame_size
        self.bg = cv2.createBackgroundSubtractorMOG2(history=80, varThreshold=32, detectShadows=False)
        self.model_name = "MOG2 blob"

    def warm_up(self, frame) -> None:
        for _ in range(15):
            self.bg.apply(frame)

    def __call__(self, frame: np.ndarray) -> List[Box]:
        import cv2
        mask = self.bg.apply(frame)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        )
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out: List[Box] = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 200:
                continue
            x, y, w, h = cv2.boundingRect(c)
            x1, y1, x2, y2 = float(x), float(y), float(x + w), float(y + h)
            aspect = h / max(w, 1)
            label = "person" if aspect > 1.4 else "car"
            cls = 0 if label == "person" else 2
            out.append(Box(x1, y1, x2, y2, 0.7, cls, label))
        return out