"""Detector factory: pick backend, fall back gracefully."""

from __future__ import annotations

import logging
import os
from typing import Optional

from cvtrack.detector.base import Detector
from cvtrack.detector.yolo import MOG2Detector, YoloDetector


log = logging.getLogger(__name__)


def make_detector(
    backend: str,
    *,
    weights: Optional[str] = None,
    device: str = "cpu",
    conf: float = 0.15,
    classes=None,
    imgsz: int = 320,
    min_box_area: float = 200.0,
    min_conf: float = 0.0,
    nms_iou: float = 0.50,
    fallback_mog2: bool = True,
) -> Detector:
    """Construct a detector.

    ``backend`` is one of:

    * ``yolo``     -- use YoloDetector; raises if the weights are missing.
    * ``auto``     -- try YOLO if weights exist, else fall back to MOG2.
    * ``mog2``     -- always MOG2 (synthetic-video fallback).
    """
    backend = backend.lower()
    if backend == "yolo":
        return YoloDetector(
            weights=weights or "",
            device=device,
            conf=conf,
            classes=classes,
            imgsz=imgsz,
            min_box_area=min_box_area,
            min_conf=min_conf,
            nms_iou=nms_iou,
        )
    if backend == "auto":
        if weights and os.path.isfile(weights):
            try:
                return YoloDetector(
                    weights=weights,
                    device=device,
                    conf=conf,
                    classes=classes,
                    imgsz=imgsz,
                    min_box_area=min_box_area,
                    min_conf=min_conf,
                    nms_iou=nms_iou,
                )
            except Exception as exc:
                if not fallback_mog2:
                    raise
                log.warning("YOLO failed (%s); falling back to MOG2", exc)
        if not fallback_mog2:
            raise FileNotFoundError(f"YOLO weights not found at {weights!r}")
        return MOG2Detector()
    if backend == "mog2":
        return MOG2Detector()
    raise ValueError(f"unknown detector backend: {backend!r}")