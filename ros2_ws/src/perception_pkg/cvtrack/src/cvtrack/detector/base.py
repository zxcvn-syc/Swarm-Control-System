"""Detector base protocol + helpers."""

from __future__ import annotations

from typing import List, Protocol

import numpy as np

from cvtrack.types import Box


class Detector(Protocol):
    """A frame -> list-of-Boxes detector."""

    model_name: str

    def __call__(self, frame: np.ndarray) -> List[Box]: ...


def class_aware_nms(boxes: List[Box], iou_thresh: float = 0.55) -> List[Box]:
    """Simple class-aware NMS.

    Boxes are sorted by descending score; any box that overlaps a kept box of
    the same class by more than ``iou_thresh`` is dropped.
    """
    if not boxes:
        return boxes
    boxes = sorted(boxes, key=lambda b: -b.score)
    keep: List[Box] = []
    for b in boxes:
        drop = False
        for k in keep:
            if b.cls == k.cls and b.iou(k) > iou_thresh:
                drop = True
                break
        if not drop:
            keep.append(b)
    return keep