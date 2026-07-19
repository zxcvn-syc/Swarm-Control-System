"""Detector package."""

from cvtrack.detector.base import Detector, class_aware_nms
from cvtrack.detector.yolo import MOG2Detector, YoloDetector

__all__ = ["Detector", "MOG2Detector", "YoloDetector", "class_aware_nms"]