"""Re-identification (ReID) appearance feature extraction."""

from __future__ import annotations

import logging
from typing import Optional, Protocol, Tuple

import numpy as np


log = logging.getLogger(__name__)


class AppearanceExtractor(Protocol):
    """Extract an L2-normalised appearance embedding from a detection crop."""

    input_hw: Tuple[int, int]  # (height, width)

    def __call__(self, image: np.ndarray, box_xyxy: Tuple[float, float, float, float]) -> Optional[np.ndarray]:
        ...


def crop_with_margin(
    frame: np.ndarray,
    box_xyxy: Tuple[float, float, float, float],
    margin: float = 0.10,
) -> Optional[np.ndarray]:
    """Crop a region with proportional margin, return None if too small.

    The margin is applied to each side as a fraction of the box's own width /
    height (so we grab a bit of background around the object).  Returns ``None``
    when the cropped box falls below ``min_side`` pixels on either side.
    """
    x1, y1, x2, y2 = box_xyxy
    w, h = max(x2 - x1, 1.0), max(y2 - y1, 1.0)
    mx, my = margin * w, margin * h
    x1m, y1m = max(0.0, x1 - mx), max(0.0, y1 - my)
    x2m, y2m = min(float(frame.shape[1]), x2 + mx), min(float(frame.shape[0]), y2 + my)
    if (x2m - x1m) < 4 or (y2m - y1m) < 4:
        return None
    crop = frame[int(y1m):int(y2m), int(x1m):int(x2m)]
    if crop.size == 0:
        return None
    return crop


def l2_normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return v
    return v / n