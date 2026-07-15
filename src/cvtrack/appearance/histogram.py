"""Lightweight appearance fallback: HSV colour-histogram embedding.

When the heavier OSNet ReID network is unavailable (no torch, no weights, or
the user just doesn't want a GPU-bound embedding), we still want *something*
that distinguishes a red car from a green pedestrian at the cosine-distance
level.  This extractor is the answer:

* Resize the cropped box to 32x32 (bilinear).
* Convert to HSV.
* Build a 3D histogram with bins (8, 8, 8) -> 512-D, L2-normalised.

The output obeys the same ``AppearanceExtractor`` protocol used by
``cvtrack.appearance.base``, so ``Gallery`` accepts it without modification.
This is intentionally cheap: a single cv2.resize + cv2.calcHist per crop,
no GPU.

The naming follows the same convention as the OSNet module
(``encode(crop_bgr) -> np.ndarray``); for backward compatibility we also
expose ``__call__(image, box_xyxy)`` so callers can use either style.
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple

import cv2
import numpy as np

from cvtrack.appearance.base import crop_with_margin, l2_normalize


log = logging.getLogger(__name__)


class HistogramExtractor:
    """HSV-histogram appearance embedding (no DL deps).

    Parameters
    ----------
    resize_hw:
        ``(height, width)`` to which the crop is resized before histogram
        extraction.  32x32 is fast and empirically good enough to
        disambiguate colour-dominant targets in aerial footage.
    h_bins, s_bins, v_bins:
        Number of histogram bins per HSV channel.  Total embedding
        dimension is ``h_bins * s_bins * v_bins``.
    min_side:
        Crops with a side below this many pixels are skipped (returns None).
    margin:
        Proportional margin around the bbox, forwarded to
        :func:`crop_with_margin`.
    """

    DEFAULT_HW: Tuple[int, int] = (32, 32)

    def __init__(
        self,
        resize_hw: Tuple[int, int] = DEFAULT_HW,
        h_bins: int = 8,
        s_bins: int = 8,
        v_bins: int = 8,
        min_side: int = 8,
        margin: float = 0.10,
    ) -> None:
        self.input_hw = resize_hw
        self.h_bins = int(h_bins)
        self.s_bins = int(s_bins)
        self.v_bins = int(v_bins)
        self.min_side = int(min_side)
        self.margin = float(margin)
        self._dim = self.h_bins * self.s_bins * self.v_bins

    @property
    def is_available(self) -> bool:
        # Pure-OpenCV; always available.
        return True

    @property
    def loaded_pretrained(self) -> bool:
        # No pretrained weights; this extractor is deterministic.
        return False

    @property
    def embedding_dim(self) -> int:
        return int(self._dim)

    def encode(self, crop_bgr: np.ndarray) -> Optional[np.ndarray]:
        """Encode a BGR crop to an L2-normalised HSV-histogram embedding."""
        if crop_bgr is None or crop_bgr.size == 0:
            return None
        if min(crop_bgr.shape[0], crop_bgr.shape[1]) < self.min_side:
            return None
        try:
            resized = cv2.resize(
                crop_bgr,
                (self.input_hw[1], self.input_hw[0]),
                interpolation=cv2.INTER_AREA,
            )
        except cv2.error as exc:
            log.debug("histogram: resize failed: %s", exc)
            return None
        if resized.ndim == 2:
            resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        # H in [0, 180), S/V in [0, 256) when the image is uint8.
        hist = cv2.calcHist(
            [hsv],
            channels=[0, 1, 2],
            mask=None,
            histSize=[self.h_bins, self.s_bins, self.v_bins],
            ranges=[0, 180, 0, 256, 0, 256],
        )
        hist = hist.astype(np.float64).reshape(-1)
        # L1-normalise first so brightness scaling doesn't dominate.
        s = float(hist.sum())
        if s > 1e-9:
            hist /= s
        return l2_normalize(hist)

    def __call__(
        self,
        image: np.ndarray,
        box_xyxy: Tuple[float, float, float, float],
    ) -> Optional[np.ndarray]:
        crop = crop_with_margin(image, box_xyxy, margin=self.margin)
        if crop is None:
            return None
        return self.encode(crop)


__all__ = ["HistogramExtractor"]