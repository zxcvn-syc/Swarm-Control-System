"""Camera Motion Compensation (CMC).

Two implementations are provided behind the same ``CameraMotionCompensator``
protocol:

* ``SparseOFCompensator``  - sparse Shi-Tomasi + Lucas-Kanade optical flow
  followed by RANSAC affine (the v4 default).
* ``EccCompensator``       - dense ECC alignment.  Slower on small frames but
  more robust when the scene has few distinct corners (e.g. smooth water).

The compensator returns a 2x3 affine ``A`` such that ``p_k = A @ [p_{k-1}; 1]``,
or ``None`` when the camera is essentially static or the warp is unreliable.
"""

from __future__ import annotations

import logging
from typing import Optional, Protocol

import cv2
import numpy as np

from cvtrack.types import Box


log = logging.getLogger(__name__)


class CameraMotionCompensator(Protocol):
    """Common interface for CMC implementations."""

    def reset(self) -> None: ...
    def __call__(self, frame: np.ndarray, fg_boxes: Optional[list] = None) -> Optional[np.ndarray]: ...


class SparseOFCompensator:
    """Shi-Tomasi + Lucas-Kanade sparse OF + RANSAC affine (the v4 path)."""

    def __init__(self, downscale: float = 0.5) -> None:
        self.downscale = downscale
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_pts: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.prev_gray = None
        self.prev_pts = None

    def __call__(self, frame: np.ndarray, fg_boxes: Optional[list] = None) -> Optional[np.ndarray]:
        if self.downscale != 1.0:
            small = cv2.resize(frame, None, fx=self.downscale, fy=self.downscale, interpolation=cv2.INTER_AREA)
        else:
            small = frame
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) < 8:
            self.prev_gray = gray
            self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=400, qualityLevel=0.01, minDistance=8)
            return None

        cur_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if cur_pts is None:
            self.prev_gray = gray
            self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=400, qualityLevel=0.01, minDistance=8)
            return None
        status = status.reshape(-1)
        src = self.prev_pts.reshape(-1, 2)[status == 1]
        dst = cur_pts.reshape(-1, 2)[status == 1]

        if fg_boxes and len(src) > 0:
            s = self.downscale
            keep = np.ones(len(src), dtype=bool)
            for b in fg_boxes:
                x1, y1 = int(b.x1 * s), int(b.y1 * s)
                x2, y2 = int(b.x2 * s), int(b.y2 * s)
                inside = ((dst[:, 0] >= x1) & (dst[:, 0] <= x2)
                          & (dst[:, 1] >= y1) & (dst[:, 1] <= y2))
                keep &= ~inside
            src = src[keep]
            dst = dst[keep]

        if len(src) < 8:
            self.prev_gray = gray
            self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=400, qualityLevel=0.01, minDistance=8)
            return None

        A, inliers = cv2.estimateAffine2D(
            src, dst, method=cv2.RANSAC, ransacReprojThreshold=3.0,
            maxIters=200, confidence=0.99,
        )
        self.prev_gray = gray
        if inliers is not None and inliers.sum() >= 12:
            keep_inl = inliers.ravel().astype(bool)
            self.prev_pts = dst[keep_inl].reshape(-1, 1, 2)
        else:
            self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=400, qualityLevel=0.01, minDistance=8)
        if A is None:
            return None
        if self.downscale != 1.0:
            s = 1.0 / self.downscale
            A = A.copy()
            A[:, :2] *= s
            A[:, 2] *= s
        return A


class EccCompensator:
    """Dense ECC alignment.  More robust when the scene is texture-poor."""

    def __init__(self, downscale: float = 0.5, num_iters: int = 50, eps: float = 1e-4) -> None:
        self.downscale = downscale
        self.num_iters = num_iters
        self.eps = eps
        self.prev_gray: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.prev_gray = None

    def __call__(self, frame: np.ndarray, fg_boxes: Optional[list] = None) -> Optional[np.ndarray]:
        if self.downscale != 1.0:
            small = cv2.resize(frame, None, fx=self.downscale, fy=self.downscale, interpolation=cv2.INTER_AREA)
        else:
            small = frame
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
        gray = cv2.equalizeHist(gray)
        if self.prev_gray is None or self.prev_gray.shape != gray.shape:
            self.prev_gray = gray
            return None
        warp = np.eye(2, 3, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, self.num_iters, self.eps)
        try:
            _, warp = cv2.findTransformECC(
                self.prev_gray, gray, warp, cv2.MOTION_AFFINE, criteria,
                inputMask=None, gaussFiltSize=5,
            )
        except cv2.error:
            self.prev_gray = gray
            return None
        self.prev_gray = gray
        A = warp.astype(np.float64)
        if self.downscale != 1.0:
            s = 1.0 / self.downscale
            A[:, :2] *= s
            A[:, 2] *= s
        return A


def make_cmc(method: str = "sparse_of", downscale: float = 0.5) -> CameraMotionCompensator:
    """Factory: pick a CMC implementation by name."""
    method = method.lower()
    if method in ("sparse", "sparse_of", "sparseof"):
        return SparseOFCompensator(downscale=downscale)
    if method in ("ecc", "dense"):
        return EccCompensator(downscale=downscale)
    raise ValueError(f"unknown CMC method: {method!r}")


def affine_is_pure_camera_pan(A: np.ndarray, *, scale_lo: float = 0.95,
                              scale_hi: float = 1.05, angle_rad: float = 0.05) -> bool:
    """Sanity-check: reject warps whose scale or rotation looks suspicious.

    A pure camera pan should have ~zero scale change and very small rotation.
    Anything larger is usually the affine absorbing foreground motion.
    """
    scale = float(np.sqrt(A[0, 0] ** 2 + A[1, 0] ** 2))
    cos_a = A[0, 0] / max(scale, 1e-6)
    angle = abs(np.arccos(max(min(cos_a, 1.0), -1.0)))
    if scale < scale_lo or scale > scale_hi or angle > angle_rad:
        return False
    return True