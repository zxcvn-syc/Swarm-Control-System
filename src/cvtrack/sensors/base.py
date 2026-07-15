"""Abstract base classes for the sensor layer.

A ``Sensor`` is anything that produces a per-tick :class:`Frame` plus
optional geometry metadata (intrinsics / extrinsics).  The current
package only ships ``VideoSensor`` (see :mod:`cvtrack.sensors.video`),
but the interface is intentionally tiny so that future sensors (LiDAR,
IMU, multi-camera rigs) can plug in without changing the pipeline.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class Frame:
    """A single tick of sensor data.

    For video sensors this is just an ``H x W x 3`` BGR image plus the
    integer frame index.  Future sensor types (LiDAR / IMU) can extend
    this dataclass with their own fields without breaking the pipeline,
    because the tracker only inspects ``data`` + ``index`` for now.
    """

    data: np.ndarray
    index: int = 0
    timestamp_s: Optional[float] = None
    extras: dict = field(default_factory=dict)


@dataclass
class Intrinsics:
    """Pinhole intrinsics (fx, fy, cx, cy) plus optional distortion.

    Only consumed by future LiDAR / multi-camera sensors; video-only
    pipelines can leave ``intrinsics=None``.
    """

    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    distortion: Optional[np.ndarray] = None  # shape (5,) or (8,) for OpenCV


@dataclass
class Extrinsics:
    """World-from-sensor transform.

    The transform is a 4x4 homogeneous matrix; ``None`` means "identity"
    (i.e. world == sensor).  This is enough for future LiDAR <-> camera
    fusion without changing the tracker contract.
    """

    world_T_sensor: Optional[np.ndarray] = None  # shape (4, 4)


class Sensor(abc.ABC):
    """Abstract base class for any per-tick data source.

    Subclasses must implement ``open``, ``read`` and ``close``.  The
    ``intrinsics`` / ``extrinsics`` properties are optional; video
    pipelines typically leave them as ``None``.
    """

    name: str = "abstract-sensor"

    def __init__(self) -> None:
        self._intrinsics: Optional[Intrinsics] = None
        self._extrinsics: Optional[Extrinsics] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    @abc.abstractmethod
    def open(self) -> None:
        """Acquire the underlying resource (file handle, network stream, ...)."""

    @abc.abstractmethod
    def read(self) -> Optional[Frame]:
        """Return the next :class:`Frame` or ``None`` at EOF / on error."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release the underlying resource."""

    # ------------------------------------------------------------------
    # Context manager sugar
    # ------------------------------------------------------------------
    def __enter__(self) -> "Sensor":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Geometry (optional)
    # ------------------------------------------------------------------
    @property
    def intrinsics(self) -> Optional[Intrinsics]:
        return self._intrinsics

    @intrinsics.setter
    def intrinsics(self, value: Optional[Intrinsics]) -> None:
        self._intrinsics = value

    @property
    def extrinsics(self) -> Optional[Extrinsics]:
        return self._extrinsics

    @extrinsics.setter
    def extrinsics(self, value: Optional[Extrinsics]) -> None:
        self._extrinsics = value


__all__ = [
    "Extrinsics",
    "Frame",
    "Intrinsics",
    "Sensor",
]