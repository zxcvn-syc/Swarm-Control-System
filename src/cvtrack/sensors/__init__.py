"""Sensor abstraction layer.

See :mod:`cvtrack.sensors.base` for the abstract ``Sensor`` class and the
default :class:`cvtrack.sensors.video.VideoSensor` implementation.  The
top-level package re-exports the most common symbols so callers can write
``from cvtrack.sensors import Sensor, VideoSensor``.
"""

from __future__ import annotations

from cvtrack.sensors.base import (
    Extrinsics,
    Frame,
    Intrinsics,
    Sensor,
)
from cvtrack.sensors.video import VideoSensor

__all__ = [
    "Extrinsics",
    "Frame",
    "Intrinsics",
    "Sensor",
    "VideoSensor",
]