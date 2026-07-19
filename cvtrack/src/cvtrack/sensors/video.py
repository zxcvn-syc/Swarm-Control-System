"""Video sensor implementation: wraps :class:`cvtrack.io.VideoReader`.

This is the v6 default sensor and the basis for the future multi-camera
rig -- just compose multiple ``VideoSensor`` instances, each pointing at
a different file / RTSP URL, and feed them into the tracker.
"""

from __future__ import annotations

from typing import Optional

from cvtrack.sensors.base import Frame, Sensor


class VideoSensor(Sensor):
    """A :class:`Sensor` backed by a cv2-based video file or stream."""

    name = "video"

    def __init__(self, path: str) -> None:
        super().__init__()
        self.path = path
        # lazy: cv2.VideoCapture isn't fully ready before open()
        self._reader = None
        self._width = 0
        self._height = 0
        self._fps = 0.0

    # ------------------------------------------------------------------
    # Sensor protocol
    # ------------------------------------------------------------------
    def open(self) -> None:
        from cvtrack.io import VideoReader  # local import to avoid cycles

        self._reader = VideoReader(self.path)
        info = self._reader.info()
        self._width = info.width
        self._height = info.height
        self._fps = info.fps

    def read(self) -> Optional[Frame]:
        if self._reader is None:
            raise RuntimeError("VideoSensor.read() called before open()")
        ok, img = self._reader.read()
        if not ok or img is None:
            return None
        idx = int(self._reader.cap.get(1))  # 0-based frame counter
        timestamp = idx / max(self._fps, 1e-6)
        return Frame(data=img, index=idx, timestamp_s=timestamp)

    def close(self) -> None:
        if self._reader is not None:
            self._reader.close()
            self._reader = None

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------
    @property
    def width(self) -> int:
        return int(self._width)

    @property
    def height(self) -> int:
        return int(self._height)

    @property
    def fps(self) -> float:
        return float(self._fps)

    def set_pos(self, frame_idx: int) -> None:
        if self._reader is not None:
            self._reader.set_pos(frame_idx)


__all__ = ["VideoSensor"]