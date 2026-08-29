"""Thread-safe state shared by the ROS executor and dashboard HTTP server."""

from __future__ import annotations

import ipaddress
import threading
import time
from typing import Any


SAFETY_STATE_NAMES = {
    0: "LOCKED",
    1: "MANUAL_READY",
    2: "AUTO_READY",
    3: "ACTIVE",
    4: "FAULT",
    5: "EMERGENCY_HOLD",
}
ACTIVATION_MODE_NAMES = {0: "NONE", 1: "MANUAL", 2: "AUTO"}


def is_loopback_host(host: str) -> bool:
    """Return whether an HTTP bind host is constrained to the local machine."""

    value = str(host).strip().strip("[]")
    if value.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(value).is_loopback
    except ValueError:
        return False


class DashboardState:
    """Store the latest status and JPEG frame without retaining image history."""

    def __init__(self, max_video_bytes: int) -> None:
        self._max_video_bytes = max(int(max_video_bytes), 1024)
        self._lock = threading.RLock()
        self._frame_changed = threading.Condition(self._lock)
        self._status: dict[str, Any] = {"available": False}
        self._status_updated_at: float | None = None
        self._frame: bytes | None = None
        self._frame_sequence = 0
        self._frame_updated_at: float | None = None

    def update_status(self, status: dict[str, Any]) -> None:
        with self._lock:
            self._status = {"available": True, **status}
            self._status_updated_at = time.monotonic()

    def update_frame(self, data: bytes) -> bool:
        """Keep one bounded JPEG frame and wake any waiting MJPEG clients."""

        frame = bytes(data)
        if (
            len(frame) < 4
            or len(frame) > self._max_video_bytes
            or not frame.startswith(b"\xff\xd8")
            or not frame.endswith(b"\xff\xd9")
        ):
            return False
        with self._frame_changed:
            self._frame = frame
            self._frame_sequence += 1
            self._frame_updated_at = time.monotonic()
            self._frame_changed.notify_all()
        return True

    def status_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            result = dict(self._status)
            result["status_age_seconds"] = self._age(self._status_updated_at, now)
            result["video"] = {
                "available": self._frame is not None,
                "age_seconds": self._age(self._frame_updated_at, now),
                "sequence": self._frame_sequence,
            }
            return result

    def wait_for_frame(self, after_sequence: int, timeout: float) -> tuple[int, bytes | None]:
        """Wait for a newer JPEG frame, returning no frame on timeout."""

        deadline = time.monotonic() + max(float(timeout), 0.0)
        with self._frame_changed:
            while self._frame_sequence <= int(after_sequence):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return self._frame_sequence, None
                self._frame_changed.wait(remaining)
            return self._frame_sequence, self._frame

    def notify_waiters(self) -> None:
        with self._frame_changed:
            self._frame_changed.notify_all()

    @staticmethod
    def _age(timestamp: float | None, now: float) -> float | None:
        return None if timestamp is None else max(0.0, now - timestamp)
