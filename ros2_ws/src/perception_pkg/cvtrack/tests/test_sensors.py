"""Smoke tests for the v6 sensor abstraction layer."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from cvtrack.sensors import (
    Extrinsics,
    Frame,
    Intrinsics,
    Sensor,
    VideoSensor,
)


def _make_synthetic_video(path: Path, n_frames: int = 6) -> None:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, 10.0, (64, 48))
    assert writer.isOpened()
    for i in range(n_frames):
        img = np.full((48, 64, 3), (i * 30) % 255, dtype=np.uint8)
        writer.write(img)
    writer.release()


def test_frame_dataclass_is_immutable_shape():
    arr = np.zeros((10, 10, 3), dtype=np.uint8)
    f = Frame(data=arr, index=7, timestamp_s=0.7, extras={"src": "test"})
    assert f.data is arr
    assert f.index == 7
    assert f.timestamp_s == 0.7
    assert f.extras["src"] == "test"


def test_intrinsics_default_values():
    intr = Intrinsics()
    assert intr.fx == 0.0
    assert intr.cy == 0.0
    assert intr.distortion is None


def test_extrinsics_identity_default():
    ext = Extrinsics()
    assert ext.world_T_sensor is None


def test_video_sensor_reads_frames(tmp_path):
    src = tmp_path / "synth.mp4"
    _make_synthetic_video(src)
    sensor = VideoSensor(str(src))
    with sensor:
        assert sensor.width == 64
        assert sensor.height == 48
        assert sensor.fps > 0.0
        first = sensor.read()
        assert first is not None
        assert first.data.shape == (48, 64, 3)
        # cv2's CAP_PROP_POS_FRAMES points to the *next* frame to read,
        # so after one read it's 1 (not 0).  Either is acceptable for the
        # sensor contract; we just want a non-negative integer.
        assert isinstance(first.index, int)
        assert first.index >= 0
        assert first.timestamp_s is not None and first.timestamp_s >= 0.0
        # Exhaust the rest.
        seen = 1
        while True:
            fr = sensor.read()
            if fr is None:
                break
            seen += 1
            assert fr.data.shape == (48, 64, 3)
        assert seen >= 2  # at least the first plus one more


def test_video_sensor_eof_returns_none(tmp_path):
    src = tmp_path / "tiny.mp4"
    _make_synthetic_video(src, n_frames=2)
    sensor = VideoSensor(str(src))
    sensor.open()
    try:
        # Exhaust frames.
        for _ in range(5):
            sensor.read()
        # Next call returns None at EOF.
        assert sensor.read() is None
    finally:
        sensor.close()


def test_video_sensor_optional_geometry():
    sensor = VideoSensor("/dev/null/does-not-exist.mp4")
    # Even without open(), the optional geometry accessors must not raise.
    assert sensor.intrinsics is None
    assert sensor.extrinsics is None
    sensor.intrinsics = Intrinsics(fx=1.0, fy=1.0, cx=32.0, cy=24.0)
    assert sensor.intrinsics.fx == 1.0


def test_sensor_is_abstract():
    """Sensor is abstract; trying to instantiate it must raise."""
    with pytest.raises(TypeError):
        Sensor()  # type: ignore[abstract]