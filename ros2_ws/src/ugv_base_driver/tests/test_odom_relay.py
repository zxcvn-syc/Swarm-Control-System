"""Unit tests for the odometry relay's pure logic (no rclpy required)."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ugv_base_driver.odom_relay import as_bool, odom_pose_fields  # noqa: E402


class _Stamp:
    def __init__(self, sec: int, nanosec: int) -> None:
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, frame_id: str) -> None:
        self.stamp = _Stamp(12, 34)
        self.frame_id = frame_id


class _Point:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


class _Quaternion:
    def __init__(self, x: float, y: float, z: float, w: float) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class _Pose:
    def __init__(self, position: _Point, orientation: _Quaternion) -> None:
        self.position = position
        self.orientation = orientation


class _PoseWithCov:
    def __init__(self, pose: _Pose) -> None:
        self.pose = pose


class _Odom:
    def __init__(self, frame_id: str, position: _Point, orientation: _Quaternion) -> None:
        self.header = _Header(frame_id)
        self.pose = _PoseWithCov(_Pose(position, orientation))


def test_fields_passthrough():
    msg = _Odom("odom", _Point(6.65, 2.43, 0.0), _Quaternion(0.0, 0.0, 0.19, 0.98))
    sec, ns, frame, pos, quat = odom_pose_fields(msg)
    assert (sec, ns) == (12, 34)
    assert frame == "odom"
    assert (pos.x, pos.y) == (6.65, 2.43)
    assert (quat.z, quat.w) == (0.19, 0.98)


def test_nonfinite_position_raises():
    msg = _Odom("odom", _Point(math.nan, 0.0, 0.0), _Quaternion(0.0, 0.0, 0.0, 1.0))
    try:
        odom_pose_fields(msg)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for NaN position")


def test_nonfinite_orientation_raises():
    msg = _Odom("odom", _Point(0.0, 0.0, 0.0), _Quaternion(0.0, math.inf, 0.0, 1.0))
    try:
        odom_pose_fields(msg)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for inf orientation")


def test_zero_quaternion_is_finite():
    # All-zero quaternion is degenerate (yaw undefined) but finite; the
    # relay passes it through and pure_pursuit handles it downstream.
    msg = _Odom("odom", _Point(0.0, 0.0, 0.0), _Quaternion(0.0, 0.0, 0.0, 0.0))
    _sec, _ns, _frame, _pos, quat = odom_pose_fields(msg)
    assert (quat.x, quat.y, quat.z, quat.w) == (0.0, 0.0, 0.0, 0.0)


def test_as_bool_variants():
    assert as_bool(True) is True
    assert as_bool(False) is False
    assert as_bool(1) is True
    assert as_bool(0) is False
    assert as_bool("true") is True
    assert as_bool("TRUE ") is True
    assert as_bool("off") is False
    assert as_bool("") is False
    assert as_bool(None) is False
    assert as_bool("garbage", default=True) is True
