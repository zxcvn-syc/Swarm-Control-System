"""Unit tests for differential-drive kinematics (no ROS dependencies).

Run locally with:  python -m pytest tests/test_diff_kinematics.py
"""

import math

import pytest

from ugv_base_driver.diff_kinematics import (
    body_twist_to_wheel_angular_speeds,
    clamp,
    configure_wheel_directions,
    limit_body_twist,
    limit_twist_rate,
    limit_wheel_angular_speeds,
    wheel_speeds_from_twist,
    wheel_angular_speeds_to_rpms,
)

WB = 0.4
WR = 0.075


def test_straight_line_equal_wheel_speeds():
    left, right = body_twist_to_wheel_angular_speeds(0.5, 0.0, WB, WR)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.5 / WR)
    assert right == pytest.approx(0.5 / WR)


def test_pure_rotation_opposite_wheel_speeds():
    # omega=1, WB=0.4 -> wheel tangential speed = +-(1.0 * 0.4 / 2) = +-0.2 m/s
    left, right = body_twist_to_wheel_angular_speeds(0.0, 1.0, WB, WR)
    assert left == pytest.approx(-(0.5 * WB) / WR)
    assert right == pytest.approx(+(0.5 * WB) / WR)
    assert left == pytest.approx(-right)


def test_positive_angular_turns_left():
    left, right = body_twist_to_wheel_angular_speeds(0.5, 0.5, WB, WR)
    assert right > left


def test_invalid_geometry_raises():
    with pytest.raises(ValueError):
        body_twist_to_wheel_angular_speeds(0.5, 0.0, 0.0, WR)
    with pytest.raises(ValueError):
        body_twist_to_wheel_angular_speeds(0.5, 0.0, WB, 0.0)


def test_clamp():
    assert clamp(2.0, 1.0) == 1.0
    assert clamp(-2.0, 1.0) == -1.0
    assert clamp(0.5, 1.0) == 0.5
    with pytest.raises(ValueError):
        clamp(0.0, -1.0)


def test_limit_body_twist_within_limits_unchanged():
    v, w = limit_body_twist(0.5, 0.3, 1.0, 1.0)
    assert v == pytest.approx(0.5)
    assert w == pytest.approx(0.3)


def test_limit_body_twist_scales_together():
    v, w = limit_body_twist(2.0, 0.5, 1.0, 1.0)
    assert v == pytest.approx(1.0)
    assert w == pytest.approx(0.25)


def test_limit_body_twist_angular_dominant():
    v, w = limit_body_twist(0.5, 2.0, 1.0, 1.0)
    assert w == pytest.approx(1.0)
    assert v == pytest.approx(0.25)


def test_limit_body_twist_rejects_bad_limits():
    with pytest.raises(ValueError):
        limit_body_twist(0.1, 0.1, 0.0, 1.0)
    with pytest.raises(ValueError):
        limit_body_twist(0.1, 0.1, 1.0, 0.0)


def test_wheel_speeds_from_twist_pipeline():
    left, right = wheel_speeds_from_twist(0.5, 0.0, WB, WR, 1.0, 1.0)
    assert left == pytest.approx(0.5 / WR)
    assert right == pytest.approx(0.5 / WR)


def test_wheel_speeds_from_twist_applies_limits():
    left, right = wheel_speeds_from_twist(2.0, 0.0, WB, WR, 1.0, 1.0)
    assert left == pytest.approx(1.0 / WR)
    assert right == pytest.approx(1.0 / WR)


def test_wheel_speed_limit_preserves_ratio():
    left, right = limit_wheel_angular_speeds(20.0, 10.0, 8.0)
    assert left == pytest.approx(8.0)
    assert right == pytest.approx(4.0)


def test_wheel_speed_limit_in_pipeline():
    left, right = wheel_speeds_from_twist(
        1.0, 1.0, WB, WR, 2.0, 2.0, max_wheel_angular_speed=10.0
    )
    assert max(abs(left), abs(right)) == pytest.approx(10.0)


def test_twist_rate_limit():
    linear, angular = limit_twist_rate(0.0, 0.0, 1.0, -2.0, 0.5, 1.0, 0.2)
    assert linear == pytest.approx(0.1)
    assert angular == pytest.approx(-0.2)


def test_wheel_direction_configuration():
    assert configure_wheel_directions(
        2.0, 3.0, swap_wheels=True, left_sign=-1, right_sign=1
    ) == pytest.approx((-3.0, 2.0))


def test_non_finite_inputs_are_rejected():
    with pytest.raises(ValueError):
        limit_body_twist(float("nan"), 0.0, 1.0, 1.0)
    with pytest.raises(ValueError):
        body_twist_to_wheel_angular_speeds(0.0, float("inf"), WB, WR)


def test_rad_s_to_rpm_conversion():
    left_rpm, right_rpm = wheel_angular_speeds_to_rpms(2 * math.pi, -2 * math.pi)
    assert left_rpm == pytest.approx(60.0)
    assert right_rpm == pytest.approx(-60.0)
