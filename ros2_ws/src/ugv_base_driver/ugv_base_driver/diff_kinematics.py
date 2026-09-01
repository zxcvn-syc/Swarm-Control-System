"""Differential-drive kinematics as pure functions (no ROS dependencies).

These helpers convert a body twist (linear velocity v along x, angular
velocity omega around z) into per-wheel angular speeds, apply symmetric
speed limits, and are unit-tested without rclpy so they can be validated
on a development machine without ROS installed.

Conventions (REP-103):
    +x forward, +z up, positive omega turns counter-clockwise (left).
    Left wheel slows down and right wheel speeds up for positive omega.

All public functions are side-effect free.
"""

from __future__ import annotations

from typing import Tuple


def body_twist_to_wheel_angular_speeds(
    linear: float, angular: float, wheel_base: float, wheel_radius: float
) -> Tuple[float, float]:
    """Convert body twist to (left, right) wheel angular speeds in rad/s.

    v_left  = v - omega * wheel_base / 2
    v_right = v + omega * wheel_base / 2
    omega_wheel = v_wheel / wheel_radius
    """
    if wheel_base <= 0.0:
        raise ValueError("wheel_base must be positive")
    if wheel_radius <= 0.0:
        raise ValueError("wheel_radius must be positive")
    half_track = wheel_base / 2.0
    v_left = linear - angular * half_track
    v_right = linear + angular * half_track
    return v_left / wheel_radius, v_right / wheel_radius


def wheel_angular_speeds_to_rpms(
    left_rad_s: float, right_rad_s: float
) -> Tuple[float, float]:
    """Convert wheel angular speeds from rad/s to RPM for firmware that
    expects revolutions per minute."""
    rad_per_sec_to_rpm = 60.0 / (2.0 * 3.141592653589793)
    return left_rad_s * rad_per_sec_to_rpm, right_rad_s * rad_per_sec_to_rpm


def clamp(value: float, limit: float) -> float:
    """Clamp value into [-limit, +limit]."""
    if limit < 0.0:
        raise ValueError("limit must be non-negative")
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def limit_body_twist(
    linear: float, angular: float, max_linear: float, max_angular: float
) -> Tuple[float, float]:
    """Limit a body twist, scaling both components together so the motion
    arc is preserved when one component exceeds its limit.

    Keeps the ratio v/omega unchanged: if linear exceeds max_linear both
    components are scaled by max_linear/linear (and likewise for angular),
    which never amplifies the smaller violator.
    """
    if max_linear <= 0.0 or max_angular <= 0.0:
        raise ValueError("limits must be positive")
    scale = 1.0
    if abs(linear) > max_linear:
        scale = min(scale, max_linear / abs(linear))
    if abs(angular) > max_angular:
        scale = min(scale, max_angular / abs(angular))
    return linear * scale, angular * scale


def wheel_speeds_from_twist(
    linear: float,
    angular: float,
    wheel_base: float,
    wheel_radius: float,
    max_linear: float,
    max_angular: float,
) -> Tuple[float, float]:
    """Convenience pipeline: limit the twist, then convert to wheel speeds."""
    limited_linear, limited_angular = limit_body_twist(
        linear, angular, max_linear, max_angular
    )
    return body_twist_to_wheel_angular_speeds(
        limited_linear, limited_angular, wheel_base, wheel_radius
    )
