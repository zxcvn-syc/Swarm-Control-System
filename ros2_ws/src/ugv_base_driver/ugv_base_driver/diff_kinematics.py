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

import math
from typing import Tuple


def _require_finite(*values: float) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError("kinematic inputs must be finite")


def body_twist_to_wheel_angular_speeds(
    linear: float, angular: float, wheel_base: float, wheel_radius: float
) -> Tuple[float, float]:
    """Convert body twist to (left, right) wheel angular speeds in rad/s.

    v_left  = v - omega * wheel_base / 2
    v_right = v + omega * wheel_base / 2
    omega_wheel = v_wheel / wheel_radius
    """
    _require_finite(linear, angular, wheel_base, wheel_radius)
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
    _require_finite(left_rad_s, right_rad_s)
    rad_per_sec_to_rpm = 60.0 / (2.0 * math.pi)
    return left_rad_s * rad_per_sec_to_rpm, right_rad_s * rad_per_sec_to_rpm


def clamp(value: float, limit: float) -> float:
    """Clamp value into [-limit, +limit]."""
    _require_finite(value, limit)
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
    _require_finite(linear, angular, max_linear, max_angular)
    if max_linear <= 0.0 or max_angular <= 0.0:
        raise ValueError("limits must be positive")
    scale = 1.0
    if abs(linear) > max_linear:
        scale = min(scale, max_linear / abs(linear))
    if abs(angular) > max_angular:
        scale = min(scale, max_angular / abs(angular))
    return linear * scale, angular * scale


def limit_twist_rate(
    current_linear: float,
    current_angular: float,
    target_linear: float,
    target_angular: float,
    max_linear_accel: float,
    max_angular_accel: float,
    dt: float,
) -> Tuple[float, float]:
    """Apply independent linear/angular acceleration limits.

    The helper is deliberately stateless so the ROS node can keep timing and
    fault handling separate from the kinematics. A zero ``dt`` leaves the
    current command unchanged.
    """
    _require_finite(
        current_linear,
        current_angular,
        target_linear,
        target_angular,
        max_linear_accel,
        max_angular_accel,
        dt,
    )
    if max_linear_accel <= 0.0 or max_angular_accel <= 0.0:
        raise ValueError("acceleration limits must be positive")
    if dt < 0.0:
        raise ValueError("dt must be non-negative")
    if dt == 0.0:
        return current_linear, current_angular

    linear_step = max_linear_accel * dt
    angular_step = max_angular_accel * dt
    next_linear = current_linear + clamp(
        target_linear - current_linear, linear_step
    )
    next_angular = current_angular + clamp(
        target_angular - current_angular, angular_step
    )
    return next_linear, next_angular


def limit_wheel_angular_speeds(
    left_rad_s: float, right_rad_s: float, max_abs_rad_s: float
) -> Tuple[float, float]:
    """Scale both wheel speeds together to preserve curvature."""
    _require_finite(left_rad_s, right_rad_s, max_abs_rad_s)
    if max_abs_rad_s <= 0.0:
        raise ValueError("max_abs_rad_s must be positive")
    peak = max(abs(left_rad_s), abs(right_rad_s))
    if peak <= max_abs_rad_s:
        return left_rad_s, right_rad_s
    scale = max_abs_rad_s / peak
    return left_rad_s * scale, right_rad_s * scale


def configure_wheel_directions(
    left_rad_s: float,
    right_rad_s: float,
    *,
    swap_wheels: bool = False,
    left_sign: int = 1,
    right_sign: int = 1,
) -> Tuple[float, float]:
    """Apply wiring-specific wheel swap and direction signs."""
    _require_finite(left_rad_s, right_rad_s)
    if left_sign not in (-1, 1) or right_sign not in (-1, 1):
        raise ValueError("wheel signs must be either -1 or 1")
    if swap_wheels:
        left_rad_s, right_rad_s = right_rad_s, left_rad_s
    return left_rad_s * left_sign, right_rad_s * right_sign


def wheel_speeds_from_twist(
    linear: float,
    angular: float,
    wheel_base: float,
    wheel_radius: float,
    max_linear: float,
    max_angular: float,
    max_wheel_angular_speed: float | None = None,
) -> Tuple[float, float]:
    """Convenience pipeline: limit the twist, then convert to wheel speeds."""
    limited_linear, limited_angular = limit_body_twist(
        linear, angular, max_linear, max_angular
    )
    left, right = body_twist_to_wheel_angular_speeds(
        limited_linear, limited_angular, wheel_base, wheel_radius
    )
    if max_wheel_angular_speed is not None:
        left, right = limit_wheel_angular_speeds(
            left, right, max_wheel_angular_speed
        )
    return left, right
