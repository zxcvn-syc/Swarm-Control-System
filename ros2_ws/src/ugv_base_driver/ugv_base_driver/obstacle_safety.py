"""Pure obstacle-clearance helpers for the UGV safety gate."""

from __future__ import annotations

import heapq
import math
import struct
from typing import Iterable, Optional, Tuple


def _require_finite(*values: float) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError("safety inputs must be finite")


def confirmed_minimum(
    values: Iterable[float], confirmation_count: int
) -> Optional[float]:
    """Return the Nth-nearest finite sample to reject isolated bad pixels."""
    if confirmation_count <= 0:
        raise ValueError("confirmation_count must be positive")
    valid = [
        float(value)
        for value in values
        if math.isfinite(float(value)) and float(value) >= 0.0
    ]
    if not valid:
        return None
    nearest = heapq.nsmallest(min(confirmation_count, len(valid)), valid)
    return nearest[-1]


def minimum_laser_distance(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    forward_half_angle: float,
    confirmation_count: int = 2,
) -> Optional[float]:
    """Find confirmed clearance in a forward-facing LaserScan sector."""
    _require_finite(
        angle_min,
        angle_increment,
        range_min,
        forward_half_angle,
    )
    if math.isnan(range_max) or range_max <= range_min:
        raise ValueError("laser range bounds are invalid")
    if angle_increment == 0.0:
        raise ValueError("angle_increment must be non-zero")
    if range_min < 0.0:
        raise ValueError("range_min must be non-negative")
    if forward_half_angle <= 0.0 or forward_half_angle > math.pi:
        raise ValueError("forward_half_angle must be in (0, pi]")

    candidates = []
    for index, raw_range in enumerate(ranges):
        angle = angle_min + index * angle_increment
        forward_angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(forward_angle) > forward_half_angle:
            continue
        distance = float(raw_range)
        if math.isinf(distance) and distance > 0.0 and math.isfinite(range_max):
            candidates.append(float(range_max))
        elif math.isfinite(distance) and range_min <= distance <= range_max:
            candidates.append(distance)
    return confirmed_minimum(candidates, confirmation_count)


def minimum_depth_distance(
    data: object,
    *,
    width: int,
    height: int,
    step: int,
    encoding: str,
    is_bigendian: bool,
    roi_x_min: float,
    roi_x_max: float,
    roi_y_min: float,
    roi_y_max: float,
    sample_stride: int,
    range_min: float,
    range_max: float,
    confirmation_count: int = 20,
) -> Optional[float]:
    """Sample a central depth-image ROI and return confirmed Z clearance."""
    if width <= 0 or height <= 0 or step <= 0:
        raise ValueError("depth image dimensions must be positive")
    if sample_stride <= 0:
        raise ValueError("sample_stride must be positive")
    _require_finite(
        roi_x_min,
        roi_x_max,
        roi_y_min,
        roi_y_max,
        range_min,
        range_max,
    )
    if not (0.0 <= roi_x_min < roi_x_max <= 1.0):
        raise ValueError("depth horizontal ROI is invalid")
    if not (0.0 <= roi_y_min < roi_y_max <= 1.0):
        raise ValueError("depth vertical ROI is invalid")
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError("depth range bounds are invalid")

    if encoding == "16UC1":
        bytes_per_pixel = 2
        unpack_code = "H"
        unit_scale = 0.001
    elif encoding == "32FC1":
        bytes_per_pixel = 4
        unpack_code = "f"
        unit_scale = 1.0
    else:
        raise ValueError(
            "unsupported depth encoding '{}'; expected 16UC1 or 32FC1".format(
                encoding
            )
        )
    if step < width * bytes_per_pixel:
        raise ValueError("depth image step is shorter than one pixel row")

    try:
        view = memoryview(data)
    except TypeError:
        view = memoryview(bytes(data))
    if view.format != "B" or view.ndim != 1:
        view = view.cast("B")
    if len(view) < step * height:
        raise ValueError("depth image data is truncated")

    x_start = max(0, min(width - 1, int(math.floor(width * roi_x_min))))
    x_stop = max(x_start + 1, min(width, int(math.ceil(width * roi_x_max))))
    y_start = max(0, min(height - 1, int(math.floor(height * roi_y_min))))
    y_stop = max(y_start + 1, min(height, int(math.ceil(height * roi_y_max))))
    prefix = ">" if is_bigendian else "<"
    unpacker = struct.Struct(prefix + unpack_code)
    candidates = []
    for y in range(y_start, y_stop, sample_stride):
        row_offset = y * step
        for x in range(x_start, x_stop, sample_stride):
            raw_value = unpacker.unpack_from(
                view, row_offset + x * bytes_per_pixel
            )[0]
            distance = float(raw_value) * unit_scale
            if math.isinf(distance) and distance > 0.0:
                candidates.append(range_max)
            elif math.isfinite(distance) and range_min <= distance <= range_max:
                candidates.append(distance)
    return confirmed_minimum(candidates, confirmation_count)


def dynamic_stop_distance(
    base_stop_distance: float,
    forward_speed: float,
    reaction_time: float,
    max_deceleration: float,
) -> float:
    """Compute base margin plus reaction and constant-deceleration distance."""
    _require_finite(
        base_stop_distance, forward_speed, reaction_time, max_deceleration
    )
    if base_stop_distance < 0.0 or reaction_time < 0.0:
        raise ValueError("distance and reaction_time must be non-negative")
    if max_deceleration <= 0.0:
        raise ValueError("max_deceleration must be positive")
    speed = max(0.0, forward_speed)
    return (
        base_stop_distance
        + speed * reaction_time
        + speed * speed / (2.0 * max_deceleration)
    )


def guarded_twist(
    linear: float,
    angular: float,
    clearance: float,
    *,
    base_stop_distance: float,
    slowdown_distance: float,
    reaction_time: float,
    max_deceleration: float,
    allow_reverse_without_rear_sensor: bool,
    allow_rotation_when_blocked: bool,
) -> Tuple[float, float, float, float]:
    """Apply clearance gating and return linear, angular, scale, stop range."""
    _require_finite(
        linear,
        angular,
        base_stop_distance,
        slowdown_distance,
        reaction_time,
        max_deceleration,
    )
    if math.isnan(clearance) or clearance < 0.0:
        raise ValueError("clearance must be non-negative")
    if base_stop_distance < 0.0 or slowdown_distance <= base_stop_distance:
        raise ValueError("slowdown_distance must exceed base_stop_distance")

    if linear < 0.0:
        if allow_reverse_without_rear_sensor:
            return linear, angular, 1.0, base_stop_distance
        return 0.0, 0.0, 0.0, base_stop_distance

    stop_distance = dynamic_stop_distance(
        base_stop_distance,
        linear,
        reaction_time,
        max_deceleration,
    )
    slowdown_start = stop_distance + slowdown_distance - base_stop_distance
    if clearance <= stop_distance:
        safe_angular = angular if allow_rotation_when_blocked else 0.0
        return 0.0, safe_angular, 0.0, stop_distance
    if clearance >= slowdown_start:
        return linear, angular, 1.0, stop_distance

    scale = (clearance - stop_distance) / (slowdown_start - stop_distance)
    return linear * scale, angular * scale, scale, stop_distance
