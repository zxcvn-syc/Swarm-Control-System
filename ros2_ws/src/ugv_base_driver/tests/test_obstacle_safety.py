import math
import struct

import pytest

from ugv_base_driver.obstacle_safety import (
    dynamic_stop_distance,
    guarded_twist,
    minimum_depth_distance,
    minimum_laser_distance,
)


def test_laser_uses_only_forward_sector_and_confirms_nearest_range():
    distance = minimum_laser_distance(
        [0.2, 3.0, 0.7, 0.8, 5.0],
        angle_min=-2.0,
        angle_increment=1.0,
        range_min=0.1,
        range_max=6.0,
        forward_half_angle=1.1,
        confirmation_count=2,
    )
    assert distance == pytest.approx(0.8)


def test_laser_positive_infinity_means_clear_to_sensor_maximum():
    distance = minimum_laser_distance(
        [math.inf, math.inf],
        angle_min=-0.1,
        angle_increment=0.2,
        range_min=0.1,
        range_max=12.0,
        forward_half_angle=0.5,
    )
    assert distance == pytest.approx(12.0)


def test_depth_16uc1_uses_millimetres_and_confirmation_count():
    values = [2000, 2000, 500, 2000, 2000, 2000, 600, 2000]
    data = struct.pack("<8H", *values)
    distance = minimum_depth_distance(
        data,
        width=4,
        height=2,
        step=8,
        encoding="16UC1",
        is_bigendian=False,
        roi_x_min=0.0,
        roi_x_max=1.0,
        roi_y_min=0.0,
        roi_y_max=1.0,
        sample_stride=1,
        range_min=0.1,
        range_max=8.0,
        confirmation_count=2,
    )
    assert distance == pytest.approx(0.6)


def test_depth_32fc1_honours_big_endian_and_row_padding():
    row = struct.pack(">2f", 2.0, 0.75) + b"PAD!"
    distance = minimum_depth_distance(
        row,
        width=2,
        height=1,
        step=12,
        encoding="32FC1",
        is_bigendian=True,
        roi_x_min=0.0,
        roi_x_max=1.0,
        roi_y_min=0.0,
        roi_y_max=1.0,
        sample_stride=1,
        range_min=0.1,
        range_max=8.0,
        confirmation_count=1,
    )
    assert distance == pytest.approx(0.75)


def test_depth_rejects_unsupported_encoding():
    with pytest.raises(ValueError):
        minimum_depth_distance(
            b"\x00",
            width=1,
            height=1,
            step=1,
            encoding="8UC1",
            is_bigendian=False,
            roi_x_min=0.0,
            roi_x_max=1.0,
            roi_y_min=0.0,
            roi_y_max=1.0,
            sample_stride=1,
            range_min=0.1,
            range_max=8.0,
        )


def test_dynamic_stop_distance_includes_reaction_and_braking():
    assert dynamic_stop_distance(0.5, 1.0, 0.2, 1.0) == pytest.approx(1.2)


def test_guard_stops_forward_motion_inside_dynamic_stop_range():
    linear, angular, scale, stop_distance = guarded_twist(
        1.0,
        0.3,
        1.1,
        base_stop_distance=0.5,
        slowdown_distance=1.5,
        reaction_time=0.2,
        max_deceleration=1.0,
        allow_reverse_without_rear_sensor=False,
        allow_rotation_when_blocked=False,
    )
    assert stop_distance == pytest.approx(1.2)
    assert (linear, angular, scale) == (0.0, 0.0, 0.0)


def test_guard_scales_twist_together_in_slowdown_zone():
    linear, angular, scale, _ = guarded_twist(
        0.5,
        0.4,
        1.2,
        base_stop_distance=0.5,
        slowdown_distance=1.5,
        reaction_time=0.0,
        max_deceleration=1.0,
        allow_reverse_without_rear_sensor=False,
        allow_rotation_when_blocked=False,
    )
    assert 0.0 < scale < 1.0
    assert linear == pytest.approx(0.5 * scale)
    assert angular == pytest.approx(0.4 * scale)


def test_reverse_is_blocked_without_rear_sensor():
    result = guarded_twist(
        -0.2,
        0.0,
        5.0,
        base_stop_distance=0.5,
        slowdown_distance=1.5,
        reaction_time=0.2,
        max_deceleration=1.0,
        allow_reverse_without_rear_sensor=False,
        allow_rotation_when_blocked=False,
    )
    assert result[:3] == (0.0, 0.0, 0.0)
