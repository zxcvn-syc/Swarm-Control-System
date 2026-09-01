import math

import pytest

from ugv_base_driver.path_tracking import (
    Point2D,
    Pose2D,
    body_velocity_to_world,
    deduplicate_points,
    lookahead_path_index,
    normalize_angle,
    pure_pursuit_command,
    scale_path_points,
)


PATH = tuple(Point2D(float(index), 0.0) for index in range(6))


def command(path=PATH, pose=Pose2D(0.0, 0.0, 0.0), progress=0):
    return pure_pursuit_command(
        path,
        pose,
        progress_index=progress,
        lookahead_distance=1.0,
        goal_tolerance=0.2,
        max_linear_speed=0.8,
        max_angular_speed=1.2,
        min_linear_speed=0.1,
        slowdown_radius=1.5,
        rotate_in_place_angle=0.8,
        heading_gain=1.8,
    )


def test_straight_path_moves_forward_without_turning():
    result = command()
    assert result.linear > 0.0
    assert result.angular == pytest.approx(0.0)
    assert not result.goal_reached


def test_large_heading_error_rotates_in_place():
    result = command(pose=Pose2D(0.0, 0.0, math.pi))
    assert result.linear == 0.0
    assert abs(result.angular) == pytest.approx(1.2)


def test_goal_tolerance_stops_vehicle():
    result = command(pose=Pose2D(4.9, 0.0, 0.0), progress=4)
    assert result.goal_reached
    assert result.linear == 0.0
    assert result.angular == 0.0


def test_lookahead_follows_path_arc_length():
    assert lookahead_path_index(PATH, 1, 2.1) == 4


def test_path_transform_and_deduplication():
    points = scale_path_points(
        [Point2D(0.0, 0.0), Point2D(0.0, 0.0), Point2D(2.0, 4.0)],
        resolution=0.5,
        origin_x=10.0,
        origin_y=-2.0,
    )
    assert points == (Point2D(10.0, -2.0), Point2D(11.0, 0.0))
    assert deduplicate_points(points) == points


def test_body_velocity_rotation():
    vx, vy = body_velocity_to_world(1.0, 0.0, math.pi / 2.0)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(1.0)


def test_normalize_angle():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(-math.pi)


def test_invalid_path_rejected():
    with pytest.raises(ValueError):
        command(path=())
