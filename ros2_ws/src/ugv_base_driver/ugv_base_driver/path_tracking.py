"""Pure 2-D path tracking helpers for differential-drive vehicles."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrackingCommand:
    linear: float
    angular: float
    closest_index: int
    lookahead_index: int
    goal_reached: bool


def normalize_angle(angle: float) -> float:
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def deduplicate_points(
    points: Iterable[Point2D], *, tolerance: float = 1e-6
) -> Tuple[Point2D, ...]:
    if tolerance < 0.0 or not math.isfinite(tolerance):
        raise ValueError("tolerance must be finite and non-negative")
    result = []
    for point in points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            raise ValueError("path points must be finite")
        if not result or math.hypot(
            point.x - result[-1].x, point.y - result[-1].y
        ) > tolerance:
            result.append(point)
    return tuple(result)


def scale_path_points(
    points: Iterable[Point2D],
    *,
    resolution: float,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
) -> Tuple[Point2D, ...]:
    if not all(math.isfinite(value) for value in (resolution, origin_x, origin_y)):
        raise ValueError("path transform values must be finite")
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    return deduplicate_points(
        Point2D(
            origin_x + point.x * resolution,
            origin_y + point.y * resolution,
        )
        for point in points
    )


def nearest_path_index(
    points: Sequence[Point2D], pose: Pose2D, *, start_index: int = 0
) -> int:
    if not points:
        raise ValueError("path must not be empty")
    if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
        raise ValueError("pose must be finite")
    start = max(0, min(int(start_index), len(points) - 1))
    return min(
        range(start, len(points)),
        key=lambda index: (
            points[index].x - pose.x
        ) ** 2
        + (points[index].y - pose.y) ** 2,
    )


def lookahead_path_index(
    points: Sequence[Point2D], start_index: int, lookahead_distance: float
) -> int:
    if not points:
        raise ValueError("path must not be empty")
    if not math.isfinite(lookahead_distance) or lookahead_distance <= 0.0:
        raise ValueError("lookahead_distance must be positive")
    index = max(0, min(int(start_index), len(points) - 1))
    travelled = 0.0
    for next_index in range(index + 1, len(points)):
        travelled += math.hypot(
            points[next_index].x - points[next_index - 1].x,
            points[next_index].y - points[next_index - 1].y,
        )
        if travelled >= lookahead_distance:
            return next_index
    return len(points) - 1


def pure_pursuit_command(
    points: Sequence[Point2D],
    pose: Pose2D,
    *,
    progress_index: int,
    lookahead_distance: float,
    goal_tolerance: float,
    max_linear_speed: float,
    max_angular_speed: float,
    min_linear_speed: float,
    slowdown_radius: float,
    rotate_in_place_angle: float,
    heading_gain: float,
) -> TrackingCommand:
    if not points:
        raise ValueError("path must not be empty")
    values = (
        lookahead_distance,
        goal_tolerance,
        max_linear_speed,
        max_angular_speed,
        min_linear_speed,
        slowdown_radius,
        rotate_in_place_angle,
        heading_gain,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("tracking parameters must be finite")
    if (
        lookahead_distance <= 0.0
        or goal_tolerance <= 0.0
        or max_linear_speed <= 0.0
        or max_angular_speed <= 0.0
        or slowdown_radius <= 0.0
        or rotate_in_place_angle <= 0.0
        or heading_gain <= 0.0
        or min_linear_speed < 0.0
        or min_linear_speed > max_linear_speed
    ):
        raise ValueError("invalid tracking parameter range")

    closest = nearest_path_index(
        points, pose, start_index=max(0, progress_index - 2)
    )
    goal = points[-1]
    goal_distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
    if goal_distance <= goal_tolerance:
        return TrackingCommand(0.0, 0.0, closest, len(points) - 1, True)

    lookahead = lookahead_path_index(points, closest, lookahead_distance)
    target = points[lookahead]
    dx = target.x - pose.x
    dy = target.y - pose.y
    target_distance = max(math.hypot(dx, dy), 1e-6)
    heading_error = normalize_angle(math.atan2(dy, dx) - pose.yaw)

    if abs(heading_error) >= rotate_in_place_angle:
        angular = max(
            -max_angular_speed,
            min(max_angular_speed, heading_gain * heading_error),
        )
        return TrackingCommand(0.0, angular, closest, lookahead, False)

    goal_scale = min(1.0, goal_distance / slowdown_radius)
    heading_scale = max(0.15, math.cos(heading_error))
    linear = max_linear_speed * goal_scale * heading_scale
    if goal_distance > 2.0 * goal_tolerance:
        linear = max(linear, min_linear_speed)

    curvature = 2.0 * math.sin(heading_error) / target_distance
    angular = curvature * linear
    if abs(angular) > max_angular_speed:
        scale = max_angular_speed / abs(angular)
        linear *= scale
        angular *= scale

    return TrackingCommand(linear, angular, closest, lookahead, False)


def body_velocity_to_world(
    velocity_x: float, velocity_y: float, yaw: float
) -> Tuple[float, float]:
    if not all(math.isfinite(value) for value in (velocity_x, velocity_y, yaw)):
        raise ValueError("velocity transform inputs must be finite")
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * velocity_x - sine * velocity_y,
        sine * velocity_x + cosine * velocity_y,
    )
