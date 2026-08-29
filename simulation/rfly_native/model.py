"""Deterministic vehicle and aircraft motion primitives for the Rfly demo."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def magnitude(x_value: float, y_value: float) -> float:
    return math.hypot(x_value, y_value)


@dataclass
class PlanarState:
    x: float
    y: float
    yaw: float = 0.0
    speed: float = 0.0
    altitude: float = 25.0
    vertical_speed: float = 0.0


@dataclass(frozen=True)
class RouteState:
    x: float
    y: float
    yaw: float
    curvature: float


class ClosedSplineRoute:
    """A C1 closed road-like route sampled from a Catmull-Rom spline."""

    def __init__(self, control_points: Sequence[tuple[float, float]], samples_per_segment: int = 30) -> None:
        if len(control_points) < 4:
            raise ValueError("a closed route needs at least four control points")
        if samples_per_segment < 4:
            raise ValueError("samples_per_segment must be at least four")
        self._points = self._sample(control_points, samples_per_segment)
        self._lengths = [0.0]
        for index, point in enumerate(self._points):
            next_point = self._points[(index + 1) % len(self._points)]
            self._lengths.append(self._lengths[-1] + magnitude(next_point[0] - point[0], next_point[1] - point[1]))
        self.length = self._lengths[-1]
        if self.length <= 0.0:
            raise ValueError("route length must be positive")

    @staticmethod
    def _catmull_rom(
        point0: tuple[float, float],
        point1: tuple[float, float],
        point2: tuple[float, float],
        point3: tuple[float, float],
        fraction: float,
    ) -> tuple[float, float]:
        fraction2 = fraction * fraction
        fraction3 = fraction2 * fraction
        x_value = 0.5 * (
            2.0 * point1[0]
            + (-point0[0] + point2[0]) * fraction
            + (2.0 * point0[0] - 5.0 * point1[0] + 4.0 * point2[0] - point3[0]) * fraction2
            + (-point0[0] + 3.0 * point1[0] - 3.0 * point2[0] + point3[0]) * fraction3
        )
        y_value = 0.5 * (
            2.0 * point1[1]
            + (-point0[1] + point2[1]) * fraction
            + (2.0 * point0[1] - 5.0 * point1[1] + 4.0 * point2[1] - point3[1]) * fraction2
            + (-point0[1] + 3.0 * point1[1] - 3.0 * point2[1] + point3[1]) * fraction3
        )
        return x_value, y_value

    @classmethod
    def _sample(cls, control_points: Sequence[tuple[float, float]], samples_per_segment: int) -> list[tuple[float, float]]:
        sampled: list[tuple[float, float]] = []
        point_count = len(control_points)
        for index in range(point_count):
            point0 = control_points[(index - 1) % point_count]
            point1 = control_points[index]
            point2 = control_points[(index + 1) % point_count]
            point3 = control_points[(index + 2) % point_count]
            for sample_index in range(samples_per_segment):
                sampled.append(cls._catmull_rom(point0, point1, point2, point3, sample_index / samples_per_segment))
        return sampled

    def state_at(self, distance: float) -> RouteState:
        normalized_distance = distance % self.length
        index = min(bisect_right(self._lengths, normalized_distance) - 1, len(self._points) - 1)
        segment_length = self._lengths[index + 1] - self._lengths[index]
        fraction = 0.0 if segment_length <= 1e-9 else (normalized_distance - self._lengths[index]) / segment_length
        current = self._points[index]
        following = self._points[(index + 1) % len(self._points)]
        previous = self._points[(index - 1) % len(self._points)]
        after_following = self._points[(index + 2) % len(self._points)]
        x_value = current[0] + (following[0] - current[0]) * fraction
        y_value = current[1] + (following[1] - current[1]) * fraction
        yaw = math.atan2(following[1] - current[1], following[0] - current[0])
        prior_yaw = math.atan2(current[1] - previous[1], current[0] - previous[0])
        next_yaw = math.atan2(after_following[1] - following[1], after_following[0] - following[0])
        span = max(magnitude(following[0] - previous[0], following[1] - previous[1]), 0.1)
        curvature = wrap_angle(next_yaw - prior_yaw) / span
        return RouteState(x_value, y_value, yaw, curvature)

    def polyline(self) -> Iterable[tuple[float, float]]:
        return tuple(self._points)


class RouteVehicle:
    """Acceleration- and lateral-acceleration-limited vehicle on a road route."""

    def __init__(
        self,
        route: ClosedSplineRoute,
        initial_distance: float,
        cruise_speed: float,
        speed_variation: float,
        acceleration_limit: float = 2.6,
        braking_limit: float = 4.2,
        lateral_acceleration_limit: float = 5.5,
        phase: float = 0.0,
    ) -> None:
        self.route = route
        self.distance = initial_distance
        self.cruise_speed = cruise_speed
        self.speed_variation = speed_variation
        self.acceleration_limit = acceleration_limit
        self.braking_limit = braking_limit
        self.lateral_acceleration_limit = lateral_acceleration_limit
        self.phase = phase
        state = route.state_at(initial_distance)
        self.state = PlanarState(state.x, state.y, state.yaw, 0.0)

    def step(self, elapsed: float, delta_time: float) -> PlanarState:
        route_state = self.route.state_at(self.distance)
        speed_request = self.cruise_speed + self.speed_variation * (
            0.58 * math.sin(0.19 * elapsed + self.phase)
            + 0.42 * math.sin(0.067 * elapsed + 1.7 * self.phase)
        )
        curve_speed_limit = math.sqrt(self.lateral_acceleration_limit / max(abs(route_state.curvature), 0.006))
        desired_speed = clamp(speed_request, 4.0, min(16.0, curve_speed_limit))
        speed_delta = desired_speed - self.state.speed
        acceleration = clamp(speed_delta / max(delta_time, 1e-3), -self.braking_limit, self.acceleration_limit)
        self.state.speed = max(0.0, self.state.speed + acceleration * delta_time)
        self.distance = (self.distance + self.state.speed * delta_time) % self.route.length
        route_state = self.route.state_at(self.distance)
        self.state.x = route_state.x
        self.state.y = route_state.y
        self.state.yaw = route_state.yaw
        return self.state


class GuidedGroundVehicle:
    """Smooth point follower used for the grey ground interception vehicles."""

    def __init__(self, initial_state: PlanarState, maximum_speed: float = 7.5) -> None:
        self.state = initial_state
        self.maximum_speed = maximum_speed
        self._velocity_x = initial_state.speed * math.cos(initial_state.yaw)
        self._velocity_y = initial_state.speed * math.sin(initial_state.yaw)

    def step(self, target_x: float, target_y: float, delta_time: float) -> PlanarState:
        offset_x = target_x - self.state.x
        offset_y = target_y - self.state.y
        distance = magnitude(offset_x, offset_y)
        target_speed = clamp(0.58 * distance, 0.0, self.maximum_speed)
        if distance > 0.4:
            desired_velocity_x = target_speed * offset_x / distance
            desired_velocity_y = target_speed * offset_y / distance
        else:
            desired_velocity_x = 0.0
            desired_velocity_y = 0.0
        acceleration_limit = 3.0
        delta_velocity_x = clamp(desired_velocity_x - self._velocity_x, -acceleration_limit * delta_time, acceleration_limit * delta_time)
        delta_velocity_y = clamp(desired_velocity_y - self._velocity_y, -acceleration_limit * delta_time, acceleration_limit * delta_time)
        self._velocity_x += delta_velocity_x
        self._velocity_y += delta_velocity_y
        self.state.x += self._velocity_x * delta_time
        self.state.y += self._velocity_y * delta_time
        self.state.speed = magnitude(self._velocity_x, self._velocity_y)
        if self.state.speed > 0.15:
            desired_yaw = math.atan2(self._velocity_y, self._velocity_x)
            self.state.yaw += clamp(wrap_angle(desired_yaw - self.state.yaw), -1.5 * delta_time, 1.5 * delta_time)
        return self.state


class SmoothUav:
    """Position controller with acceleration, tilt and altitude rate constraints."""

    def __init__(self, initial_state: PlanarState, maximum_speed: float = 18.0, maximum_acceleration: float = 6.0) -> None:
        self.state = initial_state
        self.maximum_speed = maximum_speed
        self.maximum_acceleration = maximum_acceleration
        self._velocity_x = 0.0
        self._velocity_y = 0.0

    def step(
        self,
        target_x: float,
        target_y: float,
        target_altitude: float,
        target_yaw: float,
        delta_time: float,
        wind_x: float = 0.0,
        wind_y: float = 0.0,
    ) -> PlanarState:
        error_x = target_x - self.state.x
        error_y = target_y - self.state.y
        desired_velocity_x = clamp(0.78 * error_x, -self.maximum_speed, self.maximum_speed)
        desired_velocity_y = clamp(0.78 * error_y, -self.maximum_speed, self.maximum_speed)
        desired_speed = magnitude(desired_velocity_x, desired_velocity_y)
        if desired_speed > self.maximum_speed:
            desired_velocity_x *= self.maximum_speed / desired_speed
            desired_velocity_y *= self.maximum_speed / desired_speed
        delta_velocity_x = clamp(desired_velocity_x - self._velocity_x, -self.maximum_acceleration * delta_time, self.maximum_acceleration * delta_time)
        delta_velocity_y = clamp(desired_velocity_y - self._velocity_y, -self.maximum_acceleration * delta_time, self.maximum_acceleration * delta_time)
        self._velocity_x += delta_velocity_x
        self._velocity_y += delta_velocity_y
        self.state.x += (self._velocity_x + wind_x) * delta_time
        self.state.y += (self._velocity_y + wind_y) * delta_time
        altitude_error = target_altitude - self.state.altitude
        self.state.vertical_speed = clamp(0.8 * altitude_error, -2.4, 2.4)
        self.state.altitude = clamp(self.state.altitude + self.state.vertical_speed * delta_time, 18.0, 70.0)
        yaw_delta = wrap_angle(target_yaw - self.state.yaw)
        self.state.yaw += clamp(yaw_delta, -1.4 * delta_time, 1.4 * delta_time)
        self.state.speed = magnitude(self._velocity_x, self._velocity_y)
        return self.state
