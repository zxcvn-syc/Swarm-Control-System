"""pure_pursuit — waypoint-tracking math for a differential-drive UGV.

This module is intentionally free of any rclpy / ROS message imports so
that the control logic can be unit-tested (and simulated) on a plain
Python interpreter — see ``tests/test_pure_pursuit.py``.

Algorithm
---------
Classic *pure pursuit* on the ground plane (all units metres / radians,
ROS ENU convention: +X east, +Y north, yaw CCW positive around +Z):

1. ``closest_waypoint_index`` — find the waypoint nearest to the robot.
   The search is *forward-only* from the previous index so the vehicle
   never re-targets a segment it has already passed (paths that double
   back would otherwise confuse a global search).
2. ``lookahead_target`` — walk forward from that index accumulating
   segment lengths until the arc distance from the robot reaches
   ``lookahead_distance``.  If the remaining path is shorter than the
   lookahead, the final waypoint is the target (this is what brakes the
   vehicle into the goal).
3. ``pure_pursuit_command`` — express the target in the body frame,
   compute the pure-pursuit curvature ``k = 2 * y_local / r^2`` and the
   wheel-speed pair ``(v, omega) = (v, v * k)``.

Behaviour guards baked into the command step:

* large heading error -> rotate in place (a diff-drive robot cannot
  strafe and we do not drive in reverse by default);
* speed is reduced when the goal is inside ``slowdown_radius`` and the
  command is exactly zero once the goal tolerance is met;
* ``omega`` is clamped to ``max_angular_speed``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point2 = Tuple[float, float]


@dataclass(frozen=True)
class PursuitCommand:
    """Output of one pure-pursuit control step."""

    v: float                 # forward speed, m/s (>= 0)
    omega: float             # yaw rate, rad/s (CCW positive)
    target_index: int        # waypoint index currently pursued
    goal_distance: float     # straight-line distance to the last waypoint
    done: bool               # goal reached within tolerance
    mode: str                # "DRIVE" | "ROTATE" | "DONE" | "IDLE"


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi].

    Uses the ``atan2(sin, cos)`` identity — unlike an ``fmod``-based
    wrap it maps ``+pi`` onto ``+pi`` (not ``-pi``) and stays exact at
    the boundaries.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract the planar yaw (rotation about +Z) from a quaternion.

    Uses the standard ROS/REP-103 Hamilton convention.  A degenerate
    quaternion (zero norm) yields 0.0 rather than raising.
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        return 0.0
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def closest_waypoint_index(
    waypoints: Sequence[Point2],
    x: float,
    y: float,
    start_index: int = 0,
) -> int:
    """Index of the waypoint closest to ``(x, y)``, searching forward only.

    ``start_index`` is clamped into ``[0, len-1]``.  An empty waypoint
    list returns ``-1``.
    """
    n = len(waypoints)
    if n == 0:
        return -1
    start = max(0, min(start_index, n - 1))
    best_i = start
    best_d2 = float("inf")
    for i in range(start, n):
        dx = waypoints[i][0] - x
        dy = waypoints[i][1] - y
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def lookahead_target(
    waypoints: Sequence[Point2],
    index: int,
    x: float,
    y: float,
    lookahead_distance: float,
) -> Tuple[Point2, int]:
    """Pure-pursuit target point at least ``lookahead_distance`` ahead.

    Walks from ``index`` toward the end of the path accumulating the
    distance from the robot; returns the first waypoint whose distance
    from the robot reaches ``lookahead_distance``.  When the remainder
    of the path is shorter than the lookahead, the final waypoint is
    returned (braking behaviour).  The second return value is the index
    of the returned point.
    """
    n = len(waypoints)
    if n == 0:
        return (x, y), -1
    i = max(0, min(index, n - 1))
    px, py = waypoints[i]
    d = math.hypot(px - x, py - y)
    while d < lookahead_distance and i + 1 < n:
        nx, ny = waypoints[i + 1]
        d_next = math.hypot(nx - x, ny - y)
        # stop if the next waypoint is not farther (path folding back)
        if d_next <= d and d >= lookahead_distance:
            break
        i += 1
        px, py = nx, ny
        d = d_next
    return (px, py), i


def pure_pursuit_command(
    x: float,
    y: float,
    yaw: float,
    waypoints: Sequence[Point2],
    last_index: int = 0,
    lookahead_distance: float = 0.6,
    max_linear_speed: float = 0.5,
    max_angular_speed: float = 1.2,
    goal_tolerance: float = 0.25,
    slowdown_radius: float = 1.0,
    rotate_in_place_error: float = math.pi / 2,
    min_drive_speed: float = 0.08,
) -> PursuitCommand:
    """One control step of pure pursuit.

    Parameters
    ----------
    x, y, yaw : robot pose in the world frame (metres / radians).
    waypoints : path points in the world frame, ordered start -> goal.
    last_index : waypoint index pursued in the previous tick (monotonic
        progression anchor; pass 0 for a fresh path).

    Returns a :class:`PursuitCommand`; ``waypoints=[]`` yields an IDLE
    zero command rather than an error.
    """
    if not waypoints:
        return PursuitCommand(0.0, 0.0, -1, float("inf"), False, "IDLE")
    if lookahead_distance <= 0.0:
        raise ValueError("lookahead_distance must be positive")
    if max_linear_speed < 0.0 or max_angular_speed < 0.0:
        raise ValueError("speed limits must be non-negative")

    # 1) goal check first: within tolerance -> full stop.
    gx, gy = waypoints[-1]
    goal_distance = math.hypot(gx - x, gy - y)
    if goal_distance <= goal_tolerance:
        return PursuitCommand(
            0.0, 0.0, len(waypoints) - 1, goal_distance, True, "DONE",
        )

    # 2) advance the pursued waypoint (forward-only search).
    index = closest_waypoint_index(waypoints, x, y, last_index)

    # 3) lookahead target in the world frame.
    (tx, ty), target_index = lookahead_target(
        waypoints, index, x, y, lookahead_distance,
    )

    # 4) target in the body frame.
    dx = tx - x
    dy = ty - y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

    target_distance = math.hypot(local_x, local_y)
    if target_distance < 1e-6:
        # already on top of the target: nothing to steer toward
        return PursuitCommand(
            0.0, 0.0, target_index, goal_distance, False, "IDLE",
        )

    heading_error = math.atan2(local_y, local_x)
    if abs(heading_error) > rotate_in_place_error:
        # target is essentially behind/beside us: turn first, drive later
        omega = max_angular_speed if heading_error > 0.0 else -max_angular_speed
        return PursuitCommand(
            0.0, omega, target_index, goal_distance, False, "ROTATE",
        )

    # 5) pure-pursuit curvature -> (v, omega).
    curvature = 2.0 * local_y / (target_distance * target_distance)
    v = max_linear_speed
    # slow down while closing on the goal (braking ramp)
    if goal_distance < slowdown_radius:
        v = max(min_drive_speed, max_linear_speed * goal_distance / slowdown_radius)
    # slow down in tight turns: cap omega by limiting v when curvature is large
    if abs(curvature) > 1e-6:
        v = min(v, max_angular_speed / abs(curvature))
    v = max(0.0, min(v, max_linear_speed))
    omega = v * curvature
    if omega > max_angular_speed:
        omega = max_angular_speed
    elif omega < -max_angular_speed:
        omega = -max_angular_speed

    return PursuitCommand(
        v, omega, target_index, goal_distance, False, "DRIVE",
    )


def extract_waypoints(
    poses: Sequence[object],
    frame_filter: str = "",
) -> List[Point2]:
    """Extract ``(x, y)`` waypoints from a sequence of pose-like objects.

    ``poses`` items must expose ``header.frame_id`` (str) and
    ``pose.position.x/.y`` — i.e. ``nav_msgs/Path.poses`` entries, but
    any duck-typed stand-in works so the function stays testable
    without ROS installed.  When ``frame_filter`` is a non-empty string
    only poses whose ``frame_id`` equals it are kept; this is how the
    follower picks *its own* path out of the shared ``/planned_path``
    message (planner_node tags every pose with ``drone_<id>``).
    """
    filter_ = (frame_filter or "").strip()
    out: List[Point2] = []
    for ps in poses:
        try:
            frame_id = str(ps.header.frame_id).strip()
            pos = ps.pose.position
            px = float(pos.x)
            py = float(pos.y)
        except (AttributeError, TypeError, ValueError):
            continue
        if filter_ and frame_id != filter_:
            continue
        if math.isfinite(px) and math.isfinite(py):
            out.append((px, py))
    return out


def dedupe_waypoints(
    waypoints: Sequence[Point2],
    min_spacing: float = 1e-3,
) -> List[Point2]:
    """Drop consecutive duplicates / near-duplicates from a waypoint list.

    Keeping duplicate points would make the lookahead walk stall on a
    zero-length segment.
    """
    out: List[Point2] = []
    for p in waypoints:
        if out and math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) < min_spacing:
            continue
        out.append((float(p[0]), float(p[1])))
    return out


def paths_equal(a: Sequence[Point2], b: Sequence[Point2], tol: float = 1e-6) -> bool:
    """Fuzzy equality for two waypoint lists (same length, near-equal points)."""
    if len(a) != len(b):
        return False
    return all(
        math.hypot(pa[0] - pb[0], pa[1] - pb[1]) <= tol
        for pa, pb in zip(a, b)
    )
