"""Unit tests for the pure-pursuit waypoint follower (no ROS needed)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ugv_base_driver.pure_pursuit import (  # noqa: E402
    closest_waypoint_index,
    dedupe_waypoints,
    extract_waypoints,
    lookahead_target,
    normalize_angle,
    paths_equal,
    pure_pursuit_command,
    quaternion_to_yaw,
)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

class _Header:
    def __init__(self, frame_id=""):
        self.frame_id = frame_id


class _Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _Pose:
    def __init__(self, x, y):
        self.position = _Position(x, y)


class _PoseStamped:
    def __init__(self, x, y, frame_id=""):
        self.header = _Header(frame_id)
        self.pose = _Pose(x, y)


def straight_path(length=6.0, step=0.5):
    """Waypoints along +X from the origin."""
    n = int(length / step) + 1
    return [(i * step, 0.0) for i in range(n)]


# ---------------------------------------------------------------------------
# angle / quaternion helpers
# ---------------------------------------------------------------------------

def test_normalize_angle_wraps():
    assert normalize_angle(0.0) == pytest.approx(0.0)
    assert normalize_angle(math.pi) == pytest.approx(math.pi)
    # -pi and 3*pi both wrap onto +/-pi, which are the same angle
    assert abs(abs(normalize_angle(-math.pi)) - math.pi) < 1e-9
    assert abs(abs(normalize_angle(3 * math.pi)) - math.pi) < 1e-9
    assert normalize_angle(2 * math.pi + 0.1) == pytest.approx(0.1)


def test_quaternion_to_yaw_identity():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_quaternion_to_yaw_quarter_turn():
    yaw = quaternion_to_yaw(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    assert yaw == pytest.approx(math.pi / 2)


def test_quaternion_to_yaw_degenerate_returns_zero():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 0.0) == 0.0


# ---------------------------------------------------------------------------
# waypoint extraction / bookkeeping
# ---------------------------------------------------------------------------

def test_extract_waypoints_frame_filter():
    poses = [
        _PoseStamped(0.0, 0.0, "drone_3"),
        _PoseStamped(1.0, 0.0, "drone_4"),
        _PoseStamped(2.0, 0.0, "drone_4"),
        _PoseStamped(9.0, 9.0, "drone_3"),
    ]
    assert extract_waypoints(poses, "drone_4") == [(1.0, 0.0), (2.0, 0.0)]
    assert extract_waypoints(poses, "") == [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (9.0, 9.0)]


def test_extract_waypoints_skips_garbage():
    class Broken:
        pass

    poses = [_PoseStamped(1.0, 2.0, "drone_4"), Broken()]
    assert extract_waypoints(poses, "drone_4") == [(1.0, 2.0)]
    assert extract_waypoints([], "drone_4") == []


def test_dedupe_waypoints():
    wp = [(0.0, 0.0), (0.0, 0.0), (0.5, 0.0), (0.5000005, 0.0), (1.0, 0.0)]
    assert dedupe_waypoints(wp) == [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]


def test_paths_equal_fuzzy():
    a = [(0.0, 0.0), (1.0, 0.0)]
    b = [(1e-9, 1e-9), (1.0, 1e-9)]
    c = [(0.0, 0.0), (2.0, 0.0)]
    assert paths_equal(a, b)
    assert not paths_equal(a, c)
    assert not paths_equal(a, a + [(2.0, 0.0)])


# ---------------------------------------------------------------------------
# closest index / lookahead
# ---------------------------------------------------------------------------

def test_closest_index_basic():
    wp = straight_path()
    assert closest_waypoint_index(wp, 1.2, 0.0) == 2  # (1.0, 0.0)
    assert closest_waypoint_index(wp, 5.9, 0.1) == len(wp) - 1


def test_closest_index_forward_only():
    # robot near waypoint 4 but search starts at 5: must NOT go back
    wp = straight_path()
    assert closest_waypoint_index(wp, 2.0, 0.0, start_index=5) == 5


def test_closest_index_empty():
    assert closest_waypoint_index([], 0.0, 0.0) == -1


def test_lookahead_target_reaches_distance():
    wp = straight_path()
    (tx, ty), idx = lookahead_target(wp, 0, 0.0, 0.0, 0.6)
    # waypoints sit on a 0.5 m grid: the target is the first one at or
    # beyond the lookahead distance (no interpolation)
    assert math.hypot(tx, ty) >= 0.6
    assert math.hypot(tx, ty) <= 0.6 + 0.5 + 1e-9
    assert idx >= 1


def test_lookahead_target_clamps_to_last_waypoint():
    wp = straight_path(length=2.0, step=0.5)
    (tx, ty), idx = lookahead_target(wp, 0, 0.0, 0.0, 10.0)
    assert (tx, ty) == wp[-1]
    assert idx == len(wp) - 1


# ---------------------------------------------------------------------------
# pure pursuit command
# ---------------------------------------------------------------------------

def test_drive_straight_ahead():
    cmd = pure_pursuit_command(0.0, 0.0, 0.0, straight_path(), lookahead_distance=0.6)
    assert cmd.mode == "DRIVE"
    assert cmd.v == pytest.approx(0.5)  # max_linear_speed default
    assert cmd.omega == pytest.approx(0.0, abs=1e-9)
    assert not cmd.done


def test_turn_left_has_positive_omega():
    # path bends to the left (+Y) of the robot
    wp = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0)]
    cmd = pure_pursuit_command(0.0, 0.0, 0.0, wp, lookahead_distance=1.5)
    assert cmd.mode in {"DRIVE", "ROTATE"}
    assert cmd.omega > 0.0


def test_turn_right_has_negative_omega():
    wp = [(0.0, 0.0), (1.0, 0.0), (1.0, -1.0), (1.0, -2.0)]
    cmd = pure_pursuit_command(0.0, 0.0, 0.0, wp, lookahead_distance=1.5)
    assert cmd.omega < 0.0


def test_target_behind_robot_rotates_in_place():
    # robot at origin facing +X, path is entirely behind it along -X
    wp = [(-4.0, 0.0), (-5.0, 0.0)]
    cmd = pure_pursuit_command(0.0, 0.0, 0.0, wp, lookahead_distance=0.6)
    assert cmd.mode == "ROTATE"
    assert cmd.v == 0.0
    assert cmd.omega != 0.0


def test_goal_reached_stops():
    wp = straight_path(length=6.0)
    cmd = pure_pursuit_command(5.9, 0.0, 0.0, wp, goal_tolerance=0.25)
    assert cmd.done
    assert cmd.mode == "DONE"
    assert cmd.v == 0.0
    assert cmd.omega == 0.0


def test_speed_slows_near_goal():
    wp = straight_path(length=6.0)
    far = pure_pursuit_command(0.0, 0.0, 0.0, wp, max_linear_speed=0.5)
    near = pure_pursuit_command(5.3, 0.0, 0.0, wp, max_linear_speed=0.5)
    assert far.v == pytest.approx(0.5)
    # 0.7 m from the goal, inside the 1.0 m slowdown radius -> ramped down
    assert near.v == pytest.approx(0.35, abs=0.05)
    assert near.v < far.v


def test_omega_clamped():
    # very aggressive turn demand with tiny lookahead
    wp = [(0.0, 0.0), (0.2, 0.2), (0.2, 1.5)]
    cmd = pure_pursuit_command(0.0, 0.0, 0.0, wp, lookahead_distance=0.2,
                               max_linear_speed=1.0, max_angular_speed=0.3)
    assert cmd.omega <= 0.3 + 1e-9


def test_empty_path_is_idle():
    cmd = pure_pursuit_command(0.0, 0.0, 0.0, [])
    assert cmd.mode == "IDLE"
    assert cmd.v == 0.0 and cmd.omega == 0.0


def test_invalid_parameters_raise():
    with pytest.raises(ValueError):
        pure_pursuit_command(0.0, 0.0, 0.0, straight_path(), lookahead_distance=0.0)


def test_index_advances_monotonically():
    """Simulated run along the path: the pursued index never decreases."""
    wp = straight_path(length=8.0, step=0.25)
    x, y, yaw = 0.0, 0.0, 0.0
    last_index = 0
    for _ in range(220):  # 22 s at 0.5 m/s covers the 8 m path
        cmd = pure_pursuit_command(
            x, y, yaw, wp, last_index=last_index, lookahead_distance=0.5,
            max_linear_speed=0.5,
        )
        assert cmd.target_index >= last_index
        last_index = cmd.target_index
        # crude kinematic integration to advance the "robot"
        x += cmd.v * math.cos(yaw) * 0.1
        y += cmd.v * math.sin(yaw) * 0.1
        yaw += cmd.omega * 0.1
        if cmd.done:
            break
    assert math.hypot(x - wp[-1][0], y - wp[-1][1]) < 0.5


def test_simulation_reaches_goal():
    """Closed-loop sanity: the follower drives the point-robot to the goal."""
    wp = [(0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (4.0, 3.0)]
    x, y, yaw = 0.0, 0.0, 0.0
    last_index = 0
    reached = False
    for _ in range(2000):
        cmd = pure_pursuit_command(
            x, y, yaw, wp, last_index=last_index, lookahead_distance=0.5,
            max_linear_speed=0.6, max_angular_speed=1.2, goal_tolerance=0.25,
        )
        last_index = max(last_index, cmd.target_index)
        if cmd.done:
            reached = True
            break
        dt = 0.05
        x += cmd.v * math.cos(yaw) * dt
        y += cmd.v * math.sin(yaw) * dt
        yaw += cmd.omega * dt
    assert reached, f"did not reach goal, ended at ({x:.2f}, {y:.2f})"
