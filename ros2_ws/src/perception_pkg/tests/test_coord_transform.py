"""Tests for the pure-math helpers in ``coord_transform_node``.

The ROS2 node itself needs an rclpy context, so we don't instantiate
it here.  Instead we load the source file directly with stub modules
for ``rclpy`` / ``swarm_interfaces`` so we can exercise the pure
helpers (``pixel_to_ray``, ``intersect_ray_with_ground``,
``body_to_world``, ``quaternion_to_matrix``, ``euler_to_matrix``,
``rotate_vector``) without a live ROS2 install.  This keeps the test
cheap and dependency-free.
"""

from __future__ import annotations

import importlib.util
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Stub the ROS2 dependencies so importing the node module does not
# require a sourced colcon workspace.
# ---------------------------------------------------------------------------
def _install_ros_stubs() -> None:
    """Pre-create stub modules for rclpy / swarm_interfaces / msgs."""
    rclpy = types.ModuleType('rclpy')
    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = type('Node', (), {})
    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.QoSProfile = type('QoSProfile', (), {})
    rclpy_qos.ReliabilityPolicy = type('ReliabilityPolicy', (), {})
    rclpy.exec = types.ModuleType('rclpy.executors')
    rclpy.exec.ExternalShutdownException = type(
        'ExternalShutdownException', (Exception,), {},
    )
    rclpy.init = lambda *a, **kw: None
    rclpy.try_shutdown = lambda: None
    rclpy.spin = lambda *a, **kw: None
    sys.modules.setdefault('rclpy', rclpy)
    sys.modules.setdefault('rclpy.node', rclpy_node)
    sys.modules.setdefault('rclpy.qos', rclpy_qos)
    sys.modules.setdefault('rclpy.executors', rclpy.exec)

    geom = types.ModuleType('geometry_msgs')
    geom_msg = types.ModuleType('geometry_msgs.msg')
    geom_msg.PoseStamped = type('PoseStamped', (), {})
    sys.modules.setdefault('geometry_msgs', geom)
    sys.modules.setdefault('geometry_msgs.msg', geom_msg)

    sensor = types.ModuleType('sensor_msgs')
    sensor_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msg.CameraInfo = type('CameraInfo', (), {})
    sys.modules.setdefault('sensor_msgs', sensor)
    sys.modules.setdefault('sensor_msgs.msg', sensor_msg)

    std = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    std_msg.Header = type('Header', (), {})
    sys.modules.setdefault('std_msgs', std)
    sys.modules.setdefault('std_msgs.msg', std_msg)

    swarm = types.ModuleType('swarm_interfaces')
    swarm_msg = types.ModuleType('swarm_interfaces.msg')
    swarm_msg.TargetTrack = type('TargetTrack', (), {})
    swarm_msg.TargetTrackArray = type('TargetTrackArray', (), {})
    sys.modules.setdefault('swarm_interfaces', swarm)
    sys.modules.setdefault('swarm_interfaces.msg', swarm_msg)


def _load_coord_transform_module():
    """Load ``coord_transform_node.py`` with stubbed ROS deps."""
    _install_ros_stubs()
    src = (
        Path(__file__).resolve().parent.parent
        / 'perception_pkg'
        / 'coord_transform_node.py'
    )
    spec = importlib.util.spec_from_file_location('ct_node_under_test', src)
    assert spec and spec.loader, f'cannot load {src}'
    mod = importlib.util.module_from_spec(spec)
    sys.modules['ct_node_under_test'] = mod
    spec.loader.exec_module(mod)
    return mod


ct = _load_coord_transform_module()


# ---------------------------------------------------------------------------
# 1. pixel_to_ray at the principal point returns a pure forward
#    direction; intersect_ray_with_ground under a no-op camera pose
#    (camera on the ground) lands the ray at the camera origin.
# ---------------------------------------------------------------------------
def test_pixel_to_camera_z0():
    K = np.array([
        [500.0, 0.0, 320.0],
        [0.0, 500.0, 240.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    # Principal point back-projects to (0, 0, 1) — straight forward
    # in the camera frame.
    ray = ct.pixel_to_ray(320.0, 240.0, K)
    assert ray is not None
    assert ray.shape == (3,)
    assert ray[0] == pytest.approx(0.0, abs=1e-12)
    assert ray[1] == pytest.approx(0.0, abs=1e-12)
    assert ray[2] == pytest.approx(1.0, abs=1e-12)

    # Camera sitting on the ground at (0, 0, 0), identity rotation,
    # ground at Z=0.  A pixel slightly to the right of the principal
    # point (u=320+50=370) gives a ray with X > 0.  With the camera
    # on the ground and the ground at Z=0, the ray's intersection
    # with the ground plane is the camera origin itself (t=0), so
    # ``intersect_ray_with_ground`` rejects it as a degenerate case.
    ray_right = ct.pixel_to_ray(370.0, 240.0, K)
    assert ray_right is not None
    assert ray_right[0] > 0.0
    p_degenerate = ct.intersect_ray_with_ground(
        ray_cam=ray_right,
        R_world_from_cam=np.eye(3),
        camera_world=np.array([0.0, 0.0, 0.0]),
        ground_altitude=0.0,
    )
    assert p_degenerate is None, 't=0 intersection should be rejected'

    # Place the camera 10 m above the ground at (5, -3, 10) with a
    # rotation that maps the camera forward axis (Z) to world -Z
    # (i.e. a 180° roll about the X axis).  This simulates a camera
    # pointed straight at the ground.  The principal-point ray then
    # points straight down and hits the ground at the camera's
    # (X, Y) at the configured ground altitude.
    R_straight_down = ct.euler_to_matrix(math.pi, 0.0, 0.0)
    p_pp = ct.intersect_ray_with_ground(
        ray_cam=ray,
        R_world_from_cam=R_straight_down,
        camera_world=np.array([5.0, -3.0, 10.0]),
        ground_altitude=0.0,
    )
    assert p_pp is not None
    assert p_pp[0] == pytest.approx(5.0, abs=1e-9)
    assert p_pp[1] == pytest.approx(-3.0, abs=1e-9)
    assert p_pp[2] == pytest.approx(0.0, abs=1e-9)

    # A level camera 10 m above the ground looking forward (identity
    # rotation): the principal-point ray points at the sky and never
    # hits the ground, so the function must return None.
    p_sky = ct.intersect_ray_with_ground(
        ray_cam=ray,
        R_world_from_cam=np.eye(3),
        camera_world=np.array([0.0, 0.0, 10.0]),
        ground_altitude=0.0,
    )
    assert p_sky is None


# ---------------------------------------------------------------------------
# 2. body_to_world with an identity rotation and a zero translation
#    should leave a vector untouched.  Also verify quaternion_to_matrix
#    with a unit quaternion is the identity and that body_to_world
#    composes correctly with a 90° yaw rotation.
# ---------------------------------------------------------------------------
def test_camera_to_world_identity():
    # identity quaternion
    R = ct.quaternion_to_matrix(0.0, 0.0, 0.0, 1.0)
    assert R.shape == (3, 3)
    assert np.allclose(R, np.eye(3), atol=1e-12)

    # identity body -> world
    p = ct.body_to_world(
        1.5, -2.0, 0.3,
        pose_translation=[0.0, 0.0, 0.0],
        pose_rotation=R,
    )
    assert p.shape == (3,)
    assert p[0] == pytest.approx(1.5, abs=1e-9)
    assert p[1] == pytest.approx(-2.0, abs=1e-9)
    assert p[2] == pytest.approx(0.3, abs=1e-9)

    # pure translation
    p_t = ct.body_to_world(
        0.0, 0.0, 0.0,
        pose_translation=[7.0, 8.0, 9.0],
        pose_rotation=R,
    )
    assert p_t[0] == pytest.approx(7.0, abs=1e-9)
    assert p_t[1] == pytest.approx(8.0, abs=1e-9)
    assert p_t[2] == pytest.approx(9.0, abs=1e-9)

    # 90° yaw about Z (w = cos(45°), z = sin(45°))
    c = math.cos(math.pi / 4)
    s = math.sin(math.pi / 4)
    R_yaw = ct.quaternion_to_matrix(0.0, 0.0, s, c)
    p_yaw = ct.body_to_world(
        1.0, 0.0, 0.0,
        pose_translation=[0.0, 0.0, 0.0],
        pose_rotation=R_yaw,
    )
    # A 90° CCW rotation about world +Z maps body +X to world +Y.
    assert p_yaw[0] == pytest.approx(0.0, abs=1e-9)
    assert p_yaw[1] == pytest.approx(1.0, abs=1e-9)
    assert p_yaw[2] == pytest.approx(0.0, abs=1e-9)

    # rotate_vector is consistent with body_to_world with t=0
    v = ct.rotate_vector(np.array([3.0, 4.0, 0.0]), R_yaw)
    # (3, 4) rotated 90° CCW about Z = (-4, 3).
    assert v[0] == pytest.approx(-4.0, abs=1e-9)
    assert v[1] == pytest.approx(3.0, abs=1e-9)
    assert v[2] == pytest.approx(0.0, abs=1e-9)

    # quaternion_to_matrix with a 180° roll about X (qw=0, qx=1) should
    # equal Rx(π) and produce diag(1, -1, -1).
    R_roll180 = ct.quaternion_to_matrix(1.0, 0.0, 0.0, 0.0)
    expected = np.diag([1.0, -1.0, -1.0])
    assert np.allclose(R_roll180, expected, atol=1e-12)
