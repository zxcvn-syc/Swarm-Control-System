"""coord_transform_node — pixel → world bridge for /target_track.

The perception team publishes ``swarm_interfaces/TargetTrackArray`` on
``/target_track`` with coordinates in **image pixels** (centroid of the
detection bounding box).  The scheduling and enclosure groups need
**world ENU metres** to drive the UAVs.  This node bridges the two.

Pipeline
--------

1. ``pixel_to_ray`` — invert the camera intrinsics matrix ``K`` and
   back-project ``(u, v)`` to a unit ray in the camera optical frame
   (``X`` right, ``Y`` down, ``Z`` forward).
2. ``intersect_ray_with_ground`` — rotate the ray into the world frame
   using the drone's pose (and the camera mount), then intersect with
   the ground plane ``Z_world = ground_altitude`` (a parameter,
   default 0 m).
3. The local pixel velocity is projected through the same ground-plane
   geometry, yielding a world-frame velocity in metres per second.

ROS2 wiring
-----------

The node subscribes to:

* ``/target_track`` (``TargetTrackArray``) — pixel tracks
* ``/camera_info`` (``sensor_msgs/CameraInfo``) — cached on first
  message
* ``/drone_pose`` (``geometry_msgs/PoseStamped``) — most recent UAV
  pose in ENU

and publishes:

* ``/target_track_world`` (``TargetTrackArray``) — world coordinates
* ``/target_track_debug`` (``TargetTrackArray``) — same payload but
  with a distinct ``frame_id`` for inspection

The node refuses to publish world coordinates until it has both a
camera intrinsics matrix and a fresh drone pose — this avoids
emitting ghost coordinates during the first few frames.

The pure math functions (``pixel_to_ray``,
``intersect_ray_with_ground``, ``camera_to_body``,
``body_to_world``, ``rotate_vector``, ``quaternion_to_matrix``,
``euler_to_matrix``) are intentionally rclpy-free so they can be
unit-tested without spinning up a ROS context.  See
``tests/test_coord_transform.py``.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

from swarm_interfaces.msg import TargetTrack, TargetTrackArray


# ---------------------------------------------------------------------------
# Pure math — no rclpy imports.  All public so tests can import them
# directly without going through the node.
# ---------------------------------------------------------------------------

# Minimum vertical component (world frame) of the camera ray before
# we accept the ground intersection.  Anything below this is treated
# as the ray pointing at the horizon and the back-projection is
# rejected.
_GROUND_PLANE_MIN_DZ = 1e-3

# ROS camera optical frames use X right, Y down, Z forward.  The perception
# camera is assumed to be nadir-facing by default: X maps to body X, Y maps
# to body -Y, and the optical axis maps to body -Z.  This is a proper rotation
# (unlike the tempting but invalid X/+X, Y/-Y, Z/+Z reflection).
_NADIR_CAMERA_OPTICAL_TO_BODY = np.diag([1.0, -1.0, -1.0])


def _as_bool(value: object, default: bool = False) -> bool:
    """Coerce ROS parameter values, including launch substitution strings."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off", ""}:
            return False
    return default


def _fixed_float_list(value: Sequence[float] | None, size: int) -> list[float]:
    """Sanitize a fixed-size ROS numeric array without leaking NaN values."""
    result: list[float] = []
    try:
        values = list(value) if value is not None else []
    except TypeError:
        values = []
    for item in values[:size]:
        try:
            numeric = float(item)
        except (TypeError, ValueError):
            numeric = 0.0
        result.append(numeric if math.isfinite(numeric) else 0.0)
    result.extend([0.0] * (size - len(result)))
    return result


def pixel_to_ray(u: float, v: float, K: np.ndarray) -> Optional[np.ndarray]:
    """Back-project a pixel ``(u, v)`` to a unit ray in the camera frame.

    Returns the **3-vector** ``(Xc, Yc, Zc)`` in the camera optical
    frame that the pinhole through ``(u, v)`` traces.  The vector is
    **not** normalised — its ``Zc`` component equals 1.0 (homogeneous
    trick), so callers that want a unit ray should divide by
    ``np.linalg.norm``.  ``None`` is returned if ``K`` is degenerate.
    """
    try:
        u = float(u)
        v = float(v)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(u) and math.isfinite(v)):
        return None
    K = np.asarray(K, dtype=np.float64)
    if K.shape != (3, 3):
        raise ValueError(f"K must be 3x3, got {K.shape}")
    if not np.all(np.isfinite(K)):
        return None
    try:
        Kinv = np.linalg.inv(K)
    except np.linalg.LinAlgError:
        return None
    ray = Kinv @ np.array([u, v, 1.0], dtype=np.float64)
    if not np.all(np.isfinite(ray)):
        return None
    return ray


def intersect_ray_with_ground(
    ray_cam: np.ndarray,
    R_world_from_cam: np.ndarray,
    camera_world: np.ndarray,
    ground_altitude: float,
) -> Optional[np.ndarray]:
    """Find the world-frame ground intersection of a camera ray.

    Parameters
    ----------
    ray_cam : (3,) ndarray
        Direction vector of the ray in the camera optical frame
        (output of :func:`pixel_to_ray`).
    R_world_from_cam : (3, 3) ndarray
        Rotation that maps a camera-frame vector into the world frame.
    camera_world : (3,) ndarray
        World-frame position of the camera centre.
    ground_altitude : float
        World-frame Z of the ground plane (ENU, metres).

    Returns
    -------
    (3,) ndarray or None
        World-frame point where the ray hits the ground plane, or
        ``None`` if the ray is parallel to the ground or points
        upward (would not intersect a plane below the camera).
    """
    ray_world = np.asarray(R_world_from_cam, dtype=np.float64) @ np.asarray(
        ray_cam, dtype=np.float64
    ).reshape(3)
    cam_w = np.asarray(camera_world, dtype=np.float64).reshape(3)

    dz = float(ray_world[2])
    if abs(dz) < _GROUND_PLANE_MIN_DZ:
        return None
    t = (float(ground_altitude) - float(cam_w[2])) / dz
    if t <= 0.0:
        return None
    return cam_w + t * ray_world


def project_pixel_to_ground(
    u: float,
    v: float,
    K: np.ndarray,
    R_world_from_cam: np.ndarray,
    camera_world: np.ndarray,
    ground_altitude: float,
) -> Optional[np.ndarray]:
    """Project one pixel to the configured world-frame ground plane."""
    ray_cam = pixel_to_ray(u, v, K)
    if ray_cam is None:
        return None
    return intersect_ray_with_ground(
        ray_cam, R_world_from_cam, camera_world, ground_altitude,
    )


def pixel_velocity_to_ground_velocity(
    u: float,
    v: float,
    vx_pixels_s: float,
    vy_pixels_s: float,
    K: np.ndarray,
    R_world_from_cam: np.ndarray,
    camera_world: np.ndarray,
    ground_altitude: float,
) -> Optional[np.ndarray]:
    """Convert an image-plane velocity to world metres per second.

    ``TargetTrack.vx`` and ``vy`` are Kalman velocities in pixels/second.
    Rotating them directly produces values with the wrong units.  Projecting
    the one-second displaced pixel correctly captures camera height and focal
    length while keeping the transform deterministic for a static pose.
    """
    try:
        vx = float(vx_pixels_s)
        vy = float(vy_pixels_s)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(vx) and math.isfinite(vy)):
        return None
    start = project_pixel_to_ground(
        u, v, K, R_world_from_cam, camera_world, ground_altitude,
    )
    end = project_pixel_to_ground(
        float(u) + vx, float(v) + vy,
        K, R_world_from_cam, camera_world, ground_altitude,
    )
    if start is None or end is None:
        return None
    velocity = end - start
    return velocity if np.all(np.isfinite(velocity)) else None


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert a unit quaternion to a 3x3 rotation matrix.

    Uses the standard Hamilton convention consistent with
    ``tf2::transformations.quaternion_matrix`` and
    ``geometry_msgs/Quaternion`` (x, y, z, w).  The result is the
    rotation that maps a vector expressed in the **child** frame into
    the **parent** frame, matching ROS tf2 semantics.
    """
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build a 3x3 rotation matrix from ZYX (yaw-pitch-roll) Euler angles.

    Matches the ROS REP-103 convention used for the camera mount
    transform: rotate first by ``roll`` around X, then ``pitch``
    around Y, then ``yaw`` around Z.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def camera_to_body(
    Xc: float, Yc: float, Zc: float,
    mount_roll: float, mount_pitch: float, mount_yaw: float,
) -> np.ndarray:
    """Apply the camera_optical → base_link mount rotation.

    The default mount is nadir-facing: ``camera_optical.X`` aligns with
    ``base_link.X``, ``camera_optical.Y`` with ``base_link.-Y``, and the
    optical axis points along ``base_link.-Z``.  The three ``mount_*``
    parameters are body-frame offsets for a physically mounted camera.
    """
    R_offset = euler_to_matrix(mount_roll, mount_pitch, mount_yaw)
    return R_offset @ _NADIR_CAMERA_OPTICAL_TO_BODY @ np.array(
        [Xc, Yc, Zc], dtype=np.float64,
    )


def body_to_world(
    Xb: float, Yb: float, Zb: float,
    pose_translation: Sequence[float],
    pose_rotation: np.ndarray,
) -> np.ndarray:
    """Transform a base_link vector into world ENU coordinates.

    ``pose_translation`` is the (3,) drone position in ENU metres and
    ``pose_rotation`` is the 3x3 rotation that maps a base_link vector
    into the world frame (consistent with ``quaternion_to_matrix``).
    """
    t = np.asarray(pose_translation, dtype=np.float64).reshape(3)
    R = np.asarray(pose_rotation, dtype=np.float64).reshape(3, 3)
    return R @ np.array([Xb, Yb, Zb], dtype=np.float64) + t


def rotate_vector(v, R) -> np.ndarray:
    """Apply a 3x3 rotation to a 3-vector (no translation)."""
    return np.asarray(R, dtype=np.float64) @ np.asarray(v, dtype=np.float64).reshape(3)


def rotate_covariance_2x2(sigma_pixels: np.ndarray, R_world_from_cam: np.ndarray) -> np.ndarray:
    """Rotate a 2x2 image-plane covariance into the world frame.

    The 2D image covariance is embedded in the horizontal plane of the
    camera optical frame (Xc, Yc), rotated by the upper-left 2x2 block
    of the full 3D rotation, then projected back to 2D.  This is an
    approximation that ignores the depth-axis contribution, but it is
    the standard treatment for monocular 2D trackers that emit a 2x2
    covariance in pixel space.
    """
    sigma_pixels = np.asarray(sigma_pixels, dtype=np.float64).reshape(2, 2)
    R = np.asarray(R_world_from_cam, dtype=np.float64).reshape(3, 3)
    R_xy = R[:2, :2]
    return R_xy @ sigma_pixels @ R_xy.T


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class CoordTransformNode(Node):
    """Bridge node: pixel-TrackArray → world-TrackArray."""

    def __init__(self) -> None:
        super().__init__('coord_transform_node')

        # Parameters ----------------------------------------------------
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('drone_pose_topic', '/drone_pose')
        self.declare_parameter('input_topic', '/target_track')
        self.declare_parameter('output_topic', '/target_track_world')
        self.declare_parameter('debug_topic', '/target_track_debug')
        self.declare_parameter('enabled', True)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('ground_altitude', 0.0)
        self.declare_parameter('max_pose_age_s', 0.5)
        self.declare_parameter('camera_mount_roll', 0.0)
        self.declare_parameter('camera_mount_pitch', 0.0)
        self.declare_parameter('camera_mount_yaw', 0.0)
        self.declare_parameter('frame_id', '')
        self.declare_parameter('publish_debug', True)

        camera_info_topic = self.get_parameter('camera_info_topic').value
        drone_pose_topic = self.get_parameter('drone_pose_topic').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        debug_topic = self.get_parameter('debug_topic').value

        self._enabled = _as_bool(self.get_parameter('enabled').value, True)
        self._ground_altitude = float(self.get_parameter('ground_altitude').value)
        self._max_pose_age_s = max(
            0.0, float(self.get_parameter('max_pose_age_s').value),
        )
        self._mount_roll = float(self.get_parameter('camera_mount_roll').value)
        self._mount_pitch = float(self.get_parameter('camera_mount_pitch').value)
        self._mount_yaw = float(self.get_parameter('camera_mount_yaw').value)
        world_frame = str(self.get_parameter('world_frame').value).strip()
        legacy_frame_id = str(self.get_parameter('frame_id').value).strip()
        self._frame_id = world_frame or legacy_frame_id or 'world'
        self._publish_debug = _as_bool(self.get_parameter('publish_debug').value)
        if not all(math.isfinite(value) for value in (
            self._ground_altitude, self._mount_roll,
            self._mount_pitch, self._mount_yaw,
        )):
            raise ValueError('ground altitude and camera mount angles must be finite')

        # Cached intrinsics: (3, 3) ndarray.  None until the first
        # ``CameraInfo`` arrives.
        self._K: Optional[np.ndarray] = None
        # Most recent drone pose: (translation, rotation, stamp_ns).
        self._pose_translation: Optional[np.ndarray] = None
        self._pose_rotation: Optional[np.ndarray] = None
        self._pose_stamp_ns: Optional[int] = None

        # Subscriptions -------------------------------------------------
        # CameraInfo is treated as one-shot — we only need the
        # intrinsics and they don't change during a flight.  We use
        # RELIABLE depth=1 so the cached message is delivered even if
        # the camera_info publisher starts before us.
        info_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
        )
        self._info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, info_qos,
        )

        drone_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, drone_pose_topic, self._on_drone_pose, drone_qos,
        )

        # Use BEST_EFFORT for the input tracks — the perception node
        # publishes at 10 Hz and dropping a stale frame is acceptable.
        track_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._track_sub = self.create_subscription(
            TargetTrackArray, input_topic, self._on_track, track_qos,
        )

        # Publishers ----------------------------------------------------
        out_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(TargetTrackArray, output_topic, out_qos)
        self._debug_publisher = None
        if self._publish_debug:
            self._debug_publisher = self.create_publisher(
                TargetTrackArray, debug_topic, out_qos,
            )

        self.get_logger().info(
            f'coord_transform_node ready: '
            f'input={input_topic} -> output={output_topic} '
            f'camera_info={camera_info_topic} drone_pose={drone_pose_topic} '
            f'ground_altitude={self._ground_altitude} '
            f'mount(rpy)=({self._mount_roll:.3f}, {self._mount_pitch:.3f}, {self._mount_yaw:.3f})'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        try:
            K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'ignoring invalid CameraInfo.k: {exc}')
            return
        if (
            not np.all(np.isfinite(K))
            or K[0, 0] <= 0.0
            or K[1, 1] <= 0.0
            or abs(float(np.linalg.det(K))) < 1e-12
        ):
            self.get_logger().error('ignoring invalid camera intrinsics matrix')
            return
        if self._K is None:
            self.get_logger().info(
                f'cached camera intrinsics from {msg.header.frame_id!r} '
                f'(fx={K[0, 0]:.1f}, fy={K[1, 1]:.1f}, '
                f'cx={K[0, 2]:.1f}, cy={K[1, 2]:.1f})'
            )
        self._K = K

    def _on_drone_pose(self, msg: PoseStamped) -> None:
        t = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=np.float64)
        q = msg.pose.orientation
        values = [*t, q.x, q.y, q.z, q.w]
        if not np.all(np.isfinite(values)) or np.linalg.norm(values[3:]) < 1e-9:
            self.get_logger().warn('ignoring drone pose with non-finite position or orientation')
            return
        R = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        self._pose_translation = t
        self._pose_rotation = R
        self._pose_stamp_ns = stamp_ns

    def _pose_is_fresh(self, now_ns: int) -> bool:
        if self._pose_stamp_ns is None or self._pose_translation is None:
            return False
        if self._max_pose_age_s <= 0:
            return True
        age_ns = now_ns - self._pose_stamp_ns
        max_age_ns = int(self._max_pose_age_s * 1e9)
        return -max_age_ns <= age_ns <= max_age_ns

    def _on_track(self, msg: TargetTrackArray) -> None:
        if not self._enabled:
            return
        if self._K is None:
            self.get_logger().debug('skipping: no camera_info yet')
            return
        if self._pose_translation is None or self._pose_rotation is None:
            self.get_logger().debug('skipping: no drone_pose yet')
            return

        now_ns = self.get_clock().now().nanoseconds
        if not self._pose_is_fresh(now_ns):
            self.get_logger().warn(
                'skipping: drone_pose is stale '
                f'(age={(now_ns - (self._pose_stamp_ns or now_ns)) / 1e9:.3f}s)'
            )
            return

        # Mount rotation: camera_optical -> base_link.  Combined with
        # the drone pose, this gives the full camera_optical -> world.
        R_mount = (
            euler_to_matrix(self._mount_roll, self._mount_pitch, self._mount_yaw)
            @ _NADIR_CAMERA_OPTICAL_TO_BODY
        )
        R_world_from_cam = self._pose_rotation @ R_mount

        out = TargetTrackArray()
        out.header = self._make_header(msg.header.stamp)
        out.frame_idx = msg.frame_idx
        out.tracks = []
        for track in msg.tracks:
            world_track = self._transform_track(track, R_world_from_cam)
            if world_track is not None:
                out.tracks.append(world_track)

        self._publisher.publish(out)
        if self._debug_publisher is not None:
            dbg = TargetTrackArray()
            dbg.header = Header()
            dbg.header.stamp = out.header.stamp
            dbg.header.frame_id = f'{self._frame_id}_debug'
            dbg.frame_idx = out.frame_idx
            dbg.tracks = list(out.tracks)
            self._debug_publisher.publish(dbg)

    def _transform_track(
        self,
        track: TargetTrack,
        R_world_from_cam: np.ndarray,
    ) -> Optional[TargetTrack]:
        # 1) pixel -> camera ray.
        p_world = project_pixel_to_ground(
            track.x, track.y, self._K, R_world_from_cam,
            self._pose_translation, self._ground_altitude,
        )
        if p_world is None:
            return None

        # 3) KF velocity (pixel/sec) -> world m/s through the ground-plane
        # projection.  A 5-pixel image velocity is not a 5-metre velocity.
        v_world = pixel_velocity_to_ground_velocity(
            track.x, track.y, track.vx, track.vy, self._K,
            R_world_from_cam, self._pose_translation, self._ground_altitude,
        )
        if v_world is None:
            return None

        # 4) Predicted future positions: each (px, py) -> world (X, Y).
        pred_x_world: list[float] = []
        pred_y_world: list[float] = []
        pred_x = _fixed_float_list(track.pred_x, 5)
        pred_y = _fixed_float_list(track.pred_y, 5)
        pred_conf = [max(0.0, min(1.0, value)) for value in _fixed_float_list(track.pred_conf, 5)]
        for px, py in zip(pred_x, pred_y):
            p_world_pred = project_pixel_to_ground(
                px, py, self._K, R_world_from_cam,
                self._pose_translation, self._ground_altitude,
            )
            if p_world_pred is None:
                pred_x_world.append(0.0)
                pred_y_world.append(0.0)
                continue
            pred_x_world.append(float(p_world_pred[0]))
            pred_y_world.append(float(p_world_pred[1]))

        out = TargetTrack()
        out.target_id = track.target_id
        out.x = float(p_world[0])
        out.y = float(p_world[1])
        out.vx = float(v_world[0])
        out.vy = float(v_world[1])

        out.confidence = track.confidence
        out.cls = track.cls
        out.is_confirmed = track.is_confirmed
        out.speed = float(math.sqrt(v_world[0] ** 2 + v_world[1] ** 2))
        out.motion_mode = track.motion_mode

        # Ensure exactly 5 predictions.
        while len(pred_x_world) < 5:
            pred_x_world.append(0.0)
            pred_y_world.append(0.0)
        out.pred_x = pred_x_world[:5]
        out.pred_y = pred_y_world[:5]
        out.pred_conf = pred_conf[:5]
        return out

    def _make_header(self, stamp=None) -> Header:
        h = Header()
        if stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0):
            h.stamp = stamp
        else:
            h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self._frame_id
        return h


def main(args: Optional[list] = None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = CoordTransformNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
