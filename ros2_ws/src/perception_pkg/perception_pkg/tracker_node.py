"""tracker_node — YOLOv8 + DeepSORT / BoT-SORT bridge into ROS2.

This node owns the perception pipeline that produces the swarm's
target-track output.  Every frame it runs the cvtrack runner (YOLOv8
detector + DeepSORT cascade / BoT-SORT tracker) and republishes the
currently-confirmed tracks as a single
``swarm_interfaces/TargetTrackArray`` message on ``/target_track``.

Two input modes are supported, selected by the ``input_mode``
parameter:

* ``video`` (default) — read frames from a local file or webcam via
  OpenCV ``VideoCapture``.  The source path is taken from the
  ``video_source`` parameter (e.g. ``"/home/.../pexels_aerial_2034115.mp4"``
  or ``"0"`` for the default webcam).  Useful for development and
  headless bring-up.

* ``topic`` — subscribe to a ROS2 ``sensor_msgs/Image`` topic (typically
  ``/camera/image`` from the UAV payload).  Frames are decoded through
  ``cv_bridge``.  Useful when the tracker_node runs alongside the rest
  of the swarm.

In both modes the output message is the same, so downstream nodes
(planner, scheduler) do not need to know which input source is in use.

Parameters
----------

The full set is documented in ``config/tracker_node.yaml``.  Notable
groups:

* ``tracker.kind`` -- ``botsort`` / ``deepsort`` / ``deepsort_cascade``
* ``detector.weights`` -- path to the YOLOv8 weights file
* ``detector.imgsz`` / ``detector.conf`` / ``detector.classes``
* ``publish_rate_hz`` -- cap on the publishing rate (0 = as fast as
  frames arrive; the default ``10`` keeps topics quiet for slow
  CPU pipelines).
* ``frame_id`` -- the ``header.frame_id`` stamped on outgoing messages
  (typically the UAV body frame the camera is rigidly mounted to).
* ``drone_states_topic`` -- the ``swarm_interfaces/DroneStateArray``
  topic subscribed for closed-loop time synchronization (default
  ``/drone_states``).  When a message arrives the cached
  ``header.stamp`` overrides the outgoing ``TargetTrackArray``
  timestamp so the downstream ``enclosure_node`` can reason about
  UAV/perception freshness without clock-skew assumptions.

Notes
-----

* Coordinates in the published ``TargetTrack.x`` / ``TargetTrack.y``
  fields are **pixels** in the source image's coordinate system
  (centroid of the detection bounding box), matching the raw output of
  the cvtrack runner.  Downstream nodes that need world-frame
  coordinates are responsible for the calibration (IPM, homography,
  PnP).
* Track IDs are issued by the chosen tracker (DeepSORT/BoT-SORT) and
  are unique within a single node process.  Multi-camera deployments
  should consider adding a camera-id prefix at the swarm layer.
"""

from __future__ import annotations

import ast
import logging
import math
import os
import sys
import threading
from pathlib import Path
from typing import Any, Optional

import numpy as np

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as ROSImage
    _HAS_CV_BRIDGE = True
except (ImportError, AttributeError) as _cv_bridge_err:
    # cv_bridge is optional in ``video`` mode.  The AttributeError branch
    # catches an ABI mismatch (e.g. cv_bridge built against numpy 1.x but
    # running against numpy 2.x); the node stays usable as long as the
    # user picks ``input_mode:=video``.
    _HAS_CV_BRIDGE = False
    CvBridge = None  # type: ignore[assignment]
    ROSImage = None  # type: ignore[assignment]
    # Stash the error message so __main__ can surface a single-line warning
    # without spamming the full traceback on every launch.
    _CV_BRIDGE_ERROR = repr(_cv_bridge_err)
else:
    _CV_BRIDGE_ERROR = None


def _report_cv_bridge_state() -> None:
    """Print a one-line cv_bridge availability note, if applicable.

    Called from ``main()`` once the logging system is up.  We keep the
    message terse so a broken cv_bridge doesn't fill the launch logs.
    """
    if _CV_BRIDGE_ERROR is None:
        return
    logging.getLogger(__name__).warning(
        'cv_bridge unavailable (%s). input_mode:=topic will be disabled; '
        'input_mode:=video still works.', _CV_BRIDGE_ERROR,
    )

from std_msgs.msg import Header

from swarm_interfaces.msg import TargetTrack, TargetTrackArray
from swarm_interfaces.msg import EnclosureTarget, EnclosureTargetArray
from swarm_interfaces.msg import TargetTrackDebug
from swarm_interfaces.msg import DroneStateArray

import diagnostic_msgs.msg as diag_msgs


log = logging.getLogger(__name__)


def _publish_if_active(publisher: Any, message: Any) -> bool:
    """Publish unless shutdown has invalidated the ROS context."""
    try:
        publisher.publish(message)
        return True
    except Exception:
        if rclpy.ok():
            raise
        return False


def _as_bool(value: Any, default: bool = False) -> bool:
    """Coerce ROS parameter values, including launch substitutions, to bool."""
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {'true', '1', 'yes', 'on'}:
            return True
        if normalized in {'false', '0', 'no', 'off', ''}:
            return False
    return default


def _as_list(value: Any) -> list[Any]:
    """Parse ROS list parameters that may arrive as YAML-like strings."""
    if value is None:
        return []
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return list(value)
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return []
        try:
            parsed = ast.literal_eval(text)
        except (SyntaxError, ValueError):
            parsed = [item.strip() for item in text.split(',') if item.strip()]
        if isinstance(parsed, (list, tuple)):
            return list(parsed)
        return [parsed]
    return [value]


def _class_ids(value: Any) -> list[int]:
    """Convert a detector class parameter into validated integer IDs."""
    result: list[int] = []
    for item in _as_list(value):
        try:
            class_id = int(item)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'invalid detector class ID: {item!r}') from exc
        if class_id < 0:
            raise ValueError(f'detector class ID must be non-negative: {class_id}')
        result.append(class_id)
    return result


def _finite_float(value: Any, default: float = 0.0) -> float:
    """Return a finite float suitable for a ROS numeric field."""
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return default
    return numeric if math.isfinite(numeric) else default


def _fixed_float_list(value: Any, size: int, default: float = 0.0) -> list[float]:
    """Coerce and pad a fixed-size ROS float array."""
    result = [
        _finite_float(item, default)
        for item in _as_list(value)[:size]
    ]
    result.extend([default] * (size - len(result)))
    return result


def _copy_header(header: Optional[Header], frame_id: Optional[str] = None) -> Header:
    """Copy a ROS header without mutating the message that supplied it."""
    copied = Header()
    if header is not None:
        try:
            copied.stamp = header.stamp
        except AttributeError:
            pass
        copied.frame_id = str(getattr(header, 'frame_id', ''))
    if frame_id is not None:
        copied.frame_id = str(frame_id)
    return copied


def _ensure_vendored_cvtrack() -> None:
    """Make the repository's vendored cvtrack importable from a source checkout."""
    vendored_src = Path(__file__).resolve().parents[1] / 'cvtrack' / 'src'
    if vendored_src.is_dir() and str(vendored_src) not in sys.path:
        sys.path.insert(0, str(vendored_src))


# Type hint for the message class used in _make_target_track
TargetTrack = TargetTrack
EnclosureTarget = EnclosureTarget


# ---------------------------------------------------------------------------
# Lightweight metrics recorder
# ---------------------------------------------------------------------------

class _MetricsRecorder:
    """Accumulate per-period statistics inside tracker_node."""

    def __init__(self, period_ms: int) -> None:
        self._period_ms = period_ms
        self._id_switch_count = 0
        self._miss_count = 0
        self._total_updates = 0
        self._convergence_times: list[float] = []
        self._motion_mode_counts = {0: 0, 1: 0, 2: 0, 3: 0}
        self._id_pool: set[int] = set()
        self._seen_ids: set[int] = set()
        self._active_count = 0

    def update(
        self,
        n_active: int,
        motion_modes: list[int],
        track_ids: Optional[list[int]] = None,
    ) -> None:
        self._total_updates += 1
        self._active_count = max(0, int(n_active))
        if track_ids is not None:
            self._id_pool = {int(track_id) for track_id in track_ids}
            self._seen_ids.update(self._id_pool)
        self._id_switch_count = max(0, self._id_switch_count)
        for mm in motion_modes:
            self._motion_mode_counts[mm] = self._motion_mode_counts.get(mm, 0) + 1

    def on_track_lost(self) -> None:
        self._miss_count += 1

    def on_convergence(self, elapsed_ms: float) -> None:
        self._convergence_times.append(elapsed_ms)

    def snapshot(self) -> diag_msgs.KeyValue:
        """Return a DiagnosticArray payload."""
        active = self._active_count
        miss_rate = self._miss_count / max(self._total_updates, 1)
        conv_time = (
            sum(self._convergence_times) / len(self._convergence_times)
            if self._convergence_times else 0.0
        )
        mm_dist = '; '.join(
            f'{k}={v}' for k, v in sorted(self._motion_mode_counts.items())
        )
        kv = diag_msgs.KeyValue()
        kv.key = 'perception/active_tracks'
        kv.value = str(active)
        return kv

    def diagnostic_array(self) -> diag_msgs.DiagnosticArray:
        arr = diag_msgs.DiagnosticArray()
        arr.header.stamp = rclpy.time.Time().to_msg()
        kv_map = {
            'perception/id_switch_count': str(self._id_switch_count),
            'perception/miss_rate': f'{self._miss_count / max(self._total_updates, 1):.4f}',
            'perception/convergence_time_ms': f'{sum(self._convergence_times) / max(len(self._convergence_times), 1):.1f}',
            'perception/active_tracks': str(self._active_count),
            'perception/motion_mode_distribution': '; '.join(
                f'{k}={v}' for k, v in sorted(self._motion_mode_counts.items())
            ),
        }
        for key, value in kv_map.items():
            status = diag_msgs.DiagnosticStatus()
            status.name = key
            status.level = diag_msgs.DiagnosticStatus.OK
            status.message = value
            status.values = [diag_msgs.KeyValue(key=key, value=value)]
            arr.status.append(status)
        return arr


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _declare_parameters(node: Node) -> None:
    """Declare all ROS2 parameters with safe defaults.

    Grouped under the same keys that appear in
    ``config/tracker_node.yaml`` so the YAML can override them by
    name without surprises.
    """
    node.declare_parameter('input_mode', 'video')
    node.declare_parameter('video_source', '')
    node.declare_parameter('image_topic', '/camera/image')
    node.declare_parameter('track_topic', '/target_track')
    node.declare_parameter('frame_id', 'camera_optical_frame')
    node.declare_parameter('publish_rate_hz', 10.0)
    node.declare_parameter('loop_video', False)
    node.declare_parameter('enable_fusion', False)
    # ROS2 Humble infers a bare [] override as BYTE_ARRAY.  Dynamic typing
    # keeps the documented empty-list default valid while accepting the
    # required string[] once sources are supplied by YAML or the CLI.
    source_descriptor = ParameterDescriptor(dynamic_typing=True)
    node.declare_parameter(
        'fusion_sources', [], descriptor=source_descriptor,
    )
    node.declare_parameter(
        'sources', [], descriptor=source_descriptor,
    )

    # Detector.
    # ``backend`` defaults to ``auto`` so an empty ``weights`` parameter
    # gracefully falls back to MOG2 instead of raising.  Users who want
    # explicit YOLO inference should pass ``detector.backend:=yolo``
    # alongside a valid ``detector.weights`` path.
    node.declare_parameter('detector.backend', 'auto')
    node.declare_parameter('detector.weights', '')
    node.declare_parameter('detector.device', 'cpu')
    node.declare_parameter('detector.imgsz', 480)
    node.declare_parameter('detector.conf', 0.15)
    node.declare_parameter('detector.classes', [0, 1, 2, 3, 4, 5, 7, 8])
    node.declare_parameter('detector.min_box_area', 200.0)
    node.declare_parameter('detector.min_conf', 0.0)
    node.declare_parameter('detector.nms_iou', 0.5)

    # Tracker
    node.declare_parameter('tracker.kind', 'deepsort_cascade')
    node.declare_parameter('tracker.dt', 0.05)
    node.declare_parameter('tracker.max_age', 30)
    node.declare_parameter('tracker.n_init', 3)
    node.declare_parameter('tracker.iou_thresh', 0.30)
    node.declare_parameter('tracker.high_conf', 0.35)
    node.declare_parameter('tracker.new_track_conf', 0.20)
    node.declare_parameter('tracker.lost_relink_frames', 30)
    node.declare_parameter('tracker.stationary_prune', True)
    node.declare_parameter('tracker.include_tentative', False)

    # Adaptive Kalman settings.  Declaring these keys is required before
    # ROS2 parameter files can override the values passed through in
    # _build_runner_overrides.
    node.declare_parameter('tracker.kalman.dt', 0.05)
    node.declare_parameter('tracker.kalman.sigma_p', 0.05)
    node.declare_parameter('tracker.kalman.sigma_v', 0.00625)
    node.declare_parameter('tracker.kalman.sigma_m', 0.05)
    node.declare_parameter('tracker.kalman.acceleration_gain', 0.5)
    node.declare_parameter('tracker.kalman.motion_threshold_slow', 2.0)
    node.declare_parameter('tracker.kalman.motion_threshold_fast', 20.0)
    node.declare_parameter('tracker.kalman.base_std_pos', 0.05)
    node.declare_parameter('tracker.kalman.base_std_vel', 0.00625)
    node.declare_parameter('tracker.kalman.base_std_meas', 0.05)
    node.declare_parameter('tracker.kalman.motion_adapt_gain', 0.3)
    node.declare_parameter('tracker.kalman.velocity_limit', 100.0)
    node.declare_parameter('tracker.kalman.innovation_gate', 9.4877)

    node.declare_parameter('trajectory_prediction.enabled', True)
    node.declare_parameter('trajectory_prediction.prediction_steps', 10)
    node.declare_parameter('trajectory_prediction.confidence_decay', 0.9)
    node.declare_parameter('trajectory_prediction.min_confidence', 0.1)

    # Appearance (only used by deepsort_cascade)
    node.declare_parameter('appearance.enabled', False)
    node.declare_parameter('appearance.weights', '')

    # Enclosure group integration
    node.declare_parameter('enclosure.enabled', False)
    node.declare_parameter('enclosure.topic', '/enclosure_targets')
    node.declare_parameter('enclosure.publish_rate_hz', 5.0)
    node.declare_parameter('enclosure.drone_positions', [])

    # Debug / diagnostics topics
    node.declare_parameter('enable_debug_topics', True)
    node.declare_parameter('metrics_period_ms', 1000)

    # UAV state synchronization (P1-C)
    #
    # Subscribing to /drone_states lets tracker_node align the
    # ``header.stamp`` on outgoing ``TargetTrackArray`` with the latest
    # UAV state timestamp, which enclosure_node uses to gate the
    # closed-loop control.  The topic name is exposed as a parameter so
    # a launch file or YAML config can repoint it (e.g. when running in
    # simulation with a different UAV namespace).
    node.declare_parameter('drone_states_topic', '/drone_states')


def _build_runner_overrides(node: Node) -> dict:
    """Translate ROS2 parameters into the cvtrack-runner override dict.

    The returned dict mirrors the cvtrack YAML schema so it can be fed
    straight into :meth:`CvtrackRunner.from_overrides`.  Beyond the
    detector / tracker / appearance sections, the optimized swarm
    configuration also feeds the adaptive tracker's Kalman parameters
    (``tracker.kalman``) and the trajectory-prediction settings
    (``trajectory_prediction``).  Both sub-dicts are required for
    ``tracker.kind = botsort_adaptive`` / ``deepsort_adaptive`` to take
    effect — without them the adaptive trackers fall back to their
    built-in defaults and the optimized configuration is silently
    dropped.
    """
    p = node.get_parameter
    det = {
        'backend': p('detector.backend').value,
        'weights': p('detector.weights').value,
        'device': p('detector.device').value,
        'imgsz': int(p('detector.imgsz').value),
        'conf': float(p('detector.conf').value),
        'classes': _class_ids(p('detector.classes').value),
        'min_box_area': float(p('detector.min_box_area').value),
        'min_conf': float(p('detector.min_conf').value),
        'nms_iou': float(p('detector.nms_iou').value),
    }
    tr = {
        'kind': str(p('tracker.kind').value).strip().lower(),
        'dt': float(p('tracker.dt').value),
        'max_age': int(p('tracker.max_age').value),
        'n_init': int(p('tracker.n_init').value),
        'iou_thresh': float(p('tracker.iou_thresh').value),
        'high_conf': float(p('tracker.high_conf').value),
        'new_track_conf': float(p('tracker.new_track_conf').value),
        'lost_relink_frames': int(p('tracker.lost_relink_frames').value),
        'stationary_prune': _as_bool(p('tracker.stationary_prune').value, True),
        'include_tentative': _as_bool(p('tracker.include_tentative').value),
    }
    appearance_enabled = _as_bool(p('appearance.enabled').value)
    ap = {
        'enabled': appearance_enabled,
    }
    weights = p('appearance.weights').value
    if weights:
        ap['weights'] = weights

    # Adaptive-tracker knobs.  We try to read each one as a ROS2
    # parameter (declarable in config/tracker_node.yaml); when the
    # parameter is missing the consumer gets ``None`` and the
    # optimized YAML's ``tracker.kalman`` section can still take
    # over via the YAML loader path.
    kalman_cfg: dict = {}
    for ros_name, yaml_key in (
        ('tracker.kalman.dt', 'dt'),
        ('tracker.kalman.sigma_p', 'sigma_p'),
        ('tracker.kalman.sigma_v', 'sigma_v'),
        ('tracker.kalman.sigma_m', 'sigma_m'),
        ('tracker.kalman.acceleration_gain', 'acceleration_gain'),
        ('tracker.kalman.motion_threshold_slow', 'motion_threshold_slow'),
        ('tracker.kalman.motion_threshold_fast', 'motion_threshold_fast'),
        ('tracker.kalman.base_std_pos', 'base_std_pos'),
        ('tracker.kalman.base_std_vel', 'base_std_vel'),
        ('tracker.kalman.base_std_meas', 'base_std_meas'),
        ('tracker.kalman.motion_adapt_gain', 'motion_adapt_gain'),
        ('tracker.kalman.velocity_limit', 'velocity_limit'),
        ('tracker.kalman.innovation_gate', 'innovation_gate'),
    ):
        try:
            v = p(ros_name).value
        except Exception:
            v = None
        if v is not None:
            kalman_cfg[yaml_key] = float(v)
    if kalman_cfg:
        tr['kalman'] = kalman_cfg

    # Trajectory-prediction knobs (consumed by adaptive trackers).
    tp_cfg: dict = {}
    for ros_name, yaml_key in (
        ('trajectory_prediction.enabled', 'enabled'),
        ('trajectory_prediction.prediction_steps', 'prediction_steps'),
        ('trajectory_prediction.confidence_decay', 'confidence_decay'),
        ('trajectory_prediction.min_confidence', 'min_confidence'),
    ):
        try:
            v = p(ros_name).value
        except Exception:
            v = None
        if v is not None:
            if yaml_key == 'enabled':
                tp_cfg[yaml_key] = _as_bool(v, True)
            elif yaml_key == 'prediction_steps':
                tp_cfg[yaml_key] = int(v)
            else:
                tp_cfg[yaml_key] = float(v)
    trajectory_prediction = tp_cfg

    overrides = {'detector': det, 'tracker': tr, 'appearance': ap}
    if trajectory_prediction:
        overrides['trajectory_prediction'] = trajectory_prediction
    return overrides


# ---------------------------------------------------------------------------
# Multi-source aggregation
# ---------------------------------------------------------------------------
class MultiSourceAggregator:
    """Buffer multiple ``TargetTrackArray`` sources and publish fused tracks.

    Fusion is opt-in.  If ``enable_fusion`` is false, no source subscriptions
    or timer are created and :meth:`publish_local` directly forwards the
    existing tracker output.  A requested fusion setup that has no valid
    ``fusion_sources`` also falls back to that same single-source path.
    """

    def __init__(
        self,
        node: Node,
        publisher: Any,
        *,
        frame_id: str = 'camera_optical_frame',
    ) -> None:
        self._node = node
        self._publisher = publisher
        self._frame_id = frame_id
        self._enabled = False
        self._fusion = None
        self._subscriptions: list[Any] = []
        self._timer: Optional[Any] = None
        self._lock = threading.Lock()
        self._pending: dict[str, TargetTrackArray] = {}
        self._last_header: Optional[Header] = None
        self._frame_seq = 0
        requested = _as_bool(node.get_parameter('enable_fusion').value)
        configured_sources = _as_list(node.get_parameter('fusion_sources').value)
        if not configured_sources:
            configured_sources = _as_list(node.get_parameter('sources').value)
        sources = []
        for raw_source in configured_sources:
            source = str(raw_source).strip().strip('/')
            if source and source not in sources:
                sources.append(source)

        if not requested:
            return
        if not sources:
            node.get_logger().warning(
                'enable_fusion=true but fusion_sources is empty; '
                'using direct single-source output'
            )
            return

        try:
            from cvtrack.tracker.fusion import TrackFusion

            self._fusion = TrackFusion()
            for source in sources:
                self._fusion.register_source(source)
                topic = f'/{source}/target_track'
                subscription = node.create_subscription(
                    TargetTrackArray,
                    topic,
                    lambda msg, source=source: self._source_callback(source, msg),
                    QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.RELIABLE,
                    ),
                )
                self._subscriptions.append(subscription)
            self._timer = node.create_timer(0.05, self._fusion_tick)
        except Exception as exc:
            self._fusion = None
            node.get_logger().error(
                f'failed to initialize TrackFusion: {exc}; '
                'using direct single-source output'
            )
            return

        self._enabled = True
        topics = ', '.join(f'/{source}/target_track' for source in sources)
        node.get_logger().info(
            f'multi-source fusion enabled: inputs=[{topics}] output=/target_track'
        )

    @property
    def enabled(self) -> bool:
        return self._enabled

    def publish_local(self, msg: TargetTrackArray) -> None:
        """Forward local tracker output only while fusion is disabled."""
        if not self._enabled:
            try:
                self._publisher.publish(msg)
            except Exception:
                # SIGINT can invalidate the rclpy context between a timer
                # callback and publish(). Do not hide genuine publish errors
                # while the node is otherwise still active.
                if rclpy.ok():
                    raise

    def _source_callback(self, source: str, msg: TargetTrackArray) -> None:
        with self._lock:
            self._pending[source] = msg
            self._last_header = _copy_header(msg.header)

    def _fusion_tick(self) -> None:
        if not self._enabled or self._fusion is None:
            return
        with self._lock:
            pending = self._pending
            self._pending = {}
            header = self._last_header
        if not pending:
            return

        try:
            for source, source_msg in pending.items():
                tracks = [
                    self._track_from_message(track)
                    for track in source_msg.tracks
                ]
                self._fusion.update(source, tracks)
            fused_tracks = self._fusion.fused_tracks()
        except Exception as exc:
            self._node.get_logger().error(f'trajectory fusion tick failed: {exc}')
            return

        output = TargetTrackArray()
        if header is None:
            header = Header()
            header.stamp = self._node.get_clock().now().to_msg()
        output.header = _copy_header(header, self._frame_id)
        output.frame_idx = self._frame_seq
        self._frame_seq += 1
        output.tracks = [self._message_from_track(track) for track in fused_tracks]
        self._publisher.publish(output)

    @staticmethod
    def _track_from_message(msg: TargetTrack):
        from cvtrack.types import Box, Track

        confidence = float(getattr(msg, 'confidence', 1.0))
        x = float(msg.x)
        y = float(msg.y)
        half_size = 10.0
        mean = np.array(
            [x, y, float(msg.vx), float(msg.vy)], dtype=np.float64
        )
        variance = max(1.0, (1.0 - max(0.0, min(1.0, confidence))) * 100.0)
        covariance = np.diag([variance, variance, variance, variance])
        track = Track(
            track_id=int(msg.target_id),
            label=str(getattr(msg, 'cls', 0)),
            mean=mean,
            cov=covariance,
            box=Box(
                x - half_size,
                y - half_size,
                x + half_size,
                y + half_size,
                confidence,
                int(getattr(msg, 'cls', 0)),
                str(getattr(msg, 'cls', 0)),
            ),
            confirmed=bool(getattr(msg, 'is_confirmed', True)),
        )
        motion_modes = {1: 'stationary', 2: 'slow', 3: 'fast'}
        track.motion_mode = motion_modes.get(
            int(getattr(msg, 'motion_mode', 0)), 'unknown'
        )
        pred_x = list(getattr(msg, 'pred_x', []))
        pred_y = list(getattr(msg, 'pred_y', []))
        pred_conf = list(getattr(msg, 'pred_conf', []))
        track.predicted_future = [
            (float(px), float(py), 0.0, 0.0)
            for px, py in zip(pred_x, pred_y)
        ]
        track.prediction_confidence = (
            float(pred_conf[0]) if pred_conf else confidence
        )
        setattr(track, '_fusion_pred_conf', pred_conf)
        return track

    @staticmethod
    def _message_from_track(track) -> TargetTrack:
        msg = TargetTrack()
        msg.target_id = int(track.track_id)
        msg.x = float(track.pos[0])
        msg.y = float(track.pos[1])
        msg.vx = float(track.mean[2]) if track.mean.size > 2 else 0.0
        msg.vy = float(track.mean[3]) if track.mean.size > 3 else 0.0
        msg.confidence = float(track.box.score)
        msg.cls = int(track.box.cls)
        msg.is_confirmed = bool(track.confirmed)
        msg.speed = float(np.hypot(msg.vx, msg.vy))
        motion_modes = {'stationary': 1, 'slow': 2, 'fast': 3}
        msg.motion_mode = motion_modes.get(track.motion_mode, 0)

        future = list(track.predicted_future[:5])
        pred_x = [float(item[0]) for item in future]
        pred_y = [float(item[1]) for item in future]
        stored_conf = list(getattr(track, '_fusion_pred_conf', []))[:5]
        pred_conf = [float(value) for value in stored_conf]
        while len(pred_x) < 5:
            pred_x.append(0.0)
            pred_y.append(0.0)
        while len(pred_conf) < 5:
            pred_conf.append(0.0)
        msg.pred_x = pred_x
        msg.pred_y = pred_y
        msg.pred_conf = pred_conf
        return msg


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class TrackerNode(Node):
    """Perception node: YOLOv8 + DeepSORT / BoT-SORT -> TargetTrackArray."""

    def __init__(self) -> None:
        super().__init__('tracker_node')
        _declare_parameters(self)

        # Lazy import so the message generation step doesn't depend on cvtrack.
        # Vendored cvtrack (this ROS2 package's own ``cvtrack/`` subdir) is
        # the single source of truth at runtime — we deliberately do **not**
        # fall back to ``/home/hhh/Downloads/cv_tracking_demo/src`` here.
        # That independent checkout is kept for reference / benchmarking
        # only; if its copy of ``cvtrack.runner`` ever wins the import race
        # the optimized fields (``pred_x`` / ``motion_mode`` / adaptive
        # tracker branches) get silently dropped because the reference tree
        # pre-dates them.  If the import below fails the operator will see
        # a clear warning explaining the install step, rather than us
        # silently inserting a half-broken path.
        _ensure_vendored_cvtrack()
        try:
            from cvtrack.runner import CvtrackRunner  # noqa: F401
        except ImportError as exc:
            self.get_logger().warning(
                'cvtrack is not importable: %s. Either run `pip install -e '
                '`ros2_ws/src/perception_pkg/cvtrack`` or export '
                'PYTHONPATH=<that dir>/src. The reference checkout at '
                '/home/hhh/Downloads/cv_tracking_demo is intentionally '
                'not used as a fallback (it pre-dates the optimized '
                'tracker code).' % exc
            )
            raise RuntimeError(
                'cvtrack is unavailable; install the vendored package with '
                '`pip install -e ros2_ws/src/perception_pkg/cvtrack` or '
                'add its src directory to PYTHONPATH'
            ) from exc

        overrides = _build_runner_overrides(self)
        # ``tracker.dt`` is part of the nested override, so the runner builds
        # its Kalman transition matrix with the requested value.
        self._dt = _finite_float(
            self.get_parameter('tracker.dt').value,
            default=0.05,
        )
        if self._dt <= 0.0:
            raise ValueError('tracker.dt must be a positive finite value')
        self._runner = CvtrackRunner.from_overrides(
            preset=None, overrides=overrides, fps=1.0 / max(self._dt, 1e-3)
        )

        self._track_topic = self.get_parameter('track_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._publish_rate = max(
            0.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self._input_mode = str(self.get_parameter('input_mode').value).strip().lower()
        self._loop_video = _as_bool(self.get_parameter('loop_video').value)

        self._publisher = self.create_publisher(
            TargetTrackArray, self._track_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._aggregator = MultiSourceAggregator(
            self,
            self._publisher,
            frame_id=self._frame_id,
        )

        # --- Debug / diagnostics publishers --------------------------------
        self._enable_debug = _as_bool(
            self.get_parameter('enable_debug_topics').value,
        )
        self._debug_pub: Optional[Any] = None
        self._metrics_pub: Optional[Any] = None
        self._metrics_recorder: Optional[_MetricsRecorder] = None

        if self._enable_debug:
            self._debug_pub = self.create_publisher(
                TargetTrackDebug, '/target_track_debug',
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            )
            self._metrics_pub = self.create_publisher(
                diag_msgs.DiagnosticArray, '/tracking_metrics',
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
            metrics_period_ms = int(self.get_parameter('metrics_period_ms').value)
            self._metrics_recorder = _MetricsRecorder(
                period_ms=max(0, metrics_period_ms)
            )
            self._metrics_timer = None
            if metrics_period_ms > 0:
                self._metrics_timer = self.create_timer(
                    metrics_period_ms / 1000.0, self._publish_metrics,
                )

        # Enclosure group publisher
        self._enclosure_enabled = _as_bool(
            self.get_parameter('enclosure.enabled').value,
        )
        self._enclosure_publisher = None
        if self._enclosure_enabled:
            enclosure_topic = self.get_parameter('enclosure.topic').value
            self._enclosure_publisher = self.create_publisher(
                EnclosureTargetArray, enclosure_topic,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
            enclosure_rate = float(self.get_parameter('enclosure.publish_rate_hz').value)
            if enclosure_rate > 0:
                self._enclosure_timer = self.create_timer(
                    1.0 / enclosure_rate, self._publish_enclosure
                )
            self.get_logger().info(f'Enclosure publisher enabled on {enclosure_topic}')

        # --- UAV state synchronization -------------------------------------
        # Subscribe to /drone_states so we can stamp the outgoing
        # TargetTrackArray header with the same time the UAV state was
        # produced.  enclosure_node consumes this stamp to decide whether
        # the perception output is fresh enough for the closed-loop
        # control.  The subscription is always created (even when fusion
        # is active) because the stamp override is just as valuable when
        # we are republishing fused tracks.
        self._drone_states_topic = self.get_parameter('drone_states_topic').value
        # Lock guards the _latest_drone_state dict so the publish tick
        # and the subscription callback do not race on swap-out.  The
        # state itself is small (one DroneState per UAV, typically < 8)
        # so copying it on each read is cheap.
        self._drone_state_lock = threading.Lock()
        self._latest_drone_state: dict[int, dict[str, Any]] = {}
        # Cache the most recent DroneStateArray header so _publish_tick
        # can stamp outgoing messages even when the drones[] vector is
        # empty (e.g. during pre-takeoff) — the timestamp still tells
        # the enclosure_node when the swarm last published state.
        self._latest_drone_state_header: Optional[Header] = None
        self._drone_state_sub = self.create_subscription(
            DroneStateArray,
            self._drone_states_topic,
            self._on_drone_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.get_logger().info(
            f'UAV state subscription enabled on {self._drone_states_topic}'
        )

        # Input wiring -----------------------------------------------------
        self._video_cap = None
        self._cv_bridge = CvBridge() if _HAS_CV_BRIDGE else None
        self._latest_frame = None
        self._latest_frame_lock = threading.Lock()
        self._latest_track_lock = threading.Lock()
        self._latest_records: list[Any] = []
        self._latest_records_frame_idx: Optional[int] = None
        self._latest_records_header: Optional[Header] = None
        self._frame_seq = 0  # monotonic counter used as ``frame_idx``
        # Input and inference are driven by paired timers below.  Keeping
        # capture to one source frame per inference tick is essential for
        # replay: a zero-period capture timer drains a file before the
        # publish timer can consume its frames.
        self._timer: Optional[Any] = None
        self._video_timer: Optional[Any] = None

        # Fusion deployments consume already-tracked ROS messages and do not
        # need to open the local camera/image pipeline.  When fusion is off,
        # preserve the original single-source behavior unchanged.
        if not self._aggregator.enabled:
            if self._input_mode == 'video':
                self._init_video_input()
            elif self._input_mode == 'topic':
                self._init_topic_input()
            else:
                raise ValueError(
                    f"input_mode must be 'video' or 'topic', got {self._input_mode!r}"
                )

            if self._publish_rate > 0.0:
                period = 1.0 / self._publish_rate
            elif self._input_mode == 'video':
                period = 1.0 / max(self._video_fps, 1.0)
            else:
                period = 0.01
            if self._input_mode == 'video':
                self._video_timer = self.create_timer(period, self._video_tick)
            self._timer = self.create_timer(period, self._publish_tick)
        else:
            self.get_logger().info(
                'local detector input disabled while multi-source fusion is active'
            )

        self.get_logger().info(
            f"tracker_node ready: mode={self._input_mode} "
            f"topic={self._track_topic} rate={self._publish_rate:.1f}Hz "
            f"frame_id={self._frame_id} tracker={overrides['tracker']['kind']} "
            f"weights={overrides['detector']['weights'] or '(auto)'}"
        )

    # ------------------------------------------------------------------
    # Input modes
    # ------------------------------------------------------------------
    def _init_video_input(self) -> None:
        import cv2

        source = self.get_parameter('video_source').value
        if not source:
            self.get_logger().warn(
                'input_mode=video but video_source is empty; '
                'falling back to /dev/video0.'
            )
            source = '0'
        source_text = str(source).strip()
        cap_source = (
            int(source_text)
            if source_text.lstrip('-').isdigit()
            else source_text
        )
        self._video_cap = cv2.VideoCapture(cap_source)
        if not self._video_cap.isOpened():
            raise RuntimeError(f'cannot open video source {source!r}')

        fps = float(self._video_cap.get(cv2.CAP_PROP_FPS) or 20.0) or 20.0
        self._video_fps = fps
        self.get_logger().info(f'video source opened at {fps:.1f} FPS')


    def _init_topic_input(self) -> None:
        if not _HAS_CV_BRIDGE:
            raise RuntimeError(
                'input_mode=topic requires cv_bridge, which is not installed.'
            )
        topic = self.get_parameter('image_topic').value
        self._image_sub = self.create_subscription(
            ROSImage, topic, self._image_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.get_logger().info(f'subscribed to image topic {topic}')

    def _image_callback(self, msg: 'ROSImage') -> None:
        assert self._cv_bridge is not None
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return
        with self._latest_frame_lock:
            self._latest_frame = (frame, msg.header)

    # ------------------------------------------------------------------
    # Per-frame processing
    # ------------------------------------------------------------------
    def _video_tick(self) -> None:
        if self._video_cap is None:
            return
        ok, frame = self._video_cap.read()
        if not ok:
            if self._loop_video:
                # Re-seek to the beginning of the file.
                self._video_cap.set(1, 0)  # CAP_PROP_POS_FRAMES
                ok, frame = self._video_cap.read()
                if not ok:
                    self.get_logger().warn('failed to loop video; stopping')
                    return
            else:
                self.get_logger().info('video source exhausted')
                return
        with self._latest_frame_lock:
            self._latest_frame = (frame, None)

    def _consume_latest_frame(self):
        with self._latest_frame_lock:
            if self._latest_frame is None:
                return None
            frame, src_header = self._latest_frame
            # Keep the most recent frame; we don't queue history.
            self._latest_frame = None
        if frame is None or frame.size == 0:
            return None
        return frame, src_header

    # ------------------------------------------------------------------
    # UAV state synchronization
    # ------------------------------------------------------------------
    def _on_drone_state(self, msg: DroneStateArray) -> None:
        """Cache the latest UAV states so ``_publish_tick`` can stamp messages.

        ``enclosure_node`` requires ``TargetTrackArray.header.stamp`` to
        align with the UAV state timestamp so the closed-loop can reject
        stale perception output.  We snapshot every drone into a
        ``{drone_id: state_dict}`` dictionary keyed by ``drone_id`` so
        callers can also query individual UAV positions without scanning
        the message repeatedly.

        The dictionary form (instead of stashing the raw ROS message)
        keeps the cache independent of ``swarm_interfaces`` at the point
        of use and lets unit tests inject a precomputed cache without
        needing to construct a full ``DroneStateArray``.
        """
        snapshot: dict[int, dict[str, Any]] = {}
        for state in getattr(msg, 'drones', []) or []:
            snapshot[int(state.drone_id)] = {
                'drone_id': int(state.drone_id),
                'x': float(state.x),
                'y': float(state.y),
                'z': float(state.z),
                'vx': float(state.vx),
                'vy': float(state.vy),
                'vz': float(state.vz),
                'available': bool(state.available),
                'platform_type': int(state.platform_type),
                'stamp': _copy_header(getattr(msg, 'header', None)).stamp,
            }
        with self._drone_state_lock:
            self._latest_drone_state = snapshot
            header = getattr(msg, 'header', None)
            self._latest_drone_state_header = (
                _copy_header(header) if header is not None else None
            )

    def _drone_state_snapshot(self) -> tuple[
        dict[int, dict[str, Any]], Optional[Header]
    ]:
        """Return ``(drone_state_dict, drone_state_header)`` atomically.

        Both pieces travel together — the snapshot is the
        ``DroneStateArray.header`` plus the ``drones[]`` array.  The
        header is cached separately so ``_publish_tick`` can fall back to
        it even if ``drones[]`` was empty (e.g. all UAVs disarmed).
        """
        with self._drone_state_lock:
            return (
                dict(self._latest_drone_state),
                _copy_header(self._latest_drone_state_header)
                if self._latest_drone_state_header is not None else None,
            )

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_tick(self) -> None:
        latest = self._consume_latest_frame()
        if latest is None:
            return
        frame, src_header = latest
        try:
            records = list(self._runner.step_records(frame) or [])
        except Exception as exc:
            self.get_logger().error(f'cvtrack runner failed: {exc}')
            return

        # Resolve the outgoing header.  The priority is:
        #   1. ``DroneStateArray.header.stamp`` when /drone_states is
        #      active (this is the synchronization point for the
        #      enclosure_node closed loop).
        #   2. The source ``sensor_msgs/Image`` header when running in
        #      ``input_mode=topic``.
        #   3. The local ROS2 clock otherwise (video / replay modes
        #      where no upstream stamp is available).
        _, drone_state_header = self._drone_state_snapshot()
        if src_header is not None:
            header = _copy_header(src_header, self._frame_id)
        else:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self._frame_id
        if drone_state_header is not None:
            # Override only the timestamp — ``frame_id`` is owned by the
            # camera body and should stay as set above.  Cloning the
            # header first avoids mutating the cached value if other
            # call paths read it concurrently.
            header.stamp = drone_state_header.stamp

        msg = TargetTrackArray()
        msg.header = header
        msg.frame_idx = self._frame_seq
        self._frame_seq += 1
        msg.tracks = [
            self._make_target_track(rec)
            for rec in records
        ]
        with self._latest_track_lock:
            self._latest_records = records
            self._latest_records_frame_idx = msg.frame_idx
            self._latest_records_header = _copy_header(msg.header)
        self._aggregator.publish_local(msg)
        self.get_logger().debug(
            f'published frame_idx={msg.frame_idx} n_tracks={len(msg.tracks)}'
        )

        # Wire debug topic alongside the primary track message.
        if self._debug_pub is not None:
            self._publish_debug_track(msg)

        # Update metrics recorder.
        if self._metrics_recorder is not None:
            motion_modes = [int(t.motion_mode) for t in msg.tracks]
            self._metrics_recorder.update(
                n_active=len(msg.tracks),
                motion_modes=motion_modes,
                track_ids=[int(track.target_id) for track in msg.tracks],
            )

    def _make_target_track(self, rec) -> "TargetTrack":
        """Construct a TargetTrack message from a track record."""
        msg = TargetTrack()
        msg.target_id = int(rec.target_id)
        msg.x = _finite_float(getattr(rec, 'x', 0.0))
        msg.y = _finite_float(getattr(rec, 'y', 0.0))
        msg.vx = _finite_float(getattr(rec, 'vx', 0.0))
        msg.vy = _finite_float(getattr(rec, 'vy', 0.0))

        # Enhanced fields with safe defaults
        msg.confidence = max(
            0.0, min(1.0, _finite_float(getattr(rec, 'confidence', 1.0), 1.0))
        )
        msg.cls = max(0, min(255, int(getattr(rec, 'cls', 0))))
        msg.is_confirmed = _as_bool(
            getattr(rec, 'is_confirmed', getattr(rec, 'confirmed', True)), True,
        )
        msg.speed = max(0.0, _finite_float(getattr(rec, 'speed', 0.0)))
        msg.motion_mode = max(0, min(3, int(getattr(rec, 'motion_mode', 0))))

        # Prediction arrays (5 steps ahead)
        msg.pred_x = _fixed_float_list(getattr(rec, 'pred_x', [0.0] * 5), 5)
        msg.pred_y = _fixed_float_list(getattr(rec, 'pred_y', [0.0] * 5), 5)
        msg.pred_conf = _fixed_float_list(
            getattr(rec, 'pred_conf', [1.0] * 5), 5,
        )

        return msg

    def _publish_debug_track(self, msg: TargetTrackArray) -> None:
        """Publish enriched debug message for /target_track_debug."""
        dbg = TargetTrackDebug()
        dbg.header = msg.header
        dbg.tracks = list(msg.tracks)
        dbg.source_topic = self._track_topic

        # Extract KF covariance, motion reasons, appearance scores.
        kf_cov: list[float] = []
        mm_reasons: list[str] = []
        app_scores: list[float] = []
        for track in msg.tracks:
            kf_cov.extend([0.0] * 9)  # placeholder — replace with real KF cov when available
            mm_labels = {0: 'unknown', 1: 'stationary', 2: 'slow', 3: 'fast'}
            mm_reasons.append(mm_labels.get(track.motion_mode, 'unknown'))
            app_scores.append(-1.0)  # placeholder when appearance model is absent

        dbg.kf_covariance = kf_cov
        dbg.motion_mode_reasons = mm_reasons
        dbg.appearance_scores = app_scores
        _publish_if_active(self._debug_pub, dbg)

    def _publish_metrics(self) -> None:
        """Publish /tracking_metrics DiagnosticArray at the metrics period."""
        if self._metrics_pub is None or self._metrics_recorder is None:
            return
        arr = self._metrics_recorder.diagnostic_array()
        arr.header.stamp = self.get_clock().now().to_msg()
        _publish_if_active(self._metrics_pub, arr)

    def _publish_enclosure(self) -> None:
        """Publish targets for enclosure control group."""
        if not self._enclosure_enabled or self._enclosure_publisher is None:
            return

        # Reuse the records produced by _publish_tick.  Running the detector
        # again here races the main publish path for the single latest-frame
        # buffer and can silently drop /target_track frames.
        with self._latest_track_lock:
            if self._latest_records_frame_idx is None:
                return
            records = list(self._latest_records)
            frame_idx = self._latest_records_frame_idx
            header = _copy_header(self._latest_records_header, self._frame_id)

        msg = EnclosureTargetArray()
        msg.header = header
        msg.frame_idx = frame_idx

        msg.targets = [
            self._make_enclosure_target(rec)
            for rec in records
        ]

        # Get drone positions from parameters
        drone_positions = _as_list(
            self.get_parameter('enclosure.drone_positions').value,
        )
        valid_positions = [
            position for position in drone_positions
            if isinstance(position, dict)
        ]
        if valid_positions:
            drone_x = [
                _finite_float(position.get('x', 0.0))
                for position in valid_positions[:8]
            ]
            drone_y = [
                _finite_float(position.get('y', 0.0))
                for position in valid_positions[:8]
            ]
            while len(drone_x) < 8:
                drone_x.append(0.0)
                drone_y.append(0.0)
            msg.drone_x = drone_x[:8]
            msg.drone_y = drone_y[:8]
            msg.num_drones = min(len(valid_positions), 8)
        else:
            msg.drone_x = [0.0] * 8
            msg.drone_y = [0.0] * 8
            msg.num_drones = 0

        msg.enclosure_radius = 50.0  # Default, can be made configurable
        msg.min_enclosure_dist = 20.0

        _publish_if_active(self._enclosure_publisher, msg)

    def _make_enclosure_target(self, rec) -> "EnclosureTarget":
        """Create an EnclosureTarget message from a track record."""
        msg = EnclosureTarget()
        msg.target_id = rec.target_id
        msg.x = rec.x
        msg.y = rec.y
        msg.speed = getattr(rec, 'speed', 0.0)
        msg.motion_mode = getattr(rec, 'motion_mode', 0)
        msg.confidence = getattr(rec, 'confidence', 1.0)

        # Bounding box
        if hasattr(rec, 'box'):
            msg.box_x1 = float(rec.box.x1) if hasattr(rec.box, 'x1') else 0.0
            msg.box_y1 = float(rec.box.y1) if hasattr(rec.box, 'y1') else 0.0
            msg.box_x2 = float(rec.box.x2) if hasattr(rec.box, 'x2') else 0.0
            msg.box_y2 = float(rec.box.y2) if hasattr(rec.box, 'y2') else 0.0
        else:
            msg.box_x1 = msg.box_y1 = msg.box_x2 = msg.box_y2 = 0.0

        # Predictions
        msg.pred_x = _fixed_float_list(getattr(rec, 'pred_x', []), 5)
        msg.pred_y = _fixed_float_list(getattr(rec, 'pred_y', []), 5)

        # History (the runner record does not expose the full trail yet).
        msg.history_x = [_finite_float(getattr(rec, 'x', 0.0))] * 10
        msg.history_y = [_finite_float(getattr(rec, 'y', 0.0))] * 10

        return msg

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        if self._video_cap is not None:
            try:
                self._video_cap.release()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    # Surface a one-line warning if cv_bridge failed to import (it is
    # only needed for input_mode:=topic, but a broken ABI is otherwise
    # silent).
    _report_cv_bridge_state()
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
