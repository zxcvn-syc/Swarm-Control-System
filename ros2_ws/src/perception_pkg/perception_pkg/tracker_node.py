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

import logging
import os
import sys
import threading
from typing import Any, Optional

import rclpy
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


log = logging.getLogger(__name__)


# Type hint for the message class used in _make_target_track
TargetTrack = TargetTrack
EnclosureTarget = EnclosureTarget


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

    # Appearance (only used by deepsort_cascade)
    node.declare_parameter('appearance.enabled', False)
    node.declare_parameter('appearance.weights', '')

    # Enclosure group integration
    node.declare_parameter('enclosure.enabled', False)
    node.declare_parameter('enclosure.topic', '/enclosure_targets')
    node.declare_parameter('enclosure.publish_rate_hz', 5.0)
    node.declare_parameter('enclosure.drone_positions', [])


def _build_runner_overrides(node: Node) -> dict:
    """Translate ROS2 parameters into the cvtrack-runner override dict."""
    p = node.get_parameter
    det = {
        'backend': p('detector.backend').value,
        'weights': p('detector.weights').value,
        'device': p('detector.device').value,
        'imgsz': int(p('detector.imgsz').value),
        'conf': float(p('detector.conf').value),
        'classes': list(p('detector.classes').value),
        'min_box_area': float(p('detector.min_box_area').value),
        'min_conf': float(p('detector.min_conf').value),
        'nms_iou': float(p('detector.nms_iou').value),
    }
    tr = {
        'kind': p('tracker.kind').value,
        'max_age': int(p('tracker.max_age').value),
        'n_init': int(p('tracker.n_init').value),
        'iou_thresh': float(p('tracker.iou_thresh').value),
        'high_conf': float(p('tracker.high_conf').value),
        'new_track_conf': float(p('tracker.new_track_conf').value),
        'lost_relink_frames': int(p('tracker.lost_relink_frames').value),
        'stationary_prune': bool(p('tracker.stationary_prune').value),
    }
    appearance_enabled = bool(p('appearance.enabled').value)
    ap = {
        'enabled': appearance_enabled,
    }
    weights = p('appearance.weights').value
    if weights:
        ap['weights'] = weights
    return {'detector': det, 'tracker': tr, 'appearance': ap}


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class TrackerNode(Node):
    """Perception node: YOLOv8 + DeepSORT / BoT-SORT -> TargetTrackArray."""

    def __init__(self) -> None:
        super().__init__('tracker_node')
        _declare_parameters(self)

        # Lazy import so the message generation step doesn't depend on cvtrack.
        # Auto-add the bundled cv_tracking_demo source tree to sys.path if
        # cvtrack isn't already importable — this lets users run the node
        # without a separate `pip install -e` step on the perception host.
        try:
            import cvtrack.runner  # noqa: F401
        except ImportError:
            cvtrack_src = '/home/hhh/Downloads/cv_tracking_demo/src'
            if cvtrack_src not in sys.path and os.path.isdir(cvtrack_src):
                sys.path.insert(0, cvtrack_src)
        try:
            from cvtrack.runner import CvtrackRunner
        except ImportError as exc:
            self.get_logger().fatal(
                'cvtrack is not installed. Either run `pip install -e '
                '/home/hhh/Downloads/cv_tracking_demo` or export '
                'PYTHONPATH=/home/hhh/Downloads/cv_tracking_demo/src '
                'before launching this node.'
            )
            raise RuntimeError(f'cvtrack import failed: {exc}') from exc

        overrides = _build_runner_overrides(self)
        # dt is set from the ROS2 parameter; the runner derives fps from dt.
        self._dt = float(self.get_parameter('tracker.dt').value)
        self._runner = CvtrackRunner.from_overrides(
            preset=None, overrides=overrides, fps=1.0 / max(self._dt, 1e-3)
        )
        # ``from_overrides`` reads ``fps`` and overwrites dt if the overrides
        # don't explicitly pin it.  Force the dt the user asked for:
        self._runner.settings.dt = self._dt
        self._runner.tracker.kf.dt = self._dt

        self._track_topic = self.get_parameter('track_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._input_mode = self.get_parameter('input_mode').value.lower()
        self._loop_video = bool(self.get_parameter('loop_video').value)

        self._publisher = self.create_publisher(
            TargetTrackArray, self._track_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        # Enclosure group publisher
        self._enclosure_enabled = bool(self.get_parameter('enclosure.enabled').value)
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

        # Input wiring -----------------------------------------------------
        self._video_cap = None
        self._cv_bridge = CvBridge() if _HAS_CV_BRIDGE else None
        self._latest_frame = None
        self._latest_frame_lock = threading.Lock()
        self._frame_seq = 0  # monotonic counter used as ``frame_idx``
        # Publish timer is created below, after input wiring.  Initialise
        # to None so ``_init_video_input`` (which runs first) can branch
        # on whether to drive the capture off the publish timer or its
        # own timer.
        self._timer: Optional[Any] = None
        self._video_timer: Optional[Any] = None

        if self._input_mode == 'video':
            self._init_video_input()
        elif self._input_mode == 'topic':
            self._init_topic_input()
        else:
            raise ValueError(
                f"input_mode must be 'video' or 'topic', got {self._input_mode!r}"
            )

        # Publishing tick --------------------------------------------------
        if self._publish_rate > 0.0:
            period = 1.0 / self._publish_rate
            self._timer = self.create_timer(period, self._publish_tick)
        else:
            self._timer = None

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
        cap_source = int(source) if source.isdigit() else source
        self._video_cap = cv2.VideoCapture(cap_source)
        if not self._video_cap.isOpened():
            raise RuntimeError(f'cannot open video source {source!r}')

        fps = float(self._video_cap.get(cv2.CAP_PROP_FPS) or 20.0) or 20.0
        self._video_fps = fps
        self.get_logger().info(f'video source opened at {fps:.1f} FPS')

        # Drive the capture off the same publish timer (or a fast timer
        # if publishing is rate-limited to 0).
        if self._timer is None:
            self._video_timer = self.create_timer(0.0, self._video_tick)
        else:
            self._video_timer = None

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
    # Publishing
    # ------------------------------------------------------------------
    def _publish_tick(self) -> None:
        latest = self._consume_latest_frame()
        if latest is None:
            return
        frame, src_header = latest
        try:
            records = self._runner.step_records(frame)
        except Exception as exc:
            self.get_logger().error(f'cvtrack runner failed: {exc}')
            return

        if src_header is not None:
            header = src_header
        else:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id

        msg = TargetTrackArray()
        msg.header = header
        msg.frame_idx = self._frame_seq
        self._frame_seq += 1
        msg.tracks = [
            self._make_target_track(rec)
            for rec in records
        ]
        self._publisher.publish(msg)
        self.get_logger().debug(
            f'published frame_idx={msg.frame_idx} n_tracks={len(msg.tracks)}'
        )

    def _make_target_track(self, rec) -> "TargetTrack":
        """Construct a TargetTrack message from a track record."""
        msg = TargetTrack()
        msg.target_id = int(rec.target_id)
        msg.x = float(rec.x)
        msg.y = float(rec.y)
        msg.vx = float(rec.vx)
        msg.vy = float(rec.vy)

        # Enhanced fields with safe defaults
        msg.confidence = float(getattr(rec, 'confidence', 1.0))
        msg.cls = int(getattr(rec, 'cls', 0))
        msg.is_confirmed = bool(getattr(rec, 'is_confirmed', True))
        msg.speed = float(getattr(rec, 'speed', 0.0))
        msg.motion_mode = int(getattr(rec, 'motion_mode', 0))

        # Prediction arrays (5 steps ahead)
        pred_x = getattr(rec, 'pred_x', [0.0] * 5)
        pred_y = getattr(rec, 'pred_y', [0.0] * 5)
        pred_conf = getattr(rec, 'pred_conf', [1.0] * 5)

        if len(pred_x) < 5:
            pred_x = list(pred_x) + [0.0] * (5 - len(pred_x))
        if len(pred_y) < 5:
            pred_y = list(pred_y) + [0.0] * (5 - len(pred_y))
        if len(pred_conf) < 5:
            pred_conf = list(pred_conf) + [0.0] * (5 - len(pred_conf))

        msg.pred_x = pred_x[:5]
        msg.pred_y = pred_y[:5]
        msg.pred_conf = pred_conf[:5]

        return msg

    def _publish_enclosure(self) -> None:
        """Publish targets for enclosure control group."""
        if not self._enclosure_enabled or self._enclosure_publisher is None:
            return

        latest = self._consume_latest_frame()
        if latest is None:
            return

        try:
            records = self._runner.step_records(latest[0])
        except Exception:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id

        msg = EnclosureTargetArray()
        msg.header = header
        msg.frame_idx = self._frame_seq

        msg.targets = [
            self._make_enclosure_target(rec)
            for rec in records
        ]

        # Get drone positions from parameters
        drone_positions = self.get_parameter('enclosure.drone_positions').value
        if drone_positions:
            drone_x = [float(p.get('x', 0.0)) for p in drone_positions[:8]]
            drone_y = [float(p.get('y', 0.0)) for p in drone_positions[:8]]
            while len(drone_x) < 8:
                drone_x.append(0.0)
                drone_y.append(0.0)
            msg.drone_x = drone_x[:8]
            msg.drone_y = drone_y[:8]
            msg.num_drones = uint8(min(len(drone_positions), 8))
        else:
            msg.drone_x = [0.0] * 8
            msg.drone_y = [0.0] * 8
            msg.num_drones = 0

        msg.enclosure_radius = 50.0  # Default, can be made configurable
        msg.min_enclosure_dist = 20.0

        self._enclosure_publisher.publish(msg)

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
        pred_x = getattr(rec, 'pred_x', [0.0] * 5)
        pred_y = getattr(rec, 'pred_y', [0.0] * 5)
        msg.pred_x = pred_x[:5]
        msg.pred_y = pred_y[:5]

        # History (placeholder, would need track trail data)
        msg.history_x = [rec.x] * 10
        msg.history_y = [rec.y] * 10

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