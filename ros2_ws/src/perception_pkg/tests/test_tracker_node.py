"""Node-level tests for ``tracker_node``.

These tests do **not** require a live ROS2 context or a colcon build.
Stubs are installed by ``conftest.py`` (via ``pytest_configure``) before
this module is imported, so ``tracker_node.py`` can be loaded without
``rclpy`` or ``swarm_interfaces``.

What the tests verify
---------------------
- _declare_parameters / _build_runner_overrides produce correct structures
- _make_target_track maps every TrackedTarget field to TargetTrack correctly
- _publish_tick emits a well-formed TargetTrackArray with correct frame_idx
- IDs are stable across frames
- motion_mode / speed / pred arrays are within valid ranges
"""

from __future__ import annotations

import sys
import threading
import types
from typing import Any, List
from unittest import mock

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Load tracker_node — stubs are installed by conftest.py before this runs.
# The perception_pkg/__init__.py makes the workspace root a regular package,
# so perception_pkg.tracker_node resolves correctly to the ROS2 package.
# ---------------------------------------------------------------------------

import perception_pkg.tracker_node
import perception_pkg.coord_transform_node

TrackerNode = perception_pkg.tracker_node.TrackerNode
_declare_parameters = perception_pkg.tracker_node._declare_parameters
_build_runner_overrides = perception_pkg.tracker_node._build_runner_overrides
MultiSourceAggregator = perception_pkg.tracker_node.MultiSourceAggregator


# ---------------------------------------------------------------------------
# Helpers to build fake records matching TrackedTarget / TargetTrack fields
# ---------------------------------------------------------------------------

def _make_mock_node():
    """Return a fully-stubbed Node-like object with all methods used by tracker_node."""
    from builtin_interfaces.msg import Time as _BuiltinTime

    # Simulate rclpy.time.Time — clock.now() returns this, and .to_msg() on it
    # returns builtin_interfaces.msg.Time (what std_msgs/Header expects).
    class _RclpyTime:
        def __init__(self, sec: int = 1, nanosec: int = 0):
            self._sec = sec
            self._nanosec = nanosec

        def to_msg(self) -> _BuiltinTime:
            return _BuiltinTime(sec=self._sec, nanosec=self._nanosec)

    class _Clock:
        def now(self) -> _RclpyTime:
            return _RclpyTime(sec=1, nanosec=0)

        def __call__(self) -> _RclpyTime:
            return self.now()

    node = types.SimpleNamespace()
    node.get_clock = _Clock
    node.get_logger = lambda: types.SimpleNamespace(
        warning=lambda *a, **k: None,
        info=lambda *a, **k: None,
        error=lambda *a, **k: None,
        debug=lambda *a, **k: None,
    )
    return node


def _install_node_stubs(node: types.SimpleNamespace) -> None:
    """Add all instance attributes that _publish_tick (and helpers) require."""
    _mock = _make_mock_node()
    node.get_clock = _mock.get_clock
    node.get_logger = _mock.get_logger
    node._debug_pub = None
    node._metrics_recorder = None
    node._track_topic = "/target_track"
    node._latest_track_lock = threading.Lock()
    node._latest_records = []
    node._latest_records_frame_idx = None
    node._latest_records_header = None


class FakeRecord:
    """Mimics a ``TrackedTarget`` returned by ``CvtrackRunner.step_records``."""

    def __init__(
        self,
        target_id: int = 1,
        x: float = 320.0,
        y: float = 240.0,
        vx: float = 5.0,
        vy: float = 3.0,
        confidence: float = 0.85,
        cls: int = 2,
        is_confirmed: bool = True,
        speed: float = 5.83,
        motion_mode: int = 2,
        pred_x: List[float] | None = None,
        pred_y: List[float] | None = None,
        pred_conf: List[float] | None = None,
    ):
        self.target_id = target_id
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.confidence = confidence
        self.cls = cls
        self.is_confirmed = is_confirmed
        self.speed = speed
        self.motion_mode = motion_mode
        self.pred_x = pred_x if pred_x is not None else [x + vx * i for i in range(1, 6)]
        self.pred_y = pred_y if pred_y is not None else [y + vy * i for i in range(1, 6)]
        self.pred_conf = pred_conf if pred_conf is not None else [0.9 ** i for i in range(5)]
        self.box = types.SimpleNamespace(x1=x - 20, y1=y - 20, x2=x + 20, y2=y + 20)


def _build_mock_runner(records: List[FakeRecord]):
    """Return a mock ``CvtrackRunner`` that returns ``records`` on ``step_records``."""
    mock_runner = mock.MagicMock()
    mock_runner.step_records.return_value = records
    mock_runner.settings = types.SimpleNamespace(dt=0.05)
    mock_runner.tracker = types.SimpleNamespace(kf=types.SimpleNamespace(dt=0.05))
    return mock_runner


# ---------------------------------------------------------------------------
# 1. _declare_parameters / _build_runner_overrides
# ---------------------------------------------------------------------------

def test_declare_parameters_does_not_crash():
    """``_declare_parameters`` must not raise on a minimal mock Node."""
    mock_node = mock.MagicMock()
    mock_node.declare_parameter = mock.MagicMock()
    mock_node.get_parameter = mock.MagicMock(
        return_value=types.SimpleNamespace(value=None)
    )
    _declare_parameters(mock_node)
    assert True


def test_build_runner_overrides_returns_detector_and_tracker_keys():
    """The overrides dict must contain 'detector' and 'tracker' keys."""
    mock_node = _make_mock_node()
    param_map = {
        "detector.backend": "auto",
        "detector.weights": "",
        "detector.device": "cpu",
        "detector.imgsz": 480,
        "detector.conf": 0.15,
        "detector.classes": [0, 1, 2],
        "detector.min_box_area": 200.0,
        "detector.min_conf": 0.0,
        "detector.nms_iou": 0.5,
        "tracker.kind": "deepsort_cascade",
        "tracker.max_age": 30,
        "tracker.n_init": 3,
        "tracker.iou_thresh": 0.30,
        "tracker.high_conf": 0.35,
        "tracker.new_track_conf": 0.20,
        "tracker.lost_relink_frames": 30,
        "tracker.stationary_prune": True,
        "tracker.dt": 0.05,
        "tracker.kalman.dt": 0.05,
        "tracker.kalman.sigma_p": 0.05,
        "tracker.kalman.sigma_v": 0.00625,
        "tracker.kalman.sigma_m": 0.05,
        "tracker.kalman.acceleration_gain": 0.5,
        "tracker.kalman.motion_threshold_slow": 2.0,
        "tracker.kalman.motion_threshold_fast": 20.0,
        "tracker.kalman.base_std_pos": 0.05,
        "tracker.kalman.base_std_vel": 0.00625,
        "tracker.kalman.base_std_meas": 0.05,
        "tracker.kalman.motion_adapt_gain": 0.3,
        "tracker.kalman.velocity_limit": 100.0,
        "tracker.kalman.innovation_gate": 9.4877,
        "trajectory_prediction.enabled": True,
        "trajectory_prediction.prediction_steps": 10,
        "trajectory_prediction.confidence_decay": 0.9,
        "trajectory_prediction.min_confidence": 0.1,
        "appearance.enabled": False,
        "appearance.weights": "",
    }

    def get_param(name):
        return types.SimpleNamespace(value=param_map.get(name))

    mock_node.get_parameter = get_param

    overrides = _build_runner_overrides(mock_node)
    assert "detector" in overrides
    assert "tracker" in overrides
    assert "appearance" in overrides
    assert overrides["detector"]["device"] == "cpu"
    assert overrides["tracker"]["kind"] == "deepsort_cascade"


def test_launch_parameter_coercion_keeps_false_and_list_values():
    """Launch substitutions must not turn string ``false`` into True."""
    tracker_module = perception_pkg.tracker_node
    assert tracker_module._as_bool("false") is False
    assert tracker_module._as_bool("true") is True
    assert tracker_module._class_ids("[0, 2, 5]") == [0, 2, 5]
    assert tracker_module._as_list("sensor_0, sensor_1") == ["sensor_0", "sensor_1"]


# ---------------------------------------------------------------------------
# 2. _make_target_track — field mapping from TrackedTarget → TargetTrack
# ---------------------------------------------------------------------------

def test_make_target_track_maps_all_fields():
    """All TrackedTarget fields must appear correctly in the output message."""
    rec = FakeRecord(
        target_id=42,
        x=100.5,
        y=200.5,
        vx=3.1,
        vy=-1.2,
        confidence=0.92,
        cls=5,
        is_confirmed=True,
        speed=3.32,
        motion_mode=3,
        pred_x=[110.0, 120.0, 130.0, 140.0, 150.0],
        pred_y=[205.0, 210.0, 215.0, 220.0, 225.0],
        pred_conf=[0.9, 0.8, 0.7, 0.6, 0.5],
    )
    msg = TrackerNode._make_target_track(None, rec)

    assert msg.target_id == 42
    assert msg.x == pytest.approx(100.5)
    assert msg.y == pytest.approx(200.5)
    assert msg.vx == pytest.approx(3.1)
    assert msg.vy == pytest.approx(-1.2)
    assert msg.confidence == pytest.approx(0.92)
    assert msg.cls == 5
    assert msg.is_confirmed is True
    assert msg.speed == pytest.approx(3.32)
    assert msg.motion_mode == 3
    assert msg.pred_x == [110.0, 120.0, 130.0, 140.0, 150.0]
    assert msg.pred_y == [205.0, 210.0, 215.0, 220.0, 225.0]
    assert msg.pred_conf == [0.9, 0.8, 0.7, 0.6, 0.5]


def test_make_target_track_pads_short_predictions():
    """If pred_x/y/conf are shorter than 5, pad with zeros."""
    rec = FakeRecord(
        target_id=7,
        x=50.0,
        y=60.0,
        vx=1.0,
        vy=1.0,
        pred_x=[100.0, 110.0],
        pred_y=[160.0, 170.0],
        pred_conf=[0.9],
    )
    msg = TrackerNode._make_target_track(None, rec)
    assert len(msg.pred_x) == 5
    assert len(msg.pred_y) == 5
    assert len(msg.pred_conf) == 5
    assert msg.pred_x[:2] == [100.0, 110.0]
    assert msg.pred_x[2:] == [0.0, 0.0, 0.0]


def test_make_target_track_defaults():
    """Missing attributes fall back to safe defaults."""
    minimal_rec = types.SimpleNamespace(
        target_id=99,
        x=0.0,
        y=0.0,
        vx=0.0,
        vy=0.0,
    )
    msg = TrackerNode._make_target_track(None, minimal_rec)
    assert msg.target_id == 99
    assert msg.x == 0.0
    assert msg.y == 0.0
    assert msg.vx == 0.0
    assert msg.vy == 0.0
    assert msg.confidence == 1.0  # fallback
    assert msg.cls == 0           # fallback
    assert msg.is_confirmed is True  # fallback
    assert msg.speed == 0.0      # fallback
    assert msg.motion_mode == 0   # fallback
    assert len(msg.pred_x) == 5
    assert len(msg.pred_y) == 5
    assert len(msg.pred_conf) == 5


def test_make_target_track_uses_runner_confirmation_field():
    """TrackedTarget.confirmed must survive when tentative output is enabled."""
    rec = FakeRecord(is_confirmed=True)
    del rec.is_confirmed
    rec.confirmed = False
    msg = TrackerNode._make_target_track(None, rec)
    assert msg.is_confirmed is False


# ---------------------------------------------------------------------------
# 3. _publish_tick — end-to-end with mock runner
# ---------------------------------------------------------------------------

def test_publish_tick_emits_correct_message_structure():
    """``_publish_tick`` must produce a well-formed TargetTrackArray."""
    node = TrackerNode.__new__(TrackerNode)
    node._runner = _build_mock_runner([
        FakeRecord(target_id=1, x=100, y=200, vx=5, vy=-3),
        FakeRecord(target_id=2, x=300, y=400, vx=-2, vy=1),
    ])
    node._frame_seq = 0
    node._frame_id = "camera_optical_frame"
    node._latest_frame_lock = types.SimpleNamespace()
    node._latest_frame = (np.zeros((480, 640, 3), dtype=np.uint8), None)
    node._aggregator = mock.MagicMock()
    _install_node_stubs(node)

    with mock.patch.object(
        node, "_consume_latest_frame",
        return_value=(np.zeros((480, 640, 3), dtype=np.uint8), None)
    ):
        node._publish_tick()
        node._aggregator.publish_local.assert_called_once()
        msg = node._aggregator.publish_local.call_args[0][0]

    assert hasattr(msg, "tracks")
    assert len(msg.tracks) == 2
    assert msg.tracks[0].target_id == 1
    assert msg.tracks[0].x == pytest.approx(100.0)
    assert msg.tracks[0].y == pytest.approx(200.0)
    assert msg.tracks[1].target_id == 2


def test_publish_tick_increments_frame_seq():
    """``frame_idx`` must be monotonically increasing across ticks."""
    node = TrackerNode.__new__(TrackerNode)
    node._frame_seq = 5
    node._frame_id = "camera_optical_frame"
    node._runner = _build_mock_runner([FakeRecord(target_id=1)])
    node._aggregator = mock.MagicMock()
    _install_node_stubs(node)

    with mock.patch.object(
        node, "_consume_latest_frame",
        return_value=(np.zeros((480, 640, 3), dtype=np.uint8), None)
    ):
        node._publish_tick()
        msg0 = node._aggregator.publish_local.call_args[0][0]
        assert msg0.frame_idx == 5

        node._publish_tick()
        msg1 = node._aggregator.publish_local.call_args[0][0]
        assert msg1.frame_idx == 6


def test_publish_tick_empty_frame_produces_empty_tracks():
    """When runner returns no records, tracks list is empty."""
    node = TrackerNode.__new__(TrackerNode)
    node._frame_seq = 0
    node._frame_id = "camera_optical_frame"
    node._runner = _build_mock_runner([])
    node._aggregator = mock.MagicMock()
    _install_node_stubs(node)

    with mock.patch.object(
        node, "_consume_latest_frame",
        return_value=(np.zeros((480, 640, 3), dtype=np.uint8), None)
    ):
        node._publish_tick()
        msg = node._aggregator.publish_local.call_args[0][0]
    assert msg.tracks == []


# ---------------------------------------------------------------------------
# 4. ID stability under repeated frames (DeepSORT-like behaviour)
# ---------------------------------------------------------------------------

def test_target_id_stable_across_frames():
    """The same target must keep the same ID across consecutive frames."""
    node = TrackerNode.__new__(TrackerNode)
    node._frame_seq = 0
    node._frame_id = "camera_optical_frame"
    node._runner = _build_mock_runner([
        FakeRecord(target_id=7, x=320, y=240),
    ])
    node._aggregator = mock.MagicMock()
    _install_node_stubs(node)

    ids_per_frame: List[int] = []
    with mock.patch.object(
        node, "_consume_latest_frame",
        return_value=(np.zeros((480, 640, 3), dtype=np.uint8), None)
    ):
        for _ in range(5):
            node._publish_tick()
            msg = node._aggregator.publish_local.call_args[0][0]
            ids_per_frame.append(msg.tracks[0].target_id)

    assert len(set(ids_per_frame)) == 1, (
        f"ID should be stable across frames, got: {ids_per_frame}"
    )


def test_multiple_distinct_ids_preserved():
    """Multiple distinct targets must each retain their own ID."""
    node = TrackerNode.__new__(TrackerNode)
    node._frame_seq = 0
    node._frame_id = "camera_optical_frame"
    node._runner = _build_mock_runner([
        FakeRecord(target_id=1, x=100, y=100),
        FakeRecord(target_id=2, x=200, y=200),
        FakeRecord(target_id=3, x=300, y=300),
    ])
    node._aggregator = mock.MagicMock()
    _install_node_stubs(node)

    with mock.patch.object(
        node, "_consume_latest_frame",
        return_value=(np.zeros((480, 640, 3), dtype=np.uint8), None)
    ):
        node._publish_tick()
        msg = node._aggregator.publish_local.call_args[0][0]

    ids = {t.target_id for t in msg.tracks}
    assert ids == {1, 2, 3}


# ---------------------------------------------------------------------------
# 5. Motion mode / speed / pred fields across records
# ---------------------------------------------------------------------------

def test_motion_mode_field_values():
    """motion_mode must round-trip correctly (stationary/slow/fast/unknown)."""
    for motion_mode_int, motion_mode_str, expected in [
        (1, "stationary", 1),
        (2, "slow", 2),
        (3, "fast", 3),
    ]:
        rec = FakeRecord(
            target_id=1,
            motion_mode=motion_mode_int,
        )
        msg = TrackerNode._make_target_track(None, rec)
        assert msg.motion_mode == expected, (
            f"motion_mode={motion_mode_int} should map to {expected}"
        )


def test_speed_equals_hypot_vx_vy():
    """speed should equal sqrt(vx^2 + vy^2) when record.speed is absent."""
    rec = FakeRecord(target_id=1, vx=3.0, vy=4.0)
    # Remove speed from record
    del rec.speed
    msg = TrackerNode._make_target_track(None, rec)
    # speed fallback is 0.0 when absent (the getattr fallback path)
    assert msg.speed == pytest.approx(0.0)


def test_predicted_trajectories_preserved():
    """pred_x/pred_y/pred_conf arrays must be exactly as set on the record."""
    pred_x = [150.0 + i * 5 for i in range(5)]
    pred_y = [200.0 + i * 3 for i in range(5)]
    pred_conf = [1.0 - i * 0.15 for i in range(5)]
    rec = FakeRecord(
        target_id=1, x=150.0, y=200.0,
        pred_x=pred_x, pred_y=pred_y, pred_conf=pred_conf,
    )
    msg = TrackerNode._make_target_track(None, rec)
    assert msg.pred_x == pred_x
    assert msg.pred_y == pred_y
    assert msg.pred_conf == pred_conf


# ---------------------------------------------------------------------------
# 6. MultiSourceAggregator — fusion off → direct publish
# ---------------------------------------------------------------------------

def test_aggregator_disabled_does_not_create_subscriptions():
    """When enable_fusion=false, the aggregator must not subscribe to any topic."""
    mock_node = mock.MagicMock()
    mock_node.get_parameter.return_value = types.SimpleNamespace(value=None)
    mock_pub = mock.MagicMock()

    agg = MultiSourceAggregator(mock_node, mock_pub, frame_id="camera_optical_frame")
    assert agg.enabled is False


def test_aggregator_direct_publish():
    """When disabled, publish_local must forward to the underlying publisher."""
    mock_node = mock.MagicMock()
    mock_node.get_parameter.return_value = types.SimpleNamespace(value=None)
    mock_pub = mock.MagicMock()

    agg = MultiSourceAggregator(mock_node, mock_pub)
    fake_msg = types.SimpleNamespace()
    agg.publish_local(fake_msg)
    mock_pub.publish.assert_called_once_with(fake_msg)
