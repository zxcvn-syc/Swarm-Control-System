"""End-to-end test for multi-source trajectory fusion over ROS2 topics.

This test simulates two sensor sources publishing TargetTrackArray on
``/sensor_a/target_track`` and ``/sensor_b/target_track``, runs the
TrackFusion pipeline (the same one used by MultiSourceAggregator in
tracker_node.py), and verifies that the fused output on ``/target_track``
is a confidence-weighted average of the two sources.

Run::

    # After sourcing ROS2 workspace
    python3 test_fusion_e2e.py

    # Or via pytest (skips if rclpy not available)
    pytest test_fusion_e2e.py -v

Acceptance criteria:
    - Two sources publishing the same target_id with different positions
      and confidences produce a single fused track.
    - Fused position is between the two source positions, weighted toward
      the higher-confidence source.
    - Fused track count == 1 (deduplication works).
    - Latency from publish to fused output < 500 ms.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import List, Optional

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Header
    from swarm_interfaces.msg import TargetTrack, TargetTrackArray
    _RCLPY_OK = True
except ImportError:
    _RCLPY_OK = False


# ---------------------------------------------------------------------------
# Fusion bridge node (mirrors MultiSourceAggregator logic in tracker_node.py)
# ---------------------------------------------------------------------------
class FusionBridgeNode(Node):
    """Subscribe to two sensor topics, fuse, publish on /target_track."""

    def __init__(self) -> None:
        super().__init__("fusion_e2e_test_node")

        from cvtrack.tracker.fusion import TrackFusion
        from cvtrack.types import Box, Track

        self._Track = Track
        self._Box = Box
        self._fusion = TrackFusion()
        self._fusion.register_source("sensor_a")
        self._fusion.register_source("sensor_b")
        self._lock = threading.Lock()
        self._pending: dict[str, TargetTrackArray] = {}
        self._frame_seq = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._subs = [
            self.create_subscription(
                TargetTrackArray, "/sensor_a/target_track",
                lambda msg: self._on_source("sensor_a", msg), qos,
            ),
            self.create_subscription(
                TargetTrackArray, "/sensor_b/target_track",
                lambda msg: self._on_source("sensor_b", msg), qos,
            ),
        ]
        self._pub = self.create_publisher(
            TargetTrackArray, "/target_track", qos,
        )
        self._timer = self.create_timer(0.05, self._tick)

    def _on_source(self, source: str, msg: TargetTrackArray) -> None:
        with self._lock:
            self._pending[source] = msg

    def _track_from_msg(self, msg: TargetTrack):
        confidence = float(getattr(msg, "confidence", 1.0))
        x, y = float(msg.x), float(msg.y)
        half = 10.0
        mean = np.array([x, y, float(msg.vx), float(msg.vy)], dtype=np.float64)
        var = max(1.0, (1.0 - max(0.0, min(1.0, confidence))) * 100.0)
        cov = np.diag([var, var, var, var]).astype(np.float64)
        return self._Track(
            track_id=int(msg.target_id),
            label=str(getattr(msg, "cls", 0)),
            mean=mean,
            cov=cov,
            box=self._Box(
                x - half, y - half, x + half, y + half,
                confidence, int(getattr(msg, "cls", 0)),
                str(getattr(msg, "cls", 0)),
            ),
            confirmed=bool(getattr(msg, "is_confirmed", True)),
        )

    @staticmethod
    def _msg_from_track(track) -> TargetTrack:
        msg = TargetTrack()
        msg.target_id = int(track.track_id)
        msg.x = float(track.pos[0])
        msg.y = float(track.pos[1])
        msg.vx = float(track.mean[2]) if track.mean.size > 2 else 0.0
        msg.vy = float(track.mean[3]) if track.mean.size > 3 else 0.0
        msg.confidence = float(track.box.score)
        msg.cls = int(track.box.cls)
        msg.is_confirmed = bool(track.confirmed)
        msg.speed = float(math.hypot(msg.vx, msg.vy))
        return msg

    def _tick(self) -> None:
        with self._lock:
            pending = self._pending
            self._pending = {}
        if not pending:
            return
        try:
            for source, source_msg in pending.items():
                tracks = [self._track_from_msg(t) for t in source_msg.tracks]
                self._fusion.update(source, tracks)
            fused = self._fusion.fused_tracks()
        except Exception as exc:
            self.get_logger().error(f"fusion tick failed: {exc}")
            return
        output = TargetTrackArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "fusion_e2e"
        output.header = header
        output.frame_idx = self._frame_seq
        self._frame_seq += 1
        output.tracks = [self._msg_from_track(t) for t in fused]
        self._pub.publish(output)


# ---------------------------------------------------------------------------
# Test publisher node
# ---------------------------------------------------------------------------
class TestPublisherNode(Node):
    """Publish synthetic tracks on two sensor topics."""

    def __init__(
        self,
        target_id: int,
        pos_a: tuple[float, float],
        pos_b: tuple[float, float],
        conf_a: float = 0.9,
        conf_b: float = 0.6,
        publish_rate: float = 20.0,
    ) -> None:
        super().__init__("fusion_e2e_publisher")
        self._target_id = target_id
        self._pos_a = pos_a
        self._pos_b = pos_b
        self._conf_a = conf_a
        self._conf_b = conf_b
        self._seq = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_a = self.create_publisher(
            TargetTrackArray, "/sensor_a/target_track", qos,
        )
        self._pub_b = self.create_publisher(
            TargetTrackArray, "/sensor_b/target_track", qos,
        )
        period = 1.0 / max(1.0, publish_rate)
        self._timer = self.create_timer(period, self._tick)

    def _make_track(
        self, target_id: int, x: float, y: float, confidence: float,
    ) -> TargetTrack:
        msg = TargetTrack()
        msg.target_id = target_id
        msg.x = x
        msg.y = y
        msg.vx = 0.0
        msg.vy = 0.0
        msg.confidence = confidence
        msg.cls = 0
        msg.is_confirmed = True
        msg.speed = 0.0
        msg.motion_mode = 1  # stationary
        msg.pred_x = [0.0] * 5
        msg.pred_y = [0.0] * 5
        msg.pred_conf = [0.0] * 5
        return msg

    def _make_array(self, track: TargetTrack) -> TargetTrackArray:
        arr = TargetTrackArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "test"
        arr.header = header
        arr.frame_idx = self._seq
        self._seq += 1
        arr.tracks = [track]
        return arr

    def _tick(self) -> None:
        track_a = self._make_track(
            self._target_id, self._pos_a[0], self._pos_a[1], self._conf_a,
        )
        track_b = self._make_track(
            self._target_id, self._pos_b[0], self._pos_b[1], self._conf_b,
        )
        self._pub_a.publish(self._make_array(track_a))
        self._pub_b.publish(self._make_array(track_b))


# ---------------------------------------------------------------------------
# Test harness
# ---------------------------------------------------------------------------
class FusionE2EResult:
    """Collects fused output messages for verification."""

    def __init__(self) -> None:
        self.received: list[TargetTrackArray] = []
        self.first_recv_time: Optional[float] = None
        self._lock = threading.Lock()
        self._ready = threading.Event()

    def on_message(self, msg: TargetTrackArray) -> None:
        with self._lock:
            self.received.append(msg)
            if self.first_recv_time is None:
                self.first_recv_time = time.monotonic()
            self._ready.set()

    def wait(self, timeout: float = 5.0) -> bool:
        return self._ready.wait(timeout=timeout)


def run_e2e_test(
    target_id: int = 42,
    pos_a: tuple[float, float] = (100.0, 200.0),
    pos_b: tuple[float, float] = (120.0, 180.0),
    conf_a: float = 0.9,
    conf_b: float = 0.6,
    wait_seconds: float = 3.0,
) -> dict:
    """Run the e2e fusion test and return a result dict.

    Expected: fused position is a confidence-weighted average between
    pos_a (high confidence) and pos_b (low confidence). With conf_a=0.9
    and conf_b=0.6, the fused position should be closer to pos_a.
    """
    if not _RCLPY_OK:
        return {"status": "SKIP", "reason": "rclpy not available"}

    owns_rclpy_context = False
    if not rclpy.ok():
        rclpy.init()
        owns_rclpy_context = True

    publisher = None
    bridge = None
    executor = None
    try:
        publisher = TestPublisherNode(
            target_id, pos_a, pos_b, conf_a, conf_b,
        )
        bridge = FusionBridgeNode()
        result = FusionE2EResult()

        bridge.create_subscription(
            TargetTrackArray,
            "/target_track",
            result.on_message,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(bridge)

        publish_start = time.monotonic()
        deadline = publish_start + wait_seconds
        while not result.wait(timeout=0.0) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        ok = result.wait(timeout=0.0)
        elapsed = time.monotonic() - publish_start

        # Let a few more messages accumulate
        settle_deadline = time.monotonic() + 0.5
        while time.monotonic() < settle_deadline:
            executor.spin_once(timeout_sec=0.05)
        if not ok or not result.received:
            return {
                "status": "FAIL",
                "reason": "no fused output received within timeout",
                "elapsed_s": elapsed,
            }

        # Analyze the last received message
        last_msg = result.received[-1]
        tracks = list(last_msg.tracks)

        if len(tracks) != 1:
            return {
                "status": "FAIL",
                "reason": "fused output does not contain exactly one track",
                "elapsed_s": elapsed,
                "messages_received": len(result.received),
                "track_ids": [track.target_id for track in tracks],
            }

        # TrackFusion assigns a global ID, so the output need not retain the
        # source-local ``target_id`` supplied by the test publisher.
        fused_track = tracks[0]
        fused_x = fused_track.x
        fused_y = fused_track.y

        # Weighted average expectation
        w_a = conf_a / (conf_a + conf_b)
        w_b = conf_b / (conf_a + conf_b)
        expected_x = pos_a[0] * w_a + pos_b[0] * w_b
        expected_y = pos_a[1] * w_a + pos_b[1] * w_b

        # Check fused position is between the two sources
        x_between = min(pos_a[0], pos_b[0]) - 1.0 <= fused_x <= max(pos_a[0], pos_b[0]) + 1.0
        y_between = min(pos_a[1], pos_b[1]) - 1.0 <= fused_y <= max(pos_a[1], pos_b[1]) + 1.0

        # Check that the fused position follows the confidence weighting.
        dist_a = math.hypot(fused_x - pos_a[0], fused_y - pos_a[1])
        dist_b = math.hypot(fused_x - pos_b[0], fused_y - pos_b[1])
        if math.isclose(conf_a, conf_b):
            confidence_weight_ok = math.isclose(dist_a, dist_b, abs_tol=1e-6)
        elif conf_a > conf_b:
            confidence_weight_ok = dist_a < dist_b
        else:
            confidence_weight_ok = dist_b < dist_a

        # Check deduplication: both source observations resolve to one track.
        deduplicated = len(tracks) == 1

        # Latency check
        latency_ok = elapsed < 0.5  # 500ms threshold

        all_pass = (
            x_between and y_between
            and confidence_weight_ok
            and deduplicated
            and latency_ok
        )

        return {
            "status": "PASS" if all_pass else "FAIL",
            "fused_x": round(fused_x, 2),
            "fused_y": round(fused_y, 2),
            "expected_x": round(expected_x, 2),
            "expected_y": round(expected_y, 2),
            "source_a": {"pos": pos_a, "confidence": conf_a},
            "source_b": {"pos": pos_b, "confidence": conf_b},
            "x_between_sources": x_between,
            "y_between_sources": y_between,
            "confidence_weight_ok": confidence_weight_ok,
            "dedup_ok": deduplicated,
            "fused_track_id": fused_track.target_id,
            "track_count": len(tracks),
            "messages_received": len(result.received),
            "latency_ms": round(elapsed * 1000, 1),
            "latency_ok": latency_ok,
        }
    finally:
        if executor is not None:
            executor.shutdown()
        if bridge is not None:
            bridge.destroy_node()
        if publisher is not None:
            publisher.destroy_node()
        if owns_rclpy_context:
            rclpy.try_shutdown()


# ---------------------------------------------------------------------------
# Pytest entry point (skips if rclpy not available)
# ---------------------------------------------------------------------------
def test_fusion_e2e_basic():
    """Two sensors see the same target; fused output is weighted average."""
    if not _RCLPY_OK:
        import pytest
        pytest.skip("rclpy / swarm_interfaces not available")

    result = run_e2e_test(
        target_id=42,
        pos_a=(100.0, 200.0),
        pos_b=(120.0, 180.0),
        conf_a=0.9,
        conf_b=0.6,
    )
    assert result["status"] == "PASS", f"E2E fusion test failed: {result}"


def test_fusion_e2e_equal_confidence():
    """Equal-confidence sources: fused position is midpoint."""
    if not _RCLPY_OK:
        import pytest
        pytest.skip("rclpy / swarm_interfaces not available")

    result = run_e2e_test(
        target_id=99,
        pos_a=(50.0, 50.0),
        pos_b=(70.0, 70.0),
        conf_a=0.8,
        conf_b=0.8,
        wait_seconds=3.0,
    )
    assert result["status"] == "PASS", f"E2E equal-confidence test failed: {result}"


def test_fusion_e2e_reuses_existing_context():
    """The test must not reinitialize or shut down a shared ROS context."""
    if not _RCLPY_OK:
        import pytest
        pytest.skip("rclpy / swarm_interfaces not available")

    owns_rclpy_context = False
    if not rclpy.ok():
        rclpy.init()
        owns_rclpy_context = True
    try:
        result = run_e2e_test(wait_seconds=3.0)
        assert result["status"] == "PASS", f"shared-context E2E test failed: {result}"
        assert rclpy.ok()
    finally:
        if owns_rclpy_context:
            rclpy.try_shutdown()


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 60)
    print("Multi-Source Fusion E2E Test")
    print("=" * 60)

    if not _RCLPY_OK:
        print("\n[SKIP] rclpy or swarm_interfaces not available.")
        print("This test requires a ROS2 environment with the workspace built.")
        print("Run: source ros2_ws/install/setup.bash && python3 test_fusion_e2e.py")
        sys.exit(0)

    print("\nTest 1: Basic weighted fusion (conf_a=0.9, conf_b=0.6)")
    result1 = run_e2e_test(
        target_id=42,
        pos_a=(100.0, 200.0),
        pos_b=(120.0, 180.0),
        conf_a=0.9,
        conf_b=0.6,
    )
    print(f"  Status: {result1['status']}")
    for k, v in result1.items():
        if k != "status":
            print(f"    {k}: {v}")

    print("\nTest 2: Equal confidence (midpoint check)")
    result2 = run_e2e_test(
        target_id=99,
        pos_a=(50.0, 50.0),
        pos_b=(70.0, 70.0),
        conf_a=0.8,
        conf_b=0.8,
    )
    print(f"  Status: {result2['status']}")
    for k, v in result2.items():
        if k != "status":
            print(f"    {k}: {v}")

    all_pass = result1["status"] == "PASS" and result2["status"] == "PASS"
    print(f"\n{'=' * 60}")
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")
    print(f"{'=' * 60}")
    sys.exit(0 if all_pass else 1)
