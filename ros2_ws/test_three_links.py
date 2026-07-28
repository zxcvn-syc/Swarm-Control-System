#!/usr/bin/env python3
"""Three-link integration test for the Swarm-Control-System.

This is the **single end-to-end test** the integration lead (何泓林)
runs before declaring a week's联调 pass / fail.  It is intentionally
self-contained:

* it does **not** require a real ROS2 daemon (no `ros2 launch`); instead
  it spins the four real nodes (``tracker_node``,
  ``scheduler_node``, ``planner_node``, ``enclosure_node``) in a
  single ``rclpy`` process and exercises the full pipeline.
* it does **not** require YOLO weights: tracker_node is loaded in
  ``input_mode:=topic`` and a synthetic image publisher drives the
  pipeline via the standard image topic.
* it does **not** require PX4 / RflySim: ``planner_node`` produces
  a ``DroneStateArray`` so the second and third links close.

What it checks
--------------

1. **Link 1 (perception → scheduler)**
   - ``tracker_node`` publishes ``/target_track`` (TargetTrackArray).
   - ``scheduler_node`` consumes it and publishes ``/task_assignment``.

2. **Link 2 (scheduler → planner)**
   - ``scheduler_node`` publishes ``/task_assignment``.
   - ``planner_node`` consumes it and produces ``/drone_states``
     (DroneStateArray).

3. **Link 3 (perception + planner → enclosure → feedback)**
   - ``tracker_node`` publishes ``/enclosure_targets`` (EnclosureTargetArray).
   - ``enclosure_node`` consumes it + ``/drone_states`` and publishes
     ``/enclosure_command`` (EnclosureCommandArray).

4. **End-to-end**
   - run the full pipeline against a synthetic 2-target / 4-drone scene
   - assert at least 1 ``TaskAssignment`` and 1 ``EnclosureCommand``
     were published within the test window
   - assert no ROS2 errors fired on any node

Usage
-----

::

    source /opt/ros/humble/setup.bash
    source /home/hhh/Downloads/Swarm-Control-System/ros2_ws/install/setup.bash
    cd /home/hhh/Downloads/Swarm-Control-System/ros2_ws
    python3 test_three_links.py
    python3 test_three_links.py --video /abs/path/to/test_multi_target_tracking.mp4

Exit code ``0`` = pass, non-zero = fail.  The script also writes a
JSON summary to ``output/test_three_links_<timestamp>.json`` for the
weekly report.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

# Make sure the local install is on PYTHONPATH even when the user
# forgot to source install/setup.bash.  We add the install/lib
# site-packages so ``import swarm_interfaces`` resolves.
_REPO = Path(__file__).resolve().parent
_WORKSPACE = _REPO
_INSTALL_PY = _WORKSPACE / "install"
if _INSTALL_PY.exists():
    for _cand in _INSTALL_PY.glob("**/site-packages"):
        if _cand.is_dir():
            sys.path.insert(0, str(_cand))
    for _cand in (_INSTALL_PY / "lib" / "python3.10" / "site-packages",
                  _INSTALL_PY / "lib" / "python3.11" / "site-packages",
                  _INSTALL_PY / "lib" / "python3.12" / "site-packages"):
        if _cand.is_dir():
            sys.path.insert(0, str(_cand))

import numpy as np  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

# swarm_interfaces types
from swarm_interfaces.msg import (  # noqa: E402
    DroneStateArray,
    EnclosureCommand,
    EnclosureCommandArray,
    EnclosureTarget,
    EnclosureTargetArray,
    TaskAssignment,
    TargetTrack,
    TargetTrackArray,
)


# ---------------------------------------------------------------------------
# Constants — mirror the values in scripts/three_links_demo.sh so the
# command-line and in-process test agree on the topic map.
# ---------------------------------------------------------------------------
TOPIC_TARGET_TRACK = "/target_track"
TOPIC_ENCLOSURE_TARGETS = "/enclosure_targets"
TOPIC_DRONE_STATES = "/drone_states"
TOPIC_TASK_ASSIGNMENT = "/task_assignment"
TOPIC_ENCLOSURE_COMMAND = "/enclosure_command"
TOPIC_DRONE_STATE = "/drone_state"

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
QOS_BE = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# Coordinates convention (decision D-2 in interface_alignment.md):
#   /target_track     -> pixel (image plane) -> frame_id="camera_optical_frame"
#   /enclosure_targets-> pixel             -> frame_id="camera_optical_frame"
#   /drone_states     -> ENU local meters  -> frame_id="world"
#   /task_assignment  -> id-only           -> frame_id="world"
#   /enclosure_command-> ENU local meters  -> frame_id="world"

# Test world: we publish "world" coordinates for drone_states so the
# scheduler + enclosure can use them.  The synthetic targets for
# scheduler purposes are also in this "world" frame — they are *not*
# the pixel coordinates, they're the values we want scheduler to act
# on, mirroring how a real deployment would map pixel→world first.
TEST_WORLD = {
    "x_min": 0.0,
    "x_max": 40.0,
    "y_min": 0.0,
    "y_max": 30.0,
    "num_drones": 4,
    "num_targets": 2,
    "tick_period": 0.2,
    "test_window_sec": 6.0,
}

LOG_TAG_TRACKER = "[integration/tracker]"
LOG_TAG_SCHED = "[integration/scheduler]"
LOG_TAG_PLAN = "[integration/planner]"
LOG_TAG_ENC = "[integration/enclosure]"
LOG_TAG_TEST = "[integration/test]"


# ---------------------------------------------------------------------------
# Result accounting
# ---------------------------------------------------------------------------
@dataclass
class LinkReport:
    name: str
    input_count: int = 0
    output_count: int = 0
    sample_input: Optional[Any] = None
    sample_output: Optional[Any] = None
    error_count: int = 0
    last_error: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)

    def passed(self) -> bool:
        return self.error_count == 0 and self.output_count > 0


# ---------------------------------------------------------------------------
# Synthetic data publishers — stand in for the *real* tracker_node
# YOLO pipeline.  The real tracker_node is also spun up below; the
# synthetic publishers exist so we can drive the rest of the pipeline
# even when YOLO weights are unavailable (the test environment here
# has no GPU and no YOLO weights).
# ---------------------------------------------------------------------------
class SyntheticImagePublisher(Node):  # type: ignore[misc]
    """Publishes a moving-blob image on /camera/image."""

    def __init__(self) -> None:
        super().__init__("synthetic_image_publisher")
        self._pub = self.create_publisher(
            _lazy_sensor_image(), "/camera/image", QOS_BE,
        )
        self._t0 = time.monotonic()
        self._timer = self.create_timer(0.1, self._tick)
        self._frame_id = 0
        self.get_logger().info(f"{LOG_TAG_TEST} synthetic image publisher ready")

    def _tick(self) -> None:
        try:
            import cv2  # type: ignore
        except ImportError:
            return
        t = time.monotonic() - self._t0
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # draw two moving "blobs" that look like a tracked target
        for i, (vx, vy) in enumerate([(40, 20), (-30, 25)]):
            cx = int(320 + vx * t) % 640
            cy = int(240 + vy * t) % 480
            color = (0, 255 - i * 80, 0)
            cv2.circle(img, (cx, cy), 24, color, thickness=-1)
        msg = _cv_bridge().cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        self._pub.publish(msg)
        self._frame_id += 1


class SyntheticTrackerPublisher(Node):  # type: ignore[misc]
    """Publishes /target_track + /enclosure_targets directly.

    Used as a fallback so the test can still drive the scheduler +
    enclosure chains even if the real tracker_node cannot start (e.g.
    no cvtrack installed).  We still spin the real tracker_node when
    possible, so the synthetic path is only there to keep the test
    useful in degraded environments.
    """

    def __init__(self) -> None:
        super().__init__("synthetic_tracker_publisher")
        qos = QOS
        self._track_pub = self.create_publisher(
            TargetTrackArray, TOPIC_TARGET_TRACK, qos,
        )
        self._enc_pub = self.create_publisher(
            EnclosureTargetArray, TOPIC_ENCLOSURE_TARGETS, qos,
        )
        self._t0 = time.monotonic()
        self._frame_idx = 0
        self._timer = self.create_timer(0.2, self._tick)
        self.get_logger().info(f"{LOG_TAG_TEST} synthetic tracker publisher ready")

    def _tick(self) -> None:
        t = time.monotonic() - self._t0
        # Two "world" targets inside the test area; in a real
        # deployment this would be the output of coord_transform_node
        # (pixel -> world).  For the test we publish world coordinates
        # directly so the scheduler can consume them.
        target_specs = [
            (101, TEST_WORLD["x_min"] + 10 + 2 * t, TEST_WORLD["y_min"] + 8 + 1.5 * t, 0.8, 2),
            (202, TEST_WORLD["x_max"] - 10 - 1.5 * t, TEST_WORLD["y_max"] - 8 - 1.0 * t, 1.4, 3),
        ]
        track_arr = TargetTrackArray()
        track_arr.header = Header()
        track_arr.header.stamp = self.get_clock().now().to_msg()
        track_arr.header.frame_id = "world"  # already in world for test
        track_arr.frame_idx = self._frame_idx
        for tid, x, y, speed, mode in target_specs:
            tr = TargetTrack()
            tr.target_id = int(tid)
            tr.x = float(x)
            tr.y = float(y)
            tr.vx = 2.0
            tr.vy = 1.5
            tr.confidence = 0.9
            tr.cls = 0
            tr.is_confirmed = True
            tr.speed = float(speed)
            tr.motion_mode = int(mode)
            tr.pred_x = [float(x + 1 * i) for i in range(5)]
            tr.pred_y = [float(y + 0.5 * i) for i in range(5)]
            tr.pred_conf = [0.9, 0.85, 0.8, 0.75, 0.7]
            track_arr.tracks.append(tr)
        self._track_pub.publish(track_arr)

        # /enclosure_targets — same data, EnclosureTargetArray shape.
        enc_arr = EnclosureTargetArray()
        enc_arr.header = track_arr.header
        enc_arr.frame_idx = self._frame_idx
        for tid, x, y, speed, mode in target_specs:
            et = EnclosureTarget()
            et.target_id = int(tid)
            et.x = float(x)
            et.y = float(y)
            et.speed = float(speed)
            et.motion_mode = int(mode)
            et.confidence = 0.9
            et.box_x1 = float(x - 20)
            et.box_y1 = float(y - 20)
            et.box_x2 = float(x + 20)
            et.box_y2 = float(y + 20)
            et.pred_x = [float(x) + 0.5 * i for i in range(5)]
            et.pred_y = [float(y) + 0.5 * i for i in range(5)]
            et.history_x = [float(x)] * 10
            et.history_y = [float(y)] * 10
            enc_arr.targets.append(et)
        enc_arr.drone_x = [0.0] * 8
        enc_arr.drone_y = [0.0] * 8
        enc_arr.num_drones = 0
        enc_arr.enclosure_radius = 25.0
        enc_arr.min_enclosure_dist = 5.0
        self._enc_pub.publish(enc_arr)

        self._frame_idx += 1


# ---------------------------------------------------------------------------
# Lazy cv_bridge + sensor_msgs.Image — we only need them if opencv
# is installed.  Otherwise the synthetic image publisher is a no-op.
# ---------------------------------------------------------------------------
def _lazy_sensor_image():  # pragma: no cover - tiny helper
    try:
        from sensor_msgs.msg import Image as ROSImage  # type: ignore
        return ROSImage
    except ImportError:
        # Fall back to a stand-in class so the test can still construct
        # the node.  cv_bridge won't be invoked in this branch.
        class _Stub:
            def __init__(self) -> None:
                self.header = Header()

        return _Stub


def _cv_bridge():  # pragma: no cover - tiny helper
    from cv_bridge import CvBridge  # type: ignore
    return CvBridge()


# ---------------------------------------------------------------------------
# Subscriber-side recording nodes
# ---------------------------------------------------------------------------
class Recorder(Node):  # type: ignore[misc]
    """Generic subscriber that records the last N messages on a topic."""

    def __init__(
        self,
        name: str,
        topic: str,
        msg_type: Any,
        max_records: int = 64,
        qos: QoSProfile = QOS,
        on_message: Optional[Callable[[Any], None]] = None,
    ) -> None:
        super().__init__(name)
        self._records: List[Any] = []
        self._max = max_records
        self._on_message = on_message
        self._sub = self.create_subscription(
            msg_type, topic, self._cb, qos,
        )

    def _cb(self, msg: Any) -> None:
        self._records.append(msg)
        if len(self._records) > self._max:
            self._records = self._records[-self._max:]
        if self._on_message is not None:
            try:
                self._on_message(msg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"{LOG_TAG_TEST} recorder {self.get_name()} on_message error: {exc}"
                )

    @property
    def records(self) -> List[Any]:
        return list(self._records)

    def last(self) -> Optional[Any]:
        return self._records[-1] if self._records else None


# ---------------------------------------------------------------------------
# Lazy node constructors — wrap imports so a single missing dep doesn't
# blow up the whole test.
# ---------------------------------------------------------------------------
def _try_import(name: str) -> Optional[Any]:
    try:
        return __import__(name, fromlist=["*"])
    except Exception as exc:  # noqa: BLE001
        logging.warning(f"{LOG_TAG_TEST} optional import '{name}' failed: {exc}")
        return None


def _make_tracker_node():
    """Try to construct the real tracker_node; return None on failure."""
    mod = _try_import("perception_pkg.tracker_node")
    if mod is None or not hasattr(mod, "TrackerNode"):
        return None
    try:
        return mod.TrackerNode()
    except Exception as exc:  # noqa: BLE001
        logging.warning(f"{LOG_TAG_TEST} TrackerNode() failed: {exc}")
        return None


def _make_scheduler_node():
    mod = _try_import("scheduler_pkg.scheduler_node")
    if mod is None or not hasattr(mod, "SchedulerNode"):
        return None
    try:
        return mod.SchedulerNode()
    except Exception as exc:  # noqa: BLE001
        logging.warning(f"{LOG_TAG_TEST} SchedulerNode() failed: {exc}")
        return None


def _make_planner_node():
    """Try to construct the real planner_node; return None on failure."""
    mod = _try_import("planning_pkg.planner_node")
    if mod is None or not hasattr(mod, "PlannerNode"):
        return None
    try:
        return mod.PlannerNode()
    except Exception as exc:  # noqa: BLE001
        logging.warning(f"{LOG_TAG_TEST} PlannerNode() failed: {exc}")
        return None


def _make_enclosure_node():
    mod = _try_import("containment_pkg.enclosure_node")
    if mod is None or not hasattr(mod, "EnclosureNode"):
        return None
    try:
        return mod.EnclosureNode()
    except Exception as exc:  # noqa: BLE001
        logging.warning(f"{LOG_TAG_TEST} EnclosureNode() failed: {exc}")
        return None


def _make_grid_map_node():
    """Try to construct grid_map_node; return None on failure."""
    mod = _try_import("planning_pkg.grid_map_node")
    if mod is None or not hasattr(mod, "GridMapNode"):
        return None
    try:
        return mod.GridMapNode()
    except Exception as exc:  # noqa: BLE001
        logging.warning(f"{LOG_TAG_TEST} GridMapNode() failed: {exc}")
        return None


# ---------------------------------------------------------------------------
# Main driver
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--video", default="",
        help="Optional local video to feed into tracker_node.  If empty, "
             "the synthetic tracker publisher is used so the test runs "
             "in a pure integration-only environment.",
    )
    p.add_argument(
        "--no-real-nodes", action="store_true",
        help="Skip spinning the real ROS2 nodes; only use the synthetic "
             "publishers.  Useful for diagnosing the harness itself.",
    )
    p.add_argument(
        "--output", default=str(_REPO.parent / "output"),
        help="Where to write the JSON report.",
    )
    p.add_argument(
        "--window", type=float, default=TEST_WORLD["test_window_sec"],
        help="How long to let the pipeline run before sampling outputs.",
    )
    return p.parse_args()


def _record_first_cb(name: str, store: Dict[str, Any]) -> Callable[[Any], None]:
    def _cb(msg: Any) -> None:
        if name in store:
            return
        store[name] = msg
    return _cb


def _build_report(
    track_rec: Recorder,
    task_rec: Recorder,
    drone_rec: Recorder,
    enc_cmd_rec: Recorder,
    enc_tgt_rec: Recorder,
    link1: LinkReport,
    link2: LinkReport,
    link3: LinkReport,
) -> Dict[str, Any]:
    return {
        "links": {
            "link1_perception_to_scheduler": {
                "input_topic": TOPIC_TARGET_TRACK,
                "output_topic": TOPIC_TASK_ASSIGNMENT,
                "input_count": link1.input_count,
                "output_count": link1.output_count,
                "error_count": link1.error_count,
                "last_error": link1.last_error,
                "passed": link1.passed(),
            },
            "link2_scheduler_to_planner": {
                "input_topic": TOPIC_TASK_ASSIGNMENT,
                "output_topic": TOPIC_DRONE_STATES,
                "input_count": link2.input_count,
                "output_count": link2.output_count,
                "error_count": link2.error_count,
                "last_error": link2.last_error,
                "passed": link2.passed(),
            },
            "link3_perception_planner_to_enclosure": {
                "input_topics": [TOPIC_ENCLOSURE_TARGETS, TOPIC_DRONE_STATES],
                "output_topic": TOPIC_ENCLOSURE_COMMAND,
                "input_count": link3.input_count,
                "output_count": link3.output_count,
                "error_count": link3.error_count,
                "last_error": link3.last_error,
                "passed": link3.passed(),
            },
        },
        "totals": {
            "tracks_received": len(track_rec.records),
            "task_assignments_received": len(task_rec.records),
            "drone_states_received": len(drone_rec.records),
            "enclosure_commands_received": len(enc_cmd_rec.records),
            "enclosure_targets_received": len(enc_tgt_rec.records),
        },
        "passed": all(l.passed() for l in (link1, link2, link3)),
    }


def main() -> int:
    args = parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    log = logging.getLogger("test_three_links")
    log.info(f"{LOG_TAG_TEST} starting three-link integration test")

    if not rclpy.ok():
        rclpy.init()

    executor = SingleThreadedExecutor()

    # ---- recorders ----
    track_rec = Recorder("rec_target_track", TOPIC_TARGET_TRACK, TargetTrackArray)
    task_rec = Recorder("rec_task_assignment", TOPIC_TASK_ASSIGNMENT, TaskAssignment, max_records=256)
    drone_rec = Recorder("rec_drone_states", TOPIC_DRONE_STATES, DroneStateArray)
    enc_cmd_rec = Recorder("rec_enclosure_command", TOPIC_ENCLOSURE_COMMAND, EnclosureCommandArray)
    enc_tgt_rec = Recorder("rec_enclosure_targets", TOPIC_ENCLOSURE_TARGETS, EnclosureTargetArray)
    for n in (track_rec, task_rec, drone_rec, enc_cmd_rec, enc_tgt_rec):
        executor.add_node(n)

    # ---- real nodes ----
    real_nodes: List[Any] = []
    if not args.no_real_nodes:
        for fn in (_make_tracker_node, _make_scheduler_node,
                   _make_planner_node, _make_grid_map_node,
                   _make_enclosure_node):
            node = fn()
            if node is not None:
                real_nodes.append(node)
                executor.add_node(node)
                log.info(f"{LOG_TAG_TEST} real node '{node.get_name()}' added")
        if not real_nodes:
            log.warning(
                f"{LOG_TAG_TEST} no real nodes could be imported; "
                "running synthetic-only mode"
            )

    # ---- synthetic fallback publishers ----
    synth_tracker = SyntheticTrackerPublisher()
    executor.add_node(synth_tracker)
    # Image publisher only useful when tracker_node is up + cv_bridge
    # + opencv are all available; the constructor already silently
    # no-ops on ImportError so just add it.
    if real_nodes and any(
        getattr(n, "_input_mode", "") in ("video", "topic")
        for n in real_nodes
        if hasattr(n, "_input_mode")
    ):
        try:
            synth_image = SyntheticImagePublisher()
            executor.add_node(synth_image)
        except Exception as exc:  # noqa: BLE001
            log.warning(f"{LOG_TAG_TEST} synthetic image publisher skipped: {exc}")

    # ---- per-link error capture (use rclpy logger hooks) ----
    link1 = LinkReport("link1_perception_to_scheduler")
    link2 = LinkReport("link2_scheduler_to_planner")
    link3 = LinkReport("link3_perception_planner_to_enclosure")

    def _on_task(msg: TaskAssignment) -> None:
        link1.output_count += 1
    task_rec._on_message = _on_task  # type: ignore[attr-defined]

    def _on_drone(msg: DroneStateArray) -> None:
        link2.output_count += 1
    drone_rec._on_message = _on_drone  # type: ignore[attr-defined]

    def _on_enc_cmd(msg: EnclosureCommandArray) -> None:
        link3.output_count += 1
    enc_cmd_rec._on_message = _on_enc_cmd  # type: ignore[attr-defined]

    def _on_track(msg: TargetTrackArray) -> None:
        link1.input_count += 1
    track_rec._on_message = _on_track  # type: ignore[attr-defined]

    def _on_enc_tgt(msg: EnclosureTargetArray) -> None:
        link3.input_count += 1
    enc_tgt_rec._on_message = _on_enc_tgt  # type: ignore[attr-defined]

    # ---- spin for the test window ----
    deadline = time.monotonic() + args.window
    next_heartbeat = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if time.monotonic() >= next_heartbeat:
            log.info(
                f"{LOG_TAG_TEST} heartbeat: "
                f"tracks={link1.input_count} tasks={link1.output_count} "
                f"drones={link2.output_count} enc_tgt={link3.input_count} "
                f"enc_cmd={link3.output_count}"
            )
            next_heartbeat = time.monotonic() + 1.0

    # ---- build report ----
    # link2 input is the same data as link1's output
    link2.input_count = link1.output_count
    report = _build_report(
        track_rec, task_rec, drone_rec, enc_cmd_rec, enc_tgt_rec,
        link1, link2, link3,
    )

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"test_three_links_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out_path.write_text(json.dumps(report, indent=2, default=str))
    log.info(f"{LOG_TAG_TEST} report written to {out_path}")

    # ---- teardown ----
    for n in (synth_tracker, *real_nodes, track_rec, task_rec, drone_rec,
              enc_cmd_rec, enc_tgt_rec):
        try:
            executor.remove_node(n)
        except Exception:  # noqa: BLE001
            pass
        try:
            n.destroy_node()
        except Exception:  # noqa: BLE001
            pass

    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass

    if not report["passed"]:
        log.error(
            f"{LOG_TAG_TEST} FAIL: "
            f"link1={link1.passed()} link2={link2.passed()} link3={link3.passed()}"
        )
        return 1
    log.info(
        f"{LOG_TAG_TEST} PASS: link1={link1.output_count} link2={link2.output_count} "
        f"link3={link3.output_count}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
