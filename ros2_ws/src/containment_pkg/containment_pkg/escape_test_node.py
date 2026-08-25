#!/usr/bin/env python3
"""Programmable escape-target simulator for three-scene enclosure tests.

Publishes a single moving target on ``/enclosure_targets`` (EnclosureTargetArray)
so that ``enclosure_node`` encloses it and ``containment_evaluator`` judges
whether the swarm contained or lost the target.

Trajectory modes (controlled by the ``trajectory`` parameter):

  * ``straight``  : constant velocity along the chosen 8-direction.
                    Models a *full escape* -> evaluator should verdict FAIL.
  * ``return``    : moves outward to ``return_peak`` (default 20 m, < monitor 25 m)
                    then reverses back to the start.
                    Models a *successful containment* -> evaluator should verdict
                    SUCCESS (excursion beyond block ring, then re-centred).
  * ``oscillate`` : repeatedly moves out to ``oscillate_radius`` and back, never
                    leaving the inner ring -> evaluator should verdict SUCCESS
                    (held within monitor for the whole window).

Direction is one of 8 compass directions (0=E, 1=NE, 2=N, 3=NW, 4=W, 5=SW,
6=S, 7=SE) or ``random`` (one draw at startup, optionally weighted by
``dir_probs``).  For the 8.25 single-scene smoke test, set ``escape_direction``
and ``trajectory`` explicitly; the 8.26 ``three_scene_config.yaml`` drives the
per-scene distribution and the batch runner.

Usage:
  ros2 run containment_pkg escape_test_node --ros-args \
      -p scene_name:=park -p escape_direction:=2 \
      -p trajectory:=return -p speed:=2.0 -p test_duration:=20.0
"""

import math
import os
import random

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from std_msgs.msg import Header, Int32, String
from swarm_interfaces.msg import EnclosureTarget, EnclosureTargetArray


# Compass directions: index -> angle (radians, 0 = +X / East, CCW positive).
DIRECTION_ANGLES = {
    0: 0.0,               # E
    1: math.pi / 4,       # NE
    2: math.pi / 2,       # N
    3: 3 * math.pi / 4,   # NW
    4: math.pi,           # W
    5: 5 * math.pi / 4,   # SW
    6: 3 * math.pi / 2,   # S
    7: 7 * math.pi / 4,   # SE
}


class EscapeTestNode(Node):
    def __init__(self):
        super().__init__("escape_test_node")
        self.declare_parameter("period", 0.1)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("scene_name", "park")
        self.declare_parameter("escape_direction", 0)   # 0-7 or "random"
        self.declare_parameter(
            "dir_probs",
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter("trajectory", "return")  # straight|return|oscillate
        self.declare_parameter("speed", 2.0)            # m/s (straight mode)
        self.declare_parameter("return_peak", 20.0)     # m (return mode peak)
        self.declare_parameter("return_period", 6.0)    # s for one out-and-back
        self.declare_parameter("oscillate_radius", 12.0)  # m (oscillate mode)
        self.declare_parameter("oscillate_period", 6.0)   # s
        self.declare_parameter("target_id", 1)
        self.declare_parameter("test_duration", 30.0)
        self.declare_parameter("topic", "/enclosure_targets")
        self.declare_parameter("config_file", "")  # path to three_scene_config.yaml

        self._dir_probs = list(self.get_parameter("dir_probs").value)
        self._actual_traj = str(self.get_parameter("trajectory").value)
        # Load per-scene overrides from yaml (if config_file set & scene found).
        self._load_scene_config()
        self._cx = float(self.get_parameter("start_x").value)
        self._cy = float(self.get_parameter("start_y").value)
        self._t0 = None
        self._dir = self._pick_direction()
        self._theta = DIRECTION_ANGLES[self._dir]
        self._publisher = self.create_publisher(
            EnclosureTargetArray, str(self.get_parameter("topic").value), 10
        )
        # Broadcast the *actual* sampled direction so the evaluator can record
        # it (the launch only passes the requested -1 when sampling).  Re-publish
        # on every tick: a single early publish is dropped because the
        # evaluator's subscription is not matched yet at node construction.
        self._dir_pub = self.create_publisher(Int32, "/escape_test_direction", 10)
        self._dir_msg = Int32()
        self._dir_msg.data = int(self._dir)
        # Broadcast the *actual* sampled trajectory so the evaluator records
        # it (mirrors /escape_test_direction; republished every tick so the
        # late subscriber never misses the first value).
        self._traj_pub = self.create_publisher(
            String, "/escape_test_trajectory", 10
        )
        self._traj_msg = String()
        self._traj_msg.data = self._actual_traj
        period = max(float(self.get_parameter("period").value), 0.02)
        self._timer = self.create_timer(period, self.publish)
        self._frame = 0
        self.get_logger().info(
            f"escape_test_node scene={self.get_parameter('scene_name').value} "
            f"dir={self._dir} ({math.degrees(self._theta):.0f}deg) "
            f"traj={self.get_parameter('trajectory').value}"
        )

    def _pick_direction(self):
        raw = self.get_parameter("escape_direction").value
        if isinstance(raw, str):
            if raw.lower() in ("random", "-1"):
                return int(random.choices(range(8), weights=self._dir_probs, k=1)[0])
            try:
                raw = int(raw)
            except ValueError:
                raw = 0
        raw = int(raw)
        if raw == -1:
            return int(random.choices(range(8), weights=self._dir_probs, k=1)[0])
        return raw % 8

    def _pick_trajectory(self):
        """Draw a trajectory from ``self._traj_probs`` if configured, else
        fall back to the ``trajectory`` parameter value."""
        probs = getattr(self, "_traj_probs", None)
        if probs:
            choices = list(probs.keys())
            weights = list(probs.values())
            return str(random.choices(choices, weights=weights, k=1)[0])
        return str(self.get_parameter("trajectory").value)

    @staticmethod
    def _mk_param(name, value):
        if isinstance(value, bool):
            return Parameter(name, Parameter.Type.BOOL, value)
        if isinstance(value, int):
            return Parameter(name, Parameter.Type.INTEGER, value)
        if isinstance(value, float):
            return Parameter(name, Parameter.Type.DOUBLE, value)
        return Parameter(name, Parameter.Type.STRING, str(value))

    def _load_scene_config(self):
        """Override start / speed / trajectory / dir distribution from yaml."""
        cfg_path = str(self.get_parameter("config_file").value)
        if not cfg_path:
            return
        if not os.path.exists(cfg_path):
            self.get_logger().warn(
                f"config_file not found: {cfg_path}; using launch params"
            )
            return
        try:
            import yaml
        except ImportError:
            self.get_logger().warn("PyYAML not installed; ignoring config_file")
            return
        with open(cfg_path) as f:
            data = yaml.safe_load(f) or {}
        scene = str(self.get_parameter("scene_name").value)
        scenes = data.get("scenes", {})
        if scene not in scenes:
            self.get_logger().warn(
                f"scene '{scene}' not in {cfg_path}; using launch params"
            )
            return
        s = scenes[scene]
        updates = {}
        start = s.get("start")
        if isinstance(start, (list, tuple)) and len(start) == 2:
            updates["start_x"] = float(start[0])
            updates["start_y"] = float(start[1])
        if "speed" in s:
            updates["speed"] = float(s["speed"])
        if "trajectory" in s:
            updates["trajectory"] = str(s["trajectory"])
        if "test_duration" in s:
            updates["test_duration"] = float(s["test_duration"])
        if updates:
            self.set_parameters(
                [self._mk_param(k, v) for k, v in updates.items()]
            )
        # Optional per-run trajectory distribution -> draw one trajectory for
        # this run, so the 60-test batch yields a real success-rate spread
        # instead of a fixed outcome per scene.
        self._traj_probs = {}
        td = s.get("trajectory_distribution")
        if isinstance(td, dict) and td:
            for k, v in td.items():
                w = float(v)
                if w > 0:
                    self._traj_probs[str(k)] = w
        if self._traj_probs:
            self._actual_traj = self._pick_trajectory()
            self.set_parameters([self._mk_param("trajectory", self._actual_traj)])
        # 8-direction weighted distribution -> dir_probs (index order 0..7).
        idx_map = {"E": 0, "NE": 1, "N": 2, "NW": 3, "W": 4, "SW": 5, "S": 6, "SE": 7}
        dd = s.get("direction_distribution")
        if isinstance(dd, dict) and dd:
            probs = [0.0] * 8
            for k, v in dd.items():
                if k in idx_map:
                    probs[idx_map[k]] = float(v)
            if sum(probs) > 0:
                self._dir_probs = probs
        self.get_logger().info(f"loaded scene '{scene}' from {cfg_path}")

    def _target_xy(self, t):
        """Return (x, y, speed) of the target at elapsed time ``t`` (seconds)."""
        mode = str(self.get_parameter("trajectory").value)
        if mode == "straight":
            speed = float(self.get_parameter("speed").value)
            r = speed * t
            return (
                self._cx + r * math.cos(self._theta),
                self._cy + r * math.sin(self._theta),
                speed,
            )
        if mode == "return":
            peak = float(self.get_parameter("return_peak").value)
            T = max(float(self.get_parameter("return_period").value), 0.5)
            r = peak * (1.0 - math.cos(math.pi * t / T)) / 2.0
            drdt = peak * math.pi / (2.0 * T) * math.sin(math.pi * t / T)
            return (
                self._cx + r * math.cos(self._theta),
                self._cy + r * math.sin(self._theta),
                abs(drdt),
            )
        # oscillate (default)
        amp = float(self.get_parameter("oscillate_radius").value)
        T = max(float(self.get_parameter("oscillate_period").value), 0.5)
        r = amp * (1.0 - math.cos(2.0 * math.pi * t / T)) / 2.0
        drdt = amp * math.pi / T * math.sin(2.0 * math.pi * t / T)
        return (
            self._cx + r * math.cos(self._theta),
            self._cy + r * math.sin(self._theta),
            abs(drdt),
        )

    def publish(self):
        now = self.get_clock().now()
        if self._t0 is None:
            self._t0 = now
        t = (now - self._t0).nanoseconds / 1e9
        x, y, speed = self._target_xy(t)

        msg = EnclosureTargetArray()
        msg.header = Header()
        msg.header.stamp = now.to_msg()
        msg.frame_idx = self._frame
        self._frame += 1
        tgt = EnclosureTarget()
        tgt.target_id = int(self.get_parameter("target_id").value)
        tgt.x = float(x)
        tgt.y = float(y)
        tgt.speed = float(speed)
        tgt.motion_mode = 3 if speed > 1.5 else (2 if speed > 0.3 else 1)
        tgt.confidence = 0.95
        msg.targets = [tgt]

        self._publisher.publish(msg)
        self._dir_pub.publish(self._dir_msg)
        self._traj_pub.publish(self._traj_msg)
        self.get_logger().debug(
            f"t={t:.2f} target=({x:.2f},{y:.2f}) "
            f"r={math.hypot(x - self._cx, y - self._cy):.2f}"
        )

        # Stop publishing once the scripted trajectory has played out so the
        # evaluator can finalise; keep the node alive (harmless).
        if t >= float(self.get_parameter("test_duration").value):
            self._timer.cancel()
            self.get_logger().info(
                "escape_test_node: test_duration reached, stopped publishing"
            )


def main(args=None):
    rclpy.init(args=args)
    node = EscapeTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
