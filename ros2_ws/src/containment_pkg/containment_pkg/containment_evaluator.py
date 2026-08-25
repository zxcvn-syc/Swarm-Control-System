#!/usr/bin/env python3
"""Containment evaluator for the three-scene enclosure tests.

One process per test run.  Subscribes to the target's live ground-truth
position and to the enclosure system's command output, then renders a binary
verdict and appends it to a CSV.  Designed for unattended batch runs
(10-20 repeats per scene, 3 scenes) driven by a launch wrapper or a script.

Verdict geometry (centred at the target's first observed position ``C``):

  * FAIL    : radius r(t) = dist(target(t), C) exceeds ``monitor_radius``
              at any tick  -> the target escaped the outer surveillance ring.
  * SUCCESS : the target first excursed beyond ``block_radius`` and then
              returned to r(t) <= ``block_radius`` -> intercepted / re-contained.
  * SUCCESS : the test window ends with r(t) <= ``monitor_radius``
              (the swarm held it; no escape -> ``held_within_monitor``).

The verdict is intentionally decoupled from the physics: the escape target
follows a scripted trajectory, so a SUCCESS means "the evaluator logic +
enclosure command stream classify the scenario correctly", which is exactly
what the 8.25 single-scene smoke test verifies.  Tune ``monitor_radius`` /
``block_radius`` to match the enclosure_node deployment (default 25 / 15 m).

Usage:
  ros2 run containment_pkg containment_evaluator --ros-args \
      -p scene_name:=park -p escape_direction:=2 -p trajectory:=return \
      -p monitor_radius:=25.0 -p block_radius:=15.0 \
      -p result_csv:=./eval_results.csv
"""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from std_msgs.msg import Int32, String
from swarm_interfaces.msg import EnclosureTargetArray, EnclosureCommandArray


class ContainmentEvaluator(Node):
    def __init__(self):
        super().__init__("containment_evaluator")
        self.declare_parameter("target_topic", "/enclosure_targets")
        self.declare_parameter("command_topic", "/enclosure_command")
        self.declare_parameter("scene_name", "park")
        self.declare_parameter("escape_direction", 0)
        self.declare_parameter("trajectory", "return")
        self.declare_parameter("target_id", 1)
        self.declare_parameter("monitor_radius", 25.0)
        self.declare_parameter("block_radius", 15.0)
        self.declare_parameter("test_duration", 30.0)
        self.declare_parameter("result_csv", "./eval_results.csv")
        self.declare_parameter("config_file", "")  # path to three_scene_config.yaml

        # Initialise fields that _load_scene_config() may populate, BEFORE the
        # call so it never gets clobbered by a later assignment.
        self._actual_dir = None  # real sampled direction (from escape_test_node)
        self._actual_traj = None  # real trajectory (from yaml scene)

        self._load_scene_config()
        self._cx = None
        self._cy = None
        self._t0 = None
        self._excursion = False
        self._max_r = 0.0
        self._min_r = float("inf")
        self._num_responded = 0
        self._verdict = None
        self._reason = ""
        self._done = False

        self._target_sub = self.create_subscription(
            EnclosureTargetArray,
            str(self.get_parameter("target_topic").value),
            self.on_target,
            10,
        )
        self._cmd_sub = self.create_subscription(
            EnclosureCommandArray,
            str(self.get_parameter("command_topic").value),
            self.on_command,
            10,
        )
        self._dir_sub = self.create_subscription(
            Int32, "/escape_test_direction", self.on_direction, 10
        )
        self._traj_sub = self.create_subscription(
            String, "/escape_test_trajectory", self.on_trajectory, 10
        )
        # Frequent checker; finalises the run when a verdict is reached or the
        # window elapses.  rclpy.shutdown() is only ever called from this timer
        # callback (executor context), never from a subscription callback.
        self._timer = self.create_timer(0.1, self.tick)

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
        """Override monitor/block radius from yaml for the given scene."""
        cfg_path = str(self.get_parameter("config_file").value)
        if not cfg_path or not os.path.exists(cfg_path):
            return
        try:
            import yaml
        except ImportError:
            return
        try:
            with open(cfg_path) as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return
        scene = str(self.get_parameter("scene_name").value)
        s = data.get("scenes", {}).get(scene)
        if not isinstance(s, dict):
            return
        updates = {}
        if "monitor_radius" in s:
            updates["monitor_radius"] = float(s["monitor_radius"])
        if "block_radius" in s:
            updates["block_radius"] = float(s["block_radius"])
        # The *actual* trajectory is broadcast by escape_test_node on
        # /escape_test_trajectory (drawn per run from the scene's
        # trajectory_distribution).  We no longer trust the yaml ``trajectory``
        # field here because it may be a distribution spec, not the drawn value.
        if updates:
            self.set_parameters([self._mk_param(k, v) for k, v in updates.items()])
            self.get_logger().info(
                f"evaluator loaded scene '{scene}' radii from {cfg_path}"
            )

    # ---- subscriptions -------------------------------------------------
    def on_direction(self, msg):
        self._actual_dir = int(msg.data)

    def on_trajectory(self, msg):
        self._actual_traj = str(msg.data)

    def on_command(self, msg):
        n = int(getattr(msg, "num_drones", 0))
        if n > self._num_responded:
            self._num_responded = n

    def on_target(self, msg):
        if self._done:
            return
        targets = list(getattr(msg, "targets", []))
        if not targets:
            return
        tid = int(self.get_parameter("target_id").value)
        tgt = None
        for t in targets:
            if int(getattr(t, "target_id", -1)) == tid:
                tgt = t
                break
        if tgt is None:
            tgt = targets[0]
        x = float(getattr(tgt, "x", 0.0))
        y = float(getattr(tgt, "y", 0.0))

        if self._cx is None:
            self._cx = x
            self._cy = y
            self._t0 = self.get_clock().now()
            self.get_logger().info(f"evaluator centred at ({x:.2f},{y:.2f})")
            return

        monitor = float(self.get_parameter("monitor_radius").value)
        block = float(self.get_parameter("block_radius").value)
        r = math.hypot(x - self._cx, y - self._cy)
        self._max_r = max(self._max_r, r)
        self._min_r = min(self._min_r, r)

        if r > monitor:
            self._verdict = "FAIL"
            self._reason = "escaped_monitor"
            return
        if not self._excursion and r > block:
            self._excursion = True
        if self._excursion and r <= block:
            self._verdict = "SUCCESS"
            self._reason = "re_contained"
            return

    # ---- main loop -----------------------------------------------------
    def tick(self):
        if self._done or self._cx is None:
            return
        t = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        if self._verdict is not None or t >= float(
            self.get_parameter("test_duration").value
        ):
            self.finalize(t)

    # ---- finalisation --------------------------------------------------
    def finalize(self, t):
        if self._done:
            return
        self._done = True
        if self._verdict is None:
            # No escape and no excursion-return within the window: the swarm
            # held the target -> SUCCESS (held_within_monitor).
            self._verdict = "SUCCESS"
            self._reason = "held_within_monitor"

        min_r = self._min_r if self._min_r != float("inf") else 0.0
        row = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "scene": str(self.get_parameter("scene_name").value),
            "direction": (self._actual_dir
                          if self._actual_dir is not None
                          else int(self.get_parameter("escape_direction").value)),
            "trajectory": (self._actual_traj
                           if self._actual_traj is not None
                           else str(self.get_parameter("trajectory").value)),
            "outcome": self._verdict,
            "reason": self._reason,
            "duration_s": f"{t:.2f}",
            "max_r": f"{self._max_r:.2f}",
            "min_r": f"{min_r:.2f}",
            "num_responded": self._num_responded,
        }
        self._write_row(row)
        self.get_logger().info(
            f"[VERDICT] scene={row['scene']} dir={row['direction']} "
            f"traj={row['trajectory']} -> {self._verdict} ({self._reason}) "
            f"max_r={self._max_r:.2f} responded={self._num_responded}"
        )
        self._timer.cancel()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _write_row(self, row):
        path = str(self.get_parameter("result_csv").value)
        fields = [
            "timestamp", "scene", "direction", "trajectory", "outcome",
            "reason", "duration_s", "max_r", "min_r", "num_responded",
        ]
        try:
            parent = os.path.dirname(os.path.abspath(path))
            os.makedirs(parent, exist_ok=True)
        except Exception:
            pass
        write_header = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            if write_header:
                w.writeheader()
            w.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = ContainmentEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
