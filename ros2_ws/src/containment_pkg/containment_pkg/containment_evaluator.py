#!/usr/bin/env python3
"""Containment evaluator for the three-scene enclosure tests (8.26 revision).

One process per test run.  Subscribes to:

  * ``/enclosure_targets``  -- the escape target's live ground-truth position
  * ``/enclosure_command``  -- enclosure_node's command stream (only used to
                               record ``num_responded``; NOT part of the verdict)
  * ``/drone_states``       -- the **platform** swarm state; this is the
                               *response evidence* the 8.26 gate requires
  * ``/escape_test_direction`` / ``/escape_test_trajectory`` -- the actual
                               sampled direction / trajectory (broadcast by
                               escape_test_node so the verdict row records what
                               really happened, not the launch request)

Verdict (8.26 response-evidence gate)
-------------------------------------
The verdict is no longer "does the target come back on its own".  A run must
satisfy BOTH:

  * the target was contained (it excursed beyond ``block_radius`` and returned,
    or it was held inside the monitor ring for the whole window); AND
  * at least one platform came within ``intercept_radius`` (default 5 m) of the
    target at some tick -- proof the swarm actually engaged.

Outcomes:

  * FAIL     : the target broke out past ``monitor_radius`` (escaped).
  * SUCCESS  : contained AND a platform was within ``intercept_radius``.
  * INVALID  : contained BUT no platform ever got that close -> the containment
               cannot be attributed to the system.  Reported separately; it
               counts neither as success nor as failure in the success rate.

This makes the success rate a property of the *system* (platforms must be
present and close enough), not of the scripted trajectory distribution.

Usage:
  ros2 run containment_pkg containment_evaluator --ros-args \
      -p scene_name:=park -p escape_direction:=2 -p trajectory:=return \
      -p monitor_radius:=25.0 -p block_radius:=15.0 -p intercept_radius:=5.0 \
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
from swarm_interfaces.msg import EnclosureTargetArray, EnclosureCommandArray, DroneStateArray

from .verdict import decide_verdict


class ContainmentEvaluator(Node):
    def __init__(self):
        super().__init__("containment_evaluator")
        self.declare_parameter("target_topic", "/enclosure_targets")
        self.declare_parameter("command_topic", "/enclosure_command")
        self.declare_parameter("drone_states_topic", "/drone_states")
        self.declare_parameter("scene_name", "park")
        self.declare_parameter("escape_direction", 0)
        self.declare_parameter("trajectory", "return")
        self.declare_parameter("target_id", 1)
        self.declare_parameter("monitor_radius", 25.0)
        self.declare_parameter("block_radius", 15.0)
        self.declare_parameter("intercept_radius", 5.0)
        self.declare_parameter("test_duration", 30.0)
        self.declare_parameter("result_csv", "./eval_results.csv")
        self.declare_parameter("config_file", "")  # path to three_scene_config.yaml

        # Fields populated by _load_scene_config() -- initialise BEFORE the call
        # so a later set_parameters never clobbers an unset value.
        self._actual_dir = None  # real sampled direction (from escape_test_node)
        self._actual_traj = None  # real trajectory (from yaml scene)

        self._load_scene_config()
        self._cx = None
        self._cy = None
        self._t0 = None
        self._escaped = False
        self._excursion = False
        self._recontained = False
        self._max_r = 0.0
        self._min_r = float("inf")
        self._last_target = None  # (x, y) of the most recent target sample
        self._min_platform_dist = float("inf")  # response-evidence tracking
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
        self._drone_sub = self.create_subscription(
            DroneStateArray,
            str(self.get_parameter("drone_states_topic").value),
            self.on_drone_states,
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
        """Override monitor/block/intercept radius from yaml for the scene."""
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
        if "intercept_radius" in s:
            updates["intercept_radius"] = float(s["intercept_radius"])
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
        # /enclosure_command only carries the *number* of platforms the system
        # committed; it is recorded for diagnostics but never feeds the verdict.
        n = int(getattr(msg, "num_drones", 0))
        if n > self._num_responded:
            self._num_responded = n

    def on_drone_states(self, msg):
        if self._done or self._last_target is None:
            return
        tx, ty = self._last_target
        best = float("inf")
        for d in getattr(msg, "drones", []) or []:
            try:
                dx = float(getattr(d, "x", 0.0)) - tx
                dy = float(getattr(d, "y", 0.0)) - ty
            except (TypeError, ValueError):
                continue
            best = min(best, math.hypot(dx, dy))
        if best < self._min_platform_dist:
            self._min_platform_dist = best

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
            self._last_target = (x, y)
            return

        monitor = float(self.get_parameter("monitor_radius").value)
        block = float(self.get_parameter("block_radius").value)
        r = math.hypot(x - self._cx, y - self._cy)
        self._max_r = max(self._max_r, r)
        self._min_r = min(self._min_r, r)
        self._last_target = (x, y)

        if r > monitor:
            self._escaped = True
            return
        if not self._excursion and r > block:
            self._excursion = True
        if self._excursion and r <= block:
            self._recontained = True

    # ---- main loop -----------------------------------------------------
    def tick(self):
        if self._done or self._cx is None:
            return
        t = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        # Finalise as soon as a concrete outcome is known, or the window ends.
        if self._escaped or self._recontained or t >= float(
            self.get_parameter("test_duration").value
        ):
            self.finalize(t)

    # ---- finalisation --------------------------------------------------
    def finalize(self, t):
        if self._done:
            return
        self._done = True

        escaped = self._escaped
        re_contained = self._recontained
        held = (not escaped) and (not re_contained)
        min_dist = (
            self._min_platform_dist
            if self._min_platform_dist != float("inf")
            else None
        )
        intercept = float(self.get_parameter("intercept_radius").value)

        verdict = decide_verdict(
            escaped=escaped,
            re_contained=re_contained,
            held=held,
            min_platform_dist=min_dist,
            intercept_radius=intercept,
        )
        if verdict == "FAIL":
            reason = "escaped_monitor"
        elif verdict == "INVALID":
            reason = "no_response_evidence"
        else:
            reason = "re_contained" if re_contained else "held_within_monitor"
        self._verdict = verdict
        self._reason = reason

        min_r = self._min_r if self._min_r != float("inf") else 0.0
        min_pd = min_dist if min_dist is not None else float("nan")
        row = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "scene": str(self.get_parameter("scene_name").value),
            "direction": (self._actual_dir
                          if self._actual_dir is not None
                          else int(self.get_parameter("escape_direction").value)),
            "trajectory": (self._actual_traj
                           if self._actual_traj is not None
                           else str(self.get_parameter("trajectory").value)),
            "outcome": verdict,
            "reason": reason,
            "duration_s": f"{t:.2f}",
            "max_r": f"{self._max_r:.2f}",
            "min_r": f"{min_r:.2f}",
            "min_platform_dist": (f"{min_pd:.2f}"
                                  if min_dist is not None else "NA"),
            "num_responded": self._num_responded,
        }
        self._write_row(row)
        self.get_logger().info(
            f"[VERDICT] scene={row['scene']} dir={row['direction']} "
            f"traj={row['trajectory']} -> {verdict} ({reason}) "
            f"max_r={self._max_r:.2f} min_platform_dist="
            f"{row['min_platform_dist']} responded={self._num_responded}"
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
            "reason", "duration_s", "max_r", "min_r", "min_platform_dist",
            "num_responded",
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
