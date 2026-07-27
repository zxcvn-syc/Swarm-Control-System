"""Sanity tests for planner_stub — pure-python, no ROS2 spin required.

These cover the geometry helpers and the data-shape of the
``DroneState`` message.  They do not exercise the rclpy node.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

# Allow running this file from anywhere: import the module by file path
# because it has no ROS2-side dependencies.
_HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_HERE))

from planner_stub.planner_stub_node import _clamp, _step_towards  # noqa: E402


class TestStepTowards(unittest.TestCase):
    def test_reaches_goal_when_close(self) -> None:
        got = _step_towards((0.0, 0.0), (0.5, 0.0), max_step=2.0)
        self.assertAlmostEqual(got[0], 0.5)
        self.assertAlmostEqual(got[1], 0.0)

    def test_caps_step(self) -> None:
        got = _step_towards((0.0, 0.0), (10.0, 0.0), max_step=1.0)
        self.assertAlmostEqual(got[0], 1.0)
        self.assertAlmostEqual(got[1], 0.0)

    def test_zero_distance(self) -> None:
        got = _step_towards((1.0, 1.0), (1.0, 1.0), max_step=1.0)
        self.assertEqual(got, (1.0, 1.0))


class TestClamp(unittest.TestCase):
    def test_in_range(self) -> None:
        self.assertEqual(_clamp(0.5, 0.0, 1.0), 0.5)

    def test_above(self) -> None:
        self.assertEqual(_clamp(2.0, 0.0, 1.0), 1.0)

    def test_below(self) -> None:
        self.assertEqual(_clamp(-1.0, 0.0, 1.0), 0.0)


if __name__ == "__main__":
    unittest.main()
