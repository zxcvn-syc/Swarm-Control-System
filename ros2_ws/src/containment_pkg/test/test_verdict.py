#!/usr/bin/env python3
"""Unit tests for the 8.26 containment verdict matrix (response-evidence gate).

These test the *pure* decision function in ``containment_pkg.verdict`` so the
verdict logic can be locked down without spinning up ROS.  They mirror the
four cases the project lead asked for:

  * straight (target escapes)            -> FAIL
  * return + platform within radius     -> SUCCESS
  * return + no platform within radius  -> INVALID
  * window timeout + no platform nearby -> INVALID

Run (after colcon build + source install/setup.bash):
  python3 -m pytest ros2_ws/src/containment_pkg/test/test_verdict.py -v

Or as plain unittest (no pytest / no colcon needed):
  python3 ros2_ws/src/containment_pkg/test/test_verdict.py
"""

import os
import sys
import unittest

# Make the package importable whether run via pytest (colcon-sourced) or
# invoked directly as a script.
try:
    from containment_pkg.verdict import decide_verdict
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(_here, ".."))  # .../containment_pkg
    from containment_pkg.verdict import decide_verdict


INTERCEPT = 5.0


class TestVerdict(unittest.TestCase):

    def test_straight_escapes_is_fail(self):
        # Target broke out of the monitor ring regardless of platform proximity.
        self.assertEqual(
            decide_verdict(
                escaped=True, re_contained=False, held=False,
                min_platform_dist=2.0, intercept_radius=INTERCEPT,
            ),
            "FAIL",
        )

    def test_return_with_evidence_is_success(self):
        # Target excursed and returned, and a platform got within 5 m.
        self.assertEqual(
            decide_verdict(
                escaped=False, re_contained=True, held=False,
                min_platform_dist=3.0, intercept_radius=INTERCEPT,
            ),
            "SUCCESS",
        )

    def test_return_without_evidence_is_invalid(self):
        # Target came back on its own, but no platform engaged -> not attributable.
        self.assertEqual(
            decide_verdict(
                escaped=False, re_contained=True, held=False,
                min_platform_dist=9.0, intercept_radius=INTERCEPT,
            ),
            "INVALID",
        )

    def test_timeout_no_evidence_is_invalid(self):
        # Window ended, target held inside, but no platform ever came close.
        self.assertEqual(
            decide_verdict(
                escaped=False, re_contained=False, held=True,
                min_platform_dist=None, intercept_radius=INTERCEPT,
            ),
            "INVALID",
        )

    def test_held_with_evidence_is_success(self):
        # Target never left the inner ring and a platform shadowed it.
        self.assertEqual(
            decide_verdict(
                escaped=False, re_contained=False, held=True,
                min_platform_dist=4.0, intercept_radius=INTERCEPT,
            ),
            "SUCCESS",
        )

    def test_held_without_evidence_is_invalid(self):
        # Target sat still but the swarm was nowhere near -> excluded.
        self.assertEqual(
            decide_verdict(
                escaped=False, re_contained=False, held=True,
                min_platform_dist=12.0, intercept_radius=INTERCEPT,
            ),
            "INVALID",
        )

    def test_escaped_ignores_evidence(self):
        # Even if a platform happened to be close, a break-out is always a FAIL.
        self.assertEqual(
            decide_verdict(
                escaped=True, re_contained=False, held=False,
                min_platform_dist=0.5, intercept_radius=INTERCEPT,
            ),
            "FAIL",
        )

    def test_intercept_radius_boundary(self):
        # Exactly at the threshold counts as evidence (<=).
        self.assertEqual(
            decide_verdict(
                escaped=False, re_contained=True, held=False,
                min_platform_dist=5.0, intercept_radius=INTERCEPT,
            ),
            "SUCCESS",
        )
        # One millimetre outside does not.
        self.assertEqual(
            decide_verdict(
                escaped=False, re_contained=True, held=False,
                min_platform_dist=5.001, intercept_radius=INTERCEPT,
            ),
            "INVALID",
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
