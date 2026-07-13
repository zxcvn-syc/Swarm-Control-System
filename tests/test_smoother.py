"""Tests for the RTS smoother."""

from __future__ import annotations

import numpy as np
import pytest

from cvtrack.tracker.smoother import rts_smooth_2d


def test_smoother_empty_returns_empty():
    assert rts_smooth_2d([]) == []


def test_smoother_single_point_returns_same_point():
    out = rts_smooth_2d([(0.0, 0.0)])
    assert out == [(0.0, 0.0)]


@pytest.mark.slow
def test_smoother_smooths_noisy_trajectory():
    """Smoothed trajectory should have lower variance than the raw input."""
    rng = np.random.default_rng(42)
    n = 200
    t = np.arange(n)
    truth = 5.0 + 0.5 * t.astype(float)  # constant velocity line
    obs_x = truth + rng.normal(scale=1.0, size=n)
    obs_y = np.zeros(n)
    positions = list(zip(obs_x.tolist(), obs_y.tolist()))

    smoothed = rts_smooth_2d(positions, process_var=0.001, meas_var=1.0)
    assert len(smoothed) == n

    raw_var = float(np.var(obs_x))
    smooth_x = np.array([s[0] for s in smoothed])
    smooth_var = float(np.var(smooth_x))
    assert smooth_var < raw_var, (
        f"smoother variance {smooth_var:.3f} should beat raw variance {raw_var:.3f}"
    )


def test_smoother_short_path_does_not_crash():
    """Two-point sequence should not blow up."""
    out = rts_smooth_2d([(0.0, 0.0), (1.0, 1.0)], process_var=0.1, meas_var=0.1)
    assert len(out) == 2
