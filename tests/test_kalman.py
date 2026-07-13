"""Tests for the Kalman filter implementations."""

from __future__ import annotations

import numpy as np
import pytest

from cvtrack.tracker.kalman import (
    BOTSORT_HIGH_CONF,
    BOTSORT_IOU_THRESH,
    BOTSORT_LOST_RELINK_FRAMES,
    BOTSORT_NEW_TRACK_CONF,
    KalmanBoT,
    KalmanCV2D,
)


# ---------------------------------------------------------------------------
# 2D constant-velocity KF (DeepSORT-lite)
# ---------------------------------------------------------------------------


def test_predict_update_round_trip_cv2d():
    """A stationary estimate of (0,0) with zero velocity remains (0,0)."""
    kf = KalmanCV2D(dt=0.1)
    mean, cov = kf.initiate(np.array([0.0, 0.0]))
    # Forward predict without measurement.
    mean, cov = kf.predict(mean, cov)
    # Position should still be ~ (0, 0).
    np.testing.assert_allclose(mean[:2], [0.0, 0.0], atol=1e-6)
    # Velocity should still be ~ (0, 0).
    np.testing.assert_allclose(mean[2:], [0.0, 0.0], atol=1e-6)


def test_static_input_zero_velocity_in_steady_state():
    """Stationary observations should drive the velocity estimate toward zero."""
    kf = KalmanCV2D(dt=0.1)
    mean, cov = kf.initiate(np.array([0.0, 0.0]))
    mean[2:] = np.array([5.0, 5.0])  # start with bad velocity guess
    for _ in range(200):
        mean, cov = kf.update(mean, cov, np.array([0.0, 0.0]))
        mean, cov = kf.predict(mean, cov)
    assert abs(mean[2]) < 0.5, f"x-velocity should be near zero, got {mean[2]}"
    assert abs(mean[3]) < 0.5, f"y-velocity should be near zero, got {mean[3]}"


def test_gating_rejects_outlier_cv2d():
    kf = KalmanCV2D(dt=0.1)
    mean = np.array([0.0, 0.0, 0.0, 0.0])
    cov = np.eye(4) * 1.0
    # Large Mahalanobis distance: predicted = (0,0), measurement = (1000, 1000).
    d = kf.mahalanobis(mean, cov, np.array([1000.0, 1000.0]))
    assert d > 100.0, "Outlier should yield huge Mahalanobis distance"


# ---------------------------------------------------------------------------
# 8-state BoT-SORT KF
# ---------------------------------------------------------------------------


def test_botsort_init_state_shape():
    box = np.array([100.0, 100.0, 200.0, 200.0])  # x1=100, y1=100, x2=200, y2=200
    kf = KalmanBoT(dt=0.033)
    mean, cov = kf.initiate(box)
    assert mean.shape == (8,)
    np.testing.assert_allclose(mean[:4], box, atol=1e-6)
    np.testing.assert_allclose(mean[4:], np.zeros(4), atol=1e-9)


def test_botsort_predict_returns_advanced_state():
    box = np.array([100.0, 100.0, 200.0, 200.0])
    kf = KalmanBoT(dt=0.1)
    mean, cov = kf.initiate(box)
    # Velocity starts at zero so position must stay ~ constant.
    pred_mean, pred_cov = kf.predict(mean, cov)
    assert pred_mean.shape == (8,)
    # Position stays at original values since velocity is zero.
    np.testing.assert_allclose(pred_mean[:4], box, atol=0.1)


def test_botsort_constants_sane():
    assert BOTSORT_IOU_THRESH == pytest.approx(0.30)
    assert BOTSORT_HIGH_CONF == pytest.approx(0.35)
    assert BOTSORT_NEW_TRACK_CONF == pytest.approx(0.20)
    assert 10 <= BOTSORT_LOST_RELINK_FRAMES <= 60
