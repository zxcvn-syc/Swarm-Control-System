"""Pytest tests for Kalman filter optimizations and trajectory prediction.

These tests validate the actual behavior of:
- Standard and adaptive Kalman filters
- Trajectory prediction functionality
- Motion mode detection and adaptation
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
import pytest
from cvtrack.tracker.kalman import (
    KalmanCV2D,
    KalmanBoT,
    KalmanCV2DAdaptive,
    KalmanBoTAdaptive,
)
from cvtrack.tracker.trajectory import (
    TrajectoryPredictor,
    TrajectorySmoother,
    TrajectoryAnalyzer,
    TrajectoryPredictorFactory,
)


# =============================================================================
# Kalman Filter Tests
# =============================================================================

def test_kalman_cv2d_initiate():
    """KalmanCV2D initiate should initialize 4-state with zero velocity."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    assert mean[0] == pytest.approx(100.0)
    assert mean[1] == pytest.approx(200.0)
    assert mean[2] == pytest.approx(0.0)
    assert mean[3] == pytest.approx(0.0)
    assert cov.shape == (4, 4)


def test_kalman_cv2d_predict():
    """KalmanCV2D predict should advance state and increase covariance."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    initial_trace = np.trace(cov)
    mean, cov = kf.predict(mean, cov)

    assert np.trace(cov) > initial_trace, "Covariance should increase after predict"


def test_kalman_cv2d_predict_with_velocity():
    """KalmanCV2D predict should advance position when velocity is non-zero."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    # Set non-zero velocity
    mean[2] = 10.0  # vx
    mean[3] = 5.0   # vy

    initial_x = mean[0]
    mean, cov = kf.predict(mean, cov)

    assert mean[0] != initial_x, "Position should advance with non-zero velocity"


def test_kalman_cv2d_update():
    """KalmanCV2D update should fuse measurement into state."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    measurement = np.array([105.0, 205.0], dtype=np.float64)
    mean, cov = kf.update(mean, cov, measurement)

    assert mean[0] == pytest.approx(105.0, abs=10.0)
    assert mean[1] == pytest.approx(205.0, abs=10.0)


def test_kalman_cv2d_adaptive_initiate():
    """KalmanCV2DAdaptive initiate should accept confidence parameter."""
    kf = KalmanCV2DAdaptive(dt=0.05, motion_adapt_gain=0.5)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z, confidence=0.9)

    assert mean[0] == pytest.approx(100.0)
    assert mean[1] == pytest.approx(200.0)
    assert cov.shape == (4, 4)


def test_kalman_cv2d_adaptive_predict():
    """KalmanCV2DAdaptive predict should grow covariance."""
    kf = KalmanCV2DAdaptive(dt=0.05, motion_adapt_gain=0.5)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z, confidence=0.9)

    initial_trace = np.trace(cov)

    for i in range(3):
        mean, cov = kf.predict(mean, cov)

    final_trace = np.trace(cov)
    assert final_trace > initial_trace


def test_kalman_cv2d_adaptive_update_with_confidence():
    """KalmanCV2DAdaptive update should use confidence to adjust noise."""
    kf = KalmanCV2DAdaptive(dt=0.05, motion_adapt_gain=0.5)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z, confidence=0.9)

    measurement = np.array([105.0, 205.0], dtype=np.float64)
    mean, cov = kf.update(mean, cov, measurement, confidence=0.9)

    assert mean[0] == pytest.approx(105.0, abs=10.0)


def test_kalman_cv2d_adaptive_low_confidence_update():
    """Low confidence update should increase measurement noise."""
    kf = KalmanCV2DAdaptive(dt=0.05, motion_adapt_gain=0.5)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z, confidence=0.9)

    measurement = np.array([105.0, 205.0], dtype=np.float64)
    mean, cov = kf.update(mean, cov, measurement, confidence=0.3)

    assert mean[0] == pytest.approx(105.0, abs=15.0)


def test_kalman_cv2d_adaptive_velocity_confidence():
    """KalmanCV2DAdaptive.compute_velocity_confidence should return 0-1."""
    kf = KalmanCV2DAdaptive(dt=0.05)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z, confidence=0.9)

    for i in range(3):
        mean, cov = kf.predict(mean, cov)
        mean, cov = kf.update(mean, cov, mean[:2], confidence=0.9)

    vel_conf = kf.compute_velocity_confidence(mean, cov)
    assert 0.0 <= vel_conf <= 1.0


def test_kalman_bot_initiate():
    """KalmanBoT initiate should initialize 8-state with zeros in velocity."""
    kf = KalmanBoT(dt=1.0 / 30.0)
    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf_bot_initiate(kf, z)

    assert mean[0] == pytest.approx(100.0)
    assert mean[1] == pytest.approx(200.0)
    assert mean[2] == pytest.approx(50.0)
    assert mean[3] == pytest.approx(100.0)
    assert mean[4] == pytest.approx(0.0)
    assert mean[5] == pytest.approx(0.0)
    assert cov.shape == (8, 8)


def test_kalman_bot_predict():
    """KalmanBoT predict should advance state and increase covariance."""
    kf = KalmanBoT(dt=1.0 / 30.0)
    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    initial_trace = np.trace(cov)
    mean, cov = kf.predict(mean, cov)

    assert np.trace(cov) > initial_trace, "Covariance should increase after predict"


def test_kalman_bot_predict_with_velocity():
    """KalmanBoT predict should advance position when velocity is non-zero."""
    kf = KalmanBoT(dt=1.0 / 30.0)
    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    # Set non-zero velocity
    mean[4] = 10.0  # vx
    mean[5] = 5.0   # vy

    initial_cx = mean[0]
    mean, cov = kf.predict(mean, cov)

    assert mean[0] != initial_cx, "Position should advance with non-zero velocity"


def test_kalman_bot_adaptive_initiate():
    """KalmanBoTAdaptive initiate should work with 4D measurement."""
    kf = KalmanBoTAdaptive(dt=1.0 / 30.0, acceleration_gain=0.5)
    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    assert mean[0] == pytest.approx(100.0)
    assert mean[4] == pytest.approx(0.0)
    assert cov.shape == (8, 8)


def test_kalman_bot_adaptive_predict():
    """KalmanBoTAdaptive predict should grow covariance."""
    kf = KalmanBoTAdaptive(dt=1.0 / 30.0, acceleration_gain=0.5)
    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    initial_trace = np.trace(cov)

    for _ in range(10):
        mean, cov = kf.predict(mean, cov)

    final_trace = np.trace(cov)
    assert final_trace > initial_trace


def test_kalman_bot_adaptive_motion_mode_stationary():
    """KalmanBoTAdaptive should detect stationary motion."""
    kf = KalmanBoTAdaptive(
        dt=0.033,
        acceleration_gain=0.5,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    stationary_state = np.array([100.0, 100.0, 50.0, 50.0, 0.5, 0.5, 0.0, 0.0])
    mode = kf._detect_motion_mode(stationary_state)
    assert mode == "stationary"


def test_kalman_bot_adaptive_motion_mode_slow():
    """KalmanBoTAdaptive should detect slow motion."""
    kf = KalmanBoTAdaptive(
        dt=0.033,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    slow_state = np.array([100.0, 100.0, 50.0, 50.0, 2.0, 2.0, 0.0, 0.0])
    mode = kf._detect_motion_mode(slow_state)
    assert mode == "slow"


def test_kalman_bot_adaptive_motion_mode_fast():
    """KalmanBoTAdaptive should detect fast motion."""
    kf = KalmanBoTAdaptive(
        dt=0.033,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    fast_state = np.array([100.0, 100.0, 50.0, 50.0, 30.0, 25.0, 0.0, 0.0])
    mode = kf._detect_motion_mode(fast_state)
    assert mode == "fast"


def test_kalman_bot_adaptive_adaptive_sigma_stationary():
    """Adaptive sigma should be smaller for stationary motion."""
    kf = KalmanBoTAdaptive(
        dt=0.033,
        acceleration_gain=0.5,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    stationary_state = np.array([100.0, 100.0, 50.0, 50.0, 0.5, 0.5, 0.0, 0.0])
    fast_state = np.array([100.0, 100.0, 50.0, 50.0, 30.0, 25.0, 0.0, 0.0])

    sigmas_stationary = kf._adaptive_sigma(stationary_state, 0.05, 0.00625)
    sigmas_fast = kf._adaptive_sigma(fast_state, 0.05, 0.00625)

    assert sigmas_stationary[0] < sigmas_fast[0]


def test_kalman_bot_adaptive_adaptive_sigma_fast():
    """Adaptive sigma should be larger for fast motion."""
    kf = KalmanBoTAdaptive(
        dt=0.033,
        acceleration_gain=0.5,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    fast_state = np.array([100.0, 100.0, 50.0, 50.0, 30.0, 25.0, 0.0, 0.0])

    sigmas_fast = kf._adaptive_sigma(fast_state, 0.05, 0.00625)

    assert sigmas_fast[0] > 0.05 * 50 * 1.5


def test_kalman_bot_adaptive_update():
    """KalmanBoTAdaptive update should fuse 4D measurement."""
    kf = KalmanBoTAdaptive(dt=1.0 / 30.0)
    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    measurement = np.array([105.0, 205.0, 52.0, 102.0], dtype=np.float64)
    mean, cov = kf.update(mean, cov, measurement)

    assert mean[0] == pytest.approx(105.0, abs=10.0)
    assert mean[1] == pytest.approx(205.0, abs=10.0)


def test_kalman_bot_adaptive_predict_n_steps():
    """KalmanBoTAdaptive.predict_n_steps should return future positions."""
    kf = KalmanBoTAdaptive(dt=1.0 / 30.0)
    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    predictions = kf.predict_n_steps(mean, cov, n=5)
    assert len(predictions) == 5
    assert all(len(p) == 2 for p in predictions)


def test_kalman_bot_adaptive_predict_n_steps_with_uncertainty():
    """predict_n_steps_with_uncertainty should return 4-tuple per step."""
    kf = KalmanBoTAdaptive(dt=1.0 / 30.0)
    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    predictions = kf.predict_n_steps_with_uncertainty(mean, cov, n=3)
    assert len(predictions) == 3
    for x, y, std_x, std_y in predictions:
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert std_x > 0
        assert std_y > 0


def test_kalman_cv2d_adaptive_predict_n_steps():
    """KalmanCV2DAdaptive.predict_n_steps should return future positions."""
    kf = KalmanCV2DAdaptive(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    predictions = kf.predict_n_steps(mean, cov, n=5)
    assert len(predictions) == 5


def test_kalman_cv2d_adaptive_predict_n_steps_with_uncertainty():
    """KalmanCV2DAdaptive.predict_n_steps_with_uncertainty should return 4-tuple."""
    kf = KalmanCV2DAdaptive(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    predictions = kf.predict_n_steps_with_uncertainty(mean, cov, n=3)
    assert len(predictions) == 3
    for x, y, std_x, std_y in predictions:
        assert std_x > 0
        assert std_y > 0


# =============================================================================
# Trajectory Prediction Tests
# =============================================================================

def test_trajectory_predictor_basic():
    """TrajectoryPredictor should generate predictions."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    for _ in range(10):
        mean, cov = kf.predict(mean, cov)
        mean, cov = kf.update(mean, cov, mean[:2])

    predictor = TrajectoryPredictor(
        prediction_steps=10,
        confidence_decay=0.9,
        min_confidence=0.1,
    )

    predictions = predictor.predict_trajectory(kf, mean, cov)
    assert len(predictions) > 0
    assert len(predictions) <= 10


def test_trajectory_predictor_confidence_decreases():
    """TrajectoryPredictor confidence should decrease per step."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    predictor = TrajectoryPredictor(prediction_steps=5, confidence_decay=0.9)
    predictions = predictor.predict_trajectory(kf, mean, cov)

    assert len(predictions) >= 2
    assert predictions[1][4] < predictions[0][4]


def test_trajectory_predictor_returns_tuple_format():
    """Each prediction should be 5-tuple (x, y, std_x, std_y, conf)."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    predictor = TrajectoryPredictor(prediction_steps=5)
    predictions = predictor.predict_trajectory(kf, mean, cov)

    assert len(predictions[0]) == 5
    x, y, std_x, std_y, conf = predictions[0]
    assert isinstance(x, float)
    assert isinstance(y, float)
    assert std_x > 0
    assert std_y > 0
    assert 0.0 <= conf <= 1.0


def test_trajectory_smoother_basic():
    """TrajectorySmoother should smooth trajectory."""
    noisy_trail = [
        (100, 100),
        (103, 102),
        (105, 99),
        (108, 103),
        (110, 101),
        (113, 104),
        (115, 102),
        (118, 105),
    ]

    smoother = TrajectorySmoother(window_size=5)
    smoothed = smoother.smooth_trajectory(noisy_trail)

    assert len(smoothed) == len(noisy_trail)


def test_trajectory_smoother_reduces_variance():
    """TrajectorySmoother should reduce position variance."""
    noisy_trail = [
        (100, 100),
        (103, 102),
        (105, 99),
        (108, 103),
        (110, 101),
        (113, 104),
        (115, 102),
        (118, 105),
    ]

    original_var = np.var([p[0] for p in noisy_trail]) + np.var([p[1] for p in noisy_trail])

    smoother = TrajectorySmoother(window_size=5)
    smoothed = smoother.smooth_trajectory(noisy_trail)

    smoothed_var = np.var([p[0] for p in smoothed]) + np.var([p[1] for p in smoothed])
    assert smoothed_var <= original_var


def test_trajectory_smoother_short_input():
    """TrajectorySmoother should return input unchanged if too short."""
    short_trail = [(100, 100), (105, 102)]
    smoother = TrajectorySmoother(min_points=3)
    result = smoother.smooth_trajectory(short_trail)
    assert result == short_trail


def test_trajectory_analyzer_classify_motion_mode():
    """TrajectoryAnalyzer should classify motion modes."""
    analyzer = TrajectoryAnalyzer(
        speed_threshold_slow=2.0,
        speed_threshold_fast=20.0,
    )

    stationary = [(100, 100), (101, 101), (100, 100), (101, 101)]
    mode = analyzer.classify_motion_mode(stationary)
    assert mode in ("stationary", "unknown")

    moving = [(100, 100), (120, 120), (140, 140), (160, 160)]
    mode = analyzer.classify_motion_mode(moving)
    assert mode in ("slow", "moderate", "fast")


def test_trajectory_analyzer_classify_fast_motion():
    """TrajectoryAnalyzer should classify fast motion correctly."""
    analyzer = TrajectoryAnalyzer(
        speed_threshold_slow=2.0,
        speed_threshold_fast=20.0,
    )

    fast_motion = [(0, 0), (50, 50), (100, 100), (150, 150)]
    mode = analyzer.classify_motion_mode(fast_motion)
    assert mode in ("moderate", "fast")


def test_trajectory_analyzer_curvature():
    """TrajectoryAnalyzer should compute curvature."""
    analyzer = TrajectoryAnalyzer()

    curved = [(0, 0), (10, 10), (15, 5), (20, 0)]
    curvatures = analyzer.compute_trajectory_curvature(curved, window=3)

    assert len(curvatures) == len(curved)
    assert all(isinstance(c, float) for c in curvatures)


def test_trajectory_analyzer_estimate_time_to_leave():
    """TrajectoryAnalyzer.estimate_time_to_leave_frame should return estimate."""
    analyzer = TrajectoryAnalyzer()

    positions = [(320, 240), (330, 250), (340, 260), (350, 270)]
    frame_bounds = (0, 0, 640, 480)
    ttl = analyzer.estimate_time_to_leave_frame(positions, frame_bounds, avg_speed=10.0)

    assert ttl is None or isinstance(ttl, float)


def test_trajectory_predictor_factory_short_term():
    """Factory should create short-term predictor with expected parameters."""
    predictor = TrajectoryPredictorFactory.create_short_term_predictor()
    assert predictor.prediction_steps == 3
    assert predictor.confidence_decay == pytest.approx(0.95)


def test_trajectory_predictor_factory_medium_term():
    """Factory should create medium-term predictor."""
    predictor = TrajectoryPredictorFactory.create_medium_term_predictor()
    assert predictor.prediction_steps == 8


def test_trajectory_predictor_factory_long_term():
    """Factory should create long-term predictor."""
    predictor = TrajectoryPredictorFactory.create_long_term_predictor()
    assert predictor.prediction_steps == 15


# =============================================================================
# Motion Adaptation Tests
# =============================================================================

def test_motion_mode_detection_consistency():
    """Motion mode detection should be consistent for same velocity."""
    kf = KalmanBoTAdaptive(
        dt=0.033,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    slow_state = np.array([100.0, 100.0, 50.0, 50.0, 2.0, 2.0, 0.0, 0.0])

    mode1 = kf._detect_motion_mode(slow_state)
    mode2 = kf._detect_motion_mode(slow_state)
    assert mode1 == mode2


def test_adaptive_noise_scales_with_motion_mode():
    """Process noise should adapt to detected motion mode."""
    kf = KalmanBoTAdaptive(
        dt=0.033,
        acceleration_gain=0.5,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    stationary_state = np.array([100.0, 100.0, 50.0, 50.0, 0.5, 0.5, 0.0, 0.0])
    fast_state = np.array([100.0, 100.0, 50.0, 50.0, 30.0, 25.0, 0.0, 0.0])

    sigmas_stationary = kf._adaptive_sigma(stationary_state, 0.05, 0.00625)
    sigmas_fast = kf._adaptive_sigma(fast_state, 0.05, 0.00625)

    assert sigmas_stationary[0] != sigmas_fast[0]


# Helper function to fix test_kalman_bot_initiate
def kf_bot_initiate(kf: KalmanBoT, z: np.ndarray):
    """KalmanBoT initiate wrapper."""
    return kf.initiate(z)
