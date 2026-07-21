#!/usr/bin/env python3
"""Test script for Kalman filter optimizations and trajectory prediction.

Run this script to validate the enhanced Kalman filters and trajectory
prediction functionality.
"""

import numpy as np
import sys
import os

# Add the cvtrack src path
src_path = os.path.join(os.path.dirname(__file__), '..', 'src')
sys.path.insert(0, os.path.abspath(src_path))

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


def test_kalman_filters():
    """Test standard and adaptive Kalman filters."""
    print("=" * 60)
    print("Testing Kalman Filters")
    print("=" * 60)

    # Test KalmanCV2D
    print("\n[1] Testing KalmanCV2D (Standard)")
    kf_std = KalmanCV2D(dt=0.05)

    z1 = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf_std.initiate(z1)
    print(f"  Initial state: mean={mean}, cov.shape={cov.shape}")

    for i in range(3):
        mean, cov = kf_std.predict(mean, cov)
        print(f"  After predict {i+1}: pos=({mean[0]:.2f}, {mean[1]:.2f})")

    z2 = np.array([105.0, 205.0], dtype=np.float64)
    mean, cov = kf_std.update(mean, cov, z2)
    print(f"  After update: pos=({mean[0]:.2f}, {mean[1]:.2f}), vel=({mean[2]:.2f}, {mean[3]:.2f})")

    # Test KalmanCV2DAdaptive
    print("\n[2] Testing KalmanCV2DAdaptive (Enhanced)")
    kf_adapt = KalmanCV2DAdaptive(dt=0.05, motion_adapt_gain=0.5)

    z1 = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf_adapt.initiate(z1, confidence=0.9)
    print(f"  Initial state with confidence=0.9")

    for i in range(3):
        mean, cov = kf_adapt.predict(mean, cov)
        print(f"  After predict {i+1}: pos=({mean[0]:.2f}, {mean[1]:.2f})")

    z2 = np.array([105.0, 205.0], dtype=np.float64)
    mean, cov = kf_adapt.update(mean, cov, z2, confidence=0.9)
    vel_conf = kf_adapt.compute_velocity_confidence(mean, cov)
    print(f"  After update: pos=({mean[0]:.2f}, {mean[1]:.2f}), vel=({mean[2]:.2f}, {mean[3]:.2f})")
    print(f"  Velocity confidence: {vel_conf:.3f}")

    # Test with low confidence detection
    mean, cov = kf_adapt.update(mean, cov, z2, confidence=0.3)
    print(f"  After low-conf update: R noise should be higher")

    # Test KalmanBoT
    print("\n[3] Testing KalmanBoT (Standard 8-state)")
    kf_bot = KalmanBoT(dt=1.0/30.0)

    z = np.array([100.0, 200.0, 50.0, 100.0], dtype=np.float64)
    mean, cov = kf_bot.initiate(z)
    print(f"  Initial state: pos=({mean[0]:.2f}, {mean[1]:.2f}), size=({mean[2]:.2f}, {mean[3]:.2f})")

    mean, cov = kf_bot.predict(mean, cov)
    print(f"  After predict: pos=({mean[0]:.2f}, {mean[1]:.2f}), vel=({mean[4]:.2f}, {mean[5]:.2f})")

    # Test KalmanBoTAdaptive
    print("\n[4] Testing KalmanBoTAdaptive (Enhanced)")
    kf_bot_adapt = KalmanBoTAdaptive(
        dt=1.0/30.0,
        acceleration_gain=0.5,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    mean, cov = kf_bot_adapt.initiate(z)
    print(f"  Motion mode: {kf_bot_adapt._detect_motion_mode(mean)}")

    for _ in range(10):
        mean, cov = kf_bot_adapt.predict(mean, cov)

    print(f"  After 10 predictions: pos=({mean[0]:.2f}, {mean[1]:.2f})")
    print(f"  Motion mode: {kf_bot_adapt._detect_motion_mode(mean)}")

    print("\n✓ Kalman filter tests passed!")


def test_trajectory_prediction():
    """Test trajectory prediction functionality."""
    print("\n" + "=" * 60)
    print("Testing Trajectory Prediction")
    print("=" * 60)

    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    for _ in range(10):
        mean, cov = kf.predict(mean, cov)
        z = mean[:2] + np.random.randn(2) * 2
        mean, cov = kf.update(mean, cov, z)

    print(f"\n[1] Testing TrajectoryPredictor")
    predictor = TrajectoryPredictor(
        prediction_steps=10,
        confidence_decay=0.9,
        min_confidence=0.1,
    )

    predictions = predictor.predict_trajectory(kf, mean, cov)
    print(f"  Generated {len(predictions)} predictions")
    for i, (x, y, std_x, std_y, conf) in enumerate(predictions[:5]):
        print(f"  Step {i}: pos=({x:.1f}, {y:.1f}), uncertainty=({std_x:.1f}, {std_y:.1f}), conf={conf:.2f}")

    print("\n[2] Testing TrajectorySmoother")
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
    print(f"  Original: {noisy_trail}")
    print(f"  Smoothed: {smoothed}")

    print("\n[3] Testing TrajectoryAnalyzer")
    analyzer = TrajectoryAnalyzer()
    motion_mode = analyzer.classify_motion_mode(noisy_trail)
    print(f"  Motion mode: {motion_mode}")

    curvatures = analyzer.compute_trajectory_curvature(noisy_trail)
    print(f"  Curvatures: {[f'{c:.2f}' for c in curvatures[:5]]}")

    frame_bounds = (0, 0, 640, 480)
    positions = [(320, 240), (330, 250), (340, 260), (350, 270)]
    ttl = analyzer.estimate_time_to_leave_frame(positions, frame_bounds, avg_speed=10.0)
    print(f"  Time to leave frame: {ttl:.1f}s" if ttl else "  Already outside frame")

    print("\n[4] Testing Factory Methods")
    short_pred = TrajectoryPredictorFactory.create_short_term_predictor()
    medium_pred = TrajectoryPredictorFactory.create_medium_term_predictor()
    long_pred = TrajectoryPredictorFactory.create_long_term_predictor()
    print(f"  Short-term: {short_pred.prediction_steps} steps, decay={short_pred.confidence_decay}")
    print(f"  Medium-term: {medium_pred.prediction_steps} steps, decay={medium_pred.confidence_decay}")
    print(f"  Long-term: {long_pred.prediction_steps} steps, decay={long_pred.confidence_decay}")

    print("\n✓ Trajectory prediction tests passed!")


def test_motion_adaptation():
    """Test motion mode detection and adaptation."""
    print("\n" + "=" * 60)
    print("Testing Motion Adaptation")
    print("=" * 60)

    kf_adapt = KalmanBoTAdaptive(
        dt=0.033,
        acceleration_gain=0.5,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    test_cases = [
        ("Stationary", np.array([0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0])),
        ("Slow motion", np.array([100.0, 100.0, 50.0, 50.0, 2.0, 2.0, 0.0, 0.0])),
        ("Fast motion", np.array([100.0, 100.0, 50.0, 50.0, 30.0, 25.0, 0.0, 0.0])),
    ]

    print("\nTesting motion mode detection:")
    for name, state in test_cases:
        mode = kf_adapt._detect_motion_mode(state)
        speed = np.sqrt(state[4]**2 + state[5]**2)
        print(f"  {name}: speed={speed:.1f} px/frame -> mode={mode}")

    print("\nTesting adaptive noise scaling:")
    z = np.array([100.0, 100.0, 50.0, 50.0], dtype=np.float64)
    mean, cov = kf_adapt.initiate(z)

    for name, state in test_cases:
        mean_test = state.copy()
        sigmas = kf_adapt._adaptive_sigma(mean_test, 0.05, 0.00625)
        print(f"  {name}: sigma_p scale = {sigmas[0]/0.05:.2f}x")

    print("\n✓ Motion adaptation tests passed!")


if __name__ == "__main__":
    try:
        test_kalman_filters()
        test_trajectory_prediction()
        test_motion_adaptation()

        print("\n" + "=" * 60)
        print("ALL TESTS PASSED!")
        print("=" * 60)
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
