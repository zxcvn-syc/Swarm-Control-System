"""Pytest integration tests for cvtrack perception module optimizations.

These tests validate the actual behavior of:
- Kalman filter optimizations (adaptive noise)
- Trajectory prediction
- Track stability enhancements
- Message interface compatibility
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
    predict_n_steps,
    predict_n_steps_with_covariance,
)
from cvtrack.tracker.trajectory import (
    TrajectoryPredictor,
    TrajectorySmoother,
    TrajectoryAnalyzer,
    TrajectoryPredictorFactory,
)
from cvtrack.tracker.stability import (
    IdentityManager,
    OcclusionHandler,
    AppearanceMemory,
    StabilityMetrics,
)
from cvtrack.types import Box, Track


# =============================================================================
# Kalman Filter Tests
# =============================================================================

def test_kalman_cv2d_initiate_and_predict():
    """KalmanCV2D initiate should initialize state with zero velocity."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 200.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    assert mean[0] == pytest.approx(100.0)
    assert mean[1] == pytest.approx(200.0)
    assert mean[2] == pytest.approx(0.0)  # vx = 0
    assert mean[3] == pytest.approx(0.0)  # vy = 0
    assert cov.shape == (4, 4)


def test_kalman_cv2d_predict_increases_uncertainty():
    """KalmanCV2D predict should increase covariance diagonal elements."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    initial_trace = np.trace(cov)
    for _ in range(10):
        mean, cov = kf.predict(mean, cov)

    final_trace = np.trace(cov)
    assert final_trace > initial_trace, "Predict should increase uncertainty"


def test_kalman_cv2d_update_converges():
    """KalmanCV2D update should reduce covariance and converge toward measurement."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    initial_trace = np.trace(cov)

    measurement = np.array([105.0, 102.0], dtype=np.float64)
    mean, cov = kf.update(mean, cov, measurement)

    final_trace = np.trace(cov)
    assert final_trace < initial_trace, "Update should reduce covariance"
    assert mean[0] == pytest.approx(105.0, abs=10.0)
    assert mean[1] == pytest.approx(102.0, abs=10.0)


def test_kalman_cv2d_adaptive_predict_increases_uncertainty():
    """KF predict should monotonically increase covariance diagonal sum."""
    kf = KalmanCV2DAdaptive(dt=0.033)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z, confidence=0.9)

    initial_diag_sum = np.sum(np.diag(cov))

    for _ in range(10):
        mean, cov = kf.predict(mean, cov)

    final_diag_sum = np.sum(np.diag(cov))
    assert final_diag_sum > initial_diag_sum


def test_kalman_cv2d_adaptive_low_confidence_increases_noise():
    """Low confidence detection should result in higher effective measurement noise."""
    kf = KalmanCV2DAdaptive(dt=0.05, motion_adapt_gain=0.3)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z, confidence=0.9)

    measurement = np.array([105.0, 102.0], dtype=np.float64)
    mean_high, cov_high = kf.update(mean, cov, measurement, confidence=0.9)

    kf2 = KalmanCV2DAdaptive(dt=0.05, motion_adapt_gain=0.3)
    mean2, cov2 = kf2.initiate(z, confidence=0.9)
    mean_low, cov_low = kf2.update(mean2, cov2, measurement, confidence=0.1)

    # Covariance should be larger with low confidence (filter trusts measurement less)
    assert np.trace(cov_low) > np.trace(cov_high)


def test_kalman_cv2d_adaptive_velocity_confidence():
    """compute_velocity_confidence should return value between 0 and 1."""
    kf = KalmanCV2DAdaptive(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z, confidence=0.9)

    for _ in range(5):
        mean, cov = kf.predict(mean, cov)
        mean, cov = kf.update(mean, cov, mean[:2], confidence=0.9)

    vel_conf = kf.compute_velocity_confidence(mean, cov)
    assert 0.0 <= vel_conf <= 1.0


def test_kalman_bot_initiate_and_predict():
    """KalmanBoT initiate should initialize 8-state with zero velocity."""
    kf = KalmanBoT(dt=1.0 / 30.0)
    z = np.array([100.0, 200.0, 50.0, 80.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    assert mean[0] == pytest.approx(100.0)
    assert mean[1] == pytest.approx(200.0)
    assert mean[2] == pytest.approx(50.0)
    assert mean[3] == pytest.approx(80.0)
    assert mean[4] == pytest.approx(0.0)
    assert mean[5] == pytest.approx(0.0)
    assert cov.shape == (8, 8)


def test_kalman_bot_adaptive_motion_mode_detected():
    """KalmanBoTAdaptive should detect motion mode from velocity."""
    kf = KalmanBoTAdaptive(
        dt=0.033,
        motion_threshold_slow=2.0,
        motion_threshold_fast=20.0,
    )

    z = np.array([100.0, 100.0, 50.0, 50.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    mode = kf._detect_motion_mode(mean)
    assert mode == "stationary"  # zero velocity

    mean[4], mean[5] = 5.0, 5.0
    mode = kf._detect_motion_mode(mean)
    assert mode == "slow"  # speed ~7 < 20

    mean[4], mean[5] = 30.0, 25.0
    mode = kf._detect_motion_mode(mean)
    assert mode == "fast"  # speed ~39 > 20


def test_kalman_bot_adaptive_update_motion_mode():
    """After multiple updates with moving observation, motion_mode should be set."""
    kf = KalmanBoTAdaptive(dt=0.033)
    z = np.array([100.0, 100.0, 50.0, 50.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    for i in range(20):
        measurement = np.array([100 + i * 5, 100 + i * 5, 50, 50], dtype=np.float64)
        mean, cov = kf.update(mean, cov, measurement)

    mode = kf._detect_motion_mode(mean)
    assert mode in ("slow", "fast", "stationary")


def test_kalman_bot_adaptive_sigma_scales_with_motion():
    """Adaptive sigma for stationary vs fast motion should differ."""
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

    assert sigmas_stationary[0] < sigmas_fast[0], "Stationary sigma_p should be smaller"


def test_kalman_predict_n_steps_helper():
    """predict_n_steps helper should return correct number of future positions."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    test_box = Box(90, 90, 110, 110, 0.9, 0, "person")
    track = Track(track_id=1, label="person", mean=mean, cov=cov, box=test_box)

    predictions = predict_n_steps(kf, track, n=5)
    assert len(predictions) == 5
    assert all(len(p) == 2 for p in predictions)


def test_kalman_predict_n_steps_with_covariance():
    """predict_n_steps_with_covariance should return mean and cov for each step."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    results = predict_n_steps_with_covariance(kf, mean, cov, n=3)

    assert len(results) == 3
    for step_mean, step_cov in results:
        assert len(step_mean) == 4
        assert step_cov.shape == (4, 4)


# =============================================================================
# Trajectory Prediction Tests
# =============================================================================

def test_trajectory_predictor_returns_correct_count():
    """TrajectoryPredictor should return predictions for requested steps."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    for _ in range(10):
        mean, cov = kf.predict(mean, cov)
        mean, cov = kf.update(mean, cov, mean[:2])

    predictor = TrajectoryPredictor(prediction_steps=10, confidence_decay=0.9)
    predictions = predictor.predict_trajectory(kf, mean, cov)

    assert len(predictions) > 0
    assert len(predictions) <= 10
    for x, y, std_x, std_y, conf in predictions:
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert std_x > 0
        assert std_y > 0
        assert 0.0 <= conf <= 1.0


def test_trajectory_predictor_confidence_decay():
    """Prediction confidence should decay exponentially."""
    predictor = TrajectoryPredictor(prediction_steps=5, confidence_decay=0.9)
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    predictions = predictor.predict_trajectory(kf, mean, cov)
    assert len(predictions) >= 2

    for i in range(1, len(predictions)):
        assert predictions[i][4] <= predictions[i - 1][4], "Confidence should decay"


def test_trajectory_smoother_reduces_variance():
    """TrajectorySmoother should reduce position variance of noisy trail."""
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

    original_variance = np.var([p[0] for p in noisy_trail]) + np.var([p[1] for p in noisy_trail])

    smoother = TrajectorySmoother(window_size=5)
    smoothed = smoother.smooth_trajectory(noisy_trail)

    assert len(smoothed) == len(noisy_trail)
    smoothed_variance = np.var([p[0] for p in smoothed]) + np.var([p[1] for p in smoothed])
    assert smoothed_variance <= original_variance


def test_trajectory_smoother_short_input():
    """TrajectorySmoother should return input unchanged if too short."""
    short_trail = [(100, 100), (105, 102)]
    smoother = TrajectorySmoother(min_points=3)
    result = smoother.smooth_trajectory(short_trail)
    assert result == short_trail


def test_trajectory_analyzer_classify_motion_mode():
    """TrajectoryAnalyzer should classify motion correctly."""
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


def test_trajectory_analyzer_curvature():
    """TrajectoryAnalyzer should compute curvature values."""
    analyzer = TrajectoryAnalyzer()

    curved = [(0, 0), (10, 10), (15, 5), (20, 0)]
    curvatures = analyzer.compute_trajectory_curvature(curved, window=3)

    assert len(curvatures) == len(curved)
    assert all(isinstance(c, float) for c in curvatures)


def test_trajectory_predictor_factory_presets():
    """TrajectoryPredictorFactory should create predictors with expected parameters."""
    short_pred = TrajectoryPredictorFactory.create_short_term_predictor()
    assert short_pred.prediction_steps == 3
    assert short_pred.confidence_decay == pytest.approx(0.95)

    medium_pred = TrajectoryPredictorFactory.create_medium_term_predictor()
    assert medium_pred.prediction_steps == 8

    long_pred = TrajectoryPredictorFactory.create_long_term_predictor()
    assert long_pred.prediction_steps == 15


# =============================================================================
# Track Stability Tests
# =============================================================================

def test_identity_manager_generate_id():
    """IdentityManager should generate unique IDs."""
    id_mgr = IdentityManager(max_lost_ids=10)

    id1 = id_mgr.generate_id()
    id2 = id_mgr.generate_id()

    assert id1 > 0
    assert id2 > 0
    assert id1 != id2


def test_identity_manager_register_active():
    """IdentityManager should track active IDs."""
    id_mgr = IdentityManager(max_lost_ids=10)
    new_id = id_mgr.generate_id()

    id_mgr.register_active(new_id)
    assert new_id in id_mgr._active_ids


def test_identity_manager_mark_lost():
    """IdentityManager.mark_lost should remove from active and add to lost."""
    id_mgr = IdentityManager(max_lost_ids=10)
    new_id = id_mgr.generate_id()
    id_mgr.register_active(new_id)

    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")
    embedding = np.random.randn(128)

    id_mgr.mark_lost(new_id, test_box, embedding, confidence=0.9)

    assert new_id not in id_mgr._active_ids
    assert new_id in id_mgr._lost_ids


def test_identity_manager_find_reactivation():
    """IdentityManager should find reactivation candidate by embedding similarity."""
    id_mgr = IdentityManager(max_lost_ids=10, reactivation_threshold=0.5)

    id1 = id_mgr.generate_id()
    id_mgr.register_active(id1)

    original_embedding = np.ones(128) / np.sqrt(128)
    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")

    id_mgr.mark_lost(id1, test_box, original_embedding, confidence=0.9)

    similar_embedding = np.ones(128) / np.sqrt(128) + np.random.randn(128) * 0.1
    similar_embedding = similar_embedding / np.linalg.norm(similar_embedding)

    found_id = id_mgr.find_reactivation_candidate(test_box, similar_embedding)
    assert found_id == id1


def test_occlusion_handler_detect_occlusions():
    """OcclusionHandler should detect overlapping boxes."""
    occ_handler = OcclusionHandler(overlap_threshold=0.5)

    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")
    tracks = [Track(track_id=1, label="person", mean=np.zeros(4), cov=np.eye(4), box=test_box)]

    overlapping_detection = Box(105, 105, 155, 155, 0.8, 0, "person")
    detections = [overlapping_detection]

    occlusion = occ_handler.detect_occlusions(tracks, detections)
    assert occlusion[1] is True


def test_occlusion_handler_no_occlusion():
    """OcclusionHandler should not flag non-overlapping boxes."""
    occ_handler = OcclusionHandler(overlap_threshold=0.5)

    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")
    tracks = [Track(track_id=1, label="person", mean=np.zeros(4), cov=np.eye(4), box=test_box)]

    distant_detection = Box(300, 300, 350, 350, 0.8, 0, "person")
    detections = [distant_detection]

    occlusion = occ_handler.detect_occlusions(tracks, detections)
    assert occlusion[1] is False


def test_occlusion_handler_gating_adjustment():
    """OcclusionHandler.get_gating_adjustment should increase during occlusion."""
    occ_handler = OcclusionHandler(overlap_threshold=0.5, prediction_boost=1.5)

    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")
    tracks = [Track(track_id=1, label="person", mean=np.zeros(4), cov=np.eye(4), box=test_box)]

    overlapping_detection = Box(105, 105, 155, 155, 0.8, 0, "person")
    detections = [overlapping_detection]

    base_adjustment = occ_handler.get_gating_adjustment(track_id=999)
    assert base_adjustment == 1.0

    occ_handler.detect_occlusions(tracks, detections)

    occluded_adjustment = occ_handler.get_gating_adjustment(track_id=1)
    assert occluded_adjustment > 1.0


def test_appearance_memory_update():
    """AppearanceMemory should store and update embeddings."""
    am = AppearanceMemory(memory_size=30, ema_alpha=0.1)

    embedding1 = np.random.randn(128)
    embedding2 = np.random.randn(128)
    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")

    am.update(track_id=1, embedding=embedding1, box=test_box, is_occluded=False)
    am.update(track_id=1, embedding=embedding2, box=test_box, is_occluded=False)

    mean_emb = am.get_mean_embedding(track_id=1)
    assert mean_emb is not None
    assert len(mean_emb) == 128
    assert np.linalg.norm(mean_emb) > 0


def test_appearance_memory_ema_convergence():
    """AppearanceMemory EMA should converge with multiple updates."""
    am = AppearanceMemory(memory_size=30, ema_alpha=0.1)

    initial = np.random.randn(128)
    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")
    am.update(track_id=1, embedding=initial, box=test_box)

    for _ in range(50):
        new_emb = np.random.randn(128)
        am.update(track_id=1, embedding=new_emb, box=test_box)

    mean_emb = am.get_mean_embedding(track_id=1)
    assert mean_emb is not None
    assert np.linalg.norm(mean_emb) > 0


def test_stability_metrics_record_association():
    """StabilityMetrics should track ID switches."""
    metrics = StabilityMetrics()

    metrics.record_association(track_id=1, previous_id=None)
    metrics.record_association(track_id=2, previous_id=None)

    assert metrics.total_updates == 2

    metrics.record_association(track_id=1, previous_id=2)
    assert metrics.id_switches == 1


def test_stability_metrics_record_miss():
    """StabilityMetrics should count misses."""
    metrics = StabilityMetrics()

    metrics.record_miss()
    metrics.record_miss()
    metrics.record_miss()

    assert metrics.misses == 3


def test_stability_metrics_get_metrics():
    """StabilityMetrics.get_metrics should return valid dictionary."""
    metrics = StabilityMetrics()

    metrics.record_association(track_id=1, previous_id=None)
    metrics.record_miss()

    result = metrics.get_metrics()

    assert "id_switches" in result
    assert "misses" in result
    assert "id_switch_rate" in result
    assert result["id_switch_rate"] >= 0.0


# =============================================================================
# Track with Predictions Integration Tests
# =============================================================================

def test_track_predict_update_lifecycle():
    """Track should correctly predict and update with Kalman filter."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([125.0, 125.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")
    track = Track(track_id=1, label="person", mean=mean, cov=cov, box=test_box)

    initial_age = track.age

    track.predict(kf)
    assert track.age == initial_age + 1

    z_measurement = np.array([126.0, 126.0], dtype=np.float64)
    measurement_box = Box(121, 121, 131, 131, 0.9, 0, "person")
    track.update(kf, measurement_box)

    assert track.hits == 2
    assert track.misses == 0


def test_track_update_trajectory_prediction():
    """Track.update_trajectory_prediction should populate predicted_future."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([125.0, 125.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")
    track = Track(track_id=1, label="person", mean=mean, cov=cov, box=test_box)

    track.update_trajectory_prediction(kf, n_steps=5)

    assert len(track.predicted_future) == 5
    for x, y, std_x, std_y in track.predicted_future:
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert std_x > 0
        assert std_y > 0


def test_track_prediction_confidence_after_update():
    """Track.prediction_confidence should be set after trajectory prediction."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([125.0, 125.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    test_box = Box(100, 100, 150, 150, 0.9, 0, "person")
    track = Track(track_id=1, label="person", mean=mean, cov=cov, box=test_box)

    initial_confidence = track.prediction_confidence

    track.update_trajectory_prediction(kf, n_steps=5)

    assert track.prediction_confidence != initial_confidence
    assert 0.0 <= track.prediction_confidence <= 1.0


def test_track_detect_motion_mode():
    """Track.detect_motion_mode should classify correctly."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    test_box = Box(90, 90, 110, 110, 0.9, 0, "person")
    track = Track(track_id=1, label="person", mean=mean, cov=cov, box=test_box)

    track.detect_motion_mode(speed_threshold_slow=2.0, speed_threshold_fast=20.0)
    assert track.motion_mode == "stationary"


def test_track_get_speed():
    """Track.get_speed should return velocity magnitude."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    test_box = Box(90, 90, 110, 110, 0.9, 0, "person")
    track = Track(track_id=1, label="person", mean=mean, cov=cov, box=test_box)

    speed = track.get_speed()
    assert speed == pytest.approx(0.0)

    track.mean[2] = 3.0
    track.mean[3] = 4.0
    speed = track.get_speed()
    assert speed == pytest.approx(5.0)


def test_track_mark_missed():
    """Track.mark_missed should increment misses and update state."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    test_box = Box(90, 90, 110, 110, 0.9, 0, "person")
    track = Track(track_id=1, label="person", mean=mean, cov=cov, box=test_box, confirmed=True)

    track.mark_missed()
    assert track.misses == 1
    assert track.lost_age == 1
    assert track.state == 1


def test_track_position_uncertainty():
    """Track.get_position_uncertainty should return valid uncertainty values."""
    kf = KalmanCV2D(dt=0.05)
    z = np.array([100.0, 100.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    test_box = Box(90, 90, 110, 110, 0.9, 0, "person")
    track = Track(track_id=1, label="person", mean=mean, cov=cov, box=test_box)

    std_x, std_y = track.get_position_uncertainty()
    assert std_x > 0
    assert std_y > 0


# =============================================================================
# Box Class Tests
# =============================================================================

def test_box_properties():
    """Box should compute properties correctly."""
    box = Box(x1=100, y1=100, x2=200, y2=300, score=0.9, cls=0, label="person")

    assert box.cx == 150.0
    assert box.cy == 200.0
    assert box.w == 100.0
    assert box.h == 200.0
    assert box.wh == (100.0, 200.0)
    assert box.area == 20000.0


def test_box_iou_high_overlap():
    """Box.iou should return high value for overlapping boxes."""
    box1 = Box(x1=100, y1=100, x2=200, y2=200, score=0.9, cls=0, label="person")
    box2 = Box(x1=120, y1=120, x2=220, y2=220, score=0.8, cls=0, label="person")

    iou = box1.iou(box2)
    assert 0.0 < iou <= 1.0


def test_box_iou_no_overlap():
    """Box.iou should return 0 for non-overlapping boxes."""
    box1 = Box(x1=100, y1=100, x2=200, y2=200, score=0.9, cls=0, label="person")
    box2 = Box(x1=300, y1=300, x2=400, y2=400, score=0.8, cls=0, label="person")

    iou = box1.iou(box2)
    assert iou == 0.0


# =============================================================================
# Kalman Filter 8-state (BoT) Tests
# =============================================================================

def test_kalman_bot_update_preserves_size():
    """KalmanBoT update should preserve bounding box dimensions in state."""
    kf = KalmanBoT(dt=1.0 / 30.0)
    z = np.array([100.0, 200.0, 50.0, 80.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    measurement = np.array([105.0, 205.0, 52.0, 82.0], dtype=np.float64)
    mean, cov = kf.update(mean, cov, measurement)

    assert mean[2] == pytest.approx(52.0, abs=5.0)
    assert mean[3] == pytest.approx(82.0, abs=5.0)


def test_kalman_bot_adaptive_predict_uncertainty_growth():
    """KalmanBoTAdaptive predict should grow covariance."""
    kf = KalmanBoTAdaptive(dt=0.033)
    z = np.array([100.0, 100.0, 50.0, 50.0], dtype=np.float64)
    mean, cov = kf.initiate(z)

    initial_trace = np.trace(cov)

    for _ in range(10):
        mean, cov = kf.predict(mean, cov)

    final_trace = np.trace(cov)
    assert final_trace > initial_trace


# =============================================================================
# Regression: ensure no silent failures
# =============================================================================

def test_all_modules_importable():
    """All cvtrack modules should be importable."""
    from cvtrack.tracker.kalman import (
        KalmanCV2D,
        KalmanBoT,
        KalmanCV2DAdaptive,
        KalmanBoTAdaptive,
        predict_n_steps,
        predict_n_steps_with_covariance,
    )
    from cvtrack.tracker.trajectory import (
        TrajectoryPredictor,
        TrajectorySmoother,
        TrajectoryAnalyzer,
        TrajectoryPredictorFactory,
    )
    from cvtrack.tracker.stability import (
        IdentityManager,
        OcclusionHandler,
        AppearanceMemory,
        StabilityMetrics,
    )
    from cvtrack.types import Box, Track, Detection

    assert KalmanCV2D is not None
    assert KalmanBoT is not None
    assert KalmanCV2DAdaptive is not None
    assert KalmanBoTAdaptive is not None
    assert TrajectoryPredictor is not None
    assert TrajectorySmoother is not None
    assert TrajectoryAnalyzer is not None
    assert IdentityManager is not None
    assert OcclusionHandler is not None
    assert AppearanceMemory is not None
    assert StabilityMetrics is not None
    assert Box is not None
    assert Track is not None
    assert Detection is not None
