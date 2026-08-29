import numpy as np

from simulation.rfly_native.model import ClosedSplineRoute, GuidedGroundVehicle, PlanarState, RouteVehicle, SmoothUav, magnitude, wrap_angle
from simulation.rfly_native.rfly_native_demo import Detection, VehicleDetector, VisualTrack, parse_weather_profiles


def test_closed_route_wraps_without_discontinuity() -> None:
    route = ClosedSplineRoute(((0, 0), (30, 0), (35, 20), (5, 30), (-5, 15)))
    first = route.state_at(0.05)
    last = route.state_at(route.length + 0.05)
    assert magnitude(first.x - last.x, first.y - last.y) < 1e-6
    assert abs(wrap_angle(first.yaw - last.yaw)) < 1e-6


def test_route_vehicle_respects_speed_and_turn_limits() -> None:
    route = ClosedSplineRoute(((0, 0), (60, 0), (70, 35), (20, 60), (-10, 20)))
    vehicle = RouteVehicle(route, 0.0, cruise_speed=13.0, speed_variation=3.0)
    for index in range(600):
        state = vehicle.step(index * 0.05, 0.05)
        assert 0.0 <= state.speed <= 16.0
        assert -3.15 <= state.yaw <= 3.15


def test_guided_ground_vehicle_slows_at_intercept_point() -> None:
    vehicle = GuidedGroundVehicle(PlanarState(0.0, 0.0))
    for _ in range(500):
        state = vehicle.step(20.0, 0.0, 0.04)
    assert magnitude(state.x - 20.0, state.y) < 0.8
    assert state.speed < 1.0


def test_uav_motion_is_rate_limited() -> None:
    uav = SmoothUav(PlanarState(0.0, 0.0, altitude=20.0))
    prior = uav.state
    state = uav.step(100.0, 0.0, 55.0, 0.0, 0.1)
    assert magnitude(state.x - prior.x, state.y - prior.y) < 0.1
    assert 20.0 < state.altitude < 21.0


def test_blue_vehicle_detector_accepts_blue_target() -> None:
    frame = np.full((180, 320, 3), (70, 120, 80), dtype=np.uint8)
    frame[70:100, 130:190] = (255, 0, 0)
    detection = VehicleDetector(None, enable_yolo=False).detect(frame)
    assert detection is not None
    assert detection.source == "blue-rgb"
    assert detection.center == (160.0, 85.0)


def test_blue_vehicle_detector_rejects_dark_scene_feature() -> None:
    frame = np.full((180, 320, 3), (70, 120, 80), dtype=np.uint8)
    frame[70:100, 130:190] = (10, 10, 10)
    assert VehicleDetector(None, enable_yolo=False).detect(frame) is None


def test_blue_vehicle_detector_keeps_high_confidence_edge_reacquisition() -> None:
    frame = np.full((180, 320, 3), (70, 120, 80), dtype=np.uint8)
    frame[0:24, 130:190] = (255, 0, 0)
    assert VehicleDetector(None, enable_yolo=False).detect(frame) is not None


def test_detector_reacquires_after_association_reset() -> None:
    detector = VehicleDetector(None, enable_yolo=False)
    first = np.full((180, 400, 3), (70, 120, 80), dtype=np.uint8)
    first[70:100, 15:75] = (255, 0, 0)
    assert detector.detect(first) is not None
    reentered = np.full((180, 400, 3), (70, 120, 80), dtype=np.uint8)
    reentered[70:100, 340:380] = (255, 0, 0)
    assert detector.detect(reentered) is None
    detector.reset_association()
    assert detector.detect(reentered) is not None


def test_visual_track_rejects_large_projection_outlier() -> None:
    track = VisualTrack()
    uav = PlanarState(0.0, 0.0, yaw=0.0, altitude=46.0)
    first = track.update(Detection(306, 90, 334, 122, 0.9, "blue-rgb"), (360, 640, 3), uav, 0.0)
    estimate = track.update(Detection(42, 312, 82, 350, 0.9, "blue-rgb"), (360, 640, 3), uav, 0.2)
    assert magnitude(estimate.vx, estimate.vy) <= VisualTrack._MAX_TARGET_SPEED
    assert magnitude(estimate.x - first.x, estimate.y - first.y) <= VisualTrack._MAX_MEASUREMENT_INNOVATION


def test_visual_track_coasts_through_short_occlusion() -> None:
    track = VisualTrack()
    uav = PlanarState(0.0, 0.0, yaw=0.0, altitude=46.0)
    track.update(Detection(306, 90, 334, 122, 0.9, "blue-rgb"), (360, 640, 3), uav, 0.0)
    assert track.fresh(2.4) is not None
    assert track.fresh(2.6) is None


def test_weather_profile_parser_uses_supported_udsky_enum_range() -> None:
    assert parse_weather_profiles("clear:0,fog:7") == (("CLEAR", 0), ("FOG", 7))
