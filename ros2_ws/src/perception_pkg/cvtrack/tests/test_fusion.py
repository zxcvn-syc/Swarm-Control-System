"""Behavioral tests for confidence-aware multi-source trajectory fusion."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
import pytest

from cvtrack.tracker.fusion import (
    ConsistencyGuard,
    TrackFusion,
    TrajectoryGraph,
    weighted_fuse,
)
from cvtrack.types import Box, Track


def _track(
    track_id: int,
    x: float,
    y: float,
    *,
    confidence: float = 1.0,
    variance: float = 4.0,
) -> Track:
    width = 20.0
    height = 20.0
    return Track(
        track_id=track_id,
        label="target",
        mean=np.array([x, y, 0.0, 0.0], dtype=np.float64),
        cov=np.diag([variance, variance, variance, variance]).astype(np.float64),
        box=Box(
            x - width / 2.0,
            y - height / 2.0,
            x + width / 2.0,
            y + height / 2.0,
            confidence,
            0,
            "target",
        ),
        confirmed=True,
    )


def test_track_fusion_weighted_position():
    """Two sensor reports for same target are weighted by confidence."""
    x, y, confidence = weighted_fuse([(0.0, 4.0, 0.9), (10.0, 4.0, 0.1)])

    assert x == pytest.approx(1.0)
    assert y == pytest.approx(4.0)
    assert confidence == pytest.approx(0.91)

    # Missing confidence is a supported neutral-weight degradation path.
    missing_x, _, _ = weighted_fuse([(0.0, 0.0, None), (2.0, 0.0, None)])
    assert missing_x == pytest.approx(1.0)


def test_track_fusion_id_consistency():
    """Same physical target from different sensors maintains one global ID."""
    fusion = TrackFusion(max_distance_px=50.0, iou_threshold=0.1)
    fusion.register_source("cam0")
    fusion.register_source("cam1")
    fusion.update("cam0", [_track(7, 100.0, 80.0)])
    first = fusion.fused_tracks()[0]
    fusion.update("cam1", [_track(44, 102.0, 79.0)])
    second = fusion.fused_tracks()[0]

    assert fusion.sources() == ["cam0", "cam1"]
    assert first.track_id == second.track_id
    assert second.global_id == first.track_id
    assert fusion.get_track(first.track_id) is not None


def test_track_fusion_outlier_rejection():
    """Observations deviating more than three robust sigma are rejected."""
    fusion = TrackFusion(
        outlier_sigma=3.0,
        max_distance_px=250.0,
        iou_threshold=0.0,
    )
    for index, x in enumerate((0.0, 0.2, -0.2, 100.0)):
        fusion.update(f"cam{index}", [_track(index + 1, x, 0.0)])

    fused = fusion.fused_tracks()
    assert len(fused) == 1
    assert fused[0].pos[0] == pytest.approx(0.0, abs=0.15)
    assert ConsistencyGuard().filter([0.0, 0.2, -0.2, 100.0]) == [
        0.0,
        0.2,
        -0.2,
    ]


def test_track_fusion_temporal_smoothing():
    """Fusion smooths a sudden shared measurement jump across updates."""
    fusion = TrackFusion(max_distance_px=100.0, iou_threshold=0.0)
    fusion.update("cam0", [_track(1, 0.0, 0.0)])
    fusion.update("cam1", [_track(2, 0.0, 0.0)])
    before = fusion.fused_tracks()[0].pos[0]

    fusion.update("cam0", [_track(1, 10.0, 0.0)])
    fusion.update("cam1", [_track(2, 10.0, 0.0)])
    after = fusion.fused_tracks()[0].pos[0]

    assert before == pytest.approx(0.0)
    assert before < after < 10.0


def test_track_fusion_cross_sensor_association():
    """Associated local tracks are linked in the trajectory graph."""
    graph = TrajectoryGraph()
    graph.link("cam0", 1, "cam1", 9)
    graph.add_track("cam2", 4)
    assert graph.neighbors("cam0", 1) == [("cam1", 9)]
    assert {frozenset(component) for component in graph.components()} == {
        frozenset({("cam0", 1), ("cam1", 9)}),
        frozenset({("cam2", 4)}),
    }

    fusion = TrackFusion(max_distance_px=50.0, iou_threshold=0.1)
    fusion.update("cam0", [_track(1, 20.0, 20.0)])
    fusion.update("cam1", [_track(9, 22.0, 20.0)])
    assert fusion.graph.neighbors("cam0", 1) == [("cam1", 9)]


def test_track_fusion_covariance_reduction():
    """Independent observations reduce fused position covariance."""
    fusion = TrackFusion(max_distance_px=50.0, iou_threshold=0.0)
    fusion.update("cam0", [_track(1, 10.0, 10.0, variance=9.0)])
    fusion.update("cam1", [_track(2, 12.0, 10.0, variance=4.0)])
    fused = fusion.fused_tracks()[0]

    assert fused.cov[0, 0] < 4.0
    assert fused.cov[1, 1] < 4.0
    expected_x = (10.0 / 9.0 + 12.0 / 4.0) / (1.0 / 9.0 + 1.0 / 4.0)
    assert fused.pos[0] == pytest.approx(expected_x)


def test_track_fusion_missing_sensor():
    """An empty sensor snapshot leaves other live observations usable."""
    fusion = TrackFusion(max_distance_px=50.0, iou_threshold=0.0)
    fusion.update("cam0", [_track(1, 5.0, 6.0)])
    fusion.update("cam1", [_track(2, 6.0, 6.0)])
    fusion.update("cam1", [])

    fused = fusion.fused_tracks()
    assert len(fused) == 1
    assert 5.0 <= fused[0].pos[0] <= 6.0


def test_track_fusion_new_target_appears():
    """A distant new local track creates a second global target."""
    fusion = TrackFusion(max_distance_px=30.0, iou_threshold=0.0)
    fusion.update("cam0", [_track(1, 0.0, 0.0)])
    fusion.update("cam1", [_track(2, 100.0, 100.0)])

    fused = fusion.fused_tracks()
    assert len(fused) == 2
    assert {track.track_id for track in fused} == {1, 2}


def test_track_fusion_target_disappears():
    """A target is removed after all observations exceed the idle timeout."""
    fusion = TrackFusion(max_idle_seconds=1.0)
    now = [0.0]
    fusion._clock = lambda: now[0]
    fusion.update("cam0", [_track(1, 0.0, 0.0)])
    global_id = fusion.fused_tracks()[0].track_id

    now[0] = 1.01
    assert fusion.fused_tracks() == []
    assert fusion.get_track(global_id) is None
