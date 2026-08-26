"""Regression tests for the DeepSORT cascade association path."""

from __future__ import annotations

import numpy as np

from cvtrack.appearance.gallery import Gallery
from cvtrack.tracker.deepsort import DeepSortCascade
from cvtrack.types import Box, Track


def _embedding(*values: float) -> np.ndarray:
    vector = np.asarray(values, dtype=np.float64)
    return vector / np.linalg.norm(vector)


def _box(cx: float, *, width: float = 6.0) -> Box:
    half = width / 2.0
    return Box(cx - half, 7.0, cx + half, 13.0, 0.9, 0, "person")


def _confirmed_track(
    tracker: DeepSortCascade,
    *,
    track_id: int,
    cx: float,
    embedding: np.ndarray,
    width: float = 6.0,
    misses: int = 0,
) -> Track:
    mean, cov = tracker.kf.initiate(np.array([cx, 10.0], dtype=np.float64))
    return Track(
        track_id=track_id,
        label="person",
        mean=mean,
        cov=cov,
        box=_box(cx, width=width),
        hits=3,
        recent_hits=3,
        confirmed=True,
        misses=misses,
        n_init=3,
        trail=[(cx, 10.0)],
        pred_trail=[(cx, 10.0)],
        trail_scores=[0.9],
        embedding_mean=embedding.copy(),
    )


def _gallery(embedding: np.ndarray) -> Gallery:
    gallery = Gallery(size=8, ema_alpha=0.25)
    gallery.add(embedding)
    return gallery


def test_fresh_confirmed_track_uses_appearance_cascade() -> None:
    tracker = DeepSortCascade(
        stationary_prune=False,
        use_appearance=True,
        appearance_thresh=0.3,
        iou_thresh=0.1,
    )
    identity_a = _embedding(1.0, 0.0)
    identity_b = _embedding(0.0, 1.0)
    track = _confirmed_track(tracker, track_id=101, cx=10.0, embedding=identity_a)
    tracker.tracks = [track]
    galleries = {track.track_id: _gallery(identity_a)}

    tracker.step(
        [_box(10.0), _box(12.0)],
        det_embeddings=[identity_b, identity_a],
        galleries=galleries,
    )

    assert track.box.cx == 12.0
    assert track.misses == 0


def test_appearance_gate_rejects_non_overlapping_bad_embedding() -> None:
    tracker = DeepSortCascade(
        stationary_prune=False,
        use_appearance=True,
        appearance_thresh=0.3,
        iou_thresh=0.1,
    )
    identity_a = _embedding(1.0, 0.0)
    identity_b = _embedding(0.0, 1.0)
    track = _confirmed_track(
        tracker,
        track_id=102,
        cx=10.0,
        width=2.0,
        embedding=identity_a,
        misses=1,
    )
    tracker.tracks = [track]
    galleries = {track.track_id: _gallery(identity_a)}

    tracker.step(
        [_box(12.1, width=2.0)],
        det_embeddings=[identity_b],
        galleries=galleries,
    )

    assert track.misses == 2
    assert track.box.cx != 12.1
    assert len(tracker.tracks) == 2


def test_matched_embedding_updates_exact_track_gallery() -> None:
    tracker = DeepSortCascade(
        stationary_prune=False,
        use_appearance=True,
        appearance_thresh=0.5,
        gallery_size=8,
        gallery_ema_alpha=0.25,
    )
    identity = _embedding(1.0, 0.0)
    observation = _embedding(0.98, 0.2)
    track = _confirmed_track(tracker, track_id=103, cx=10.0, embedding=identity)
    tracker.tracks = [track]
    gallery = _gallery(identity)
    galleries = {track.track_id: gallery}

    tracker.step(
        [_box(10.0)],
        det_embeddings=[observation],
        galleries=galleries,
    )

    assert len(gallery) == 2
    np.testing.assert_allclose(track.embedding_mean, gallery.mean)
