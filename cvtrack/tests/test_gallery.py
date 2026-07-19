"""Tests for the per-track ReID embedding Gallery.

The Gallery object in cvtrack.appearance.gallery is *per-track*: it stores the
last N embeddings for a single identity. Multi-track bookkeeping is done by the
tracker (one Gallery per Track).  The tests below mirror that.
"""

from __future__ import annotations

import numpy as np
import pytest

from cvtrack.appearance.gallery import Gallery


def _emb(values):
    """Build a length-4 L2-normalized embedding from `values`."""
    v = np.asarray(values, dtype=np.float32)
    n = np.linalg.norm(v)
    if n > 0:
        v = v / n
    return v.astype(np.float32)


def test_gallery_add_and_mean_is_average():
    g = Gallery(size=10, ema_alpha=0.5)  # weighted average
    g.add(_emb([1.0, 0.0, 0.0, 0.0]))
    g.add(_emb([0.0, 1.0, 0.0, 0.0]))
    mean = g.mean  # property
    assert mean is not None
    # Final mean should be a unit vector in the 2-D subspace spanned by inputs.
    assert abs(np.linalg.norm(mean) - 1.0) < 1e-5


def test_gallery_ema_smooths_recent_observations():
    g = Gallery(size=10, ema_alpha=1.0)  # full EMA -> newest value dominates
    g.add(_emb([1.0, 0.0, 0.0, 0.0]))
    g.add(_emb([0.0, 1.0, 0.0, 0.0]))
    # EMA alpha=1.0 means "replace with newest", so mean -> newest.
    mean = g.mean
    np.testing.assert_allclose(mean, _emb([0.0, 1.0, 0.0, 0.0]), atol=1e-5)


def test_gallery_capacity_is_bounded():
    g = Gallery(size=3)
    for i in range(10):
        g.add(_emb([float(i), 0.0, 0.0, 0.0]))
    assert len(g) == 3  # FIFO eviction kept size at capacity


def test_gallery_mean_empty_returns_none():
    g = Gallery()
    assert g.mean is None
    assert len(g) == 0


def test_gallery_cosine_distance_semantics():
    g = Gallery(size=5, ema_alpha=0.0)
    for _ in range(5):
        g.add(_emb([1.0, 0.0, 0.0, 0.0]))
    close = g.cosine_distance_to(_emb([1.0, 0.0, 0.0, 0.0]))
    far = g.cosine_distance_to(_emb([0.0, 0.0, 0.0, 1.0]))
    assert 0.0 <= close <= 0.1
    assert 0.9 <= far <= 1.0


def test_gallery_cosine_to_returns_similarity():
    g = Gallery(size=5, ema_alpha=0.0)
    for _ in range(5):
        g.add(_emb([1.0, 0.0, 0.0, 0.0]))
    sim = g.cosine_to(_emb([0.0, 1.0, 0.0, 0.0]))
    # Similarity = 1 - distance.
    assert 0.0 <= sim <= 1.0
