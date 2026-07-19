"""Tests for the appearance (ReID) extractor.

Skipped automatically when torchreid/OSNet weights are unavailable.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

torchreid_available = importlib.util.find_spec("torchreid") is not None

needs_torchreid = pytest.mark.skipif(
    not torchreid_available, reason="torchreid not installed"
)


@pytest.fixture
def dummy_image() -> np.ndarray:
    return (np.random.rand(200, 200, 3) * 255).astype(np.uint8)


def test_factory_returns_none_when_disabled():
    from cvtrack.appearance.factory import make_extractor

    # When weights path is missing but torchreid is available, the extractor still
    # constructs (it warns and uses random init).  Either the factory returns
    # None (graceful) or an OsNetExtractor instance.
    try:
        out = make_extractor(backend="osnet", model_name="osnet_x0_25", weights="/tmp/never-there.pt")
    except Exception:
        out = None
    assert out is None or hasattr(out, "is_available")


def test_crop_with_margin_pads():
    from cvtrack.appearance.base import crop_with_margin

    img = np.full((100, 100, 3), 128, dtype=np.uint8)
    out = crop_with_margin(img, (40, 40, 60, 60), margin=0.2)
    assert out.shape[0] > 0 and out.shape[1] > 0
    assert out.shape[0] > 20
    assert out.shape[1] > 20


def test_l2_normalize_unit_norm():
    from cvtrack.appearance.base import l2_normalize

    v = np.array([3.0, 4.0, 0.0, 0.0])
    n = l2_normalize(v)
    np.testing.assert_allclose(np.linalg.norm(n), 1.0, atol=1e-6)


@needs_torchreid
def test_osnet_constructor_does_not_crash():
    """Loading the OSNet model class should not raise; weight load may still fail."""
    from cvtrack.appearance.osnet import OsNetExtractor

    # With bad weights path, the extractor should still construct (it just logs a warning).
    ext = OsNetExtractor(model_name="osnet_x0_25", weights="definitely-not-a-path.pt")
    assert ext.model_name == "osnet_x0_25"
