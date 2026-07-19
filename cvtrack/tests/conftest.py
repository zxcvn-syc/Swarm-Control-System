"""Test fixtures and helpers for cvtrack."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

# Ensure src/ is on sys.path when running from the repo root without `pip install -e`.
ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if SRC.exists() and str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def pytest_collection_modifyitems(config, items):
    """Mark tests heavier than ~1s as @pytest.mark.slow unless already marked."""
    slow_marker = pytest.mark.slow
    for item in items:
        if "slow" in item.keywords:
            continue
        # Without per-test timing data, default to a simple heuristic: anything that
        # imports torch at module level (appearance) is slow.
        if any(name in item.nodeid for name in ("test_appearance", "test_pipeline_smoke", "test_smoother_noise", "test_kalman_long")):
            item.add_marker(slow_marker)
