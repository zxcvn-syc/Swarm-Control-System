"""Tests for YAML config loading and preset merging."""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

from cvtrack.config import _deep_merge, _validate, load_config


def test_deep_merge_overrides_scalar():
    base = {"detector": {"weights": "a.pt", "imgsz": 320}}
    over = {"detector": {"imgsz": 480}}
    out = _deep_merge(base, over)
    assert out["detector"]["weights"] == "a.pt"
    assert out["detector"]["imgsz"] == 480


def test_load_default_yaml_succeeds(tmp_path):
    p = tmp_path / "cfg.yaml"
    p.write_text("detector:\n  weights: a.pt\n  imgsz: 320\n")
    cfg = load_config(str(p))
    assert cfg.raw["detector"]["weights"] == "a.pt"
    assert cfg.raw["detector"]["imgsz"] == 320


def test_extends_merges_preset():
    with tempfile.TemporaryDirectory() as td:
        base = Path(td) / "base.yaml"
        base.write_text("detector:\n  weights: a.pt\n  imgsz: 320\n")
        child = Path(td) / "child.yaml"
        child.write_text("extends: base\ndetector:\n  imgsz: 480\n")
        cfg = load_config(str(child), configs_dir=td)
        assert cfg.raw["detector"]["weights"] == "a.pt"
        assert cfg.raw["detector"]["imgsz"] == 480


def test_validate_rejects_unknown_top_level():
    with pytest.raises(ValueError):
        _validate({"not_a_real_section": {"x": 1}})


def test_validate_rejects_bad_tracker_kind():
    with pytest.raises(ValueError):
        _validate({"tracker": {"kind": "magic"}})


def test_validate_requires_reid_weights_when_enabled():
    with pytest.raises(ValueError):
        _validate({"appearance": {"enabled": True, "weights": ""}})


def test_bundled_presets_load_clean():
    """Smoke test: the configs bundled with the repo load without raising."""
    from cvtrack.config import _configs_dir

    for name in ("default", "drone", "street"):
        cfg = load_config(name, configs_dir=_configs_dir())
        assert cfg.raw.get("tracker"), f"{name} must define a tracker section"
