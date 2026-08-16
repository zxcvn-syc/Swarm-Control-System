"""Tests for CLI-to-YAML precedence and required inputs."""

from __future__ import annotations

import pytest

from cvtrack.pipeline import _args_to_overrides, _build_parser


def test_source_is_required() -> None:
    parser = _build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args([])


def test_cli_defaults_do_not_override_yaml_values() -> None:
    args = _build_parser().parse_args(["--source", "clip.mp4"])
    overrides = _args_to_overrides(args)

    assert args.device is None
    assert args.reid_model is None
    assert args.cmc_method is None
    assert args.predict_horizon is None
    assert "stationary_prune" not in overrides.get("tracker", {})
    assert "model" not in overrides.get("appearance", {})
    assert "predict_horizon" not in overrides.get("pipeline", {})


def test_explicit_cli_values_become_overrides() -> None:
    args = _build_parser().parse_args(
        [
            "--source",
            "clip.mp4",
            "--device",
            "cuda:0",
            "--reid-model",
            "osnet_x0_25",
            "--cmc-method",
            "ecc",
            "--no-stationary-prune",
            "--predict-horizon",
            "7",
            "--write-future-csv",
        ]
    )
    overrides = _args_to_overrides(args)

    assert overrides["detector"]["device"] == "cuda:0"
    assert overrides["appearance"]["model"] == "osnet_x0_25"
    assert overrides["tracker"]["cmc_method"] == "ecc"
    assert overrides["tracker"]["stationary_prune"] is False
    assert overrides["pipeline"]["predict_horizon"] == 7
    assert overrides["output"]["write_future_csv"] is True
