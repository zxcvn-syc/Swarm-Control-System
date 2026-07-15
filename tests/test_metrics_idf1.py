"""Dedicated IDF1 metric tests (extracted from test_metrics.py for clarity)."""

from __future__ import annotations

import pytest

from cvtrack.tracker.metrics import idf1
from cvtrack.types import Box


def _b(x1, y1, x2, y2):
    return Box(x1=x1, y1=y1, x2=x2, y2=y2, score=1.0, cls=0, label="obj")


def test_idf1_empty_inputs():
    out = idf1([], [], [], [])
    assert out["idf1"] == 0.0
    assert out["idp"] == 0.0
    assert out["idr"] == 0.0
    assert out["mapping"] == {}
    assert out["tp"] == 0
    assert out["fp"] == 0
    assert out["fn"] == 0


def test_idf1_perfect_match_is_one():
    boxes = [_b(0.0, 0.0, 10.0, 10.0), _b(0.0, 0.0, 10.0, 10.0)]
    out = idf1([1, 1], [1, 1], boxes, boxes)
    assert out["idf1"] == pytest.approx(1.0)
    assert out["idp"] == pytest.approx(1.0)
    assert out["idr"] == pytest.approx(1.0)


def test_idf1_complete_swap_is_zero():
    boxes_a = [_b(0.0, 0.0, 10.0, 10.0), _b(100.0, 100.0, 110.0, 110.0)]
    boxes_c = [_b(200.0, 200.0, 210.0, 210.0), _b(300.0, 300.0, 310.0, 310.0)]
    out = idf1([1, 2], [3, 4], boxes_a, boxes_c)
    # No overlapping pairs -> no co-occurrence -> idf1 = 0.
    assert out["idf1"] == pytest.approx(0.0, abs=1e-6)
    assert out["tp"] == 0
    # fp/fn are 0 when co_counts is empty (no pairs to compute from).
    assert out["fp"] == 0
    assert out["fn"] == 0


def test_idf1_partial_overlap_yields_value_between_0_and_1():
    a = _b(0.0, 0.0, 10.0, 10.0)
    b = _b(100.0, 100.0, 110.0, 110.0)
    out = idf1([1, 1, 2], [1, 2, 2], [a, a, b], [a, b, b])
    assert 0.0 <= out["idf1"] <= 1.0


def test_idf1_handles_mismatched_lengths():
    out = idf1([1], [1, 1], [_b(0, 0, 1, 1)], [_b(0, 0, 1, 1), _b(0, 0, 1, 1)])
    assert out["idf1"] == 0.0


def test_idf1_greedy_mapping_picks_best_1to1():
    a = _b(0.0, 0.0, 10.0, 10.0)
    b = _b(100.0, 100.0, 110.0, 110.0)
    # 4 observations: gt={1,2}, pred={7,8}
    # (1->7) co-occurs 2 times; (2->8) co-occurs 2 times.
    out = idf1([1, 1, 2, 2], [7, 7, 8, 8], [a, a, b, b], [a, a, b, b])
    # Greedy picks (1->7, 2->8) all matched.
    assert out["idf1"] == pytest.approx(1.0)
    assert out["mapping"] == {1: 7, 2: 8}


def test_idf1_idp_idr_components():
    a = _b(0.0, 0.0, 10.0, 10.0)
    b = _b(100.0, 100.0, 110.0, 110.0)
    out = idf1([1, 1, 2, 2], [7, 7, 8, 8], [a, a, b, b], [a, a, b, b])
    # All 4 obs are co-occurring within same id groups -> perfect precision/recall.
    assert out["idp"] == pytest.approx(1.0)
    assert out["idr"] == pytest.approx(1.0)
