"""F4d: single-dwell six-bar."""

import math

import pytest

import dwell
import pipeline


def test_kasa_fits_a_circle_exactly():
    pts = [(3 + 5 * math.cos(t / 10), -2 + 5 * math.sin(t / 10)) for t in range(20)]
    cx, cy, R, dev = dwell.kasa(pts)
    assert (cx, cy, R) == pytest.approx((3, -2, 5), abs=1e-9) and dev < 1e-9


def test_dwell_mechanism_dwells_and_swings():
    r = pipeline.design_from_params(kind="dwell", crank_length_mm=30, dwell_deg=90, output_swing_deg=40)
    assert r["status"] == "ok"
    d = r["dwell"]
    assert d["dwell_motion"] <= 1.0 and d["swing"] == pytest.approx(40, abs=2)
    assert min(d["mu1"][0], d["mu6"][0]) >= dwell.MU_MIN


def test_the_trivial_arcs_are_not_used():
    r = pipeline.design_from_params(kind="dwell", crank_length_mm=30, dwell_deg=90, output_swing_deg=40)
    d = r["dwell"]
    assert min(abs(d["p"]), abs(d["p"] - d["b"])) > 0.4 * d["a"]            # P is neither A nor B
