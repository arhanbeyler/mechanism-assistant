"""F4c: Whitworth quick return."""

import math

import pytest

import pipeline
import strength
import whitworth as w


def test_time_ratio_and_stroke_by_simulation():
    for S, Q in ((200, 1.5), (100, 2.0), (150, 1.3)):
        g = w.geometry(S, Q)
        assert g["r"] > g["d"]                                    # the lever turns full circles
        me = w.measure(w.frames(g))
        assert me["stroke"] == pytest.approx(S, rel=1e-4)
        assert me["Q"] == pytest.approx(Q, abs=0.01)
        assert (180 + 2 * g["alpha"]) / (180 - 2 * g["alpha"]) == pytest.approx(Q)


def test_slot_spans_the_pin_radii():
    g = w.geometry(200, 1.5)
    me = w.measure(w.frames(g))
    assert me["rho_min"] == pytest.approx(g["r"] - g["d"], rel=1e-3) and me["rho_max"] == pytest.approx(g["r"] + g["d"], rel=1e-3)


def test_pipeline_routes_to_whitworth():
    r = pipeline.design_from_params(kind="quick_return", stroke_mm=200, time_ratio=1.5, max_load_N=500, whitworth="yes")
    assert r["status"] == "ok" and r["mechanism_type"] == "whitworth"
    assert r["whitworth"]["T_max"] > 0
