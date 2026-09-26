"""F1: slider-crank. Each result is checked against a relation that is independent of how it was computed."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pipeline  # noqa: E402
import slider_crank as sc  # noqa: E402
import strength  # noqa: E402


@pytest.mark.parametrize("a, b, e", [(30, 90, 0), (30, 120, 10), (20, 70, -8)])
def test_simulated_stroke_matches_closed_form(a, b, e):
    m = sc.measure(sc.simulate(a, b, e, 3600))
    assert m["stroke"] == pytest.approx(sc.stroke_of(a, b, e), abs=1e-3)


def test_inline_geometry():
    """In-line: stroke = 2a, extreme positions b - a and b + a, Q = 1, largest rod angle asin(a/b)."""
    m = sc.measure(sc.simulate(30, 90, 0, 3600))
    assert (m["x_min"], m["x_max"]) == (pytest.approx(60), pytest.approx(120))
    assert m["Q"] == pytest.approx(1.0, abs=1e-3)
    assert m["phi_max"] == pytest.approx(math.degrees(math.asin(1 / 3)), abs=0.05)


def test_offset_gives_quick_return():
    q = [sc.measure(sc.simulate(30, 100, e, 3600))["Q"] for e in (0, 10, 20)]
    assert q[0] == pytest.approx(1.0, abs=1e-3) and q[0] < q[1] < q[2]


@pytest.mark.parametrize("S, e", [(60, 0), (60, 10), (100, 25)])
def test_synthesis_meets_its_rules(S, e):
    g = sc.synthesize(S, e)
    m = sc.measure(sc.simulate(g["a"], g["b"], g["e"]))
    assert m["stroke"] == pytest.approx(S, abs=0.01)
    assert g["ratio"] >= sc.RATIO_MIN - 1e-9 and m["phi_max"] <= sc.PHI_MAX + 1e-6
    assert g["b"] == int(g["b"])                                     # whole millimetres
    shorter = sc.solve_crank(S, g["b"] - 1, e)                       # 1 mm shorter rod breaks a rule
    if shorter:
        ms = sc.measure(sc.simulate(shorter, g["b"] - 1, e, 360))
        assert (g["b"] - 1) / shorter < sc.RATIO_MIN or ms["phi_max"] > sc.PHI_MAX


def test_crank_torque_equals_moment_of_rod_force():
    """Virtual work T = F |dx/dθ| must equal |r_OA × F_rod| with F_rod = F / cos(φ) along the rod."""
    a, b, e, F = 30.0, 100.0, 10.0, 500.0
    for deg in (20, 75, 130, 200, 290):
        th = math.radians(deg)
        x, phi = sc.position(a, b, e, th)
        A = (a * math.cos(th), a * math.sin(th))
        u = ((x - A[0]) / b, (e - A[1]) / b)                        # rod direction A -> B
        Frod = F / math.cos(phi)
        moment = abs(A[0] * u[1] - A[1] * u[0]) * Frod
        assert F * abs(sc.dx_dth(a, b, e, th)) == pytest.approx(moment, rel=1e-9)


def test_peak_acceleration_inline():
    """In-line slider-crank: |a| at the outer dead centre = r ω² (1 + r/b) exactly."""
    frames = sc.simulate(30, 90, 0, 3600)
    _, acc = sc.kinematics_at(frames, 600)
    w = 600 * 2 * math.pi / 60
    assert abs(acc[0]) == pytest.approx(30 * w * w * (1 + 30 / 90) / 1000, rel=1e-4)


def test_sizing_passes_its_checks():
    g = sc.synthesize(60, 0)
    forc = sc.forces(sc.simulate(g["a"], g["b"], 0), g["a"], 200.0)
    sec, checks = sc.size(forc, g["a"], g["b"], strength.MATERIALS["PETG"])
    assert sec is not None and set(sec["links"]) == {"Crank", "Coupler"}
    assert all(c["util"] <= 1.0 for c in checks)


def test_pipeline_slider_crank():
    r = pipeline.design_from_params(kind="slider_crank", stroke_mm=60, offset_mm=0, max_load_N=200,
                                    speed_rpm=600, material="PETG")
    assert r["status"] == "ok" and r["fusion"]["type"] == "slider_crank"
    assert r["slider"]["motion"]["a_max"] == pytest.approx(158, rel=0.01)


def test_crank_length_gives_inline_stroke(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: {"mechanism_type": "slider_crank", "crank_length_mm": 25,
                                                            "stroke_mm": None, "offset_mm": None, "material": None})
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok" and r["slider"]["stroke"] == pytest.approx(50, abs=0.01)
