"""H2: fillet-welded cantilever (Shigley ch. 9)."""

import math

import pytest

import welded


def _lines(pattern, b, d):
    top = [((-b / 2, d / 2), (b / 2, d / 2)), ((-b / 2, -d / 2), (b / 2, -d / 2))]
    sides = [((-b / 2, -d / 2), (-b / 2, d / 2)), ((b / 2, -d / 2), (b / 2, d / 2))]
    return {"top_bottom": top, "sides": sides, "all_around": top + sides}[pattern]


def test_unit_properties_by_integration_along_the_lines():
    b, d, n = 30.0, 50.0, 20000
    for pattern in welded.PATTERNS:
        A = I = 0.0
        for (x0, y0), (x1, y1) in _lines(pattern, b, d):
            seg = math.hypot(x1 - x0, y1 - y0)
            for k in range(n):
                y = y0 + (y1 - y0) * (k + 0.5) / n
                A += seg / n
                I += y * y * seg / n
        Au, Iu = welded.unit_props(pattern, b, d)
        assert A == pytest.approx(Au) and I == pytest.approx(Iu, rel=1e-6)


def test_stresses_combine_as_vectors():
    s = welded.weld_stress(5000, 100, 20, 40, 6, "all_around")
    assert s["tau1"] == pytest.approx(5000 / (0.707 * 6 * 120))
    assert s["tau"] == pytest.approx(math.hypot(s["tau1"], s["tau2"]))


def test_design_takes_the_smallest_leg_and_sizes_the_bar():
    a, checks = welded.design(5000, 100)
    assert a and all(c["util"] <= 1 for c in checks)
    assert a["d"] == 2 * a["b"] and a["h"] >= welded.min_leg(a["b"])
    smaller = [h for h in welded.LEGS if welded.min_leg(a["b"]) <= h < a["h"]]
    for h in smaller:                                          # every smaller leg fails
        assert welded.weld_stress(5000, 100, a["b"], a["d"], h, "all_around")["tau"] > a["tau_all"]


def test_weld_on_the_sides_only_is_weakest_in_bending():
    t = {p: welded.weld_stress(10000, 150, 30, 60, 6, p)["tau2"] for p in welded.PATTERNS}
    assert t["all_around"] < t["top_bottom"] < t["sides"]
