"""H4: torsion spring (Shigley 10-12) and compression-spring fatigue (Zimmerli, Gerber)."""

import math

import pytest

import springs
import torsion_spring as ts


def test_shigley_example_10_8_torsion_spring():
    # 0.072 in music wire, 4.25 body turns, OD 19/32 in, 1 in legs: M = 7.55 lbf in, k' = 29.2 lbf in/turn, 93 deg
    d = 0.072
    D = 19 / 32 - d
    assert ts.k_i(D / d) == pytest.approx(1.115, abs=1e-3)
    Na = ts.active_turns(4.25, D, 1, 1)
    k = ts.rate_per_turn(d, D, Na, 28.5e6)
    Sy = 0.78 * 201 / d ** 0.145 * 1000
    M = Sy * math.pi * d ** 3 / (ts.k_i(D / d) * 32)
    assert Na == pytest.approx(4.657, abs=1e-3) and k == pytest.approx(29.2, abs=0.05)
    assert M == pytest.approx(7.55, abs=0.01) and 360 * M / k == pytest.approx(93.1, abs=0.2)


def test_torsion_design_meets_the_moment_and_clears_the_arbor():
    s, checks = ts.design(2000, 120, arbor=10)
    assert s and all(c["util"] <= 1 for c in checks)
    assert s["k_turn"] * s["turns"] == pytest.approx(2000)
    assert s["theta_deg"] == pytest.approx(120, rel=0.1)          # whole quarter body turns
    assert s["ID_wound"] >= 11 and s["ID_wound"] < s["D"] - s["d"]  # the coil closes down onto the arbor


def test_gerber_point_lies_on_the_curve():
    mat = springs.SPRING_MATERIALS["music_wire"]
    nf, ta, tm, Sse, Ssu = springs.fatigue(2.5, 18.0, 50, 200, mat)
    assert (nf * ta) / Sse + (nf * tm / Ssu) ** 2 == pytest.approx(1.0)


def test_cycling_load_needs_a_stronger_spring():
    mat = springs.SPRING_MATERIALS["music_wire"]
    static, _ = springs.design(200, 20, mat)
    cyc, checks = springs.design(200, 20, mat, F_min=50)
    assert cyc and all(c["util"] <= 1 for c in checks)
    assert cyc["d"] >= static["d"] and cyc["mass_index"] > static["mass_index"]
