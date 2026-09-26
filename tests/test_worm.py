"""G1b: worm gear set (Shigley 15-4, AGMA)."""

import math

import pytest

import worm


def test_shigley_example_15_4():
    # single-start steel worm, 1800 rev/min, 24-tooth chill-cast bronze wheel, P_t = 4 /in, d_W = 3 in,
    # phi_n = 14.5 deg, 3 hp out, K_a = 1.25, F = 2 in
    m = 25.4 / 4
    T_G = 3 * worm.HP / (2 * math.pi * (1800 / 24) / 60) * 1000
    a = worm.analyse(m, 1, 24, 3 * 25.4, 1800, 1.25 * T_G, F=2 * 25.4, casting="chill", phi_n=math.radians(14.5))
    assert a["lead_deg"] == pytest.approx(4.764, abs=1e-3)
    assert a["Vs_ft_min"] == pytest.approx(1419, abs=1)
    assert a["f"] == pytest.approx(0.0178, abs=1e-4)
    assert a["efficiency"] == pytest.approx(0.818, abs=1e-3)
    assert a["WGt"] / worm.LBF == pytest.approx(1050, abs=1)
    assert a["WWt"] / worm.LBF == pytest.approx(107, abs=0.5)
    assert (a["Cs"], a["Cm"], a["Cv"]) == pytest.approx((1000, 0.823, 0.211), abs=1e-3)
    assert a["W_all"] / worm.LBF == pytest.approx(1456, abs=2)


def test_friction_curve_is_continuous():
    assert worm.friction(10) == pytest.approx(worm.friction(10.0001), abs=2e-3)
    assert worm.friction(0) == 0.15 and worm.friction(1) < 0.15


def test_efficiency_matches_the_force_ratio():
    # e = output work / input work = W_Gt * v_G / (W_Wt * v_W)
    a = worm.analyse(2.0, 1, 40, 20.0, 1400, 50000)
    vW = math.pi * a["dW"] * 1400
    vG = math.pi * a["dG"] * 1400 / 40
    assert a["WGt"] * vG / (a["WWt"] * vW) == pytest.approx(a["efficiency"], rel=1e-9)


def test_self_locking_design_has_a_small_lead_angle():
    d, checks = worm.design(30, 50000, 1400, self_locking=True)
    assert d and all(c["util"] <= 1 for c in checks)
    assert d["self_locking"] and d["lead_deg"] < 5.5 and d["NW"] == 1
    assert d["efficiency"] < 0.8                                    # the price of self-locking
    free, _ = worm.design(30, 50000, 1400)
    assert free["efficiency"] > d["efficiency"]


def test_short_ratio_cannot_self_lock():
    d, _ = worm.design(15, 30000, 1400, self_locking=True)         # N_G >= 21 needs 2 starts: lead too big
    assert d is None
