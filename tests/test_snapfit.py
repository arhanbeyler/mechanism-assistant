"""I1: cantilever snap-fit."""

import math

import pytest

import snapfit


def test_formulas_are_beam_theory():
    w, h, l, y, E = 6.0, 1.5, 15.0, 1.0, 2500.0
    eps = snapfit.strain(y, h, l)
    I = w * h ** 3 / 12
    assert snapfit.deflection_force(w, h, l, E, eps) == pytest.approx(3 * E * I * y / l ** 3)
    M = 3 * E * I * y / l ** 3 * l                              # root moment P l
    assert M * (h / 2) / I / E == pytest.approx(eps)            # root strain sigma / E


def test_wedge_forces():
    assert snapfit.mating_force(10, 30, mu=0) == pytest.approx(10 * math.tan(math.radians(30)))
    assert snapfit.mating_force(10, 90) == math.inf              # a square retaining face locks
    assert snapfit.mating_force(10, 45) > snapfit.mating_force(10, 30)


def test_design_is_the_shortest_within_the_strain():
    d, checks = snapfit.design(1.0, "PLA")
    assert d and all(c["util"] <= 1 for c in checks)
    shorter = d["l"] - 1
    assert all(snapfit.strain(1.0, h, shorter) > d["eps_allow"] for h in snapfit.THICK)
    petg, _ = snapfit.design(1.0, "PETG")
    assert petg["l"] < d["l"]                                    # PETG takes more strain: a shorter beam
    rep, _ = snapfit.design(1.0, "PLA", repeated=True)
    assert rep["l"] > d["l"]


def test_releasable_hook_holds_the_asked_force():
    d, checks = snapfit.design(1.0, "PETG", F_hold=15, alpha_out=45)
    assert d["W_out"] >= 15 and all(c["util"] <= 1 for c in checks)
    locked, _ = snapfit.design(1.0, "PLA")
    assert locked["locked"] and locked["W_out"] is None
