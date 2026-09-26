"""I2: printed flexure / living hinge."""

import math

import pytest

import flexure


def test_strain_is_half_thickness_over_radius():
    t, L, th = 0.6, 10.0, math.radians(90)
    assert flexure.strain(t, th, L) == pytest.approx((t / 2) / (L / th))


def test_moment_is_beam_bending():
    d, _ = flexure.design(90, "PETG")
    I = d["w"] * d["t"] ** 3 / 12
    # M = E I / rho with rho = L / theta
    assert d["M"] == pytest.approx(d["E"] * I / d["radius"])


def test_shortest_strip_and_cycles_need_a_longer_one():
    d, checks = flexure.design(90, "PLA")
    assert d and all(c["util"] <= 1 for c in checks)
    shorter = d["L"] - 0.5
    assert all(flexure.strain(t, math.radians(90), shorter) > d["eps_allow"] for t in flexure.THICK)
    cyc, _ = flexure.design(90, "PLA", cyclic=True)
    assert cyc["L"] > d["L"]
