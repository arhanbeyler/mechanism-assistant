"""H5b: disc clutch, uniform wear (Shigley 16-5)."""

import math

import pytest

import clutch


def test_torque_forms_agree():
    ri, ro, pa, f = 50.0, 90.0, 1.0, 0.3
    F = clutch.force(pa, ri, ro)
    assert clutch.torque(2, f, pa, ri, ro) == pytest.approx(2 * f * F * (ro + ri) / 2)


def test_best_inner_radius_is_ro_over_root3():
    ro = 100.0
    best = max((clutch.torque(2, 0.3, 1.0, ri, ro), ri) for ri in [ro * k / 1000 for k in range(100, 1000)])
    assert best[1] == pytest.approx(ro / math.sqrt(3), abs=0.2)


def test_design_and_extra_faces_for_a_small_diameter():
    d, checks = clutch.design(100000)
    assert d and d["N"] == 2 and all(c["util"] <= 1 for c in checks)
    small, _ = clutch.design(100000, od_max=2 * d["ro"] - 20)
    assert small["N"] > 2
