"""G1c: straight bevel gears (Tredgold)."""

import math

import pytest

import bevel
import gears
import strength

STEEL = strength.MATERIALS["steel"]


def test_shigley_bevel_force_example():
    # pinion 1:3, W_t = 406 lbf  ->  W_r = 140 lbf, W_a = 46.7 lbf (Shigley's bevel force analysis example)
    m, z1, z2 = 5.08, 15, 45
    g = bevel.geometry(m, z1, z2)
    T1 = 406 * g["dm1"] / 2
    f = bevel.forces(T1, m, z1, z2)
    assert f["Wt"] == pytest.approx(406)
    assert f["Wr1"] == pytest.approx(140.2, abs=0.2) and f["Wa1"] == pytest.approx(46.7, abs=0.1)
    assert (f["Wr2"], f["Wa2"]) == pytest.approx((f["Wa1"], f["Wr1"]))     # 90 deg shafts: swapped on the gear


def test_cones_and_virtual_teeth():
    g = bevel.geometry(2, 16, 32)
    assert math.degrees(g["gamma1"] + g["gamma2"]) == pytest.approx(90)
    assert g["d1"] / 2 / math.sin(g["gamma1"]) == pytest.approx(g["d2"] / 2 / math.sin(g["gamma2"]))   # one apex
    assert g["zv1"] == pytest.approx(16 * math.hypot(16, 32) / 32)


def test_mitre_virtual_contact_ratio_exceeds_the_spur_pair():
    # 1:1 bevel: each virtual gear has z / cos 45 teeth, so the contact ratio is that of a larger spur pair
    assert bevel.virtual_contact_ratio(2, 20, 20) > gears.contact_ratio(2, 20, 20)


def test_design_passes():
    for ratio, T in ((1, 5000), (2, 10000), (3, 20000)):
        d, checks = bevel.design(ratio, T, STEEL, 1000)
        assert d and all(c["util"] <= 1 for c in checks)
        assert d["face_width"] <= d["Ao"] / 3 + 1e-9
        assert d["Wt"] * d["dm1"] / 2 == pytest.approx(T)
