"""G1a: helical gear pair."""

import math

import pytest

import gears
import helical
import strength

STEEL = strength.MATERIALS["steel"]


def test_shigley_helical_geometry_example():
    # 18 teeth, transverse diametral pitch 6 /in, psi 25 deg, phi_n 20 deg: d = 3 in, p_x = 1.123 in, phi_t = 21.88 deg
    mn = 25.4 / 6 * math.cos(math.radians(25))
    g = helical.geometry(mn, 18, 36, 25)
    assert g["d1"] / 25.4 == pytest.approx(3.0)
    assert g["px"] / 25.4 == pytest.approx(1.123, abs=1e-3)
    assert math.degrees(g["phi_t"]) == pytest.approx(21.88, abs=0.01)


def test_rack_undercut_limit_at_30_degrees():
    assert helical.z_min(30) == pytest.approx(11.5, abs=0.05)
    assert helical.z_min(0.0) == pytest.approx(gears.z_min())     # psi = 0: the spur limit


def test_zero_helix_is_a_spur_pair():
    assert helical.transverse_contact_ratio(2, 20, 40, 1e-9) == pytest.approx(gears.contact_ratio(2, 20, 40), rel=1e-6)


def test_design_passes_and_the_helix_overlaps():
    d, checks = helical.design(3, 10000, STEEL, 1400)
    assert d and all(c["util"] <= 1 for c in checks)
    assert d["eps_f"] >= 1 and d["ratio"] == pytest.approx(3)
    assert d["Wa"] == pytest.approx(d["Wt"] * math.tan(math.radians(d["psi"])))
    assert d["center_distance"] == pytest.approx((d["d1"] + d["d2"]) / 2)


def test_helical_is_smaller_than_spur_for_the_same_job():
    # the virtual tooth number raises Y and the wider face spreads the load: lighter than the spur pair
    h, _ = helical.design(4, 20000, STEEL, 1000)
    s, _ = gears.design(4, 20000, STEEL, 1000)
    assert h["center_distance"] <= s["center_distance"]
