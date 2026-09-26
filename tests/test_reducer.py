"""E1 single-stage reducer: the coupling constraints between gear, shafts, bearings and housing hold."""

import math

import pytest

import gears
import reducer
import shafts

CASES = [(10000.0, 1450.0, 3.0, "steel"), (50000.0, 1450.0, 4.0, "steel"), (200000.0, 1450.0, 5.0, "steel"),
         (2000.0, 300.0, 3.0, "PLA")]


@pytest.fixture(scope="module", params=CASES, ids=lambda c: f"{c[0] / 1000:g}Nm-i{c[2]:g}-{c[3]}")
def result(request):
    T1, rpm, i, mat = request.param
    r, checks = reducer.design(T1, rpm, i, mat)
    assert r is not None, [c for c in checks if c["util"] > 1]
    return r, checks, request.param


def test_all_checks_pass(result):
    r, checks, _ = result
    assert all(c["util"] <= 1.0 for c in checks)
    assert {c["group"] for c in checks} >= {"Gear", "Input shaft", "Output shaft", "Housing"}


def test_gear_force_reaches_both_shafts(result):
    """W = 2 T1 / d1 / cos(phi) is the radial load at mid-span of both shafts (M = W L / 4)."""
    r, _, (T1, _, _, _) = result
    W = 2 * T1 / r["gear"]["d1"] / math.cos(gears.PHI)
    for s in (r["shaft_in"], r["shaft_out"]):
        assert s["F"] == pytest.approx(W)
        assert s["M"] == pytest.approx(W * s["layout"]["span"] / 4)


def test_torque_and_speed_follow_the_ratio(result):
    r, _, (T1, rpm, _, _) = result
    i = r["gear"]["ratio"]
    assert r["shaft_out"]["T"] == pytest.approx(T1 * i)
    assert r["shaft_out"]["rpm"] == pytest.approx(rpm / i)


def test_geometry_fits_together(result):
    r, _, _ = result
    g, s1, s2 = r["gear"], r["shaft_in"], r["shaft_out"]
    assert (g["bore1"], g["bore2"]) == (s1["d"], s2["d"])                          # gear sits on the shaft
    assert s1["key"]["length"] <= g["face_width"] and s2["key"]["length"] <= g["face_width"]   # key inside gear
    assert s1["layout"]["span"] == s2["layout"]["span"]                             # common housing walls
    assert g["center_distance"] >= (s1["bearing"]["OD"] + s2["bearing"]["OD"]) / 2 + reducer.WEB
    for s in (s1, s2):
        assert s["layout"]["hub"][1] - s["layout"]["hub"][0] >= g["face_width"]  # hub seat holds the gear


def test_gear_mass_lowers_critical_speed(result):
    r, _, _ = result
    s2 = r["shaft_out"]
    bare = shafts.beam(s2["layout"], 1.0)["n_crit"]
    assert s2["beam"]["n_crit"] < bare


def test_housing_encloses_bearings_and_gears():
    r, _ = reducer.design(50000.0, 1450.0, 4.0)
    h, g = r["housing"], r["gear"]
    (y0, y1), (z0, z1) = h["y"], h["z"]
    for yc, od in h["seats"]:
        assert y0 < yc - od / 2 and yc + od / 2 < y1 and z0 < -od / 2 and od / 2 < z1
    ra = max(g["d1"], g["d2"]) / 2 + g["module"]
    assert z0 + h["base_t"] < -ra                                                    # gears clear the base
