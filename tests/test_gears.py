"""C9: duz disli modulu testleri."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gears  # noqa: E402
import strength  # noqa: E402

PETG = strength.MATERIALS["PETG"]


def test_min_teeth_no_undercut():
    """20 derece basinc acisi: z_min = 2 / sin^2(20) = 17.1."""
    assert gears.z_min() == pytest.approx(17.1, abs=0.05)


def test_lewis_table_values():
    assert gears.lewis_y(17) == pytest.approx(0.303)
    assert gears.lewis_y(20) == pytest.approx(0.322)
    assert gears.lewis_y(23) == pytest.approx((0.331 + 0.337) / 2)   # ara degerleme


def test_contact_ratio_known_value():
    """z1 = z2 = 20, m = 1: kavrama orani ~1.56 (klasik ders ornegi)."""
    assert gears.contact_ratio(1.0, 20, 20) == pytest.approx(1.557, abs=0.01)


def test_lewis_hand_calculation():
    """m=2, z1=20, b=20, T1=10 N m -> Ft = 2*10000/40 = 500 N, sigma = 500/(20*2*0.322) = 38.8 MPa (Kv=1)."""
    c = gears.check_pair(2.0, 20, 40, 20.0, 10000.0, PETG)
    pinion = [x for x in c if x["name"].startswith("Pinyon")][0]
    assert pinion["util"] * (35 / 3) == pytest.approx(500 / (20 * 2 * 0.322), rel=1e-6)


def test_design_meets_all_checks():
    g, checks = gears.design(3.0, 2000.0, PETG)
    assert g is not None
    assert g["z1"] >= 17
    assert abs(g["ratio"] - 3.0) / 3.0 <= 0.02
    assert all(c["util"] <= 1.0 for c in checks)
    assert g["module"] in gears.MODULES


def test_stronger_material_smaller_gears():
    g_pla, _ = gears.design(2.0, 5000.0, strength.MATERIALS["PLA"])
    g_st, _ = gears.design(2.0, 5000.0, strength.MATERIALS["steel"])
    assert g_st["volume"] < g_pla["volume"]


def test_impossible_torque_returns_none():
    g, checks = gears.design(2.0, 1e9, strength.MATERIALS["PLA"])
    assert g is None and checks


def test_involute_tooth_geometry():
    """Evolvent noktalari taban dairesi ile uc dairesi arasinda; yanlar x eksenine simetrik."""
    right, left, rf, ra = gears.involute_tooth(2.0, 20)
    rb = 20 * math.cos(gears.PHI)
    for (x1, y1), (x2, y2) in zip(right, left):
        r = math.hypot(x1, y1)
        assert max(rb, rf) - 1e-9 <= r <= ra + 1e-9
        assert x1 == pytest.approx(x2) and y1 == pytest.approx(-y2)
    # uc noktasinda dis kalinligi pozitif (sivri degil)
    assert left[-1][1] > 0
