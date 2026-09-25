"""C11b: mil (DE-Goodman) testleri."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import shafts  # noqa: E402


def test_marin_surface_factor():
    """Talasli yuzey: ka = 4.51 * Sut^-0.265; Sut = 630 MPa -> 0.817 (elle: e^(-0.265 ln 630) = 0.1812)."""
    Se = shafts.endurance(630.0, 2.0)             # d <= 2.79: kb = 1
    assert Se / (0.5 * 630 * shafts.KE_90) == pytest.approx(0.8172, abs=1e-3)


def test_kb_size_factor_ranges():
    assert shafts.kb(10) == pytest.approx(1.24 * 10 ** -0.107)
    assert shafts.kb(80) == pytest.approx(1.51 * 80 ** -0.157)


def test_d_and_n_are_inverse():
    """de_goodman_d ile bulunan capta emniyet katsayisi tam hedef olmali."""
    M, T, Sut, Se, Kf, Kfs, n = 50000.0, 80000.0, 630.0, 250.0, 2.14, 3.0, 2.0
    d = shafts.de_goodman_d(M, T, Sut, Se, Kf, Kfs, n)
    assert shafts.de_goodman_n(d, M, T, Sut, Se, Kf, Kfs) == pytest.approx(n)


def test_pure_torsion_reduces_to_static_formula():
    """M = 0: d^3 = 16 n sqrt(3) Kfs T / (pi Sut)."""
    T, Sut, n = 100000.0, 630.0, 2.0
    d = shafts.de_goodman_d(0.0, T, Sut, 300.0, 1.0, 1.0, n)
    assert d ** 3 == pytest.approx(16 * n * math.sqrt(3) * T / (math.pi * Sut))


def test_design_meets_targets_and_is_standard():
    g, checks = shafts.design(60000.0, 100000.0, "1045", "keyseat", n=2.0)
    assert g["d"] in shafts.STANDARD_D and g["d"] >= g["d_min"]
    assert g["nf"] >= 2.0 and all(c["util"] <= 1.0 for c in checks)


def test_worse_notch_needs_thicker_shaft():
    d_plain, _ = shafts.design(60000.0, 100000.0, "1045", "plain")
    d_ring, _ = shafts.design(60000.0, 100000.0, "1045", "ring_groove")
    assert d_ring["d_min"] > d_plain["d_min"]


def test_stronger_steel_thinner_shaft():
    s1020, _ = shafts.design(60000.0, 100000.0, "1020")
    s1045, _ = shafts.design(60000.0, 100000.0, "1045")
    assert s1045["d_min"] < s1020["d_min"]
