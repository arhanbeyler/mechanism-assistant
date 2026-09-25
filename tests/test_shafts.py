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


# ------------------------------------------------------------------ geometri (C11b)

def test_key_table_din6885():
    assert shafts.key_for(28) == (8, 7, 4.0)      # 22 < d <= 30
    assert shafts.key_for(30) == (8, 7, 4.0)
    assert shafts.key_for(31) == (10, 8, 5.0)


@pytest.mark.parametrize("feature", list(shafts.FEATURES))
def test_layout_is_continuous_and_hub_at_midspan(feature):
    lay = shafts.layout(28.0, feature)
    segs = lay["segments"]
    assert segs[0][0] == 0 and segs[-1][1] == pytest.approx(lay["length"])
    assert all(a[1] == pytest.approx(b[0]) for a, b in zip(segs, segs[1:]))   # bosluksuz
    assert min(r for *_, r in segs) <= 14.0 and all(r >= 14.0 - 1.0 for *_, r in segs)
    (xb1, B, _), (xb2, _, _) = lay["bearings"]
    assert sum(lay["hub"]) / 2 == pytest.approx((xb1 + xb2) / 2)            # gobek aciklik ortasinda (M maks)
    assert lay["hub"][0] - (xb1 + B / 2) > 0                                # gobek yatakla cakismaz


def test_layout_draws_only_the_analysed_notch():
    """Hesap tek bir K_t ile yapildi: baska kademe/kanal cizilmemeli."""
    for feature in ("keyseat", "plain"):
        assert len(shafts.layout(28.0, feature)["segments"]) == 1
    sh = shafts.layout(20.0, "round_shoulder")
    assert sh["feature"]["D"] / 20.0 == pytest.approx(1.5) and sh["feature"]["fillet"] == pytest.approx(2.0)
    assert shafts.layout(20.0, "sharp_shoulder")["feature"]["fillet"] == pytest.approx(0.4)


def test_keyseat_inside_hub():
    lay = shafts.layout(28.0, "keyseat")
    ft = lay["feature"]
    assert lay["hub"][0] < ft["x"] - ft["length"] / 2 and ft["x"] + ft["length"] / 2 < lay["hub"][1]
    assert ft["t1"] < ft["h"]                     # kama yuvadan tasar (gobek tarafi)


def test_profile_fillet_is_tangent_quarter_arc():
    pts = shafts.profile(shafts.layout(20.0, "round_shoulder"))
    arc = next(p for p in pts if p[0] == "arc")
    _, c, s, e = arc
    assert math.dist(c, s) == pytest.approx(2.0) and math.dist(c, e) == pytest.approx(2.0)
    assert s[0] == pytest.approx(e[0] - 2.0) and s[1] == pytest.approx(e[1] + 2.0)
    assert pts[0] == (0.0, 0.0) and pts[-1][1] == 0.0      # eksen uzerinde kapanir


def test_span_is_respected_or_widened():
    assert shafts.layout(20.0, "plain", span=200)["span"] == 200
    assert shafts.layout(20.0, "plain", span=10)["span"] >= 3 * 20.0
