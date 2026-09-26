"""G3: roller chain and V-belt drives (Shigley ch. 17)."""

import math

import pytest

import chain
import vbelt

HP = 745.7


def test_chain_rating_reproduces_table_17_20():
    # No. 40 on a 17-tooth sprocket at 50 rev/min: 0.37 hp (link-plate limit)
    h, limit = chain.rated_hp(chain.chain(40), 17, 50)
    assert h == pytest.approx(0.37, abs=0.005) and limit == "link plate"
    # at high speed the roller-bushing limit takes over and the rating falls again
    lo, hi = chain.rated_hp(chain.chain(40), 17, 7000), chain.rated_hp(chain.chain(40), 17, 3000)
    assert lo[1] == "roller-bushing" and lo[0] < hi[0]


def test_chain_shigley_example_strand_choice():
    # 90 hp at 300 rev/min, 2:1, Ks = 1.3 and nd = 1.5: a 3-strand No. 140 carries it, a 3-strand No. 120 does not
    Ks = 1.3 * 1.5
    for no, ok in ((140, True), (120, False)):
        info, checks = chain.evaluate(chain.chain(no), 3, 17, 34, 300, 90 * HP, Ks)
        assert (checks[0]["util"] <= 1.0) is ok
    assert 90 * Ks / 2.5 == pytest.approx(70.2)          # the per-strand requirement Shigley reads the table with


def test_chain_length_and_centre_are_inverse():
    p, N1, N2 = 12.7, 19, 57
    for C in (300.0, 508.0, 700.0):
        L = chain.links(C, p, N1, N2)
        assert chain.centre(L, p, N1, N2) == pytest.approx(C, rel=1e-9)


def test_chain_design_whole_even_links_and_ratio():
    d, checks = chain.design(1500, 1400, 3)
    assert d and all(c["util"] <= 1 for c in checks)
    assert d["links"] % 2 == 0 and d["N1"] >= 17
    assert d["N2"] / d["N1"] == pytest.approx(3, rel=0.02)
    assert chain.links(d["C"], d["p"], d["N1"], d["N2"]) == pytest.approx(d["links"], abs=1e-6)
    # speed-up: the driver is the large sprocket
    u, _ = chain.design(1500, 400, 1 / 3)
    assert not u["small_is_input"] and u["n1"] == pytest.approx(1200)


def test_chordal_speed_variation():
    assert chain.chordal(17) == pytest.approx(0.0171, abs=2e-4)
    assert chain.chordal(25) < chain.chordal(17)


def test_vbelt_shigley_example_17_4():
    # 10 hp at 1750 rev/min, 7.4 in and 11 in sheaves, B112 belts, Ks 1.3
    V = math.pi * 7.4 * 1750 / 12
    assert vbelt.h_tab("B", 7.4, V) == pytest.approx(4.693, abs=0.002)
    C = vbelt.centre(112 + 1.8, 11, 7.4)
    assert C == pytest.approx(42.4, abs=0.05)
    phi = math.pi - 2 * math.asin(3.6 / (2 * C))
    assert phi == pytest.approx(3.057, abs=0.001)
    assert vbelt.k1(11, 7.4, C) == pytest.approx(0.99, abs=0.005) and vbelt.k2("B", 112) == 1.05
    t = vbelt.tensions("B", 13, 3, 1750, 7.4, 11, phi)
    assert (t["Fc"], t["dF"], t["F1"], t["F2"], t["Fi"]) == pytest.approx((11.1, 42.2, 64.4, 22.2, 32.2), abs=0.06)
    Np, hours = vbelt.life_hours("B", t["F1"], 7.4, 11, 113.8, V)
    assert Np == vbelt.NP_MAX and hours > 46000                  # "more than 1e9 passes, t > 46 000 h"
    info, _ = vbelt.evaluate("B", 7.4, 11, 1750, 10 * HP, 1.3, C_in=C)
    assert info["belt"] == "B112" and info["belts"] == 3


def test_vbelt_length_formula_round_trip():
    for C in (20.0, 35.0):
        assert vbelt.centre(vbelt.pitch_length(C, 12, 5), 12, 5) == pytest.approx(C, rel=1e-9)


def test_vbelt_design_passes_and_ratio():
    d, checks = vbelt.design(750, 1400, 2.5)
    assert d and all(c["util"] <= 1 for c in checks)
    assert d["D"] / d["d"] == pytest.approx(2.5) and d["n_fs"] >= 1
    assert vbelt.h_tab("A", 4.0, 6000) is None                  # beyond 5000 ft/min: not rated
