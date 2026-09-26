"""G2: planetary gear set."""

import math

import pytest

import planetary as pl
import rack_pinion
import strength

STEEL = strength.MATERIALS["steel"]


def test_design_meets_assembly_and_neighbour_conditions():
    for target, T in ((5, 20000), (4, 5000), (8, 50000), (3.5, 2000)):
        d, checks = pl.design(target, T, STEEL)
        assert d and all(c["util"] <= 1 for c in checks)
        assert d["Zr"] == d["Zs"] + 2 * d["Zp"]
        assert (d["Zs"] + d["Zr"]) % d["n"] == 0
        assert d["ratio"] == pytest.approx(target, rel=0.02)
        assert (d["Zs"] + d["Zp"]) * math.sin(math.pi / d["n"]) > d["Zp"] + 2


def test_willis_equation_by_velocities():
    # pitch-point speeds: sun-planet contact moves with the sun, planet-ring contact is at rest (ring fixed)
    Zs, Zp, m, ws = 20, 25, 1.0, 1.0
    rs, rp = m * Zs / 2, m * Zp / 2
    v_sun_contact = ws * rs
    # planet centre moves at the mean of its two contact speeds (0 at the ring side)
    v_centre = v_sun_contact / 2
    wc = v_centre / (rs + rp)
    assert ws / wc == pytest.approx(pl.ratio(Zs, Zp))


def test_ring_reaction_from_statics():
    d, _ = pl.design(5, 20000, STEEL)
    Ft = d["Ft"] / d["K_gamma"]                                # nominal share of one planet
    T_ring = Ft * d["n"] * d["dr"] / 2
    assert T_ring == pytest.approx(d["T_in"] * d["Zr"] / d["Zs"])
    assert d["T_out"] == pytest.approx(d["T_in"] + T_ring)    # moment balance: carrier = sun + ring


def test_internal_contact_ratio_tends_to_the_rack():
    m, zp = 2.0, 20
    assert pl.contact_ratio_internal(m, zp, 4000) == pytest.approx(rack_pinion.contact_ratio_rack(m, zp), rel=2e-3)
    assert pl.contact_ratio_internal(m, zp, 60) > rack_pinion.contact_ratio_rack(m, zp)   # internal mesh: longer contact


def test_phases_consistent_only_with_the_assembly_condition():
    wrap = lambda x, p: (x + p / 2) % p - p / 2

    def ring_mismatch(Zs, Zp, n):
        Zr = Zs + 2 * Zp
        out, psi = pl.phases(Zs, Zp, Zr, n)
        pp, pr = 2 * math.pi / Zp, 2 * math.pi / Zr
        return max(abs(wrap(th + wrap(phi - th, pp) * Zp / Zr - psi, pr)) for th, phi in out)

    assert ring_mismatch(20, 20, 5) < 1e-9          # (20 + 60) / 5 = 16
    assert ring_mismatch(17, 25, 4) < 1e-9          # (17 + 67) / 4 = 21
    assert ring_mismatch(20, 21, 3) > 1e-3          # (20 + 62) / 3 is not whole: planets 2, 3 cannot mesh


def test_planet_is_checked_as_an_idler():
    info, checks = pl.evaluate(1.0, 20, 20, 3, 10, 5000, STEEL)
    sun, planet = checks[0], checks[1]
    # same force, similar Y: the planet's utilisation is higher by about 1 / 0.7
    assert planet["util"] / sun["util"] == pytest.approx(1 / pl.IDLER, rel=0.01)
