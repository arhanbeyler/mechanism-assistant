"""H3: ISO fits and press fits."""

import math

import pytest

import fits


def test_shigley_medium_drive_fit_48mm():
    # H7/s6 at 48 mm: hole 48.000/48.025, shaft 48.043/48.059 -> interference 0.018..0.059 mm
    assert fits.limits(48, "s6") == (0, 25, 43, 59)
    assert fits.fit_range(48, "s6") == (18, 59)


def test_clearance_fits_are_negative():
    lo, hi = fits.fit_range(25, "g6")
    assert lo < 0 and hi < 0 and fits.fit_range(25, "h6")[1] == 0


def test_same_material_solid_shaft_pressure():
    # Shigley 3-57: p = E delta (r_o^2 - R^2) / (2 R r_o^2), delta radial
    E, nu, R, ro, delta = 207000.0, 0.3, 20.0, 40.0, 0.02
    p = fits.pressure(delta, R, ro, E, nu, E, nu)
    assert p == pytest.approx(E * delta * (ro ** 2 - R ** 2) / (2 * R * ro ** 2))


def test_torque_capacity_and_hub_stress():
    a = fits.analyse(40, "s6", 80, 40)
    assert a["T_min"] == pytest.approx(fits.MU * a["p_min"] * 2 * math.pi * 20 * 40 * 20)
    assert a["hub_st_max"] == pytest.approx(a["p_max"] * (40 ** 2 + 20 ** 2) / (40 ** 2 - 20 ** 2))


def test_design_passes_or_explains():
    d, checks = fits.design(200000, 40)
    assert d and d["fit"] == "s6" and all(c["util"] <= 1 for c in checks)
    small, why = fits.design(50000, 25)       # no standard fit: min interference too small or hub overstressed
    assert small is None and any(c["util"] > 1 for c in why)
