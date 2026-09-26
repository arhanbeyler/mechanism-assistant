"""H5b: rigid flange coupling."""

import math

import pytest

import bolted
import coupling


def test_friction_capacity_and_proportions():
    c, checks = coupling.design(100000)
    assert c and all(x["util"] <= 1 for x in checks)
    assert (c["D_hub"], c["L_hub"], c["D_bolt"], c["D_flange"], c["t"]) == tuple(k * c["d"] for k in (2, 1.5, 3, 4, 0.5))
    b = c["bolt"]
    assert b["T_fric"] == pytest.approx(coupling.MU * c["N"] * b["Fi"] * c["D_bolt"] / 2)
    assert b["T_fric"] >= c["Td"]
    smaller = [s for s in bolted.SIZES if s[0] < b["d"]]
    for s in smaller:                                           # the next smaller bolt would slip
        Fi = bolted.PRELOAD * s[2] * (580.0 if s[0] < 16 else 600.0)
        assert coupling.MU * c["N"] * Fi * c["D_bolt"] / 2 < c["Td"]


def test_hub_torsion_formula():
    c, _ = coupling.design(100000)
    D, d = c["D_hub"], c["d"]
    assert c["tau_hub"] == pytest.approx(16 * c["Td"] * D / (math.pi * (D ** 4 - d ** 4)))


def test_too_much_torque_for_the_shaft_is_reported():
    c, checks = coupling.design(500000, 40)
    assert c is None and any(x["util"] > 1 for x in checks)
