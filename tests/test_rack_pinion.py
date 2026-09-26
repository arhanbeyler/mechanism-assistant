"""G4: rack and pinion."""

import math

import pytest

import gears
import pipeline
import rack_pinion as rp
import strength


def test_contact_ratio_with_a_rack():
    # z = 18, m = 2: sqrt(20^2 - 16.914^2) = 10.672, r sin(phi) = 6.156, m / sin(phi) = 5.848
    assert rp.contact_ratio_rack(2, 18) == pytest.approx((10.672 - 6.156 + 5.848) / (math.pi * 2 * math.cos(gears.PHI)), rel=1e-3)
    # a rack meshes longer than any finite gear of the same pinion
    assert rp.contact_ratio_rack(2, 18) > gears.contact_ratio(2, 18, 400)


def test_design_passes_and_uses_torque():
    d, checks = rp.design(2000, 500, strength.MATERIALS["steel"], 60)
    assert all(c["util"] <= 1 for c in checks)
    assert d["z"] >= 18 and d["T"] == pytest.approx(2000 * d["d"] / 2)
    assert d["per_turn"] == pytest.approx(math.pi * d["d"])


def test_rack_outline_pitch_and_space_at_centre():
    m, d = 2.0, 36.0
    pts = rp.rack_outline(m, d, 100, 5)
    tips = sorted({round(x, 6) for x, y in pts if abs(y - (-d / 2 + m)) < 1e-9})
    widths = [b - a for a, b in zip(tips, tips[1:])]
    assert min(widths) == pytest.approx(math.pi * m / 2 - 2 * m * math.tan(gears.PHI))          # tooth tip land
    assert all(abs(x) > 0.5 for x in tips)                                                     # a space at x = 0


def test_pipeline_rack_pinion():
    r = pipeline.design_from_params(kind="rack_pinion", force_N=500, travel_mm=150, material="aluminum")
    assert r["status"] == "ok", r["message"]
    assert pipeline.design_from_spec({"mechanism_type": "rack_pinion"})["status"] == "missing"
