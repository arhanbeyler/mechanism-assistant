"""H3c: involute spline (DIN 5480)."""

import math

import pytest

import din5480
import pipeline


@pytest.mark.parametrize("dB, m, z, x", [(25, 1.25, 18, 0.45), (30, 2, 14, -0.05), (35, 2, 16, 0.2), (40, 2, 18, 0.45),
                                         (50, 2, 24, -0.05), (60, 2, 28, 0.45), (100, 3, 32, 0.117)])
def test_sizes_of_the_standard(dB, m, z, x):
    g = din5480.geometry(dB, m)
    assert g["z"] == z and g["x"] == pytest.approx(x, abs=1e-3)
    assert -0.05 - 1e-9 <= g["x"] <= 0.45 + 1e-9


def test_geometry_rules():
    g = din5480.geometry(30, 2)
    assert (g["da1"], g["df1"], g["da2"], g["df2"]) == (29.6, 25.6, 26.0, 30.4)
    assert (g["da1"] - g["da2"]) / 2 == pytest.approx(0.9 * 2)              # working depth 0.9 m
    # tooth thickness on d = m z, then the involute gives it back at the reference radius
    assert din5480.thickness_at(g, g["d"] / 2) == pytest.approx(math.pi + 2 * -0.05 * 2 * math.tan(math.radians(30)))
    assert din5480.thickness_at(g, g["da1"] / 2) < g["s"]                  # thinner towards the tip
    assert g["db"] < g["da2"]                                               # flanks all above the base circle


def test_design_is_the_smallest_that_carries_it():
    s, checks = din5480.design(400e3)
    assert s and all(c["util"] <= 1 for c in checks) and s["cap"] >= s["Td"]
    i = din5480.REF_DIAMETERS.index(s["dB"])
    for dB in din5480.REF_DIAMETERS[:i]:                                    # every smaller one fails
        g = din5480.geometry(dB, din5480.module_for(dB))
        assert din5480.capacity(g, 1.5 * dB) < s["Td"] or 16 * s["Td"] / (math.pi * g["df1"] ** 3) > din5480.SY / 4


def test_pipeline_involute_and_default():
    r = pipeline.design_from_spec({"mechanism_type": "spline", "torque_Nm": 400, "spline_profile": "involute"})
    assert r["status"] == "ok" and r["fusion"]["profile"] == "involute" and "DIN 5480 W 35" in r["message"]
    r = pipeline.design_from_spec({"mechanism_type": "spline", "torque_Nm": 400})
    assert r["status"] == "ok" and r["fusion"]["profile"] == "straight"
    r = pipeline.design_from_spec({"mechanism_type": "spline", "torque_Nm": 1e6, "spline_profile": "involute"})
    assert r["status"] == "failed"
