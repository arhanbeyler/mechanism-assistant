"""E2: multi-load shaft and the two-stage reducer."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import reducer2  # noqa: E402
import shaft_multi as sm  # noqa: E402
import shafts  # noqa: E402


# ---------------------------------------------------------------- multi-load shaft

def test_reactions_and_moment_by_statics():
    loads = [(60.0, 800.0, -300.0), (140.0, -500.0, -900.0)]
    (R1y, R1z), (R2y, R2z) = sm.reactions(loads, 0.0, 200.0)
    assert R1y + R2y + 300.0 == pytest.approx(0) and R1z + R2z - 1200.0 == pytest.approx(0)   # sum F = 0
    assert R2y * 200 + 800 * 60 - 500 * 140 == pytest.approx(0)                               # sum M about xb1 = 0
    My, _ = sm.moment(loads, 0.0, 200.0, 100.0)
    assert My == pytest.approx(R1y * 100 + 800 * 40)
    assert sm.moment(loads, 0.0, 200.0, 200.0) == (pytest.approx(0, abs=1e-9), pytest.approx(0, abs=1e-9))


def test_deflection_matches_closed_form():
    EI = shafts.E_STEEL * math.pi * 30 ** 4 / 64
    xs, ((thy, yy), _) = sm._deflect(lambda x: 15.0, [(100.0, 1000.0, 0.0)], 0.0, 200.0)
    assert max(abs(v) for v in yy) == pytest.approx(1000 * 200 ** 3 / (48 * EI), rel=1e-4)
    assert abs(thy[0]) == pytest.approx(1000 * 200 ** 2 / (16 * EI), rel=1e-4)


def test_two_planes_add_as_vectors():
    """The same load split into y and z must deflect the shaft exactly as much as the load in one plane."""
    one = sm._deflect(lambda x: 12.0, [(80.0, 500.0, 0.0)], 0.0, 200.0)[1][0][1]
    ys, zs = sm._deflect(lambda x: 12.0, [(80.0, 300.0, 400.0)], 0.0, 200.0)[1]
    assert max(math.hypot(a, b) for a, b in zip(ys[1], zs[1])) == pytest.approx(max(abs(v) for v in one), rel=1e-9)


def test_rayleigh_with_weights_is_near_exact():
    EI = shafts.E_STEEL * 1e3 * math.pi * 30 ** 4 / 64
    m = shafts.RHO_STEEL * math.pi * 15 ** 2
    exact = (math.pi / 300) ** 2 * math.sqrt(EI / m) * 60 / (2 * math.pi)
    n = sm.critical_speed({}, [], 0.0, 300.0, lambda x: 15.0)
    assert exact <= n <= 1.005 * exact


def test_torque_free_intermediate_seats_have_no_key():
    seats = [{"x": -20.0, "Fy": 500.0, "Fz": 800.0, "width": 30, "mass": 1.0, "T": 0.0}]
    r, _ = sm.design(seats, (0.0, 0.0), -60.0, 60.0, rpm=500)
    assert r is not None and r["keys"] == [None]


# ---------------------------------------------------------------- two-stage reducer

CASES = [(10000.0, 1450.0, 12.0), (20000.0, 1450.0, 16.0), (5000.0, 1450.0, 25.0)]


@pytest.fixture(scope="module", params=CASES, ids=lambda c: f"{c[0] / 1000:g}Nm-i{c[2]:g}")
def result(request):
    r, checks = reducer2.design(*request.param)
    assert r is not None, [c for c in checks if c["util"] > 1]
    return r, checks, request.param


def test_all_checks_pass_and_ratio(result):
    r, checks, (_, _, i) = result
    assert all(c["util"] <= 1.0 for c in checks)
    assert abs(r["ratio"] - i) / i <= 0.03


def test_torque_and_speed_chain(result):
    r, _, (T1, rpm, _) = result
    g1, g2 = r["stages"]
    assert r["T"][2] == pytest.approx(T1 * g1["ratio"] * g2["ratio"])
    assert r["rpm"][2] == pytest.approx(rpm / (g1["ratio"] * g2["ratio"]))


def test_intermediate_shaft_loads(result):
    """Tangential forces of gear 2 and pinion 3 point the same way, radial ones oppose (collinear layout)."""
    r, _, _ = result
    seats = r["shafts"][1]["seats"]
    assert seats[0]["Fz"] > 0 and seats[1]["Fz"] > 0
    assert seats[0]["Fy"] > 0 > seats[1]["Fy"]


def test_geometry_fits(result):
    r, _, _ = result
    (g1, g2), s = r["stages"], r["shafts"]
    assert (g1["bore1"], g1["bore2"], g2["bore1"], g2["bore2"]) == (s[0]["d"], s[1]["d"], s[1]["d"], s[2]["d"])
    ra2 = g1["d2"] / 2 + g1["module"]
    assert g2["center_distance"] >= ra2 + s[2]["d"] / 2 + reducer2.CLEAR - 1e-9      # gear 2 clears the output shaft
    for k, b in ((s[0]["keys"][0], g1["face_width"]), (s[2]["keys"][0], g2["face_width"])):
        assert k["length"] <= b
    x1, x2 = r["x_stages"]
    assert x2 - x1 >= (g1["face_width"] + g2["face_width"]) / 2                      # stages do not overlap
