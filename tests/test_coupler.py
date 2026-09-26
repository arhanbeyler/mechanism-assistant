"""F2: coupler curves and the Hoeckens straight-line linkage."""

import math

import pytest

import coupler
import pipeline
import strength


def test_curve_keeps_link_lengths():
    m = dict(coupler.HOECKENS, gamma=30.0)            # an off-line coupler point too
    for f in coupler.curve(m, n=180):
        assert math.dist(f["A"], (0, 0)) == pytest.approx(m["a"])
        assert math.dist(f["A"], f["B"]) == pytest.approx(m["b"])
        assert math.dist(f["B"], (m["d"], 0)) == pytest.approx(m["c"])
        assert math.dist(f["A"], f["P"]) == pytest.approx(m["p"])
        ab = math.atan2(f["B"][1] - f["A"][1], f["B"][0] - f["A"][0])
        ap = math.atan2(f["P"][1] - f["A"][1], f["P"][0] - f["A"][0])
        assert math.remainder(ap - ab, 2 * math.pi) == pytest.approx(math.radians(30))


def test_non_grashof_crank_locks():
    assert coupler.curve({"a": 3, "b": 1, "c": 1, "d": 1, "p": 1}) is None


def test_line_fit_exact_line():
    pts = [(t, 2 * t + 1) for t in range(10)]
    _, u, dev = coupler.line_fit(pts)
    assert dev == pytest.approx(0, abs=1e-9)
    assert abs(u[1] / u[0]) == pytest.approx(2)


def test_hoeckens_straight_part():
    """Known behaviour: about 4.4 crank lengths of straight path over more than half a turn."""
    seg = coupler.straight_segment(coupler.curve(coupler.HOECKENS))
    assert 4.3 < seg["length"] < 4.6
    assert seg["fraction"] > 0.55
    assert seg["deviation"] <= coupler.STRAIGHT_TOL * seg["length"] + 1e-12
    assert abs(seg["direction"][1]) < 1e-3              # parallel to the ground line
    assert seg["centroid"][1] == pytest.approx(4.0, abs=0.01)


def test_design_scales_to_the_stroke():
    d, checks = coupler.design_straight(100, strength.MATERIALS["PLA"])
    assert 100 <= d["stroke"] < 100.5
    assert d["mech"]["a"] == pytest.approx(round(d["mech"]["a"], 1))
    m = d["mech"]
    assert (m["b"], m["c"], m["d"], m["p"]) == pytest.approx((2.5 * m["a"], 2.5 * m["a"], 2 * m["a"], 5 * m["a"]))
    assert all(c["util"] <= 1 for c in checks if c["group"] == "Motion")


def test_torque_statics_equals_virtual_work():
    """Crank torque from the coupler statics (rocker = two-force member) = F |dP/dtheta| at every sample."""
    d, _ = coupler.design_straight(80, strength.MATERIALS["aluminum"], load_N=50)
    speed = coupler.point_speed(d["frames"])
    for i in d["idx"][::10]:
        vx, vy = speed[i]
        v = math.hypot(vx, vy)
        T, *_ = coupler.statics(d["frames"][i], d["mech"], (-50 * vx / v, -50 * vy / v))
        assert abs(T) == pytest.approx(50 * v, rel=2e-3)


def test_coupler_bending_only_with_a_point_load():
    four_bar = {"F": 10, "F_ab_max": 20, "M_crank_max": 500, "M_rocker": 400}
    names = [c["name"] for c in strength.check_link("Coupler", four_bar, 60, strength.MATERIALS["PLA"], 10, 4, 4)]
    assert "Coupler axial stretch" in names                  # four-bar unchanged
    loaded = dict(four_bar, M_coupler=1000, coupler_span=50, coupler_overhang=50)
    c = strength.check_link("Coupler", loaded, 50, strength.MATERIALS["PLA"], 10, 4, 4)
    stiff = next(x for x in c if x["group"] == "Stiffness")
    mat = strength.MATERIALS["PLA"]
    E, I = mat["E"], 4 * 10 ** 3 / 12
    G = E / (2 * (1 + mat["nu"]))
    # overhanging beam: A-B simply supported (50), B-P cantilever (50), P = M / overhang = 20 N at P
    delta = 20 * 50 ** 2 * (50 + 50) / (3 * E * I) + 20 * 50 / (5 / 6 * G * 40)
    assert stiff["util"] == pytest.approx(delta / (100 / strength.DEFLECTION_RATIO), rel=1e-9)


def test_pipeline_straight_line():
    r = pipeline.design_from_params(kind="straight_line", stroke_mm=120, max_load_N=10, speed_rpm=20, material="PETG")
    assert r["status"] == "ok", r["message"]
    assert r["straight"]["stroke"] >= 120
    f = r["fusion"]
    assert f["type"] == "straight_line" and f["layer_order"][0] == "Ground"
    P, A = f["points_mm"]["P"], f["points_mm"]["A"]
    assert math.dist(P, A) == pytest.approx(f["lengths_mm"]["p"], abs=0.01)


def test_pipeline_straight_line_missing_and_too_heavy():
    assert pipeline.design_from_spec({"mechanism_type": "straight_line"})["status"] == "missing"
    r = pipeline.design_from_params(kind="straight_line", stroke_mm=200, max_load_N=300, material="PLA")
    assert r["status"] == "failed" and "No PLA" in r["message"]
