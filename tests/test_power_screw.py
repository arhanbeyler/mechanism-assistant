"""G5: power screw (Tr)."""

import math

import pytest

import pipeline
import power_screw as ps


def test_torques_match_shigley_example_8_1():
    """Square thread (alpha = 0), d_m = 30 mm, lead 8 mm, F = 6.4 kN, f = f_c = 0.08, d_c = 40 mm."""
    T_R, T_L, T_c = ps.torques({"dm": 30, "l": 8}, 6400, 0.08, 0.08, 40, alpha=0.0)
    assert T_R / 1000 == pytest.approx(15.94, abs=0.01)
    assert T_L / 1000 == pytest.approx(-0.466, abs=0.005)            # not self-locking
    assert T_c / 1000 == pytest.approx(10.24, abs=0.01)


def test_iso_trapezoidal_diameters():
    th = ps.thread(20, 4)
    assert (th["dm"], th["dr"]) == (18.0, 15.0)                      # d - P/2, d - P - 2 a_c (a_c = 0.5)


def test_self_locking_rule():
    """T_L > 0 exactly when pi f d_m sec(alpha) > lead."""
    for d, P in ps.SIZES:
        th = ps.thread(d, P)
        T_L = ps.torques(th, 1000, ps.F_LOW, 0, 0)[1]
        assert (T_L > 0) == (math.pi * ps.F_LOW * th["dm"] / math.cos(ps.ALPHA) > th["l"])


def test_design_is_self_locking_and_passes():
    d, checks = ps.design(5000, "bronze", 2.0, 300)
    assert d["thread"]["d"] == 18 and all(c["util"] <= 1 for c in checks)
    assert d["nut_len"] >= d["thread"]["d"]
    assert d["efficiency"] == pytest.approx(5000 * 4 / (2 * math.pi * (d["T_R"] + d["T_c"])))


def test_pipeline_power_screw():
    r = pipeline.design_from_params(kind="power_screw", force_N=20000, travel_mm=400, screw_ends="jack", speed_rpm=100)
    assert r["status"] == "ok", r["message"]
    assert r["screw"]["C"] == 0.25 and r["fusion"]["designation"] == "TR32x6"
    assert pipeline.design_from_spec({"mechanism_type": "power_screw"})["status"] == "missing"
