"""E3: cam + follower spring. The follower must never leave the cam, and the spring must be a valid spring."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cam_system as cs  # noqa: E402
import cams  # noqa: E402
import pipeline  # noqa: E402

H, RISE, DWELL, RET = 20.0, 120.0, 60.0, 120.0


@pytest.fixture(scope="module")
def cam():
    d, _ = cams.design(H, RISE, DWELL, RET, "cycloidal")
    return d, cams.svaj(H, RISE, DWELL, RET, "cycloidal", n=720)


def test_peak_acceleration_is_norton_cycloidal(cam):
    """Cycloidal rise: a_max = 2 pi h / beta^2 per rad^2, times w^2 at speed."""
    _, table = cam
    acc = cs.follower_accel(table, 600)
    w = 600 * 2 * math.pi / 60
    beta = math.radians(RISE)
    assert max(abs(x) for _, _, x in acc) == pytest.approx(2 * math.pi * H / beta ** 2 * w * w / 1000, rel=1e-3)


@pytest.mark.parametrize("rpm, mass, load", [(300, 0.2, 0.0), (600, 0.2, 0.0), (1200, 0.5, 10.0)])
def test_follower_never_leaves_the_cam(cam, rpm, mass, load):
    d, table = cam
    r, checks = cs.design(d, table, rpm, mass, load)
    assert r is not None and all(c["util"] <= 1.0 for c in checks)
    k, y0 = r["k"], r["preload_mm"]
    w = rpm * 2 * math.pi / 60
    for _, s, _, a in table:                                  # contact force recomputed from the table
        xdd = a * w * w / 1000
        spring = k * (y0 + s)
        assert mass * xdd + spring + load > 0
        assert spring >= cs.N_JUMP * (-mass * xdd) - load - 1e-6
    sp = r["spring"]
    assert sp["k"] == pytest.approx(k) and sp["y_work"] == pytest.approx(y0 + H)


def test_faster_cam_needs_a_stronger_spring(cam):
    d, table = cam
    slow, _ = cs.design(d, table, 300, 0.2)
    fast, _ = cs.design(d, table, 900, 0.2)
    assert fast["F_top"] > slow["F_top"] and fast["k"] > slow["k"]


def test_impossible_speed_is_rejected(cam):
    d, table = cam
    r, checks = cs.design(d, table, 3000, 2.0)
    assert r is None and any(c["util"] > 1.0 for c in checks)


def _cam_params(**kw):
    p = dict(kind="cam", cam_rise_mm=H, rise_deg=RISE, dwell_top_deg=DWELL, return_deg=RET, motion_law="cycloidal")
    p.update(kw)
    return p


def test_pipeline_adds_the_spring_only_with_speed_and_mass():
    r = pipeline.design_from_params(**_cam_params())
    assert r["status"] == "ok" and "cam_spring" not in r
    r = pipeline.design_from_params(**_cam_params(speed_rpm=600, follower_mass_kg=0.2))
    assert r["status"] == "ok" and r["cam_spring"]["contact_min"] > 0
    assert any(c["name"].startswith("Jump") for c in r["checks"])
    r = pipeline.design_from_params(**_cam_params(speed_rpm=600))
    assert "cam_spring" not in r and "both the cam speed and the follower mass" in r["message"]


def test_pipeline_reports_an_impossible_spring():
    r = pipeline.design_from_params(**_cam_params(speed_rpm=3000, follower_mass_kg=2.0))
    assert r["status"] == "failed" and "no spring keeps" in r["message"]
