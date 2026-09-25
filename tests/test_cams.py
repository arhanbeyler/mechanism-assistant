"""C11a: kam modulu testleri."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cams  # noqa: E402


@pytest.mark.parametrize("motion", ["cycloidal", "harmonic", "poly345"])
def test_laws_boundary_values(motion):
    """Yukselmenin basinda s=0, sonunda s=h; sikloidal ve 3-4-5'te uclarda v=0."""
    h, beta = 20.0, math.radians(120)
    s0, v0, _ = cams.law(motion, 0.0, h, beta)
    s1, v1, _ = cams.law(motion, 1.0, h, beta)
    assert s0 == pytest.approx(0.0, abs=1e-12) and s1 == pytest.approx(h)
    if motion != "harmonic":
        assert v0 == pytest.approx(0.0, abs=1e-9) and v1 == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("motion", ["cycloidal", "harmonic", "poly345"])
def test_velocity_is_derivative_of_displacement(motion):
    h, beta, x, dx = 20.0, math.radians(100), 0.37, 1e-6
    s_a, v, a = cams.law(motion, x, h, beta)
    s_b, v_b, _ = cams.law(motion, x + dx, h, beta)
    assert (s_b - s_a) / (dx * beta) == pytest.approx(v, rel=1e-4)
    assert (v_b - v) / (dx * beta) == pytest.approx(a, rel=1e-3)


def test_cycloidal_peak_acceleration():
    """Sikloidal: a_max = 2 pi h / beta^2 (Norton)."""
    h, beta = 20.0, math.radians(120)
    amax = max(abs(cams.law("cycloidal", i / 1000, h, beta)[2]) for i in range(1001))
    assert amax == pytest.approx(2 * math.pi * h / beta ** 2, rel=1e-4)


def test_design_is_smallest_passing_base_circle():
    d, checks = cams.design(20, 120, 60, 120)
    assert d is not None and all(c["util"] <= 1.0 for c in checks)
    table = cams.svaj(20, 120, 60, 120)
    smaller = cams.check(d["Rb"] - 0.5, d["Rf"], table)
    assert max(c["util"] for c in smaller) > 1.0


def test_steeper_motion_needs_bigger_cam():
    fast, _ = cams.design(20, 60, 60, 60)
    slow, _ = cams.design(20, 150, 30, 150)
    assert fast["Rb"] > slow["Rb"]


def test_profile_is_offset_of_pitch_curve():
    """Kam yuzeyi ile hatve egrisi arasindaki en kisa mesafe her yerde makara yaricapi."""
    d, _ = cams.design(20, 120, 60, 120)
    pts = cams.profile(d, n=720)
    Rp = d["Rb"] + d["Rf"]
    pitch = []
    for th_deg, s, _, _ in cams.svaj(20, 120, 60, 120, n=720):
        ang = math.pi / 2 - math.radians(th_deg)
        pitch.append(((Rp + s) * math.cos(ang), (Rp + s) * math.sin(ang)))
    for i in range(0, 720, 37):
        dmin = min(math.dist(pts[i], p) for p in pitch)
        assert dmin == pytest.approx(d["Rf"], rel=0.02)
