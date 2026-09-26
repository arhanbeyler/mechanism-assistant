"""H5a: flywheel (Shigley 16-12)."""

import math

import pytest

import flywheel as f


def test_energy_between_the_speed_limits_is_the_fluctuation():
    info, _ = f.analyse(500, 300, 0.1)
    w_max, w_min = (2 * math.pi * info[k] / 60 for k in ("rpm_max", "rpm_min"))
    assert 0.5 * info["I"] * (w_max ** 2 - w_min ** 2) == pytest.approx(500)


def test_disk_has_the_inertia():
    for D_max in (None, 400):
        info, _ = f.analyse(2000, 120, 0.1, D_max)
        r, t = info["D"] / 2000, info["t"] / 1000
        assert math.pi * f.RHO * t * r ** 4 / 2 == pytest.approx(info["I"])
        assert 0.5 * info["mass"] * r * r == pytest.approx(info["I"])


def test_press_energy_and_proportion_check():
    assert f.press_energy(1000, 0.1) == pytest.approx(900)
    _, checks = f.analyse(2000, 120, 0.1, 400)          # a 400 mm limit makes a 6 m thick "disk": flagged
    assert any(c["util"] > 1 for c in checks)
