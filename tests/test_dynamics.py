"""K2: four-bar dynamic force analysis (matrix method) cross-checked."""

import math

import pytest

import dynamics
import kinematics as kin
import strength

SEC = {"Crank": {"width": 12, "thickness": 5}, "Coupler": {"width": 10, "thickness": 5},
       "Rocker": {"width": 12, "thickness": 5}}


def _mech():
    mech = kin.synthesize_crank_rocker(30, 60)
    return mech, kin.simulate(mech, 360)


def test_matrix_torque_equals_the_energy_method():
    mech, frames = _mech()
    d = dynamics.analyse(mech, frames, SEC, 2700, 900, 0.0)          # inertia only: no sign flips of the load
    assert max(abs(a - b) for a, b in zip(d["T"], d["T_energy"])) < 1e-6 * 1 + 1e-3 * max(map(abs, d["T"]))


def test_slow_crank_gives_the_static_torque():
    mech, frames = _mech()
    d = dynamics.analyse(mech, frames, SEC, 2700, 0.5, 50.0)
    static = strength.analyze_forces(mech, frames, 50.0)["M_crank_max"] / 1000
    assert abs(d["T_max"]) == pytest.approx(static, rel=2e-3)


def test_mean_torque_is_the_load_work_per_turn():
    mech, frames = _mech()
    d = dynamics.analyse(mech, frames, SEC, 2700, 300, 50.0)
    work = 50.0 * mech["c"] / 1000 * 2 * math.radians(60)            # the rocker swings 60 deg out and back
    assert d["T_mean"] == pytest.approx(work / (2 * math.pi), rel=5e-3)


def test_inertia_torque_scales_with_speed_squared_and_averages_zero():
    mech, frames = _mech()
    d1 = dynamics.analyse(mech, frames, SEC, 7850, 600, 0.0)
    d2 = dynamics.analyse(mech, frames, SEC, 7850, 1200, 0.0)
    assert abs(d2["T_max"]) == pytest.approx(4 * abs(d1["T_max"]), rel=1e-6)
    assert abs(d1["T_mean"]) < 1e-3 * abs(d1["T_max"])
