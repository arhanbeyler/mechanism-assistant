"""E4: crank press system."""

import math

import pytest

import press


def test_work_per_stroke_is_force_times_working_stroke():
    ths, Ts = press.torque_curve(30, 90, 20000, 6)
    assert sum(Ts) * (2 * math.pi / len(ths)) == pytest.approx(20000 * 6, rel=2e-3)


def test_flywheel_lets_a_small_motor_do_it():
    d, checks = press.design(20000, 60, 6, 40)
    assert all(c["util"] <= 1 for c in checks)
    assert d["W"] == pytest.approx(120, rel=2e-3)
    assert d["P_motor"] == pytest.approx(d["W"] * 40 / 60 / press.ETA, rel=1e-3)     # the mean power
    assert d["P_no_flywheel"] > 10 * d["P_motor"]
    assert d["dE"] < d["W"] and d["dE"] > 0.8 * d["W"]                               # a short working stroke


def test_flywheel_on_the_fast_shaft_is_much_smaller():
    d, _ = press.design(20000, 60, 6, 40)
    w_fly, w_crank = 2 * math.pi * d["flywheel_rpm"] / 60, 2 * math.pi * 40 / 60
    I_crank = d["dE"] / (d["Cs"] * w_crank ** 2)
    assert d["flywheel"]["I"] == pytest.approx(I_crank * (w_crank / w_fly) ** 2)


def test_every_module_has_its_own_extractor():
    # a second PRESS = ... once silently replaced the press-fit extractor for both kinds
    import llm_extract
    ex = llm_extract.MODULE_EXTRACTORS
    assert len({id(v) for v in ex.values()}) == len(ex)
    assert "crank press" in ex["press"][1] and "press fit" in ex["press_fit"][1]
