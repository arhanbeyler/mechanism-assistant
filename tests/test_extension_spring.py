"""H4c: extension spring with hooks (Shigley 10-10)."""

import math

import pytest

import extension_spring as es


def test_force_is_initial_tension_plus_rate_times_extension():
    s, checks = es.design(50, 20)
    assert s and all(c["util"] <= 1 for c in checks)
    assert s["F"] == pytest.approx(s["Fi"] + s["k"] * 20)
    assert s["F"] == pytest.approx(50, rel=0.05)                      # whole quarter body turns
    lo, hi = es.tau_i_range(s["C"])
    assert lo <= 8 * s["Fi"] * s["D"] / (math.pi * s["d"] ** 3) <= hi


def test_hooks_are_more_stressed_than_the_body():
    st = es.stresses(1.5, 12.0, 50)
    assert st["sigma_A"] > st["tau_body"] and st["tau_B"] > st["tau_body"] * 0.9


def test_active_coils_include_the_hooks():
    s, _ = es.design(50, 20)
    assert s["Na"] == pytest.approx(s["Nb"] + 81000 / es.E_WIRE)
