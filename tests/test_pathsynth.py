"""F2b: path synthesis (a crank-rocker's coupler point through given points)."""

import math

import pytest

import pathsynth as ps


def test_nelder_mead_finds_a_quadratic_minimum():
    f = lambda x: (x[0] - 3) ** 2 + 10 * (x[1] + 1) ** 2
    x, v = ps.nelder_mead(f, [0.0, 0.0], [1.0, 1.0], iters=400)
    assert x == pytest.approx([3, -1], abs=1e-4) and v < 1e-8


def test_distance_to_a_closed_polyline():
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert ps.distance_to_curve((5, -2), square)[0] == pytest.approx(2)
    assert ps.distance_to_curve((5, 5), square)[0] == pytest.approx(5)
    assert ps.distance_to_curve((-3, 14), square)[0] == pytest.approx(5)     # nearest a corner


def test_three_points_are_hit_by_a_grashof_crank_rocker():
    T = [(0, 0), (40, 10), (80, 0)]
    x, v, scale = ps.synthesize(T, starts=6)
    r = ps.report(x, T)
    assert r["max_dev"] < 0.005 * scale and r["grashof"]
    assert ps.MU_MIN - 1 <= r["mu_min"] and r["mu_max"] <= 180 - ps.MU_MIN + 1
    # the coupler point really is on the coupler's line
    f = r["frames"][0]
    ab = (f["B"][0] - f["A"][0], f["B"][1] - f["A"][1])
    ap = (f["P"][0] - f["A"][0], f["P"][1] - f["A"][1])
    assert abs(ab[0] * ap[1] - ab[1] * ap[0]) < 1e-6 * r["b"] * max(1.0, abs(r["p"]))
