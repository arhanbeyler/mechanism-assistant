"""H3b: straight-sided spline (ISO 14)."""

import pytest

import spline


def test_capacity_formula():
    # 6 x 23 x 26: h = 1.5 - 0.6 = 0.9, r_m = 12.25, 50 mm, 100 MPa, 75 % carrying
    assert spline.capacity(6, 23, 26, 50) == pytest.approx(0.75 * 6 * 0.9 * 50 * 12.25 * 100)


def test_design_is_the_smallest_that_carries_it():
    s, checks = spline.design(100000)
    assert s and all(c["util"] <= 1 for c in checks)
    assert s["cap"] >= s["Td"] and s["L"] <= 1.5 * s["D"]
    i = next(k for k, x in enumerate(spline.SIZES) if x[1:4] == (s["N"], s["d"], s["D"]))
    for series, N, d, D, B in spline.SIZES[:i]:                   # every smaller spline fails at 1.5 D
        assert spline.capacity(N, d, D, 1.5 * D) < s["Td"] or 16 * s["Td"] / (3.14159 * d ** 3) > spline.SY / 4


def test_sliding_under_load_needs_a_bigger_spline():
    fixed, _ = spline.design(100000, "fixed")
    sliding, _ = spline.design(100000, "sliding_loaded")
    assert sliding is None or sliding["D"] > fixed["D"]
