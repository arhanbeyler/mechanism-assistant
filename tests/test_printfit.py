"""B8c preparation: print-in-place clearance coupon."""

import pytest

import printfit


@pytest.mark.parametrize("c", printfit.CLEARANCES)
def test_gap_is_the_clearance_everywhere(c):
    assert printfit.min_gap(c) == pytest.approx(c, abs=1e-6)
    assert printfit.captured(c)


def test_coupon_spinners_do_not_touch():
    cp = printfit.coupon()
    xs = [s["x"] for s in cp["spinners"]]
    r_out = max(p[0] for s in cp["spinners"] for p in s["ring"])
    assert all(b - a > 2 * r_out + 4 for a, b in zip(xs, xs[1:]))
    assert [s["c"] for s in cp["spinners"]] == printfit.CLEARANCES
