"""F4b: Watt II six-bar for a large swing."""

import pytest

import pipeline
import sixbar


def test_six_bar_reaches_what_a_four_bar_cannot():
    m, frames = sixbar.synthesize(30, 150)
    assert m and m["ok"]
    me = sixbar.measure(frames)
    assert me["swing"] == pytest.approx(150, abs=1.0)
    assert min(me["mu1_min"], me["mu2_min"]) >= sixbar.MU_MIN and max(me["mu1_max"], me["mu2_max"]) <= sixbar.MU_MAX
    # the ternary link: the arm keeps its angle gamma to the rocker at every step
    for f in frames[::30]:
        import math
        rho = math.atan2(f["B"][1], f["B"][0])
        arm = math.atan2(f["A2"][1], f["A2"][0])
        assert (arm - rho - m["gamma"] + math.pi) % (2 * math.pi) - math.pi == pytest.approx(0, abs=1e-9)


def test_pipeline_switches_to_the_six_bar_for_a_large_swing():
    r = pipeline.design_from_params(crank_length_mm=30, output_swing_deg=180)
    assert r["status"] == "ok" and r["mechanism_type"] == "six_bar"
    assert r["sixbar"]["swing"] == pytest.approx(180, abs=1.0)
    small = pipeline.design_from_params(crank_length_mm=30, output_swing_deg=60)
    assert small["mechanism_type"] == "crank_rocker"


def test_every_link_is_buildable():
    m, _ = sixbar.synthesize(30, 180)
    lens = [m["m1"][k] for k in "abcd"] + [m["a2"], m["b2"], m["c2"], m["d2"]]
    assert min(lens) >= sixbar.LINK_MIN and max(lens) / min(lens) <= sixbar.RATIO_MAX
