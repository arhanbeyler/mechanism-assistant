"""F4: Scotch yoke and crank + slotted lever quick return."""

import math

import pytest

import pipeline
import slotted
import strength


def test_yoke_is_harmonic_and_torque_is_virtual_work():
    d, checks = slotted.yoke_design(80, strength.MATERIALS["PLA"], 100)
    for f in d["frames"][::30]:
        th = math.radians(f["theta"])
        assert f["x"] == pytest.approx(40 * math.cos(th), abs=1e-9)
        assert 100 * abs(f["dx"]) <= d["T_max"] + 1e-9                 # T = F |dx/dtheta| <= F r
    assert d["T_max"] == pytest.approx(100 * 40)
    assert all(c["util"] <= 1 for c in checks)


def test_slot_bar_formula():
    mat = strength.MATERIALS["steel"]
    c = slotted.slot_bar(10000, 500, 30, 6, 5, mat, 100)
    I = 5 * (30 ** 3 - 6 ** 3) / 12
    assert c[0]["util"] == pytest.approx(10000 * 15 / I / (mat["sy"] / mat["n"]))


@pytest.mark.parametrize("Q", [1.3, 1.6, 2.0])
def test_lever_stroke_and_time_ratio(Q):
    d, _ = slotted.lever_design(200, Q, strength.MATERIALS["steel"])
    m = d["measure"]
    assert m["stroke"] == pytest.approx(200, abs=0.01)
    assert m["Q"] == pytest.approx(Q, abs=0.02)                        # 0.5 deg sampling
    assert d["d"] + d["r"] <= (1 - slotted.LEVER_MARGIN) * d["L"] + 1e-9   # the pin stays on the lever
    # extreme ram positions: crank square to the lever (O2P . OP ~ 0)
    f = d["frames"][m["i_max"]]
    P = f["P"]
    cos_ang = (P[0] * P[0] + (P[1] - d["d"]) * P[1]) / (d["r"] * math.hypot(*P))
    assert abs(cos_ang) < 0.01


def test_lever_statics_equals_virtual_work():
    d, _ = slotted.lever_design(150, 1.5, strength.MATERIALS["steel"], 400)
    for f in d["frames"][::45]:
        _, _, T = slotted.lever_statics(d, f, 400)
        assert T == pytest.approx(400 * abs(f["dx"]), rel=1e-9, abs=1e-6)


def test_pipeline_slotted():
    r = pipeline.design_from_params(kind="scotch_yoke", stroke_mm=60, max_load_N=200, speed_rpm=600, material="PETG")
    assert r["status"] == "ok", r["message"]
    assert r["yoke"]["motion"]["a_max"] < r["yoke"]["motion"]["a_max_slider_crank"]
    r = pipeline.design_from_params(kind="quick_return", stroke_mm=150, time_ratio=1.7, max_load_N=300,
                                    material="aluminum")
    assert r["status"] == "ok", r["message"]
    assert r["lever"]["Q_meas"] == pytest.approx(1.7, abs=0.02)
    assert pipeline.design_from_spec({"mechanism_type": "quick_return"})["status"] == "missing"
    assert pipeline.design_from_spec({"mechanism_type": "scotch_yoke", "crank_length_mm": 25})["yoke"]["stroke"] == 50
