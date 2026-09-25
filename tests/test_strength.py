"""Mukavemet modulu testleri."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import kinematics as kin  # noqa: E402
import pipeline  # noqa: E402
import strength  # noqa: E402

PLA = strength.MATERIALS["PLA"]


@pytest.fixture(scope="module")
def mech30():
    m = kin.synthesize_crank_rocker(30, 60)
    return m, kin.simulate(m), {k: m[k] for k in "abcd"}


def test_coupler_force_is_load_over_sin_mu(mech30):
    """Biyel iki kuvvet elemani: F_AB = F / sin(mu); en buyugu sin(mu) en kucukken."""
    m, frames, _ = mech30
    f = strength.analyze_forces(m, frames, 100.0)
    min_sin = min(math.sin(math.radians(fr["mu"])) for fr in frames)
    assert f["F_ab_max"] == pytest.approx(100.0 / min_sin)
    assert f["M_rocker"] == pytest.approx(100.0 * m["c"])


def test_input_torque_bounded_by_crank_length(mech30):
    """Krank momenti en fazla F_AB * a olabilir (moment kolu <= krank boyu)."""
    m, frames, _ = mech30
    f = strength.analyze_forces(m, frames, 100.0)
    assert 0 < f["M_crank_max"] <= f["F_ab_max"] * m["a"] + 1e-9


def test_check_section_hand_calculation():
    """Elle hesap: w=20, t=4, d=4, F_AB=100 N, PLA (izin verilen 40/3 MPa)."""
    forces = {"F_ab_max": 100.0, "M_crank_max": 1000.0, "M_rocker": 2000.0}
    lengths = {"a": 30.0, "b": 50.0, "c": 60.0, "d": 70.0}
    c = {x["name"]: x for x in strength.check_section(forces, lengths, PLA, 20, 4, 4)}
    allow = 40.0 / 3.0
    assert c["Biyel net kesit"]["util"] == pytest.approx((100 / 64) / allow)
    tau = 4 * 100 / (math.pi * 16)
    assert c["Pim kayması"]["util"] == pytest.approx(tau / (0.5 * allow))
    assert c["Pim ezilmesi"]["util"] == pytest.approx((100 / 16) / allow)
    S_net = 4 * (20 ** 3 - 4 ** 3) / (6 * 20)
    assert c["Kol eğilme + eksenel"]["util"] == pytest.approx((2000 / S_net + 100 / 64) / allow)
    I_min = 20 * 4 ** 3 / 12
    P_cr = math.pi ** 2 * 2500 * I_min / 70 ** 2
    assert c["Sabit burkulması"]["util"] == pytest.approx(100 * 3 / P_cr)


def test_sized_section_passes_and_is_minimal(mech30):
    m, frames, lengths = mech30
    forces = strength.analyze_forces(m, frames, 50.0)
    sec, checks = strength.size_section(forces, lengths, PLA)
    assert sec is not None
    assert all(c["util"] <= 1.0 for c in checks)
    # 1 mm daha dar ayni kesit en az bir kontrolden kalmali (ya da kenar mesafesi siniri)
    w, t, d = sec["width"], sec["thickness"], sec["hole"]
    if w - 1 >= max(math.ceil(2.5 * d), 8):
        narrower = strength.check_section(forces, lengths, PLA, w - 1, t, d)
        assert max(c["util"] for c in narrower) > 1.0


def test_steel_needs_smaller_section_than_pla(mech30):
    m, frames, lengths = mech30
    forces = strength.analyze_forces(m, frames, 200.0)
    pla, _ = strength.size_section(forces, lengths, PLA)
    steel, _ = strength.size_section(forces, lengths, strength.MATERIALS["steel"])
    assert steel["width"] * steel["thickness"] < pla["width"] * pla["thickness"]


def test_impossible_load_returns_none(mech30):
    m, frames, lengths = mech30
    forces = strength.analyze_forces(m, frames, 1e6)
    sec, checks = strength.size_section(forces, lengths, PLA)
    assert sec is None and checks


# ---------------------------------------------------------------- akis (sahte LLM)

def _spec(load=None, mat=None):
    return {"mechanism_type": "crank_rocker", "crank_length_mm": 30, "output_swing_deg": 60,
            "quick_return": False, "time_ratio": None, "max_load_N": load, "material": mat}


def test_pipeline_sizes_section_when_load_given(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec(50, "PETG"))
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok"
    assert r["fusion"]["section_mm"] != strength.DEFAULT_SECTION
    assert any(c["name"].startswith("Burkulma") for c in r["checks"])
    assert r["strength"]["material"].startswith("PETG")


def test_pipeline_does_not_invent_load(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec())
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok"
    assert "strength" not in r
    assert r["fusion"]["section_mm"] == strength.DEFAULT_SECTION
    assert "mukavemet hesabı yapılmadı" in r["message"]


def test_pipeline_assumes_pla_and_says_so(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec(50, None))
    r = pipeline.design_from_text("x")
    assert "PLA" in r["message"]


def test_pipeline_impossible_load_fails(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec(1e6, "PLA"))
    r = pipeline.design_from_text("x")
    assert r["status"] == "failed"
    assert "uygun kesit bulunamadı" in r["message"]
