"""Dişli ve yay modullerinin tasarim akisi testleri (sahte LLM cevabiyla)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pipeline  # noqa: E402


def _fake(monkeypatch, **spec):
    base = {"mechanism_type": "unknown", "material": None, "quick_return": False}
    base.update(spec)
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: base)


def test_gear_pipeline_ok(monkeypatch):
    _fake(monkeypatch, mechanism_type="spur_gear", gear_ratio=3, torque_Nm=2, speed_rpm=None, material="PLA")
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok" and r["mechanism_type"] == "spur_gear"
    assert r["fusion"]["type"] == "spur_gear" and r["gear"]["z2"] == 3 * r["gear"]["z1"]


def test_gear_does_not_invent_torque(monkeypatch):
    _fake(monkeypatch, mechanism_type="spur_gear", gear_ratio=3, torque_Nm=None, speed_rpm=None)
    r = pipeline.design_from_text("x")
    assert r["status"] == "missing" and "tork" in r["message"]


def test_spring_pipeline_ok(monkeypatch):
    _fake(monkeypatch, mechanism_type="spring", spring_force_N=100, spring_deflection_mm=20, spring_shape="helical",
          spring_taper=None, outer_diameter_max_mm=None)
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok" and r["spring"]["k"] > 0 and r["fusion"]["type"] == "spring"


def test_conical_spring_without_ratio_uses_default_and_says_so(monkeypatch):
    _fake(monkeypatch, mechanism_type="spring", spring_force_N=50, spring_deflection_mm=15, spring_shape="conical",
          spring_taper=None, outer_diameter_max_mm=None)
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok" and r["spring"]["taper"] == 0.5
    assert "Konik oranı belirtilmediği" in r["message"]


def test_plastic_spring_note(monkeypatch):
    _fake(monkeypatch, mechanism_type="spring", spring_force_N=50, spring_deflection_mm=15, spring_shape="helical",
          spring_taper=None, outer_diameter_max_mm=None, material="PLA")
    r = pipeline.design_from_text("x")
    assert "çelik telden" in r["message"]


def test_modules_params_are_complete():
    for key, mod in pipeline.MODULES.items():
        for p in mod["params"]:
            assert {"key", "label", "min", "max", "step", "default"} <= set(p)
            assert p["min"] <= p["default"] <= p["max"]
        # kaydirici degerleriyle tasarim cagrilabilmeli
        r = pipeline.design_from_params(kind=key, **{p["key"]: p["default"] for p in mod["params"]})
        assert r["status"] == "ok", (key, r["message"])
