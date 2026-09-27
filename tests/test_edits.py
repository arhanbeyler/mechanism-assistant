"""K1: edits of the current design in the chat. The LLM's change list is faked; what is tested is that the
engine applies it correctly and never accepts a number that is not in the user's message."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pipeline  # noqa: E402

FOURBAR = {"mechanism_type": "crank_rocker", "crank_length_mm": 30, "output_swing_deg": 60, "input_swing_deg": None,
           "quick_return": False, "time_ratio": None, "max_load_N": None, "material": None}


def change(field, op="set", number=None, choice=None):
    return {"field": field, "op": op, "number": number, "choice": choice}


def test_set_value_from_text():
    spec, note = pipeline.apply_edit(FOURBAR, {"changes": [change("crank_length_mm", number=40)]}, "krankı 40 mm yap")
    assert spec["crank_length_mm"] == 40 and "30 → 40" in note
    assert FOURBAR["crank_length_mm"] == 30                      # the previous spec is not modified


@pytest.mark.parametrize("text, number", [("yükü 10 kg yap", 98.1), ("make the crank 4 cm", 40.0),
                                          ("load 2 kN", 2000.0)])
def test_unit_converted_numbers_are_accepted(text, number):
    field = "max_load_N" if "kg" in text or "kN" in text else "crank_length_mm"
    spec, _ = pipeline.apply_edit(FOURBAR, {"changes": [change(field, number=number)]}, text)
    assert spec is not None and spec[field] == pytest.approx(number)


def test_invented_number_is_rejected():
    """'A bit longer' has no amount: a number from the LLM must not be used."""
    spec, question = pipeline.apply_edit(FOURBAR, {"changes": [change("crank_length_mm", "multiply", 1.2)]},
                                         "biraz daha uzun olsun")
    assert spec is None and "1.2" in question


def test_missing_amount_is_asked():
    spec, question = pipeline.apply_edit(FOURBAR, {"changes": [change("crank_length_mm", "add", None)]},
                                         "a bit longer please")
    assert spec is None and question.startswith("How much")


def test_relative_changes():
    spec, _ = pipeline.apply_edit(FOURBAR, {"changes": [change("crank_length_mm", "add", 5)]}, "krankı 5 mm uzat")
    assert spec["crank_length_mm"] == 35
    spec, _ = pipeline.apply_edit(FOURBAR, {"changes": [change("crank_length_mm", "multiply", 2)]},
                                  "krankı iki katına çıkar")
    assert spec["crank_length_mm"] == 60


def test_choice_field():
    spec, note = pipeline.apply_edit(FOURBAR, {"changes": [change("material", choice="aluminum")]}, "alüminyum olsun")
    assert spec["material"] == "aluminum" and "aluminum" in note


def test_input_angle_switches_the_fourbar_kind():
    spec, _ = pipeline.apply_edit(FOURBAR, {"changes": [change("input_swing_deg", number=45)]}, "giriş 45 derece dönsün")
    assert spec["mechanism_type"] == "rocker_rocker"
    back, _ = pipeline.apply_edit(spec, {"changes": [change("input_swing_deg", number=360)]}, "tam tur, 360 derece")
    assert back["mechanism_type"] == "crank_rocker" and back["input_swing_deg"] is None


def test_edit_flow_redesigns(monkeypatch):
    monkeypatch.setattr(pipeline.llm_extract, "extract_edit",
                        lambda t, p: {"is_edit": True, "changes": [change("output_swing_deg", number=50)]})
    r = pipeline.design_from_text("salınımı 50 derece yap", prev_spec=FOURBAR)
    assert r["status"] == "ok" and r["edit"] and r["target"]["swing"] == 50
    assert r["message"].startswith("Changed:")


def test_new_request_is_not_an_edit(monkeypatch):
    monkeypatch.setattr(pipeline.llm_extract, "extract_edit", lambda t, p: {"is_edit": False, "changes": []})
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: dict(FOURBAR, crank_length_mm=25))
    r = pipeline.design_from_text("25 mm krank, 60° salınım", prev_spec=FOURBAR)
    assert not r.get("edit") and r["lengths"]["a"] == pytest.approx(25)


def test_yes_no_fields_are_edited_as_choices():
    import llm_extract
    numeric, choice = llm_extract.editable_fields("quick_return")
    assert choice["whitworth"] == ["yes", "no"] and "whitworth" not in numeric
    prev = {"mechanism_type": "quick_return", "stroke_mm": 150, "time_ratio": 1.5, "whitworth": False}
    edit = {"is_edit": True, "changes": [{"field": "whitworth", "op": "set", "number": None, "choice": "yes"}]}
    spec, note = pipeline.apply_edit(prev, edit, "Whitworth olsun")
    assert spec["whitworth"] is True and spec["stroke_mm"] == 150
    r = pipeline.design_from_spec(spec)
    assert r["status"] == "ok" and r.get("whitworth")
