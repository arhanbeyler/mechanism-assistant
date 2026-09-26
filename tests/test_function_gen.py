"""B9: limited-angle four-bar (Freudenstein function generator) tests."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import function_gen as fg  # noqa: E402
import kinematics as kin  # noqa: E402
import pipeline  # noqa: E402


@pytest.fixture(scope="module")
def design():
    return fg.synthesize(40, 60, 90)


def test_precision_points_are_exact(design):
    """Freudenstein: at the 3 (Chebyshev) precision points the output angle must equal the wanted value."""
    m = design
    a, b, c, d = m["a"], m["b"], m["c"], m["d"]
    for th in fg.chebyshev(m["th0"], m["dth"]):
        want = m["phi0"] + m["dphi"] * (th - m["th0"]) / m["dth"]
        # same assembly branch: simulate from the start to this angle
        fr = fg.simulate_range(a, b, c, d, m["th0"], th - m["th0"], m["phi0"], n=41)
        got = fg._unwrap([f["phi"] for f in fr])[-1]
        diff = (got - want + math.pi) % (2 * math.pi) - math.pi
        assert abs(math.degrees(diff)) < 0.05


def test_design_meets_targets_in_simulation(design):
    mech, frames = fg.frames_for(design)
    meas = fg.measure_range(frames)
    assert meas["in_swing"] == pytest.approx(60, abs=0.5)
    assert abs(meas["out_swing"]) == pytest.approx(90, abs=1.5)
    assert design["err_deg"] <= 1.0
    assert 45 - 0.5 <= meas["mu_min"] and meas["mu_max"] <= 135 + 0.5
    assert design["a"] == pytest.approx(40)


def test_input_does_not_need_full_rotation():
    """Such designs are usually non-Grashof or double-rockers: the input does not make full turns."""
    kinds = set()
    for tin, tout in ((60, 90), (45, 120), (120, 60)):
        m = fg.synthesize(40, tin, tout)
        kinds.add(kin.grashof(m["a"], m["b"], m["c"], m["d"])[1])
    assert any("NON-Grashof" in k or "double-rocker" in k for k in kinds)


def test_candidates_are_all_valid_and_sorted():
    cands = fg.candidates(40, 60, 90, top=10)
    assert cands
    assert all(c["err_deg"] <= 1.0 and c["worst_dev"] <= 45.0 for c in cands)
    ratios = [round(c["ratio"], 2) for c in cands]
    assert ratios == sorted(ratios)


def _spec(**kw):
    s = {"mechanism_type": "rocker_rocker", "crank_length_mm": 40, "input_swing_deg": 60,
         "output_swing_deg": 90, "quick_return": False, "time_ratio": None, "max_load_N": None, "material": None}
    s.update(kw)
    return s


def test_pipeline_rocker_rocker(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec(max_load_N=20, material="PETG"))
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok"
    assert r["mechanism_type"] == "rocker_rocker"
    assert any(c["name"].startswith("Structural error") and c["ok"] for c in r["checks"])
    lo, hi = r["fusion"]["input_range_deg"]
    assert hi - lo == pytest.approx(60, abs=0.5)


def test_pipeline_rocker_rocker_asks_for_input_angle(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec(input_swing_deg=None))
    r = pipeline.design_from_text("x")
    assert r["status"] == "missing"
    assert "input arm turning angle" in r["message"]


@pytest.mark.parametrize("text, angle", [
    ("4 cm kolu 45° çevirince diğer kol 120° dönsün, PETG, 2 kg", 45),
    ("kolu 30 derece itince diğer taraf 75 derece açılsın", 30),
    ("giriş kolu 60° dönünce çıkış 90° dönsün", 60),
    ("kolu 22,5° döndürünce", 22.5),
])
def test_backstop_reads_input_turn_from_sentence(text, angle):
    """Even if the LLM misses the input angle, the number in the user's sentence is used."""
    llm_missed = {"mechanism_type": "crank_rocker", "input_swing_deg": None, "quick_return": False}
    spec, note = pipeline.input_turn_backstop(text, llm_missed)
    assert spec["mechanism_type"] == "rocker_rocker"
    assert spec["input_swing_deg"] == pytest.approx(angle)
    assert note


@pytest.mark.parametrize("text", ["30 mm krank, 60° salınım", "krank tam tur dönsün, kol 60 derece sallansın",
                                  "40 mm krank, 50 derece, hızlı dönüşlü"])
def test_backstop_leaves_crank_rocker_alone(text):
    spec, note = pipeline.input_turn_backstop(text, {"mechanism_type": "crank_rocker", "input_swing_deg": None})
    assert spec["mechanism_type"] == "crank_rocker" and note is None


def test_params_360_means_crank_rocker():
    r = pipeline.design_from_params(30, 60, input_swing_deg=360)
    assert r["mechanism_type"] == "crank_rocker"
