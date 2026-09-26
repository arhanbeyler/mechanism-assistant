"""
Design-flow tests. A fake (fixed) reply replaces the LLM: the tests are fast and always give the same
result. The LLM itself is not tested here; what the code does with its output is.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pipeline  # noqa: E402


def spec(crank=None, swing=None, quick=False, Q=None):
    return {"mechanism_type": "crank_rocker", "crank_length_mm": crank,
            "output_swing_deg": swing, "quick_return": quick, "time_ratio": Q,
            "max_load_N": None, "material": None}


@pytest.fixture
def fake_llm(monkeypatch):
    """Makes pipeline.extract_spec return the given fake reply."""
    def use(s):
        monkeypatch.setattr(pipeline, "extract_spec", lambda text: s)
    return use


def test_no_quick_return_uses_q1(fake_llm):
    fake_llm(spec(30, 60))
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok"
    assert r["target"]["Q"] == 1.0


def test_quick_return_without_number_engine_picks_q(fake_llm):
    """The LLM invents no number; the engine picks Q and says so in the message."""
    fake_llm(spec(30, 50, quick=True))
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok"
    assert r["target"]["Q"] > 1.0
    assert r["measured"]["mu_min"] >= 45.0 - 0.5
    assert "Q not given" in r["message"]


def test_given_q_is_used_and_verified(fake_llm):
    fake_llm(spec(30, 50, quick=True, Q=1.2))
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok"
    assert r["measured"]["Q"] == pytest.approx(1.2, abs=0.03)
    assert any(c["name"].startswith("Time ratio") and c["ok"] for c in r["checks"])


def test_too_large_q_is_rejected_with_suggestion(fake_llm):
    fake_llm(spec(25, 60, quick=True, Q=1.5))
    r = pipeline.design_from_text("x")
    assert r["status"] == "failed"
    assert "Q of at most" in r["message"]


def test_missing_crank_asks_instead_of_guessing(fake_llm):
    fake_llm(spec(None, 40))
    r = pipeline.design_from_text("x")
    assert r["status"] == "missing"
    assert "crank" in r["message"]
    assert "lengths" not in r


def test_invalid_q_rejected(fake_llm):
    fake_llm(spec(30, 50, quick=True, Q=0.8))
    assert pipeline.design_from_text("x")["status"] == "missing"


def test_max_feasible_q():
    q = pipeline.max_feasible_q(30, 50)
    assert q == pytest.approx(1.15)
