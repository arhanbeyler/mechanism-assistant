"""H4b: disc (Belleville) springs, DIN 2092."""

import pytest

import disc_spring as ds


def test_din_2093_catalogue_forces_for_the_40_mm_discs():
    # A40 40/20.4/2.25 h0 0.9, B40 1.5 h0 1.15, C40 1.0 h0 1.3: about 6.5, 2.6 and 1.0 kN at s = 0.75 h0
    for t, h0, F in ((2.25, 0.9, 6500), (1.5, 1.15, 2620), (1.0, 1.3, 1020)):
        assert ds.force(40, 20.4, t, h0, 0.75 * h0) == pytest.approx(F, rel=0.02)
    assert ds.k1(40, 20.4) == pytest.approx(0.686, abs=2e-3)


def test_deflection_inverts_the_force():
    s = ds.deflection_at(40, 20.4, 1.5, 1.15, 2000)
    assert ds.force(40, 20.4, 1.5, 1.15, s) == pytest.approx(2000, rel=1e-6)
    assert ds.deflection_at(40, 20.4, 1.5, 1.15, 1e6) is None          # beyond 0.75 h0


def test_stack_adds_forces_in_parallel_and_deflections_in_series():
    b, checks = ds.design(5000, 2)
    assert b and all(c["util"] <= 1 for c in checks)
    per_disc = ds.force(b["De"], b["Di"], b["t"], b["h0"], b["s"])
    assert b["np"] * per_disc == pytest.approx(5000, rel=1e-6)
    assert b["y"] == pytest.approx(b["ns"] * b["s"])
    assert b["y"] == pytest.approx(2, rel=0.10)
