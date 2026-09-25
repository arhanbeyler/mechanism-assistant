"""
Kinematik motor testleri. Calistirmak icin proje klasorunde:  python -m pytest
Her test bir muhendislik gercegini kontrol eder; biri bozulursa hemen gorulur.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import kinematics as kin  # noqa: E402


# ---------------------------------------------------------------- Grashof

@pytest.mark.parametrize("a, b, c, d, ok, kind", [
    (2, 5, 4, 6, True, "krank-kol"),        # en kisa = krank
    (6, 5, 4, 2, True, "cift krank"),       # en kisa = sabit
    (5, 2, 4, 6, True, "cift kol"),         # en kisa = biyel
    (2, 7, 3, 9, False, "DEGIL"),           # s + l > p + q
])
def test_grashof_classification(a, b, c, d, ok, kind):
    got_ok, got_kind = kin.grashof(a, b, c, d)
    assert got_ok == ok
    assert kind in got_kind


# ---------------------------------------------------------------- Q = 1 sentez

@pytest.mark.parametrize("a, psi", [(20, 60), (30, 45), (40, 50), (25, 80)])
def test_q1_rocker_length_formula(a, psi):
    """Norton: Q=1 icin c = a / sin(psi/2)."""
    m = kin.synthesize_crank_rocker(a, psi)
    assert m["c"] == pytest.approx(a / math.sin(math.radians(psi) / 2))


@pytest.mark.parametrize("a, psi", [(20, 60), (30, 45), (40, 50)])
def test_q1_design_meets_targets_in_simulation(a, psi):
    m = kin.synthesize_crank_rocker(a, psi)
    meas = kin.measure(kin.simulate(m))
    assert meas["swing"] == pytest.approx(psi, abs=0.1)
    assert meas["Q"] == pytest.approx(1.0, abs=0.02)
    assert meas["mu_min"] >= 45.0 - 0.5
    assert meas["mu_max"] <= 135.0 + 0.5
    assert kin.grashof(m["a"], m["b"], m["c"], m["d"])[0]


def test_q1_large_swing_is_rejected():
    """110 derece Q=1 ile iletim acisi sinirini saglayamaz (limit 90 - psi/2 = 35)."""
    m = kin.synthesize_crank_rocker(25, 110)
    assert m["mu_min"] < kin.MU_MIN_DEG


# ---------------------------------------------------------------- hizli donus

@pytest.mark.parametrize("a, psi, Q", [(25, 60, 1.1), (25, 90, 1.3), (30, 50, 1.2)])
def test_quick_return_matches_simulation(a, psi, Q):
    """Sentezin iddia ettigi salinim ve zaman orani, bagimsiz simulasyonla tutmali.
    (Bu test, farkli montaj kollari hatasini yakalar.)"""
    m = kin.synthesize_quick_return(a, psi, Q)
    meas = kin.measure(kin.simulate(m))
    assert meas["swing"] == pytest.approx(psi, abs=0.1)
    assert meas["Q"] == pytest.approx(Q, abs=0.03)
    assert m["a"] == pytest.approx(a)


def test_quick_return_worsens_transmission_angle():
    """Bulgu: ayni salinimda Q arttikca en kotu iletim acisi kotulesir."""
    mus = [kin.synthesize_quick_return(25, 60, Q)["mu_min"] for Q in (1.0, 1.2, 1.5)]
    assert mus[0] > mus[1] > mus[2]


# ---------------------------------------------------------------- simulasyon

def test_simulation_preserves_link_lengths():
    """Her karede uzuv boylari sabit kalmali (rijit cisim)."""
    m = kin.synthesize_quick_return(25, 60, 1.1)
    o2 = m["o2"]
    for f in kin.simulate(m, n=90):
        A, B = f["A"], f["B"]
        assert math.dist(o2, A) == pytest.approx(m["a"], abs=1e-6)
        assert math.dist(A, B) == pytest.approx(m["b"], abs=1e-6)
        assert math.dist((0, 0), B) == pytest.approx(m["c"], abs=1e-6)
