"""
Yerel FEA testleri (gmsh + scikit-fem). Kaba agla ~2 s.
Gerilme el hesabiyla ortusmeli; sehim el hesabindan BUYUK olmali (el hesabi kok
bolgesini mukemmel ankastre sayar, gercekte parca kucuk bir delikten tutulur).
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
pytest.importorskip("gmsh")
pytest.importorskip("skfem")
import fea_local  # noqa: E402


@pytest.fixture(scope="module")
def al_bar():
    return fea_local.compare(60, 9, 3, 3, 49.05, 68900, 0.33, h=2.0)


def test_mid_section_stress_matches_beam_theory(al_bar):
    assert abs(al_bar["err_sigma_pct"]) < 5.0


def test_fea_deflection_not_below_ideal_clamp(al_bar):
    """Delikten tutulan kok, ideal ankastreden daha esnek: FEA sehimi el hesabindan kucuk olamaz."""
    assert al_bar["fea"]["tip_delta"] >= al_bar["hand"]["delta"]
    assert al_bar["err_delta_pct"] < 30.0            # ince uzuvda fark makul kalir


def test_verify_design_runs_for_kol_and_krank():
    out = fea_local.verify_design(
        lengths={"a": 30, "b": 52, "c": 60, "d": 73.5},
        section={"hole": 3.0, "links": {"Kol": {"width": 9, "thickness": 3},
                                        "Krank": {"width": 8, "thickness": 3}}},
        load=49.05, material={"E": 68900, "nu": 0.33}, crank_torque=2064.0)
    assert set(out) == {"Kol", "Krank"}
    assert out["Krank"]["F"] == pytest.approx(2064.0 / 30)
