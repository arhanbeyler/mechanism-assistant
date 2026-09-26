"""
Local FEA tests (gmsh + scikit-fem). About 2 s with a coarse mesh.
The stress must match the hand calculation; the deflection must be LARGER than the hand calculation
(which treats the root as perfectly clamped, while the real part is held by a small hole).
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
    """A root held by a hole is more flexible than an ideal clamp: the FEA deflection cannot be smaller."""
    assert al_bar["fea"]["tip_delta"] >= al_bar["hand"]["delta"]
    assert al_bar["err_delta_pct"] < 30.0            # the difference stays moderate for a slender link


def test_verify_design_runs_for_rocker_and_crank():
    out = fea_local.verify_design(
        lengths={"a": 30, "b": 52, "c": 60, "d": 73.5},
        section={"hole": 3.0, "links": {"Rocker": {"width": 9, "thickness": 3},
                                        "Crank": {"width": 8, "thickness": 3}}},
        load=49.05, material={"E": 68900, "nu": 0.33}, crank_torque=2064.0)
    assert set(out) == {"Rocker", "Crank"}
    assert out["Crank"]["F"] == pytest.approx(2064.0 / 30)
