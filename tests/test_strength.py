"""Strength module tests."""

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
    """The coupler is a two-force member: F_AB = F / sin(mu); largest where sin(mu) is smallest."""
    m, frames, _ = mech30
    f = strength.analyze_forces(m, frames, 100.0)
    min_sin = min(math.sin(math.radians(fr["mu"])) for fr in frames)
    assert f["F_ab_max"] == pytest.approx(100.0 / min_sin)
    assert f["M_rocker"] == pytest.approx(100.0 * m["c"])


def test_input_torque_bounded_by_crank_length(mech30):
    """The crank moment is at most F_AB * a (moment arm <= crank length)."""
    m, frames, _ = mech30
    f = strength.analyze_forces(m, frames, 100.0)
    assert 0 < f["M_crank_max"] <= f["F_ab_max"] * m["a"] + 1e-9


def test_check_link_hand_calculation():
    """Hand calculation: w=20, t=4, d=4, F_AB=100 N, PLA (allowable 40/3 MPa)."""
    forces = {"F_ab_max": 100.0, "M_crank_max": 1000.0, "M_rocker": 2000.0}
    allow = 40.0 / 3.0
    S_net = 4 * (20 ** 3 - 4 ** 3) / (6 * 20)
    I_min = 20 * 4 ** 3 / 12

    kt = 3.00 - 3.13 * 0.2 + 3.66 * 0.2 ** 2 - 1.53 * 0.2 ** 3     # d/w = 4/20, PLA is brittle

    coupler = {x["name"]: x for x in strength.check_link("Coupler", forces, 50.0, PLA, 20, 4, 4)}
    assert coupler["Coupler net section"]["util"] == pytest.approx(kt * (100 / 64) / allow)
    assert coupler["Coupler pin bearing"]["util"] == pytest.approx((100 / 16) / allow)
    P_cr = math.pi ** 2 * 2500 * I_min / 50 ** 2
    assert coupler["Coupler buckling"]["util"] == pytest.approx(100 * 3 / P_cr)
    stretch = 100 * 50 / (2500 * 20 * 4)                             # F L / (E A)
    assert coupler["Coupler axial stretch"]["util"] == pytest.approx(stretch / (50 / 200))

    rocker = {x["name"]: x for x in strength.check_link("Rocker", forces, 60.0, PLA, 20, 4, 4)}
    assert rocker["Rocker bending + axial"]["util"] == pytest.approx((2000 / S_net + kt * 100 / 64) / allow)
    P_tip, I_in = 2000 / 60, 4 * 20 ** 3 / 12
    G = 2500 / (2 * (1 + 0.36))
    deflection = (P_tip * 60 ** 3 / (3 * 2500 * I_in)                # bending
                  + P_tip * 60 / ((5 / 6) * G * 20 * 4)               # Timoshenko shear
                  + P_tip * 60 ** 2 / (6.0 * 2500 * 4 * 2 ** 2))      # hub rotation (C = 6)
    assert rocker["Rocker deflection (bending + shear + hub)"]["util"] == pytest.approx(deflection / (60 / 200))

    tau = 4 * 100 / (math.pi * 16)
    assert strength.check_pin(forces, PLA, 4)["util"] == pytest.approx(tau / (0.5 * allow))


def test_hub_model_is_conservative_against_fea():
    """B7g: the hand calculation with the hub model must not under-predict the FEA deflection (C at the
    lower bound), nor overdo it. Wide link + small pin: the case where the hub effect dominates."""
    pytest.importorskip("gmsh")
    import fea_local
    L, w, t, d, F, E, nu = 60, 29, 3, 4, 50.0, 2000.0, 0.38
    fea = fea_local.solve_cantilever(L, w, t, d, F, E, nu, h=2.0)["tip_delta"]
    mat = {"sy": 35, "E": E, "nu": nu, "n": 3.0, "brittle": False}
    forces = {"F_ab_max": 0.0, "M_crank_max": 0.0, "M_rocker": F * L}
    c = {x["name"]: x for x in strength.check_link("Rocker", forces, L, mat, w, t, d)}
    model = c["Rocker deflection (bending + shear + hub)"]["util"] * L / 200
    assert model >= fea * 0.98                  # safe side
    assert model <= fea * 1.5                   # but not more than 1.5 times over


@pytest.mark.parametrize("r, expected", [(0.0, 3.00), (0.5, 2.159), (0.3, 2.349)])
def test_kt_roark_values(r, expected):
    """Roark: a small hole gives Kt -> 3 (infinite plate); it falls as the hole grows (net section)."""
    assert strength.kt_hole_tension(r * 10, 10) == pytest.approx(expected, abs=0.002)


def test_kt_only_for_brittle_materials():
    """No Kt in the static check of ductile steel; brittle PLA gets it."""
    forces = {"F_ab_max": 100.0, "M_crank_max": 0.0, "M_rocker": 0.0}
    steel = strength.MATERIALS["steel"]
    net = {x["name"]: x for x in strength.check_link("Coupler", forces, 50.0, steel, 20, 4, 4)}
    assert net["Coupler net section"]["util"] == pytest.approx((100 / 64) / (235 / 2))
    assert "Kt" not in net["Coupler net section"]["detail"]
    pla = {x["name"]: x for x in strength.check_link("Coupler", forces, 50.0, PLA, 20, 4, 4)}
    assert "Kt" in pla["Coupler net section"]["detail"]


def _volume(sec, lengths):
    return sum(s["width"] * s["thickness"] * lengths[strength.LINK_KEYS[n]]
               for n, s in sec["links"].items())


def test_sized_links_pass_and_each_is_minimal(mech30):
    m, frames, lengths = mech30
    forces = strength.analyze_forces(m, frames, 50.0)
    sec, checks = strength.size_links(forces, lengths, PLA)
    assert sec is not None
    assert all(c["util"] <= 1.0 for c in checks)
    d, ds = sec["hole"], sec["shaft"]
    for name, s in sec["links"].items():
        w, t = s["width"], s["thickness"]
        d_big = max(strength.link_holes(name, d, ds))
        if w - 1 >= max(math.ceil(2.5 * d_big), 8):   # 1 mm narrower: at least one check must fail
            narrower = strength.check_link(name, forces, lengths[strength.LINK_KEYS[name]],
                                           PLA, w - 1, t, d, ds)
            assert max(c["util"] for c in narrower) > 1.0


def test_shaft_and_pin_are_separate(mech30):
    """B7h: the torque-carrying shaft (O2, O4) may be thicker than the pin (A, B); the coupler only has pins."""
    m, frames, lengths = mech30
    sec, checks = strength.size_links(strength.analyze_forces(m, frames, 50.0), lengths, PLA)
    assert sec["shaft"] >= sec["hole"]
    assert strength.link_holes("Coupler", sec["hole"], sec["shaft"]) == (sec["hole"], sec["hole"])
    assert strength.link_holes("Ground", sec["hole"], sec["shaft"]) == (sec["shaft"], sec["shaft"])
    assert any(c["name"].startswith("Shaft (steel)") for c in checks)


def test_shaft_torsion_hand_calculation():
    """Steel shaft: tau = 16 T / (pi d^3) + 4 F / (pi d^2), allowable 0.5 * 235 / 2."""
    forces = {"F_ab_max": 100.0, "M_crank_max": 2000.0, "M_rocker": 3000.0}
    tau = 16 * 3000 / (math.pi * 8 ** 3) + 4 * 100 / (math.pi * 8 ** 2)
    c = strength.check_shaft(forces, PLA, 8)
    assert c["util"] == pytest.approx(tau / (0.5 * 235 / 2))


def test_governing_checks_match_load_type(mech30):
    """The crank is bent by the drive torque -> its governing check comes from the moment (bending
    stress or bending / hub deflection); the ground link carries axial load only -> no bending.
    (The first assumption "a short crank needs a smaller section" was wrong: bending needs width.)"""
    m, frames, lengths = mech30
    forces = strength.analyze_forces(m, frames, 50.0)
    sec, checks = strength.size_links(forces, lengths, PLA)
    crank = [c for c in checks if c["name"].startswith("Crank")]
    governing = max(crank, key=lambda c: c["util"])["name"]
    assert governing in ("Crank bending + axial", "Crank deflection (bending + shear + hub)")
    assert not any("Ground bending" in c["name"] or "Ground deflection" in c["name"] for c in checks)


def test_per_link_uses_less_material_than_uniform(mech30):
    """Per-link sections must be lighter than giving every link the worst link's section."""
    m, frames, lengths = mech30
    sec, _ = strength.size_links(strength.analyze_forces(m, frames, 50.0), lengths, PLA)
    worst = max(sec["links"].values(), key=lambda s: s["width"] * s["thickness"])
    uniform = sum(worst["width"] * worst["thickness"] * L for L in lengths.values())
    assert _volume(sec, lengths) < uniform


def test_steel_needs_less_material_than_pla(mech30):
    m, frames, lengths = mech30
    forces = strength.analyze_forces(m, frames, 200.0)
    pla, _ = strength.size_links(forces, lengths, PLA)
    steel, _ = strength.size_links(forces, lengths, strength.MATERIALS["steel"])
    assert _volume(steel, lengths) < _volume(pla, lengths)


def test_impossible_load_returns_none(mech30):
    m, frames, lengths = mech30
    sec, checks = strength.size_links(strength.analyze_forces(m, frames, 1e6), lengths, PLA)
    assert sec is None and any(c["util"] > 1.0 for c in checks)


# ---------------------------------------------------------------- pipeline (fake LLM)

def _spec(load=None, mat=None):
    return {"mechanism_type": "crank_rocker", "crank_length_mm": 30, "output_swing_deg": 60,
            "quick_return": False, "time_ratio": None, "max_load_N": load, "material": mat}


def test_pipeline_sizes_section_when_load_given(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec(50, "PETG"))
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok"
    assert r["fusion"]["section_mm"] != strength.DEFAULT_SECTION
    assert any(c["name"].startswith("Buckling") for c in r["checks"])
    assert r["strength"]["material"].startswith("PETG")


def test_pipeline_does_not_invent_load(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec())
    r = pipeline.design_from_text("x")
    assert r["status"] == "ok"
    assert "strength" not in r
    sec = r["fusion"]["section_mm"]                    # default section (+ boss data)
    assert sec["hole"] == strength.DEFAULT_SECTION["hole"]
    assert all(s["width"] == 10.0 and s["thickness"] == 4.0 for s in sec["links"].values())
    assert "no strength calculation" in r["message"]


def test_pipeline_assumes_pla_and_says_so(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec(50, None))
    r = pipeline.design_from_text("x")
    assert "PLA" in r["message"]


def test_pipeline_impossible_load_fails(monkeypatch):
    monkeypatch.setattr(pipeline, "extract_spec", lambda t: _spec(1e6, "PLA"))
    r = pipeline.design_from_text("x")
    assert r["status"] == "failed"
    assert "section carries" in r["message"]
