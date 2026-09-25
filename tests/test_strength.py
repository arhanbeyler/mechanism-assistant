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


def test_check_link_hand_calculation():
    """Elle hesap: w=20, t=4, d=4, F_AB=100 N, PLA (izin verilen 40/3 MPa)."""
    forces = {"F_ab_max": 100.0, "M_crank_max": 1000.0, "M_rocker": 2000.0}
    allow = 40.0 / 3.0
    S_net = 4 * (20 ** 3 - 4 ** 3) / (6 * 20)
    I_min = 20 * 4 ** 3 / 12

    kt = 3.00 - 3.13 * 0.2 + 3.66 * 0.2 ** 2 - 1.53 * 0.2 ** 3     # d/w = 4/20, PLA kirilgan

    biyel = {x["name"]: x for x in strength.check_link("Biyel", forces, 50.0, PLA, 20, 4, 4)}
    assert biyel["Biyel net kesit"]["util"] == pytest.approx(kt * (100 / 64) / allow)
    assert biyel["Biyel pim ezilmesi"]["util"] == pytest.approx((100 / 16) / allow)
    P_cr = math.pi ** 2 * 2500 * I_min / 50 ** 2
    assert biyel["Biyel burkulması"]["util"] == pytest.approx(100 * 3 / P_cr)
    uzama = 100 * 50 / (2500 * 20 * 4)                               # F L / (E A)
    assert biyel["Biyel eksenel uzama"]["util"] == pytest.approx(uzama / (50 / 200))

    kol = {x["name"]: x for x in strength.check_link("Kol", forces, 60.0, PLA, 20, 4, 4)}
    assert kol["Kol eğilme + eksenel"]["util"] == pytest.approx((2000 / S_net + kt * 100 / 64) / allow)
    P_tip, I_in = 2000 / 60, 4 * 20 ** 3 / 12
    G = 2500 / (2 * (1 + 0.36))
    sehim = (P_tip * 60 ** 3 / (3 * 2500 * I_in)                     # egilme
             + P_tip * 60 / ((5 / 6) * G * 20 * 4)                    # Timoshenko kayma
             + P_tip * 60 ** 2 / (6.0 * 2500 * 4 * 2 ** 2))           # gobek donmesi (C = 6)
    assert kol["Kol sehimi (eğilme + kayma + göbek)"]["util"] == pytest.approx(sehim / (60 / 200))

    tau = 4 * 100 / (math.pi * 16)
    assert strength.check_pin(forces, PLA, 4)["util"] == pytest.approx(tau / (0.5 * allow))


def test_hub_model_is_conservative_against_fea():
    """B7g: gobek modelli el hesabi, FEA sehimini asagi tahmin etmemeli (C alt sinirda)
    ve asiri da abartmamali. Genis uzuv + kucuk pim: gobek etkisinin baskin oldugu durum."""
    pytest.importorskip("gmsh")
    import fea_local
    L, w, t, d, F, E, nu = 60, 29, 3, 4, 50.0, 2000.0, 0.38
    fea = fea_local.solve_cantilever(L, w, t, d, F, E, nu, h=2.0)["tip_delta"]
    mat = {"sy": 35, "E": E, "nu": nu, "n": 3.0, "brittle": False}
    forces = {"F_ab_max": 0.0, "M_crank_max": 0.0, "M_rocker": F * L}
    c = {x["name"]: x for x in strength.check_link("Kol", forces, L, mat, w, t, d)}
    model = c["Kol sehimi (eğilme + kayma + göbek)"]["util"] * L / 200
    assert model >= fea * 0.98                  # guvenli taraf
    assert model <= fea * 1.5                   # ama 1.5 kattan fazla abartmiyor


@pytest.mark.parametrize("r, expected", [(0.0, 3.00), (0.5, 2.159), (0.3, 2.349)])
def test_kt_roark_values(r, expected):
    """Roark: delik kucukken Kt -> 3 (sonsuz plaka), delik buyudukce azalir (net kesite gore)."""
    assert strength.kt_hole_tension(r * 10, 10) == pytest.approx(expected, abs=0.002)


def test_kt_only_for_brittle_materials():
    """Sunek celikte statik hesapta Kt yok; kirilgan PLA'da var."""
    forces = {"F_ab_max": 100.0, "M_crank_max": 0.0, "M_rocker": 0.0}
    steel = strength.MATERIALS["steel"]
    net = {x["name"]: x for x in strength.check_link("Biyel", forces, 50.0, steel, 20, 4, 4)}
    assert net["Biyel net kesit"]["util"] == pytest.approx((100 / 64) / (235 / 2))
    assert "Kt" not in net["Biyel net kesit"]["detail"]
    pla = {x["name"]: x for x in strength.check_link("Biyel", forces, 50.0, PLA, 20, 4, 4)}
    assert "Kt" in pla["Biyel net kesit"]["detail"]


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
        if w - 1 >= max(math.ceil(2.5 * d_big), 8):   # 1 mm daha dar: en az bir kontrol kalmali
            narrower = strength.check_link(name, forces, lengths[strength.LINK_KEYS[name]],
                                           PLA, w - 1, t, d, ds)
            assert max(c["util"] for c in narrower) > 1.0


def test_shaft_and_pin_are_separate(mech30):
    """B7h: tork tasiyan mil (O2, O4) pimden (A, B) kalin olabilir; biyel sadece pim tasir."""
    m, frames, lengths = mech30
    sec, checks = strength.size_links(strength.analyze_forces(m, frames, 50.0), lengths, PLA)
    assert sec["shaft"] >= sec["hole"]
    assert strength.link_holes("Biyel", sec["hole"], sec["shaft"]) == (sec["hole"], sec["hole"])
    assert strength.link_holes("Sabit", sec["hole"], sec["shaft"]) == (sec["shaft"], sec["shaft"])
    assert any(c["name"].startswith("Mil (çelik)") for c in checks)


def test_shaft_torsion_hand_calculation():
    """Celik mil: tau = 16 T / (pi d^3) + 4 F / (pi d^2), izin 0.5 * 235 / 2."""
    forces = {"F_ab_max": 100.0, "M_crank_max": 2000.0, "M_rocker": 3000.0}
    tau = 16 * 3000 / (math.pi * 8 ** 3) + 4 * 100 / (math.pi * 8 ** 2)
    c = strength.check_shaft(forces, PLA, 8)
    assert c["util"] == pytest.approx(tau / (0.5 * 235 / 2))


def test_governing_checks_match_load_type(mech30):
    """Krank tahrik torkuyla egilmeye calisir -> kritik kontrolu momentten gelir (egilme
    gerilmesi ya da egilme/gobek sehimi); sabit uzuv sadece eksenel yuk tasir -> egilme yok.
    (Ilk varsayim 'kisa krank daha kucuk olmali' yanlisti: krank egilme icin genislik ister.)"""
    m, frames, lengths = mech30
    forces = strength.analyze_forces(m, frames, 50.0)
    sec, checks = strength.size_links(forces, lengths, PLA)
    crank = [c for c in checks if c["name"].startswith("Krank")]
    governing = max(crank, key=lambda c: c["util"])["name"]
    assert governing in ("Krank eğilme + eksenel", "Krank sehimi (eğilme + kayma + göbek)")
    assert not any("Sabit eğilme" in c["name"] or "Sabit sehimi" in c["name"] for c in checks)


def test_per_link_uses_less_material_than_uniform(mech30):
    """Uzuv basina kesit, en kotu uzvun kesitini her yere vermekten hafif olmali."""
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
    sec = r["fusion"]["section_mm"]                    # varsayilan kesit (+ gobek bilgisi)
    assert sec["hole"] == strength.DEFAULT_SECTION["hole"]
    assert all(s["width"] == 10.0 and s["thickness"] == 4.0 for s in sec["links"].values())
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
