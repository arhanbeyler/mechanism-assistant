"""H1: bolted tension joint (Shigley ch. 8)."""

import pytest

import bolted


def test_wileman_close_to_the_frustum_model():
    for d, grip in ((10, 20), (12, 40), (16, 50), (24, 80)):
        w = bolted.member_stiffness_wileman(d, grip, bolted.MEMBERS["steel"])
        f = bolted.member_stiffness_frustum(d, grip, bolted.MEMBERS["steel"]["E"])
        assert w == pytest.approx(f, rel=0.08)


def test_load_sharing_is_the_spring_model():
    size = next(s for s in bolted.SIZES if s[0] == 12)
    a = bolted.analyse(size, 4, 40000, 30)
    Pb = 10000
    delta = Pb / (a["kb"] + a["km"])                      # both springs stretch by the same amount
    assert a["F_bolt_max"] - a["Fi"] == pytest.approx(a["kb"] * delta)
    assert a["Fi"] - a["F_member"] == pytest.approx(a["km"] * delta)


def test_goodman_factor_puts_the_load_line_on_the_goodman_line():
    size = next(s for s in bolted.SIZES if s[0] == 16)
    a = bolted.analyse(size, 6, 60000, 40, fluctuating=True)
    c = bolted.CLASSES["8.8"]
    n, sa, si = a["n_f"], a["sigma_a"], a["sigma_i"]
    assert (n * sa) / c["Se"] + (si + n * sa) / c["Sut"] == pytest.approx(1.0)   # sigma_i stays, the load grows


def test_preload_and_torque():
    size = next(s for s in bolted.SIZES if s[0] == 16)
    a = bolted.analyse(size, 4, 1000, 30)
    assert a["Fi"] == pytest.approx(0.75 * 157 * 600)
    assert a["T"] == pytest.approx(0.2 * a["Fi"] * 16)


def test_design_defaults_to_four_bolts_and_passes():
    a, checks = bolted.design(50000, 40, fluctuating=True)
    assert a["N"] == 4 and all(c["util"] <= 1 for c in checks)
    assert a["L"] >= 40 + a["nut_h"]
    more, _ = bolted.design(50000, 40, N=8, fluctuating=True)
    assert more["d"] <= a["d"]


def test_force_backstop_reads_kn_as_a_force():
    import pipeline
    text = "50 kN değişken yükü taşıyacak cıvatalı flanş"
    spec, note = pipeline.force_backstop(text, {"mechanism_type": "bolted_joint", "force_N": 490500})
    assert spec["force_N"] == 50000 and note
    same, none = pipeline.force_backstop(text, {"mechanism_type": "bolted_joint", "force_N": 50000})
    assert same["force_N"] == 50000 and none is None
    kg, _ = pipeline.force_backstop("300 kg kaldırsın", {"force_N": 2943})     # a mass stays converted
    assert kg["force_N"] == 2943
