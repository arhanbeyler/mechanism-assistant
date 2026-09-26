"""F5: toggle clamp."""

import math

import pytest

import pipeline
import strength
import toggle


@pytest.fixture(scope="module")
def clamp():
    d, checks = toggle.design(90, 90, 60, strength.MATERIALS["steel"], F=1000)
    return d, checks


def _cross(u, v):
    return u[0] * v[1] - u[1] * v[0]


def test_dead_centre_and_opening(clamp):
    d, checks = clamp
    m, fr, an = d["mech"], d["frames"], d["analysis"]
    dc = fr[an["k_dc"]]
    O2, A, B = m["O2"], dc["A"], dc["B"]
    # O2, A, B collinear with A between: the handle link and the coupler in line
    assert abs(_cross((A[0] - O2[0], A[1] - O2[1]), (B[0] - A[0], B[1] - A[1]))) < 1e-6 * m["a"] * m["b"] * 50
    assert math.dist(O2, B) == pytest.approx(m["a"] + m["b"], rel=1e-4)
    assert fr[0]["psi"] - an["psi_dc"] == pytest.approx(90, abs=0.01)
    assert all(c["util"] <= 1 for c in checks if c["group"] == "Motion")


def test_overcentre_locks(clamp):
    """Past the dead centre the coupler line crosses O2: a spindle force now turns the handle the other way,
    into the stop; the arm lifts only a little."""
    d, _ = clamp
    m, fr, an = d["mech"], d["frames"], d["analysis"]

    def side(f):
        w = (f["B"][0] - f["A"][0], f["B"][1] - f["A"][1])
        return _cross(w, (m["O2"][0] - f["A"][0], m["O2"][1] - f["A"][1]))

    assert side(fr[an["k_dc"] - 10]) * side(fr[-1]) < 0
    assert 0 < an["backoff_deg"] < 1


def test_handle_torque_statics_equals_virtual_work(clamp):
    d, _ = clamp
    m, fr = d["mech"], d["frames"]
    for i in (20, 80, 150, 220):
        dpsi = abs(fr[i + 1]["psi"] - fr[i - 1]["psi"]) / (fr[i + 1]["th"] - fr[i - 1]["th"])
        assert toggle.handle_torque(m, fr[i], 1000, 60) == pytest.approx(1000 * 60 * dpsi, rel=5e-3)
    assert toggle.handle_torque(m, fr[d["analysis"]["k_dc"]], 1000, 60) < 0.02 * toggle.handle_torque(m, fr[80], 1000, 60)


def test_locked_forces_balance(clamp):
    """Arm in equilibrium: spindle force + coupler force + pivot reaction = 0, moments about O4 = 0."""
    d, _ = clamp
    m, fr, an = d["mech"], d["frames"], d["analysis"]
    A, B = fr[-1]["A"], fr[-1]["B"]
    c = math.hypot(*B)
    u = (B[0] / c, B[1] / c)
    w = ((B[0] - A[0]) / m["b"], (B[1] - A[1]) / m["b"])
    # torque of the coupler force about O4 equals the spindle torque
    assert an["F_AB"] * c * abs(_cross(u, w)) == pytest.approx(1000 * 60, rel=1e-9)
    assert an["M_arm"] == pytest.approx(1000 * (60 - c))
    assert d["section"]["hole"] in strength.PIN_DIAMETERS


def test_spacer_clears(clamp):
    d, _ = clamp
    sec = {"links": {n: dict(d["section"]["links"][n], boss0=max(d["section"]["links"][n]["width"], 2.5 * 12))
                     for n in d["section"]["links"]}}
    p, gap = toggle.spacer(d["mech"], d["frames"], 60, 120, sec, 6)
    assert gap >= toggle.SPACER_GAP


def test_pipeline_toggle_clamp():
    r = pipeline.design_from_params(kind="toggle_clamp", opening_deg=90, arm_length_mm=60, clamp_force_N=500,
                                    material="aluminum")
    assert r["status"] == "ok", r["message"]
    f = r["fusion"]
    assert f["type"] == "toggle_clamp" and f["input_range_deg"][1] - f["input_range_deg"][0] == pytest.approx(93)
    assert any("Base post" in c["name"] and c["ok"] for c in r["checks"])
    assert pipeline.design_from_spec({"mechanism_type": "toggle_clamp", "opening_deg": 90})["status"] == "missing"
