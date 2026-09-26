"""F6: ratchet and pawl."""

import math

import pytest

import pipeline
import ratchet


@pytest.fixture(scope="module")
def pla():
    d, checks = ratchet.design(5000, "PLA", 24)
    g = d["geometry"]
    pose = ratchet.pawl_pose(g, d["lp"])
    head, _ = ratchet.pawl_points(g, pose, d["w_pawl"])
    return d, checks, g, pose, head


def test_hook_beats_friction(pla):
    d, checks, g, pose, _ = pla
    assert math.tan(math.radians(g["alpha"])) > d["mu"]
    assert all(c["util"] <= 1 for c in checks)


def test_pivot_on_the_tangent(pla):
    """The pivot lies on the tangent at the contact point (so the face's normal force pulls the pawl in)."""
    _, _, g, pose, _ = pla
    C, pv = pose["C"], pose["pivot"]
    radial = (C[0] / pose["rc"], C[1] / pose["rc"])
    assert abs((pv[0] - C[0]) * radial[0] + (pv[1] - C[1]) * radial[1]) < 1e-9
    # the face normal is tilted from that tangent by the hook angle plus the contact point's polar angle
    # (the face leans alpha from the radius through the tip, the tangent is taken at C): the check with
    # alpha alone is on the safe side
    n, t = pose["normal"], pose["tangent"]
    effective = math.degrees(math.acos(n[0] * t[0] + n[1] * t[1]))
    assert effective == pytest.approx(g["alpha"] + math.degrees(math.atan2(C[1], C[0])), abs=0.05)
    assert effective >= g["alpha"]


def test_pawl_clear_at_rest_and_riding(pla):
    _, _, g, pose, head = pla
    wheel = ratchet.outline(g)
    assert not any(ratchet._inside(wheel, p) for p in head)
    table, sign = ratchet.ride_table(g, pose, head)
    step = 360 / g["z"]
    assert table[0] == 0 and max(table) > 0
    assert table[int(step)] == pytest.approx(0, abs=0.1)              # dropped into the next tooth space
    # with the lift from the table the head stays out of the wheel (checked at half-degree steps)
    for k in range(1, 2 * int(step)):
        phi = math.radians(k / 2)
        lift = table[k // 2]
        for p in head:
            q = ratchet._rot(ratchet._rot(p, pose["pivot"], sign * math.radians(max(lift, table[min(k // 2 + 1, 359)]))),
                             (0, 0), -phi)
            assert not ratchet._inside(wheel, q)


def test_tooth_force_from_torque(pla):
    d, _, _, pose, _ = pla
    assert d["F"] == pytest.approx(5000 / pose["rc"])


def test_pipeline_ratchet():
    r = pipeline.design_from_params(kind="ratchet", torque_Nm=50, teeth=20, material="steel")
    assert r["status"] == "ok", r["message"]
    assert len(r["fusion"]["table"]) == 360 and r["ratchet"]["backlash"] == pytest.approx(18)
    assert pipeline.design_from_spec({"mechanism_type": "ratchet"})["status"] == "missing"
