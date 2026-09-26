"""F3: external Geneva mechanism."""

import math

import pytest

import geneva
import pipeline
import strength


@pytest.mark.parametrize("n", [4, 6, 8])
def test_tangential_entry_and_exit(n):
    g = geneva.geometry(n, 100, 5)
    assert g["a"] == pytest.approx(100 * math.sin(math.pi / n))
    m = geneva.motion(g)
    al = math.degrees(g["alpha"])
    moving = [k for k in m if k[0] <= al + 1e-9]
    assert abs(moving[0][2]) < 1e-9 and abs(moving[-1][2]) < 0.02      # zero wheel speed at entry / exit
    assert abs(moving[len(moving) // 2][2]) > 0.5
    assert m[0][1] == pytest.approx(0, abs=1e-9)
    assert m[-1][1] == pytest.approx(-2 * math.pi / n)                  # one step per driver turn
    assert 2 * math.degrees(g["alpha"]) == pytest.approx(180 - 360 / n)


def test_closed_forms_match_numerical_derivatives():
    g = geneva.geometry(6, 100, 5)
    h = 1e-5
    for th in (-0.9, -0.4, 0.2, 0.7):
        w = (geneva.wheel_angle(g, th + h) - geneva.wheel_angle(g, th - h)) / (2 * h)
        acc = (geneva.wheel_angle(g, th + h) - 2 * geneva.wheel_angle(g, th) + geneva.wheel_angle(g, th - h)) / h ** 2
        lam = g["a"] / g["C"]
        den = 1 - 2 * lam * math.cos(th) + lam * lam
        assert -lam * (math.cos(th) - lam) / den == pytest.approx(w, rel=1e-6)
        assert -lam * (lam * lam - 1) * math.sin(th) / den ** 2 == pytest.approx(acc, rel=1e-3)


@pytest.mark.parametrize("n,ratio", [(4, math.sqrt(2) / 2 / (1 - math.sqrt(2) / 2)), (6, 1.0)])
def test_peak_wheel_speed(n, ratio):
    d, _ = geneva.design(n, 100, strength.MATERIALS["steel"])
    assert d["w_ratio_max"] == pytest.approx(ratio, rel=1e-3)             # lambda / (1 - lambda) at theta = 0


def test_pin_force_times_radius_is_wheel_torque():
    g = geneva.geometry(4, 80, 5)
    ld = geneva.loads(g, 0.01, 60)
    om = 2 * math.pi
    peak = max(abs(k[3]) for k in geneva.motion(g)) * om * om * 0.01 * 1000
    assert ld["T_wheel"] == pytest.approx(peak)
    # the pin sits between r = C - a (mid-index) and r = R (entry) from the wheel centre
    assert ld["T_wheel"] / g["R"] <= ld["F_pin"] <= ld["T_wheel"] / (g["C"] - g["a"])


def test_no_interference_during_the_index():
    """Point sampling: the locking disk (with its relief) and the pin never overlap the wheel while it turns,
    and without the relief they would."""
    g = geneva.geometry(6, 100, 5)
    g["relief_r"] = geneva.relief_radius(g)
    C, h = g["C"], 1.0
    pts = [(x * h, y * h) for x in range(int((C - g["R_o"]) / h), int(g["a"] / h) + 4)
           for y in range(-int(g["a"] / h) - 3, int(g["a"] / h) + 4)]
    al = math.degrees(g["alpha"])
    hits = no_relief = 0
    for th_deg in [x * 3.0 for x in range(int(-al / 3) - 3, int(al / 3) + 4)]:
        th = math.radians(th_deg)
        phi = geneva.wheel_angle(g, max(-g["alpha"], min(g["alpha"], th)))
        c, s = math.cos(-phi), math.sin(-phi)
        rc = (C * math.cos(th), C * math.sin(th))
        pin = (g["a"] * math.cos(th), g["a"] * math.sin(th))
        for x, y in pts:
            disk = math.hypot(x, y) <= g["r_lock"]
            pinp = math.hypot(x - pin[0], y - pin[1]) <= g["d_pin"] / 2
            if not (disk or pinp):
                continue
            q = (c * (x - C) - s * y, s * (x - C) + c * y)
            if geneva.in_wheel(g, q):
                relieved = math.hypot(x - rc[0], y - rc[1]) < g["relief_r"]
                hits += pinp or (disk and not relieved)
                no_relief += disk
    assert hits == 0
    assert no_relief > 100


def test_lock_engages_right_after_the_index():
    for n in (4, 6, 8):
        free, delta, zeta = geneva.lock_free_angle(geneva.geometry(n, 100, 5))
        assert free < geneva.FREE_MAX and zeta > 20


def test_pipeline_geneva():
    r = pipeline.design_from_params(kind="geneva", slots=4, center_distance_mm=80, speed_rpm=60, load_mass_kg=2,
                                    load_diameter_mm=300, material="PETG")
    assert r["status"] == "ok", r["message"]
    t = r["fusion"]["table"]
    assert len(t) == 360 and t[0] == 0 and t[-1] == -90
    assert all(b <= a + 1e-9 for a, b in zip(t, t[1:]))                  # the wheel never turns back
    assert r["geneva"]["F_pin"] > 0


def test_pipeline_geneva_step_and_missing():
    assert pipeline.design_from_spec({"mechanism_type": "geneva", "slots": 6})["status"] == "missing"
    r = pipeline.design_from_spec({"mechanism_type": "geneva", "step_deg": 45, "center_distance_mm": 120})
    assert r["status"] == "ok" and r["geneva"]["n"] == 8
    assert pipeline.design_from_spec({"mechanism_type": "geneva", "step_deg": 50,
                                      "center_distance_mm": 120})["status"] == "missing"


@pytest.mark.parametrize("n,d", [(4, 8), (6, 8), (8, 5)])
def test_relief_clears_the_swept_outline(n, d):
    """Finer than the sizing sweep (0.25 deg): every wheel outline point inside the disk radius is inside the
    relief. The plain rim-radius relief does not manage this (it is 0.1..0.5 mm short)."""
    g = geneva.geometry(n, 100, d)
    rr = geneva.relief_radius(g)
    assert rr > g["relief_r"]                                          # more than rim + clearance
    pts = geneva._outline(g, 0.25)
    worst = -1.0
    for i in range(int(2 * math.degrees(g["alpha"]) / 0.25) + 1):
        th = -g["alpha"] + math.radians(0.25 * i)
        phi = geneva.wheel_angle(g, th)
        c, s = math.cos(phi), math.sin(phi)
        for q in pts:
            p = (g["C"] + c * q[0] - s * q[1], s * q[0] + c * q[1])
            if math.hypot(*p) < g["r_lock"]:
                worst = max(worst, math.hypot(p[0] - g["C"] * math.cos(th), p[1] - g["C"] * math.sin(th)) - rr)
    assert worst < 0
