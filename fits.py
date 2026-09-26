"""
H3: ISO 286 hole-basis fits and press (shrink) fits, Shigley 3-16 / 7-8.

Limits (micrometres) for the H7 hole and the shafts g6, h6, k6, n6, p6, s6 (ISO 286 tables, Shigley A-11 /
A-12), sizes 3..120 mm.  Clearance / interference: hole limits minus shaft limits.
Press fit (thick-walled cylinders, Lame; radial interference delta = diametral / 2):
    p = delta / { R/E_o [(r_o^2 + R^2)/(r_o^2 - R^2) + nu_o] + R/E_i [(R^2 + r_i^2)/(R^2 - r_i^2) - nu_i] }
    (solid shaft r_i = 0).  Torque capacity T = mu p (2 pi R L) R with the SMALLEST interference;
    stresses with the LARGEST: hub bore tangential p (r_o^2 + R^2)/(r_o^2 - R^2), radial -p, von Mises <= S_y / n;
    shaft (solid) -p both ways.  Assembly by heating the hub: dT = (delta_max_diametral + clearance 0.001 d) / (alpha d).
Choice: the loosest fit (k6 -> n6 -> p6 -> s6), then the shortest (1..2 d) and slimmest (2..2.5 d) hub, whose
minimum interference carries the torque x 2 and whose maximum interference keeps the hub below yield / 1.25
(C45 quenched and tempered by default).  Units: mm, N, MPa, N*mm (limits in micrometres).
"""

import math

import strength

# upper size of the range: (H7 upper), shafts {fit: (es, ei)}   (H7 lower = 0)
TABLE = [
    (6, 12, {"g6": (-4, -12), "h6": (0, -8), "k6": (9, 1), "n6": (16, 8), "p6": (20, 12), "s6": (27, 19)}),
    (10, 15, {"g6": (-5, -14), "h6": (0, -9), "k6": (10, 1), "n6": (19, 10), "p6": (24, 15), "s6": (32, 23)}),
    (18, 18, {"g6": (-6, -17), "h6": (0, -11), "k6": (12, 1), "n6": (23, 12), "p6": (29, 18), "s6": (39, 28)}),
    (30, 21, {"g6": (-7, -20), "h6": (0, -13), "k6": (15, 2), "n6": (28, 15), "p6": (35, 22), "s6": (48, 35)}),
    (50, 25, {"g6": (-9, -25), "h6": (0, -16), "k6": (18, 2), "n6": (33, 17), "p6": (42, 26), "s6": (59, 43)}),
    (65, 30, {"g6": (-10, -29), "h6": (0, -19), "k6": (21, 2), "n6": (39, 20), "p6": (51, 32), "s6": (72, 53)}),
    (80, 30, {"g6": (-10, -29), "h6": (0, -19), "k6": (21, 2), "n6": (39, 20), "p6": (51, 32), "s6": (78, 59)}),
    (100, 35, {"g6": (-12, -34), "h6": (0, -22), "k6": (25, 3), "n6": (45, 23), "p6": (59, 37), "s6": (93, 71)}),
    (120, 35, {"g6": (-12, -34), "h6": (0, -22), "k6": (25, 3), "n6": (45, 23), "p6": (59, 37), "s6": (101, 79)}),
]
PRESS_FITS = ("k6", "n6", "p6", "s6")
DESCRIPTION = {"g6": "sliding (clearance)", "h6": "location (clearance)", "k6": "transition", "n6": "light press",
               "p6": "press", "s6": "heavy press / shrink"}
MU = 0.12                          # steel on steel, press fit (dry, conservative)
ALPHA = 11.7e-6                    # steel, 1/K
TORQUE_SF = 2.0
HUB_SF = 1.25
# press-fit steels (hub and shaft): E, nu, S_y
STEELS = {"C45": {"name": "C45 quenched and tempered", "E": 207000.0, "nu": 0.30, "sy": 430.0},
          "steel": dict(strength.MATERIALS["steel"])}


def limits(d, fit):
    """(hole lo, hole hi, shaft lo, shaft hi) in micrometres for H7/<fit> at nominal size d."""
    if not 3 < d <= TABLE[-1][0]:
        raise ValueError(f"sizes 3..{TABLE[-1][0]} mm")
    upper, h7, shafts = next(row for row in TABLE if d <= row[0])
    es, ei = shafts[fit]
    return 0, h7, ei, es


def fit_range(d, fit):
    """Minimum and maximum diametral interference (um); negative = clearance."""
    hl, hh, sl, sh = limits(d, fit)
    return sl - hh, sh - hl


def pressure(delta_rad, R, ro, E_o, nu_o, E_i, nu_i, ri=0.0):
    hub = R / E_o * ((ro ** 2 + R ** 2) / (ro ** 2 - R ** 2) + nu_o)
    shaft = R / E_i * ((R ** 2 + ri ** 2) / (R ** 2 - ri ** 2) - nu_i)
    return delta_rad / (hub + shaft)


def hub_von_mises(p, R, ro):
    st = p * (ro ** 2 + R ** 2) / (ro ** 2 - R ** 2)
    sr = -p
    return math.sqrt(st * st - st * sr + sr * sr), st


def analyse(d, fit, D_hub, L, T=None, hub_key="C45", shaft_key="C45", mu=MU):
    hub, shaft = STEELS[hub_key], STEELS[shaft_key]
    imin, imax = fit_range(d, fit)
    R, ro = d / 2, D_hub / 2
    out = {"d": d, "fit": fit, "i_min": imin, "i_max": imax, "limits": limits(d, fit), "D_hub": D_hub, "L": L, "mu": mu}
    for key, i_um in (("min", imin), ("max", imax)):
        delta = max(i_um, 0) * 1e-3 / 2
        p = pressure(delta, R, ro, hub["E"], hub["nu"], shaft["E"], shaft["nu"])
        vm, st = hub_von_mises(p, R, ro)
        out[f"p_{key}"] = p
        out[f"T_{key}"] = mu * p * 2 * math.pi * R * L * R
        out[f"hub_vm_{key}"] = vm
        out[f"hub_st_{key}"] = st
    out["dT_assembly"] = (max(imax, 0) * 1e-3 + 0.001 * d) / (ALPHA * d)
    out["hub_sy"], out["shaft_sy"] = hub["sy"], shaft["sy"]
    return out


def checks_for(a, T):
    rows = []
    if T:
        rows.append({"group": "Fit", "name": f"Torque carried at the smallest interference (×{TORQUE_SF:g})",
                     "util": TORQUE_SF * T / a["T_min"] if a["T_min"] > 0 else 9.9,
                     "detail": f"{a['T_min'] / 1000:.1f} N·m (p = {a['p_min']:.1f} MPa) for {T / 1000:.1f} N·m"})
    rows += [{"group": "Hub", "name": f"Hub bore at the largest interference (÷{HUB_SF:g})",
              "util": HUB_SF * a["hub_vm_max"] / a["hub_sy"],
              "detail": f"σ_vM = {a['hub_vm_max']:.0f} MPa (σ_t = {a['hub_st_max']:.0f}, p = {a['p_max']:.0f} MPa), S_y {a['hub_sy']:.0f}"},
             {"group": "Shaft", "name": "Shaft surface pressure", "util": HUB_SF * a["p_max"] / a["shaft_sy"],
              "detail": f"-{a['p_max']:.0f} MPa both ways"}]
    return rows


def design(T, d, D_hub=None, L=None, hub_key="C45", shaft_key="C45"):
    """Loosest press fit (then the shortest, slimmest hub) that carries T (N*mm) on a d mm shaft.
    Returns (design, checks)."""
    last = None
    for fit in PRESS_FITS:
        for Lk in ((L,) if L else (1.0, 1.5, 2.0)):
            for Dk in ((D_hub,) if D_hub else (2.0, 2.5)):
                Lh = Lk if L else Lk * d
                Dh = Dk if D_hub else Dk * d
                a = analyse(d, fit, Dh, Lh, T, hub_key, shaft_key)
                checks = checks_for(a, T)
                if last is None or max(c["util"] for c in checks) < max(c["util"] for c in last):
                    last = checks
                if max(c["util"] for c in checks) <= 1.0:
                    return dict(a, T=T, description=DESCRIPTION[fit], hub=STEELS[hub_key]["name"]), checks
    return None, last
