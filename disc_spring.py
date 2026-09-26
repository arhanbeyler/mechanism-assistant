"""
H4b: disc (Belleville) spring stack, DIN 2092 formulas, DIN 2093 series proportions.

One disc: outside D_e, inside D_i (delta = D_e / D_i), thickness t, free cone height h0, deflection s:
    K1 = (1/pi) ((delta-1)/delta)^2 / ((delta+1)/(delta-1) - 2/ln delta)
    F  = 4E/(1-nu^2) t^4/(K1 D_e^2) (s/t) [(h0/t - s/t)(h0/t - s/(2t)) + 1]
    sigma_OM = -4E/(1-nu^2) t^2/(K1 D_e^2) (s/t) (3/pi)          (top inner edge: the static design point)
Series (DIN 2093 proportions): A  D_e/t 18, h0/t 0.4;  B  28, 0.75;  C  40, 1.3  (thickness rounded to a stock
size); D_i from the DIN 2093 size list.  Working deflection at most 0.75 h0; |sigma_OM| <= 1400 MPa (spring steel
51CrV4, static); E 206 GPa.  Stack: n_p discs nested (same direction: forces add) and n_s in series (alternating:
deflections add).  Choice: the lightest stack (n_s n_p D_e^2 t) whose deflection at the force is within 10 % of
the asked one.  Units: mm, N, MPa.
"""

import math

E, NU = 206000.0, 0.3
SIGMA_OM = 1400.0
S_MAX = 0.75
SIZES = [(8, 4.2), (10, 5.2), (12.5, 6.2), (14, 7.2), (16, 8.2), (18, 9.2), (20, 10.2), (22.5, 11.2), (25, 12.2),
         (28, 14.2), (31.5, 16.3), (35.5, 18.3), (40, 20.4), (45, 22.4), (50, 25.4), (56, 28.5), (63, 31.0),
         (71, 36.0), (80, 41.0), (90, 46.0), (100, 51.0), (112, 57.0), (125, 64.0)]
SERIES = {"A": (18.0, 0.4), "B": (28.0, 0.75), "C": (40.0, 1.3)}
THICK = [0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5,
         3.0, 3.5, 4.0, 5.0, 6.0, 7.0, 8.0]


def k1(De, Di):
    d = De / Di
    L = math.log(d)
    return (1 / math.pi) * ((d - 1) / d) ** 2 / ((d + 1) / (d - 1) - 2 / L)


def force(De, Di, t, h0, s):
    return 4 * E / (1 - NU * NU) * t ** 4 / (k1(De, Di) * De * De) * (s / t) * ((h0 / t - s / t) * (h0 / t - s / (2 * t)) + 1)


def sigma_om(De, Di, t, s):
    return -4 * E / (1 - NU * NU) * t * t / (k1(De, Di) * De * De) * (s / t) * 3 / math.pi


def deflection_at(De, Di, t, h0, F):
    """Smallest s with force(s) = F (bisection on the rising branch), None beyond 0.75 h0."""
    hi = S_MAX * h0
    if force(De, Di, t, h0, hi) < F:
        return None
    lo = 0.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if force(De, Di, t, h0, mid) < F else (lo, mid)
    return (lo + hi) / 2


def design(F, y, rod=None):
    """Lightest disc stack giving deflection y (mm) at force F (N). rod: guide rod diameter (D_i must clear it)."""
    best = None
    for De, Di in SIZES:
        if rod and Di < rod + 0.2:
            continue
        for series, (ratio, h0t) in SERIES.items():
            t = min(THICK, key=lambda x: abs(x - De / ratio))
            h0 = round(h0t * t, 2)
            for np_ in (1, 2, 3):
                s = deflection_at(De, Di, t, h0, F / np_)
                if s is None or s <= 0:
                    continue
                ns = max(1, round(y / s))
                if abs(ns * s - y) > 0.10 * y or ns > 40:
                    continue
                sig = sigma_om(De, Di, t, s)
                if abs(sig) > SIGMA_OM:
                    continue
                mass = ns * np_ * De * De * t
                if best is None or mass < best["mass"]:
                    best = {"De": De, "Di": Di, "t": t, "h0": h0, "series": series, "np": np_, "ns": ns, "s": s,
                            "y": ns * s, "F": F, "sigma_om": sig, "mass": mass,
                            "L0": ns * (h0 + np_ * t), "L_loaded": ns * (h0 + np_ * t) - ns * s,
                            "F_75": np_ * force(De, Di, t, h0, S_MAX * h0), "K1": k1(De, Di)}
    if best is None:
        return None, [{"group": "Stack", "name": "Disc spring stack", "util": 9.9,
                       "detail": f"no DIN 2093-like disc stack gives {y:g} mm at {F:g} N"}]
    b = best
    checks = [{"group": "Stress", "name": "σ_OM at the working deflection (static)", "util": abs(b["sigma_om"]) / SIGMA_OM,
               "detail": f"{b['sigma_om']:.0f} MPa, limit {SIGMA_OM:g} (51CrV4)"},
              {"group": "Deflection", "name": "Deflection per disc within 0.75 h0", "util": b["s"] / (S_MAX * b["h0"]),
               "detail": f"s = {b['s']:.3f} of {S_MAX * b['h0']:.3f} mm"},
              {"group": "Deflection", "name": "Stack deflection at the force", "util": abs(b["y"] - y) / (0.10 * y),
               "detail": f"{b['y']:.2f} mm (asked {y:g}, within 10 %)"}]
    return b, checks
