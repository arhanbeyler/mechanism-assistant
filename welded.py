"""
H2: fillet-welded cantilever (Shigley ch. 9): a rectangular bar b x d welded to a wall, a load P at arm L
bends the weld group (Shigley Table 9-2, welds treated as lines).

Throat t = 0.707 h (leg h).  Weld patterns (bending about the horizontal axis):
    top_bottom  A_u = 2b,        I_u = b d^2 / 2
    sides       A_u = 2d,        I_u = d^3 / 6
    all_around  A_u = 2(b + d),  I_u = d^2 (3b + d) / 6
    A = 0.707 h A_u, I = 0.707 h I_u;  primary shear tau' = P / A,  secondary tau'' = M (d/2) / I, M = P L,
    tau = sqrt(tau'^2 + tau''^2)  <=  0.30 S_ut of the electrode (AISC; E70XX: S_ut 482 MPa)
Also: the bar in bending sigma = 6 M / (b d^2) <= 0.6 S_y (AISC), the minimum fillet for the thickness joined
(AISC: t <= 6: 3 mm, <= 12: 5 mm, <= 20: 6 mm, else 8 mm) and h <= the bar's thickness.
Without b, d the smallest flat bar d = 2b that passes in bending is taken.  Units: mm, N, MPa, N*mm.
"""

import math

import strength

LEGS = [3, 4, 5, 6, 8, 10, 12, 14, 16, 20]
ELECTRODES = {"E60": {"Sut": 427.0, "Sy": 345.0}, "E70": {"Sut": 482.0, "Sy": 393.0}}
BARS = [10, 12, 15, 16, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 100]
PATTERNS = {"top_bottom": "top and bottom", "sides": "both sides", "all_around": "all around"}


def unit_props(pattern, b, d):
    if pattern == "top_bottom":
        return 2 * b, b * d * d / 2
    if pattern == "sides":
        return 2 * d, d ** 3 / 6
    return 2 * (b + d), d * d * (3 * b + d) / 6


def min_leg(t):
    return 3 if t <= 6 else 5 if t <= 12 else 6 if t <= 20 else 8


def weld_stress(P, L, b, d, h, pattern):
    Au, Iu = unit_props(pattern, b, d)
    A, I = 0.707 * h * Au, 0.707 * h * Iu
    t1 = P / A
    t2 = P * L * (d / 2) / I
    return {"A": A, "I": I, "tau1": t1, "tau2": t2, "tau": math.hypot(t1, t2)}


def bar_for(P, L, bar_mat):
    """Smallest flat bar b x 2b that carries M = P L in bending (0.6 S_y)."""
    M = P * L
    for b in BARS:
        d = 2 * b
        if 6 * M / (b * d * d) <= 0.6 * bar_mat["sy"]:
            return float(b), float(d)
    return None, None


def design(P, L, b=None, d=None, pattern="all_around", electrode="E70", bar_key="steel"):
    """Smallest fillet leg h that passes. Returns (design, checks)."""
    bar_mat = strength.MATERIALS[bar_key]
    if not (b and d):
        b, d = bar_for(P, L, bar_mat)
        if b is None:
            return None, [{"group": "Bar", "name": "Bar in bending", "util": 9.9, "detail": f"no flat bar up to {BARS[-1]} mm"}]
    tau_all = 0.30 * ELECTRODES[electrode]["Sut"]
    sigma_bar = 6 * P * L / (b * d * d)
    h_min = min_leg(min(b, d))
    last = None
    for h in LEGS:
        if h < h_min:
            continue
        s = weld_stress(P, L, b, d, h, pattern)
        checks = [
            {"group": "Weld", "name": f"Weld throat shear ({PATTERNS[pattern]})", "util": s["tau"] / tau_all,
             "detail": f"τ = √(τ'² + τ''²) = √({s['tau1']:.1f}² + {s['tau2']:.1f}²) = {s['tau']:.1f} MPa ≤ 0.30 S_ut = {tau_all:.0f} MPa ({electrode})"},
            {"group": "Weld", "name": "Leg within the bar thickness", "util": h / min(b, d),
             "detail": f"h = {h} mm, AISC minimum {h_min} mm for {min(b, d):g} mm"},
            {"group": "Bar", "name": "Bar in bending at the wall", "util": sigma_bar / (0.6 * bar_mat["sy"]),
             "detail": f"σ = {sigma_bar:.1f} MPa ≤ 0.6 S_y = {0.6 * bar_mat['sy']:.0f} MPa ({bar_mat['name']})"},
        ]
        last = checks
        if max(c["util"] for c in checks) <= 1.0:
            return dict(s, h=float(h), throat=0.707 * h, b=b, d=d, L=L, P=P, pattern=pattern, electrode=electrode,
                        tau_all=tau_all, sigma_bar=sigma_bar, weld_length=unit_props(pattern, b, d)[0],
                        bar=bar_mat["name"]), checks
    return None, last or []
