"""
H3b: straight-sided spline (ISO 14, light and medium series) joining a hub to a shaft.

Size N x d x D (teeth, minor and major diameter) with the tooth width B from the ISO 14 tables.
Torque carried by the flanks (bearing pressure), only 75 % of the teeth taken as carrying (pitch errors):
    T = 0.75 N h L r_m p_allow,   h = (D - d)/2 - 2 c (c = 0.3 mm chamfers),  r_m = (D + d) / 4
    p_allow (steel on steel): fixed hub 100 MPa, sliding when unloaded 40 MPa, sliding under load 10 MPa
Shaft in torsion on the minor diameter:  16 T / (pi d^3) <= 0.5 S_y / 2 (C45, S_y 430).
Choice: the smallest spline (then the light series) whose hub length L <= 1.5 D carries K T (service factor
1.25).  Units: mm, N, N*mm, MPa.
"""

import math

K_SERVICE = 1.25
CHAMFER = 0.3
SY = 430.0
P_ALLOW = {"fixed": 100.0, "sliding": 40.0, "sliding_loaded": 10.0}
# (series, N, d, D, B)  ISO 14
SIZES = [("medium", 6, 11, 14, 3), ("medium", 6, 13, 16, 3.5), ("medium", 6, 16, 20, 4), ("medium", 6, 18, 22, 5),
         ("medium", 6, 21, 25, 5), ("light", 6, 23, 26, 6), ("medium", 6, 23, 28, 6), ("light", 6, 26, 30, 6),
         ("medium", 6, 26, 32, 6), ("light", 6, 28, 32, 7), ("medium", 6, 28, 34, 7), ("light", 8, 32, 36, 6),
         ("medium", 8, 32, 38, 6), ("light", 8, 36, 40, 7), ("medium", 8, 36, 42, 7), ("light", 8, 42, 46, 8),
         ("medium", 8, 42, 48, 8), ("light", 8, 46, 50, 9), ("medium", 8, 46, 54, 9), ("light", 8, 52, 58, 10),
         ("medium", 8, 52, 60, 10), ("light", 8, 56, 62, 10), ("medium", 8, 56, 65, 10), ("light", 8, 62, 68, 12),
         ("medium", 8, 62, 72, 12), ("light", 10, 72, 78, 12), ("medium", 10, 72, 82, 12)]


def capacity(N, d, D, L, use="fixed"):
    h = (D - d) / 2 - 2 * CHAMFER
    return 0.75 * N * h * L * (D + d) / 4 * P_ALLOW[use]


def design(T, use="fixed", K=K_SERVICE):
    """Smallest spline carrying K T (N*mm) with a hub no longer than 1.5 D. Returns (design, checks)."""
    Td = K * T
    for series, N, d, D, B in SIZES:
        tau = 16 * Td / (math.pi * d ** 3)
        if tau > 0.5 * SY / 2:
            continue
        per_mm = capacity(N, d, D, 1.0, use)
        L = max(math.ceil(Td / per_mm), round(0.8 * d))
        if L > 1.5 * D:
            continue
        cap = capacity(N, d, D, L, use)
        checks = [{"group": "Flanks", "name": f"Flank pressure ({use.replace('_', ' ')})", "util": Td / cap,
                   "detail": f"{cap / 1000:.0f} N·m at p = {P_ALLOW[use]:g} MPa over {L} mm for {Td / 1000:.0f} N·m"},
                  {"group": "Shaft", "name": "Shaft torsion (minor Ø)", "util": tau / (0.5 * SY / 2),
                   "detail": f"τ = {tau:.0f} MPa on Ø{d}"},
                  {"group": "Hub", "name": "Hub length", "util": L / (1.5 * D), "detail": f"L = {L} mm ≤ 1.5 D"}]
        return {"series": series, "N": N, "d": d, "D": D, "B": B, "L": float(L), "T": T, "Td": Td, "cap": cap,
                "use": use, "p": P_ALLOW[use], "tau": tau, "hub_od": round(1.8 * D)}, checks
    return None, [{"group": "Flanks", "name": "Spline size", "util": 9.9,
                   "detail": f"no ISO 14 spline up to 10 x 72 x 82 carries {Td / 1000:.0f} N·m ({use})"}]
