"""
H4c: helical extension spring with full-loop hooks (Shigley 10-10), music wire.

Close-wound body (N_b coils), initial tension F_i: the spring only extends past F_i.
    rate  k = d^4 G / (8 D^3 N_a),  N_a = N_b + G / E   (the hooks' share)
    force at the extension y:  F = F_i + k y
    initial tension from the preferred stress range (Shigley 10-10): tau_i = 231 exp(-0.105 C) +- 6.9 (4 - (C-3)/6.5)
        MPa, uncorrected tau = 8 F_i D / (pi d^3); the middle of the range is taken
    body torsion   tau = K_B 8 F D / (pi d^3)                                    <= 0.45 S_ut / n
    hook bending at A (the loop's inside, C1 = C):  sigma_A = F [K_A 16 D/(pi d^3) + 4/(pi d^2)],
        K_A = (4 C1^2 - C1 - 1) / (4 C1 (C1 - 1))                                <= 0.75 S_ut / n
    hook torsion at B (the bend into the loop, r2 = 2 d, C2 = 4):  tau_B = K_B2 8 F D/(pi d^3),
        K_B2 = (4 C2 - 1) / (4 C2 - 4)                                           <= 0.40 S_ut / n
n = 1.2 (static; Shigley Table 10-7 fractions).  Choice: the lightest (d^2 D N_b).  Units: mm, N, MPa.
"""

import math

import springs

E_WIRE = 196500.0
N_S = 1.2
C_RANGE = [c / 2 for c in range(8, 25)]
C2 = 4.0


def tau_i_range(C):
    mid = 231 * math.exp(-0.105 * C)
    half = 6.9 * (4 - (C - 3) / 6.5)
    return mid - half, mid + half


def stresses(d, D, F):
    C = D / d
    KB = springs.bergstrasser(C)
    KA = (4 * C * C - C - 1) / (4 * C * (C - 1))
    KB2 = (4 * C2 - 1) / (4 * C2 - 4)
    return {"tau_body": KB * 8 * F * D / (math.pi * d ** 3),
            "sigma_A": F * (KA * 16 * D / (math.pi * d ** 3) + 4 / (math.pi * d * d)),
            "tau_B": KB2 * 8 * F * D / (math.pi * d ** 3)}


def design(F, y, mat_key="music_wire"):
    """Lightest extension spring reaching F (N) after extending y (mm) past its free length."""
    mat = springs.SPRING_MATERIALS[mat_key]
    G = mat["G"]
    best = best_any = None
    for d in springs.WIRES:
        Sut = mat["A"] / d ** mat["m"]
        for C in C_RANGE:
            D = C * d
            lo, hi = tau_i_range(C)
            tau_i = (lo + hi) / 2
            Fi = tau_i * math.pi * d ** 3 / (8 * D)
            if Fi >= F:
                continue
            k = (F - Fi) / y
            Na = d ** 4 * G / (8 * D ** 3 * k)
            Nb = round((Na - G / E_WIRE) * 4) / 4
            if Nb < 3:
                continue
            Na = Nb + G / E_WIRE
            k = d ** 4 * G / (8 * D ** 3 * Na)
            Fy = Fi + k * y
            st = stresses(d, D, Fy)
            checks = [
                {"group": "Stress", "name": "Body torsion", "util": N_S * st["tau_body"] / (0.45 * Sut),
                 "detail": f"τ = {st['tau_body']:.0f} MPa, 0.45 S_ut = {0.45 * Sut:.0f}"},
                {"group": "Stress", "name": "Hook bending (A)", "util": N_S * st["sigma_A"] / (0.75 * Sut),
                 "detail": f"σ_A = {st['sigma_A']:.0f} MPa, 0.75 S_ut = {0.75 * Sut:.0f}"},
                {"group": "Stress", "name": "Hook torsion (B)", "util": N_S * st["tau_B"] / (0.40 * Sut),
                 "detail": f"τ_B = {st['tau_B']:.0f} MPa, 0.40 S_ut = {0.40 * Sut:.0f}"},
                {"group": "Geometry", "name": "Spring index", "util": max(4 / C, C / 12), "detail": f"C = {C:.1f}"},
            ]
            worst = max(c["util"] for c in checks)
            L0 = (Nb + 1) * d + 2 * (D - d)                       # body + two full loops (inside length)
            info = {"d": d, "D": D, "C": C, "Nb": Nb, "Na": Na, "k": k, "Fi": Fi, "tau_i": tau_i, "F": Fy, "y": y,
                    "L0": L0, "L_max": L0 + y, "OD": D + d, "Sut": Sut, "material": mat["name"], **st,
                    "mass": d * d * D * (Nb + 2)}
            if worst <= 1.0:
                if best is None or info["mass"] < best[0]["mass"]:
                    best = (info, checks)
            elif best_any is None or worst < best_any[1]:
                best_any = (checks, worst)
    if best:
        return best
    return None, (best_any[0] if best_any else [])
