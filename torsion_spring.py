"""
H4: helical torsion spring (Shigley 10-12), straight legs, closed coils, loaded to close the coils.

Request: the moment M (N*mm) at the working angle theta (deg) -> rate k' = M / theta (per turn).
    stress (inner fibre)  sigma = K_i 32 M / (pi d^3),  K_i = (4C^2 - C - 1) / (4C (C - 1)),  C = D / d
    allowed               S_y = f S_ut (music wire 0.78, stainless 302 0.61; Shigley Table 10-6), S_ut = A / d^m
    rate                  k' = d^4 E / (10.8 D N_a)  per turn,  N_a = N_b + (l1 + l2) / (3 pi D)  (the legs)
    winding down          D' = D N_b / (N_b + theta_turns): the inside diameter D' - d must still clear the arbor
    body length           L_b = d (N_b + 1)
Checks: 4 <= C <= 12, N_b >= 3, sigma <= S_y / 1.2, arbor clearance (10 % of the arbor) if an arbor is given.
Choice: the lightest spring (d^2 D N_b).  Units: mm, N, N*mm, MPa.
"""

import math

import springs

E = {"music_wire": 196500.0, "stainless": 193000.0}          # Shigley Table 10-5
BEND_FRAC = {"music_wire": 0.78, "stainless": 0.61}
N_STATIC = 1.2
C_RANGE = [c / 2 for c in range(8, 25)]                         # 4.0 .. 12.0


def k_i(C):
    return (4 * C * C - C - 1) / (4 * C * (C - 1))


def active_turns(Nb, D, l1, l2):
    return Nb + (l1 + l2) / (3 * math.pi * D)


def rate_per_turn(d, D, Na, Emod):
    return d ** 4 * Emod / (10.8 * D * Na)


def evaluate(d, D, Nb, M, mat_key, legs, arbor=None):
    mat = springs.SPRING_MATERIALS[mat_key]
    Emod = E[mat_key]
    C = D / d
    Na = active_turns(Nb, D, legs, legs)
    k = rate_per_turn(d, D, Na, Emod)
    turns = M / k
    Sut = mat["A"] / d ** mat["m"]
    Sy = BEND_FRAC[mat_key] * Sut
    sigma = k_i(C) * 32 * M / (math.pi * d ** 3)
    D_wound = D * Nb / (Nb + turns)
    ID = D_wound - d
    checks = [
        {"group": "Geometry", "name": "Spring index C = D/d", "util": max(4 / C, C / 12), "detail": f"C = {C:.1f} (4–12)"},
        {"group": "Geometry", "name": "Body turns", "util": 3 / Nb, "detail": f"N_b = {Nb:g} (at least 3)"},
        {"group": "Strength", "name": "Bending at the inner fibre", "util": N_STATIC * sigma / Sy,
         "detail": f"σ = K_i 32M/(πd³) = {sigma:.0f} MPa, S_y = {Sy:.0f} MPa (K_i {k_i(C):.3f}, n ≥ {N_STATIC:g})"},
    ]
    if arbor:
        checks.append({"group": "Geometry", "name": "Clears the arbor when wound", "util": 1.1 * arbor / ID,
                       "detail": f"inside Ø {D - d:.1f} → {ID:.1f} mm over a Ø{arbor:g} arbor"})
    info = {"d": d, "D": D, "C": C, "Nb": Nb, "Na": Na, "k_turn": k, "k_deg": k / 360, "turns": turns,
            "theta_deg": 360 * turns, "sigma": sigma, "Sy": Sy, "Sut": Sut, "D_wound": D_wound, "ID_wound": ID,
            "OD": D + d, "Lb": d * (Nb + 1), "legs": legs, "material": mat["name"], "M": M, "arbor": arbor}
    return info, checks


def design(M, theta_deg, mat_key="music_wire", arbor=None, legs=None):
    """Lightest torsion spring giving M (N*mm) at theta_deg. Returns (design, checks)."""
    best = best_any = None
    k_target = M / (theta_deg / 360.0)
    for d in springs.WIRES:
        for C in C_RANGE:
            D = C * d
            l = legs or max(2 * D, 10.0)
            Na = d ** 4 * E[mat_key] / (10.8 * D * k_target)
            Nb = round((Na - 2 * l / (3 * math.pi * D)) * 4) / 4           # whole quarter turns (leg angle)
            if Nb < 1:
                continue
            info, checks = evaluate(d, D, Nb, M, mat_key, l, arbor)
            worst = max(c["util"] for c in checks)
            mass = d * d * D * Nb
            if worst <= 1.0:
                if best is None or mass < best[0]["mass"]:
                    best = (dict(info, mass=mass), checks)
            elif best_any is None or worst < best_any[1]:
                best_any = (checks, worst)
    if best:
        return best
    return None, (best_any[0] if best_any else [])
