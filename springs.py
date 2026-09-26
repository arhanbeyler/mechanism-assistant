"""
C10: Compression spring module: cylindrical (helical) and conical. Shigley, Chapter 10.

Request: max force F (N) and the deflection y (mm) at that force -> spring rate k = F / y.
Wire: steel spring wire (plastic springs are not printed; music wire if no material is given).

Cylindrical: k = d^4 G / (8 D^3 Na)
Conical (radius R1 -> R2 linear):  y = 64 F / (G d^4) * integral(R^3 dn) = 16 F Na (R1+R2)(R1^2+R2^2) / (G d^4)
             -> k = G d^4 / (16 Na (R1+R2)(R1^2+R2^2))     (linear range: until the largest coil bottoms out)
Stress:      tau = K_B 8 F D / (pi d^3),  K_B = (4C + 2) / (4C - 3)  (Bergstrasser), C = D / d, D = largest coil
Wire strength: S_ut = A / d^m,  S_sy = frac * S_ut  (Shigley Tables 10-4, 10-6: set not removed)
Ends: squared and ground -> Nt = Na + 2, Ls = d Nt, L0 = Ls + (1 + xi) y (xi = 0.15), p = (L0 - 2d) / Na
Checks: 4 <= C <= 12, 3 <= Na <= 15, working safety n >= 1.2, no yield when compressed SOLID (n_s >= 1),
        buckling (cylindrical, both ends fixed, alpha = 0.5): L0 <= 2.63 D / alpha
Choice: the LIGHTEST passing spring (mass ~ d^2 D_mean Nt).
"""

import math

SPRING_MATERIALS = {
    # A [MPa mm^m], m, G [MPa], S_sy / S_ut (static, set not removed)
    "music_wire": {"name": "Music wire (ASTM A228)", "A": 2211.0, "m": 0.145, "G": 81000.0, "frac": 0.45},
    "stainless": {"name": "Stainless 302 (A313)", "A": 1867.0, "m": 0.146, "G": 69000.0, "frac": 0.35},
}
WIRES = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.5, 2.8, 3.0, 3.2, 3.5,
         4.0, 4.5, 5.0, 5.5, 6.0]
XI = 0.15            # clash allowance
N_STATIC = 1.2       # working safety factor
ALPHA = 0.5          # buckling: both ends fixed (between parallel plates)


def bergstrasser(C):
    return (4 * C + 2) / (4 * C - 3)


def rate(d, G, Na, D1, taper=1.0):
    """Spring rate (N/mm). taper = D2 / D1 (1: cylindrical)."""
    R1, R2 = D1 / 2, taper * D1 / 2
    return G * d ** 4 / (16 * Na * (R1 + R2) * (R1 ** 2 + R2 ** 2))


def check(d, D1, Na, F, y, mat, taper=1.0, od_max=None):
    """Checks and geometry of one spring. Returns (checks, geometry)."""
    G = mat["G"]
    C = D1 / d
    C2 = taper * D1 / d
    k = rate(d, G, Na, D1, taper)
    Sut = mat["A"] / d ** mat["m"]
    Ssy = mat["frac"] * Sut
    KB = bergstrasser(C)
    tau = KB * 8 * F * D1 / (math.pi * d ** 3)
    Nt = Na + 2
    Ls = d * Nt
    y_work = F / k                          # real deflection with the rounded Na
    L0 = Ls + (1 + XI) * y_work
    F_s = k * (L0 - Ls)
    tau_s = KB * 8 * F_s * D1 / (math.pi * d ** 3)
    checks = [
        {"group": "Geometry", "name": "Spring index C = D/d", "util": max(4 / C, C / 12, (3 / C2) if taper < 1 else 0),
         "detail": f"C = {C:.1f}" + (f", {C2:.1f} at the small end" if taper < 1 else "") + " (4–12)"},
        {"group": "Geometry", "name": "Active coils", "util": max(3 / Na, Na / 15), "detail": f"Na = {Na:g} (3–15)"},
        {"group": "Stress", "name": "Working stress", "util": N_STATIC * tau / Ssy,
         "detail": f"τ = {tau:.0f} MPa, S_sy = {Ssy:.0f} MPa, n = {Ssy / tau:.2f} (at least {N_STATIC})"},
        {"group": "Stress", "name": "Yield when solid", "util": tau_s / Ssy,
         "detail": f"τ_s = {tau_s:.0f} MPa ≤ S_sy"},
        {"group": "Force", "name": "Target force / deflection", "util": 0.0 if abs(y_work - y) / y <= 0.05 else 2.0,
         "detail": f"deflection {y_work:.2f} mm at F = {F:g} N (target {y:g} mm)"},
    ]
    if taper >= 1.0:
        L_cr = 2.63 * D1 / ALPHA
        checks.append({"group": "Buckling", "name": "Buckling", "util": L0 / L_cr,
                       "detail": f"L0 = {L0:.1f} mm ≤ {L_cr:.1f} mm"})
    if od_max:
        checks.append({"group": "Geometry", "name": "Outer diameter limit", "util": (D1 + d) / od_max,
                       "detail": f"Ø{D1 + d:.1f} ≤ Ø{od_max:g} mm"})
    geo = {"d": d, "D1": D1, "D2": taper * D1, "Na": Na, "Nt": Nt, "L0": L0, "Ls": Ls, "k": k,
           "pitch": (L0 - 2 * d) / Na, "y_work": y_work, "taper": taper, "OD": D1 + d,
           "mass_index": d * d * D1 * (1 + taper) / 2 * Nt}
    return checks, geo


def design(F, y, mat, taper=1.0, od_max=None):
    """Lightest valid spring. Returns (geometry, checks) or (None, checks of the best candidate)."""
    k_target = F / y
    best = best_any = None
    for d in WIRES:
        for C10 in range(40, 121, 5):                    # C = 4.0 ... 12.0
            D1 = C10 / 10 * d
            Na_exact = mat["G"] * d ** 4 / (16 * k_target * (D1 / 2) * (1 + taper) * (D1 / 2) ** 2 * (1 + taper ** 2))
            Na = max(0.25, round(Na_exact * 4) / 4)
            checks, geo = check(d, D1, Na, F, y, mat, taper, od_max)
            worst = max(c["util"] for c in checks)
            if worst <= 1.0:
                if best is None or geo["mass_index"] < best[0]["mass_index"] - 1e-12:
                    best = (geo, checks)
            elif best_any is None or worst < best_any[2]:
                best_any = (geo, checks, worst)
    if best:
        return best
    return None, (best_any[1] if best_any else [])


def helix_points(geo, per_turn=24, compression=0.0):
    """Wire centre-line points (mm) along the spring axis z. One closed coil at each end (pitch = d),
    pitch p in the active coils; for a conical spring the radius goes linearly D1/2 -> D2/2 in the active part.
    Centre line z = 0 -> L0 (= 2d + p Na); cutting with the planes z = 0 and z = L0 (grinding) removes half
    of the end wire: squared and ground ends.
    compression: the spring compressed by this much (mm): active pitch p - compression / Na (animation frames)."""
    d, Na = geo["d"], geo["Na"]
    p = geo["pitch"] - compression / Na
    R1, R2 = geo["D1"] / 2, geo["D2"] / 2
    Nt = Na + 2
    pts = []
    n_steps = int(round(Nt * per_turn))
    for i in range(n_steps + 1):
        n = i / per_turn                                  # coil count (0 .. Nt)
        if n <= 1:
            z = d * n
            R = R1
        elif n <= 1 + Na:
            z = d + p * (n - 1)
            R = R1 + (R2 - R1) * (n - 1) / Na
        else:
            z = d + p * Na + d * (n - 1 - Na)
            R = R2
        a = 2 * math.pi * n
        pts.append((R * math.cos(a), R * math.sin(a), z))
    return pts
