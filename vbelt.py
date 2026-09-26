"""
G3: V-belt drive (classical A, B, C sections), Shigley 17-4.

Belt speed V = pi d n (ft/min); allowed power per belt H_a = K1 K2 H_tab
    H_tab: Table 17-12 by section, small sheave pitch diameter d and V (linear between 1000..5000 ft/min;
           below 1000 ft/min linear to zero, above 5000 not allowed)
    K1: angle of contact on the small sheave (Table 17-13),  K2: belt length (Table 17-14)
Design power H_d = K_s H_nom;  belts N_b = ceil(H_d / H_a);  factor of safety n_fs = H_a N_b / (K_s H_nom)
Geometry: pitch length L_p = 2C + pi (D + d)/2 + (D - d)^2 / (4C); the next standard belt (inside circumference
    + Table 17-11 conversion), then C = 0.25 {[L_p - pi/2 (D + d)] + sqrt([L_p - pi/2 (D + d)]^2 - 2 (D - d)^2)}
Tensions (per belt): F_c = K_c (V/1000)^2,  dF = 63025 H_d / N_b / (n d/2),
    F1 = F_c + dF exp(f phi) / (exp(f phi) - 1) (f = 0.5123),  F2 = F1 - dF,  initial F_i = (F1 + F2)/2 - F_c
Life: peak tensions T1 = F1 + K_b/d, T2 = F1 + K_b/D;  passes N_p = [(K/T1)^-b + (K/T2)^-b]^-1 (Table 17-17),
    hours t = N_p L_p / (720 V)  (N_p capped at 1e9, the table's range)
Choice: the smallest pair of sheaves (pitch-circle areas x sheave width (N_b - 1) e + 2 f) that passes, N_b <= 5.
Units outside: mm, N, W, N*mm, rev/min; inside inches / lbf / hp as in the tables.
"""

import bisect
import math

import chain

IN = 25.4
HP = 745.7
LBF = 4.448
F_EFF = 0.5123
V_MAX = 5000.0                      # ft/min
LIFE_MIN = 20000.0                  # h
NP_MAX = 1e9                        # Table 17-17 holds up to 1e9 passes; beyond that the life is reported as "more than"
NB_MAX = 5
SERVICE = {"smooth": 1.1, "moderate": 1.3, "heavy": 1.4}     # Table 17-15, normal-torque motor
SPEEDS = (1000, 2000, 3000, 4000, 5000)
# Section: groove pitch e and edge f of the sheave (mm, ISO 4183), top width a, thickness b (in), length conversion Lc (Table 17-11), Kb, Kc (17-16), K, b (17-17),
# H_tab rows (d in, hp at SPEEDS; the last row holds for any larger d), standard inside circumferences (17-10)
SECTIONS = {
    "A": {"e": 15.0, "f": 10.0, "a": 0.5, "b": 11 / 32, "Lc": 1.3, "Kb": 220, "Kc": 0.561, "K": 674, "bexp": 11.089,
          "rows": [(3.0, (0.66, 1.01, 1.12, 0.93, 0.38)), (3.4, (0.81, 1.31, 1.57, 1.53, 1.12)),
                   (3.8, (0.93, 1.55, 1.92, 2.00, 1.71)), (4.2, (1.03, 1.74, 2.20, 2.38, 2.19)),
                   (4.6, (1.11, 1.89, 2.44, 2.69, 2.58)), (5.0, (1.17, 2.03, 2.64, 2.96, 2.89))],
          "lengths": [26, 31, 33, 35, 38, 42, 46, 48, 51, 53, 55, 57, 60, 62, 64, 66, 68, 71, 75, 78, 80, 85, 90,
                      96, 105, 112, 120, 128],
          "K2": [(35, 0.85), (46, 0.90), (55, 0.95), (75, 1.00), (90, 1.05), (112, 1.10), (1e9, 1.15)]},
    "B": {"e": 19.0, "f": 12.5, "a": 21 / 32, "b": 7 / 16, "Lc": 1.8, "Kb": 576, "Kc": 0.965, "K": 1193, "bexp": 10.926,
          "rows": [(5.4, (1.59, 2.62, 3.24, 3.34, 2.82)), (5.8, (1.72, 2.87, 3.61, 3.85, 3.45)),
                   (6.2, (1.82, 3.09, 3.94, 4.28, 4.00)), (6.6, (1.92, 3.29, 4.23, 4.67, 4.48)),
                   (7.0, (2.01, 3.46, 4.49, 5.01, 4.90))],
          "lengths": [35, 38, 42, 46, 48, 51, 53, 55, 57, 60, 62, 64, 65, 66, 68, 71, 75, 78, 79, 81, 83, 85, 90,
                      93, 97, 100, 103, 105, 112, 120, 128, 131, 136, 144, 158, 173, 180, 195, 210, 240, 270, 300],
          "K2": [(46, 0.85), (60, 0.90), (75, 0.95), (97, 1.00), (120, 1.05), (144, 1.10), (180, 1.15), (1e9, 1.20)]},
    "C": {"e": 25.5, "f": 17.0, "a": 7 / 8, "b": 17 / 32, "Lc": 2.9, "Kb": 1600, "Kc": 1.716, "K": 2038, "bexp": 11.173,
          "rows": [(9.0, (3.34, 5.65, 7.21, 7.86, 7.39)), (10.0, (3.64, 6.25, 8.11, 9.06, 8.89)),
                   (11.0, (3.88, 6.74, 8.84, 10.0, 10.1)), (12.0, (4.09, 7.15, 9.46, 10.9, 11.1))],
          "lengths": [51, 60, 68, 75, 81, 85, 90, 96, 105, 112, 120, 128, 136, 144, 158, 162, 173, 180, 195, 210,
                      240, 270, 300, 330, 360, 390, 420],
          "K2": [(75, 0.85), (96, 0.90), (120, 0.95), (158, 1.00), (195, 1.05), (240, 1.10), (300, 1.15), (1e9, 1.20)]},
}
# Table 17-13: (D - d)/C -> K1 (V-V drives)
K1_TABLE = [(0.0, 1.00), (0.1, 0.99), (0.2, 0.97), (0.3, 0.96), (0.4, 0.94), (0.5, 0.93), (0.6, 0.91),
            (0.7, 0.89), (0.8, 0.87), (0.9, 0.85), (1.0, 0.82), (1.1, 0.80), (1.2, 0.77), (1.3, 0.73),
            (1.4, 0.70), (1.5, 0.65)]


def _interp(table, x):
    xs = [t[0] for t in table]
    if x <= xs[0]:
        return table[0][1]
    if x >= xs[-1]:
        return table[-1][1]
    i = bisect.bisect_right(xs, x)
    (x0, y0), (x1, y1) = table[i - 1], table[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def h_tab(sec, d, V):
    """Table 17-12 rating (hp per belt) for pitch diameter d (in) at V (ft/min); None beyond 5000 ft/min."""
    if V > V_MAX:
        return None
    rows = SECTIONS[sec]["rows"]
    d = min(d, rows[-1][0])
    by_speed = [(s, _interp([(r[0], r[1][k]) for r in rows], d)) for k, s in enumerate(SPEEDS)]
    return _interp([(0.0, 0.0)] + by_speed, V)


def k1(D, d, C):
    return _interp(K1_TABLE, (D - d) / C)


def k2(sec, L_inside):
    return next(k for top, k in SECTIONS[sec]["K2"] if L_inside <= top)


def pitch_length(C, D, d):
    return 2 * C + math.pi * (D + d) / 2 + (D - d) ** 2 / (4 * C)


def centre(Lp, D, d):
    a = Lp - math.pi / 2 * (D + d)
    return 0.25 * (a + math.sqrt(max(a * a - 2 * (D - d) ** 2, 0.0)))


def tensions(sec, H_d_hp, Nb, n, d, D, phi):
    s = SECTIONS[sec]
    V = math.pi * d * n / 12
    Fc = s["Kc"] * (V / 1000) ** 2
    dF = 63025 * H_d_hp / Nb / (n * d / 2)
    e = math.exp(F_EFF * phi)
    F1 = Fc + dF * e / (e - 1)
    F2 = F1 - dF
    return {"Fc": Fc, "dF": dF, "F1": F1, "F2": F2, "Fi": (F1 + F2) / 2 - Fc}


def life_hours(sec, F1, d, D, Lp, V):
    s = SECTIONS[sec]
    T1, T2 = F1 + s["Kb"] / d, F1 + s["Kb"] / D
    Np = min(1 / ((s["K"] / T1) ** -s["bexp"] + (s["K"] / T2) ** -s["bexp"]), NP_MAX)
    return Np, Np * Lp / (720 * V)


def evaluate(sec, d, D, n1, P_W, Ks, C_in=None):
    """One candidate: small sheave d at n1, large D (inches). Returns (info, checks) or None (too fast)."""
    s = SECTIONS[sec]
    V = math.pi * d * n1 / 12
    tab = h_tab(sec, d, V)
    if tab is None:
        return None
    C0 = C_in or (D + d)
    Lp_need = pitch_length(C0, D, d)
    Li = next((L for L in s["lengths"] if L + s["Lc"] >= Lp_need), None)
    if Li is None:
        return None
    Lp = Li + s["Lc"]
    C = centre(Lp, D, d)
    if D - d >= 2 * C:
        return None                                         # the sheaves would overlap: not a drive
    phi = math.pi - 2 * math.asin((D - d) / (2 * C))
    Ha = k1(D, d, C) * k2(sec, Li) * tab
    Hd = Ks * P_W / HP
    Nb = max(1, math.ceil(Hd / Ha - 1e-9))
    t = tensions(sec, Hd, Nb, n1, d, D, phi)
    Np, hours = life_hours(sec, t["F1"], d, D, Lp, V)
    T1 = Hd * HP / (2 * math.pi * n1 / 60) * 1000
    T2 = T1 * D / d
    b1, b2 = chain.shaft_bore(T1), chain.shaft_bore(T2)
    checks = [
        {"group": "Rating", "name": f"Belts (Shigley 17-12, {Nb} x {sec}{Li})", "util": Nb / NB_MAX,
         "detail": f"{Ha * HP / 1000:.2f} kW per belt, n_fs = {Ha * Nb / Hd:.2f} (at most {NB_MAX} belts)"},
        {"group": "Rating", "name": "Belt speed", "util": V / V_MAX,
         "detail": f"{V * 0.00508:.1f} m/s (at most {V_MAX * 0.00508:.1f})"},
        {"group": "Rating", "name": "Belt life (Shigley 17-17)", "util": LIFE_MIN / hours,
         "detail": f"{'more than ' if Np >= NP_MAX else ''}{hours:,.0f} h (at least {LIFE_MIN:,.0f} h)"},
        {"group": "Geometry", "name": "Centre distance", "util": max(D / C, C / (3 * (D + d))),
         "detail": f"C = {C * IN:.0f} mm (between D and 3(D + d))"},
    ]
    for name, T, b, dia in (("Small", T1, b1, d), ("Large", T2, b2, D)):
        if b is None:
            checks.append({"group": "Shafts", "name": f"{name} sheave shaft", "util": 9.9,
                           "detail": f"{T / 1000:.0f} N·m is beyond Ø{chain.BORES[-1]}"})
        else:
            root = (dia - 2 * s["b"]) * IN
            checks.append({"group": "Shafts", "name": f"{name} sheave shaft and hub",
                           "util": (b + 2 * max(3.0, b / 4)) / root,
                           "detail": f"Ø{b:g} steel shaft, {T / 1000:.1f} N·m, groove bottom Ø{root:.0f} mm"})
    info = {"section": sec, "belt": f"{sec}{Li}", "Li": Li * IN, "Lp": Lp * IN, "belts": Nb,
            "d": d * IN, "D": D * IN, "C": C * IN, "wrap": math.degrees(phi), "V": V * 0.00508,
            "H_tab": tab * HP, "H_a": Ha * HP, "H_d": Hd * HP, "n_fs": Ha * Nb / Hd, "K1": k1(D, d, C),
            "K2": k2(sec, Li), "F1": t["F1"] * LBF, "F2": t["F2"] * LBF, "Fi": t["Fi"] * LBF, "Fc": t["Fc"] * LBF,
            "shaft_load": Nb * (t["F1"] + t["F2"]) * LBF, "life_h": hours, "T1": T1, "T2": T2,
            "bore1": b1, "bore2": b2, "width": (Nb - 1) * s["e"] + 2 * s["f"], "e": s["e"], "top": s["a"] * IN, "thick": s["b"] * IN, "Ks": Ks}
    return info, checks


def design(P_W, n_in, ratio, Ks=1.1, C_mm=None):
    """Lightest V-belt drive for power P_W at n_in rev/min and speed ratio n_in / n_out. Returns (design, checks)."""
    small_in = ratio >= 1
    i = ratio if small_in else 1 / ratio
    n1 = n_in if small_in else n_in * i
    best = best_any = None
    for sec, s in SECTIONS.items():
        base = s["rows"][0][0]
        for d in [base + 0.2 * k for k in range(0, 31)]:
            got = evaluate(sec, d, d * i, n1, P_W, Ks, C_mm / IN if C_mm else None)
            if got is None:
                continue
            info, checks = got
            worst = max(c["util"] for c in checks)
            size = (info["d"] ** 2 + info["D"] ** 2) * info["width"]
            cand = (dict(info, size=size, small_is_input=small_in, n1=n1, ratio=i if small_in else 1 / i),
                    checks, worst)
            if worst <= 1.0:
                if best is None or size < best[0]["size"]:
                    best = cand
            elif best_any is None or worst < best_any[2]:
                best_any = cand
    if best:
        return best[0], best[1]
    return None, (best_any[1] if best_any else [])
