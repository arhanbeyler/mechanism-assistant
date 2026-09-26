"""
G5: power screw (ISO metric trapezoidal, Tr), Shigley ch. 8.

Thread: major diameter d, pitch P, lead l = starts * P, flank half-angle alpha = 15 deg (30 deg thread).
    pitch diameter d_m = d - P/2,  minor diameter d_r = d - P - 2 a_c  (a_c: crest clearance, ISO 2904)
Torques (square-thread formulas with sec(alpha) for the flank angle):
    raising   T_R = F d_m / 2 * (l + pi f d_m sec a) / (pi d_m - f l sec a)
    lowering  T_L = F d_m / 2 * (pi f d_m sec a - l) / (pi d_m + f l sec a)
    + collar  T_c = F f_c d_c / 2  (a thrust washer; a ball thrust bearing is taken as f_c = 0.01)
Self-locking when T_L > 0, i.e. pi f d_m sec a > l; checked with the LOW friction value (well lubricated),
the torques with the HIGH one (the worst case for each).  Efficiency e = F l / (2 pi T_R).
Screw body: axial F / A_r and torsion 16 T / (pi d_r^3) combined (von Mises); a screw pushing a load is a
column: Euler / J.B. Johnson with the free length and end condition C (Shigley 4-43, 4-44).
Threads in the nut (n_t engaged): bearing 2F / (pi d_m n_t P), root bending 6F / (pi d_r n_t P) with the
load shared by the first thread at 38 % (Shigley 8-10 note), shear of the nut threads 3F / (pi d n_t P).
The smallest Tr size, then the shortest nut (in whole threads) that pass are chosen.
Units: N, mm, N*mm. Standard library only.
"""

import math

import strength

ALPHA = math.radians(15.0)
F_HIGH, F_LOW = 0.15, 0.08          # thread friction: torques / self-locking check
FC = {"washer": 0.15, "ball": 0.01}  # collar friction
FIRST_THREAD = 0.38                 # share of the load on the first engaged thread (bending)
NUT_MAX = 2.5                       # longest nut, in major diameters
# ISO 2904 preferred sizes (d, P) and the crest clearance a_c by pitch
SIZES = [(8, 1.5), (10, 2), (12, 3), (14, 3), (16, 4), (18, 4), (20, 4), (22, 5), (24, 5), (26, 5), (28, 5),
         (30, 6), (32, 6), (36, 6), (40, 7), (44, 7), (48, 8), (52, 8), (60, 9), (70, 10), (80, 10)]
STEEL = strength.MATERIALS["steel"]
NUT = {"bronze": {"name": "Bronze (nut)", "sy": 140.0}, "steel": {"name": "Steel (nut)", "sy": STEEL["sy"]},
       "PLA": {"name": "PLA (nut)", "sy": strength.MATERIALS["PLA"]["sy"]}}


def clearance(P):
    return 0.25 if P <= 1.5 else 0.5 if P <= 5 else 1.0


def thread(d, P, starts=1):
    return {"d": d, "P": P, "l": starts * P, "starts": starts, "dm": d - P / 2, "dr": d - P - 2 * clearance(P)}


def torques(th, F, f, fc, dc, alpha=ALPHA):
    dm, l, sec = th["dm"], th["l"], 1 / math.cos(alpha)
    T_R = F * dm / 2 * (l + math.pi * f * dm * sec) / (math.pi * dm - f * l * sec)
    T_L = F * dm / 2 * (math.pi * f * dm * sec - l) / (math.pi * dm + f * l * sec)
    T_c = F * fc * dc / 2
    return T_R, T_L, T_c


def column(th, F, L, C, sy, E, n):
    """Allowable-load utilisation of the screw as a column (root diameter), Euler or Johnson."""
    dr = th["dr"]
    A, I = math.pi * dr ** 2 / 4, math.pi * dr ** 4 / 64
    k = math.sqrt(I / A)
    slender = L / k
    crit = math.sqrt(2 * math.pi ** 2 * C * E / sy)
    if slender > crit:
        Pcr, kind = C * math.pi ** 2 * E * A / slender ** 2, "Euler"
    else:
        Pcr, kind = A * (sy - (sy * slender / (2 * math.pi)) ** 2 / (C * E)), "Johnson"
    return F * n / Pcr, f"{kind}: P_cr = {Pcr:.0f} N (L/k = {slender:.0f})"


def check(th, F, n_t, nut, n, L=None, C=1.0, collar="ball", compression=True):
    """All checks for thread th with n_t engaged threads. Returns (checks, T_R, T_L, T_c, efficiency)."""
    dc = 1.5 * th["d"]
    T_R, T_L, T_c = torques(th, F, F_HIGH, FC[collar], dc)
    T_Ll = torques(th, F, F_LOW, FC[collar], dc)[1]
    T = T_R + T_c
    dr, dm, P, d = th["dr"], th["dm"], th["P"], th["d"]
    sig = F / (math.pi * dr ** 2 / 4)
    tau = 16 * T_R / (math.pi * dr ** 3)
    vm = math.sqrt(sig ** 2 + 3 * tau ** 2)
    allow_s = STEEL["sy"] / n
    allow_n = nut["sy"] / n
    brg = 2 * F / (math.pi * dm * n_t * P)
    bend = 6 * FIRST_THREAD * F / (math.pi * dr * P)                 # the first thread's share, root bending
    shear = 3 * F / (math.pi * d * n_t * P)                           # nut threads sheared at the major diameter
    out = [
        {"group": "Screw", "name": "Self-locking (lubricated, f = 0.08)", "util": 0.5 if T_Ll > 0 else 2.0,
         "detail": f"lowering torque {T_Ll / 1000:.2f} N·m ({'holds by itself' if T_Ll > 0 else 'runs back: needs a brake'})"},
        {"group": "Screw", "name": "Body: axial + torsion (von Mises)", "util": vm / allow_s,
         "detail": f"σ = {sig:.1f}, τ = {tau:.1f} → σ' = {vm:.1f} MPa (steel)"},
        {"group": "Nut", "name": f"Thread bearing ({nut['name']})", "util": brg / allow_n,
         "detail": f"σ_B = {brg:.1f} MPa over {n_t} threads"},
        {"group": "Nut", "name": "Thread root bending (first thread 38 %)", "util": bend / allow_s,
         "detail": f"σ_b = {bend:.1f} MPa"},
        {"group": "Nut", "name": f"Nut thread shear ({nut['name']})", "util": shear / (0.577 * allow_n),
         "detail": f"τ = {shear:.1f} MPa"},
    ]
    if compression and L:
        u, det = column(th, F, L, C, STEEL["sy"], STEEL["E"], n)
        out.append({"group": "Screw", "name": "Buckling (column)", "util": u, "detail": det})
    eff = F * th["l"] / (2 * math.pi * T)
    return out, T_R, T_L, T_c, eff


def design(F, nut_key="bronze", n=2.0, L=None, C=1.0, collar="ball", starts=1, self_locking=True):
    """Smallest Tr thread and nut length for axial load F (N). L: free (column) length, mm.
    self_locking: the load must not run the screw back (a jack); sizes that would are skipped.
    The nut is at least one major diameter long (guidance), at most NUT_MAX diameters.
    Returns (design, checks); design None if no listed size passes."""
    nut = NUT[nut_key]
    best = None
    for d, P in SIZES:
        th = thread(d, P, starts)
        n_min = max(2, math.ceil(d / P))
        n_max = max(n_min, int(NUT_MAX * d / P))
        for n_t in range(n_min, n_max + 1):
            c, T_R, T_L, T_c, eff = check(th, F, n_t, nut, n, L, C, collar)
            worst = max(x["util"] for x in c if self_locking or not x["name"].startswith("Self"))
            if worst <= 1.0:
                return ({"thread": th, "n_t": n_t, "nut_len": n_t * P, "T_R": T_R, "T_L": T_L, "T_c": T_c,
                         "efficiency": eff, "nut": nut, "collar": collar, "L": L, "C": C}, c)
            if best is None or worst < best[0]:
                best = (worst, th, n_t, c)
    return None, best[3]
