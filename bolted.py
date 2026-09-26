"""
H1: bolted tension joint (Shigley ch. 8): N bolts share an external separating load P; ISO metric coarse
bolts with nuts; two clamped members of the same material, total grip l.

Stiffness: bolt  k_b = A_d A_t E / (A_d l_t + A_t l_d)  (unthreaded part l_d, threaded part in the grip l_t;
    threaded length L_T = 2d + 6 (L <= 125), 2d + 12 (<= 200), 2d + 25; bolt length = grip + nut, rounded up
    to a standard length);  members (Wileman, Shigley 8-23)  k_m = E d A exp(B d / l).
Joint constant C = k_b / (k_b + k_m);  preload F_i = 0.75 F_p (reusable), F_p = A_t S_p;  torque T = K F_i d
(K = 0.20, as received).
Per bolt, P_b = P / N:  yield factor n_p = S_p A_t / (C P_b + F_i) (>= 1),  load factor n_L = (S_p A_t - F_i) /
(C P_b) (>= 2),  separation n_0 = F_i / (P_b (1 - C)) (>= 1.5);  if the load fluctuates 0..P (Goodman, 8-38):
n_f = S_e (S_ut - sigma_i) / (sigma_a (S_ut + S_e)) (>= 1.5), sigma_a = C P_b / (2 A_t), sigma_i = F_i / A_t.
Also: nut (ISO: as strong as the bolt), bolt spacing on the bolt circle 3..10 d (4 d taken for sealing).
Choice: the smallest bolt that passes with the given N (without N: 4 bolts, more only if M36 is not enough).  Units: mm, N, MPa, N*mm.
"""

import math

E_STEEL = 207000.0
K_TORQUE = 0.20
PRELOAD = 0.75
# d, pitch, tensile stress area (Shigley Table 8-1), nut height (ISO 4032), across flats (hex head / nut)
SIZES = [(5, 0.8, 14.2, 4.7, 8), (6, 1.0, 20.1, 5.2, 10), (8, 1.25, 36.6, 6.8, 13), (10, 1.5, 58.0, 8.4, 16),
         (12, 1.75, 84.3, 10.8, 18), (14, 2.0, 115.0, 12.8, 21), (16, 2.0, 157.0, 14.8, 24), (20, 2.5, 245.0, 18.0, 30),
         (24, 3.0, 353.0, 21.5, 36), (30, 3.5, 561.0, 25.6, 46), (36, 4.0, 817.0, 31.0, 55)]
# ISO property classes (Shigley Table 8-11): proof, tensile, yield (MPa); endurance S_e (Table 8-17)
CLASSES = {"8.8": {"Sp": 600.0, "Sut": 830.0, "Sy": 660.0, "Se": 129.0},
           "10.9": {"Sp": 830.0, "Sut": 1040.0, "Sy": 940.0, "Se": 162.0},
           "12.9": {"Sp": 970.0, "Sut": 1220.0, "Sy": 1100.0, "Se": 190.0}}
# Wileman member stiffness constants (Shigley Table 8-8) and E (MPa)
MEMBERS = {"steel": {"name": "steel", "E": 207000.0, "A": 0.78715, "B": 0.62873},
           "aluminum": {"name": "aluminium", "E": 71000.0, "A": 0.79670, "B": 0.63816},
           "cast_iron": {"name": "grey cast iron", "E": 100000.0, "A": 0.77871, "B": 0.61616}}
LENGTHS = [8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 180, 200]
N_CHOICES = (2, 3, 4, 6, 8, 10, 12, 16, 20)
N_DEFAULT = 4
REQUIRED = {"n_p": 1.0, "n_L": 2.0, "n_0": 1.5, "n_f": 1.5}


def thread_length(L, d):
    return 2 * d + 6 if L <= 125 else 2 * d + 12 if L <= 200 else 2 * d + 25


def bolt_length(grip, d, nut_h):
    need = grip + nut_h + 2 * 0.8 * d / 8 + 2.0          # nut + two thread pitches (approx.) past the nut
    return next((L for L in LENGTHS if L >= need), None)


def bolt_stiffness(d, At, grip, L, E=E_STEEL):
    LT = thread_length(L, d)
    ld = max(L - LT, 0.0)
    ld = min(ld, grip)
    lt = grip - ld
    Ad = math.pi * d * d / 4
    return Ad * At * E / (Ad * lt + At * ld), ld, lt


def member_stiffness_wileman(d, grip, mem):
    return mem["E"] * d * mem["A"] * math.exp(mem["B"] * d / grip)


def member_stiffness_frustum(d, grip, E, alpha=math.radians(30)):
    """Two identical members, 30 deg frusta, washer face 1.5 d (Shigley 8-22)."""
    t = math.tan(alpha)
    return math.pi * E * d * t / (2 * math.log((t * grip + 0.5 * d) * 5 / (t * grip + 2.5 * d)))


def analyse(size, N, P, grip, mem_key="steel", cls="8.8", fluctuating=False):
    d, pitch, At, nut_h, af = size
    c = dict(CLASSES[cls])
    if cls == "8.8" and d < 16:                          # ISO 898-1: 8.8 up to M16 is a little weaker
        c.update(Sp=580.0, Sut=800.0, Sy=640.0)
    mem = MEMBERS[mem_key]
    L = bolt_length(grip, d, nut_h)
    if L is None:
        return None
    kb, ld, lt = bolt_stiffness(d, At, grip, L)
    km = member_stiffness_wileman(d, grip, mem)
    C = kb / (kb + km)
    Fp = At * c["Sp"]
    Fi = PRELOAD * Fp
    Pb = P / N
    out = {"d": d, "pitch": pitch, "At": At, "nut_h": nut_h, "af": af, "L": L, "ld": ld, "lt": lt, "kb": kb, "km": km,
           "C": C, "Fp": Fp, "Fi": Fi, "T": K_TORQUE * Fi * d, "N": N, "P_bolt": Pb, "cls": cls, "member": mem["name"],
           "n_p": c["Sp"] * At / (C * Pb + Fi), "n_L": (c["Sp"] * At - Fi) / (C * Pb), "n_0": Fi / (Pb * (1 - C)),
           "F_bolt_max": Fi + C * Pb, "F_member": Fi - (1 - C) * Pb}
    if fluctuating:
        sa = C * Pb / (2 * At)
        si = Fi / At
        out.update(sigma_a=sa, sigma_i=si, n_f=c["Se"] * (c["Sut"] - si) / (sa * (c["Sut"] + c["Se"])))
    return out


def checks_for(a, fluctuating):
    rows = [("n_p", "Bolt yield (proof) factor", f"n_p = {a['n_p']:.2f} (at least {REQUIRED['n_p']:g}); peak bolt force {a['F_bolt_max']:.0f} N"),
            ("n_L", "Load factor", f"n_L = {a['n_L']:.2f} (at least {REQUIRED['n_L']:g})"),
            ("n_0", "Joint separation", f"n_0 = {a['n_0']:.2f} (at least {REQUIRED['n_0']:g}); clamp left {a['F_member']:.0f} N")]
    if fluctuating:
        rows.append(("n_f", "Fatigue (Goodman, 0..P)", f"n_f = {a['n_f']:.2f} (at least {REQUIRED['n_f']:g}); σ_a = {a['sigma_a']:.1f} MPa"))
    return [{"group": "Bolt" if k in ("n_p", "n_L", "n_f") else "Joint", "name": name,
             "util": REQUIRED[k] / a[k], "detail": detail} for k, name, detail in rows]


def design(P, grip, N=None, mem_key="steel", cls="8.8", fluctuating=False):
    """Smallest bolt for total separating load P with N bolts; without N, 4 bolts first (more only when even
    M36 is not enough). Returns (design, checks)."""
    best_any = None
    counts = (N,) if N else tuple(n for n in N_CHOICES if n >= N_DEFAULT)
    for n in counts:
        for size in SIZES:
            a = analyse(size, n, P, grip, mem_key, cls, fluctuating)
            if a is None:
                continue
            checks = checks_for(a, fluctuating)
            worst = max(c["util"] for c in checks)
            if worst <= 1.0:
                a["bolt_circle"] = max(n * 4 * a["d"] / math.pi, 2 * a["af"] + 4 * a["d"])     # spacing >= 4 d
                return a, checks
            if best_any is None or worst < best_any[1]:
                best_any = (checks, worst)
    return None, (best_any[0] if best_any else [])
