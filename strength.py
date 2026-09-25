"""
Mukavemet modulu — dort cubuk uzuvlarinin ve pimlerin boyutlandirilmasi.

Birimler: N, mm, MPa (= N/mm^2), N*mm.

Yuk modeli (yari-statik, atalet ihmal):
    F: kolun ucunda (B) kola dik etkiyen yuk.
    Biyel iki kuvvet elemanidir; kolun O4'e gore moment dengesi:
        F * c = F_AB * c * sin(mu)   ->   F_AB = F / sin(mu)
    Kolun O4 pimi: F_AB'nin kola dik bileseni F'yi dengeler, kalan
    eksenel bilesen F_AB*|cos mu| <= F_AB. Tum pimler en fazla F_AB tasir.

Kontroller (n: emniyet katsayisi, kullanim orani <= 1 olmali):
    Burkulma (her uzuv, mafsalli-mafsalli, zayif eksen): F_AB * n <= pi^2 E I_min / L^2
    Biyel net kesit:   F_AB / ((w-d) t)                    <= sy / n
    Krank:             M_krank / S_net + F_AB / A_net      <= sy / n
    Kol:               F c / S_net + F_AB / A_net          <= sy / n
    Pim kaymasi (tek tesirli): 4 F_AB / (pi d^2)           <= 0.5 sy / n  (Tresca)
    Pim ezilmesi:      F_AB / (d t)                        <= sy / n
Kesit butun uzuvlarda ayni (ayni plaka kalinligi, ayni pimler: uretimi kolay).
"""

import math

# Tipik degerler (3B baski icin %100 doluluk, katmanlar uzuv duzleminde varsayildi).
# Dogrulanmali: baski yonu ve dolulukla plastiklerin dayanimi belirgin degisir.
MATERIALS = {
    "PLA":      {"name": "PLA (3B baskı)",      "sy": 40.0,  "E": 2500.0,   "n": 3.0},
    "PETG":     {"name": "PETG (3B baskı)",     "sy": 35.0,  "E": 2000.0,   "n": 3.0},
    "aluminum": {"name": "Alüminyum 6061-T6",  "sy": 276.0, "E": 69000.0,  "n": 2.0},
    "steel":    {"name": "Çelik S235",          "sy": 235.0, "E": 200000.0, "n": 2.0},
}

DEFAULT_SECTION = {"width": 10.0, "thickness": 4.0, "hole": 4.0}  # yuk verilmediginde

PIN_DIAMETERS = [3, 4, 5, 6, 8, 10, 12]
THICKNESSES = [3, 4, 5, 6, 8, 10, 12, 15, 20]
MAX_WIDTH = 60


def _cross2(u, v):
    return u[0] * v[1] - u[1] * v[0]


def analyze_forces(mech, frames, F):
    """Cevrim boyunca kuvvetler. Dondurur: en buyuk biyel kuvveti, krank momenti, kol momenti."""
    o2 = mech["o2"]
    F_ab_max = M_crank_max = 0.0
    for fr in frames:
        s = math.sin(math.radians(fr["mu"]))
        F_ab = F / max(abs(s), 1e-6)
        A, B = fr["A"], fr["B"]
        r = (A[0] - o2[0], A[1] - o2[1])                    # krank vektoru
        ab = (B[0] - A[0], B[1] - A[1])
        u = (ab[0] / math.hypot(*ab), ab[1] / math.hypot(*ab))  # biyel kuvvet yonu
        M = F_ab * abs(_cross2(r, u))                          # krankta moment = tahrik torku
        F_ab_max = max(F_ab_max, F_ab)
        M_crank_max = max(M_crank_max, M)
    return {"F": F, "F_ab_max": F_ab_max, "M_crank_max": M_crank_max,
            "M_rocker": F * mech["c"]}


def check_section(forces, lengths, mat, w, t, d):
    """Verilen kesit icin tum kontrollerin kullanim oranlari (<= 1 gecer)."""
    n, sy, E = mat["n"], mat["sy"], mat["E"]
    allow = sy / n
    F_ab = forces["F_ab_max"]
    A_net = (w - d) * t
    S_net = t * (w ** 3 - d ** 3) / (6 * w)          # delikli kesit, duzlem ici egilme
    I_min = min(w * t ** 3, t * w ** 3) / 12.0

    checks = []
    for name, key in (("Krank", "a"), ("Biyel", "b"), ("Kol", "c"), ("Sabit", "d")):
        P_cr = math.pi ** 2 * E * I_min / lengths[key] ** 2
        checks.append({"group": "Burkulma", "name": f"{name} burkulması",
                       "util": F_ab * n / P_cr,
                       "detail": f"F_AB = {F_ab:.0f} N, P_kr = {P_cr:.0f} N"})
    checks.append({"group": "Gerilme", "name": "Biyel net kesit",
                   "util": (F_ab / A_net) / allow,
                   "detail": f"σ = {F_ab / A_net:.1f} MPa"})
    s_crank = forces["M_crank_max"] / S_net + F_ab / A_net
    checks.append({"group": "Gerilme", "name": "Krank eğilme + eksenel",
                   "util": s_crank / allow, "detail": f"σ = {s_crank:.1f} MPa"})
    s_rocker = forces["M_rocker"] / S_net + F_ab / A_net
    checks.append({"group": "Gerilme", "name": "Kol eğilme + eksenel",
                   "util": s_rocker / allow, "detail": f"σ = {s_rocker:.1f} MPa"})
    tau = 4 * F_ab / (math.pi * d ** 2)
    checks.append({"group": "Pim", "name": "Pim kayması",
                   "util": tau / (0.5 * allow), "detail": f"τ = {tau:.1f} MPa"})
    s_br = F_ab / (d * t)
    checks.append({"group": "Pim", "name": "Pim ezilmesi",
                   "util": s_br / allow, "detail": f"σ_ez = {s_br:.1f} MPa"})
    return checks


def size_section(forces, lengths, mat):
    """
    Butun kontrolleri gecen EN HAFIF kesit (w * t en kucuk).
    Dondurur: (kesit, kontroller) ya da hicbiri gecmezse (None, en iyi aday kontrolleri).
    """
    best = best_any = None
    for d in PIN_DIAMETERS:
        for t in THICKNESSES:
            for w in range(max(math.ceil(2.5 * d), 8), MAX_WIDTH + 1):  # kenar mesafesi
                checks = check_section(forces, lengths, mat, w, t, d)
                worst = max(c["util"] for c in checks)
                cand = ({"width": float(w), "thickness": float(t), "hole": float(d)}, checks, worst)
                if worst <= 1.0:
                    if best is None or w * t < best[0]["width"] * best[0]["thickness"]:
                        best = cand
                    break          # bu (d, t) icin daha genisi sadece agirlastirir
                if best_any is None or worst < best_any[2]:
                    best_any = cand
    if best:
        return best[0], best[1]
    return None, best_any[1]
