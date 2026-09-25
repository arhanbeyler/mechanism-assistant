"""
C10: Basma yayi modulu — silindirik (helis) ve konik. Shigley, Bolum 10.

Istek: en buyuk kuvvet F (N) ve bu kuvvetteki cokme y (mm) -> yay sabiti k = F / y.
Tel: celik yay teli (plastik yay basilmaz; malzeme verilmezse muzik teli).

Silindirik:  k = d^4 G / (8 D^3 Na)
Konik (yaricap R1 -> R2 dogrusal):  y = 64 F / (G d^4) * integral(R^3 dn) = 16 F Na (R1+R2)(R1^2+R2^2) / (G d^4)
             -> k = G d^4 / (16 Na (R1+R2)(R1^2+R2^2))     (dogrusal bolge: en buyuk sarim oturana kadar)
Gerilme:     tau = K_B 8 F D / (pi d^3),  K_B = (4C + 2) / (4C - 3)  (Bergstrasser), C = D / d, D = en buyuk sarim
Tel dayanimi: S_ut = A / d^m,  S_sy = frac * S_ut  (Shigley Tablo 10-4, 10-6: set giderilmemis)
Uclar: kare ve taslanmis -> Nt = Na + 2, Ls = d Nt, L0 = Ls + (1 + xi) y (xi = 0.15), p = (L0 - 2d) / Na
Kontroller: 4 <= C <= 12, 3 <= Na <= 15, calisma emniyeti n >= 1.2, TAM SIKISMADA akma yok (n_s >= 1),
            burkulma (silindirik, iki ucu sabit, alpha = 0.5): L0 <= 2.63 D / alpha
Secim: kontrollerden gecen EN HAFIF yay (kutle ~ d^2 D_ort Nt).
"""

import math

SPRING_MATERIALS = {
    # A [MPa mm^m], m, G [MPa], S_sy / S_ut (statik, set giderilmemis)
    "music_wire": {"name": "Müzik teli (ASTM A228)", "A": 2211.0, "m": 0.145, "G": 81000.0, "frac": 0.45},
    "stainless": {"name": "Paslanmaz 302 (A313)", "A": 1867.0, "m": 0.146, "G": 69000.0, "frac": 0.35},
}
WIRES = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.5, 2.8, 3.0, 3.2, 3.5,
         4.0, 4.5, 5.0, 5.5, 6.0]
XI = 0.15            # carpma payi (clash allowance)
N_STATIC = 1.2       # calisma emniyet katsayisi
ALPHA = 0.5          # burkulma: iki ucu sabit (paralel plakalar arasinda)


def bergstrasser(C):
    return (4 * C + 2) / (4 * C - 3)


def rate(d, G, Na, D1, taper=1.0):
    """Yay sabiti (N/mm). taper = D2 / D1 (1: silindirik)."""
    R1, R2 = D1 / 2, taper * D1 / 2
    return G * d ** 4 / (16 * Na * (R1 + R2) * (R1 ** 2 + R2 ** 2))


def check(d, D1, Na, F, y, mat, taper=1.0, od_max=None):
    """Bir yay icin kontroller ve geometri. Dondurur: (kontroller, geometri)."""
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
    y_work = F / k                          # yuvarlanmis Na ile gercek cokme
    L0 = Ls + (1 + XI) * y_work
    F_s = k * (L0 - Ls)
    tau_s = KB * 8 * F_s * D1 / (math.pi * d ** 3)
    checks = [
        {"group": "Geometri", "name": "Yay indeksi C = D/d", "util": max(4 / C, C / 12, (3 / C2) if taper < 1 else 0),
         "detail": f"C = {C:.1f}" + (f", küçük uçta {C2:.1f}" if taper < 1 else "") + " (4–12)"},
        {"group": "Geometri", "name": "Aktif sarım sayısı", "util": max(3 / Na, Na / 15), "detail": f"Na = {Na:g} (3–15)"},
        {"group": "Gerilme", "name": "Çalışma gerilmesi", "util": N_STATIC * tau / Ssy,
         "detail": f"τ = {tau:.0f} MPa, S_sy = {Ssy:.0f} MPa, n = {Ssy / tau:.2f} (en az {N_STATIC})"},
        {"group": "Gerilme", "name": "Tam sıkışmada (solid) akma", "util": tau_s / Ssy,
         "detail": f"τ_s = {tau_s:.0f} MPa ≤ S_sy"},
        {"group": "Kuvvet", "name": "Hedef kuvvet / çökme", "util": 0.0 if abs(y_work - y) / y <= 0.05 else 2.0,
         "detail": f"F = {F:g} N'da çökme {y_work:.2f} mm (hedef {y:g} mm)"},
    ]
    if taper >= 1.0:
        L_cr = 2.63 * D1 / ALPHA
        checks.append({"group": "Burkulma", "name": "Burkulma", "util": L0 / L_cr,
                       "detail": f"L0 = {L0:.1f} mm ≤ {L_cr:.1f} mm"})
    if od_max:
        checks.append({"group": "Geometri", "name": "Dış çap sınırı", "util": (D1 + d) / od_max,
                       "detail": f"Ø{D1 + d:.1f} ≤ Ø{od_max:g} mm"})
    geo = {"d": d, "D1": D1, "D2": taper * D1, "Na": Na, "Nt": Nt, "L0": L0, "Ls": Ls, "k": k,
           "pitch": (L0 - 2 * d) / Na, "y_work": y_work, "taper": taper, "OD": D1 + d,
           "mass_index": d * d * D1 * (1 + taper) / 2 * Nt}
    return checks, geo


def design(F, y, mat, taper=1.0, od_max=None):
    """En hafif uygun yay. Dondurur: (geometri, kontroller) ya da (None, en iyi adayin kontrolleri)."""
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


def helix_points(geo, per_turn=24):
    """Yay ekseni z boyunca tel merkez cizgisi noktalari (mm). Uclarda 1'er kapali sarim (hatve = d),
    aktif sarimlarda hatve p; konik yayda yaricap aktif bolgede D1/2 -> D2/2 dogrusal.
    Merkez cizgisi z = 0 -> L0 (= 2d + p Na); z = 0 ve z = L0 duzlemleriyle kesilince (taslama)
    uclardaki telin yarisi gider: kare ve taslanmis uclar."""
    d, Na, p = geo["d"], geo["Na"], geo["pitch"]
    R1, R2 = geo["D1"] / 2, geo["D2"] / 2
    Nt = Na + 2
    pts = []
    n_steps = int(round(Nt * per_turn))
    for i in range(n_steps + 1):
        n = i / per_turn                                  # sarim sayisi (0 .. Nt)
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
