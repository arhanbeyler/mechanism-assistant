"""
C9: Duz disli cifti modulu (20 derece basinc acili, tam boy evolvent).

Istek: cevrim orani i (= z2/z1, cikis yavaslar), giris torku T1, malzeme (ve istege bagli devir).
Kontroller:
  Alttan kesme:  z1 >= z_min = 2 / sin^2(phi)  (~17)
  Oran hatasi:   |z2/z1 - i| / i <= %2
  Lewis egilme:  sigma = Kv * Ft / (b * m * Y)  <= sy / n     (Y: Shigley Tablo 14-2, Kv: Barth)
                 Ft = 2 T1 / d1,  d1 = m z1,  Kv = (3.05 + V) / 3.05,  V = pi d1 n1 / 60000  [m/s]
  Kavrama orani: eps = (sqrt(ra1^2 - rb1^2) + sqrt(ra2^2 - rb2^2) - a sin phi) / (pi m cos phi) >= 1.2
  Temas (Hertz, bilgi): sigma_c = Cp sqrt(Kv Ft / (b d1 I)),  I = cos(phi) sin(phi) / 2 * mG / (mG + 1)
Secim: kontrollerden gecen en hafif (hacim ~ b (d1^2 + d2^2)) z1, modul, disli genisligi.
Birimler: mm, N, MPa, N*mm. Sadece standart kutuphane.
"""

import math

import strength

PHI = math.radians(20.0)
MODULES = [0.5, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]   # ISO 54 (1. seri + 0.6)
FACE_K = (8, 10, 12)          # disli genisligi b = k m
Z1_RANGE = range(12, 41)

# Lewis form faktoru Y, 20 derece tam boy (Shigley, Tablo 14-2)
_LEWIS = [(12, 0.245), (13, 0.261), (14, 0.277), (15, 0.290), (16, 0.296), (17, 0.303), (18, 0.309),
          (19, 0.314), (20, 0.322), (21, 0.328), (22, 0.331), (24, 0.337), (26, 0.346), (28, 0.353),
          (30, 0.359), (34, 0.371), (38, 0.384), (43, 0.397), (50, 0.409), (60, 0.422), (75, 0.435),
          (100, 0.447), (150, 0.460), (300, 0.472), (400, 0.480)]


def lewis_y(z):
    """Tablodan dogrusal ara degerleme (z > 400: 0.485'e yaklasir, kremayer)."""
    if z <= _LEWIS[0][0]:
        return _LEWIS[0][1]
    for (z0, y0), (z1, y1) in zip(_LEWIS, _LEWIS[1:]):
        if z <= z1:
            return y0 + (y1 - y0) * (z - z0) / (z1 - z0)
    return 0.485


def z_min(phi=PHI):
    return 2.0 / math.sin(phi) ** 2


def contact_ratio(m, z1, z2, phi=PHI):
    r1, r2 = m * z1 / 2, m * z2 / 2
    ra1, ra2 = r1 + m, r2 + m
    rb1, rb2 = r1 * math.cos(phi), r2 * math.cos(phi)
    a = r1 + r2
    return (math.sqrt(ra1 ** 2 - rb1 ** 2) + math.sqrt(ra2 ** 2 - rb2 ** 2) - a * math.sin(phi)) / (math.pi * m * math.cos(phi))


def check_pair(m, z1, z2, b, T1, mat, rpm=None):
    """Bir disli cifti icin kontroller. Dondurur: kontrol listesi (util <= 1 gecer)."""
    d1 = m * z1
    Ft = 2 * T1 / d1
    V = math.pi * d1 * (rpm or 0) / 60000.0
    Kv = (3.05 + V) / 3.05
    allow = mat["sy"] / mat["n"]
    checks = []
    for name, z in (("Pinyon", z1), ("Çark", z2)):
        sigma = Kv * Ft / (b * m * lewis_y(z))
        checks.append({"group": "Eğilme", "name": f"{name} diş dibi eğilmesi (Lewis)",
                       "util": sigma / allow, "detail": f"σ = {sigma:.1f} MPa, Y = {lewis_y(z):.3f}"})
    eps = contact_ratio(m, z1, z2)
    checks.append({"group": "Kavrama", "name": "Kavrama oranı", "util": 1.2 / eps,
                   "detail": f"ε = {eps:.2f} (en az 1.2)"})
    nu, E = mat.get("nu", 0.3), mat["E"]
    Cp = math.sqrt(1.0 / (math.pi * 2 * (1 - nu ** 2) / E))
    mG = z2 / z1
    I = math.cos(PHI) * math.sin(PHI) / 2 * mG / (mG + 1)
    sc = Cp * math.sqrt(Kv * Ft / (b * d1 * I))
    checks.append({"group": "Temas", "name": "Temas basıncı (Hertz, bilgi)", "util": 0.0,
                   "detail": f"σ_c = {sc:.0f} MPa"})
    return checks


def design(ratio, T1_Nmm, mat, rpm=None):
    """En hafif uygun cift. Dondurur: (tasarim dict, kontroller) ya da (None, en iyi adayin kontrolleri)."""
    best = best_any = None
    bore1, bore2 = shaft_bore(T1_Nmm), shaft_bore(T1_Nmm * ratio)
    if bore1 is None or bore2 is None:
        return None, [{"group": "Mil", "name": "Mil burulması", "util": 99.0,
                       "detail": f"{T1_Nmm * ratio / 1000:.0f} N·m için Ø{BORES[-1]} mm çelik mil yetmiyor"}]
    for z1 in Z1_RANGE:
        if z1 < z_min() - 1e-9:
            continue
        z2 = max(z1, round(ratio * z1))
        err = abs(z2 / z1 - ratio) / ratio
        if err > 0.02:
            continue
        for m in MODULES:
            for k in FACE_K:
                b = k * m
                checks = check_pair(m, z1, z2, b, T1_Nmm, mat, rpm)
                checks += [hub_check(m, z1, bore1), hub_check(m, z2, bore2)]
                worst = max(c["util"] for c in checks)
                vol = b * ((m * z1) ** 2 + (m * z2) ** 2)
                cand = ({"module": m, "z1": z1, "z2": z2, "face_width": b, "ratio": z2 / z1,
                         "ratio_err_pct": 100 * err, "d1": m * z1, "d2": m * z2,
                         "bore1": bore1, "bore2": bore2,
                         "center_distance": m * (z1 + z2) / 2, "volume": vol}, checks, worst)
                if worst <= 1.0:
                    if best is None or vol < best[0]["volume"] - 1e-9:
                        best = cand
                    break                 # bu modulde daha genis disli sadece agirlastirir
                if best_any is None or worst < best_any[2]:
                    best_any = cand
    if best:
        return best[0], best[1]
    return None, (best_any[1] if best_any else [])


BORES = [3, 4, 5, 6, 8, 10, 12, 15, 20, 25, 30]


def shaft_bore(T_Nmm):
    """Celik mil capi: burulma 16 T / (pi d^3) <= 0.5 sy / n (S235, n = 2). Standart caplardan."""
    st = strength.MATERIALS["steel"]
    allow = 0.5 * st["sy"] / st["n"]
    for d in BORES:
        if 16 * T_Nmm / (math.pi * d ** 3) <= allow:
            return float(d)
    return None


def hub_check(m, z, bore):
    """Mil deligi ile dis dibi arasinda en az 2 m et kalmali."""
    rf = m * z / 2 - 1.25 * m
    return {"group": "Göbek", "name": f"z = {z} göbek eti", "util": (bore / 2 + 2 * m) / rf,
            "detail": f"mil Ø{bore:g}, diş dibi Ø{2 * rf:.1f} mm"}


def involute_tooth(m, z, n=12, phi=PHI):
    """Tek disin yan profilleri (disli merkezinde, dis ekseni +x uzerinde).
    Dondurur: (sag yan, sol yan) nokta listeleri, taban dairesinden (ya da diş dibinden) uca."""
    r = m * z / 2
    rb = r * math.cos(phi)
    ra = r + m
    rf = r - 1.25 * m
    inv = lambda a: math.tan(a) - a
    half = math.pi / (2 * z) + inv(phi)            # dis ekseninden, taban dairesindeki yarim aci
    r_start = max(rb, rf)
    pts = []
    for i in range(n + 1):
        rr = r_start + (ra - r_start) * i / n
        alpha = math.acos(min(1.0, rb / rr))
        ang = half - inv(alpha)
        pts.append((rr * math.cos(ang), rr * math.sin(ang)))
    right = [(x, -y) for x, y in pts]
    left = pts
    return right, left, rf, ra
