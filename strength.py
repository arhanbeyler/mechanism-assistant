"""
Mukavemet modulu — dort cubuk uzuvlarinin ve pimlerin boyutlandirilmasi.

Birimler: N, mm, MPa (= N/mm^2), N*mm.

Yuk modeli (yari-statik, atalet ihmal):
    F: kolun ucunda (B) kola dik etkiyen yuk.
    Biyel iki kuvvet elemanidir; kolun O4'e gore moment dengesi:
        F * c = F_AB * c * sin(mu)   ->   F_AB = F / sin(mu)
    Kolun O4 pimi: F_AB'nin kola dik bileseni F'yi dengeler, kalan
    eksenel bilesen F_AB*|cos mu| <= F_AB. Tum pimler en fazla F_AB tasir.

Kontroller (n: emniyet katsayisi, kullanim orani <= 1 olmali), her uzuv icin:
    Burkulma (mafsalli-mafsalli, zayif eksen): F_AB * n <= pi^2 E I_min / L^2
    Net kesit (eksenel):   F_AB / ((w-d) t)                   <= sy / n
    Krank ek olarak:       M_krank / S_net + F_AB / A_net     <= sy / n
    Kol ek olarak:         F c / S_net + F_AB / A_net         <= sy / n
    Pim ezilmesi:          F_AB / (d t)                        <= sy / n
    Rijitlik (sehim):      krank/kol: P L^3 / (3 E I_duzlem)   <= L / 200   (uctan yuklu konsol)
                           biyel/sabit: F_AB L / (E A)         <= L / 200   (eksenel uzama)
Pim (tum mafsallarda ayni cap):
    Kayma (tek tesirli):   4 F_AB / (pi d^2)                   <= 0.5 sy / n  (Tresca)

Gerilme yigilmasi (Kt), sadece KIRILGAN malzemelerde (PLA, PETG) ve sadece EKSENEL
gerilmeye: delik duzlem ici egilmenin tarafsiz ekseninde kaldigi icin egilme tepe
gerilmesini belirgin artirmaz. Sunek metallerde statik yukte yerel akma yuku dagittigi
icin kullanilmaz (yorulmada gerekir). Roark, sonlu genislikte delikli plaka, cekme,
net kesite gore:  Kt = 3.00 - 3.13 (d/w) + 3.66 (d/w)^2 - 1.53 (d/w)^3

Boyutlandirma: pim capi ortak; her uzvun genislik ve kalinligi ayri secilir.
Hedef: toplam hacim (sum w * t * L) en kucuk.
"""

import math

# Tipik degerler (3B baski icin %100 doluluk, katmanlar uzuv duzleminde varsayildi).
# Dogrulanmali: baski yonu ve dolulukla plastiklerin dayanimi belirgin degisir.
MATERIALS = {
    "PLA":      {"name": "PLA (3B baskı)",      "sy": 40.0,  "E": 2500.0,   "nu": 0.36, "n": 3.0, "brittle": True},
    "PETG":     {"name": "PETG (3B baskı)",     "sy": 35.0,  "E": 2000.0,   "nu": 0.38, "n": 3.0, "brittle": True},
    "aluminum": {"name": "Alüminyum 6061-T6",  "sy": 276.0, "E": 69000.0,  "nu": 0.33, "n": 2.0, "brittle": False},
    "steel":    {"name": "Çelik S235",          "sy": 235.0, "E": 200000.0, "nu": 0.30, "n": 2.0, "brittle": False},
}

DEFLECTION_RATIO = 200     # izin verilen sehim: L / 200


def kt_hole_tension(d, w):
    """Roark: sonlu genislikte plakada merkez delik, cekme, net kesite gore Kt (0 <= d/w < 1)."""
    r = d / w
    return 3.00 - 3.13 * r + 3.66 * r ** 2 - 1.53 * r ** 3

LINK_KEYS = {"Krank": "a", "Biyel": "b", "Kol": "c", "Sabit": "d"}

# Yuk verilmediginde kullanilan kesit
DEFAULT_SECTION = {"hole": 4.0,
                   "links": {name: {"width": 10.0, "thickness": 4.0} for name in LINK_KEYS}}

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


def check_link(name, forces, length, mat, w, t, d):
    """Tek uzvun kontrolleri (kullanim oranlari, <= 1 gecer)."""
    n, sy, E = mat["n"], mat["sy"], mat["E"]
    allow = sy / n
    F_ab = forces["F_ab_max"]
    A_net = (w - d) * t
    S_net = t * (w ** 3 - d ** 3) / (6 * w)          # delikli kesit, duzlem ici egilme
    I_min = min(w * t ** 3, t * w ** 3) / 12.0

    P_cr = math.pi ** 2 * E * I_min / length ** 2
    checks = [{"group": "Burkulma", "name": f"{name} burkulması", "util": F_ab * n / P_cr,
               "detail": f"F_AB = {F_ab:.0f} N, P_kr = {P_cr:.0f} N"}]
    moment = {"Krank": forces["M_crank_max"], "Kol": forces["M_rocker"]}.get(name, 0.0)
    kt = kt_hole_tension(d, w) if mat.get("brittle") else 1.0
    sigma = moment / S_net + kt * F_ab / A_net
    label = f"{name} eğilme + eksenel" if moment else f"{name} net kesit"
    checks.append({"group": "Gerilme", "name": label, "util": sigma / allow,
                   "detail": f"σ = {sigma:.1f} MPa" + (f" (Kt = {kt:.2f})" if kt > 1 else "")})

    # Rijitlik: krank ve kol uctan yuklu konsol (duzlem ici), digerleri eksenel uzama
    limit = length / DEFLECTION_RATIO
    if moment:
        P_tip = moment / length                      # uctaki dik kuvvet
        I_in = t * w ** 3 / 12.0
        delta = P_tip * length ** 3 / (3 * E * I_in)
        kind = "eğilme sehimi"
    else:
        delta = F_ab * length / (E * w * t)
        kind = "eksenel uzama"
    checks.append({"group": "Rijitlik", "name": f"{name} {kind}", "util": delta / limit,
                   "detail": f"δ = {delta:.2f} mm (sınır L/{DEFLECTION_RATIO} = {limit:.2f} mm)"})
    s_br = F_ab / (d * t)
    checks.append({"group": "Pim", "name": f"{name} pim ezilmesi", "util": s_br / allow,
                   "detail": f"σ_ez = {s_br:.1f} MPa"})
    return checks


def check_pin(forces, mat, d):
    tau = 4 * forces["F_ab_max"] / (math.pi * d ** 2)
    return {"group": "Pim", "name": "Pim kayması", "util": tau / (0.5 * mat["sy"] / mat["n"]),
            "detail": f"τ = {tau:.1f} MPa"}


def _lightest_link(name, forces, length, mat, d):
    """Verilen pim capi icin bu uzvun en hafif (w * t en kucuk) gecen kesiti."""
    best = None
    for t in THICKNESSES:
        for w in range(max(math.ceil(2.5 * d), 8), MAX_WIDTH + 1):  # kenar mesafesi
            checks = check_link(name, forces, length, mat, w, t, d)
            if max(c["util"] for c in checks) <= 1.0:
                if best is None or w * t < best[0] * best[1]:
                    best = (w, t, checks)
                break          # butun kullanim oranlari w ile azalir: daha genisi agirlastirir
    return best


def size_links(forces, lengths, mat):
    """
    Ortak pim capi + uzuv basina en hafif kesit; toplam hacim en kucuk.
    Dondurur: (kesit, kontroller). Uygun kesit yoksa (None, en zor kontroller).
    """
    best = None
    for d in PIN_DIAMETERS:
        pin = check_pin(forces, mat, d)
        if pin["util"] > 1.0:
            continue
        links, checks, volume = {}, [pin], 0.0
        for name, key in LINK_KEYS.items():
            found = _lightest_link(name, forces, lengths[key], mat, d)
            if found is None:
                break
            w, t, c = found
            links[name] = {"width": float(w), "thickness": float(t)}
            checks += c
            volume += w * t * lengths[key]
        else:
            if best is None or volume < best[2]:
                best = ({"hole": float(d), "links": links}, checks, volume)
    if best:
        return best[0], best[1]
    # Hicbiri gecmedi: en buyuk kesitteki kontroller neyin yetmedigini gosterir
    d, w, t = PIN_DIAMETERS[-1], MAX_WIDTH, THICKNESSES[-1]
    fail = [check_pin(forces, mat, d)]
    for name, key in LINK_KEYS.items():
        fail += check_link(name, forces, lengths[key], mat, w, t, d)
    return None, fail
