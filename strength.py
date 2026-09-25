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
    Rijitlik (sehim):      krank/kol: P L^3/(3 E I) + P L/(kappa G A) + P L^2/(C E t r^2)  <= L / 200
                           (uctan yuklu konsol: egilme + Timoshenko kayma + gobek donmesi, C = HUB_C)
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

# Gobek esnekligi (B7g): milden/pimden tutulan uzuv, kucuk delik cevresinde doner.
# Donme rijitligi k = C E t r^2  ->  uc sehimine katkisi  delta_hub = P L^2 / (C E t r^2).
# C, yerel FEA taramasiyla kalibre edildi (tools/hub_calibration.py: 21 geometri,
# L = 40-100, w = 9-29, d = 3-6 mm). d/w <= 0.3 icin C = 5.96-9.17 ve L'den bagimsiz
# (L^2 olceklemesi dogrulandi). Guvenli tarafta kalmak icin ALT SINIR kullanilir.
HUB_C = 6.0


def kt_hole_tension(d, w):
    """Roark: sonlu genislikte plakada merkez delik, cekme, net kesite gore Kt (0 <= d/w < 1)."""
    r = d / w
    return 3.00 - 3.13 * r + 3.66 * r ** 2 - 1.53 * r ** 3

LINK_KEYS = {"Krank": "a", "Biyel": "b", "Kol": "c", "Sabit": "d"}

# B7h: mafsal turleri. O2 (tahrik mili) ve O4 (cikis mili) MIL: tork tasir, uzuv milden
# tutulur (gobek). A ve B serbest PIM: sadece kuvvet tasir. Her uzvun (baslangic, bitis) ucu:
LINK_ENDS = {"Krank": ("O2", "A"), "Biyel": ("A", "B"), "Kol": ("O4", "B"), "Sabit": ("O2", "O4")}
SHAFT_POINTS = {"O2", "O4"}


def hole_at(point, d_pin, d_shaft):
    return d_shaft if point in SHAFT_POINTS else d_pin


def link_holes(name, d_pin, d_shaft):
    """(baslangic, bitis) delik caplari."""
    return tuple(hole_at(p, d_pin, d_shaft) for p in LINK_ENDS[name])


# Yuk verilmediginde kullanilan kesit
DEFAULT_SECTION = {"hole": 4.0, "shaft": 4.0,
                   "links": {name: {"width": 10.0, "thickness": 4.0} for name in LINK_KEYS}}

PIN_DIAMETERS = [3, 4, 5, 6, 8, 10, 12]
THICKNESSES = [3, 4, 5, 6, 8, 10, 12, 15, 20]
MAX_WIDTH = 60
W_MIN = 6            # en ince govde (baski icin)

# B8a: yerel gobek (boss). Her delik kendi dairesel gobeginin icinde: D = max(w, 2.5 d).
# Uzuv = iki uc gobegi + aralarinda w genisliginde govde. Kenar mesafesi kurali artik
# sadece gobekte gecerli; govde, kontrollerin gerektirdigi kadar ince olabilir.
BOSS_RATIO = 2.5


def boss_diameters(name, w, d_pin, d_shaft):
    """(baslangic, bitis) gobek caplari."""
    return tuple(max(w, BOSS_RATIO * h) for h in link_holes(name, d_pin, d_shaft))


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


def check_link(name, forces, length, mat, w, t, d, d_shaft=None):
    """Tek uzvun kontrolleri (kullanim oranlari, <= 1 gecer).
    d: pim capi (A, B); d_shaft: mil capi (O2, O4), verilmezse d."""
    n, sy, E = mat["n"], mat["sy"], mat["E"]
    allow = sy / n
    F_ab = forces["F_ab_max"]
    d_shaft = d if d_shaft is None else d_shaft
    holes = link_holes(name, d, d_shaft)
    bosses = boss_diameters(name, w, d, d_shaft)
    d_small, d_root, D_root = min(holes), holes[0], bosses[0]   # kok = milli uc (krank, kol)
    # Gobek kesiti (delikten gecen): en kucuk net alanli uc
    i_net = min(range(2), key=lambda i: (bosses[i] - holes[i]) * t)
    d_net, D_net = holes[i_net], bosses[i_net]
    A_net = (D_net - d_net) * t
    S_hub = t * (D_root ** 3 - d_root ** 3) / (6 * D_root)       # kokte (mil gobegi) egilme
    # Govde kesiti (gobegin hemen disi): delik yok
    S_web, A_web = t * w ** 2 / 6.0, w * t
    I_min = min(w * t ** 3, t * w ** 3) / 12.0

    P_cr = math.pi ** 2 * E * I_min / length ** 2
    checks = [{"group": "Burkulma", "name": f"{name} burkulması", "util": F_ab * n / P_cr,
               "detail": f"F_AB = {F_ab:.0f} N, P_kr = {P_cr:.0f} N"}]
    moment = {"Krank": forces["M_crank_max"], "Kol": forces["M_rocker"]}.get(name, 0.0)
    kt = kt_hole_tension(d_net, D_net) if mat.get("brittle") else 1.0
    s_hub = moment / S_hub + kt * F_ab / A_net
    s_web = moment / S_web + F_ab / A_web            # ihtiyatli: gobek kenarindaki moment ~ M
    sigma, where = (s_hub, "göbek") if s_hub >= s_web else (s_web, "gövde")
    label = f"{name} eğilme + eksenel" if moment else f"{name} net kesit"
    checks.append({"group": "Gerilme", "name": label, "util": sigma / allow,
                   "detail": f"σ = {sigma:.1f} MPa ({where})" + (f", Kt = {kt:.2f}" if kt > 1 and where == "göbek" else "")})

    # Rijitlik: krank ve kol uctan yuklu konsol (duzlem ici), digerleri eksenel uzama
    limit = length / DEFLECTION_RATIO
    if moment:
        P_tip = moment / length                      # uctaki dik kuvvet
        I_in = t * w ** 3 / 12.0
        G = E / (2 * (1 + mat.get("nu", 0.35)))
        d_bend = P_tip * length ** 3 / (3 * E * I_in)                 # Euler-Bernoulli
        d_shear = P_tip * length / ((5.0 / 6.0) * G * w * t)         # Timoshenko kayma
        d_hub = P_tip * length ** 2 / (HUB_C * E * t * (d_root / 2) ** 2)  # gobek (mil) donmesi
        delta = d_bend + d_shear + d_hub
        kind = "sehimi (eğilme + kayma + göbek)"
    else:
        delta = F_ab * length / (E * w * t)
        kind = "eksenel uzama"
    checks.append({"group": "Rijitlik", "name": f"{name} {kind}", "util": delta / limit,
                   "detail": f"δ = {delta:.2f} mm (sınır L/{DEFLECTION_RATIO} = {limit:.2f} mm)"})
    s_br = F_ab / (d_small * t)                         # en kucuk delikte ezilme en buyuk
    checks.append({"group": "Pim", "name": f"{name} pim ezilmesi", "util": s_br / allow,
                   "detail": f"σ_ez = {s_br:.1f} MPa"})
    return checks


def check_pin(forces, mat, d):
    tau = 4 * forces["F_ab_max"] / (math.pi * d ** 2)
    return {"group": "Pim", "name": "Pim kayması", "util": tau / (0.5 * mat["sy"] / mat["n"]),
            "detail": f"τ = {tau:.1f} MPa"}


SHAFT_MATERIAL = "steel"   # tork tasiyan miller celik (plastik uzvun gobeginden gecer)


def check_shaft(forces, mat, d):
    """Mil (O2, O4), CELIK: burulma 16 T / (pi d^3) + dogrudan kayma 4 F / (pi d^2) (ihtiyatli toplam).
    mat parametresi kullanilmaz (uzuv malzemesi); mil her zaman SHAFT_MATERIAL."""
    sm = MATERIALS[SHAFT_MATERIAL]
    T = max(forces["M_crank_max"], forces["M_rocker"])
    tau = 16 * T / (math.pi * d ** 3) + 4 * forces["F_ab_max"] / (math.pi * d ** 2)
    return {"group": "Pim", "name": "Mil (çelik) burulma + kayma", "util": tau / (0.5 * sm["sy"] / sm["n"]),
            "detail": f"τ = {tau:.1f} MPa (T = {T / 1000:.2f} N·m)"}


def _lightest_link(name, forces, length, mat, d, d_shaft=None):
    """Verilen pim/mil caplari icin bu uzvun en hafif (w * t en kucuk) gecen kesiti."""
    d_shaft = d if d_shaft is None else d_shaft
    best = None
    for t in THICKNESSES:
        for w in range(W_MIN, MAX_WIDTH + 1):          # kenar mesafesini gobek sagliyor
            checks = check_link(name, forces, length, mat, w, t, d, d_shaft)
            if max(c["util"] for c in checks) <= 1.0:
                if best is None or w * t < best[0] * best[1]:
                    best = (w, t, checks)
                break          # butun kullanim oranlari w ile azalir: daha genisi agirlastirir
    return best


def size_links(forces, lengths, mat):
    """
    Pim capi (A, B) + mil capi (O2, O4) + uzuv basina en hafif kesit; toplam hacim en kucuk.
    Dondurur: (kesit, kontroller). Uygun kesit yoksa (None, en zor kontroller).
    """
    best = None
    for d in PIN_DIAMETERS:
        pin = check_pin(forces, mat, d)
        if pin["util"] > 1.0:
            continue
        for ds in (x for x in PIN_DIAMETERS if x >= d):
            shaft = check_shaft(forces, mat, ds)
            if shaft["util"] > 1.0:
                continue
            links, checks, volume = {}, [pin, shaft], 0.0
            for name, key in LINK_KEYS.items():
                found = _lightest_link(name, forces, lengths[key], mat, d, ds)
                if found is None:
                    break
                w, t, c = found
                b0, b1 = boss_diameters(name, w, d, ds)
                links[name] = {"width": float(w), "thickness": float(t),
                               "boss0": float(b0), "boss1": float(b1)}
                checks += c
                volume += t * (w * lengths[key] + math.pi * (b0 ** 2 + b1 ** 2) / 8)  # govde + gobek yarilari
            else:
                if best is None or volume < best[2] - 1e-9:
                    best = ({"hole": float(d), "shaft": float(ds), "links": links}, checks, volume)
    if best:
        return best[0], best[1]
    # Hicbiri gecmedi: en buyuk kesitteki kontroller neyin yetmedigini gosterir
    d, w, t = PIN_DIAMETERS[-1], MAX_WIDTH, THICKNESSES[-1]
    fail = [check_pin(forces, mat, d), check_shaft(forces, mat, d)]
    for name, key in LINK_KEYS.items():
        fail += check_link(name, forces, lengths[key], mat, w, t, d, d)
    return None, fail


def section_with_bosses(section):
    """Eski/varsayilan kesite gobek caplarini ekler (Fusion ve FEA icin)."""
    d, ds = section["hole"], section.get("shaft", section["hole"])
    links = {}
    for name, s in section["links"].items():
        b0, b1 = boss_diameters(name, s["width"], d, ds)
        links[name] = dict(s, boss0=s.get("boss0", b0), boss1=s.get("boss1", b1))
    return dict(section, shaft=ds, links=links)
