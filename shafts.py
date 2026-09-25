"""
C11b: Mil tasarimi — Shigley, Bolum 6-7 (yorulma, DE-Goodman).

Donen mil: egilme tam degisken (Ma = M, Mm = 0), tork sabit (Tm = T, Ta = 0).
DE-Goodman:  d = [ 16 n / pi * ( 2 Kf Ma / Se + sqrt(3) Kfs Tm / Sut ) ]^(1/3)
             (genel: 1/Se sqrt(4 (Kf Ma)^2 + 3 (Kfs Ta)^2) + 1/Sut sqrt(4 (Kf Mm)^2 + 3 (Kfs Tm)^2))
Se = ka kb kc kd ke Se',  Se' = 0.5 Sut (Sut <= 1400 MPa)
  ka = a Sut^b  (talasli: a = 4.51, b = -0.265)
  kb = 1.24 d^-0.107 (2.79-51 mm), 1.51 d^-0.157 (51-254 mm)   -> d'ye bagli: iterasyon
  kc = 1 (egilme), kd = 1 (oda sicakligi), ke = 0.897 (%90 guvenilirlik)
Gerilme yigilmasi (ilk iterasyon, Shigley Tablo 7-1; muhafazakar Kf = Kt):
  kama yuvasi (parmak freze) Kt 2.14 / Kts 3.0; keskin omuz 2.7 / 2.2; yuvarlatilmis omuz 1.7 / 1.5;
  segman kanali 5.0 / 3.0; duz mil 1.0 / 1.0
Akma: von Mises tepe  sigma' = sqrt((32 Kf M / (pi d^3))^2 + 3 (16 Kfs T / (pi d^3))^2)  <= Sy / ny
Birimler: N, mm, MPa, N*mm.
"""

import math

SHAFT_MATERIALS = {
    "1020": {"name": "AISI 1020 CD", "Sut": 470.0, "Sy": 390.0},
    "1045": {"name": "AISI 1045 CD", "Sut": 630.0, "Sy": 530.0},
}
FEATURES = {
    "keyseat": ("kama yuvası", 2.14, 3.0),
    "sharp_shoulder": ("keskin omuz", 2.7, 2.2),
    "round_shoulder": ("yuvarlatılmış omuz", 1.7, 1.5),
    "ring_groove": ("segman kanalı", 5.0, 3.0),
    "plain": ("düz mil", 1.0, 1.0),
}
STANDARD_D = [6, 8, 10, 12, 14, 15, 16, 17, 18, 20, 22, 25, 28, 30, 32, 35, 38, 40, 42, 45, 48, 50,
              55, 60, 65, 70, 75, 80, 90, 100, 110, 120]
KE_90 = 0.897


def kb(d):
    if d <= 2.79:
        return 1.0
    if d <= 51:
        return 1.24 * d ** -0.107
    return 1.51 * d ** -0.157


def endurance(Sut, d):
    ka = 4.51 * Sut ** -0.265
    return ka * kb(d) * 1.0 * 1.0 * KE_90 * 0.5 * min(Sut, 1400.0)


def de_goodman_d(M, T, Sut, Se, Kf, Kfs, n):
    return (16 * n / math.pi * (2 * Kf * M / Se + math.sqrt(3) * Kfs * T / Sut)) ** (1 / 3)


def de_goodman_n(d, M, T, Sut, Se, Kf, Kfs):
    """Verilen cap icin yorulma emniyet katsayisi."""
    return math.pi * d ** 3 / (16 * (2 * Kf * M / Se + math.sqrt(3) * Kfs * T / Sut))


def von_mises_peak(d, M, T, Kf, Kfs):
    s = 32 * Kf * M / (math.pi * d ** 3)
    t = 16 * Kfs * T / (math.pi * d ** 3)
    return math.sqrt(s * s + 3 * t * t)


def design(M_Nmm, T_Nmm, material="1045", feature="keyseat", n=2.0, ny=1.5):
    """Dondurur: (tasarim, kontroller). kb capa bagli oldugu icin iterasyon; sonra standart capa yukari."""
    mat = SHAFT_MATERIALS[material]
    fname, Kf, Kfs = FEATURES[feature]
    Sut, Sy = mat["Sut"], mat["Sy"]
    d = 20.0
    for _ in range(50):                          # kb(d) <-> d yakinsamasi
        d_new = de_goodman_d(M_Nmm, T_Nmm, Sut, endurance(Sut, d), Kf, Kfs, n)
        if abs(d_new - d) < 1e-6:
            break
        d = d_new
    d_min = d
    d_std = next((x for x in STANDARD_D if x >= d_min - 1e-9), None)
    if d_std is None:
        return None, [{"group": "Çap", "name": "Standart çap", "util": 99.0,
                       "detail": f"gereken Ø{d_min:.1f} mm > Ø{STANDARD_D[-1]} mm"}]
    Se = endurance(Sut, d_std)
    nf = de_goodman_n(d_std, M_Nmm, T_Nmm, Sut, Se, Kf, Kfs)
    sv = von_mises_peak(d_std, M_Nmm, T_Nmm, Kf, Kfs)
    checks = [
        {"group": "Yorulma", "name": "DE-Goodman yorulma emniyeti", "util": n / nf,
         "detail": f"n_f = {nf:.2f} (hedef {n:g}), S_e = {Se:.0f} MPa, K_f/K_fs = {Kf:g}/{Kfs:g} ({fname})"},
        {"group": "Akma", "name": "İlk çevrimde akma (von Mises)", "util": ny * sv / Sy,
         "detail": f"σ' = {sv:.0f} MPa, n_y = {Sy / sv:.2f} (en az {ny:g})"},
    ]
    return {"d_min": d_min, "d": float(d_std), "Se": Se, "nf": nf, "ny": Sy / sv, "Kf": Kf, "Kfs": Kfs,
            "feature": feature, "material": mat["name"], "M": M_Nmm, "T": T_Nmm}, checks


# ------------------------------------------------------------------ geometri (C11b: Fusion + kart)
# DIN 6885-1 paralel kama: (mil capi ust siniri, b, h, t1 mil yuvasi derinligi)  [mm]
KEYS = [(8, 2, 2, 1.2), (10, 3, 3, 1.8), (12, 4, 4, 2.5), (17, 5, 5, 3.0), (22, 6, 6, 3.5), (30, 8, 7, 4.0),
        (38, 10, 8, 5.0), (44, 12, 8, 5.0), (50, 14, 9, 5.5), (58, 16, 10, 6.0), (65, 18, 11, 7.0),
        (75, 20, 12, 7.5), (85, 22, 14, 9.0), (95, 25, 14, 9.0), (110, 28, 16, 10.0), (130, 32, 18, 11.0)]
# Tablo 7-1'in dayandigi oranlar: omuz D/d = 1.5; keskin r/d = 0.02, yuvarlatilmis r/d = 0.1
SHOULDER_RATIO = 1.5
FILLET_RATIO = {"sharp_shoulder": 0.02, "round_shoulder": 0.1}


def key_for(d):
    return next(k for k in KEYS if d <= k[0])[1:]


def groove_for(d):
    """Segman kanali (DIN 471'e yakin): (genislik m, derinlik t)."""
    m = 1.1 if d <= 17 else 1.3 if d <= 25 else 1.6 if d <= 38 else 1.85 if d <= 48 else 2.15
    return m, round(max(0.2, 0.03 * d), 2)


def layout(d, feature, span=None):
    """Iki yatakli, sol ucu tahrikli mil: sol uc (tork girisi) | yatak 1 | gobek oturmasi (aciklik
    ortasi, en buyuk M) | yatak 2. Sadece hesaptaki gerilme yigilmasi cizilir (baska Kt yok).
    Dondurur: {"length", "segments": [(x0, x1, r)], "bearings": [(xc, genislik, dis cap)],
    "hub": (x0, x1), "feature": {...}} (x ekseni mil ekseni, mm)."""
    B = max(6.0, round(0.5 * d))                  # yatak genisligi (gorsel)
    L = float(span) if span else round(5 * d)
    L = max(L, 3 * d + B)                         # gobek + omuz yataklara carpmasin
    stub = round(1.5 * d)
    xb1 = stub + B / 2
    xb2 = xb1 + L
    total = xb2 + B / 2
    xm = (xb1 + xb2) / 2
    Lh = 1.5 * d
    hub = (xm - Lh / 2, xm + Lh / 2)
    r = d / 2
    segs = [(0.0, total, r)]
    feat = {"kind": feature}
    if feature in FILLET_RATIO:
        D = round(SHOULDER_RATIO * d)
        Lc = max(3.0, round(0.25 * d))
        xs = hub[0]                               # omuz yuzu = gobegin dayandigi yuz
        segs = [(0.0, xs - Lc, r), (xs - Lc, xs, D / 2), (xs, total, r)]
        feat.update(x=xs, D=float(D), fillet=round(FILLET_RATIO[feature] * d, 2), collar=Lc)
    elif feature == "ring_groove":
        m, t = groove_for(d)
        xg = hub[1] + 0.5                         # gobegi sagdan tutan segman
        segs = [(0.0, xg, r), (xg, xg + m, r - t), (xg + m, total, r)]
        feat.update(x=xg, width=m, depth=t)
    elif feature == "keyseat":
        b, h, t1 = key_for(d)
        lk = max(2 * b, round(Lh - 0.2 * d))      # kama gobekten kisa
        feat.update(x=xm, length=float(lk), b=b, h=h, t1=t1)
    return {"length": total, "segments": segs, "d": d, "span": L,
            "bearings": [(xb1, B, float(round(1.8 * d))), (xb2, B, float(round(1.8 * d)))],
            "hub": hub, "feature": feat}


def profile(lay):
    """Donel profil (x, r) noktalari, soldan saga ust hat; eksen uzerinden kapatilir. Omuz radyusu
    yay yerine ('arc', merkez, bas, son) olarak isaretlenir."""
    pts = [(0.0, 0.0)]
    feat = lay["feature"]
    for x0, x1, r in lay["segments"]:
        if pts[-1][1] != r:
            if feat.get("fillet") and abs(x0 - feat["x"]) < 1e-9:   # omuz -> kucuk cap: ic kose radyusu
                rf, rs = feat["fillet"], r
                pts.append((x0, rs + rf))
                pts.append(("arc", (x0 + rf, rs + rf), (x0, rs + rf), (x0 + rf, rs)))
                pts.append((x0 + rf, rs))
            else:
                pts.append((x0, r))
        pts.append((x1, r))
    pts.append((lay["length"], 0.0))
    return pts
