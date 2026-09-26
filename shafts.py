"""
C11b: Shaft design: Shigley, Chapters 6-7 (fatigue, DE-Goodman) + bearings, key, deflection, critical speed.

Rotating shaft: fully reversed bending (Ma = M, Mm = 0), steady torque (Tm = T, Ta = 0).
DE-Goodman:  d = [ 16 n / pi * ( 2 Kf Ma / Se + sqrt(3) Kfs Tm / Sut ) ]^(1/3)
             (general: 1/Se sqrt(4 (Kf Ma)^2 + 3 (Kfs Ta)^2) + 1/Sut sqrt(4 (Kf Mm)^2 + 3 (Kfs Tm)^2))
Se = ka kb kc kd ke Se',  Se' = 0.5 Sut (Sut <= 1400 MPa)
  ka = a Sut^b  (machined: a = 4.51, b = -0.265)
  kb = 1.24 d^-0.107 (2.79-51 mm), 1.51 d^-0.157 (51-254 mm)   -> depends on d: iteration
  kc = 1 (bending), kd = 1 (room temperature), ke = 0.897 (90 % reliability)
Stress concentration (first iteration, Shigley Table 7-1; conservative Kf = Kt):
  keyseat (end-milled) Kt 2.14 / Kts 3.0; sharp shoulder 2.7 / 2.2; well-rounded shoulder 1.7 / 1.5;
  retaining-ring groove 5.0 / 3.0; plain shaft 1.0 / 1.0
Yield: von Mises peak  sigma' = sqrt((32 Kf M / (pi d^3))^2 + 3 (16 Kfs T / (pi d^3))^2)  <= Sy / ny

Load model: one radial force F on the hub at mid-span between two bearings (M = F L / 4), torque input
at the left end. The bearing seats are stepped with sharp shoulders (small bearing corner radius:
Table 7-1 "sharp").
Units: N, mm, MPa, N*mm, kg.
"""

import math

SHAFT_MATERIALS = {
    "1020": {"name": "AISI 1020 CD", "Sut": 470.0, "Sy": 390.0},
    "1045": {"name": "AISI 1045 CD", "Sut": 630.0, "Sy": 530.0},
}
FEATURES = {
    "keyseat": ("keyseat", 2.14, 3.0),
    "sharp_shoulder": ("sharp shoulder", 2.7, 2.2),
    "round_shoulder": ("rounded shoulder", 1.7, 1.5),
    "ring_groove": ("retaining-ring groove", 5.0, 3.0),
    "plain": ("plain shaft", 1.0, 1.0),
}
STANDARD_D = [6, 8, 10, 12, 14, 15, 16, 17, 18, 20, 22, 25, 28, 30, 32, 35, 38, 40, 42, 45, 48, 50,
              55, 60, 65, 70, 75, 80, 90, 100, 110, 120]
KE_90 = 0.897
E_STEEL = 207000.0           # MPa
RHO_STEEL = 7.85e-6          # kg/mm^3

# Deep-groove ball bearings (ISO 15 dimensions; C: basic dynamic load rating, kN, approximate SKF values)
# (name, bore, outer diameter, width, C)
BEARINGS = [
    ("6200", 10, 30, 9, 5.07), ("6201", 12, 32, 10, 6.89), ("6202", 15, 35, 11, 7.8), ("6203", 17, 40, 12, 9.56),
    ("6204", 20, 47, 14, 13.5), ("6205", 25, 52, 15, 14.8), ("6206", 30, 62, 16, 20.3), ("6207", 35, 72, 17, 27.0),
    ("6208", 40, 80, 18, 32.5), ("6209", 45, 85, 19, 35.1), ("6210", 50, 90, 20, 37.1), ("6211", 55, 100, 21, 46.2),
    ("6212", 60, 110, 22, 55.3), ("6213", 65, 120, 23, 58.5), ("6214", 70, 125, 24, 63.7), ("6215", 75, 130, 25, 68.9),
    ("6216", 80, 140, 26, 72.8),
    ("6300", 10, 35, 11, 8.52), ("6301", 12, 37, 12, 12.5), ("6302", 15, 42, 13, 11.9), ("6303", 17, 47, 14, 14.3),
    ("6304", 20, 52, 15, 16.8), ("6305", 25, 62, 17, 23.4), ("6306", 30, 72, 19, 29.6), ("6307", 35, 80, 21, 35.1),
    ("6308", 40, 90, 23, 42.3), ("6309", 45, 100, 25, 55.3), ("6310", 50, 110, 27, 65.0), ("6311", 55, 120, 29, 74.1),
    ("6312", 60, 130, 31, 85.2),
]
SHOULDER_FRAC = 0.2          # bearing shoulder diameter da_min ~ bore + 0.2 (OD - bore) (catalogue: to hold the bearing)
LIFE_H = 20000.0             # target L10 life, hours (Shigley Table 11-4: machines in full-day use, 20-30 thousand h)
SLOPE_MAX = 0.001            # rad, slope at a deep-groove ball bearing (Shigley Table 7-2, 0.001-0.003; lower bound)
DEFL_MAX = 0.13              # mm, deflection at the hub (Table 7-2, spur gear, module ~1.3-2.5)
CRIT_RATIO = 2.0             # critical speed >= 2 x running speed (Shigley 7-6)
KEY_SY = 390.0               # key material AISI 1020 CD (Shigley example 7-6)
KEY_LENGTHS = [6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40, 45, 50, 56, 63, 70, 80, 90, 100, 110, 125, 140]
KEY_MAX_RATIO = 1.5          # key length <= 1.5 d (a longer one does not share the load; Shigley 7-7)


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
    """Fatigue safety factor of a given diameter."""
    return math.pi * d ** 3 / (16 * (2 * Kf * M / Se + math.sqrt(3) * Kfs * T / Sut))


def von_mises_peak(d, M, T, Kf, Kfs):
    s = 32 * Kf * M / (math.pi * d ** 3)
    t = 16 * Kfs * T / (math.pi * d ** 3)
    return math.sqrt(s * s + 3 * t * t)


# ------------------------------------------------------------------ key (DIN 6885)
# DIN 6885-1 parallel key: (upper shaft diameter, b, h, t1 keyseat depth in the shaft)  [mm]
KEYS = [(8, 2, 2, 1.2), (10, 3, 3, 1.8), (12, 4, 4, 2.5), (17, 5, 5, 3.0), (22, 6, 6, 3.5), (30, 8, 7, 4.0),
        (38, 10, 8, 5.0), (44, 12, 8, 5.0), (50, 14, 9, 5.5), (58, 16, 10, 6.0), (65, 18, 11, 7.0),
        (75, 20, 12, 7.5), (85, 22, 14, 9.0), (95, 25, 14, 9.0), (110, 28, 16, 10.0), (130, 32, 18, 11.0)]


def key_for(d):
    return next(k for k in KEYS if d <= k[0])[1:]


def key_design(T, d, ny):
    """Key length from shear (tau = F / (b l) <= 0.577 Sy / ny) and crushing (F / (l t) <= Sy / ny, t = the shallow
    side of the seat), F = 2T/d. At least d, rounded up to a standard length.
    Returns (key dict, checks) or (None, checks)."""
    b, h, t1 = key_for(d)
    F = 2 * T / d
    t = min(t1, h - t1)                            # the hub side (h - t1) is usually the shallower one
    need = max(F * ny / (b * 0.577 * KEY_SY), F * ny / (t * KEY_SY))
    length = next((L for L in KEY_LENGTHS if L >= max(need, d)), None)
    if length is None or length > KEY_MAX_RATIO * d:     # one key is not enough at this d -> next diameter
        return None, [{"group": "Key", "name": "Key length", "util": need / (KEY_MAX_RATIO * d),
                       "detail": f"needs {need:.0f} mm > 1.5·d = {KEY_MAX_RATIO * d:.0f} mm"}]
    n_shear = 0.577 * KEY_SY * b * length / F
    n_crush = KEY_SY * t * length / F
    checks = [
        {"group": "Key", "name": "Key shear", "util": ny / n_shear,
         "detail": f"{b}×{h}×{length} (DIN 6885, 1020 CD), F = {F:.0f} N, n = {n_shear:.1f}"},
        {"group": "Key", "name": "Key crushing", "util": ny / n_crush,
         "detail": f"contact height {t:g} mm, n = {n_crush:.1f} (at least {ny:g})"},
    ]
    return {"b": b, "h": h, "t1": t1, "length": float(length), "F": F}, checks


# ------------------------------------------------------------------ bearings and bearing seats

def bearing_candidates(d):
    """Bearings that fit a shaft with this hub diameter (the shoulder must hold them): largest bore of each series."""
    out = []
    for series in ("62", "63"):
        fits = [b for b in BEARINGS if b[0].startswith(series) and b[1] + SHOULDER_FRAC * (b[2] - b[1]) <= d + 1e-9]
        if fits:
            out.append(max(fits, key=lambda b: b[1]))
    return out


def l10_hours(C_kN, P_N, rpm):
    if P_N <= 0:
        return 1e9                                          # no radial load: the life is not limited by it
    return (C_kN * 1000.0 / P_N) ** 3 * 1e6 / (60.0 * rpm)


def journal_checks(brg, R, T, mat, n, ny, rpm):
    """Left bearing seat (it also carries the torque): M = R B/2 at the shoulder, sharp-shoulder Kt; and bearing life."""
    name, bore, OD, B, C = brg
    Kf, Kfs = FEATURES["sharp_shoulder"][1:]
    M = R * B / 2
    Se = endurance(mat["Sut"], bore)
    nf = de_goodman_n(bore, M, T, mat["Sut"], Se, Kf, Kfs)
    sv = von_mises_peak(bore, M, T, Kf, Kfs)
    checks = [
        {"group": "Bearing", "name": "Bearing seat fatigue", "util": n / nf,
         "detail": f"Ø{bore} sharp shoulder (K_f {Kf:g}/{Kfs:g}), M = {M / 1000:.1f} N·m, n_f = {nf:.2f}"},
        {"group": "Bearing", "name": "Bearing seat yield", "util": ny * sv / mat["Sy"],
         "detail": f"σ' = {sv:.0f} MPa, n_y = {mat['Sy'] / sv:.2f}"},
    ]
    if rpm:
        life = l10_hours(C, R, rpm)
        checks.append({"group": "Bearing", "name": f"Bearing life ({name})", "util": LIFE_H / life,
                       "detail": f"L10 = {life:,.0f} h (target {LIFE_H:,.0f}), P = {R:.0f} N, C = {C:g} kN, {rpm:.0f} rpm"})
    return checks


# ------------------------------------------------------------------ geometry

def groove_for(d):
    """Retaining-ring groove (close to DIN 471): (width m, depth t)."""
    m = 1.1 if d <= 17 else 1.3 if d <= 25 else 1.6 if d <= 38 else 1.85 if d <= 48 else 2.15
    return m, round(max(0.2, 0.03 * d), 2)


# Ratios behind Table 7-1: shoulder D/d = 1.5; sharp r/d = 0.02, well-rounded r/d = 0.1
SHOULDER_RATIO = 1.5
FILLET_RATIO = {"sharp_shoulder": 0.02, "round_shoulder": 0.1}


def _set_radius(segs, x0, x1, r):
    """Give the interval [x0, x1] radius r (splitting segments where needed)."""
    out = []
    for a, b, rr in segs:
        if b <= x0 or a >= x1:
            out.append((a, b, rr))
            continue
        if a < x0:
            out.append((a, x0, rr))
        out.append((max(a, x0), min(b, x1), r))
        if b > x1:
            out.append((x1, b, rr))
    return out


def layout(d, feature, span=None, bearing=None, key_len=None, hub_len=None):
    """Two-bearing shaft driven at the left end: left end (torque input) | bearing 1 | hub seat (mid-span,
    largest M) | bearing 2. The stress raisers drawn are exactly the ones analysed (hub feature +
    bearing shoulders). bearing: a BEARINGS row (else a visual bearing with bore = d).
    Returns {"length", "segments": [(x0, x1, r)], "bearings": [(xc, width, OD)], "journal",
    "hub": (x0, x1), "feature": {...}, "span", "d"} (x axis = shaft axis, mm)."""
    if bearing:
        bname, bore, OD, B = bearing[0], float(bearing[1]), float(bearing[2]), float(bearing[3])
    else:
        bname, bore, OD, B = None, float(d), float(round(1.8 * d)), max(6.0, round(0.5 * d))
    Lh = max(1.5 * d, (key_len or 0) + 0.2 * d, hub_len or 0)   # hub seat longer than the key (and the part on it)
    Lc = max(3.0, round(0.25 * d))
    L = float(span) if span else round(5 * d)
    L = max(L, Lh + 2 * Lc + B + 0.5 * d)         # hub + collar must not reach the bearings
    stub = round(1.5 * d)
    xb1 = stub + B / 2
    xb2 = xb1 + L
    total = xb2 + B / 2
    xm = (xb1 + xb2) / 2
    hub = (xm - Lh / 2, xm + Lh / 2)
    r = d / 2
    segs = [(0.0, total, r)]
    if bore < d:                                  # bearing seats (shoulder at the inner face of the bearing)
        segs = _set_radius(segs, 0.0, xb1 + B / 2, bore / 2)
        segs = _set_radius(segs, xb2 - B / 2, total, bore / 2)
    feat = {"kind": feature}
    if feature in FILLET_RATIO:
        D = round(SHOULDER_RATIO * d)
        xs = hub[0]                               # shoulder face = the face the hub rests against
        segs = _set_radius(segs, xs - Lc, xs, D / 2)
        feat.update(x=xs, D=float(D), fillet=round(FILLET_RATIO[feature] * d, 2), collar=Lc)
    elif feature == "ring_groove":
        m, t = groove_for(d)
        xg = hub[1] + 0.5                         # ring holding the hub from the right
        segs = _set_radius(segs, xg, xg + m, r - t)
        feat.update(x=xg, width=m, depth=t)
    elif feature == "keyseat":
        b, h, t1 = key_for(d)
        lk = float(key_len) if key_len else max(2 * b, round(Lh - 0.2 * d))
        feat.update(x=xm, length=lk, b=b, h=h, t1=t1)
    return {"length": total, "segments": segs, "d": d, "span": L, "journal": bore, "bearing_name": bname,
            "bearings": [(xb1, B, OD), (xb2, B, OD)], "hub": hub, "feature": feat}


def profile(lay):
    """Revolve profile (x, r) points, upper outline from left to right; closed along the axis. A shoulder
    fillet is marked as ('arc', centre, start, end)."""
    pts = [(0.0, 0.0)]
    feat = lay["feature"]
    for x0, x1, r in lay["segments"]:
        if pts[-1][1] != r:
            if feat.get("fillet") and abs(x0 - feat["x"]) < 1e-9:   # shoulder -> smaller diameter: inner-corner fillet
                rf, rs = feat["fillet"], r
                pts.append((x0, rs + rf))
                pts.append(("arc", (x0 + rf, rs + rf), (x0, rs + rf), (x0 + rf, rs)))
                pts.append((x0 + rf, rs))
            else:
                pts.append((x0, r))
        pts.append((x1, r))
    pts.append((lay["length"], 0.0))
    return pts


# ------------------------------------------------------------------ deflection and critical speed

def radius_at(lay, x):
    for a, b, r in lay["segments"]:
        if a <= x <= b:
            return r
    return lay["segments"][-1][2]


def beam(lay, F, masses=(), N=4000):
    """Stepped shaft simply supported by two bearings, F at mid-span: M/(EI) integrated twice numerically
    (Shigley 4-10 method), y(bearings) = 0. Critical speed: Rayleigh with this static deflection curve as
    the shape: w^2 = F y(at F) / sum(m y^2) (shaft mass + masses [(x, kg)]; the overhangs are not included).
    Returns {"slope1", "slope2" (rad), "y_hub" (mm), "n_crit" (rpm)}."""
    (xb1, _, _), (xb2, _, _) = lay["bearings"]
    L = xb2 - xb1
    xm = (xb1 + xb2) / 2
    h = L / N
    xs = [xb1 + i * h for i in range(N + 1)]
    kap = []                                      # curvature under a unit load (1 N): the shape does not depend on F; scaled by F later
    for x in xs:
        M = 0.5 * (x - xb1) if x <= xm else 0.5 * (xb2 - x)
        I = math.pi * (2 * radius_at(lay, x)) ** 4 / 64
        kap.append(M / (E_STEEL * I))
    th, y = [0.0], [0.0]
    for i in range(N):
        th.append(th[-1] + (kap[i] + kap[i + 1]) / 2 * h)
        y.append(y[-1] + (th[i] + th[i + 1]) / 2 * h)
    c1 = -y[-1] / L                               # y(xb2) = 0
    th = [t + c1 for t in th]
    y = [yy + c1 * (x - xb1) for yy, x in zip(y, xs)]
    y1 = abs(y[N // 2])                           # hub deflection at 1 N
    mass_term = sum(RHO_STEEL * math.pi * radius_at(lay, x) ** 2 * h * yy ** 2 for x, yy in zip(xs, y))
    for xmass, m in masses:
        i = min(N, max(0, round((xmass - xb1) / h)))
        mass_term += m * y[i] ** 2
    w = math.sqrt(1.0 * y1 * 1e-3 / (mass_term * 1e-6))
    return {"slope1": F * abs(th[0]), "slope2": F * abs(th[-1]), "y_hub": F * y1, "n_crit": w * 60 / (2 * math.pi)}


# ------------------------------------------------------------------ design

def _hub_checks(d, M, T, mat, feature, n, ny):
    fname, Kf, Kfs = FEATURES[feature]
    Se = endurance(mat["Sut"], d)
    nf = de_goodman_n(d, M, T, mat["Sut"], Se, Kf, Kfs)
    sv = von_mises_peak(d, M, T, Kf, Kfs)
    return Se, nf, sv, [
        {"group": "Fatigue", "name": "DE-Goodman fatigue safety", "util": n / nf,
         "detail": f"n_f = {nf:.2f} (target {n:g}), S_e = {Se:.0f} MPa, K_f/K_fs = {Kf:g}/{Kfs:g} ({fname})"},
        {"group": "Yield", "name": "First-cycle yield (von Mises)", "util": ny * sv / mat["Sy"],
         "detail": f"σ' = {sv:.0f} MPa, n_y = {mat['Sy'] / sv:.2f} (at least {ny:g})"},
    ]


def design(M_Nmm, T_Nmm, material="1045", feature="keyseat", n=2.0, ny=1.5, span=None, rpm=None,
           F_N=None, hub_len=None, hub_mass=0.0):
    """Returns (design, checks). First the hub section (DE-Goodman, kb <-> d iteration) -> smallest standard
    diameter; then bearings, key, deflection and critical speed at that diameter. If one fails, the next
    standard diameter (Shigley's iterative shaft design). If none works: (None, checks of the last one).
    From a system (E1): instead of M, the radial force F_N on the hub (M = F L / 4, L = the shaft's span),
    the length hub_len and the mass hub_mass (kg, critical speed) of the part on the hub."""
    mat = SHAFT_MATERIALS[material]
    _, Kf, Kfs = FEATURES[feature]
    d = 20.0
    for _ in range(50):                          # kb(d) <-> d (and, with F given, span) convergence
        M_it = F_N * layout(d, feature, span, hub_len=hub_len)["span"] / 4 if F_N is not None else M_Nmm
        d_new = de_goodman_d(M_it, T_Nmm, mat["Sut"], endurance(mat["Sut"], d), Kf, Kfs, n)
        if abs(d_new - d) < 1e-6:
            break
        d = d_new
    d_min = d
    last = [{"group": "Diameter", "name": "Standard diameter", "util": 99.0,
             "detail": f"needs Ø{d_min:.1f} mm > Ø{STANDARD_D[-1]} mm"}]
    raised_by = None                              # checks failing at the first diameter tried (why it was raised)
    for d_std in (x for x in STANDARD_D if x >= d_min - 1e-9):
        g, checks = _evaluate(float(d_std), M_Nmm, T_Nmm, mat, feature, n, ny, span, rpm, F_N, hub_len, hub_mass)
        last = checks
        if g is not None and all(c["util"] <= 1.0 for c in checks):
            g.update(d_min=d_min, raised=raised_by is not None, raised_by=raised_by or [])
            return g, checks
        if raised_by is None:
            raised_by = [c["name"] for c in checks if c["util"] > 1.0]
    return None, last


def _evaluate(d, M, T, mat, feature, n, ny, span, rpm, F_N=None, hub_len=None, hub_mass=0.0):
    checks, key = [], None
    if feature == "keyseat":
        key, kc = key_design(T, d, ny)
        checks += kc
        if key is None:
            return None, checks
    klen = key and key["length"]
    L0 = layout(d, feature, span, key_len=klen, hub_len=hub_len)["span"]   # span before the bearing is known
    F = F_N if F_N is not None else 4 * M / L0    # force at mid-span: M = F L / 4
    R = F / 2
    brg, jc = None, []
    for cand in bearing_candidates(d):
        jc = journal_checks(cand, R, T, mat, n, ny, rpm)
        if all(c["util"] <= 1.0 for c in jc):
            brg = cand
            break
    lay = layout(d, feature, span, bearing=brg, key_len=klen, hub_len=hub_len)
    if F_N is not None:
        M = F_N * lay["span"] / 4
    else:
        F = 4 * M / lay["span"]
    Se, nf, sv, hub = _hub_checks(d, M, T, mat, feature, n, ny)
    checks = hub + checks
    if brg is None:
        checks += jc or [{"group": "Bearing", "name": "Bearing selection", "util": 99.0,
                          "detail": f"no bearing fits a Ø{d:g} shaft (catalogue Ø10–Ø80)"}]
        return None, checks
    checks += jc
    xm = sum(b[0] for b in lay["bearings"]) / 2
    bm = beam(lay, F, [(xm, hub_mass)] if hub_mass else ())
    checks += [
        {"group": "Deflection", "name": "Slope at the bearings", "util": max(bm["slope1"], bm["slope2"]) / SLOPE_MAX,
         "detail": f"{max(bm['slope1'], bm['slope2']) * 1000:.2f} mrad (deep-groove ball limit {SLOPE_MAX * 1000:g} mrad)"},
        {"group": "Deflection", "name": "Deflection at the hub", "util": bm["y_hub"] / DEFL_MAX,
         "detail": f"{bm['y_hub'] * 1000:.1f} µm (spur gear limit {DEFL_MAX * 1000:.0f} µm)"},
    ]
    if rpm:
        extra = f", incl. {hub_mass:.2f} kg on the hub" if hub_mass else ""
        checks.append({"group": "Vibration", "name": "Critical speed", "util": CRIT_RATIO * rpm / bm["n_crit"],
                       "detail": f"{bm['n_crit']:,.0f} rpm (at least {CRIT_RATIO:g}× the running {rpm:.0f} rpm{extra})"})
    return {"d": d, "Se": Se, "nf": nf, "ny": mat["Sy"] / sv, "Kf": FEATURES[feature][1], "Kfs": FEATURES[feature][2],
            "feature": feature, "material": mat["name"], "M": M, "T": T, "F": F, "rpm": rpm, "hub_mass": hub_mass,
            "bearing": {"name": brg[0], "bore": brg[1], "OD": brg[2], "B": brg[3], "C": brg[4],
                        "life_h": l10_hours(brg[4], F / 2, rpm) if rpm else None},
            "key": key, "beam": bm, "layout": lay}, checks
