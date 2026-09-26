"""
G1b: worm gear set (steel worm, bronze wheel, shafts at 90 deg), Shigley 15-4 (AGMA method).

Geometry: axial module m of the worm = transverse module of the wheel; N_W starts, N_G teeth;
    d_G = m N_G, d_W = q m (diameter quotient q), C = (d_W + d_G) / 2, lead L = pi m N_W,
    lead angle tan(lambda) = L / (pi d_W);  axial pressure angle 20 deg (tan phi_n = tan phi_x cos lambda);
    AGMA recommends C^0.875 / 3 <= d_W <= C^0.875 / 1.6 (inches): the lower bound is kept (a slender worm
    bends), a thicker worm is only noted (a self-locking, small-lead worm needs one);  wheel face F_G = 2/3 d_W (= F_e).
Speeds: sliding V_s = pi n_W d_W / (12 cos lambda) ft/min.
Friction (15-38):  f = 0.15 (V_s = 0);  0.124 exp(-0.074 V_s^0.645) (V_s <= 10);  0.103 exp(-0.110 V_s^0.45) + 0.012.
Efficiency, worm driving:  e = (cos phi_n - f tan lambda) / (cos phi_n + f cot lambda).
Forces: W_Gt = 2 T_G / d_G (x application factor K_a),  W_Wt = W_Gt (cos phi_n sin lambda + f cos lambda) /
    (cos phi_n cos lambda - f sin lambda),  separating W_r = W_Gt sin phi_n / (cos phi_n cos lambda - f sin lambda).
Rating (15-28):  (W_t)all = C_s d_G^0.8 F_e C_m C_v  (lbf, in)
    C_s (bronze): C <= 3 in: 270 + 10.37 C^3; else sand cast 1000 (d_G <= 2.5) / 1190 - 477 log d_G,
          chilled 1000 (d_G <= 8) / 1412 - 456 log d_G, centrifugal 1000 (d_G <= 25) / 1251 - 180 log d_G
    C_m (ratio m_G): 0.02 sqrt(-m^2 + 40 m - 76) + 0.46 (3 < m <= 20); 0.0107 sqrt(-m^2 + 56 m + 5145) (<= 76);
          1.1483 - 0.00658 m
    C_v: 0.659 exp(-0.0011 V_s) (< 700); 13.31 V_s^-0.571 (<= 3000); 65.52 V_s^-0.774
Self-locking (the wheel cannot turn the worm, at rest): cos(phi_n) tan(lambda) <= 0.6 f_static (f_static = 0.15;
    the 40 % margin because vibration can undo it, i.e. lambda below about 5.5 deg, the usual rule).
Also: N_G >= 21 (20 deg, Shigley Table 15-10), steel shafts, wheel hub.
Choice: the smallest set (d_G^2 F_G + d_W^2 d_G) that passes.  Units outside: mm, N, W, N*mm, rev/min.
"""

import math

import chain
import gears

IN = 25.4
LBF = 4.448
HP = 745.7
PHI_X = math.radians(20.0)
F_STATIC = 0.15
LOCK_MARGIN = 0.6                          # only 60 % of the static friction is counted on (vibration): lambda <~ 5.5 deg
KA = 1.25                                  # application factor (reported)
Q = (8, 10, 12.5, 16, 20)                  # diameter quotients d_W / m (DIN 3976 series)
STARTS = (1, 2, 3, 4)
NG_MIN = 21
CASTING = {"sand": "sand-cast bronze", "chill": "chill-cast bronze", "centrifugal": "centrifugally cast bronze"}


def friction(Vs):
    if Vs <= 0:
        return 0.15
    if Vs <= 10:
        return 0.124 * math.exp(-0.074 * Vs ** 0.645)
    return 0.103 * math.exp(-0.110 * Vs ** 0.45) + 0.012


def c_s(C_in, dG_in, casting="sand"):
    if C_in <= 3:
        return 270 + 10.37 * C_in ** 3
    if casting == "sand":
        return 1000.0 if dG_in <= 2.5 else 1190 - 477 * math.log10(dG_in)
    if casting == "chill":
        return 1000.0 if dG_in <= 8 else 1412 - 456 * math.log10(dG_in)
    return 1000.0 if dG_in <= 25 else 1251 - 180 * math.log10(dG_in)


def c_m(mG):
    if mG <= 20:
        return 0.02 * math.sqrt(-mG * mG + 40 * mG - 76) + 0.46
    if mG <= 76:
        return 0.0107 * math.sqrt(-mG * mG + 56 * mG + 5145)
    return 1.1483 - 0.00658 * mG


def c_v(Vs):
    if Vs < 700:
        return 0.659 * math.exp(-0.0011 * Vs)
    if Vs <= 3000:
        return 13.31 * Vs ** -0.571
    return 65.52 * Vs ** -0.774


def analyse(m, NW, NG, dW, nW, T_G, F=None, casting="sand", phi_x=PHI_X, phi_n=None):
    """Geometry, speeds, forces and rating. m, dW, F in mm; T_G (N*mm) the output torque (x K_a outside)."""
    dG = m * NG
    C = (dW + dG) / 2
    L = math.pi * m * NW
    lam = math.atan(L / (math.pi * dW))
    phin = phi_n if phi_n is not None else math.atan(math.tan(phi_x) * math.cos(lam))
    Vs = math.pi * nW * dW / IN / (12 * math.cos(lam))
    f = friction(Vs)
    cn, sl, cl = math.cos(phin), math.sin(lam), math.cos(lam)
    e = (cn - f * math.tan(lam)) / (cn + f / math.tan(lam))
    WGt = 2 * T_G / dG
    den = cn * cl - f * sl
    WWt = WGt * (cn * sl + f * cl) / den
    Wr = WGt * math.sin(phin) / den
    Fe = min(F if F else 2 / 3 * dW, 2 / 3 * dW)
    allow = c_s(C / IN, dG / IN, casting) * (dG / IN) ** 0.8 * (Fe / IN) * c_m(NG / NW) * c_v(Vs) * LBF
    nG = nW * NW / NG
    return {"dG": dG, "dW": dW, "C": C, "L": L, "lead_deg": math.degrees(lam), "phi_n": math.degrees(phin),
            "Vs_ft_min": Vs, "Vs": Vs * 0.00508, "f": f, "efficiency": e, "WGt": WGt, "WWt": WWt, "Wr": Wr,
            "W_all": allow, "Fe": Fe, "F_G": 2 / 3 * dW, "rpm_out": nG, "self_locking": LOCK_MARGIN * F_STATIC >= cn * math.tan(lam),
            "lock_util": cn * math.tan(lam) / (LOCK_MARGIN * F_STATIC), "Cs": c_s(C / IN, dG / IN, casting), "Cm": c_m(NG / NW),
            "Cv": c_v(Vs)}


def design(ratio, T_out, nW, self_locking=False, casting="sand"):
    """Worm set for output torque T_out (N*mm) at worm speed nW, ratio N_G/N_W. Returns (design, checks)."""
    best = best_any = None
    for NW in STARTS:
        NG = int(round(ratio * NW))
        if NG < NG_MIN or abs(NG / NW - ratio) > 0.02 * ratio:
            continue
        for m in gears.MODULES:
            for q in Q:
                dW = q * m
                a = analyse(m, NW, NG, dW, nW, KA * T_out, casting=casting)
                C_in = a["C"] / IN
                lo, hi = C_in ** 0.875 / 3 * IN, C_in ** 0.875 / 1.6 * IN
                T_W = a["WWt"] * dW / 2                      # worm torque (incl. K_a), N*mm
                bW, bG = gears.shaft_bore(T_W), chain.shaft_bore(KA * T_out)
                checks = [
                    {"group": "Rating", "name": "Wheel tooth load (AGMA, Shigley 15-28)", "util": a["WGt"] / a["W_all"],
                     "detail": f"W_t = {a['WGt']:.0f} N (K_a {KA:g}) vs {a['W_all']:.0f} N allowed ({CASTING[casting]})"},
                    {"group": "Geometry", "name": "Worm not too slender (AGMA)", "util": lo / dW,
                     "detail": f"d_W = {dW:g} mm ≥ {lo:.1f} mm for C = {a['C']:.1f} mm"
                               + ("" if dW <= hi else f" (thicker than AGMA's usual {hi:.1f} mm: needed for a small lead angle)")},
                    {"group": "Geometry", "name": "Wheel teeth", "util": NG_MIN / NG, "detail": f"N_G = {NG} ≥ {NG_MIN}"},
                ]
                if self_locking:
                    checks.append({"group": "Self-locking", "name": "Wheel cannot drive the worm (at rest)",
                                   "util": a["lock_util"],
                                   "detail": f"cos φ_n tan λ = {a['lock_util'] * LOCK_MARGIN * F_STATIC:.3f} ≤ {LOCK_MARGIN:g} × f = {LOCK_MARGIN * F_STATIC:.2f} (λ = {a['lead_deg']:.2f}°)"})
                for name, b, T in (("Worm", bW, T_W), ("Wheel", bG, KA * T_out)):
                    checks.append({"group": "Shafts", "name": f"{name} shaft (steel)", "util": 0.0 if b else 9.9,
                                   "detail": f"Ø{b:g}, {T / 1000:.1f} N·m" if b else f"{T / 1000:.0f} N·m is too much"})
                if bG:
                    checks.append(gears.hub_check(m, NG, bG))
                if bW:
                    checks.append({"group": "Hub", "name": "Worm shaft under the thread root", "util": bW / (dW - 2.5 * m),
                                   "detail": f"Ø{bW:g} vs root Ø{dW - 2.5 * m:.1f} mm"})
                worst = max(c["util"] for c in checks)
                size = a["dG"] ** 2 * a["F_G"] + dW ** 2 * a["dG"]
                cand = (dict(a, module=m, NW=NW, NG=NG, q=q, ratio=NG / NW, T_out=T_out, T_W=T_W / KA,
                             P_out=T_out * 2 * math.pi * a["rpm_out"] / 60000, bore_worm=bW, bore_wheel=bG, Ka=KA,
                             casting=casting, size=size), checks, worst)
                if worst <= 1.0:
                    if best is None or size < best[0]["size"]:
                        best = cand
                elif best_any is None or worst < best_any[2]:
                    best_any = cand
    if best:
        d = best[0]
        d["P_in"] = d["P_out"] / d["efficiency"]
        d["P_loss"] = d["P_in"] - d["P_out"]
        return d, best[1]
    return None, (best_any[1] if best_any else [])


# ---------------------------------------------------------------- geometry for drawing
def worm_section(m, NW, dW, rotate=0.0, n=16, clear=0.0):
    """Transverse section of one start of an Archimedean (ZA) worm, right hand: its axial section is a
    straight-sided rack (axial pressure angle 20 deg, half thickness px/4 - (r - r_W) tan(phi_x) - clear), and a
    point at angle theta is in the thread when its axial position theta L / (2 pi) falls inside that tooth, so
    the section spans the angles +-Theta(r) = +-pi w(r) / L. Sweeping it along the axis with a twist of
    2 pi per lead gives the exact thread. Returns (right flank, left flank, r_in, r_a) as (x, y) lists, the tooth
    centred on angle `rotate`; the flanks run from r_in (just inside the root, joined to the core) to the tip."""
    px = math.pi * m
    L = NW * px
    rW, ra = dW / 2, dW / 2 + m
    r_in = dW / 2 - 1.25 * m - 0.1 * m
    right, left = [], []
    for i in range(n + 1):
        r = r_in + (ra - r_in) * i / n
        half = px / 4 - (r - rW) * math.tan(PHI_X) - clear
        th = math.pi * 2 * half / L
        right.append((r * math.cos(rotate - th), r * math.sin(rotate - th)))
        left.append((r * math.cos(rotate + th), r * math.sin(rotate + th)))
    return right, left, r_in, ra


def worm_length(m, NW, NG, dW):
    """Threaded length: the wheel's tip circle reaches the worm over 2 sqrt(ra_G^2 - (C - ra_W)^2), plus a pitch each end."""
    dG = m * NG
    C = (dW + dG) / 2
    raG, raW = dG / 2 + m, dW / 2 + m
    return 2 * math.sqrt(max(raG ** 2 - (C - raW) ** 2, 0.0)) + 2 * math.pi * m
