"""
H3c: involute spline (DIN 5480) joining a hub to a shaft: 30° pressure angle, flank-centred.

Size W d_B x m x z (reference diameter, module, teeth).  Geometry by the DIN 5480-1 rules:
    d = m z,  profile shift  x m = (d_B - m z - 1.1 m) / 2   with -0.05 <= x <= 0.45  (this fixes z;
          where two z fit, the standard's table takes the even one: W 30x2x14, W 25x1.25x18, W 60x2x28)
    shaft tip  d_a1 = d_B - 0.2 m      hub tip  d_a2 = d_B - 2 m     (working depth 0.9 m)
    shaft root d_f1 = d_B - 2.2 m      hub root d_f2 = d_B + 0.2 m   (approximate: the exact root follows the tool)
    tooth thickness on d:  s = pi m / 2 + 2 x m tan 30°  (nominal, before the 9H / 8f tolerances)
The module is the usual one for the diameter (0.8 up to 15, 1.25 up to 25, 2 up to 60, 3 up to 100, 5 above),
as in the common W 20x1.25, W 40x2, W 80x3 sizes.
Torque carried by the flanks as for ISO 14 (the same allowable pressures, 75 % of the teeth carrying):
    T = 0.75 z (0.9 m) L r_m p_allow,   r_m = (d_B - 1.1 m) / 2
Shaft in torsion on the root diameter:  16 T / (pi d_f1^3) <= 0.5 S_y / 2 (C45).
Choice: the smallest d_B whose hub length L <= 1.5 d_B carries K T.  Units: mm, N, N*mm, MPa.
"""

import math

from spline import K_SERVICE, P_ALLOW, SY

ALPHA = math.radians(30.0)
REF_DIAMETERS = [6, 8, 10, 12, 15, 17, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100,
                 110, 120, 130, 140, 150, 160, 170, 180, 190, 200]


def inv(a):
    return math.tan(a) - a


def module_for(dB):
    return 0.8 if dB <= 15 else 1.25 if dB <= 25 else 2.0 if dB <= 60 else 3.0 if dB <= 100 else 5.0


def teeth(dB, m):
    """z with -0.05 <= x <= 0.45; of two, the even one."""
    lo, hi = math.ceil((dB - 2.0 * m) / m - 1e-9), math.floor((dB - 1.0 * m) / m + 1e-9)
    zs = list(range(lo, hi + 1))
    return next((z for z in zs if z % 2 == 0), zs[0]) if len(zs) > 1 else zs[0]


def geometry(dB, m, z=None):
    z = z or teeth(dB, m)
    x = (dB - m * z - 1.1 * m) / (2 * m)
    d = m * z
    s = math.pi * m / 2 + 2 * x * m * math.tan(ALPHA)
    g = {"dB": dB, "m": m, "z": z, "x": x, "d": d, "db": d * math.cos(ALPHA), "s": s,
         "da1": dB - 0.2 * m, "df1": dB - 2.2 * m, "da2": dB - 2.0 * m, "df2": dB + 0.2 * m}
    g["s_tip"] = thickness_at(g, g["da1"] / 2)
    return g


def half_angle(g, rho):
    """Half the tooth's angular thickness at radius rho (involute flank)."""
    a = math.acos(min(1.0, g["db"] / (2 * rho)))
    return g["s"] / g["d"] + inv(ALPHA) - inv(a)


def thickness_at(g, rho):
    return 2 * rho * half_angle(g, rho)


def capacity(g, L, use="fixed"):
    return 0.75 * g["z"] * 0.9 * g["m"] * L * (g["dB"] - 1.1 * g["m"]) / 2 * P_ALLOW[use]


def design(T, use="fixed", K=K_SERVICE):
    """Smallest DIN 5480 spline carrying K T (N*mm) with a hub no longer than 1.5 d_B. Returns (design, checks)."""
    Td = K * T
    for dB in REF_DIAMETERS:
        g = geometry(dB, module_for(dB))
        tau = 16 * Td / (math.pi * g["df1"] ** 3)
        if tau > 0.5 * SY / 2:
            continue
        L = max(math.ceil(Td / capacity(g, 1.0, use)), round(0.8 * dB))
        if L > 1.5 * dB:
            continue
        cap = capacity(g, L, use)
        checks = [{"group": "Flanks", "name": f"Flank pressure ({use.replace('_', ' ')})", "util": Td / cap,
                   "detail": f"{cap / 1000:.0f} N·m at p = {P_ALLOW[use]:g} MPa over {L} mm for {Td / 1000:.0f} N·m"},
                  {"group": "Shaft", "name": "Shaft torsion (root Ø)", "util": tau / (0.5 * SY / 2),
                   "detail": f"τ = {tau:.0f} MPa on Ø{g['df1']:.1f}"},
                  {"group": "Teeth", "name": "Tooth tip not pointed", "util": 0.2 * g["m"] / g["s_tip"],
                   "detail": f"tip {g['s_tip']:.2f} mm ≥ 0.2 m"},
                  {"group": "Hub", "name": "Hub length", "util": L / (1.5 * dB), "detail": f"L = {L} mm ≤ 1.5 d_B"}]
        return dict(g, profile="involute", L=float(L), T=T, Td=Td, cap=cap, use=use, p=P_ALLOW[use], tau=tau,
                    hub_od=round(1.8 * dB), name=f"W {dB} x {g['m']:g} x {g['z']}"), checks
    return None, [{"group": "Flanks", "name": "Spline size", "util": 9.9,
                   "detail": f"no DIN 5480 spline up to W 200 carries {Td / 1000:.0f} N·m ({use})"}]
