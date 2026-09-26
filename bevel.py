"""
G1c: straight bevel gear pair, shafts at 90 deg, 20 deg pressure angle (Tredgold's approximation).

Geometry: pitch cone angles tan(gamma) = z1 / z2, Gamma = 90 - gamma;  module m at the large (outer) end,
    d = m z,  outer cone distance A_o = d1 / (2 sin gamma);  face F <= A_o / 3 and <= 10 m.
Tredgold: the teeth on the back cone are those of a spur gear of z' = z / cos(gamma) teeth (the virtual gear,
    pitch radius r / cos gamma): its Lewis Y, contact ratio and undercut limit (z' >= 17) apply.
Loads at the mean section (cone distance A_o - F/2):  d_m = d (1 - F / (2 A_o)),  W_t = 2 T / d_m,
    pinion radial W_r = W_t tan(phi) cos(gamma), axial W_a = W_t tan(phi) sin(gamma) (the gear's are swapped).
Bending (Lewis at the mean section, module m_m = m (1 - F / (2 A_o))):  sigma = K_v W_t / (F m_m Y(z')),
    K_v = (3.05 + V) / 3.05 (as the spur pair).
Choice: the lightest pair (F (d1^2 + d2^2)).  Units: mm, N, N*mm, MPa.
"""

import math

import gears

FACE_K = (6, 8, 10)
Z1_RANGE = range(12, 41)


def cones(z1, z2):
    g = math.atan2(z1, z2)
    return g, math.pi / 2 - g


def geometry(m, z1, z2, F=None):
    g1, g2 = cones(z1, z2)
    d1, d2 = m * z1, m * z2
    Ao = d1 / (2 * math.sin(g1))
    F = F if F is not None else min(Ao / 3, 10 * m)
    k = 1 - F / (2 * Ao)
    return {"gamma1": g1, "gamma2": g2, "d1": d1, "d2": d2, "Ao": Ao, "F": F, "dm1": d1 * k, "dm2": d2 * k,
            "mm": m * k, "zv1": z1 / math.cos(g1), "zv2": z2 / math.cos(g2)}


def virtual_contact_ratio(m, z1, z2):
    """Contact ratio of the two virtual spur gears (Tredgold) at the large end."""
    g = geometry(m, z1, z2)
    rv1, rv2 = g["d1"] / 2 / math.cos(g["gamma1"]), g["d2"] / 2 / math.cos(g["gamma2"])
    phi = gears.PHI
    rb1, rb2 = rv1 * math.cos(phi), rv2 * math.cos(phi)
    return (math.sqrt((rv1 + m) ** 2 - rb1 ** 2) + math.sqrt((rv2 + m) ** 2 - rb2 ** 2)
            - (rv1 + rv2) * math.sin(phi)) / (math.pi * m * math.cos(phi))


def forces(T1, m, z1, z2, F=None):
    g = geometry(m, z1, z2, F)
    Wt = 2 * T1 / g["dm1"]
    tp = math.tan(gears.PHI)
    return {"Wt": Wt, "Wr1": Wt * tp * math.cos(g["gamma1"]), "Wa1": Wt * tp * math.sin(g["gamma1"]),
            "Wr2": Wt * tp * math.cos(g["gamma2"]), "Wa2": Wt * tp * math.sin(g["gamma2"])}


def check_pair(m, z1, z2, F, T1, mat, rpm=None):
    g = geometry(m, z1, z2, F)
    fo = forces(T1, m, z1, z2, F)
    V = math.pi * g["dm1"] * (rpm or 0) / 60000.0
    Kv = (3.05 + V) / 3.05
    allow = mat["sy"] / mat["n"]
    checks = []
    for name, zv in (("Pinion", g["zv1"]), ("Gear", g["zv2"])):
        Y = gears.lewis_y(zv)
        s = Kv * fo["Wt"] / (F * g["mm"] * Y)
        checks.append({"group": "Bending", "name": f"{name} tooth bending (Lewis, virtual z {zv:.0f})",
                       "util": s / allow, "detail": f"σ = {s:.1f} MPa, Y = {Y:.3f} (mean section)"})
    eps = virtual_contact_ratio(m, z1, z2)
    checks += [{"group": "Contact ratio", "name": "Contact ratio (virtual gears)", "util": 1.2 / eps, "detail": f"ε = {eps:.2f}"},
               {"group": "Geometry", "name": "No undercut (virtual pinion)", "util": gears.z_min() / g["zv1"],
                "detail": f"z' = {g['zv1']:.1f} ≥ {gears.z_min():.1f}"},
               {"group": "Geometry", "name": "Face width", "util": F / min(g["Ao"] / 3, 10 * m),
                "detail": f"F = {F:g} mm ≤ A_o/3 = {g['Ao'] / 3:.1f} and 10 m"}]
    return checks, dict(fo, Kv=Kv, eps=eps)


def design(ratio, T1, mat, rpm=None):
    best = best_any = None
    bore1, bore2 = gears.shaft_bore(T1), gears.shaft_bore(T1 * ratio)
    if bore1 is None or bore2 is None:
        return None, [{"group": "Shaft", "name": "Shaft torsion", "util": 99.0, "detail": "beyond Ø30"}]
    for z1 in Z1_RANGE:
        z2 = max(z1, round(ratio * z1))
        err = abs(z2 / z1 - ratio) / ratio
        if err > 0.02:
            continue
        for m in gears.MODULES:
            g0 = geometry(m, z1, z2)
            for k in FACE_K:
                F = min(k * m, math.floor(g0["Ao"] / 3 * 10) / 10)
                checks, fo = check_pair(m, z1, z2, F, T1, mat, rpm)
                g = geometry(m, z1, z2, F)
                checks += [_hub(m, g["dm1"], g["mm"], bore1, "Pinion"), _hub(m, g["dm2"], g["mm"], bore2, "Gear")]
                worst = max(c["util"] for c in checks)
                vol = F * (g["d1"] ** 2 + g["d2"] ** 2)
                cand = (dict(g, module=m, z1=z1, z2=z2, face_width=F, ratio=z2 / z1, ratio_err_pct=100 * err,
                             gamma1_deg=math.degrees(g["gamma1"]), gamma2_deg=math.degrees(g["gamma2"]),
                             bore1=bore1, bore2=bore2, volume=vol, **fo), checks, worst)
                if worst <= 1.0:
                    if best is None or vol < best[0]["volume"] - 1e-9:
                        best = cand
                    break
                if best_any is None or worst < best_any[2]:
                    best_any = cand
    if best:
        return best[0], best[1]
    return None, (best_any[1] if best_any else [])


def _hub(m, dm, mm_, bore, name):
    """At the small end the root must still clear the bore by 2 modules."""
    rf = dm / 2 - 1.25 * mm_
    return {"group": "Hub", "name": f"{name} hub web", "util": (bore / 2 + 2 * m) / rf,
            "detail": f"shaft Ø{bore:g}, mean root Ø{2 * rf:.1f} mm"}
