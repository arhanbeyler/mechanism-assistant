"""
G1a: helical gear pair on parallel shafts (Shigley ch. 13-14), 20 deg normal pressure angle.

Normal module m_n (standard), helix angle psi (default 20 deg, reported); the pair has opposite hands.
    transverse module m_t = m_n / cos psi,  pressure angle tan phi_t = tan phi_n / cos psi
    pitch diameter d = z m_t,  centre distance a = m_n (z1 + z2) / (2 cos psi)
    addendum m_n, dedendum 1.25 m_n (both in the normal plane)
Loads: tangential W_t = 2 T / d1, axial thrust W_a = W_t tan psi, radial W_r = W_t tan phi_t.
Checks: Lewis bending in the normal plane: normal module and Y of the virtual tooth number z / cos^3 psi,
    K_v = (5.56 + sqrt V) / 5.56 (hobbed);  transverse contact ratio (transverse plane, addendum m_n);
    face (axial) contact ratio  F sin psi / (pi m_n) >= 1 so the helix really overlaps;
    no undercut  z >= 2 cos psi / sin^2 phi_t;  steel shafts and hubs (as spur gears).
Choice: the lightest pair (F (d1^2 + d2^2)).  Units: mm, N, N*mm, MPa.
"""

import math

import gears

PHI_N = gears.PHI
PSI_DEFAULT = 20.0
FACE_K = (8, 10, 12, 14, 16)          # face width in normal modules (helical gears run wider than spur)
Z1_RANGE = range(10, 41)


def geometry(mn, z1, z2, psi_deg):
    psi = math.radians(psi_deg)
    mt = mn / math.cos(psi)
    phit = math.atan(math.tan(PHI_N) / math.cos(psi))
    return {"mt": mt, "phi_t": phit, "d1": z1 * mt, "d2": z2 * mt, "a": mn * (z1 + z2) / (2 * math.cos(psi)),
            "px": math.pi * mn / math.sin(psi), "psi": psi}


def z_min(psi_deg):
    psi = math.radians(psi_deg)
    phit = math.atan(math.tan(PHI_N) / math.cos(psi))
    return 2 * math.cos(psi) / math.sin(phit) ** 2


def transverse_contact_ratio(mn, z1, z2, psi_deg):
    g = geometry(mn, z1, z2, psi_deg)
    r1, r2 = g["d1"] / 2, g["d2"] / 2
    rb1, rb2 = r1 * math.cos(g["phi_t"]), r2 * math.cos(g["phi_t"])
    ra1, ra2 = r1 + mn, r2 + mn
    pt = math.pi * g["mt"]
    return (math.sqrt(ra1 ** 2 - rb1 ** 2) + math.sqrt(ra2 ** 2 - rb2 ** 2) - g["a"] * math.sin(g["phi_t"])) / (pt * math.cos(g["phi_t"]))


def face_contact_ratio(mn, F, psi_deg):
    return F * math.sin(math.radians(psi_deg)) / (math.pi * mn)


def check_pair(mn, z1, z2, F, T1, mat, psi_deg, rpm=None):
    g = geometry(mn, z1, z2, psi_deg)
    Wt = 2 * T1 / g["d1"]
    V = math.pi * g["d1"] * (rpm or 0) / 60000.0
    Kv = (5.56 + math.sqrt(V)) / 5.56
    allow = mat["sy"] / mat["n"]
    cos3 = math.cos(g["psi"]) ** 3
    checks = []
    for name, z in (("Pinion", z1), ("Gear", z2)):
        Y = gears.lewis_y(z / cos3)
        s = Kv * Wt / (F * mn * Y)
        checks.append({"group": "Bending", "name": f"{name} tooth bending (Lewis, virtual z {z / cos3:.0f})",
                       "util": s / allow, "detail": f"σ = {s:.1f} MPa, Y = {Y:.3f}"})
    et, ef = transverse_contact_ratio(mn, z1, z2, psi_deg), face_contact_ratio(mn, F, psi_deg)
    checks += [{"group": "Contact ratio", "name": "Transverse contact ratio", "util": 1.2 / et, "detail": f"ε_α = {et:.2f}"},
               {"group": "Contact ratio", "name": "Face (axial) contact ratio", "util": 1.0 / ef,
                "detail": f"ε_β = {ef:.2f} (at least 1: the helix overlaps)"},
               {"group": "Geometry", "name": "No undercut", "util": z_min(psi_deg) / z1,
                "detail": f"z₁ = {z1} ≥ {z_min(psi_deg):.1f}"}]
    return checks, {"Wt": Wt, "Wa": Wt * math.tan(g["psi"]), "Wr": Wt * math.tan(g["phi_t"]), "Kv": Kv,
                    "eps_t": et, "eps_f": ef}


def design(ratio, T1, mat, rpm=None, psi_deg=PSI_DEFAULT):
    """Lightest helical pair. Returns (design, checks)."""
    best = best_any = None
    bore1, bore2 = gears.shaft_bore(T1), gears.shaft_bore(T1 * ratio)
    if bore1 is None or bore2 is None:
        return None, [{"group": "Shaft", "name": "Shaft torsion", "util": 99.0,
                       "detail": f"a Ø{gears.BORES[-1]} mm steel shaft is not enough for {T1 * ratio / 1000:.0f} N·m"}]
    for z1 in Z1_RANGE:
        z2 = max(z1, round(ratio * z1))
        err = abs(z2 / z1 - ratio) / ratio
        if err > 0.02:
            continue
        for mn in gears.MODULES:
            g = geometry(mn, z1, z2, psi_deg)
            for k in FACE_K:
                F = k * mn
                checks, loads = check_pair(mn, z1, z2, F, T1, mat, psi_deg, rpm)
                checks += [_hub(mn, g["d1"], bore1, "Pinion"), _hub(mn, g["d2"], bore2, "Gear")]
                worst = max(c["util"] for c in checks)
                vol = F * (g["d1"] ** 2 + g["d2"] ** 2)
                cand = ({"module": mn, "mt": g["mt"], "psi": psi_deg, "phi_t": math.degrees(g["phi_t"]), "z1": z1, "z2": z2,
                         "face_width": F, "ratio": z2 / z1, "ratio_err_pct": 100 * err, "d1": g["d1"], "d2": g["d2"],
                         "center_distance": g["a"], "bore1": bore1, "bore2": bore2, "volume": vol, **loads}, checks, worst)
                if worst <= 1.0:
                    if best is None or vol < best[0]["volume"] - 1e-9:
                        best = cand
                    break
                if best_any is None or worst < best_any[2]:
                    best_any = cand
    if best:
        return best[0], best[1]
    return None, (best_any[1] if best_any else [])


def _hub(mn, d, bore, name):
    rf = d / 2 - 1.25 * mn
    return {"group": "Hub", "name": f"{name} hub web", "util": (bore / 2 + 2 * mn) / rf,
            "detail": f"shaft Ø{bore:g}, root Ø{2 * rf:.1f} mm"}
