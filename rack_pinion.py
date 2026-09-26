"""
G4: rack and pinion (20 deg full-depth involute pinion on a straight-flanked rack).

Request: the force on the rack F, its travel S, optional pinion speed.
    pinion torque T = F d / 2,  rack travel per pinion turn = pi d,  d = m z
Checks (as for spur gears, gears.py):
    Lewis bending of the pinion (Y(z)) and of the rack (Y = 0.485, the rack row of Shigley Table 14-2), Barth K_v
    contact ratio with a rack:  eps = (sqrt(ra^2 - rb^2) - r sin(phi) + m / sin(phi)) / (pi m cos(phi)) >= 1.2
    no undercut: z >= 18 (2 / sin^2 phi = 17.1);  the pinion's steel shaft in torsion and its hub web
Choice: the smallest volume (pinion disk + rack bar over the travel).
Units: mm, N, MPa, N*mm.
"""

import math

import gears

Z_RANGE = range(18, 41)
Y_RACK = 0.485
BACK = 2.0                   # rack back thickness below the root line, in modules (at least 5 mm)


def contact_ratio_rack(m, z, phi=gears.PHI):
    r = m * z / 2
    ra, rb = r + m, r * math.cos(phi)
    return (math.sqrt(ra * ra - rb * rb) - r * math.sin(phi) + m / math.sin(phi)) / (math.pi * m * math.cos(phi))


def check(m, z, b, F, mat, rpm=None, bore=None):
    d = m * z
    V = math.pi * d * (rpm or 0) / 60000.0
    Kv = (3.05 + V) / 3.05
    allow = mat["sy"] / mat["n"]
    out = []
    for name, Y in (("Pinion", gears.lewis_y(z)), ("Rack", Y_RACK)):
        s = Kv * F / (b * m * Y)
        out.append({"group": "Bending", "name": f"{name} tooth bending (Lewis)", "util": s / allow,
                    "detail": f"σ = {s:.1f} MPa, Y = {Y:.3f}"})
    eps = contact_ratio_rack(m, z)
    out.append({"group": "Contact ratio", "name": "Contact ratio (pinion on rack)", "util": 1.2 / eps,
                "detail": f"ε = {eps:.2f} (at least 1.2)"})
    if bore:
        out.append(gears.hub_check(m, z, bore))
    return out


def design(F, S, mat, rpm=None):
    """Lightest pinion + rack for rack force F (N) over travel S (mm). Returns (design, checks)."""
    best = best_any = None
    for z in Z_RANGE:
        for m in gears.MODULES:
            d = m * z
            T = F * d / 2
            bore = gears.shaft_bore(T)
            if bore is None:
                continue
            for k in gears.FACE_K:
                b = k * m
                c = check(m, z, b, F, mat, rpm, bore)
                worst = max(x["util"] for x in c)
                back = max(BACK * m, 5.0)
                vol = b * math.pi * d * d / 4 + b * (2.25 * m + back) * (S + d)
                cand = ({"module": m, "z": z, "d": d, "face_width": b, "T": T, "bore": bore, "back": back,
                         "eps": contact_ratio_rack(m, z), "per_turn": math.pi * d, "turns": S / (math.pi * d),
                         "rack_len": S + d + 4 * math.pi * m, "volume": vol}, c, worst)
                if worst <= 1.0:
                    if best is None or vol < best[0]["volume"]:
                        best = cand
                    break
                if best_any is None or worst < best_any[2]:
                    best_any = cand
    if best:
        return best[0], best[1]
    return None, (best_any[1] if best_any else [])


def rack_outline(m, d, length, back, phi=gears.PHI):
    """Closed rack outline (pitch line y = -d/2, teeth pointing +y), a tooth SPACE centred at x = 0.
    Points counter-clockwise from the bottom left."""
    r = d / 2
    y_f, y_a = -r - 1.25 * m, -r + m
    p = math.pi * m
    wr, wt = p / 4 + 1.25 * m * math.tan(phi), p / 4 - m * math.tan(phi)
    n = int(math.ceil(length / 2 / p)) + 1
    xs = [p / 2 + k * p for k in range(-n, n)]                          # tooth centres (a space at x = 0)
    x0, x1 = xs[0] - p / 2, xs[-1] + p / 2
    pts = [(x0, y_f - back), (x1, y_f - back), (x1, y_f)]
    for xc in reversed(xs):
        pts += [(xc + wr, y_f), (xc + wt, y_a), (xc - wt, y_a), (xc - wr, y_f)]
    pts.append((x0, y_f))
    return pts
