"""
F4c: Whitworth quick return. The crank (radius r, pivot O2 = (0, d)) is LONGER than the distance d to the lever
pivot O (origin), so the slotted lever turns full circles, fast on one half-turn and slow on the other. A rod
from the lever's other arm (radius R) drives the ram along the x axis (a slider-crank on the lever).

Lever angle phi = atan2(P) with the crank pin P = (r cos th, d + r sin th). The lever passes phi = 0 and 180 deg
(the ram's extremes, the rod in line) when P crosses the x axis: sin th = -d / r, so the lever's upper half-turn
takes the crank 180 + 2 alpha and the lower one 180 - 2 alpha, sin(alpha) = d / r:
    Q = (180 + 2 alpha) / (180 - 2 alpha)   ->  alpha = 90 (Q - 1) / (Q + 1),  d = r sin(alpha)
Ram: the rod pin on the lever at angle phi + 180 (the arm opposite the slot), x = R cos(phi + pi) + sqrt(l^2 - (R sin)^2)
stroke 2 R.  Choice: R = S / 2, rod l = 3.5 R, crank r = 0.8 R (reported); the lever's slot spans r - d .. r + d.
Everything is checked by simulation (stroke, Q). Statics: a cutting force F on the slow stroke; crank torque by
virtual work T = F |dx/dtheta|; the rod force F / cos(rod angle).  Units: mm, N, N*mm, degrees.
"""

import math

import slotted
import strength

N_SIM = 1440
ROD = 3.5
CRANK = 0.8


def geometry(S, Q):
    alpha = math.radians(90 * (Q - 1) / (Q + 1))
    R = S / 2
    r = CRANK * R
    return {"r": r, "d": r * math.sin(alpha), "R": R, "l": ROD * R, "alpha": math.degrees(alpha), "Q": Q, "stroke": S}


def frames(g, n=N_SIM):
    r, d, R, l = g["r"], g["d"], g["R"], g["l"]
    out, prev = [], None
    for i in range(n):
        th = 2 * math.pi * i / n
        P = (r * math.cos(th), d + r * math.sin(th))
        phi = math.atan2(P[1], P[0])
        if prev is not None:
            phi = prev + math.remainder(phi - prev, 2 * math.pi)
        prev = phi
        a = phi + math.pi                                  # the rod arm
        Ar = (R * math.cos(a), R * math.sin(a))
        x = Ar[0] + math.sqrt(l * l - Ar[1] ** 2)
        out.append({"theta": math.degrees(th), "P": P, "phi": math.degrees(phi), "rho": math.hypot(*P), "Ar": Ar,
                    "x": x, "rod_angle": math.degrees(math.asin(Ar[1] / l))})
    h = 2 * math.pi / n
    for i, f in enumerate(out):
        f["dx"] = (out[(i + 1) % n]["x"] - out[i - 1]["x"]) / (2 * h)
    return out


def measure(fr):
    xs = [f["x"] for f in fr]
    n = len(fr)
    i_max, i_min = xs.index(max(xs)), xs.index(min(xs))
    go = ((i_min - i_max) % n) * 360.0 / n
    return {"stroke": max(xs) - min(xs), "Q": max(go, 360 - go) / min(go, 360 - go), "i_max": i_max, "i_min": i_min,
            "to_minus": go, "rho_min": min(f["rho"] for f in fr), "rho_max": max(f["rho"] for f in fr),
            "rod_angle_max": max(abs(f["rod_angle"]) for f in fr)}


def design(S, Q, mat, F=0.0):
    g = geometry(S, Q)
    fr = frames(g)
    me = measure(fr)
    slow = "-x" if me["to_minus"] > 180 else "+x"
    checks = [{"group": "Motion", "name": "Stroke (simulation)", "util": abs(me["stroke"] - S) / (0.01 * S),
               "detail": f"{me['stroke']:.2f} mm (target {S:g})"},
              {"group": "Motion", "name": "Time ratio Q (simulation)", "util": abs(me["Q"] - Q) / 0.03,
               "detail": f"{me['Q']:.3f} (target {Q:g})"},
              {"group": "Motion", "name": "Rod angle", "util": me["rod_angle_max"] / 20.0,
               "detail": f"at most {me['rod_angle_max']:.1f}° (20° limit: side load on the ram)"}]
    info = dict(g, frames=fr, measure=me, slow_direction=slow)
    if not F:
        return dict(info, section=None), checks
    n = len(fr)
    a, b = (me["i_max"], me["i_min"]) if slow == "-x" else (me["i_min"], me["i_max"])
    work = [fr[(a + k) % n] for k in range((b - a) % n + 1)]
    T_crank = max(F * abs(f["dx"]) for f in work)
    T_lever = F * g["R"] * 1.0                                   # the rod arm at most square to the rod
    N = max(T_lever / f["rho"] for f in work)
    forces = {"F": F, "F_ab_max": max(N, F / math.cos(math.radians(me["rod_angle_max"]))), "M_crank_max": T_crank,
              "M_rocker": 0.0}
    section, s_checks = slotted._size(forces, g["r"], ("Lever", T_lever, N, g["R"] + g["r"] + g["d"], g["r"] + g["d"]), mat)
    return dict(info, section=section, T_max=T_crank, T_lever=T_lever, N_max=N), checks + list(s_checks or [])
