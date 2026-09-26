"""
F4: slot mechanisms. A pin rides in a slot instead of a connecting rod.

Scotch yoke. Crank radius r = stroke / 2 turns about the origin; its pin runs in a slot square to the slide
direction, so the yoke moves x = r cos(theta): pure simple harmonic motion (no rod angle, a_max = r w^2
against r w^2 (1 + r/b) for a slider-crank). A slide force F (along x) is taken by the slot flank:
pin force = F, crank torque T = F r |sin(theta)|; one side bar of the yoke frame spans the slot and bends.

Crank and slotted lever (the shaper's quick return). The lever pivots at the origin; the crank turns about
O2 = (0, d) above it with r < d, its pin sliding in the lever's slot. The lever's extreme positions are where
the crank is square to the lever, cos(beta) = r / d with beta measured from the line of centres, so the
crank turns 2 beta for one stroke and 360 - 2 beta for the other:  Q = (360 - 2 beta) / (2 beta).
The lever tip (radius L) drives the ram through a pin in a vertical slot: ram x = L sin(psi),
stroke S = 2 L r / d. Given S and Q: beta = 180 / (Q + 1), r / d = cos(beta), L = S d / (2 r); d is chosen
so the pin's travel along the lever (d - r .. d + r) stays LEVER_MARGIN short of the tip.
Forces: a cutting force F on the slow stroke; lever torque F L cos(psi), pin force on the lever slot
N = F L cos(psi) / |O P|, crank torque from statics (= virtual work F |dx/dtheta|, tested).
Units: mm, N, N*mm, degrees. Standard library only.
"""

import math

import strength

N_SIM = 720
CLEAR = 0.3                  # mm, pin-to-slot clearance per side
LEVER_MARGIN = 0.15          # the pin stays this fraction of L short of the lever tip
STEEL = strength.MATERIALS["steel"]
SHAFTS = strength.PIN_DIAMETERS + [16, 20, 25, 30]     # torque-carrying crank shaft (larger than pins)


def _cross(u, v):
    return u[0] * v[1] - u[1] * v[0]


# ---------------------------------------------------------------- slotted members

def slot_bar(M, N, w, s, t, mat, span):
    """A bar of width w with a slot of width s (two side strips), thickness t: bending M about the slot's
    middle plane, pin bearing N on the slot flank, stiffness (tip deflection of a cantilever of the span)."""
    allow = mat["sy"] / mat["n"]
    I = t * (w ** 3 - s ** 3) / 12.0
    sig = M * (w / 2) / I if I > 0 else float("inf")
    br = N / (max(s - 2 * CLEAR, 1e-9) * t)
    P = M / span if span else 0.0
    delta = P * span ** 3 / (3 * mat["E"] * I) if I > 0 else float("inf")
    return [{"group": "Stress", "name": "bending (slotted section)", "util": sig / allow, "detail": f"σ = {sig:.1f} MPa"},
            {"group": "Pin", "name": "slot flank bearing", "util": br / allow, "detail": f"σ_br = {br:.1f} MPa"},
            {"group": "Stiffness", "name": "deflection", "util": delta / (span / strength.DEFLECTION_RATIO),
             "detail": f"δ = {delta:.2f} mm (limit L/{strength.DEFLECTION_RATIO})"}]


def _lightest_slot_bar(name, M, N, s, mat, span):
    best = None
    for t in strength.THICKNESSES:
        for w in range(int(s + 2 * strength.W_MIN), strength.MAX_WIDTH + 1):
            c = slot_bar(M, N, w, s, t, mat, span)
            if max(x["util"] for x in c) <= 1.0:
                if best is None or w * t < best[0] * best[1]:
                    best = (w, t, [dict(x, name=f"{name} {x['name']}") for x in c])
                break
    return best


def _size(crank_forces, crank_len, slotted, mat):
    """Pin + shaft + lightest crank section + lightest slotted bar (smallest volume).
    slotted: (name, M, N, span, length). Returns (section, checks) or (None, checks at the largest sizes)."""
    name, M, N, span, length = slotted
    best = None
    for d in strength.PIN_DIAMETERS:
        pin = strength.check_pin(crank_forces, mat, d)
        if pin["util"] > 1:
            continue
        for ds in (x for x in SHAFTS if x >= d):
            shaft = strength.check_shaft(crank_forces, mat, ds)
            if shaft["util"] > 1:
                continue
            crank = strength._lightest_link("Crank", crank_forces, crank_len, mat, d, ds)
            bar = _lightest_slot_bar(name, M, N, d + 2 * CLEAR, mat, span)
            if crank is None or bar is None:
                continue
            (wc, tc, cc), (wb, tb, cb) = crank, bar
            b0, b1 = strength.boss_diameters("Crank", wc, d, ds)
            vol = tc * (wc * crank_len + math.pi * (b0 ** 2 + b1 ** 2) / 8) + tb * wb * length
            if best is None or vol < best[2]:
                best = ({"hole": float(d), "shaft": float(ds),
                         "links": {"Crank": {"width": float(wc), "thickness": float(tc), "boss0": b0, "boss1": b1},
                                   name: {"width": float(wb), "thickness": float(tb)}}},
                        [pin, shaft] + cc + cb, vol)
            break
    if best:
        return best[0], best[1]
    d = strength.PIN_DIAMETERS[-1]
    fail = [strength.check_pin(crank_forces, mat, d), strength.check_shaft(crank_forces, mat, d)]
    fail += strength.check_link("Crank", crank_forces, crank_len, mat, strength.MAX_WIDTH, strength.THICKNESSES[-1], d, d)
    fail += [dict(x, name=f"{name} {x['name']}") for x in
             slot_bar(M, N, strength.MAX_WIDTH, d + 2 * CLEAR, strength.THICKNESSES[-1], mat, span)]
    return None, fail


# ---------------------------------------------------------------- Scotch yoke

def yoke_frames(r, n=N_SIM):
    """x (yoke position), pin height in the slot and dx/dtheta over one crank turn."""
    out = []
    for i in range(n):
        th = 2 * math.pi * i / n
        out.append({"theta": math.degrees(th), "x": r * math.cos(th), "y": r * math.sin(th), "dx": -r * math.sin(th)})
    return out


def yoke_design(S, mat, F=0.0):
    """Scotch yoke for stroke S (mm) and slide force F (N). Returns (design, checks)."""
    r = S / 2
    frames = yoke_frames(r)
    slot_len = 2 * r                                  # pin centre travel along the slot
    design = {"r": r, "stroke": S, "frames": frames, "slot_len": slot_len}
    if not F:
        return dict(design, section=None), []
    forces = {"F": F, "F_ab_max": F, "M_crank_max": F * r, "M_rocker": 0.0}
    # one side bar of the frame, span = slot length + pin, load F in its middle: M = F L / 4
    span = slot_len + 10.0
    section, checks = _size(forces, r, ("Yoke", F * span / 4, F, span, span), mat)
    return dict(design, section=section, T_max=F * r), checks


# ---------------------------------------------------------------- crank and slotted lever (quick return)

def lever_geometry(S, Q):
    """r, d, L for stroke S and time ratio Q (> 1)."""
    beta = math.pi / (Q + 1)
    c = math.cos(beta)                                # r / d
    L = S / (2 * c)
    d = (1 - LEVER_MARGIN) * L / (1 + c)
    return {"r": c * d, "d": d, "L": L, "beta": beta, "Q": Q, "stroke": S}


def lever_frames(g, n=N_SIM):
    """Crank angle theta from +x; pin P, lever angle psi (from +y, positive towards +x), ram x, dx/dtheta."""
    r, d, L = g["r"], g["d"], g["L"]
    out = []
    for i in range(n):
        th = 2 * math.pi * i / n
        P = (r * math.cos(th), d + r * math.sin(th))
        psi = math.atan2(P[0], P[1])
        rho = math.hypot(*P)
        # d(psi)/d(theta) = (P x dP/dtheta) / rho^2 (sign: psi grows clockwise from +y)
        dP = (-r * math.sin(th), r * math.cos(th))
        dpsi = -_cross(P, dP) / rho ** 2
        out.append({"theta": math.degrees(th), "P": P, "psi": math.degrees(psi), "rho": rho,
                    "x": L * math.sin(psi), "dx": L * math.cos(psi) * dpsi})
    return out


def lever_measure(frames):
    """Stroke and the crank angles of the two strokes (from the ram's extreme positions)."""
    xs = [f["x"] for f in frames]
    n = len(frames)
    i_max, i_min = xs.index(max(xs)), xs.index(min(xs))
    go = ((i_min - i_max) % n) * 360.0 / n                  # crank turn from +x extreme to -x extreme
    return {"stroke": max(xs) - min(xs), "to_minus": go, "to_plus": 360 - go,
            "Q": max(go, 360 - go) / min(go, 360 - go), "i_max": i_max, "i_min": i_min}


def lever_statics(g, fr, F):
    """Ram force F (along x, resisting) -> lever torque, pin force on the lever slot, crank torque."""
    psi = math.radians(fr["psi"])
    T_lever = F * g["L"] * math.cos(psi)
    N = T_lever / fr["rho"]
    n_hat = (math.cos(psi), -math.sin(psi))                 # lever normal (psi grows towards +x)
    P = fr["P"]
    arm = (P[0], P[1] - g["d"])                             # O2 -> P
    T_crank = abs(_cross(arm, (N * n_hat[0], N * n_hat[1])))
    return T_lever, N, T_crank


def lever_design(S, Q, mat, F=0.0):
    """Quick-return shaper linkage for stroke S (mm), time ratio Q and cutting force F (N) on the slow
    stroke. Returns (design, checks)."""
    g = lever_geometry(S, Q)
    frames = lever_frames(g)
    meas = lever_measure(frames)
    slow = "-x" if meas["to_minus"] > meas["to_plus"] else "+x"
    design = dict(g, frames=frames, measure=meas, slow_direction=slow)
    if not F:
        return dict(design, section=None), []
    n = len(frames)
    a, b = (meas["i_max"], meas["i_min"]) if slow == "-x" else (meas["i_min"], meas["i_max"])
    work = [frames[(a + k) % n] for k in range(((b - a) % n) + 1)]        # the slow (cutting) stroke
    st = [lever_statics(g, f, F) for f in work]
    T_lever = max(s[0] for s in st)
    N = max(s[1] for s in st)
    T_crank = max(s[2] for s in st)
    forces = {"F": F, "F_ab_max": max(N, F), "M_crank_max": T_crank, "M_rocker": 0.0}
    section, checks = _size(forces, g["r"], ("Lever", T_lever, N, g["L"], g["L"]), mat)
    return dict(design, section=section, T_lever=T_lever, N_max=N, T_max=T_crank), checks
