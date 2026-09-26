"""
E2: Two-stage spur gear reducer (three shafts). Built on the same coupling idea as the single stage (reducer.py),
with the multi-load shaft of shaft_multi.py for all three shafts.

    motor -> stage 1 (pinion 1 on the input shaft, gear 2 on the intermediate shaft)
          -> stage 2 (pinion 3 on the intermediate shaft, gear 4 on the output shaft)

Layout: shaft axes along x in one plane: input y = 0, intermediate y = a1, output y = a1 + a2. Stage 1 gears at
x1 = -D/2, stage 2 at x2 = +D/2 (D = axial pitch of the stages); housing walls at x = -+L/2.
Mesh forces (input turning +x, radial toward each gear's own centre, tangential against the driver's rotation):
    pinion 1: (-Wr1, -Wt1)   gear 2: (+Wr1, +Wt1)   pinion 3: (-Wr2, +Wt2)   gear 4: (+Wr2, -Wt2)   as (Fy, Fz)
so on the intermediate shaft the tangential forces add and the radial ones partly cancel.
Ratio split: the i1 that makes the lightest pair of gear sets (scan around sqrt(i)).
Coupling constraints, iterated until nothing changes: gear bores = shaft diameters, gear faces >= keys, bearings of
neighbouring shafts side by side in the walls, gear 2 must clear the output shaft and gear 4 the input shaft.
Losses neglected. Units: N, mm, N*mm, kg, rpm.
"""

import math

import gears
import reducer
import shaft_multi
import strength

SINGLE_MAX = 6.0     # above this ratio the reducer module uses two stages (one spur stage stays <= ~6)
MAX_RATIO = 36.0     # two stages of at most 6 each
GAP_AX = 6.0         # mm, axial gap between the two stages
CLEAR = 3.0          # mm, gear tip to a neighbouring shaft
MARGIN = 6.0         # mm, gear face to the bearing / wall


def _gear_mass(d, b, bore, material):
    return reducer.gear_mass(d, b, bore, material)


def split(ratio, T1, mat, rpm):
    """Stage ratios from the lightest gear sets (volume ~ b (d1^2 + d2^2) per stage)."""
    best = None
    root = math.sqrt(ratio)
    for k in range(0, 60):
        i1 = root * (0.85 + 0.0125 * k)
        if not 1.0 < i1 < ratio:
            continue
        g1, _ = gears.design(i1, T1, mat, rpm, bores=(0.0, 0.0))
        if g1 is None:
            continue
        g2, _ = gears.design(ratio / g1["ratio"], T1 * g1["ratio"], mat, rpm / g1["ratio"], bores=(0.0, 0.0))
        if g2 is None:
            continue
        vol = g1["volume"] + g2["volume"]
        if best is None or vol < best[0]:
            best = (vol, g1["ratio"])
    return best[1] if best else None


def design(T1_Nmm, rpm1, ratio, gear_material="steel", shaft_material="1045", n=2.0):
    """Returns (design, checks) or (None, checks)."""
    mat = strength.MATERIALS[gear_material]
    i1 = split(ratio, T1_Nmm, mat, rpm1)
    if i1 is None:
        return None, [{"group": "Gears", "name": "Ratio split", "util": 99.0,
                       "detail": f"no two-stage gear sets carry {T1_Nmm / 1000:g} N·m at i = {ratio:g}"}]
    bores = [0.0, 0.0, 0.0]                                   # input, intermediate, output shaft
    min_face = [0.0, 0.0]
    min_center = [0.0, 0.0]
    span = None
    for _ in range(15):
        g1, c1 = gears.design(i1, T1_Nmm, mat, rpm1, bores=(bores[0], bores[1]), min_face=min_face[0],
                              min_center=min_center[0])
        if g1 is None:
            return None, [dict(c, group="Stage 1") for c in c1]
        T2, rpm2 = T1_Nmm * g1["ratio"], rpm1 / g1["ratio"]
        g2, c2 = gears.design(ratio / g1["ratio"], T2, mat, rpm2, bores=(bores[1], bores[2]), min_face=min_face[1],
                              min_center=min_center[1])
        if g2 is None:
            return None, [dict(c, group="Stage 2") for c in c2]
        T3, rpm3 = T2 * g2["ratio"], rpm2 / g2["ratio"]
        b1, b2 = g1["face_width"], g2["face_width"]
        D = (b1 + b2) / 2 + GAP_AX
        x1, x2 = -D / 2, D / 2
        L = span or (D + max(b1, b2) + 2 * MARGIN + 20.0)     # first guess; the bearings widen it below
        xb1, xb2 = -L / 2, L / 2
        Wt1, Wt2 = 2 * T1_Nmm / g1["d1"], 2 * T2 / g2["d1"]
        Wr1, Wr2 = Wt1 * math.tan(gears.PHI), Wt2 * math.tan(gears.PHI)
        m = [_gear_mass(g1["d1"], b1, bores[0], gear_material), _gear_mass(g1["d2"], b1, bores[1], gear_material),
             _gear_mass(g2["d1"], b2, bores[1], gear_material), _gear_mass(g2["d2"], b2, bores[2], gear_material)]
        shaft_in = [{"x": x1, "Fy": -Wr1, "Fz": -Wt1, "width": b1, "mass": m[0], "T": T1_Nmm}]
        shaft_mid = [{"x": x1, "Fy": Wr1, "Fz": Wt1, "width": b1, "mass": m[1], "T": T2},
                     {"x": x2, "Fy": -Wr2, "Fz": Wt2, "width": b2, "mass": m[2], "T": T2}]
        shaft_out = [{"x": x2, "Fy": Wr2, "Fz": -Wt2, "width": b2, "mass": m[3], "T": T3}]
        s = []
        for seats, torques, rpm, stubs in ((shaft_in, (T1_Nmm, 0.0), rpm1, (1, 0)),
                                           (shaft_mid, (0.0, 0.0), rpm2, (0, 0)),
                                           (shaft_out, (0.0, T3), rpm3, (0, 1))):
            d_guess = max(bores[len(s)], 12.0)
            r, ch = shaft_multi.design(seats, torques, xb1, xb2, shaft_material, n, rpm=rpm,
                                       stub_left=1.5 * d_guess * stubs[0], stub_right=1.5 * d_guess * stubs[1])
            if r is None:
                name = ("Input", "Intermediate", "Output")[len(s)]
                return None, ([dict(c, group="Stage 1") for c in c1] + [dict(c, group="Stage 2") for c in c2]
                              + [dict(c, group=f"{name} shaft · {c['group']}") for c in ch])
            s.append((r, ch))
        d = [r["d"] for r, _ in s]
        OD = [r["bearing"]["OD"] for r, _ in s]
        B = max(r["bearing"]["B"] for r, _ in s)
        keys = [[k for k in r["keys"] if k] for r, _ in s]
        new_face = [max([k["length"] for k in keys[0] + keys[1][:1]] or [0.0]),
                    max([k["length"] for k in keys[1][1:] + keys[2]] or [0.0])]
        ra2 = g1["d2"] / 2 + g1["module"]
        ra4 = g2["d2"] / 2 + g2["module"]
        new_center = [(OD[0] + OD[1]) / 2 + reducer.WEB,
                      max((OD[1] + OD[2]) / 2 + reducer.WEB, ra2 + d[2] / 2 + CLEAR)]
        new_span = D + max(b1, b2) + 2 * MARGIN + B
        # gear 4 must also clear the input shaft: a1 + a2 >= ra4 + d_in / 2 + CLEAR (grow stage 1 if needed)
        if g1["center_distance"] + g2["center_distance"] < ra4 + d[0] / 2 + CLEAR:
            new_center[0] = max(new_center[0], ra4 + d[0] / 2 + CLEAR - g2["center_distance"])
        stable = (d == bores and b1 >= new_face[0] - 1e-9 and b2 >= new_face[1] - 1e-9
                  and g1["center_distance"] >= new_center[0] - 1e-9 and g2["center_distance"] >= new_center[1] - 1e-9
                  and span is not None and abs(span - max(new_span, span)) < 1e-9)
        if stable:
            break
        bores, min_face = d, [max(min_face[0], new_face[0]), max(min_face[1], new_face[1])]
        min_center = [max(min_center[0], new_center[0]), max(min_center[1], new_center[1])]
        span = max(new_span, span or 0.0)
    else:
        return None, [{"group": "System", "name": "Convergence", "util": 99.0,
                       "detail": "gear / shaft / bearing constraints did not settle in 15 rounds"}]
    a1, a2 = g1["center_distance"], g2["center_distance"]
    checks = ([dict(c, group="Stage 1") for c in c1] + [dict(c, group="Stage 2") for c in c2]
              + [dict(c, group=f"{name} shaft · {c['group']}") for name, (_, ch) in zip(("Input", "Intermediate", "Output"), s)
                 for c in ch]
              + [{"group": "Housing", "name": "Gear 2 clears the output shaft", "util": (ra2 + d[2] / 2 + CLEAR) / a2,
                  "detail": f"a2 = {a2:g} mm ≥ r_a2 + r_shaft + {CLEAR:g} = {ra2 + d[2] / 2 + CLEAR:.1f} mm"},
                 {"group": "Housing", "name": "Gear 4 clears the input shaft", "util": (ra4 + d[0] / 2 + CLEAR) / (a1 + a2),
                  "detail": f"a1 + a2 = {a1 + a2:g} mm ≥ {ra4 + d[0] / 2 + CLEAR:.1f} mm"},
                 {"group": "Housing", "name": "Keys fit their gears",
                  "util": max(new_face[0] / b1, new_face[1] / b2),
                  "detail": f"longest keys {new_face[0]:g} / {new_face[1]:g} mm, gear faces {b1:g} / {b2:g} mm"},
                 {"group": "Housing", "name": "Material between bearing seats",
                  "util": reducer.WEB / max(1e-9, min(a1 - (OD[0] + OD[1]) / 2, a2 - (OD[1] + OD[2]) / 2)),
                  "detail": f"{min(a1 - (OD[0] + OD[1]) / 2, a2 - (OD[1] + OD[2]) / 2):.1f} mm (at least {reducer.WEB:g})"}])
    shafts_out = [r for r, _ in s]
    bmax = max(shafts_out[i]["bearing"]["OD"] for i in range(3))
    ra_max = max(ra2, ra4, g1["d1"] / 2 + g1["module"], g2["d1"] / 2 + g2["module"])
    housing = {"x_walls": [-span / 2, span / 2], "t": float(max(r["bearing"]["B"] for r in shafts_out)),
               "y": [min(-(shafts_out[0]["bearing"]["OD"] / 2 + reducer.WALL_MARGIN),     # bearings or gear tips
                         -(g1["d1"] / 2 + g1["module"] + reducer.CLEAR)),
                     max(a1 + a2 + shafts_out[2]["bearing"]["OD"] / 2 + reducer.WALL_MARGIN,
                         a1 + a2 + ra4 + reducer.CLEAR, a1 + ra2 + reducer.CLEAR)],
               "z": [-(ra_max + reducer.CLEAR) - reducer.BASE_T, bmax / 2 + reducer.WALL_MARGIN],
               "base_t": reducer.BASE_T,
               "seats": [(0.0, shafts_out[0]["bearing"]["OD"]), (a1, shafts_out[1]["bearing"]["OD"]),
                         (a1 + a2, shafts_out[2]["bearing"]["OD"])]}
    return {"stages": [g1, g2], "shafts": shafts_out, "housing": housing, "x_stages": [x1, x2], "span": span,
            "T": [T1_Nmm, T2, T3], "rpm": [rpm1, rpm2, rpm3], "ratio": g1["ratio"] * g2["ratio"],
            "power_W": T1_Nmm / 1000 * rpm1 * 2 * math.pi / 60, "gear_material": mat["name"],
            "forces": {"Wt1": Wt1, "Wr1": Wr1, "Wt2": Wt2, "Wr2": Wr2}}, checks
