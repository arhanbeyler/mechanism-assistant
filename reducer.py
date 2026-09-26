"""
E1: Single-stage spur gear reducer: the first *system* (module outputs feed each other, nothing typed twice).

    motor (T1, n1) -> gear pair (gears.design) -> gear forces W = Wt / cos(phi) at mid-span of both shafts
    -> input / output shaft (shafts.design: DE-Goodman, bearings, key, deflection, critical speed incl.
       gear mass) -> housing (bearing seats side by side in two walls)

Coupling constraints are fed back until nothing changes:
    gear bore      = shaft hub diameter        (gear hub web check uses the real bore)
    gear face      >= key length               (the key must sit inside the gear)
    centre distance>= (OD1 + OD2) / 2 + WEB    (both bearings fit in the same wall)
    common span    = max(span1, span2)         (both shafts share the housing walls)
Losses are neglected (T2 = i T1): conservative for the output shaft and bearings.
Units: N, mm, N*mm, kg, rpm.
"""

import math

import gears
import shafts
import strength

WEB = 5.0            # mm, wall material between the two bearing seats
WALL_MARGIN = 8.0    # mm, wall material around the bearings
CLEAR = 5.0          # mm, gear tip to housing base
BASE_T = 6.0         # mm, base plate thickness
DENSITY = {"PLA": 1.24e-6, "PETG": 1.27e-6, "aluminum": 2.70e-6, "steel": 7.85e-6}   # kg/mm^3


def gear_mass(d, b, bore, material):
    """Solid gear (pitch-circle disc minus bore): enough for the critical-speed estimate."""
    return DENSITY[material] * math.pi / 4 * (d ** 2 - bore ** 2) * b


def housing(g, s1, s2):
    """Open-top housing: two walls (bearing seats) at x = +-span/2, base plate below the gears.
    Coordinates: gear mid-plane x = 0, input shaft on y = 0, output shaft on y = a, axes along x."""
    a = g["center_distance"]
    b1, b2 = s1["bearing"], s2["bearing"]
    span = s1["layout"]["span"]
    ra = max(g["d1"], g["d2"]) / 2 + g["module"]
    t = float(max(b1["B"], b2["B"]))
    z0 = -(ra + CLEAR) - BASE_T
    return {"x_walls": [-span / 2, span / 2], "t": t,
            "y": [min(-(b1["OD"] / 2 + WALL_MARGIN), -(g["d1"] / 2 + g["module"] + CLEAR)),   # bearings or gear tips
                  max(a + b2["OD"] / 2 + WALL_MARGIN, a + g["d2"] / 2 + g["module"] + CLEAR)],
            "z": [z0, max(b1["OD"], b2["OD"]) / 2 + WALL_MARGIN], "base_t": BASE_T,
            "seats": [(0.0, b1["OD"]), (a, b2["OD"])]}


def design(T1_Nmm, rpm1, ratio, gear_material="steel", shaft_material="1045", feature="keyseat", n=2.0):
    """Returns (design, checks) or (None, checks). Checks: {"group", "name", "util", "detail"}."""
    mat = strength.MATERIALS[gear_material]
    bores, min_face, min_center, span = (0.0, 0.0), 0.0, 0.0, None   # bores come from the shafts, not a table
    for _ in range(12):
        g, g_checks = gears.design(ratio, T1_Nmm, mat, rpm1, bores=bores, min_face=min_face, min_center=min_center)
        if g is None:
            return None, [dict(c, group="Gear") for c in g_checks]
        i, b = g["ratio"], g["face_width"]
        T2, rpm2 = T1_Nmm * i, rpm1 / i
        Wt = 2 * T1_Nmm / g["d1"]
        W = Wt / math.cos(gears.PHI)
        m1 = gear_mass(g["d1"], b, bores[0], gear_material)
        m2 = gear_mass(g["d2"], b, bores[1], gear_material)
        s1, c1 = shafts.design(None, T1_Nmm, shaft_material, feature, n, span=span, rpm=rpm1,
                               F_N=W, hub_len=b, hub_mass=m1)
        s2, c2 = shafts.design(None, T2, shaft_material, feature, n, span=span, rpm=rpm2,
                               F_N=W, hub_len=b, hub_mass=m2)
        if s1 is None or s2 is None:
            return None, ([dict(c, group="Gear") for c in g_checks] + [dict(c, group="Input shaft") for c in c1]
                          + [dict(c, group="Output shaft") for c in c2])
        new = ((s1["d"], s2["d"]),
               max(s1["key"]["length"], s2["key"]["length"]) if feature == "keyseat" else 0.0,
               (s1["bearing"]["OD"] + s2["bearing"]["OD"]) / 2 + WEB,
               max(s1["layout"]["span"], s2["layout"]["span"]))
        spans_equal = abs(s1["layout"]["span"] - s2["layout"]["span"]) < 1e-9
        if (new[0] == bores and b >= new[1] - 1e-9 and g["center_distance"] >= new[2] - 1e-9 and spans_equal):
            break
        bores, min_face, min_center, span = new
    else:
        return None, [{"group": "System", "name": "Convergence", "util": 99.0,
                       "detail": "gear / shaft / bearing constraints did not settle in 12 rounds"}]
    a = g["center_distance"]
    gap = a - (s1["bearing"]["OD"] + s2["bearing"]["OD"]) / 2
    checks = ([dict(c, group="Gear") for c in g_checks] + [dict(c, group="Input shaft") for c in c1]
              + [dict(c, group="Output shaft") for c in c2]
              + [{"group": "Housing", "name": "Material between bearing seats", "util": WEB / max(gap, 1e-9),
                  "detail": f"{gap:.1f} mm (at least {WEB:g} mm)"}])
    return {"gear": g, "shaft_in": s1, "shaft_out": s2, "housing": housing(g, s1, s2),
            "T1": T1_Nmm, "T2": T2, "rpm1": rpm1, "rpm2": rpm2, "power_W": T1_Nmm / 1000 * rpm1 * 2 * math.pi / 60,
            "Wt": Wt, "Wr": Wt * math.tan(gears.PHI), "W": W, "gear_masses": (m1, m2),
            "gear_material": mat["name"]}, checks
