"""
E2: Shaft with several gears (Shigley ch. 7, the classic intermediate-shaft problem).

Same checks as shafts.py (single load), generalised:
  * loads: gear mesh forces (Fy, Fz) at axial positions -> bending in two planes, M(x) = sqrt(My^2 + Mz^2)
  * torque only where it is transmitted (between the gears / to the shaft end that carries it)
  * DE-Goodman fatigue + von Mises yield at every gear seat (keyseat, K_t 2.14 / 3.0) with the local M and T
  * DIN 6885 key per torque-carrying seat
  * bearings from the catalogue at both ends; each seat checked (sharp shoulder, M = R B / 2, its own T); L10
  * deflection and slope by numerical integration in both planes (shafts.beam's method):
    slope at the bearings <= 0.001 rad, deflection at each gear <= 0.13 mm, slope at each gear <= 0.0005 rad
    (Shigley Table 7-2, spur gears)
  * critical speed: Rayleigh with the static deflection under the weights (shaft + gears), >= 2x running speed
One body diameter d for all gear seats, stepped down to the bearing seats; raised until every check passes.
Frame: x along the shaft, bearings at xb1 < xb2; y, z transverse. Units: N, mm, N*mm, kg, rpm.
"""

import math

import shafts

G = 9.81


def _set_radius(segs, x0, x1, r):
    return shafts._set_radius(segs, x0, x1, r)


def layout(d, seats, xb1, xb2, bearing, stub_left=0.0, stub_right=0.0):
    """seats: [{"x", "width", "key"}]; bearing: BEARINGS row. Returns a layout compatible with
    fusion_lib.shaft (segments, bearings, journal, feature + keys)."""
    name, bore, OD, B = bearing[0], float(bearing[1]), float(bearing[2]), float(bearing[3])
    x0 = xb1 - B / 2 - stub_left
    total = (xb2 + B / 2 + stub_right) - x0
    shift = -x0                                     # layout x starts at 0 like shafts.layout
    segs = [(0.0, total, d / 2)]
    if bore < d:
        segs = _set_radius(segs, 0.0, xb1 + B / 2 + shift, bore / 2)
        segs = _set_radius(segs, xb2 - B / 2 + shift, total, bore / 2)
    keys = []
    for s in seats:
        k = s.get("key")
        if k:
            keys.append({"kind": "keyseat", "x": s["x"] + shift, "length": k["length"], "b": k["b"], "h": k["h"],
                         "t1": k["t1"]})
    hubs = [(s["x"] + shift - s["width"] / 2, s["x"] + shift + s["width"] / 2) for s in seats]
    return {"length": total, "segments": segs, "d": d, "span": xb2 - xb1, "journal": bore, "bearing_name": name,
            "bearings": [(xb1 + shift, B, OD), (xb2 + shift, B, OD)], "hub": hubs[0], "hubs": hubs,
            "feature": keys[0] if keys else {"kind": "plain"}, "keys": keys, "shift": shift}


def reactions(loads, xb1, xb2):
    """Bearing reactions of a simply supported shaft, per plane: R1 = -sum F (xb2 - x) / L, R2 = -sum F (x - xb1) / L
    for applied point loads [(x, Fy, Fz)]. Returns ((R1y, R1z), (R2y, R2z))."""
    L = xb2 - xb1
    R1 = tuple(-sum(ld[1 + c] * (xb2 - ld[0]) for ld in loads) / L for c in (0, 1))
    R2 = tuple(-sum(ld[1 + c] * (ld[0] - xb1) for ld in loads) / L for c in (0, 1))
    return R1, R2


def moment(loads, xb1, xb2, x):
    """Bending moment (My, Mz) at x: M = R1 (x - xb1) + sum over loads left of x of F (x - xi)."""
    R1, _ = reactions(loads, xb1, xb2)
    return tuple(R1[c] * (x - xb1) + sum(ld[1 + c] * (x - ld[0]) for ld in loads if ld[0] < x) for c in (0, 1))


def _deflect(radius_at, loads, xb1, xb2, N=1200):
    """Deflection (y, z) and slope along the span by double integration of M / (E I), zero at the bearings."""
    L = xb2 - xb1
    h = L / N
    xs = [xb1 + i * h for i in range(N + 1)]
    res = []
    for c in (0, 1):
        kap = []
        for x in xs:
            M = moment(loads, xb1, xb2, x)[c]
            I = math.pi * (2 * radius_at(x)) ** 4 / 64
            kap.append(M / (shafts.E_STEEL * I))
        th, y = [0.0], [0.0]
        for i in range(N):
            th.append(th[-1] + (kap[i] + kap[i + 1]) / 2 * h)
            y.append(y[-1] + (th[i] + th[i + 1]) / 2 * h)
        c1 = -y[-1] / L
        res.append(([t + c1 for t in th], [yy + c1 * (x - xb1) for yy, x in zip(y, xs)]))
    return xs, res


def check_shaft(d, seats, torques, xb1, xb2, mat, n, ny, rpm, stub_left=0.0, stub_right=0.0):
    """seats: [{"x", "Fy", "Fz", "width", "mass", "T"}] (T: torque carried through that seat).
    torques: (T at the left bearing seat, T at the right bearing seat). Returns (result, checks) or (None, checks)."""
    checks = []
    Kf, Kfs = shafts.FEATURES["keyseat"][1:]
    Se = shafts.endurance(mat["Sut"], d)
    loads = [(s["x"], s["Fy"], s["Fz"]) for s in seats]
    for i, s in enumerate(seats, 1):
        My, Mz = moment(loads, xb1, xb2, s["x"])
        M = math.hypot(My, Mz)
        nf = shafts.de_goodman_n(d, M, s["T"], mat["Sut"], Se, Kf, Kfs)
        sv = shafts.von_mises_peak(d, M, s["T"], Kf, Kfs)
        checks += [{"group": f"Gear seat {i}", "name": "DE-Goodman fatigue safety", "util": n / nf,
                    "detail": f"M = {M / 1000:.1f} N·m, T = {s['T'] / 1000:.1f} N·m, n_f = {nf:.2f} (target {n:g})"},
                   {"group": f"Gear seat {i}", "name": "First-cycle yield (von Mises)", "util": ny * sv / mat["Sy"],
                    "detail": f"σ' = {sv:.0f} MPa, n_y = {mat['Sy'] / sv:.2f}"}]
        if s["T"] > 0:
            key, kc = shafts.key_design(s["T"], d, ny)
            checks += [dict(c, group=f"Gear seat {i}") for c in kc]
            if key is None:
                return None, checks
            s["key"] = key                     # key <= gear face is a system constraint (the gear gets wider)
    R1, R2 = reactions(loads, xb1, xb2)
    R = (math.hypot(*R1), math.hypot(*R2))
    brg, bc = None, []
    for cand in shafts.bearing_candidates(d):
        bc = []
        for side, (Rs, Ts) in enumerate(zip(R, torques), 1):
            bc += [dict(c, group=f"Bearing {side}") for c in shafts.journal_checks(cand, Rs, Ts, mat, n, ny, rpm)]
        if all(c["util"] <= 1.0 for c in bc):
            brg = cand
            break
    if brg is None:
        checks += bc or [{"group": "Bearing", "name": "Bearing selection", "util": 99.0,
                          "detail": f"no bearing fits a Ø{d:g} shaft"}]
        return None, checks
    checks += bc
    lay = layout(d, seats, xb1, xb2, brg, stub_left, stub_right)
    shift = lay["shift"]
    radius_at = lambda x: shafts.radius_at(lay, x + shift)
    xs, ((thy, yy), (thz, yz)) = _deflect(radius_at, loads, xb1, xb2)
    N = len(xs) - 1
    idx = lambda x: min(N, max(0, round((x - xb1) / (xs[1] - xs[0]))))
    slope_b = max(math.hypot(thy[0], thz[0]), math.hypot(thy[-1], thz[-1]))
    checks.append({"group": "Deflection", "name": "Slope at the bearings", "util": slope_b / shafts.SLOPE_MAX,
                   "detail": f"{slope_b * 1000:.2f} mrad (limit {shafts.SLOPE_MAX * 1000:g})"})
    for i, s in enumerate(seats, 1):
        j = idx(s["x"])
        y = math.hypot(yy[j], yz[j])
        sl = math.hypot(thy[j], thz[j])
        checks += [{"group": "Deflection", "name": f"Deflection at gear {i}", "util": y / shafts.DEFL_MAX,
                    "detail": f"{y * 1000:.1f} µm (limit {shafts.DEFL_MAX * 1000:.0f} µm)"},
                   {"group": "Deflection", "name": f"Slope at gear {i}", "util": sl / GEAR_SLOPE_MAX,
                    "detail": f"{sl * 1000:.2f} mrad (spur gear limit {GEAR_SLOPE_MAX * 1000:g})"}]
    n_crit = critical_speed(lay, seats, xb1, xb2, radius_at)
    if rpm:
        checks.append({"group": "Vibration", "name": "Critical speed", "util": shafts.CRIT_RATIO * rpm / n_crit,
                       "detail": f"{n_crit:,.0f} rpm (at least {shafts.CRIT_RATIO:g}× the running {rpm:.0f} rpm, gears included)"})
    life = [shafts.l10_hours(brg[4], Rs, rpm) if rpm else None for Rs in R]
    return {"d": d, "bearing": {"name": brg[0], "bore": brg[1], "OD": brg[2], "B": brg[3], "C": brg[4], "life_h": life},
            "reactions": R, "layout": lay, "seats": seats, "n_crit": n_crit,
            "keys": [s.get("key") for s in seats]}, checks


GEAR_SLOPE_MAX = 0.0005      # rad, uncrowned spur gears (Shigley Table 7-2)


def critical_speed(lay, seats, xb1, xb2, radius_at, N=80):
    """Rayleigh: w^2 = g sum(W y) / sum(W y^2), y = static deflection under the weights (shaft + gears)."""
    L = xb2 - xb1
    h = L / N
    loads = [(xb1 + (i + 0.5) * h, shafts.RHO_STEEL * math.pi * radius_at(xb1 + (i + 0.5) * h) ** 2 * h * G, 0.0)
             for i in range(N)]
    loads += [(s["x"], s["mass"] * G, 0.0) for s in seats]
    xs, ((_, y), _) = _deflect(radius_at, loads, xb1, xb2, N=400)
    step = xs[1] - xs[0]
    yat = lambda x: y[min(len(y) - 1, max(0, round((x - xb1) / step)))]
    num = sum(W * yat(x) for x, W, _ in loads)
    den = sum(W * yat(x) ** 2 for x, W, _ in loads)
    w = math.sqrt(G * 1000 * num / den) if den > 0 else float("inf")   # y in mm -> g in mm/s^2
    return w * 60 / (2 * math.pi)


def design(seats, torques, xb1, xb2, material="1045", n=2.0, ny=1.5, rpm=None, stub_left=0.0, stub_right=0.0):
    """Smallest standard body diameter passing every check. Returns (result, checks) or (None, checks)."""
    mat = shafts.SHAFT_MATERIALS[material]
    last = []
    for d in shafts.STANDARD_D:
        if d < 10:
            continue
        r, checks = check_shaft(float(d), [dict(s) for s in seats], torques, xb1, xb2, mat, n, ny, rpm,
                                stub_left, stub_right)
        last = checks
        if r is not None and all(c["util"] <= 1.0 for c in checks):
            r["material"] = mat["name"]
            return r, checks
    return None, last
