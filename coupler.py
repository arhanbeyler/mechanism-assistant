"""
F2: coupler-point curves of a four-bar, and the Hoeckens straight-line linkage.

Frame: O2 (crank pivot) at the origin, O4 (rocker pivot) at (d, 0). The coupler point P is fixed to the
coupler AB: at distance p from A, at angle gamma from the line A -> B.

Straight-line linkage (Hoeckens, a cognate of Chebyshev's): a : b : c : d = 1 : 2.5 : 2.5 : 2 and P on AB
extended with AP = 2 AB. Its coupler point runs on an almost straight line for about half a turn of the
crank, at almost constant speed, then lifts off and returns fast (walking machines, film advance, conveyor
pushers). The straight part is found on the simulated curve (not taken from a table): the longest run of
samples whose deviation from their best-fit line stays within STRAIGHT_TOL of the run's length.

Loads: a force F at P that resists the motion along the working (straight) stroke; nothing on the return.
Crank torque by virtual work T = F |dP/dtheta|, checked against the statics of the coupler (the rocker is a
two-force member). The links are sized with strength.size_links; the coupler also carries the bending
moment of F about B (P overhangs B).
"""

import math

import strength
from kinematics import MU_MAX_DEG, MU_MIN_DEG, _circle_intersections, grashof

HOECKENS = {"a": 1.0, "b": 2.5, "c": 2.5, "d": 2.0, "p": 5.0, "gamma": 0.0}
BRANCH = 1               # B on the left of A -> O4: the branch whose coupler point has the straight part
STRAIGHT_TOL = 0.002     # deviation from the best-fit line <= 0.2 % of the straight length
N = 720                  # samples per crank turn (0.5 deg)


def _cross(u, v):
    return u[0] * v[1] - u[1] * v[0]


def curve(m, n=N, branch=BRANCH):
    """Coupler-point path over one crank turn. m: {a, b, c, d, p, gamma(deg)}.
    Returns a list of frames {theta (deg), A, B, P, mu (deg)}; None if the crank cannot turn fully."""
    a, b, c, d = m["a"], m["b"], m["c"], m["d"]
    O4 = (d, 0.0)
    g = math.radians(m.get("gamma", 0.0))
    frames, prev = [], None
    for i in range(n):
        th = 2 * math.pi * i / n
        A = (a * math.cos(th), a * math.sin(th))
        sols = _circle_intersections(A, b, O4, c)
        if not sols:
            return None
        if prev is None:
            B = max(sols, key=lambda s: branch * _cross((O4[0] - A[0], O4[1] - A[1]), (s[0] - A[0], s[1] - A[1])))
        else:
            B = min(sols, key=lambda s: math.dist(s, prev))
        prev = B
        phi = math.atan2(B[1] - A[1], B[0] - A[0]) + g
        P = (A[0] + m["p"] * math.cos(phi), A[1] + m["p"] * math.sin(phi))
        ab, ob = (B[0] - A[0], B[1] - A[1]), (B[0] - O4[0], B[1] - O4[1])
        cos_mu = (ab[0] * ob[0] + ab[1] * ob[1]) / (b * c)
        frames.append({"theta": math.degrees(th), "A": A, "B": B, "P": P,
                       "mu": math.degrees(math.acos(max(-1.0, min(1.0, cos_mu))))})
    return frames


def line_fit(points):
    """Least-squares line (total, not vertical-only): (centroid, unit direction, largest deviation)."""
    n = len(points)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    u = (math.cos(ang), math.sin(ang))
    dev = max(abs(_cross(u, (p[0] - cx, p[1] - cy))) for p in points)
    return (cx, cy), u, dev


def _extent(points, c, u):
    s = [(p[0] - c[0]) * u[0] + (p[1] - c[1]) * u[1] for p in points]
    return max(s) - min(s)


def _longest_run(pts, dev_max):
    """Longest cyclic run of consecutive points whose best-fit line deviation is <= dev_max (two pointers:
    a shorter run inside a straight run is straight too). Returns (i0, count) or None."""
    n = len(pts)
    best, j = None, 2
    for i in range(n):
        j = max(j, i + 2)
        while j - i < n:                                   # grow the run [i, j] while it stays straight
            if line_fit([pts[k % n] for k in range(i, j + 1)])[2] > dev_max:
                break
            j += 1
        if j - i >= 3 and (best is None or j - i > best[1]):
            best = (i, j - i)                              # samples i .. j-1 passed
    return best


def straight_segment(frames, tol=STRAIGHT_TOL):
    """Longest run of consecutive samples (cyclic) that is straight within tol x its own length.
    The relative limit is not monotone (a short piece of a gentle curve looks worse than the whole), so
    the absolute limit tol x L is used and L iterated to a fixed point.
    Returns {i0, count, length, deviation, direction, centroid, theta0, theta1, fraction}."""
    pts = [f["P"] for f in frames]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    L = math.hypot(max(xs) - min(xs), max(ys) - min(ys))     # start from the curve's size (too long)
    seg = None
    for _ in range(20):
        run = _longest_run(pts, tol * L)
        if run is None:
            return None
        seg = segment_at(frames, *run)
        if abs(seg["length"] - L) < 1e-6 * L:
            break
        L = seg["length"]
    return seg


def segment_at(frames, i0, count):
    """The run of samples i0 .. i0+count-1 as a segment (used again on a scaled copy of the linkage)."""
    n = len(frames)
    run = [frames[k % n]["P"] for k in range(i0, i0 + count)]
    c, u, dev = line_fit(run)
    return {"i0": i0, "count": count, "length": _extent(run, c, u), "deviation": dev, "direction": u,
            "centroid": c, "theta0": frames[i0]["theta"], "theta1": frames[(i0 + count - 1) % n]["theta"],
            "fraction": count / n}


def point_speed(frames):
    """|dP/dtheta| (mm per rad) at every sample, central differences."""
    n = len(frames)
    h = 2 * (2 * math.pi / n)
    out = []
    for i in range(n):
        p0, p1 = frames[i - 1]["P"], frames[(i + 1) % n]["P"]
        out.append((p1[0] - p0[0], p1[1] - p0[1]))
    return [(v[0] / h, v[1] / h) for v in out]


def statics(fr, m, F_ext):
    """Joint forces with a force F_ext (vector) at P. The rocker is a two-force member, so the force at B
    acts along O4-B; moments about A give its size. Returns (crank torque, |F_A|, |F_B|, coupler moment at B)."""
    A, B, P = fr["A"], fr["B"], fr["P"]
    O4 = (m["d"], 0.0)
    L = math.dist(B, O4)
    uc = ((B[0] - O4[0]) / L, (B[1] - O4[1]) / L)
    lam = -_cross((P[0] - A[0], P[1] - A[1]), F_ext) / _cross((B[0] - A[0], B[1] - A[1]), uc)
    FA = (-(lam * uc[0] + F_ext[0]), -(lam * uc[1] + F_ext[1]))       # on the coupler, from the crank
    T = -_cross(A, FA)                                                # motor torque that holds the crank
    M_B = abs(_cross((P[0] - B[0], P[1] - B[1]), F_ext))
    return T, math.hypot(*FA), abs(lam), M_B


def scaled(k, base=HOECKENS):
    return {key: (v * k if key != "gamma" else v) for key, v in base.items()}


def design_straight(stroke_mm, mat, load_N=0.0, rpm=None, tol=STRAIGHT_TOL):
    """Hoeckens straight-line linkage whose straight stroke is at least stroke_mm.
    Returns (design, checks); design["section"] is None if no section carries the load.
    Checks: {group, name, util, detail}."""
    unit = straight_segment(curve(HOECKENS), tol)                      # the shape scales: search it once
    a = math.ceil(stroke_mm / unit["length"] * 10) / 10                # crank to 0.1 mm, stroke >= target
    m = scaled(a)
    frames = curve(m)
    seg = segment_at(frames, unit["i0"], unit["count"])
    n = len(frames)
    idx = [(seg["i0"] + k) % n for k in range(seg["count"])]
    speed = point_speed(frames)
    along = [abs(speed[i][0] * seg["direction"][0] + speed[i][1] * seg["direction"][1]) for i in idx]
    v_mean = sum(along) / len(along)
    v_var = (max(along) - min(along)) / v_mean

    # Working stroke loaded: F against the motion, along the path's velocity
    T_max = F_A = F_B = M_B = 0.0
    T_vw = 0.0
    if load_N > 0:
        for i in idx:
            vx, vy = speed[i]
            vn = math.hypot(vx, vy)
            F_ext = (-load_N * vx / vn, -load_N * vy / vn)
            T, fa, fb, mb = statics(frames[i], m, F_ext)
            T_max, F_A, F_B, M_B = max(T_max, abs(T)), max(F_A, fa), max(F_B, fb), max(M_B, mb)
            T_vw = max(T_vw, load_N * vn)
    mu_work = [frames[i]["mu"] for i in idx]
    mu_all = [f["mu"] for f in frames]

    forces = {"F": load_N, "F_ab_max": max(F_A, F_B), "M_crank_max": T_max, "M_rocker": 0.0,
              "M_coupler": M_B, "coupler_span": m["b"], "coupler_overhang": m["p"] - m["b"]}
    lengths = {"a": m["a"], "b": m["b"], "c": m["c"], "d": m["d"]}
    section, s_checks = strength.size_links(forces, lengths, mat)

    ok, kind = grashof(m["a"], m["b"], m["c"], m["d"])
    checks = [
        {"group": "Motion", "name": "Crank turns fully (Grashof)", "util": 0.0 if ok else 2.0, "detail": kind},
        {"group": "Motion", "name": "Straight stroke", "util": stroke_mm / seg["length"],
         "detail": f"{seg['length']:.1f} mm straight (asked {stroke_mm:g} mm)"},
        {"group": "Motion", "name": "Straightness", "util": seg["deviation"] / (tol * seg["length"]),
         "detail": f"largest deviation {seg['deviation']:.3f} mm ({100 * seg['deviation'] / seg['length']:.2f} % "
                   f"of the stroke)"},
        {"group": "Motion", "name": "Transmission angle (working stroke)",
         "util": max(MU_MIN_DEG / min(mu_work), max(mu_work) / MU_MAX_DEG),
         "detail": f"{min(mu_work):.0f}..{max(mu_work):.0f}° (return stroke, unloaded: {min(mu_all):.0f}..{max(mu_all):.0f}°)"},
    ] + s_checks
    design = {"mech": m, "frames": frames, "segment": seg, "idx": idx, "stroke": seg["length"],
              "speed_variation": v_var, "v_mean_per_rad": v_mean, "T_max": T_max, "T_virtual": T_vw,
              "forces": forces, "section": section, "mu_work": (min(mu_work), max(mu_work)),
              "mu_all": (min(mu_all), max(mu_all)), "grashof": kind,
              "v_mean_mm_s": v_mean * rpm * 2 * math.pi / 60 if rpm else None}
    return design, checks
