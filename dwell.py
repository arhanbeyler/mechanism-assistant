"""
F4d: single-dwell six-bar (Norton ch. 3: a four-bar coupler curve with a near-circular arc + a dyad).

Stage 1: a crank-rocker (crank a, O2 at the origin, O4 at (d, 0)) whose coupler point P (on the coupler's line,
at p from A) runs along a nearly circular arc while the crank turns through the dwell angle. The arc is found
by optimisation (Nelder-Mead from seeded starts): the circle fitted to P over the dwell window (Kasa least
squares) must hold P within a small band, with Grashof and transmission-angle penalties.
Stage 2: link 5 of the arc's radius from P to D, D at the arc's centre, and the output rocker O6-D: while P is on
the arc D does not move, so the output dwells. O6 and the rocker length are chosen for the asked output swing
with the best transmission angle at D.
Everything is simulated over a crank turn: the dwell (output motion within the window), the swing, both
transmission angles.  Units: mm, degrees.
"""

import math
import random

import pathsynth

N = 360
MU_MIN = 35.0
DEPART = 1.0          # P leaves the circle by at least a crank length outside the dwell


def kasa(points):
    """Least-squares circle through points: (cx, cy, R, max radial deviation)."""
    n = len(points)
    sx = sum(p[0] for p in points) / n
    sy = sum(p[1] for p in points) / n
    u = [(p[0] - sx, p[1] - sy) for p in points]
    suu = sum(x * x for x, _ in u)
    svv = sum(y * y for _, y in u)
    suv = sum(x * y for x, y in u)
    suuu = sum(x ** 3 for x, _ in u)
    svvv = sum(y ** 3 for _, y in u)
    suvv = sum(x * y * y for x, y in u)
    svuu = sum(y * x * x for x, y in u)
    A = [[suu, suv], [suv, svv]]
    b = [0.5 * (suuu + suvv), 0.5 * (svvv + svuu)]
    det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
    if abs(det) < 1e-12:
        return None
    uc = (b[0] * A[1][1] - b[1] * A[0][1]) / det
    vc = (A[0][0] * b[1] - A[1][0] * b[0]) / det
    cx, cy = uc + sx, vc + sy
    R = math.sqrt(uc * uc + vc * vc + (suu + svv) / n)
    dev = max(abs(math.hypot(p[0] - cx, p[1] - cy) - R) for p in points)
    return cx, cy, R, dev


def _stage1(x, dwell_deg):
    """x = (b, c, d, p, th0) with a = 1. Returns (frames, window indices, circle) or None."""
    b, c, d, p, th0 = x
    cv = pathsynth.curve((0.0, 0.0, d, 0.0, 1.0, b, c, p), N)
    if cv is None:
        return None
    pts, frames = cv
    k0 = int(round(th0 % 360))
    idx = [(k0 + k) % N for k in range(int(round(dwell_deg)) + 1)]
    circ = kasa([pts[i] for i in idx])
    return (pts, frames, idx, circ) if circ else None


def objective(x, dwell_deg):
    b, c, d, p, th0 = x
    if min(b, c, d) < 1.2 or max(b, c, d, abs(p)) > 8:
        return 1e3
    L = sorted((1.0, b, c, d))
    pen = 0.0
    if L[0] != 1.0 or L[0] + L[3] > L[1] + L[2]:
        pen += 10 + 100 * (L[0] + L[3] - L[1] - L[2]) ** 2
    s = _stage1(x, dwell_deg)
    if s is None:
        return 1e2 + pen
    pts, frames, idx, (cx, cy, R, dev) = s
    mu_bad = sum(max(0.0, MU_MIN - f["mu"]) + max(0.0, f["mu"] - (180 - MU_MIN)) for f in frames) / N
    size = max(max(q[0] for q in pts) - min(q[0] for q in pts), max(q[1] for q in pts) - min(q[1] for q in pts))
    # not the trivial arcs: P at A (the crank circle) or at B (the rocker's arc), a centre on a pivot
    if min(abs(p), abs(p - b)) < 0.5:
        pen += 10 * (0.5 - min(abs(p), abs(p - b))) + 1
    if min(math.hypot(cx, cy), math.hypot(cx - d, cy)) < 0.8:
        pen += 5 * (0.8 - min(math.hypot(cx, cy), math.hypot(cx - d, cy))) + 1
    # outside the window P must leave the circle clearly, or the output hardly moves
    depart = max(abs(math.hypot(q[0] - cx, q[1] - cy) - R) for q in pts)
    if depart < DEPART:
        pen += (DEPART - depart) ** 2 * 5
    return (dev / size) ** 2 * 1e4 + pen + 0.01 * mu_bad ** 2 + (0.0 if R < 3 * size else (R / size - 3) ** 2)


def stage1(dwell_deg, starts=30, seed=3):
    rnd = random.Random(seed)
    best = (None, math.inf)
    f = lambda x: objective(x, dwell_deg)
    for _ in range(starts):
        x0 = [rnd.uniform(2, 4), rnd.uniform(2, 4), rnd.uniform(2.5, 4.5), rnd.uniform(1, 5), rnd.uniform(0, 360)]
        x, v = pathsynth.nelder_mead(f, x0, [0.3, 0.3, 0.3, 0.5, 20.0], iters=400)
        x, v = pathsynth.nelder_mead(f, x, [0.05, 0.05, 0.05, 0.1, 3.0], iters=300)
        if v < best[1]:
            best = (x, v)
    return best


def _circ(p0, r0, p1, r1, prev):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    dist = math.hypot(dx, dy)
    if dist == 0 or dist > r0 + r1 or dist < abs(r0 - r1):
        return None
    k = (r0 * r0 - r1 * r1 + dist * dist) / (2 * dist)
    h = math.sqrt(max(0.0, r0 * r0 - k * k))
    mx, my = p0[0] + k * dx / dist, p0[1] + k * dy / dist
    sols = [(mx + h * dy / dist, my - h * dx / dist), (mx - h * dy / dist, my + h * dx / dist)]
    return min(sols, key=lambda s: math.dist(s, prev))


def simulate(m):
    """Whole six-bar: frames with P, D, the output angle and both transmission angles; None if it locks."""
    x = (0.0, 0.0, m["d"], 0.0, m["a"], m["b"], m["c"], m["p"])
    pts, frames = pathsynth.curve(x, N)
    O6, c6, r5 = m["O6"], m["c6"], m["r5"]
    prev = m["D0"]
    out = []
    order = list(range(m["k0"], N)) + list(range(0, m["k0"]))
    for i in order:
        P = pts[i]
        D = _circ(P, r5, O6, c6, prev)
        if D is None or math.dist(D, prev) > 0.3 * c6:
            return None
        prev = D
        v1, v2 = (P[0] - D[0], P[1] - D[1]), (O6[0] - D[0], O6[1] - D[1])
        mu6 = math.degrees(math.acos(max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (r5 * c6)))))
        out.append(dict(frames[i], D=D, mu6=mu6, phi=math.degrees(math.atan2(D[1] - O6[1], D[0] - O6[0]))))
    return out


def _unwrap(a):
    o = [a[0]]
    for v in a[1:]:
        o.append(o[-1] + (v - o[-1] + 180) % 360 - 180)
    return o


def measure(fr, dwell_deg):
    ph = _unwrap([f["phi"] for f in fr])
    w = int(round(dwell_deg)) + 1
    return {"swing": max(ph) - min(ph), "dwell_motion": max(ph[:w]) - min(ph[:w]),
            "mu1_min": min(f["mu"] for f in fr), "mu1_max": max(f["mu"] for f in fr),
            "mu6_min": min(f["mu6"] for f in fr), "mu6_max": max(f["mu6"] for f in fr)}


def design(a, dwell_deg, swing_deg):
    """Dwell six-bar with crank a (mm), a dwell over dwell_deg of the crank and an output swing swing_deg."""
    x, v = stage1(dwell_deg)
    s = _stage1(x, dwell_deg)
    if s is None:
        return None, None
    b, c, d, p, th0 = x
    pts, frames, idx, (cx, cy, R, dev) = s
    k = a                                                 # scale: a = 1 in the search
    m = {"a": a, "b": b * k, "c": c * k, "d": d * k, "p": p * k, "k0": idx[0], "r5": R * k,
         "Dc": (cx * k, cy * k), "arc_dev": dev * k}
    ptsk = [(q[0] * k, q[1] * k) for q in pts]
    best = None
    for c6f in (0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.3, 1.6, 2.0, 2.5, 3.0):
        c6 = c6f * m["r5"]
        for ang in range(0, 360, 5):
            O6 = (m["Dc"][0] + c6 * math.cos(math.radians(ang)), m["Dc"][1] + c6 * math.sin(math.radians(ang)))
            mm = dict(m, O6=O6, c6=c6, D0=m["Dc"])
            fr = simulate(mm)
            if not fr:
                continue
            me = measure(fr, dwell_deg)
            mus = min(me["mu1_min"], 180 - me["mu1_max"], me["mu6_min"], 180 - me["mu6_max"])
            key = (abs(me["swing"] - swing_deg) <= 2.0 and mus >= MU_MIN, -abs(me["swing"] - swing_deg), mus)
            if best is None or key > best[0]:
                best = (key, dict(mm, measured=me, ok=key[0] and me["dwell_motion"] <= 1.0), fr)
    if best is None:
        return None, None
    return best[1], best[2]
