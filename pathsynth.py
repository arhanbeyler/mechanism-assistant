"""
F2b: path synthesis: a crank-rocker whose coupler point passes through given points (no timing).

The coupler is one straight bar A-B-P with the point P on the line AB at the signed distance p from A (a
three-hole bar, as the Hoeckens linkage; easy to print). Unknowns: O2 (x, y), O4 (x, y), crank a, coupler b,
rocker c, point p: 8 numbers. Objective: the sum of squared distances from every target point to the closed
coupler curve (120 crank steps, the nearest chord), scaled by the size of the point set, plus penalties that
keep a Grashof crank-rocker (the crank the shortest link, s + l <= p + q), a transmission angle within
30..150 deg all round and link lengths within 1:8. Minimised by Nelder-Mead from many seeded random starts
(standard library; deterministic). Precision points in the classical sense are 3 to 5: more points are
fitted in the least-squares sense and the deviations reported.
Units: mm, degrees.
"""

import math
import random

N_CURVE = 120
MU_MIN = 30.0
STARTS = 40
ITER = 500


def _circ(p0, r0, p1, r1, sign):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    dist = math.hypot(dx, dy)
    if dist == 0 or dist > r0 + r1 or dist < abs(r0 - r1):
        return None
    k = (r0 * r0 - r1 * r1 + dist * dist) / (2 * dist)
    h = math.sqrt(max(0.0, r0 * r0 - k * k))
    mx, my = p0[0] + k * dx / dist, p0[1] + k * dy / dist
    return (mx - sign * h * dy / dist, my + sign * h * dx / dist)


def curve(x, n=N_CURVE, sign=1):
    """Coupler-point path, A, B and the transmission angles over a crank turn; None if it locks."""
    o2x, o2y, o4x, o4y, a, b, c, p = x
    pts, frames = [], []
    for i in range(n):
        th = 2 * math.pi * i / n
        A = (o2x + a * math.cos(th), o2y + a * math.sin(th))
        B = _circ(A, b, (o4x, o4y), c, sign)
        if B is None:
            return None
        ux, uy = (B[0] - A[0]) / b, (B[1] - A[1]) / b
        P = (A[0] + p * ux, A[1] + p * uy)
        v1, v2 = (A[0] - B[0], A[1] - B[1]), (o4x - B[0], o4y - B[1])
        cmu = (v1[0] * v2[0] + v1[1] * v2[1]) / (math.hypot(*v1) * math.hypot(*v2))
        mu = math.degrees(math.acos(max(-1.0, min(1.0, cmu))))
        pts.append(P)
        frames.append({"theta": math.degrees(th), "A": A, "B": B, "P": P, "mu": mu})
    return pts, frames


def distance_to_curve(q, pts):
    """Distance from q to the closed polyline pts and the index of the nearest segment."""
    best, idx = math.inf, 0
    n = len(pts)
    for i in range(n):
        p0, p1 = pts[i], pts[(i + 1) % n]
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0] - p0[0]) * dx + (q[1] - p0[1]) * dy) / L2))
        d = math.hypot(q[0] - p0[0] - t * dx, q[1] - p0[1] - t * dy)
        if d < best:
            best, idx = d, i
    return best, idx


def objective(x, targets, scale):
    o2x, o2y, o4x, o4y, a, b, c, p = x
    if min(a, b, c) <= 0.02 * scale:
        return 1e6
    d = math.hypot(o4x - o2x, o4y - o2y)
    L = sorted((a, b, c, d))
    pen = 0.0
    if a != L[0] or L[0] + L[3] > L[1] + L[2]:            # Grashof crank-rocker, crank the shortest
        pen += 100 * ((L[0] + L[3] - L[1] - L[2]) / scale) ** 2 + (1.0 if a != L[0] else 0.0) + 10.0
    if L[3] / L[0] > 8:
        pen += (L[3] / L[0] - 8) ** 2
    cv = curve(x)
    if cv is None:
        return 1e3 + pen
    pts, frames = cv
    mu_bad = sum(max(0.0, MU_MIN - f["mu"]) + max(0.0, f["mu"] - (180 - MU_MIN)) for f in frames) / len(frames)
    err = sum(distance_to_curve(q, pts)[0] ** 2 for q in targets) / (scale * scale)
    return err + pen + 0.01 * mu_bad ** 2


def nelder_mead(f, x0, step, iters=ITER, tol=1e-12):
    n = len(x0)
    simplex = [list(x0)] + [[x0[j] + (step[j] if j == i else 0.0) for j in range(n)] for i in range(n)]
    vals = [f(s) for s in simplex]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: vals[i])
        simplex, vals = [simplex[i] for i in order], [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) < tol:
            break
        cen = [sum(s[j] for s in simplex[:-1]) / n for j in range(n)]
        xr = [cen[j] + (cen[j] - simplex[-1][j]) for j in range(n)]
        fr = f(xr)
        if fr < vals[0]:
            xe = [cen[j] + 2 * (cen[j] - simplex[-1][j]) for j in range(n)]
            fe = f(xe)
            simplex[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            simplex[-1], vals[-1] = xr, fr
        else:
            xc = [cen[j] + 0.5 * (simplex[-1][j] - cen[j]) for j in range(n)]
            fc = f(xc)
            if fc < vals[-1]:
                simplex[-1], vals[-1] = xc, fc
            else:
                for i in range(1, n + 1):
                    simplex[i] = [simplex[0][j] + 0.5 * (simplex[i][j] - simplex[0][j]) for j in range(n)]
                    vals[i] = f(simplex[i])
    i = min(range(n + 1), key=lambda k: vals[k])
    return simplex[i], vals[i]


def synthesize(targets, starts=STARTS, seed=1):
    """Best crank-rocker for the target points. Returns (x, value, scale)."""
    xs, ys = [q[0] for q in targets], [q[1] for q in targets]
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    scale = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    rnd = random.Random(seed)
    best = (None, math.inf)
    f = lambda x: objective(x, targets, scale)
    for _ in range(starts):
        ang = rnd.uniform(0, 2 * math.pi)
        dist = scale * rnd.uniform(0.6, 1.6)
        gx, gy = cx + dist * math.cos(ang), cy + dist * math.sin(ang)       # where the ground link sits
        spread = rnd.uniform(0.5, 1.5) * scale
        g_dir = ang + math.pi / 2
        x0 = [gx - spread / 2 * math.cos(g_dir), gy - spread / 2 * math.sin(g_dir),
              gx + spread / 2 * math.cos(g_dir), gy + spread / 2 * math.sin(g_dir),
              scale * rnd.uniform(0.15, 0.4), scale * rnd.uniform(0.6, 1.4), scale * rnd.uniform(0.6, 1.4),
              scale * rnd.uniform(-0.5, 2.0)]
        x, v = nelder_mead(f, x0, [0.1 * scale] * 8)
        x, v = nelder_mead(f, x, [0.02 * scale] * 8)                          # restart: polish
        if v < best[1]:
            best = (x, v)
    return best[0], best[1], scale


def report(x, targets):
    """Deviations, crank angles at the targets, transmission angles and Grashof of a solution."""
    pts, frames = curve(x, 720)
    dev, where = [], []
    for q in targets:
        dd, i = distance_to_curve(q, pts)
        dev.append(dd)
        where.append(frames[i]["theta"])
    o2x, o2y, o4x, o4y, a, b, c, p = x
    d = math.hypot(o4x - o2x, o4y - o2y)
    L = sorted((a, b, c, d))
    return {"a": a, "b": b, "c": c, "d": d, "p": p, "O2": (o2x, o2y), "O4": (o4x, o4y), "deviations": dev,
            "max_dev": max(dev), "theta_at": where, "mu_min": min(f["mu"] for f in frames),
            "mu_max": max(f["mu"] for f in frames), "grashof": a == L[0] and L[0] + L[3] <= L[1] + L[2],
            "frames": frames, "path": pts}
