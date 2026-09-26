"""
F1: Slider-crank (piston) mechanism: synthesis, simulation and sizing. Norton, Chapter 4.

Frame: crank pivot O2 at the origin, slide axis along +x at height e (offset; 0 = in-line).
    a = crank, b = connecting rod (coupler), theta = crank angle from +x.
    Piston pin:  x(theta) = a cos(theta) + sqrt(b^2 - (a sin(theta) - e)^2),  y = e
    Rod angle to the slide axis:  phi = asin((a sin(theta) - e) / b)
    Stroke (exact, with offset):  S = sqrt((a+b)^2 - e^2) - sqrt((b-a)^2 - e^2)   (in-line: S = 2a)
Synthesis: the stroke is given; a follows from S (bisection when e != 0). The rod length b is the smallest
whole millimetre with b >= RATIO_MIN * a (rule of thumb for smooth running) AND |phi|max <= PHI_MAX
(the rod's pressure angle on the piston: side load on the guide).
Forces (quasi-static, a constant piston force F along the slide, inertia neglected):
    rod force  F_rod = F / cos(phi);  crank torque from virtual work  T(theta) = F |dx/dtheta|
Sizing: strength.check_link / check_pin / check_shaft for the crank (O2 shaft, A pin) and the rod (A, B pins).
Units: mm, N, N*mm, degrees, rpm. Standard library only (also runs inside Fusion).
"""

import math

import strength

RATIO_MIN = 3.0          # rod / crank >= 3 (smoothness rule of thumb)
PHI_MAX = 20.0           # degrees, largest rod angle to the slide axis (side load on the guide)
N_SIM = 720


def position(a, b, e, th):
    """Piston pin x and rod angle phi (rad) at crank angle th (rad); None if the rod cannot reach."""
    s = a * math.sin(th) - e
    if abs(s) > b:
        return None
    return a * math.cos(th) + math.sqrt(b * b - s * s), math.asin(s / b)


def dx_dth(a, b, e, th):
    s = a * math.sin(th) - e
    return -a * math.sin(th) - s * a * math.cos(th) / math.sqrt(b * b - s * s)


def stroke_of(a, b, e):
    return math.sqrt((a + b) ** 2 - e * e) - math.sqrt((b - a) ** 2 - e * e)


def solve_crank(S, b, e):
    """Crank a giving stroke S with rod b and offset e. The crank must turn fully: b >= a + |e|.
    The stroke grows with a, so bisection on 0 < a < b - |e|. None if S cannot be reached."""
    if e == 0:
        return S / 2 if S / 2 <= b else None
    lo, hi = 0.0, b - abs(e)
    if hi <= 0 or stroke_of(hi, b, e) < S:
        return None
    for _ in range(200):
        mid = (lo + hi) / 2
        if stroke_of(mid, b, e) < S:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def simulate(a, b, e, n=N_SIM):
    """One crank turn: [{"theta", "x", "phi", "dx"}] (theta and phi in degrees, dx = dx/dtheta in mm/rad)."""
    out = []
    for i in range(n):
        th = 2 * math.pi * i / n
        x, phi = position(a, b, e, th)
        out.append({"theta": math.degrees(th), "x": x, "phi": math.degrees(phi), "dx": dx_dth(a, b, e, th)})
    return out


def measure(frames):
    """Stroke, time ratio (forward / return crank angle), largest rod angle."""
    xs = [f["x"] for f in frames]
    n = len(frames)
    i_max, i_min = xs.index(max(xs)), xs.index(min(xs))
    out_deg = ((i_min - i_max) % n) * 360.0 / n          # crank angle from the outer to the inner dead centre
    back_deg = 360.0 - out_deg
    return {"stroke": max(xs) - min(xs), "x_min": min(xs), "x_max": max(xs),
            "Q": max(out_deg, back_deg) / min(out_deg, back_deg),
            "phi_max": max(abs(f["phi"]) for f in frames)}


def synthesize(stroke, offset=0.0, rod=None):
    """Returns the geometry {a, b, e, ratio} or None. rod: the user's rod length (else the engine picks the
    smallest whole millimetre meeting RATIO_MIN and PHI_MAX)."""
    e = float(offset or 0.0)
    if rod:
        a = solve_crank(stroke, float(rod), e)
        return None if a is None else {"a": a, "b": float(rod), "e": e, "ratio": rod / a}
    b = math.ceil(RATIO_MIN * stroke / 2 + abs(e) - 1e-9)
    for _ in range(2000):
        a = solve_crank(stroke, b, e)
        if a is not None and b / a >= RATIO_MIN - 1e-9 and measure(simulate(a, b, e, 360))["phi_max"] <= PHI_MAX:
            return {"a": a, "b": float(b), "e": e, "ratio": b / a}
        b += 1
    return None


def forces(frames, a, F):
    """Largest rod force and crank torque over the cycle for a constant piston force F."""
    F_rod = max(F / math.cos(math.radians(f["phi"])) for f in frames)
    T = max(F * abs(f["dx"]) for f in frames)
    return {"F": F, "F_ab_max": F_rod, "M_crank_max": T, "M_rocker": 0.0}


def size(forc, a, b, mat):
    """Pin (A, B) + shaft (O2) diameters and the lightest crank / rod sections (smallest volume).
    Returns (section, checks) or (None, checks at the largest section)."""
    lengths = {"Crank": a, "Coupler": b}
    best = None
    for d in strength.PIN_DIAMETERS:
        pin = strength.check_pin(forc, mat, d)
        if pin["util"] > 1.0:
            continue
        for ds in (x for x in strength.PIN_DIAMETERS if x >= d):
            shaft = strength.check_shaft(forc, mat, ds)
            if shaft["util"] > 1.0:
                continue
            links, checks, volume = {}, [pin, shaft], 0.0
            for name, L in lengths.items():
                found = strength._lightest_link(name, forc, L, mat, d, ds)
                if found is None:
                    break
                w, t, c = found
                b0, b1 = strength.boss_diameters(name, w, d, ds)
                links[name] = {"width": float(w), "thickness": float(t), "boss0": float(b0), "boss1": float(b1)}
                checks += c
                volume += t * (w * L + math.pi * (b0 ** 2 + b1 ** 2) / 8)
            else:
                if best is None or volume < best[2] - 1e-9:
                    best = ({"hole": float(d), "shaft": float(ds), "links": links}, checks, volume)
    if best:
        return best[0], best[1]
    d, w, t = strength.PIN_DIAMETERS[-1], strength.MAX_WIDTH, strength.THICKNESSES[-1]
    fail = [strength.check_pin(forc, mat, d), strength.check_shaft(forc, mat, d)]
    for name, L in lengths.items():
        fail += strength.check_link(name, forc, L, mat, w, t, d, d)
    return None, fail


def kinematics_at(frames, rpm):
    """Piston velocity (m/s) and acceleration (m/s^2) over the cycle at a constant crank speed."""
    w = rpm * 2 * math.pi / 60
    n = len(frames)
    h = 2 * math.pi / n
    v = [f["dx"] * w / 1000 for f in frames]
    acc = [(frames[(i + 1) % n]["dx"] - frames[i - 1]["dx"]) / (2 * h) * w * w / 1000 for i in range(n)]
    return v, acc
