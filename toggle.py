"""
F5: toggle clamp (over-centre locking four-bar).

Links: base (ground), handle link O2-A (the handle is fixed to it), coupler A-B, clamp arm O4-B with the
spindle at arm_length from O4. When the handle link and the coupler are in line (O2, A, B collinear:
the "dead centre" or toggle), the arm is at its extreme position and cannot drive the handle back: the
mechanical advantage is infinite. The handle goes OVERCENTRE degrees past that line onto a stop, so a
force on the spindle pushes the handle further into the stop: the clamp locks itself.

Synthesis (normalised: arm link O4-B = 1): clamped arm along +x (B1 = (1, 0)), open arm turned by the
opening angle psi (B2). At the toggle A1 lies on O2-B1 at distance a from O2. With O2 = B1 + rho (cos beta,
sin beta), turning the handle back by the handle travel gives A2; |A2 - B2| = rho - a (the coupler length)
fixes rho by bisection. a / c and beta are swept; the linkage is simulated from open to past the dead centre
(same assembly branch, arm moving one way, transmission angle) and the best one kept (largest smallest
transmission angle, then the most compact).

Forces: holding force F at the spindle (perpendicular to the arm). Clamped: the coupler is a two-force
member, F_AB = F L / (c sin mu_B); past the dead centre its line passes a*sin(e) from O2, which presses the
handle onto the stop. Handle force to close against a clamping force: virtual work F_h = F (L / L_h) dpsi/dtheta.
"""

import math

import strength
from kinematics import MU_MAX_DEG, MU_MIN_DEG, _circle_intersections

OVERCENTRE = 3.0             # deg, handle travel past the dead centre (onto the stop)
A_RATIOS = (0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0)
LOADED_TRAVEL = 20.0         # deg of handle travel before the dead centre where the clamping force builds up
MU_OPEN_MIN = 20.0           # deg, smallest transmission angle on the unloaded part (no jamming)
N = 240                      # samples over the handle travel


def _rot(v, ang):
    c, s = math.cos(ang), math.sin(ang)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


def _cross(u, v):
    return u[0] * v[1] - u[1] * v[0]


def _mu(A, B):
    """Transmission angle at B between the coupler (B -> A) and the arm (B -> O4 = origin)."""
    u, v = (A[0] - B[0], A[1] - B[1]), (-B[0], -B[1])
    cm = (u[0] * v[0] + u[1] * v[1]) / (math.hypot(*u) * math.hypot(*v))
    return math.degrees(math.acos(max(-1.0, min(1.0, cm))))


def simulate(m, n=N):
    """From open (handle at th_open) through the dead centre to th_open + sign * (travel + overcentre).
    n samples up to the dead centre (frame n is exactly on it), then the same step on to the stop.
    Returns frames {th (deg, handle travel from open), A, B, psi (deg, arm angle), mu} or None (locks)."""
    O2, a, b, c = m["O2"], m["a"], m["b"], m["c"]
    step = m["travel"] / n
    ts = [step * i for i in range(n + 1)]
    k = max(1, math.ceil(OVERCENTRE / step))
    ts += [m["travel"] + OVERCENTRE * j / k for j in range(1, k + 1)]
    frames, prev = [], m["B_open"]
    for t in ts:
        th = math.radians(m["th_open"] + m["sign"] * t)
        A = (O2[0] + a * math.cos(th), O2[1] + a * math.sin(th))
        sols = _circle_intersections(A, b, (0.0, 0.0), c)
        if not sols:
            return None
        B = min(sols, key=lambda s: math.dist(s, prev))
        prev = B
        frames.append({"th": t, "A": A, "B": B, "psi": math.degrees(math.atan2(B[1], B[0])), "mu": _mu(A, B)})
    return frames


def _candidate(a, beta, psi, travel, sign):
    """Normalised linkage (c = 1) for handle link a and O2 direction beta from B1, or None."""
    B1, B2 = (1.0, 0.0), (math.cos(psi), math.sin(psi))
    u = (math.cos(beta), math.sin(beta))

    def f(rho):
        O2 = (B1[0] + rho * u[0], B1[1] + rho * u[1])
        A1 = (O2[0] - a * u[0], O2[1] - a * u[1])                  # on O2 -> B1, a from O2
        A2 = _rot((A1[0] - O2[0], A1[1] - O2[1]), -sign * travel)
        A2 = (O2[0] + A2[0], O2[1] + A2[1])
        return math.dist(A2, B2) - (rho - a), O2, A1, A2

    lo, hi = a * 1.05, 12.0
    flo, fhi = f(lo)[0], f(hi)[0]
    if flo * fhi > 0:
        return None
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if f(mid)[0] * flo > 0:
            lo, flo = mid, f(mid)[0]
        else:
            hi = mid
    rho = 0.5 * (lo + hi)
    _, O2, A1, A2 = f(rho)
    th_open = math.degrees(math.atan2(A2[1] - O2[1], A2[0] - O2[0]))
    return {"a": a, "b": rho - a, "c": 1.0, "O2": O2, "d": math.hypot(*O2), "th_open": th_open, "sign": sign,
            "travel": math.degrees(travel), "B_open": B2, "psi_open": math.degrees(psi)}


def _mu_margins(frames, travel):
    """Smallest distance of the transmission angle from 0 / 180°: near the lock (last LOADED_TRAVEL degrees
    before the dead centre and past it) and over the whole stroke."""
    near = [min(f["mu"], 180 - f["mu"]) for f in frames if f["th"] >= travel - LOADED_TRAVEL]
    whole = [min(f["mu"], 180 - f["mu"]) for f in frames]
    return min(near), min(whole)


def _evaluate(a, beta, psi, travel, sign):
    """(key, linkage) of one candidate, or None. key: feasible and compact first, else the least infeasible."""
    m = _candidate(a, beta, psi, travel, sign)
    if m is None or m["b"] <= 0.2 or m["d"] > 6:
        return None
    fr = simulate(m, 120)
    if fr is None:
        return None
    k_dc = 120                                                          # the dead-centre frame
    if math.dist(fr[k_dc]["B"], (1.0, 0.0)) > 1e-3:                   # another assembly branch
        return None
    psis = [f["psi"] for f in fr[:k_dc + 1]]
    if any(p1 > p0 + 1e-9 for p0, p1 in zip(psis, psis[1:])):           # the arm must close one way
        return None
    loaded, whole = _mu_margins(fr, m["travel"])
    score = min(loaded / MU_MIN_DEG, whole / MU_OPEN_MIN)
    size = m["a"] + m["b"] + m["d"]
    return ((1, -size) if score >= 1 else (0, score)), m


def synthesize(psi_deg, travel_deg):
    """Best normalised toggle linkage for arm opening psi and handle travel (open -> dead centre):
    a coarse sweep of a / c and beta, then a finer one around the five best."""
    psi, travel = math.radians(psi_deg), math.radians(travel_deg)
    found = []
    for a in A_RATIOS:
        for k in range(144):
            for sign in (1, -1):
                r = _evaluate(a, 2 * math.pi * k / 144, psi, travel, sign)
                if r:
                    found.append((r[0], a, 2 * math.pi * k / 144, sign, r[1]))
    if not found:
        return None
    found.sort(key=lambda x: x[0], reverse=True)
    best = found[0]
    for _, a0, b0, sign, _ in found[:5]:
        for i in range(-3, 4):
            for j in range(-12, 13):
                a = a0 + 0.05 * i
                if a <= 0.1:
                    continue
                r = _evaluate(a, b0 + math.radians(0.25 * j), psi, travel, sign)
                if r and r[0] > best[0]:
                    best = (r[0], a, b0, sign, r[1])
    return best[4]


def scale(m, k):
    out = dict(m)
    for key in ("a", "b", "c", "d"):
        out[key] = m[key] * k
    out["O2"] = (m["O2"][0] * k, m["O2"][1] * k)
    out["B_open"] = (m["B_open"][0] * k, m["B_open"][1] * k)
    return out


def analyse(m, frames, L_arm, F):
    """Dead-centre index, arm back-off past the dead centre, arm-speed ratio curve, locked-pose forces.
    Locked (on the stop): the workpiece pushes the spindle with F, opening the arm (torque F L_arm about O4).
    The coupler is a two-force member along A-B: its compression lam balances that torque at B; the arm
    pivot O4 takes the rest; past the dead centre the coupler line passes e from O2, so the handle link is
    pressed onto the stop with lam * e."""
    k_dc = next(i for i, f in enumerate(frames) if f["th"] >= m["travel"] - 1e-9)
    psi_dc = frames[k_dc]["psi"]
    backoff = frames[-1]["psi"] - psi_dc                               # >= 0: the arm lifts a little
    curve = []
    for i in range(1, k_dc):
        dpsi = (frames[i + 1]["psi"] - frames[i - 1]["psi"]) / (frames[i + 1]["th"] - frames[i - 1]["th"])
        curve.append((frames[i]["th"], abs(dpsi)))
    fr = frames[-1]
    A, B = fr["A"], fr["B"]
    c = math.hypot(*B)
    u = (B[0] / c, B[1] / c)                                           # along the arm
    nrm = (-u[1], u[0]) if frames[0]["psi"] > psi_dc else (u[1], -u[0])   # the arm's opening direction
    w = ((B[0] - A[0]) / m["b"], (B[1] - A[1]) / m["b"])               # coupler, A -> B
    lam = -F * L_arm / (c * _cross(u, w)) if F else 0.0                 # > 0: compression
    if F and lam < 0:
        lam = abs(lam)
    Fw = (F * nrm[0], F * nrm[1])
    R4 = math.hypot(Fw[0] + lam * w[0], Fw[1] + lam * w[1])            # pin O4
    e = abs(_cross(w, (m["O2"][0] - A[0], m["O2"][1] - A[1])))
    return {"k_dc": k_dc, "backoff_deg": backoff, "backoff_mm": math.radians(backoff) * L_arm, "psi_dc": psi_dc,
            "ratio_curve": curve, "F_AB": lam, "R_O4": R4, "T_stop": lam * e, "e": e, "mu_locked": fr["mu"],
            "M_arm": F * (L_arm - c)}


SPACER_GAP = 1.0             # mm, spacer post to a moving link


def link_regions(m, fr, L_arm, L_handle, section):
    """Moving link outlines in pose fr: (start, end, width, boss radius) for the handle link (O2 -> grip),
    the coupler and the arm (O4 -> spindle)."""
    O2, A, B = m["O2"], fr["A"], fr["B"]
    ua = (A[0] - O2[0], A[1] - O2[1])
    la = math.hypot(*ua)
    H = (O2[0] + L_handle * ua[0] / la, O2[1] + L_handle * ua[1] / la)
    lb = math.hypot(*B)
    S = (L_arm * B[0] / lb, L_arm * B[1] / lb)
    out = []
    for name, p, q in (("Crank", O2, H), ("Coupler", A, B), ("Rocker", (0.0, 0.0), S)):
        s = section["links"][name]
        out.append((p, q, s["width"], max(s["width"], s.get("boss0", 0.0)) / 2))
    return out


def spacer(m, frames, L_arm, L_handle, section, r_post):
    """The two base plates (bottom: O2, top: O4, so every pin joins neighbouring layers) are tied by a post.
    Candidate spots on the base line beyond O4 or O2 and beside its middle; the one with the largest gap to
    every moving link over the whole stroke: the nearest one with 3 mm to spare, else the largest gap.
    Returns (point, gap mm)."""
    from interference import region_distance
    O2 = m["O2"]
    d = math.hypot(*O2)
    u = (-O2[0] / d, -O2[1] / d)                          # O2 -> O4
    nrm = (-u[1], u[0])
    cands = [(k * u[0], k * u[1]) for k in range(6, 121, 3)]                           # beyond O4
    cands += [(O2[0] - k * u[0], O2[1] - k * u[1]) for k in range(6, 121, 3)]          # beyond O2
    mid = (O2[0] / 2, O2[1] / 2)
    cands += [(mid[0] + sg * k * nrm[0], mid[1] + sg * k * nrm[1]) for k in range(10, 121, 5) for sg in (1, -1)]
    scored = []
    for p in cands:
        gap = float("inf")
        for fr in frames[::3]:
            for a, b, w, R in link_regions(m, fr, L_arm, L_handle, section):
                gap = min(gap, region_distance(p, a, b, w, R, R) - r_post)
        scored.append((p, gap))
    clear = [x for x in scored if x[1] >= 3 * SPACER_GAP]
    if clear:                                                  # the nearest clear spot (short plates)
        return min(clear, key=lambda x: math.dist(x[0], mid))
    return max(scored, key=lambda x: x[1])


def handle_torque(m, fr, F, L_arm):
    """Handle torque (N mm) that holds a spindle force F in pose fr, from statics: arm torque F L_arm ->
    coupler compression (two-force member) -> its moment about O2. Zero at the dead centre."""
    A, B = fr["A"], fr["B"]
    c = math.hypot(*B)
    u = (B[0] / c, B[1] / c)
    w = ((B[0] - A[0]) / m["b"], (B[1] - A[1]) / m["b"])
    lam = F * L_arm / (c * abs(_cross(u, w)))
    return lam * abs(_cross(w, (m["O2"][0] - A[0], m["O2"][1] - A[1])))


def size_clamp(an, m, L_arm, mat):
    """Pins (steel, single shear) + lightest section per link for the locked pose. Handle link: coupler
    compression + the stop torque; coupler: compression; arm: bending at B (the spindle overhangs B);
    base: the larger pin force. Returns (section, checks) with section None if nothing fits."""
    steel = dict(strength.MATERIALS["steel"], n=mat["n"])
    P = max(an["F_AB"], an["R_O4"])
    link_forces = {"Crank": {"F_ab_max": an["F_AB"], "M_crank_max": an["T_stop"], "M_rocker": 0.0},
                   "Coupler": {"F_ab_max": an["F_AB"], "M_crank_max": 0.0, "M_rocker": 0.0},
                   "Rocker": {"F_ab_max": P, "M_crank_max": 0.0, "M_rocker": an["M_arm"]},
                   "Ground": {"F_ab_max": P, "M_crank_max": 0.0, "M_rocker": 0.0}}
    lengths = {"Crank": m["a"], "Coupler": m["b"], "Rocker": L_arm, "Ground": m["d"]}
    best = None
    for d in strength.PIN_DIAMETERS:
        pin = strength.check_pin({"F_ab_max": P}, steel, d)
        pin["name"] = "Pin shear (steel)"
        if pin["util"] > 1:
            continue
        links, checks, vol = {}, [pin], 0.0
        for name, L in lengths.items():
            found = strength._lightest_link(name, link_forces[name], L, mat, d, d)
            if found is None:
                break
            w_, t_, c_ = found
            links[name] = {"width": float(w_), "thickness": float(t_)}
            checks += c_
            boss = max(w_, strength.BOSS_RATIO * d)
            vol += t_ * (w_ * L + math.pi * boss ** 2 / 4)             # web + the two boss halves
        else:
            if best is None or vol < best[2]:
                best = ({"hole": float(d), "shaft": float(d), "links": links}, checks, vol)
    if best:
        return best[0], best[1]
    d = strength.PIN_DIAMETERS[-1]
    fail = [strength.check_pin({"F_ab_max": P}, steel, d)]
    for name, L in lengths.items():
        fail += strength.check_link(name, link_forces[name], L, mat, strength.MAX_WIDTH, strength.THICKNESSES[-1], d, d)
    return None, fail


def design(psi_deg, travel_deg, L_arm, mat, F=0.0, L_handle=None):
    """Toggle clamp for arm opening psi, handle travel, spindle distance L_arm (mm) and holding force F (N).
    The arm link O4-B is half the spindle distance. Returns (design, checks) or (None, reason)."""
    unit = synthesize(psi_deg, travel_deg)
    if unit is None:
        return None, f"No toggle linkage found for a {psi_deg:g}° opening with {travel_deg:g}° of handle travel."
    c = L_arm / 2
    m = scale(unit, c)
    L_handle = L_handle or 2.0 * L_arm
    frames = simulate(m)
    an = analyse(m, frames, L_arm, F)
    near = [f["mu"] for f in frames if f["th"] >= m["travel"] - LOADED_TRAVEL]
    mus = [f["mu"] for f in frames]
    section, s_checks = size_clamp(an, m, L_arm, mat) if F else (None, [])
    checks = [
        {"group": "Motion", "name": "Arm opening (simulation)",
         "util": 0.0 if abs(frames[0]["psi"] - an["psi_dc"] - psi_deg) < 0.5 else 2.0,
         "detail": f"{frames[0]['psi'] - an['psi_dc']:.1f}° (target {psi_deg:g}°)"},
        {"group": "Motion", "name": "Dead centre reached, then the stop",
         "util": 0.0 if an["backoff_deg"] >= 0 else 2.0,
         "detail": f"{OVERCENTRE:g}° past the dead centre the arm lifts {an['backoff_deg']:.2f}° "
                   f"({an['backoff_mm']:.2f} mm at the spindle): the clamp must squeeze more than this to stay locked"},
        {"group": "Motion", "name": "Transmission angle near the lock",
         "util": max(MU_MIN_DEG / min(near), max(near) / MU_MAX_DEG),
         "detail": f"{min(near):.0f}..{max(near):.0f}° over the last {LOADED_TRAVEL:g}° of handle travel "
                   f"(limits {MU_MIN_DEG:g}..{MU_MAX_DEG:g}°)"},
        {"group": "Motion", "name": "Transmission angle, whole stroke (unloaded)",
         "util": MU_OPEN_MIN / min(min(mus), 180 - max(mus)),
         "detail": f"{min(mus):.0f}..{max(mus):.0f}° (at least {MU_OPEN_MIN:g}° from 0 / 180° so it cannot jam)"},
    ] + s_checks
    return {"mech": m, "frames": frames, "analysis": an, "L_arm": L_arm, "L_handle": L_handle,
            "section": section}, checks
