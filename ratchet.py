"""
F6: ratchet wheel and pawl (holds a torque one way, turns freely the other way).

Wheel (frame: centre at the origin, turns freely counter-clockwise): z saw teeth, tip radius R, tooth height
h = H_RATIO * pitch. Tooth k: tip at angle a_k = k 2 pi / z; its locking face runs inwards from the tip,
undercut by the hook angle alpha (the tip overhangs); the back flank rises from that root to the next tip
the pawl meets. Turning counter-clockwise, the pawl rides up the back flanks and drops at each face;
turning clockwise, it butts against a face.

Pawl: pivot on the tangent to the circle through the contact point (mid-face), a pawl length lp away. The
face pushes the pawl along its normal, which the hook tilts by alpha from that tangent: a moment
N lp sin(alpha) pulls the pawl in, friction mu N lp cos(alpha) pushes it out, so it self-engages when
tan(alpha) > mu (checked, with a margin).

Loads: holding torque T -> contact force F = T / r_c (r_c: contact radius). Tooth: bending at the root
(load at mid-face height), shear; face bearing over the contact; pawl: compression and buckling; pivot pin
shear; wheel shaft torsion (steel). The smallest wheel (radius in 5 mm steps, thickness from the list) that
passes is chosen, unless the diameter is given.
"""

import math

import strength

H_RATIO = 0.45           # tooth height / circular pitch
MU = {"PLA": 0.35, "PETG": 0.35, "aluminum": 0.3, "steel": 0.15}     # dry friction, pawl on tooth
HOOK_MARGIN = 5.0        # deg, hook angle beyond the friction angle
LP_RATIO = 0.8           # pawl length (pivot to contact) / tip radius
CLEAR = 0.2              # mm, pawl to tooth face in the locked pose
B_MAX_RATIO = 1.0        # wheel thickness (face width) at most one circular pitch
LIFT_MARGIN = 0.3        # deg, the riding pawl is kept this much clear (Fusion rounds joint angles to 0.1 deg)
STEEL = strength.MATERIALS["steel"]


def hook_angle(mat_key):
    """Hook angle: friction angle + margin, at least 12 deg."""
    return max(12.0, math.degrees(math.atan(MU.get(mat_key, 0.3))) + HOOK_MARGIN)


def geometry(z, R, alpha_deg):
    p = 2 * math.pi * R / z
    h = H_RATIO * p
    al = math.radians(alpha_deg)
    step = 2 * math.pi / z
    # locking face from the tip (R, 0) inwards by h, leaning alpha from the radius towards +angle (undercut)
    tip = (R, 0.0)
    root = (R - h, h * math.tan(al))                     # in the tooth's frame (tip on +x)
    return {"z": z, "R": R, "h": h, "pitch": p, "alpha": alpha_deg, "step": step, "tip": tip, "root": root,
            "r_root": math.hypot(*root)}


def outline(g):
    """Closed wheel outline (counter-clockwise), two points per tooth: tip, then the root of the next tooth's face.
    Going counter-clockwise from tip k the back flank falls to root k+1, then face k+1 rises to tip k+1."""
    pts = []
    rr, dr = g["r_root"], math.atan2(g["root"][1], g["root"][0])
    for k in range(g["z"]):
        a = k * g["step"]
        pts.append((g["R"] * math.cos(a), g["R"] * math.sin(a)))
        b = (k + 1) * g["step"] + dr                                      # root of the next tooth (undercut)
        pts.append((rr * math.cos(b), rr * math.sin(b)))
    return pts


def _inside(pts, q):
    """Point in polygon (even-odd)."""
    x, y = q
    inside = False
    n = len(pts)
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def pawl_pose(g, lp):
    """Locked pose around tooth 0 (tip on +x): contact point C at mid-face, the tangent there, pivot and the
    pawl's working edge (along the face, a clearance off it)."""
    tip, root = g["tip"], g["root"]
    # face direction (tip -> root) and its outward normal towards -angle (where the pawl sits)
    fx, fy = root[0] - tip[0], root[1] - tip[1]
    fl = math.hypot(fx, fy)
    f = (fx / fl, fy / fl)
    nrm = (-f[1], f[0])                                   # out of the face, into the tooth space (-angle side)
    C = (tip[0] + f[0] * fl / 2 + nrm[0] * CLEAR, tip[1] + f[1] * fl / 2 + nrm[1] * CLEAR)
    rc = math.hypot(*C)
    tang = (C[1] / rc, -C[0] / rc)                        # tangent at C pointing clockwise (towards -y)
    pivot = (C[0] + tang[0] * lp, C[1] + tang[1] * lp)
    e1 = (tip[0] + f[0] * 0.15 * fl + nrm[0] * CLEAR, tip[1] + f[1] * 0.15 * fl + nrm[1] * CLEAR)
    e2 = (tip[0] + f[0] * 0.92 * fl + nrm[0] * CLEAR, tip[1] + f[1] * 0.92 * fl + nrm[1] * CLEAR)
    return {"C": C, "rc": rc, "pivot": pivot, "face": f, "normal": nrm, "edge": (e1, e2), "tangent": tang}


def pawl_points(g, pose, w_head):
    """Pawl head in the locked pose: a wedge in the tooth space (front on the locking face, underside along
    the previous tooth's back flank, both a clearance off), topped by a block outside the tip circle.
    Returns the outline (e2, e1, A, B, P_flank) and the top middle point where the bar to the pivot starts."""
    e1, e2 = pose["edge"]
    step = g["step"]
    rr, dr = g["r_root"], math.atan2(g["root"][1], g["root"][0])
    root0 = (rr * math.cos(dr), rr * math.sin(dr))
    tipm1 = (g["R"] * math.cos(-step), g["R"] * math.sin(-step))
    fx, fy = tipm1[0] - root0[0], tipm1[1] - root0[1]
    fl = math.hypot(fx, fy)
    fn = (-fy / fl, fx / fl)                              # flank normal, pointing out of the wheel
    if fn[0] * root0[0] + fn[1] * root0[1] < 0:
        fn = (-fn[0], -fn[1])
    pf = (root0[0] + fx * 0.5 + fn[0] * CLEAR, root0[1] + fy * 0.5 + fn[1] * CLEAR)
    # deep corner: where the face and flank lines, each moved a clearance into the tooth space, meet
    f, nrm = pose["face"], pose["normal"]
    p1 = (g["tip"][0] + nrm[0] * CLEAR, g["tip"][1] + nrm[1] * CLEAR)
    p2 = (root0[0] + fn[0] * CLEAR, root0[1] + fn[1] * CLEAR)
    d2 = (fx / fl, fy / fl)
    den = f[0] * d2[1] - f[1] * d2[0]
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
    e2 = (p1[0] + f[0] * t, p1[1] + f[1] * t)
    top = g["R"] + w_head
    up = lambda q: (q[0] * top / math.hypot(*q), q[1] * top / math.hypot(*q))
    # front edge: the offset face line continued past the tip (the undercut tip overhangs a radial line)
    bq = p1[0] * f[0] + p1[1] * f[1]
    t_out = -bq - math.sqrt(bq * bq - (p1[0] ** 2 + p1[1] ** 2 - top * top))
    A = (p1[0] + f[0] * t_out, p1[1] + f[1] * t_out)
    B = up(pf)
    return [e2, e1, A, B, pf], ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)


def _rot(p, c, ang):
    ca, sa = math.cos(ang), math.sin(ang)
    x, y = p[0] - c[0], p[1] - c[1]
    return (c[0] + ca * x - sa * y, c[1] + sa * x + ca * y)


def _local_outline(g):
    """Teeth -2..2 around the locked tooth, closed through the centre (enough for the pawl's reach)."""
    pts = [(0.0, 0.0)]
    rr, dr = g["r_root"], math.atan2(g["root"][1], g["root"][0])
    for k in range(-2, 3):
        a = k * g["step"]
        pts.append((rr * math.cos(a + dr), rr * math.sin(a + dr)))      # root of tooth k (face bottom)
        pts.append((g["R"] * math.cos(a), g["R"] * math.sin(a)))        # its tip
    return pts


def ride_table(g, pose, head):
    """Pawl lift (deg about its pivot, away from the wheel) for the wheel turned 0..359 deg counter-clockwise,
    one value per degree. The least lift at which no sample of the pawl head is inside the wheel: scanned in
    0.2 deg steps (more lift is not always clearer near the root corner, so no plain bisection), then refined.
    One tooth period is computed and repeated."""
    wheel = _local_outline(g)
    pv = pose["pivot"]
    probe = _rot(pose["C"], pv, math.radians(1))
    lift_sign = 1 if math.hypot(*probe) > pose["rc"] else -1
    samples = []
    for i in range(len(head)):
        a, b = head[i], head[(i + 1) % len(head)]
        samples += [(a[0] + (b[0] - a[0]) * k / 10, a[1] + (b[1] - a[1]) * k / 10) for k in range(10)]

    def hits(phi, lift):
        for s in samples:
            q = _rot(_rot(s, pv, lift_sign * math.radians(lift)), (0.0, 0.0), -phi)
            if _inside(wheel, q):
                return True
        return False

    def least(phi):
        lift = 0.0
        while lift <= 45 and hits(phi, lift):
            lift += 0.2
        lo, hi = max(0.0, lift - 0.2), lift
        if lift == 0.0:
            return 0.0
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if hits(phi, mid) else (lo, mid)
        return hi + LIFT_MARGIN

    step = math.degrees(g["step"])
    n = int(math.ceil(step / 0.25)) + 1
    period = [least(math.radians(0.25 * i)) for i in range(n)]
    table = []
    for deg in range(360):
        u = (deg % step) / 0.25
        i = int(u)
        f = u - i
        v = period[i] * (1 - f) + period[min(i + 1, n - 1)] * f
        table.append(round(v if i + 1 < n or f == 0 else period[i], 4))
    return table, lift_sign


def checks_for(g, b, lp, w_pawl, d_pin, d_shaft, T, mat):
    """Strength checks (utilisations) for wheel thickness b, pawl width w_pawl, pin and shaft diameters."""
    n = mat["n"]
    allow = mat["sy"] / n
    pose = pawl_pose(g, lp)
    F = T / pose["rc"]
    s = 0.9 * g["pitch"]                                  # tooth root length
    M = F * g["h"] / 2
    sig = 6 * M / (b * s * s)
    tau = F / (b * s)
    brg = F / (b * 0.6 * g["h"])                          # contact over most of the face
    al = math.radians(g["alpha"])
    N = F / math.cos(al)
    I = min(b * w_pawl ** 3, w_pawl * b ** 3) / 12
    Pcr = math.pi ** 2 * mat["E"] * I / lp ** 2
    sig_p = N / (w_pawl * b)
    tau_pin = 4 * N / (math.pi * d_pin ** 2)
    tau_sh = 16 * T / (math.pi * d_shaft ** 3)
    return [
        {"group": "Tooth", "name": "Tooth bending at the root", "util": sig / allow, "detail": f"σ = {sig:.1f} MPa"},
        {"group": "Tooth", "name": "Tooth shear", "util": tau / (0.5 * allow), "detail": f"τ = {tau:.1f} MPa"},
        {"group": "Tooth", "name": "Face bearing", "util": brg / allow, "detail": f"σ_br = {brg:.1f} MPa"},
        {"group": "Pawl", "name": "Pawl buckling", "util": N * n / Pcr, "detail": f"N = {N:.0f} N, P_cr = {Pcr:.0f} N"},
        {"group": "Pawl", "name": "Pawl compression", "util": sig_p / allow, "detail": f"σ = {sig_p:.1f} MPa"},
        {"group": "Pin", "name": "Pawl pivot pin shear (steel)", "util": tau_pin / (0.5 * STEEL["sy"] / n),
         "detail": f"τ = {tau_pin:.1f} MPa"},
        {"group": "Pin", "name": "Wheel shaft torsion (steel)", "util": tau_sh / (0.5 * STEEL["sy"] / n),
         "detail": f"τ = {tau_sh:.1f} MPa"},
    ]


def design(T_Nmm, mat_key, z=24, D=None):
    """Ratchet for a holding torque T (N mm). Returns (design, checks)."""
    mat = strength.MATERIALS[mat_key]
    alpha = hook_angle(mat_key)
    radii = [D / 2] if D else [5.0 * k for k in range(3, 61)]
    best = None
    for R in radii:
        g = geometry(z, R, alpha)
        if g["r_root"] < 0.5 * R:
            continue                                        # teeth too deep for this few
        lp = LP_RATIO * R
        for b in [t for t in strength.THICKNESSES if t <= max(3.0, B_MAX_RATIO * g["pitch"])]:
            for d_pin in strength.PIN_DIAMETERS:
                w_pawl = max(2.5 * d_pin, 0.8 * g["h"] + 4)
                ds = next((d for d in strength.PIN_DIAMETERS + [16, 20, 25, 30]
                           if 16 * T_Nmm / (math.pi * d ** 3) <= 0.5 * STEEL["sy"] / mat["n"]), 30)
                if ds / 2 + 3 > g["r_root"] - g["h"]:
                    continue                                # no room for the shaft
                c = checks_for(g, b, lp, w_pawl, d_pin, ds, T_Nmm, mat)
                worst = max(x["util"] for x in c)
                vol = math.pi * R * R * b
                key = (worst <= 1.0, -vol if worst <= 1.0 else -worst)
                if best is None or key > best[0]:
                    best = (key, (g, b, lp, w_pawl, d_pin, ds, c))
                if worst <= 1.0:
                    break
            if best and best[0][0] and best[1][1] == b and best[1][0] is g:
                break
        if best and best[0][0] and not D:
            break                                           # the smallest passing radius
    if best is None:                                        # e.g. a diameter too small for the teeth and a shaft
        return None, [{"group": "Geometry", "name": "Room for the teeth and the shaft", "util": 9.9,
                       "detail": f"{z} teeth and the shaft for {T_Nmm / 1000:g} N·m do not fit Ø{D:g} mm" if D
                                 else f"no wheel up to Ø600 mm fits {z} teeth and the shaft"}]
    g, b, lp, w_pawl, d_pin, ds, c = best[1]
    mu = MU.get(mat_key, 0.3)
    checks = [{"group": "Motion", "name": "Pawl self-engages (hook > friction angle)",
               "util": math.degrees(math.atan(mu)) / g["alpha"],
               "detail": f"hook {g['alpha']:.1f}° > friction angle {math.degrees(math.atan(mu)):.1f}° (μ = {mu:g})"}] + c
    return {"geometry": g, "b": b, "lp": lp, "w_pawl": w_pawl, "d_pin": d_pin, "d_shaft": ds, "mu": mu,
            "F": T_Nmm / pawl_pose(g, lp)["rc"], "backlash_deg": 360.0 / g["z"]}, checks
