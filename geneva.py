"""
F3: external Geneva mechanism (intermittent rotation), Norton / Shigley geometry.

Frame: driver (crank + pin + locking disk) turns about the origin, counter-clockwise; the wheel with n radial
slots turns about (C, 0). theta: driver angle from the line of centres; phi: wheel rotation from its rest
position (negative = clockwise; one index turns it by -2 pi / n).

Geometry for a tangential (shock-free) pin entry: the slot is radial and the pin enters along it, so
    crank radius a = C sin(pi / n),  pin entry radius R = C cos(pi / n),
    the driver indexes during 2 alpha = pi - 2 pi / n and dwells for the rest of the turn.
Wheel (rest position): n slots at angles pi + pi/n + k 2 pi/n (obround, pin + clearance wide, from the rim
down to where the pin comes closest, C - a), and n concave locking arcs centred on the driver axis
(angles pi + k 2 pi/n, radius r_lock + clearance). Driver: locking disk of radius r_lock with a crescent
relief (a circle centred at distance C in the pin's direction) so the wheel's tips pass during indexing; its
radius comes from sweeping the wheel's outline over the index (relief_radius), not from the rim radius.

Loads: the output inertia J (load + wheel) driven through the angular acceleration of the index:
T_wheel = J alpha_w, pin force F = T_wheel / r (r: pin to wheel centre, force normal to the slot),
driver torque T_d = T_wheel |dphi/dtheta|. Pin (steel): shear and bending as a cantilever from the crank
plate; slot flank: bearing stress on the wheel material; both shafts: torsion.
"""

import math

import strength

CLEAR = 0.3          # mm, clearance per side: pin in the slot, locking disk in the wheel's arcs
GAP = 1.0            # mm, crank plate above the wheel
WEB = 3.0            # mm, wheel material between the shaft and the slot bottoms
LOCK_RATIO = 1.25    # locking disk radius r_lock = a - LOCK_RATIO * pin diameter
PIN_MIN_RATIO = 8    # pin at least a / 8 (wear and stiffness; strength alone allows toy pins)
N_MIN, N_MAX = 3, 12
N_STEPS = 720        # samples per driver turn (0.5 deg)
FREE_MAX = 1.0       # deg, driver angle after an index before the disk blocks overrunning
DENSITY = {"PLA": 1.24e-6, "PETG": 1.27e-6, "aluminum": 2.70e-6, "steel": 7.85e-6}   # kg/mm^3
STEEL = strength.MATERIALS["steel"]


def geometry(n, C, d_pin, clr=CLEAR):
    b = math.pi / n
    a = C * math.sin(b)
    R = C * math.cos(b)
    rp = d_pin / 2
    R_o = math.sqrt(R * R + rp * rp)                  # wheel rim: the slot mouth corners
    r_lock = a - LOCK_RATIO * d_pin
    return {"n": n, "C": C, "a": a, "R": R, "R_o": R_o, "d_pin": d_pin, "clr": clr,
            "alpha": math.pi / 2 - b,                  # half the driver's indexing angle
            "slot_w": d_pin + 2 * clr,
            "slot_r_min": C - a - clr,                 # slot centreline end (the pin centre comes to C - a)
            "slot_bottom": C - a - rp - 2 * clr,       # deepest wheel material removed
            "r_lock": r_lock, "r_arc": r_lock + clr,
            "relief_r": R_o + clr, "step_deg": 360.0 / n}


def _slot_dist(g, q, k):
    """Signed distance of wheel-frame point q from slot k's outline (< 0: inside the slot)."""
    be = math.pi + math.pi / g["n"] + k * 2 * math.pi / g["n"]
    u = (math.cos(be), math.sin(be))
    s = max(g["slot_r_min"], q[0] * u[0] + q[1] * u[1])      # the slot runs outwards from slot_r_min
    return math.hypot(q[0] - s * u[0], q[1] - s * u[1]) - g["slot_w"] / 2


def _arc_dist(g, q, k):
    """Signed distance from locking arc k's circle (< 0: inside the cut)."""
    ga = math.pi + k * 2 * math.pi / g["n"]
    return math.hypot(q[0] - g["C"] * math.cos(ga), q[1] - g["C"] * math.sin(ga)) - g["r_arc"]


def in_wheel(g, q, tol=0.0):
    """Is the wheel-frame point q (rest position) wheel material?"""
    if math.hypot(*q) > g["R_o"] + tol:
        return False
    return all(_slot_dist(g, q, k) >= -tol and _arc_dist(g, q, k) >= -tol for k in range(g["n"]))


def _outline(g, step_deg=0.5):
    """Points on the wheel's outline (rest position): rim, slot walls and bottoms, locking-arc edges."""
    n, pts = g["n"], []
    for i in range(int(360 / step_deg)):
        t = math.radians(i * step_deg)
        pts.append((g["R_o"] * math.cos(t), g["R_o"] * math.sin(t)))
    for k in range(n):
        be = math.pi + math.pi / n + k * 2 * math.pi / n
        u, v = (math.cos(be), math.sin(be)), (-math.sin(be), math.cos(be))
        h = g["slot_w"] / 2
        for j in range(41):
            r = g["slot_r_min"] + (g["R_o"] - g["slot_r_min"]) * j / 40
            pts += [(r * u[0] + sg * h * v[0], r * u[1] + sg * h * v[1]) for sg in (-1, 1)]
            t = math.pi / 2 + math.pi * j / 40                    # round slot bottom
            pts.append((g["slot_r_min"] * u[0] + h * (math.cos(t) * v[0] - math.sin(t) * u[0]),
                        g["slot_r_min"] * u[1] + h * (math.cos(t) * v[1] - math.sin(t) * u[1])))
        ga = math.pi + k * 2 * math.pi / n
        for j in range(181):
            t = ga + math.pi - math.pi / 2 + math.pi * j / 180   # the half of the arc circle facing the wheel
            pts.append((g["C"] * math.cos(ga) + g["r_arc"] * math.cos(t), g["C"] * math.sin(ga) + g["r_arc"] * math.sin(t)))
    return [q for q in pts if in_wheel(g, q, tol=1e-6)]


def relief_radius(g, dth_deg=1.0):
    """Relief radius that clears the wheel over the whole index: the farthest wheel outline point that comes
    within the locking disk's reach, measured from the relief centre (distance C in the pin's direction),
    plus the clearance. A plain rim-radius circle leaves the rim next to the engaged slot ~0.1 mm proud a few
    degrees off mid-index."""
    pts = _outline(g)
    C, reach = g["C"], g["r_lock"] - 1e-6                # strictly inside the disk
    need = g["R_o"]
    steps = int(2 * math.degrees(g["alpha"]) / dth_deg)
    for i in range(steps + 1):
        th = -g["alpha"] + 2 * g["alpha"] * i / steps
        phi = wheel_angle(g, th)
        c, s = math.cos(phi), math.sin(phi)
        rc = (C * math.cos(th), C * math.sin(th))
        for q in pts:
            p = (C + c * q[0] - s * q[1], s * q[0] + c * q[1])
            if p[0] * p[0] + p[1] * p[1] <= reach * reach:
                need = max(need, math.hypot(p[0] - rc[0], p[1] - rc[1]))
    return need + g["clr"]


def wheel_angle(g, theta):
    """Wheel rotation phi (rad) from rest while the pin is in a slot (|theta| <= alpha): slot direction
    towards the pin, measured from the rest direction of the entering slot (pi + pi/n)."""
    a, C = g["a"], g["C"]
    psi = math.atan2(a * math.sin(theta), a * math.cos(theta) - C)
    return math.remainder(psi - (math.pi + math.pi / g["n"]), 2 * math.pi)


def motion(g, n_steps=N_STEPS):
    """Over one driver turn from the start of an index (theta = -alpha): list of (theta deg, phi rad,
    dphi/dtheta, d2phi/dtheta2). phi goes 0 -> -2 pi/n during the index, then stays (dwell)."""
    lam = g["a"] / g["C"]
    out = []
    for i in range(n_steps):
        th = -g["alpha"] + 2 * math.pi * i / n_steps
        if th <= g["alpha"] + 1e-12:
            den = 1 - 2 * lam * math.cos(th) + lam * lam
            w = -lam * (math.cos(th) - lam) / den            # clockwise wheel for a ccw driver
            acc = -lam * (lam * lam - 1) * math.sin(th) / den ** 2
            out.append((math.degrees(th), wheel_angle(g, th), w, acc))
        else:
            out.append((math.degrees(th), -2 * math.pi / g["n"], 0.0, 0.0))
    return out


def lock_free_angle(g):
    """After an index, the driver angle during which the wheel is not yet blocked against overrunning.
    The disk rim blocks wheel rotation only where it lies in the wheel's arc; rotation one way pushes the
    arc's points on one side of the line of centres into the disk, the other way those on the other side,
    so both sides must be covered. The relief (radius relief_r at distance C) cuts the disk rim over
    +-delta around the pin, so after the pin leaves (theta = alpha) the far side is covered from
    theta = delta on: free angle = delta - alpha. The pin leaves tangentially (wheel speed zero), so a
    fraction of a degree is harmless. Also returns delta and the arc's half span zeta (arc meets the rim)."""
    C, r, rr, ra, Ro = g["C"], g["r_lock"], g["relief_r"], g["r_arc"], g["R_o"]
    delta = math.acos(max(-1.0, min(1.0, (C * C + r * r - rr * rr) / (2 * C * r))))
    zeta = math.acos(max(-1.0, min(1.0, (C * C + ra * ra - Ro * Ro) / (2 * C * ra))))
    return math.degrees(delta - g["alpha"]), math.degrees(delta), math.degrees(zeta)


def wheel_inertia(g, t, rho):
    """Wheel as a plain disk of the rim radius (conservative: slots and arcs remove material)."""
    m = rho * math.pi * g["R_o"] ** 2 * t
    return 0.5 * m * (g["R_o"] / 1000) ** 2            # kg m^2 (rho in kg/mm^3)


def loads(g, J, rpm, n_steps=N_STEPS):
    """Peak wheel torque (N mm), pin force (N) and driver torque (N mm) for a total output inertia J (kg m^2)
    at a driver speed rpm (rad/s^2 = acc * omega^2)."""
    om = rpm * 2 * math.pi / 60
    a, C = g["a"], g["C"]
    Tw = F = Td = 0.0
    for th_deg, _, w, acc in motion(g, n_steps):
        th = math.radians(th_deg)
        if abs(th) > g["alpha"] + 1e-12:
            continue                                   # dwell
        T = J * acc * om * om * 1000                   # N mm
        r = math.sqrt(a * a + C * C - 2 * a * C * math.cos(th))
        Tw, F, Td = max(Tw, abs(T)), max(F, abs(T) / r), max(Td, abs(T * w))
    return {"T_wheel": Tw, "F_pin": F, "T_driver": Td, "omega": om}


def _shaft_d(T, n):
    for d in strength.PIN_DIAMETERS + [16, 20, 25, 30]:
        if 16 * T / (math.pi * d ** 3) <= 0.5 * STEEL["sy"] / n:
            return d
    return None


def checks_for(g, t, mat, ld, d_ws):
    """Strength checks for pin diameter g["d_pin"], wheel thickness t: {group, name, util, detail}."""
    n_s = mat["n"]
    F, d = ld["F_pin"], g["d_pin"]
    arm = GAP + t / 2                                   # pin: cantilever from the crank plate
    tau = 4 * F / (math.pi * d * d)
    sig = 32 * F * arm / (math.pi * d ** 3)
    br = F / (d * t)
    return [
        {"group": "Pin", "name": "Pin shear (steel)", "util": tau / (0.5 * STEEL["sy"] / n_s),
         "detail": f"τ = {tau:.1f} MPa"},
        {"group": "Pin", "name": "Pin bending (steel)", "util": sig / (STEEL["sy"] / n_s),
         "detail": f"σ = {sig:.1f} MPa"},
        {"group": "Slot", "name": "Slot flank bearing", "util": br / (mat["sy"] / n_s),
         "detail": f"σ_br = {br:.1f} MPa ({mat['name']})"},
        {"group": "Wheel", "name": "Material under the slots", "util": (d_ws / 2 + WEB) / g["slot_bottom"],
         "detail": f"slot bottom at r = {g['slot_bottom']:.1f} mm, shaft Ø{d_ws:g} + {WEB:g} mm web"},
    ]


def design(n, C, mat, rpm=None, J_load=0.0, rho=DENSITY["PLA"]):
    """Geneva for n slots at centre distance C (mm). With rpm and a load inertia (kg m^2) the pin and the wheel
    thickness are sized; otherwise the pin follows the proportion d ~ a / 5 and t = 5 mm.
    Returns (design, checks). Checks: {group, name, util, detail}."""
    base = geometry(n, C, 4.0)
    loaded = bool(rpm) and J_load > 0
    best = None
    pins = [d for d in strength.PIN_DIAMETERS if d >= base["a"] / PIN_MIN_RATIO] or strength.PIN_DIAMETERS[-1:]
    if not loaded:
        pins = [min(strength.PIN_DIAMETERS, key=lambda d: abs(d - base["a"] / 5))]
    for d in pins:
        g = geometry(n, C, float(d))
        thick = [t for t in strength.THICKNESSES if t >= d] if loaded else [5]   # bearing length >= d
        for t in thick:
            J = J_load + wheel_inertia(g, t, rho)
            ld = loads(g, J, rpm) if loaded else {"T_wheel": 0.0, "F_pin": 0.0, "T_driver": 0.0, "omega": 0.0}
            if loaded:
                d_ws = _shaft_d(ld["T_wheel"], mat["n"]) or 30
                d_ds = _shaft_d(ld["T_driver"], mat["n"]) or 30
            else:
                d_ws = d_ds = d                        # no torque known: shafts like the pin
            c = checks_for(g, t, mat, ld, d_ws)
            worst = max(x["util"] for x in c)
            if best is None or worst < best[0]:        # the first passing one, else the least bad
                best = (worst, (g, t, ld, d_ws, d_ds, c, J))
            if worst <= 1.0:
                break
        if best[0] <= 1.0:
            break
    g, t, ld, d_ws, d_ds, c, J = best[1]
    if not loaded:
        c = [x for x in c if x["group"] == "Wheel"]      # no forces: only the geometric check
    g = dict(g, relief_r=relief_radius(g))           # only the chosen geometry: the sweep takes a moment
    free, delta, zeta = lock_free_angle(g)
    kin = motion(g)
    idx = [k for k in kin if abs(k[0]) <= math.degrees(g["alpha"]) + 1e-9]      # the index
    w_max = max(abs(k[2]) for k in idx)
    acc_max = max(abs(k[3]) for k in idx)
    checks = [
        {"group": "Motion", "name": "Number of slots", "util": 0.0 if N_MIN <= n <= N_MAX else 2.0,
         "detail": f"n = {n} (step {360 / n:g}°, {N_MIN}..{N_MAX})"},
        {"group": "Motion", "name": "Wheel locked during the dwell", "util": max(0.0, free) / FREE_MAX
            if zeta > 0 else 2.0,
         "detail": f"locking disk holds the wheel {max(0.0, free):.2f}° after the pin leaves (arc ±{zeta:.0f}°)"},
        {"group": "Wheel", "name": "Wheel rim clears the driver shaft",
         "util": (d_ds / 2 + WEB) / (g["C"] - g["R_o"]),
         "detail": f"rim {g['C'] - g['R_o']:.1f} mm from the driver axis, shaft Ø{d_ds:g} + {WEB:g} mm"},
    ] + c
    return {"geometry": g, "t": t, "loads": ld, "J": J, "shaft_wheel": d_ws, "shaft_driver": d_ds,
            "index_deg": 2 * math.degrees(g["alpha"]), "dwell_deg": 360 - 2 * math.degrees(g["alpha"]),
            "w_ratio_max": w_max, "acc_max": acc_max, "motion": kin, "loaded": loaded}, checks
