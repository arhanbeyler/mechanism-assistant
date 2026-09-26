"""
Strength module: sizing of the four-bar links and pins.

Units: N, mm, MPa (= N/mm^2), N*mm.

Load model (quasi-static, inertia neglected):
    F: load at the rocker tip (B), perpendicular to the rocker.
    The coupler is a two-force member; moment balance of the rocker about O4:
        F * c = F_AB * c * sin(mu)   ->   F_AB = F / sin(mu)
    The rocker's O4 pin: the component of F_AB perpendicular to the rocker balances F, the remaining
    axial component F_AB*|cos mu| <= F_AB. Every pin carries at most F_AB.

Checks (n: safety factor, utilisation must be <= 1), for every link:
    Buckling (pinned-pinned, weak axis):  F_AB * n <= pi^2 E I_min / L^2
    Net section (axial):   F_AB / ((w-d) t)                   <= sy / n
    Crank, in addition:    M_crank / S_net + F_AB / A_net     <= sy / n
    Rocker, in addition:   F c / S_net + F_AB / A_net         <= sy / n
    Pin bearing:           F_AB / (d t)                        <= sy / n
    Stiffness (deflection): crank / rocker: P L^3/(3 E I) + P L/(kappa G A) + P L^2/(C E t r^2)  <= L / 200
                           (tip-loaded cantilever: bending + Timoshenko shear + hub rotation, C = HUB_C)
                           coupler / ground: F_AB L / (E A)    <= L / 200   (axial stretch)
Pin (same diameter at every free joint):
    Shear (single):        4 F_AB / (pi d^2)                   <= 0.5 sy / n  (Tresca)

Stress concentration (Kt), only for BRITTLE materials (PLA, PETG) and only on the AXIAL stress: the
hole sits on the neutral axis of in-plane bending, so it barely raises the peak bending stress. Not used
for ductile metals under static load, where local yielding spreads the load (it matters in fatigue).
Roark, finite-width plate with a hole, tension, on the net section:
    Kt = 3.00 - 3.13 (d/w) + 3.66 (d/w)^2 - 1.53 (d/w)^3

Sizing: one common pin diameter; width and thickness chosen separately for every link.
Goal: the smallest total volume (sum w * t * L).
"""

import math

# Typical values (3D print at 100 % infill, layers assumed in the plane of the link).
# To be verified: the strength of printed plastics depends strongly on print direction and infill.
MATERIALS = {
    "PLA":      {"name": "PLA (3D print)",      "sy": 40.0,  "E": 2500.0,   "nu": 0.36, "n": 3.0, "brittle": True},
    "PETG":     {"name": "PETG (3D print)",     "sy": 35.0,  "E": 2000.0,   "nu": 0.38, "n": 3.0, "brittle": True},
    "aluminum": {"name": "Aluminium 6061-T6",   "sy": 276.0, "E": 69000.0,  "nu": 0.33, "n": 2.0, "brittle": False},
    "steel":    {"name": "Steel S235",          "sy": 235.0, "E": 200000.0, "nu": 0.30, "n": 2.0, "brittle": False},
}

DEFLECTION_RATIO = 200     # allowed deflection: L / 200

# Hub compliance (B7g): a link held by a shaft / pin rotates around the small hole.
# Rotational stiffness k = C E t r^2  ->  tip deflection share  delta_hub = P L^2 / (C E t r^2).
# C was calibrated with a local FEA sweep (tools/hub_calibration.py: 21 geometries,
# L = 40-100, w = 9-29, d = 3-6 mm). For d/w <= 0.3, C = 5.96-9.17 and independent of L
# (L^2 scaling verified). The LOWER bound is used to stay on the safe side.
HUB_C = 6.0


def kt_hole_tension(d, w):
    """Roark: central hole in a finite-width plate, tension, Kt on the net section (0 <= d/w < 1)."""
    r = d / w
    return 3.00 - 3.13 * r + 3.66 * r ** 2 - 1.53 * r ** 3

LINK_KEYS = {"Crank": "a", "Coupler": "b", "Rocker": "c", "Ground": "d"}

# B7h: joint types. O2 (drive shaft) and O4 (output shaft) are SHAFTS: they carry torque, the link is
# held by the shaft (hub). A and B are free PINS: force only. (start, end) point of every link:
LINK_ENDS = {"Crank": ("O2", "A"), "Coupler": ("A", "B"), "Rocker": ("O4", "B"), "Ground": ("O2", "O4")}
SHAFT_POINTS = {"O2", "O4"}


def hole_at(point, d_pin, d_shaft):
    return d_shaft if point in SHAFT_POINTS else d_pin


def link_holes(name, d_pin, d_shaft):
    """(start, end) hole diameters."""
    return tuple(hole_at(p, d_pin, d_shaft) for p in LINK_ENDS[name])


# Section used when no load is given
DEFAULT_SECTION = {"hole": 4.0, "shaft": 4.0,
                   "links": {name: {"width": 10.0, "thickness": 4.0} for name in LINK_KEYS}}

PIN_DIAMETERS = [3, 4, 5, 6, 8, 10, 12]
THICKNESSES = [3, 4, 5, 6, 8, 10, 12, 15, 20]
MAX_WIDTH = 60
W_MIN = 6            # thinnest web (for printing)

# B8a: local boss. Every hole sits in its own circular boss: D = max(w, 2.5 d).
# Link = two end bosses + a web of width w between them. The edge-distance rule now applies only to
# the boss; the web can be as thin as the checks allow.
BOSS_RATIO = 2.5


def boss_diameters(name, w, d_pin, d_shaft):
    """(start, end) boss diameters."""
    return tuple(max(w, BOSS_RATIO * h) for h in link_holes(name, d_pin, d_shaft))


def _cross2(u, v):
    return u[0] * v[1] - u[1] * v[0]


def analyze_forces(mech, frames, F):
    """Forces over the cycle. Returns the largest coupler force, crank moment and rocker moment."""
    o2 = mech["o2"]
    F_ab_max = M_crank_max = 0.0
    for fr in frames:
        s = math.sin(math.radians(fr["mu"]))
        F_ab = F / max(abs(s), 1e-6)
        A, B = fr["A"], fr["B"]
        r = (A[0] - o2[0], A[1] - o2[1])                    # crank vector
        ab = (B[0] - A[0], B[1] - A[1])
        u = (ab[0] / math.hypot(*ab), ab[1] / math.hypot(*ab))  # coupler force direction
        M = F_ab * abs(_cross2(r, u))                          # moment on the crank = drive torque
        F_ab_max = max(F_ab_max, F_ab)
        M_crank_max = max(M_crank_max, M)
    return {"F": F, "F_ab_max": F_ab_max, "M_crank_max": M_crank_max,
            "M_rocker": F * mech["c"]}


def check_link(name, forces, length, mat, w, t, d, d_shaft=None):
    """Checks of one link (utilisations, <= 1 passes).
    d: pin diameter (A, B); d_shaft: shaft diameter (O2, O4), d if not given."""
    n, sy, E = mat["n"], mat["sy"], mat["E"]
    allow = sy / n
    F_ab = forces["F_ab_max"]
    d_shaft = d if d_shaft is None else d_shaft
    holes = link_holes(name, d, d_shaft)
    bosses = boss_diameters(name, w, d, d_shaft)
    d_small, d_root, D_root = min(holes), holes[0], bosses[0]   # root = shaft end (crank, rocker)
    # Hub section (through the hole): the end with the smallest net area
    i_net = min(range(2), key=lambda i: (bosses[i] - holes[i]) * t)
    d_net, D_net = holes[i_net], bosses[i_net]
    A_net = (D_net - d_net) * t
    S_hub = t * (D_root ** 3 - d_root ** 3) / (6 * D_root)       # bending at the root (shaft boss)
    # Web section (just outside the boss): no hole
    S_web, A_web = t * w ** 2 / 6.0, w * t
    I_min = min(w * t ** 3, t * w ** 3) / 12.0

    P_cr = math.pi ** 2 * E * I_min / length ** 2
    checks = [{"group": "Buckling", "name": f"{name} buckling", "util": F_ab * n / P_cr,
               "detail": f"F_AB = {F_ab:.0f} N, P_cr = {P_cr:.0f} N"}]
    moment = {"Crank": forces["M_crank_max"], "Rocker": forces["M_rocker"],
              "Coupler": forces.get("M_coupler", 0.0)}.get(name, 0.0)   # coupler: only with a coupler-point load
    kt = kt_hole_tension(d_net, D_net) if mat.get("brittle") else 1.0
    s_hub = moment / S_hub + kt * F_ab / A_net
    s_web = moment / S_web + F_ab / A_web            # conservative: moment at the boss edge ~ M
    sigma, where = (s_hub, "boss") if s_hub >= s_web else (s_web, "web")
    label = f"{name} bending + axial" if moment else f"{name} net section"
    checks.append({"group": "Stress", "name": label, "util": sigma / allow,
                   "detail": f"σ = {sigma:.1f} MPa ({where})" + (f", Kt = {kt:.2f}" if kt > 1 and where == "boss" else "")})

    # Stiffness: crank and rocker as tip-loaded cantilevers (in-plane), the others axial stretch
    limit = length / DEFLECTION_RATIO
    if moment and name == "Coupler":
        # F2: the coupler point P overhangs B; A-B is a simply supported span, B-P a cantilever
        Lb, Lo = forces["coupler_span"], forces["coupler_overhang"]
        P_tip = moment / Lo
        I_in = t * w ** 3 / 12.0
        G = E / (2 * (1 + mat.get("nu", 0.35)))
        delta = P_tip * Lo ** 2 * (Lb + Lo) / (3 * E * I_in) + P_tip * Lo / ((5.0 / 6.0) * G * w * t)
        limit = (Lb + Lo) / DEFLECTION_RATIO
        kind = "deflection at the coupler point (bending + shear)"
    elif moment:
        P_tip = moment / length                      # perpendicular force at the tip
        I_in = t * w ** 3 / 12.0
        G = E / (2 * (1 + mat.get("nu", 0.35)))
        d_bend = P_tip * length ** 3 / (3 * E * I_in)                 # Euler-Bernoulli
        d_shear = P_tip * length / ((5.0 / 6.0) * G * w * t)         # Timoshenko shear
        d_hub = P_tip * length ** 2 / (HUB_C * E * t * (d_root / 2) ** 2)  # hub (shaft) rotation
        delta = d_bend + d_shear + d_hub
        kind = "deflection (bending + shear + hub)"
    else:
        delta = F_ab * length / (E * w * t)
        kind = "axial stretch"
    checks.append({"group": "Stiffness", "name": f"{name} {kind}", "util": delta / limit,
                   "detail": f"δ = {delta:.2f} mm (limit L/{DEFLECTION_RATIO} = {limit:.2f} mm)"})
    s_br = F_ab / (d_small * t)                         # bearing stress is largest at the smallest hole
    checks.append({"group": "Pin", "name": f"{name} pin bearing", "util": s_br / allow,
                   "detail": f"σ_br = {s_br:.1f} MPa"})
    return checks


def check_pin(forces, mat, d):
    tau = 4 * forces["F_ab_max"] / (math.pi * d ** 2)
    return {"group": "Pin", "name": "Pin shear", "util": tau / (0.5 * mat["sy"] / mat["n"]),
            "detail": f"τ = {tau:.1f} MPa"}


SHAFT_MATERIAL = "steel"   # torque-carrying shafts are steel (they pass through the plastic link's boss)


def check_shaft(forces, mat, d):
    """Shaft (O2, O4), STEEL: torsion 16 T / (pi d^3) + direct shear 4 F / (pi d^2) (conservative sum).
    The mat argument (link material) is not used; the shaft is always SHAFT_MATERIAL."""
    sm = MATERIALS[SHAFT_MATERIAL]
    T = max(forces["M_crank_max"], forces["M_rocker"])
    tau = 16 * T / (math.pi * d ** 3) + 4 * forces["F_ab_max"] / (math.pi * d ** 2)
    return {"group": "Pin", "name": "Shaft (steel) torsion + shear", "util": tau / (0.5 * sm["sy"] / sm["n"]),
            "detail": f"τ = {tau:.1f} MPa (T = {T / 1000:.2f} N·m)"}


def _lightest_link(name, forces, length, mat, d, d_shaft=None):
    """For the given pin / shaft diameters, this link's lightest passing section (smallest w * t)."""
    d_shaft = d if d_shaft is None else d_shaft
    best = None
    for t in THICKNESSES:
        for w in range(W_MIN, MAX_WIDTH + 1):          # the boss takes care of the edge distance
            checks = check_link(name, forces, length, mat, w, t, d, d_shaft)
            if max(c["util"] for c in checks) <= 1.0:
                if best is None or w * t < best[0] * best[1]:
                    best = (w, t, checks)
                break          # every utilisation falls with w: a wider one is only heavier
    return best


def size_links(forces, lengths, mat):
    """
    Pin diameter (A, B) + shaft diameter (O2, O4) + lightest section per link; smallest total volume.
    Returns (section, checks). If nothing fits: (None, the hardest checks).
    """
    best = None
    for d in PIN_DIAMETERS:
        pin = check_pin(forces, mat, d)
        if pin["util"] > 1.0:
            continue
        for ds in (x for x in PIN_DIAMETERS if x >= d):
            shaft = check_shaft(forces, mat, ds)
            if shaft["util"] > 1.0:
                continue
            links, checks, volume = {}, [pin, shaft], 0.0
            for name, key in LINK_KEYS.items():
                found = _lightest_link(name, forces, lengths[key], mat, d, ds)
                if found is None:
                    break
                w, t, c = found
                b0, b1 = boss_diameters(name, w, d, ds)
                links[name] = {"width": float(w), "thickness": float(t),
                               "boss0": float(b0), "boss1": float(b1)}
                checks += c
                volume += t * (w * lengths[key] + math.pi * (b0 ** 2 + b1 ** 2) / 8)  # web + boss halves
            else:
                if best is None or volume < best[2] - 1e-9:
                    best = ({"hole": float(d), "shaft": float(ds), "links": links}, checks, volume)
    if best:
        return best[0], best[1]
    # Nothing passed: the checks at the largest section show what is not enough
    d, w, t = PIN_DIAMETERS[-1], MAX_WIDTH, THICKNESSES[-1]
    fail = [check_pin(forces, mat, d), check_shaft(forces, mat, d)]
    for name, key in LINK_KEYS.items():
        fail += check_link(name, forces, lengths[key], mat, w, t, d, d)
    return None, fail


def section_with_bosses(section):
    """Adds the boss diameters to an old / default section (for Fusion and FEA)."""
    d, ds = section["hole"], section.get("shaft", section["hole"])
    links = {}
    for name, s in section["links"].items():
        b0, b1 = boss_diameters(name, s["width"], d, ds)
        links[name] = dict(s, boss0=s.get("boss0", b0), boss1=s.get("boss1", b1))
    return dict(section, shaft=ds, links=links)
