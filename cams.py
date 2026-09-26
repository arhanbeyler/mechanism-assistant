"""
C11a: Disk cam + translating roller follower (zero eccentricity). Norton, Chapter 8.

Motion: rise (h, beta1) - dwell at top (beta2) - return (beta3) - dwell at bottom (the rest).
Motion laws (theta: angle within the segment, beta: segment angle; derivatives per radian):
  cycloidal:  s = h (x - sin(2 pi x)/(2 pi)),  v = h/beta (1 - cos 2 pi x),  a = 2 pi h/beta^2 sin 2 pi x
  harmonic:   s = h/2 (1 - cos pi x),          v = pi h/(2 beta) sin pi x,     a = pi^2 h/(2 beta^2) cos pi x
  3-4-5:      s = h (10x^3 - 15x^4 + 6x^5),     v = h/beta (30x^2 - 60x^3 + 30x^4), a = h/beta^2 (60x - 180x^2 + 120x^3)
  (x = theta / beta)
Checks (Rp = Rb + Rf, pitch curve radius):
  Pressure angle   phi = atan(v / (Rp + s))            |phi| <= 30 degrees
  Pitch curvature  rho = ((Rp+s)^2 + v^2)^1.5 / ((Rp+s)^2 + 2 v^2 - a (Rp+s))
  Cam surface      rho_cam = rho - Rf > 0 (no undercut); in the convex (positive) part rho >= 2 Rf
Choice: the SMALLEST base circle radius Rb that passes the checks.
"""

import math

MAX_PRESSURE_DEG = 30.0


def law(name, x, h, beta):
    """(s, v, a) — x in [0, 1]."""
    if name == "harmonic":
        return (h / 2 * (1 - math.cos(math.pi * x)), math.pi * h / (2 * beta) * math.sin(math.pi * x),
                math.pi ** 2 * h / (2 * beta ** 2) * math.cos(math.pi * x))
    if name == "poly345":
        return (h * (10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5), h / beta * (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4),
                h / beta ** 2 * (60 * x - 180 * x ** 2 + 120 * x ** 3))
    return (h * (x - math.sin(2 * math.pi * x) / (2 * math.pi)), h / beta * (1 - math.cos(2 * math.pi * x)),
            2 * math.pi * h / beta ** 2 * math.sin(2 * math.pi * x))            # cycloidal (default)


def svaj(h, rise_deg, dwell_top_deg, return_deg, motion="cycloidal", n=720):
    """Over a full turn: [(theta_deg, s, v, a)]; v and a per radian."""
    b1, b2, b3 = (math.radians(x) for x in (rise_deg, dwell_top_deg, return_deg))
    out = []
    for i in range(n):
        th = 2 * math.pi * i / n
        if th < b1:
            s, v, a = law(motion, th / b1, h, b1)
        elif th < b1 + b2:
            s, v, a = h, 0.0, 0.0
        elif th < b1 + b2 + b3:
            s, v, a = law(motion, (th - b1 - b2) / b3, h, b3)
            s, v, a = h - s, -v, -a
        else:
            s, v, a = 0.0, 0.0, 0.0
        out.append((math.degrees(th), s, v, a))
    return out


def check(Rb, Rf, table):
    Rp = Rb + Rf
    phi_max, rho_min_pos, rho_min_cam = 0.0, float("inf"), float("inf")
    for _, s, v, a in table:
        r = Rp + s
        phi_max = max(phi_max, abs(math.degrees(math.atan2(v, r))))
        den = r * r + 2 * v * v - a * r
        rho = (r * r + v * v) ** 1.5 / den if abs(den) > 1e-12 else float("inf")
        if rho > 0:                                        # convex: cam surface = rho - Rf
            rho_min_pos = min(rho_min_pos, rho)
            rho_min_cam = min(rho_min_cam, rho - Rf)
    checks = [
        {"group": "Pressure angle", "name": "Largest pressure angle", "util": phi_max / MAX_PRESSURE_DEG,
         "detail": f"φ = {phi_max:.1f}° (at most {MAX_PRESSURE_DEG:g}°)"},
        {"group": "Curvature", "name": "Undercut on the cam surface", "util": 0.0 if rho_min_cam > 0 else 2.0,
         "detail": f"ρ_cam,min = {rho_min_cam:.1f} mm (must be > 0)"},
        {"group": "Curvature", "name": "Pitch curvature / roller", "util": (2 * Rf) / rho_min_pos,
         "detail": f"ρ_min = {rho_min_pos:.1f} mm ≥ 2·R_roller = {2 * Rf:.1f} mm"},
    ]
    return checks


def design(h, rise_deg, dwell_top_deg, return_deg, motion="cycloidal", Rf=None):
    """Smallest base circle. Returns (design, checks) or (None, checks of the best candidate)."""
    if rise_deg + dwell_top_deg + return_deg > 360:
        return None, [{"group": "Angles", "name": "Sum of the angles", "util": 99.0,
                       "detail": f"{rise_deg + dwell_top_deg + return_deg:g}° > 360°"}]
    Rf = Rf or max(3.0, round(h / 3))                     # roller radius (if not given)
    table = svaj(h, rise_deg, dwell_top_deg, return_deg, motion)
    best_any = None
    for Rb10 in range(int(Rf * 10) + 10, 3000, 5):       # Rb: 0.5 mm steps
        Rb = Rb10 / 10
        checks = check(Rb, Rf, table)
        worst = max(c["util"] for c in checks)
        if worst <= 1.0:
            return {"Rb": Rb, "Rf": Rf, "h": h, "rise": rise_deg, "dwell_top": dwell_top_deg,
                    "return": return_deg, "dwell_bottom": 360 - rise_deg - dwell_top_deg - return_deg,
                    "motion": motion, "bore": max(4.0, round(Rb / 3))}, checks
        if best_any is None or worst < best_any[1]:
            best_any = (checks, worst)
    return None, best_any[0]


def profile(design_, n=360):
    """Cam surface points (mm, in cam coordinates). The follower is along +y; as the cam turns clockwise by
    theta the contact point is at angle -theta on the cam. Surface = the pitch curve offset inwards by the
    roller radius."""
    Rp = design_["Rb"] + design_["Rf"]
    table = svaj(design_["h"], design_["rise"], design_["dwell_top"], design_["return"], design_["motion"], n)
    pts = []
    for th_deg, s, v, _ in table:
        th = math.radians(th_deg)
        r = Rp + s
        # pitch point (cam coordinates): angle pi/2 - th (follower on +y, the cam turns)
        ang = math.pi / 2 - th
        px, py = r * math.cos(ang), r * math.sin(ang)
        # tangent of the pitch curve (d/dth): dr = v, dang = -1
        tx = v * math.cos(ang) + r * math.sin(ang)
        ty = v * math.sin(ang) - r * math.cos(ang)
        L = math.hypot(tx, ty)
        nx, ny = ty / L, -tx / L                          # inward normal (towards the centre)
        if nx * px + ny * py > 0:
            nx, ny = -nx, -ny
        pts.append((px + design_["Rf"] * nx, py + design_["Rf"] * ny))
    return pts
