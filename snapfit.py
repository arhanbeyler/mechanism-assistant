"""
I1: cantilever snap-fit for 3D printing (constant rectangular section; the Bayer / Covestro snap-fit method).

Undercut (hook height) y, beam length l, thickness h, width w; E the material's modulus.
    strain at the root when the hook passes:   eps = 1.5 y h / l^2      (<- y = 0.67 eps l^2 / h)
    deflection force:                          P = w h^2 E eps / (6 l)   (= 3 E I y / l^3, beam theory)
    mating force over a ramp of angle alpha:   W = P (mu + tan a) / (1 - mu tan a);  no pull-out (locked)
                                               when mu tan a' >= 1 for the retaining face a'
Allowed strain for one assembly (printed parts, flat on the bed so the beam runs along the layers):
PLA 2 %, PETG 3 %; repeated snapping 60 % of it.  Friction mu 0.35 (printed plastic on itself).
Choice: the shortest beam (then the thinnest printable wall) that stays within the strain and, if asked,
holds the pull-out force.  Units: mm, N, MPa.
"""

import math

import strength

STRAIN = {"PLA": 0.02, "PETG": 0.03}
MU = 0.35
THICK = (1.2, 1.5, 2.0, 2.5, 3.0, 4.0)          # at least 3 perimeters of a 0.4 mm nozzle
REPEAT = 0.6


def strain(y, h, l):
    return 1.5 * y * h / (l * l)


def deflection_force(w, h, l, E, eps):
    return w * h * h * E * eps / (6 * l)


def mating_force(P, alpha_deg, mu=MU):
    t = math.tan(math.radians(alpha_deg))
    if mu * t >= 1:
        return math.inf
    return P * (mu + t) / (1 - mu * t)


def design(y, mat_key="PLA", F_hold=None, w=None, alpha_in=30.0, alpha_out=90.0, repeated=False):
    """Shortest cantilever snap for undercut y. Returns (design, checks)."""
    if mat_key not in STRAIN:
        mat_key = "PLA"
    E = strength.MATERIALS[mat_key]["E"]
    eps_all = STRAIN[mat_key] * (REPEAT if repeated else 1.0)
    width = w or max(6.0, 4 * y)
    for l in range(4, 81):
        for h in THICK:
            eps = strain(y, h, l)
            if eps > eps_all:
                continue
            P = deflection_force(width, h, l, E, eps)
            W_in = mating_force(P, alpha_in)
            W_out = mating_force(P, alpha_out)
            if F_hold and W_out < F_hold:
                continue
            checks = [
                {"group": "Strain", "name": "Root strain while the hook passes", "util": eps / eps_all,
                 "detail": f"ε = 1.5 y h / l² = {100 * eps:.2f} % ≤ {100 * eps_all:.1f} % ({mat_key}{', repeated use' if repeated else ''})"},
                {"group": "Geometry", "name": "Printable wall", "util": THICK[0] / h, "detail": f"h = {h:g} mm (≥ 3 perimeters)"},
            ]
            if F_hold:
                checks.append({"group": "Holding", "name": "Pull-out force", "util": F_hold / W_out if W_out < math.inf else 0.0,
                               "detail": ("locked (the retaining face does not let it slide)" if W_out == math.inf
                                          else f"{W_out:.1f} N ≥ {F_hold:g} N")})
            locked = W_out == math.inf
            return {"y": y, "l": float(l), "h": h, "w": width, "E": E, "eps": eps, "eps_allow": eps_all, "P": P,
                    "W_in": W_in, "W_out": None if locked else W_out, "locked": locked,     # JSON has no infinity
                    "alpha_in": alpha_in, "alpha_out": alpha_out, "material": mat_key, "repeated": repeated}, checks
    return None, [{"group": "Strain", "name": "Root strain while the hook passes", "util": 9.9,
                   "detail": f"no beam up to 80 mm keeps {y:g} mm of undercut within {100 * eps_all:.1f} %"}]
