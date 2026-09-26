"""
I2: printed flexure / living hinge: a thin strip (length L, thickness t, width w) bent through an angle theta.

A strip in pure bending takes a circular arc: curvature theta / L, surface strain eps = t theta / (2 L)
(<= the elastic limit S_y / E: PLA 1.6 %, PETG 1.75 %, since a hinge stays bent and flexes again and again,
unlike a snap-fit's moment; half of that if it opens and closes many times), rotational stiffness
k = E I / L with I = w t^3 / 12, moment at the full angle M = k theta, stress E eps.
The hinge's pivot is not fixed: it travels about L/2 along the strip, so a short strip is a better hinge.
Choice: the shortest strip, then the thinnest printable one (0.4..1.2 mm, flat on the bed).  Units: mm, N, MPa.
"""

import math

import strength

STRAIN = {k: strength.MATERIALS[k]["sy"] / strength.MATERIALS[k]["E"] for k in ("PLA", "PETG")}
THICK = (0.4, 0.6, 0.8, 1.0, 1.2)
CYCLIC = 0.5


def strain(t, theta_rad, L):
    return t * theta_rad / (2 * L)


def design(theta_deg, mat_key="PLA", w=10.0, cyclic=False):
    if mat_key not in STRAIN:
        mat_key = "PLA"
    E = strength.MATERIALS[mat_key]["E"]
    th = math.radians(theta_deg)
    eps_all = STRAIN[mat_key] * (CYCLIC if cyclic else 1.0)
    for L10 in range(10, 801, 5):
        L = L10 / 10
        for t in THICK:
            eps = strain(t, th, L)
            if eps > eps_all:
                continue
            k = E * w * t ** 3 / 12 / L
            checks = [{"group": "Strain", "name": "Surface strain at the full angle", "util": eps / eps_all,
                       "detail": f"ε = t θ / (2L) = {100 * eps:.2f} % ≤ {100 * eps_all:.1f} % ({mat_key}{', many cycles' if cyclic else ''})"}]
            return {"theta_deg": theta_deg, "L": L, "t": t, "w": w, "E": E, "eps": eps, "eps_allow": eps_all,
                    "k_Nmm_rad": k, "k_Nmm_deg": k * math.pi / 180, "M": k * th, "sigma": E * eps,
                    "pivot_shift": L / 2, "radius": L / th, "material": mat_key, "cyclic": cyclic}, checks
    return None, [{"group": "Strain", "name": "Surface strain at the full angle", "util": 9.9,
                   "detail": f"no strip up to 80 mm bends {theta_deg:g}° within {100 * eps_all:.1f} %"}]
