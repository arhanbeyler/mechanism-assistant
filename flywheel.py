"""
H5a: flywheel (Shigley 16-12): the energy fluctuation dE of one cycle kept within a coefficient of speed
fluctuation C_s = (w_max - w_min) / w  at the mean speed w.

    I = dE / (C_s w^2)       (from dE = I/2 (w_max^2 - w_min^2) = I C_s w^2)
Solid steel disk (rho 7850 kg/m^3): I = pi rho t r^4 / 2.  Proportion t = 0.1 D (a compact disk), or with a
maximum diameter the disk becomes thicker (t <= D/2).
Stress in a rotating solid disk: sigma_max = (3 + nu) / 8 rho w^2 r^2 at the centre (Shigley 3-55)
    <= S_y / 2 (S235);  rim speed reported.
Energy per cycle from a punch or press: dE = E_work (1 - fraction of the cycle it lasts), the motor
supplying the average (Shigley's punch press).  Units outside: mm, kg, J, rev/min, MPa.
"""

import math

import strength

RHO = 7850.0                 # kg/m^3
NU = 0.3
T_RATIO = 0.1
N_STRESS = 2.0
CS_TYPICAL = {"press": 0.10, "crusher": 0.20, "machine_tool": 0.03, "pump": 0.04, "engine": 0.03,
              "generator": 0.003}


def inertia(dE, rpm, Cs):
    w = 2 * math.pi * rpm / 60
    return dE / (Cs * w * w)


def disk_for(I, D_max=None):
    """Solid disk (D, t in mm) with inertia I (kg m^2): t = 0.1 D, or the given max diameter."""
    k = math.pi * RHO / 2
    D = (I / (k * T_RATIO / 16)) ** 0.2                  # I = k (0.1 D) (D/2)^4, metres
    if D_max and D * 1000 > D_max:
        D = D_max / 1000
    t = I / (k * (D / 2) ** 4)
    return D * 1000, t * 1000


def analyse(dE, rpm, Cs, D_max=None, bore=None):
    I = inertia(dE, rpm, Cs)
    D, t = disk_for(I, D_max)
    w = 2 * math.pi * rpm / 60
    r = D / 2000
    sigma = (3 + NU) / 8 * RHO * w * w * r * r / 1e6
    mass = RHO * math.pi * r * r * t / 1000
    bore = bore or max(10.0, round(D / 10 / 5) * 5)
    Sy = strength.MATERIALS["steel"]["sy"]
    checks = [
        {"group": "Strength", "name": "Rotating disk stress (centre)", "util": N_STRESS * sigma / Sy,
         "detail": f"σ = (3+ν)/8 ρω²r² = {sigma:.1f} MPa, S_y/{N_STRESS:g} = {Sy / N_STRESS:.0f} MPa (steel)"},
        {"group": "Geometry", "name": "Proportions", "util": max(t / (D / 2), 5.0 / t),
         "detail": f"Ø{D:.0f} × {t:.1f} mm (t between 5 mm and D/2)"},
    ]
    info = {"dE": dE, "rpm": rpm, "Cs": Cs, "I": I, "D": D, "t": t, "mass": mass, "sigma": sigma,
            "rim_speed": w * r, "rpm_max": rpm * (1 + Cs / 2), "rpm_min": rpm * (1 - Cs / 2),
            "E_stored": 0.5 * I * w * w, "bore": bore}
    return info, checks


def press_energy(E_work, fraction):
    """Energy the flywheel must give up while the tool works for `fraction` of the cycle."""
    return E_work * (1 - fraction)
