"""
H5b: rigid flange coupling (two flanged hubs keyed to the shafts, bolted together).

Proportions (the usual textbook set): hub outside 2 d, hub length 1.5 d, bolt circle 3 d, flange outside 4 d,
flange thickness 0.5 d (d: shaft diameter; sized in torsion when not given).
Design torque K T (service factor K = 1.5 unless given).
Checks:
    key (DIN 6885, shafts.key_design) in the hub, its length within the hub;
    bolts in clearance holes, the torque carried by friction: mu N F_i (D_b / 2) >= K T, F_i = 0.75 A_t S_p
        (bolted.py, class 8.8, mu = 0.15 dry steel faces);  the smallest bolt that does it with N bolts;
    hub in torsion (hollow)  16 K T D / (pi (D^4 - d^4)) <= 0.5 S_y / 2;
    flange shear at the hub  2 K T / (pi D^2 t) <= 0.5 S_y / 2  (steel S235).
Units: mm, N, N*mm, MPa.
"""

import math

import bolted
import chain
import shafts
import strength

K_SERVICE = 1.5
MU = 0.15
STEEL = strength.MATERIALS["steel"]


def proportions(d):
    return {"D_hub": 2 * d, "L_hub": 1.5 * d, "D_bolt": 3 * d, "D_flange": 4 * d, "t": 0.5 * d}


def design(T, d=None, N=None, K=K_SERVICE):
    """Coupling for torque T (N*mm) on shaft d. Returns (design, checks)."""
    Td = K * T
    if not d:
        d = chain.shaft_bore(Td)
        if d is None:
            return None, [{"group": "Shaft", "name": "Shaft torsion", "util": 9.9, "detail": "beyond Ø100"}]
    d = float(d)
    N = N or (4 if d <= 50 else 6)
    g = proportions(d)
    key, k_checks = shafts.key_design(Td, d, 2.0)
    checks = list(k_checks)
    if key:
        checks.append({"group": "Key", "name": "Key fits the hub", "util": key["length"] / g["L_hub"],
                       "detail": f"{key['length']:g} mm in a {g['L_hub']:g} mm hub"})
    allow = 0.5 * STEEL["sy"] / 2
    D, t = g["D_hub"], g["t"]
    tau_hub = 16 * Td * D / (math.pi * (D ** 4 - d ** 4))
    tau_fl = 2 * Td / (math.pi * D * D * t)
    checks += [{"group": "Hub", "name": "Hub in torsion", "util": tau_hub / allow, "detail": f"τ = {tau_hub:.1f} MPa"},
               {"group": "Flange", "name": "Flange shear at the hub", "util": tau_fl / allow, "detail": f"τ = {tau_fl:.1f} MPa"}]
    bolt = None
    for size in bolted.SIZES:
        Fi = bolted.PRELOAD * size[2] * (580.0 if size[0] < 16 else 600.0)
        T_fric = MU * N * Fi * g["D_bolt"] / 2
        if T_fric >= Td and size[0] <= 0.3 * g["D_bolt"]:
            bolt = {"d": size[0], "pitch": size[1], "nut_h": size[3], "af": size[4], "Fi": Fi, "T_fric": T_fric,
                    "L": bolted.bolt_length(2 * t, size[0], size[3])}
            break
    checks.append({"group": "Bolts", "name": f"{N} bolts carry the torque by friction", "util": Td / bolt["T_fric"] if bolt else 9.9,
                   "detail": (f"M{bolt['d']} class 8.8: μ N F_i r = {bolt['T_fric'] / 1000:.0f} N·m ≥ {Td / 1000:.0f} N·m"
                              if bolt else "no bolt up to M36 fits the bolt circle")})
    if not key or not bolt:
        return None, checks
    return dict(g, d=d, N=N, K=K, T=T, Td=Td, key=key, bolt=bolt, tau_hub=tau_hub, tau_flange=tau_fl), checks
