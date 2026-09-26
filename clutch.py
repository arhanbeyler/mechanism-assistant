"""
H5b: dry disc (plate) clutch / brake, uniform-wear model (Shigley 16-5).

Uniform wear: p r = p_a r_i;  actuating force F = 2 pi p_a r_i (r_o - r_i);
torque with N friction faces  T = N pi f p_a r_i (r_o^2 - r_i^2)  = N f F (r_o + r_i) / 2;
for a given outside radius the torque is largest at r_i = r_o / sqrt(3).
Design torque = service factor K x T (default 1.3, reported).  Lining (Shigley Table 16-3, lower ends taken):
molded f 0.35 / p_max 1.0 MPa, woven 0.30 / 0.5, sintered metal 0.30 / 1.0.
With a maximum outside diameter the faces are added (single plate = 2 faces, then 4, 6).
Units: mm, N, N*mm, MPa.
"""

import math

LININGS = {"molded": {"name": "molded (resin)", "f": 0.35, "p": 1.0},
           "woven": {"name": "woven", "f": 0.30, "p": 0.5},
           "sintered": {"name": "sintered metal", "f": 0.30, "p": 1.0}}
K_SERVICE = 1.3
FACES = (2, 4, 6)


def torque(N, f, pa, ri, ro):
    return N * math.pi * f * pa * ri * (ro * ro - ri * ri)


def force(pa, ri, ro):
    return 2 * math.pi * pa * ri * (ro - ri)


def design(T, lining="molded", od_max=None, K=K_SERVICE):
    """Smallest clutch for torque T (N*mm). Returns (design, checks)."""
    L = LININGS[lining]
    f, pa = L["f"], L["p"]
    Td = K * T
    last = None
    for N in FACES:
        ro = (Td / (N * math.pi * f * pa * 2 / (3 * math.sqrt(3)))) ** (1 / 3)
        ro = math.ceil(ro)
        ri = round(ro / math.sqrt(3), 1)
        Tc = torque(N, f, pa, ri, ro)
        F = force(pa, ri, ro)
        checks = [{"group": "Torque", "name": f"Torque capacity (uniform wear, {N} faces)", "util": Td / Tc,
                   "detail": f"{Tc / 1000:.1f} N·m for {T / 1000:.1f} × {K:g} = {Td / 1000:.1f} N·m"}]
        if od_max:
            checks.append({"group": "Geometry", "name": "Outside diameter", "util": 2 * ro / od_max,
                           "detail": f"Ø{2 * ro:.0f} ≤ Ø{od_max:g} mm"})
        last = checks
        if max(c["util"] for c in checks) <= 1.0:
            return {"ro": ro, "ri": ri, "N": N, "f": f, "pa": pa, "F": F, "T_cap": Tc, "T": T, "K": K,
                    "lining": L["name"], "p_min": pa * ri / ro}, checks
    return None, last
