"""
G2: planetary gear set (sun input, carrier output, ring fixed), 20 deg full-depth spur gears.

Kinematics (Willis):  ring fixed  ->  i = n_sun / n_carrier = 1 + Z_r / Z_s,  Z_r = Z_s + 2 Z_p (one module,
standard centres); a planet turns relative to the carrier at -(Z_r / Z_p) times the carrier.
Assembly: (Z_s + Z_r) / n must be a whole number (equally spaced planets);
neighbours: (Z_s + Z_p) sin(pi / n) >= Z_p + 2 + 0.5 (tips at least half a module apart).
Loads: the input torque is shared by n planets with a load-sharing factor K_g (3: 1.10, 4: 1.25, 5: 1.35,
an assumption, reported);  F_t = K_g 2 T_s / (d_s n) on every sun and ring mesh.
Checks: Lewis bending of the sun, of the planet (an idler: fully reversed bending, allowed stress x 0.7)
and of the ring (internal teeth taken as a rack, Y = 0.485, on the safe side), Barth K_v at the mesh speed
relative to the carrier;  contact ratio sun-planet (external) and planet-ring (internal:
eps = [sqrt(ra_p^2 - rb_p^2) - sqrt(ra_r^2 - rb_r^2) + a sin phi] / (pi m cos phi), ra_r = r_r - m);
the planet pin (steel, held by one carrier plate: a cantilever loaded by 2 F_t at mid-face);
sun shaft and carrier output shaft in torsion; hubs.
Choice: the smallest ring (outer diameter^2 x face width) that passes.  Units: mm, N, N*mm, MPa.
"""

import math

import chain
import gears
import strength

N_PLANETS = (3, 4, 5)
K_GAMMA = {3: 1.10, 4: 1.25, 5: 1.35}
IDLER = 0.7
Y_RING = 0.485
Z_RANGE = range(17, 81)
NEIGHBOUR_GAP = 0.5            # modules between planet tips
PINS = [3, 4, 5, 6, 8, 10, 12, 16, 20, 25, 30]
RIM = 3.0                      # ring rim below the tooth roots, in modules (at least 5 mm)
STEEL = strength.MATERIALS["steel"]


def ratio(Zs, Zp):
    return 1 + (Zs + 2 * Zp) / Zs


def contact_ratio_internal(m, zp, zr, phi=gears.PHI):
    rp, rr = m * zp / 2, m * zr / 2
    a = rr - rp
    return (math.sqrt((rp + m) ** 2 - (rp * math.cos(phi)) ** 2) - math.sqrt((rr - m) ** 2 - (rr * math.cos(phi)) ** 2)
            + a * math.sin(phi)) / (math.pi * m * math.cos(phi))


def neighbour_util(Zs, Zp, n):
    return (Zp + 2 + NEIGHBOUR_GAP) / ((Zs + Zp) * math.sin(math.pi / n))


def pin_diameter(F_pin, lever):
    """Smallest steel planet pin for a cantilever moment F_pin * lever (bending) and shear."""
    allow = STEEL["sy"] / STEEL["n"]
    for d in PINS:
        sb = 32 * F_pin * lever / (math.pi * d ** 3)
        tau = 4 * F_pin / (math.pi * d * d)
        if sb <= allow and tau <= 0.5 * allow:
            return float(d), max(sb, 2 * tau) / allow
    return None, 9.9


def evaluate(m, Zs, Zp, n, b, T_in, mat, rpm=None):
    Zr = Zs + 2 * Zp
    i = ratio(Zs, Zp)
    ds = m * Zs
    Kg = K_GAMMA[n]
    Ft = Kg * 2 * T_in / (ds * n)
    n_c = (rpm or 0) / i
    V = math.pi * ds * ((rpm or 0) - n_c) / 60000.0             # mesh speed relative to the carrier
    Kv = (3.05 + V) / 3.05
    allow = mat["sy"] / mat["n"]
    checks = []
    for name, Y, lim in (("Sun", gears.lewis_y(Zs), allow), ("Planet (idler, x0.7)", gears.lewis_y(Zp), IDLER * allow),
                         ("Ring (as a rack)", Y_RING, allow)):
        s = Kv * Ft / (b * m * Y)
        checks.append({"group": "Bending", "name": f"{name} tooth bending (Lewis)", "util": s / lim,
                       "detail": f"σ = {s:.1f} MPa of {lim:.0f}, Y = {Y:.3f}"})
    e1, e2 = gears.contact_ratio(m, Zs, Zp), contact_ratio_internal(m, Zp, Zr)
    checks += [{"group": "Contact ratio", "name": "Sun-planet contact ratio", "util": 1.2 / e1, "detail": f"ε = {e1:.2f}"},
               {"group": "Contact ratio", "name": "Planet-ring contact ratio", "util": 1.2 / e2, "detail": f"ε = {e2:.2f}"},
               {"group": "Geometry", "name": f"{n} planets clear each other", "util": neighbour_util(Zs, Zp, n),
                "detail": f"tip gap {((Zs + Zp) * math.sin(math.pi / n) - Zp - 2) * m:.1f} mm"}]
    d_pin, u_pin = pin_diameter(2 * Ft, b / 2 + 1.0)
    bore_s = gears.shaft_bore(T_in)
    T_out = T_in * i
    bore_c = chain.shaft_bore(T_out)
    checks.append({"group": "Shafts", "name": "Planet pin (steel)", "util": u_pin,
                   "detail": f"Ø{d_pin:g}, {2 * Ft:.0f} N" if d_pin else f"{2 * Ft:.0f} N is beyond Ø{PINS[-1]}"})
    if d_pin:
        checks.append(gears.hub_check(m, Zp, d_pin))
    if bore_s is None:
        checks.append({"group": "Shafts", "name": "Sun shaft", "util": 9.9, "detail": "beyond Ø30"})
    else:
        checks.append(gears.hub_check(m, Zs, bore_s))
    checks.append({"group": "Shafts", "name": "Carrier output shaft", "util": 0.0 if bore_c else 9.9,
                   "detail": f"Ø{bore_c:g} steel, {T_out / 1000:.1f} N·m" if bore_c else "beyond Ø100"})
    rim = max(RIM * m, 5.0)
    R_out = m * Zr / 2 + 1.25 * m + rim
    info = {"module": m, "Zs": Zs, "Zp": Zp, "Zr": Zr, "n": n, "ratio": i, "face_width": b, "a": m * (Zs + Zp) / 2,
            "ds": ds, "dp": m * Zp, "dr": m * Zr, "R_out": R_out, "Ft": Ft, "K_gamma": Kg, "T_in": T_in,
            "T_out": T_out, "pin": d_pin, "bore_sun": bore_s, "bore_carrier": bore_c, "rpm_carrier": n_c,
            "rpm_planet_rel": n_c * Zr / Zp, "eps_sp": e1, "eps_pr": e2, "volume": R_out ** 2 * b}
    return info, checks


def design(target, T_in, mat, rpm=None, n_planets=None):
    """Lightest planetary set with ratio within 2 % of target. Returns (design, checks)."""
    best = best_any = None
    for n in ((n_planets,) if n_planets else N_PLANETS):
        for Zs in Z_RANGE:
            for Zp in Z_RANGE:
                i = ratio(Zs, Zp)
                if abs(i - target) > 0.02 * target or (2 * Zs + 2 * Zp) % n or neighbour_util(Zs, Zp, n) > 1:
                    continue
                for m in gears.MODULES:
                    for k in gears.FACE_K:
                        info, checks = evaluate(m, Zs, Zp, n, k * m, T_in, mat, rpm)
                        worst = max(c["util"] for c in checks)
                        cand = (info, checks, worst)
                        if worst <= 1.0:
                            if best is None or info["volume"] < best[0]["volume"]:
                                best = cand
                            break
                        if best_any is None or worst < best_any[2]:
                            best_any = cand
    if best:
        return best[0], best[1]
    return None, (best_any[1] if best_any else [])


def phases(Zs, Zp, Zr, n):
    """Mesh phases for drawing (every gear built with a tooth on its +x axis, the ring with a SPACE there):
    planet k sits at theta_k = 2 pi k / n, turned by phi_k; the ring is turned by psi.
    A sun tooth at theta + e faces a planet space at theta + pi - e Zs/Zp (external mesh, opposite turns);
    a planet tooth at theta + t faces a ring space at theta + t Zp/Zr (internal mesh, same turns)."""
    ps, pp, pr = 2 * math.pi / Zs, 2 * math.pi / Zp, 2 * math.pi / Zr
    wrap = lambda x, p: (x + p / 2) % p - p / 2
    out = []
    for k in range(n):
        th = 2 * math.pi * k / n
        e = wrap(th, ps)                              # the nearest sun tooth is at th - e
        out.append((th, th + math.pi + e * Zs / Zp - pp / 2))
    th0, phi0 = out[0]
    t = wrap(phi0 - th0, pp)                          # planet 0's tooth nearest the ring is at th0 + t
    psi = th0 + t * Zp / Zr
    return out, psi
