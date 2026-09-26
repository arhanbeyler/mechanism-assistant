"""
K2: dynamic force analysis of the four-bar (Norton ch. 11, matrix method), constant crank speed.

Links 2 (crank), 3 (coupler), 4 (rocker) as uniform bars of their section (w x t, density rho):
m = rho w t L, I_G = m (L^2 + w^2) / 12, centre of mass mid-way between the pins. Velocities and
accelerations of the centres and the link angles come from central differences of the position simulation
(crank at a constant w: d/dt = w d/dtheta). The external load is a torque F c on the rocker that always
resists its motion (the tip load of the static analysis). Gravity is left out (a machine lying flat).
Per crank angle a 9 x 9 linear system gives the pin forces F12, F32, F43, F14 and the drive torque T12:
    link 2:  F12 + F32 = m2 aG2;          R12 x F12 + R32 x F32 + T12 = I2 a2
    link 3:  -F32 + F43 = m3 aG3;         -R23 x F32 + R43 x F43 = I3 a3
    link 4:  -F43 + F14 = m4 aG4;         -R34 x F43 + R14 x F14 + M4 = I4 a4
Independent check (energy): T12 w2 + M4 w4 = sum(m aG . vG + I a w).
Shaking force on the frame: -(F12 + F14).  Flywheel: the drive torque's swing about its mean gives the energy
fluctuation dE = max - min of the integral of (T12 - T_mean) d(theta).
Units inside: SI (m, kg, N, N*m); lengths in mm and densities in kg/m^3 at the interface.
"""

import math

DENSITY = {"PLA": 1240.0, "PETG": 1270.0, "aluminum": 2700.0, "steel": 7850.0}


def _solve(A, b):
    """Gaussian elimination with partial pivoting (9 x 9)."""
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            for k in range(c, n + 1):
                M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x


def _unwrap(angles):
    out = [angles[0]]
    for a in angles[1:]:
        d = a - out[-1]
        d -= 2 * math.pi * round(d / (2 * math.pi))
        out.append(out[-1] + d)
    return out


def _diff(vals, h, periodic_span=0.0):
    """First and second central derivatives of a periodic sequence (the last value wraps to the first,
    plus periodic_span for angles that turn a full turn over the cycle)."""
    n = len(vals)
    d1, d2 = [], []
    for i in range(n):
        prev = vals[i - 1] - (periodic_span if i == 0 else 0.0)
        nxt = vals[(i + 1) % n] + (periodic_span if i == n - 1 else 0.0)
        d1.append((nxt - prev) / (2 * h))
        d2.append((nxt - 2 * vals[i] + prev) / (h * h))
    return d1, d2


def link_props(L, w, t, rho):
    m = rho * (L / 1000) * (w / 1000) * (t / 1000)
    return m, m * ((L / 1000) ** 2 + (w / 1000) ** 2) / 12


def analyse(mech, frames, section_links, rho, rpm, F_tip=0.0):
    """Drive torque and pin forces over a turn. section_links: {"Crank": {"width", "thickness"}, ...}."""
    n = len(frames)
    w2 = 2 * math.pi * rpm / 60
    h = 2 * math.pi / n                                       # crank step (rad)
    O2 = (mech["o2"][0] / 1000, mech["o2"][1] / 1000)
    O4 = (0.0, 0.0)
    A = [(f["A"][0] / 1000, f["A"][1] / 1000) for f in frames]
    B = [(f["B"][0] / 1000, f["B"][1] / 1000) for f in frames]
    props = {name: link_props(mech[key], section_links[name]["width"], section_links[name]["thickness"], rho)
             for name, key in (("Crank", "a"), ("Coupler", "b"), ("Rocker", "c"))}
    G = {"Crank": [((O2[0] + a[0]) / 2, (O2[1] + a[1]) / 2) for a in A],
         "Coupler": [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in zip(A, B)],
         "Rocker": [(b[0] / 2, b[1] / 2) for b in B]}
    th = {"Crank": _unwrap([math.atan2(a[1] - O2[1], a[0] - O2[0]) for a in A]),
          "Coupler": _unwrap([math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(A, B)]),
          "Rocker": _unwrap([math.atan2(b[1], b[0]) for b in B])}
    kin = {}
    for name in props:
        span = th[name][-1] - th[name][0]
        span = 2 * math.pi * round((span + (th[name][1] - th[name][0])) / (2 * math.pi))   # full turns over the cycle
        dth, ddth = _diff(th[name], h, span)
        gx1, gx2 = _diff([g[0] for g in G[name]], h)
        gy1, gy2 = _diff([g[1] for g in G[name]], h)
        kin[name] = {"w": [v * w2 for v in dth], "al": [v * w2 * w2 for v in ddth],
                     "v": [(x * w2, y * w2) for x, y in zip(gx1, gy1)],
                     "a": [(x * w2 * w2, y * w2 * w2) for x, y in zip(gx2, gy2)]}
    c = mech["c"] / 1000
    out = {k: [] for k in ("theta", "T", "T_energy", "F12", "F32", "F43", "F14", "shake", "M4")}
    (m2, I2), (m3, I3), (m4, I4) = props["Crank"], props["Coupler"], props["Rocker"]
    for i in range(n):
        g2, g3, g4 = G["Crank"][i], G["Coupler"][i], G["Rocker"][i]
        R12, R32 = (O2[0] - g2[0], O2[1] - g2[1]), (A[i][0] - g2[0], A[i][1] - g2[1])
        R23, R43 = (A[i][0] - g3[0], A[i][1] - g3[1]), (B[i][0] - g3[0], B[i][1] - g3[1])
        R34, R14 = (B[i][0] - g4[0], B[i][1] - g4[1]), (O4[0] - g4[0], O4[1] - g4[1])
        w4 = kin["Rocker"]["w"][i]
        M4 = -math.copysign(F_tip * c, w4) if abs(w4) > 1e-12 else 0.0
        a2, a3, a4 = kin["Crank"]["a"][i], kin["Coupler"]["a"][i], kin["Rocker"]["a"][i]
        #      F12x     F12y    F32x     F32y    F43x     F43y    F14x    F14y  T12
        Am = [[1, 0, 1, 0, 0, 0, 0, 0, 0],
              [0, 1, 0, 1, 0, 0, 0, 0, 0],
              [-R12[1], R12[0], -R32[1], R32[0], 0, 0, 0, 0, 1],
              [0, 0, -1, 0, 1, 0, 0, 0, 0],
              [0, 0, 0, -1, 0, 1, 0, 0, 0],
              [0, 0, R23[1], -R23[0], -R43[1], R43[0], 0, 0, 0],
              [0, 0, 0, 0, -1, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, -1, 0, 1, 0],
              [0, 0, 0, 0, R34[1], -R34[0], -R14[1], R14[0], 0]]
        bv = [m2 * a2[0], m2 * a2[1], I2 * kin["Crank"]["al"][i],
              m3 * a3[0], m3 * a3[1], I3 * kin["Coupler"]["al"][i],
              m4 * a4[0], m4 * a4[1], I4 * kin["Rocker"]["al"][i] - M4]
        x = _solve(Am, bv)
        power = sum(m * (kin[nm]["a"][i][0] * kin[nm]["v"][i][0] + kin[nm]["a"][i][1] * kin[nm]["v"][i][1])
                    + I * kin[nm]["al"][i] * kin[nm]["w"][i]
                    for nm, (m, I) in props.items())
        out["theta"].append(frames[i]["theta"])
        out["T"].append(x[8])
        out["T_energy"].append((power - M4 * w4) / kin["Crank"]["w"][i])
        out["F12"].append(math.hypot(x[0], x[1]))
        out["F32"].append(math.hypot(x[2], x[3]))
        out["F43"].append(math.hypot(x[4], x[5]))
        out["F14"].append(math.hypot(x[6], x[7]))
        out["shake"].append(math.hypot(x[0] + x[6], x[1] + x[7]))
        out["M4"].append(M4)
    T_mean = sum(out["T"]) / n
    cum, s = [], 0.0
    for T in out["T"]:
        s += (T - T_mean) * h
        cum.append(s)
    out.update(T_mean=T_mean, T_max=max(out["T"], key=abs), dE=max(cum) - min(cum), masses={k: v[0] for k, v in props.items()},
               F_pin_max=max(max(out["F32"]), max(out["F43"])), F_ground_max=max(max(out["F12"]), max(out["F14"])),
               shake_max=max(out["shake"]), rpm=rpm)
    return out
