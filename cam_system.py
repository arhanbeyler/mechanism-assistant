"""
E3: Cam + follower return spring as one system (Norton, ch. 8 and 15: follower jump).

The cam module sets the motion s(theta); at a given cam speed the follower accelerates with
    x'' = a(theta) * w^2        (a: d2s/dtheta2 per rad^2, w = 2 pi rpm / 60)
Contact force between roller and cam (damping and gravity neglected):
    F_c = m x'' + k (y0 + s) + F_ext     (m: follower mass, y0: spring preload deflection, F_ext: load pressing
                                          the follower onto the cam)
The follower jumps off the cam if F_c < 0. The spring must supply  F_s(theta) >= N_JUMP (-m x'') - F_ext  and at
least F_MIN everywhere. The preload deflection y0 is swept; for each y0 the smallest rate that does this is
k = max_i D_i / (y0 + s_i), and springs.design builds the spring for the peak force k (y0 + h) at deflection
y0 + h (so k = F / y): the modules feed each other. The lightest valid spring wins. (Minimising only the peak
force drives k -> 0: a "constant-force" spring with an endless preload, hundreds of coils.)
Units: mm, N, kg, rpm, m/s^2.
"""

import math

import cams
import springs

N_JUMP = 1.3          # safety on the inertia term (Norton: design margin against jump)
F_MIN = 2.0           # N, the spring always keeps at least this much on the roller


def follower_accel(table, rpm):
    """[(theta_deg, s_mm, x_dd m/s^2)] over one cam turn."""
    w = rpm * 2 * math.pi / 60
    return [(th, s, a * w * w / 1000.0) for th, s, _, a in table]


def demand(acc, mass, F_ext):
    """Spring force needed at each point: [(s, D)]."""
    return [(s, max(F_MIN, N_JUMP * (-mass * xdd) - F_ext)) for _, s, xdd in acc]


def _installed(geo, need, h, mat):
    """The real spring (rounded coils -> real rate): installed preload and checks at the real top force."""
    k = geo["k"]
    y0 = max(D / k - s for s, D in need)                        # installed preload for the real rate
    checks = springs.check(geo["d"], geo["D1"], geo["Na"], k * (y0 + h), y0 + h, mat)[0]
    return y0, [c for c in checks if c["group"] != "Force"]      # "Force" = the design target, not a check here


def design(cam_design, table, rpm, mass, F_ext=0.0, mat=None):
    """Returns (system dict, checks) or (None, checks). cam_design: cams.design() output; table: cams.svaj."""
    mat = mat or springs.SPRING_MATERIALS["music_wire"]
    acc = follower_accel(table, rpm)
    h = cam_design["h"]
    need = demand(acc, mass, F_ext)
    best, best_fail = None, None
    for i in range(1, 121):                                    # preload deflection 0.05 h ... 6 h
        y0 = 0.05 * h * i
        k = max(D / (y0 + s) for s, D in need)
        geo, s_checks = springs.design(k * (y0 + h), y0 + h, mat)
        if geo is None:
            best_fail = best_fail or s_checks
            continue
        y0_real, real_checks = _installed(geo, need, h, mat)
        if max(c["util"] for c in real_checks) > 1.0:          # fails once the coils are rounded
            best_fail = best_fail or real_checks
            continue
        if best is None or geo["mass_index"] < best[0]["mass_index"] - 1e-12:
            best = (geo, y0_real, real_checks)
    if best is None:
        return None, [dict(c, group=f"Spring {c['group'].lower()}") for c in (best_fail or [])]
    geo, y0, s_checks = best
    k = geo["k"]
    checks = [dict(c, group=f"Spring {c['group'].lower()}") for c in s_checks]
    contact = [(th, mass * xdd + k * (y0 + s) + F_ext) for th, s, xdd in acc]
    c_min = min(c for _, c in contact)
    inertia = max(-mass * xdd for _, _, xdd in acc)
    use = max(D / (k * (y0 + s)) for s, D in need)             # demanded / supplied spring force
    checks.insert(0, {"group": "Jump", "name": "Follower stays on the cam", "util": use if c_min > 0 else 99.0,
                      "detail": f"smallest contact force {c_min:.1f} N (largest pull-off inertia {inertia:.1f} N, "
                                f"safety {N_JUMP:g} on inertia)"})
    geo = dict(geo, L0=geo["Ls"] + (1 + springs.XI) * (y0 + h), y_work=y0 + h)
    return {"k": k, "preload_N": k * y0, "preload_mm": y0, "F_top": k * (y0 + h), "spring": geo,
            "rpm": rpm, "mass": mass, "F_ext": F_ext, "contact_min": c_min,
            "contact_max": max(c for _, c in contact), "inertia_max": inertia,
            "a_max": max(abs(x) for _, _, x in acc),
            "contact_curve": [(round(th, 1), round(c, 2)) for th, c in contact[::4]]}, checks
