"""
B9: Four-bar with a limited input angle: function generator synthesis (Freudenstein).

Request: while the input link (an "input arm" rather than a crank) turns by delta_theta, the output
rocker turns by delta_phi (linear relation). The input does NOT have to make full turns: the result can
be a double-rocker, a triple-rocker (non-Grashof) or a crank-rocker.

Freudenstein (O2 at the origin, O4 = (d, 0); theta: input angle, phi: output angle, from the x axis):
    K1 cos(phi) - K2 cos(theta) + K3 = cos(theta - phi)
    K1 = d/a,  K2 = d/c,  K3 = (a^2 - b^2 + c^2 + d^2) / (2 a c)
3 precision points (Chebyshev spacing) -> 3x3 linear system -> K1, K2, K3 -> link ratios (d = 1),
then scaled so that the input arm = a. The free start angles (theta0, phi0) are swept; every candidate
is verified by an INDEPENDENT simulation (assembly, same branch, structural error, transmission angle).
Standard library only (also runs inside Fusion).
"""

import math

import kinematics as kin


def _solve3(M, v):
    """3x3 linear system (Cramer)."""
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    D = det(M)
    if abs(D) < 1e-12:
        return None
    out = []
    for j in range(3):
        Mj = [row[:] for row in M]
        for i in range(3):
            Mj[i][j] = v[i]
        out.append(det(Mj) / D)
    return out


def chebyshev(x0, dx, n=3):
    return [x0 + dx / 2 * (1 - math.cos((2 * j - 1) * math.pi / (2 * n))) for j in range(1, n + 1)]


def _position(a, b, c, d, th, prev_B):
    """A and B for input angle th (B: the solution closest to the previous B -> same assembly branch)."""
    A = (a * math.cos(th), a * math.sin(th))
    sols = kin._circle_intersections(A, b, (d, 0.0), c)
    if not sols:
        return A, None
    B = min(sols, key=lambda s: math.dist(s, prev_B)) if prev_B else sols
    return A, B


def simulate_range(a, b, c, d, th0, dth, phi0, n=61):
    """Input th0 -> th0 + dth. First B: the branch closest to phi0. Returns frames or None (locks)."""
    frames, prev = [], (d + c * math.cos(phi0), c * math.sin(phi0))
    for k in range(n):
        th = th0 + dth * k / (n - 1)
        A, B = _position(a, b, c, d, th, prev)
        if B is None:
            return None
        if math.dist(B, prev) > 0.5 * c and k > 0:      # branch jump (passing a singular position)
            return None
        prev = B
        ab = (A[0] - B[0], A[1] - B[1])
        cb = (d - B[0], -B[1])
        cos_mu = (ab[0] * cb[0] + ab[1] * cb[1]) / (b * c)
        mu = math.degrees(math.acos(max(-1.0, min(1.0, cos_mu))))
        phi = math.atan2(B[1], B[0] - d)
        frames.append({"theta": math.degrees(th), "A": A, "B": B, "mu": mu, "phi": phi})
    return frames


def _unwrap(angles):
    out = [angles[0]]
    for x in angles[1:]:
        out.append(out[-1] + (x - out[-1] + math.pi) % (2 * math.pi) - math.pi)
    return out


def candidates(a_len, in_swing_deg, out_swing_deg, mu_design=45.0, step_deg=5, max_ratio=8.0, top=40):
    """Kinematically valid candidates (error <= 1 degree, transmission angle within the design range),
    sorted from the most balanced link ratio down. If one fails a later check (strength, layer
    interference) the pipeline tries the next."""
    out = []
    synthesize(a_len, in_swing_deg, out_swing_deg, mu_design, step_deg, max_ratio, collect=out)
    out.sort(key=lambda m: (round(m["ratio"], 2), m["size"]))
    # drop near-duplicates (neighbouring sweep points)
    distinct = []
    for m in out:
        if all(max(abs(m[k] - n[k]) for k in "abcd") > 0.05 * m["a"] for n in distinct):
            distinct.append(m)
        if len(distinct) >= top:
            break
    return distinct


def synthesize(a_len, in_swing_deg, out_swing_deg, mu_design=45.0, step_deg=5, max_ratio=8.0, collect=None):
    """
    Returns the best design (dict) or None.
    Choice: among those with structural error <= 1 degree and transmission angle within
    [mu_design, 180 - mu_design], the most BALANCED ratio (longest / shortest link smallest; ties: the
    smaller one). "Most compact" produced links too short for their bosses. If nothing is valid, the one
    with the best transmission angle.
    """
    dth, dphi_abs = math.radians(in_swing_deg), math.radians(out_swing_deg)
    best = best_ok = None
    for sign in (1, -1):                                  # the output may turn the same or the opposite way
        dphi = sign * dphi_abs
        for i in range(0, 360, step_deg):
            th0 = math.radians(i)
            ths = chebyshev(th0, dth)
            for j in range(0, 360, step_deg):
                phi0 = math.radians(j)
                phis = [phi0 + dphi * (t - th0) / dth for t in ths]
                K = _solve3([[math.cos(p), -math.cos(t), 1.0] for t, p in zip(ths, phis)],
                            [math.cos(t - p) for t, p in zip(ths, phis)])
                if not K or K[0] <= 0 or K[1] <= 0:
                    continue
                d = 1.0
                a, c = d / K[0], d / K[1]
                b2 = a * a + c * c + d * d - 2 * a * c * K[2]
                if b2 <= 0:
                    continue
                b = math.sqrt(b2)
                if max(a, b, c, d) / min(a, b, c, d) > max_ratio:
                    continue
                fr = simulate_range(a, b, c, d, th0, dth, phi0)
                if not fr:
                    continue
                phi_sim = _unwrap([f["phi"] for f in fr])
                # Structural error: largest difference between the simulated output and the wanted
                # linear relation (phi_sim[0] and phi0 may differ by 2*pi: shifted)
                shift = phi_sim[0] - phi0
                err = max(abs(ps - shift - (phi0 + dphi * k / (len(fr) - 1))) for k, ps in enumerate(phi_sim))
                mus = [f["mu"] for f in fr]
                worst = max(abs(90 - min(mus)), abs(90 - max(mus)))
                s = a_len / a
                ratio = max(a, b, c, d) / min(a, b, c, d)
                cand = {"a": a * s, "b": b * s, "c": c * s, "d": d * s, "th0": th0, "phi0": phi0,
                        "dth": dth, "dphi": dphi, "err_deg": math.degrees(err),
                        "mu_min": min(mus), "mu_max": max(mus), "worst_dev": worst,
                        "size": (b + c + d) * s, "ratio": ratio}
                if math.degrees(err) <= 1.0 and worst <= 90 - mu_design:
                    if collect is not None:
                        collect.append(cand)
                    if best_ok is None or (round(ratio, 2), cand["size"]) < (round(best_ok["ratio"], 2), best_ok["size"]):
                        best_ok = cand
                if best is None or (math.degrees(err), worst) < (best["err_deg"], best["worst_dev"]):
                    best = cand
    return best_ok or best


def frames_for(m, n=61):
    """Moves the design into our coordinates (O4 at the origin) and simulates it.
    Returns (kinematics-compatible mech, frames)."""
    a, b, c, d = m["a"], m["b"], m["c"], m["d"]
    fr = simulate_range(a, b, c, d, m["th0"], m["dth"], m["phi0"], n)
    shift = (-d, 0.0)                                     # O4 -> origin
    out = []
    for f in fr:
        out.append({"theta": f["theta"], "mu": f["mu"],
                    "A": (f["A"][0] + shift[0], f["A"][1] + shift[1]),
                    "B": (f["B"][0] + shift[0], f["B"][1] + shift[1]),
                    "rocker": math.degrees(f["phi"])})
    mech = dict(m, o2=(-d, 0.0), b_ref=out[0]["B"], Q=1.0)
    return mech, out


def measure_range(frames):
    """From the simulation: input and output swing, transmission angle range."""
    th = [f["theta"] for f in frames]
    ph = [math.degrees(x) for x in _unwrap([math.radians(f["rocker"]) for f in frames])]
    return {"in_swing": th[-1] - th[0], "out_swing": ph[-1] - ph[0],
            "mu_min": min(f["mu"] for f in frames), "mu_max": max(f["mu"] for f in frames)}
