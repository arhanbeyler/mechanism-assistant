"""
F4b: Watt II six-bar for a large output swing: two four-bars in series sharing a ternary link.

Stage 1: a crank-rocker (crank a, rocker swing psi1, kinematics.synthesize_crank_rocker; O4 at the origin).
Stage 2: the rocker's second arm (the ternary link, pivot O4) is the input of a function generator
(function_gen, Freudenstein): while the arm turns psi1, the output turns psi (linearly). Its frame sits at O4,
turned so the output pivot O6 lies beyond O4 on the line O2 -> O4; the ternary link's arms are fixed at the
angle gamma that lines the arm's range up with stage 2's input range.
Candidates: psi1 in 60..90 deg and stage-2 input arm 0.6..1.2 x the rocker (links >= 12 mm and 0.4 a, longest /
shortest <= 10, so it can be built); every candidate is SIMULATED
as a whole over a crank turn (both loops by circle intersections, same branches): the total swing within 1 deg,
both transmission angles within 40..140 deg. Choice: the best worst transmission angle, then the smallest.
Statics with an output tip load F: stage 2 as a four-bar with F at its output tip; its largest moment on the
ternary link becomes an equivalent tip force on stage 1's rocker (virtual work).
Units: mm, degrees.
"""

import math

import function_gen
import kinematics as kin
import strength

PSI1 = (60, 70, 80, 90)
ARM2 = (0.6, 0.8, 1.0, 1.2, 1.5, 2.0)
MU_MIN, MU_MAX = 40.0, 140.0
LINK_MIN = 12.0            # every link at least this (room for two bosses) and 0.4 x the crank
RATIO_MAX = 10.0


def _circ(p0, r0, p1, r1, prev):
    sols = kin._circle_intersections(p0, r0, p1, r1)
    if not sols:
        return None
    return min(sols, key=lambda s: math.dist(s, prev)) if prev else sols[0]


def simulate(m, n=360):
    """Whole six-bar over a crank turn. m: the design dict. Returns frames or None (locks / branch jump)."""
    frames1 = kin.simulate(m["m1"], n)
    O4, O6 = (0.0, 0.0), m["O6"]
    c2arm, b2, c2, g = m["a2"], m["b2"], m["c2"], m["gamma"]
    rhos = [math.degrees(math.atan2(f["B"][1], f["B"][0])) for f in frames1]
    k = min(range(len(frames1)), key=lambda i: math.cos(math.radians(rhos[i] - m["rho_min"])) * -1)
    frames1 = frames1[k:] + frames1[:k]                   # begin where stage 2's start is known
    prev = m["B2_start"]
    out = []
    for f in frames1:
        rho = math.atan2(f["B"][1], f["B"][0])
        A2 = (c2arm * math.cos(rho + g), c2arm * math.sin(rho + g))
        B2 = _circ(A2, b2, O6, c2, prev)
        if B2 is None or (prev and math.dist(B2, prev) > 0.5 * c2):
            return None
        prev = B2
        v1, v2 = (A2[0] - B2[0], A2[1] - B2[1]), (O6[0] - B2[0], O6[1] - B2[1])
        mu2 = math.degrees(math.acos(max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (b2 * c2)))))
        out.append(dict(f, rho=math.degrees(rho), A2=A2, B2=B2, mu2=mu2,
                        phi=math.degrees(math.atan2(B2[1] - O6[1], B2[0] - O6[0]))))
    return out


def _unwrap_deg(a):
    out = [a[0]]
    for x in a[1:]:
        out.append(out[-1] + (x - out[-1] + 180) % 360 - 180)
    return out


def measure(frames):
    ph = _unwrap_deg([f["phi"] for f in frames])
    return {"swing": max(ph) - min(ph), "mu1_min": min(f["mu"] for f in frames), "mu1_max": max(f["mu"] for f in frames),
            "mu2_min": min(f["mu2"] for f in frames), "mu2_max": max(f["mu2"] for f in frames)}


def _assemble(m1, frames1, fg):
    """Place stage 2 (function_gen design fg, its own frame: input pivot at the origin, O6 at (d, 0))."""
    rhos = _unwrap_deg([math.degrees(math.atan2(f["B"][1], f["B"][0])) for f in frames1])
    rho_min = math.radians(min(rhos))
    o2 = m1["o2"]
    R = math.atan2(-o2[1], -o2[0])                         # direction O2 -> O4: stage 2 continues that way
    th0 = fg["th0"]
    # the arm turns from rho_min upwards while stage 2's input goes th0 -> th0 + dth (same sense assumed;
    # if stage 1's rocker turns the other way, the arm is mirrored by gamma accordingly)
    gamma = th0 + R - rho_min
    O6 = (fg["d"] * math.cos(R), fg["d"] * math.sin(R))
    A2s = (fg["a"] * math.cos(th0 + R), fg["a"] * math.sin(th0 + R))
    # stage 2's starting B in global coordinates (from function_gen's own start)
    fr2 = function_gen.simulate_range(fg["a"], fg["b"], fg["c"], fg["d"], th0, fg["dth"], fg["phi0"])
    Bl = fr2[0]["B"]
    B2s = (Bl[0] * math.cos(R) - Bl[1] * math.sin(R), Bl[0] * math.sin(R) + Bl[1] * math.cos(R))
    # stage 1 keeps its own start (b_ref also picks its assembly branch in kinematics.simulate); the
    # six-bar simulation starts stage 2 from the B2 nearest to stage 2's own start, found below
    k = min(range(len(frames1)), key=lambda i: abs(rhos[i] - math.degrees(rho_min)))
    return {"m1": m1, "rho_min": math.degrees(rho_min), "a2": fg["a"], "b2": fg["b"], "c2": fg["c"], "d2": fg["d"], "O6": O6, "gamma": gamma,
            "B2_start": B2s, "A2_start": A2s, "fg": fg}


def synthesize(a, psi):
    """Best Watt II six-bar with crank a (mm) and total output swing psi (deg). Returns (design, frames) or (None, None)."""
    best = None
    for psi1 in PSI1:
        m1 = kin.synthesize_crank_rocker(a, psi1)
        if m1 is None:
            continue
        frames1 = kin.simulate(m1, 360)
        for k in ARM2:
            for fg in function_gen.candidates(k * m1["c"], psi1, psi, mu_design=MU_MIN, max_ratio=RATIO_MAX)[:6]:
                m = _assemble(m1, frames1, fg)
                fr = simulate(m)
                if not fr:
                    continue
                me = measure(fr)
                mus = (me["mu1_min"], me["mu2_min"], 180 - me["mu1_max"], 180 - me["mu2_max"])
                lens = [m1["a"], m1["b"], m1["c"], m1["d"], fg["a"], fg["b"], fg["c"], fg["d"]]
                buildable = min(lens) >= max(LINK_MIN, 0.4 * a) and max(lens) / min(lens) <= RATIO_MAX
                ok = abs(me["swing"] - psi) <= 1.0 and min(mus) >= MU_MIN and buildable
                size = m1["b"] + m1["c"] + m1["d"] + fg["b"] + fg["c"] + fg["d"]
                key = (ok, round(min(mus), 1), -size)
                if best is None or key > best[0]:
                    best = (key, dict(m, psi1=psi1, measured=me, ok=ok), fr)
    if best is None:
        return None, None
    return best[1], best[2]


def forces(m, frames, F):
    """Stage 2 with F at the output tip; its moment on the ternary link as a tip force on stage 1's rocker."""
    O6 = m["O6"]
    mech2 = {"a": m["a2"], "b": m["b2"], "c": m["c2"], "d": m["d2"], "o2": (-O6[0], -O6[1])}
    fr2 = [{"A": (f["A2"][0] - O6[0], f["A2"][1] - O6[1]), "B": (f["B2"][0] - O6[0], f["B2"][1] - O6[1]),
            "mu": f["mu2"]} for f in frames]
    f2 = strength.analyze_forces(mech2, fr2, F)                  # M_crank_max: the moment on the ternary arm
    F1 = f2["M_crank_max"] / m["m1"]["c"]
    f1 = strength.analyze_forces(m["m1"], frames, F1)
    return f1, f2, F1
