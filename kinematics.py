"""
Kinematics engine: crank-rocker four-bar linkage.

Naming (Norton):
    a = crank (O2-A)       b = coupler (A-B)
    c = rocker (O4-B)      d = ground link (O2-O4)

Synthesis: crank length (a) and rocker swing angle (psi) are given, time ratio Q = 1 (no quick
return). Norton's graphical method:
    c = a / sin(psi/2)
    O2 lies on the line through the rocker's two extreme positions B1-B2;
    |O2B1| = b + a (extended), |O2B2| = b - a (folded).
The remaining free parameter b is swept so that the transmission angle stays as close to 90 degrees
as needed.
"""

import math
import sys

# Note: this file deliberately uses only the Python standard library (no numpy), because Fusion 360's
# own Python has no numpy. Points are (x, y) tuples.

MU_MIN_DEG = 40.0
MU_MAX_DEG = 140.0


# ---------------------------------------------------------------- 2D vector helpers

def _add(p, q):
    return (p[0] + q[0], p[1] + q[1])


def _sub(p, q):
    return (p[0] - q[0], p[1] - q[1])


def _scale(p, k):
    return (p[0] * k, p[1] * k)


def _norm(p):
    return math.hypot(p[0], p[1])


def _dot(p, q):
    return p[0] * q[0] + p[1] * q[1]


def _cross2(u, v):
    """2D cross product (z component): its sign gives the turning direction."""
    return u[0] * v[1] - u[1] * v[0]


def _linspace(start, stop, n, endpoint=True):
    step = (stop - start) / ((n - 1) if endpoint else n)
    return [start + i * step for i in range(n)]


# ---------------------------------------------------------------- checks

def grashof(a, b, c, d):
    """Grashof check and class. Returns (satisfied, class)."""
    links = sorted([a, b, c, d])
    s, p, q, l = links[0], links[1], links[2], links[3]
    if s + l > p + q:
        return False, "NON-Grashof (no link can fully rotate)"
    if math.isclose(s + l, p + q):
        return True, "Special Grashof (has a change point)"
    shortest = min((a, "crank"), (b, "coupler"), (c, "rocker"), (d, "ground"))[1]
    kind = {
        "crank": "crank-rocker",
        "ground": "double-crank",
        "coupler": "double-rocker",
        "rocker": "crank-rocker (driven from the rocker side)",
    }[shortest]
    return True, kind


def transmission_extremes(a, b, c, d):
    """Extreme transmission angles (degrees). They occur when the crank is collinear with the ground link."""
    def mu(diag):
        cos_mu = (b * b + c * c - diag * diag) / (2 * b * c)
        return math.degrees(math.acos(max(-1.0, min(1.0, cos_mu))))
    return mu(abs(d - a)), mu(d + a)


# ---------------------------------------------------------------- synthesis

def synthesize_crank_rocker(a, psi_deg, mu_design=45.0, b_steps=2000):
    """
    Four-bar synthesis for crank (a) and swing (psi).

    Trade-off: a longer b brings the transmission angle towards 90 degrees but makes the mechanism
    bigger. The SHORTEST b that keeps the transmission angle within [mu_design, 180 - mu_design] is
    chosen. If none does, the one with the best transmission angle is returned.
    """
    psi = math.radians(psi_deg)
    c = a / math.sin(psi / 2)

    # Coordinates: O4 at the origin, B1/B2 are the rocker's extreme positions (symmetric about x)
    bx = c * math.cos(psi / 2)
    by = c * math.sin(psi / 2)

    best = None
    for b in _linspace(1.05 * a, 15 * a, b_steps):   # short to long
        o2 = (bx, -by - (b - a))            # on the line B1-B2, below B2
        d = math.hypot(*o2)
        mu1, mu2 = transmission_extremes(a, b, c, d)
        worst = max(abs(90 - mu1), abs(90 - mu2))  # largest deviation from 90
        cand = dict(a=a, b=b, c=c, d=d, o2=o2, b_ref=(bx, by), mu_min=min(mu1, mu2),
                    mu_max=max(mu1, mu2), worst_dev=worst, size=b + c + d, Q=1.0)
        if worst <= 90 - mu_design:
            return cand                     # first (most compact) solution meeting the target
        if best is None or worst < best["worst_dev"]:
            best = cand
    return best


def synthesize_quick_return(a, psi_deg, Q, mu_design=45.0, n=3600):
    """
    Quick-return (Q > 1) crank-rocker synthesis (Norton).

    alpha = 180 (Q-1)/(Q+1). O2 lies on the circle that sees the chord B1-B2 under the angle alpha.
    Extended position: |O2B1| = b + a, folded: |O2B2| = b - a.
    Designed with a unit rocker (c = 1), then scaled so that the crank = a.
    The circle is swept; the MOST COMPACT design meeting the transmission target is chosen.
    """
    if Q <= 1.0:
        return synthesize_crank_rocker(a, psi_deg, mu_design)
    psi = math.radians(psi_deg)
    alpha = math.pi * (Q - 1) / (Q + 1)
    B1 = (math.cos(psi / 2), math.sin(psi / 2))
    B2 = (math.cos(psi / 2), -math.sin(psi / 2))
    chord = _norm(_sub(B1, B2))
    R = chord / (2 * math.sin(alpha))
    mid = _scale(_add(B1, B2), 0.5)

    best = best_ok = None
    for side in (1, -1):
        center = _add(mid, (side * R * math.cos(alpha), 0.0))
        for t in _linspace(0, 2 * math.pi, n, endpoint=False):
            o2 = _add(center, (R * math.cos(t), R * math.sin(t)))
            L1, L2 = _norm(_sub(B1, o2)), _norm(_sub(B2, o2))
            if min(L1, L2) < 1e-9:
                continue
            v1, v2 = _scale(_sub(B1, o2), 1 / L1), _scale(_sub(B2, o2), 1 / L2)
            ang = math.acos(max(-1.0, min(1.0, _dot(v1, v2))))
            if abs(ang - alpha) > 1e-6 or abs(L1 - L2) < 1e-6:
                continue                     # wrong arc of the circle
            a_u, b_u = abs(L1 - L2) / 2, (L1 + L2) / 2
            # The extended (ext) and folded (fold) positions must be on the SAME assembly branch:
            # the signed coupler-rocker angle has the same sign in both.
            ext, fold = (B1, B2) if L1 > L2 else (B2, B1)
            A_ext = _add(o2, _scale(_sub(ext, o2), a_u / _norm(_sub(ext, o2))))
            A_fold = _sub(o2, _scale(_sub(fold, o2), a_u / _norm(_sub(fold, o2))))
            s_ext = _cross2(_sub(ext, A_ext), _scale(ext, -1))
            s_fold = _cross2(_sub(fold, A_fold), _scale(fold, -1))
            if s_ext * s_fold <= 0:
                continue                     # different assembly branches: never happens in reality
            k = a / a_u                      # scale: crank = a
            b, c, d = b_u * k, k, _norm(o2) * k
            if not grashof(a, b, c, d)[0] or min(a, b, c, d) != a:
                continue                     # must be a crank-rocker (shortest = crank)
            mu1, mu2 = transmission_extremes(a, b, c, d)
            worst = max(abs(90 - mu1), abs(90 - mu2))
            cand = dict(a=a, b=b, c=c, d=d, o2=_scale(o2, k), b_ref=_scale(ext, k),
                        mu_min=min(mu1, mu2), mu_max=max(mu1, mu2),
                        worst_dev=worst, size=b + c + d, Q=Q)
            # Ties keep the first one found (1e-9 tolerance): rounding noise between mirror-symmetric
            # solutions must not change the choice, so the same input always gives the same output.
            if worst <= 90 - mu_design and (best_ok is None or cand["size"] < best_ok["size"] - 1e-9):
                best_ok = cand
            if best is None or worst < best["worst_dev"] - 1e-9:
                best = cand
    return best_ok or best


# ---------------------------------------------------------------- position analysis

def _circle_intersections(p0, r0, p1, r1):
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    dist = math.hypot(dx, dy)
    if dist > r0 + r1 or dist < abs(r0 - r1):
        return []  # the mechanism cannot reach this position (it locks)
    k = (r0 * r0 - r1 * r1 + dist * dist) / (2 * dist)
    h = math.sqrt(max(0.0, r0 * r0 - k * k))
    mx, my = x0 + k * dx / dist, y0 + k * dy / dist
    return [(mx + h * dy / dist, my - h * dx / dist),
            (mx - h * dy / dist, my + h * dx / dist)]


def simulate(m, n=360):
    """The crank turns 360 degrees; at every step the points A, B and the transmission angle."""
    o2 = tuple(m["o2"])
    o4 = (0.0, 0.0)
    ground_angle = math.atan2(-o2[1], -o2[0])  # direction O2 -> O4

    # Start: the extended position from the synthesis (crank towards the rocker tip B_ref) -> right branch
    b_ref = m.get("b_ref")
    th0 = 0.0
    if b_ref is not None:
        th0 = math.atan2(b_ref[1] - o2[1], b_ref[0] - o2[0]) - ground_angle

    frames, prev_b = [], b_ref
    for step in _linspace(0, 2 * math.pi, n, endpoint=False):
        th = (th0 + step) % (2 * math.pi)
        A = _add(o2, (m["a"] * math.cos(ground_angle + th), m["a"] * math.sin(ground_angle + th)))
        sols = _circle_intersections(A, m["b"], o4, m["c"])
        if not sols:
            raise RuntimeError(f"The mechanism locks at crank angle {math.degrees(th):.1f} degrees")
        # Continuity: take the solution closest to the previous B (the assembly branch must not change)
        if prev_b is None:
            B = max(sols, key=lambda s: s[0])
        else:
            B = min(sols, key=lambda s: math.dist(s, prev_b))
        prev_b = B
        ab, ob = _sub(B, A), _sub(B, o4)
        cos_mu = _dot(ab, ob) / (_norm(ab) * _norm(ob))
        mu = math.degrees(math.acos(max(-1.0, min(1.0, cos_mu))))
        rocker = math.degrees(math.atan2(B[1], B[0]))
        frames.append(dict(theta=math.degrees(th), A=A, B=B, mu=mu, rocker=rocker))
    return frames


# ---------------------------------------------------------------- report

def measure(frames):
    """Values measured from the simulation: swing, time ratio, transmission angle range."""
    rock = [f["rocker"] for f in frames]
    mus = [f["mu"] for f in frames]
    # Time ratio: crank rotation between the rocker's two extreme positions
    n = len(frames)
    i_max, i_min = rock.index(max(rock)), rock.index(min(rock))
    go = ((i_min - i_max) % n) * 360.0 / n
    return dict(swing=max(rock) - min(rock),
                Q=max(go, 360 - go) / min(go, 360 - go),
                mu_min=min(mus), mu_max=max(mus))


def report(m, frames, psi_target):
    ok, kind = grashof(m["a"], m["b"], m["c"], m["d"])
    meas = measure(frames)
    swing, q_meas = meas["swing"], meas["Q"]
    mus = (meas["mu_min"], meas["mu_max"])
    mu_ok = MU_MIN_DEG <= min(mus) and max(mus) <= MU_MAX_DEG

    print("=" * 52)
    print(f"  Crank   a = {m['a']:8.2f} mm")
    print(f"  Coupler b = {m['b']:8.2f} mm")
    print(f"  Rocker  c = {m['c']:8.2f} mm")
    print(f"  Ground  d = {m['d']:8.2f} mm")
    print("-" * 52)
    print(f"  Grashof          : {'✓' if ok else '✗'} {kind}")
    print(f"  Swing (target)   : {psi_target:.2f} deg")
    print(f"  Swing (simulated): {swing:.2f} deg")
    print(f"  Time ratio Q     : target {m.get('Q', 1.0):.2f} / simulated {q_meas:.2f}")
    print(f"  Total length     : {m['b'] + m['c'] + m['d']:.1f} mm (b+c+d)")
    print(f"  Transmission     : {min(mus):.1f} .. {max(mus):.1f} deg "
          f"{'✓' if mu_ok else '✗'} (limits {MU_MIN_DEG:g}..{MU_MAX_DEG:g})")
    print("=" * 52)
    return ok and mu_ok


def to_fusion_dict(m, frames, passed):
    """Data used by the Fusion builder: link lengths + start position.
    Start: 45 degrees past the extended position (the exactly collinear pose is hard to read)."""
    f0 = frames[len(frames) // 8]
    return {
        "type": "crank_rocker",
        "passed_checks": passed,
        "lengths_mm": {k: round(m[k], 3) for k in ("a", "b", "c", "d")},
        "points_mm": {
            "O2": [round(v, 3) for v in m["o2"]],
            "O4": [0.0, 0.0],
            "A": [round(float(v), 3) for v in f0["A"]],
            "B": [round(float(v), 3) for v in f0["B"]],
        },
    }


def export_json(m, frames, passed, path=None):
    """File read by the Fusion script (output/mechanism.json)."""
    import json
    import os

    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "mechanism.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = to_fusion_dict(m, frames, passed)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print(f"  Saved: {path}")


def animate(m, frames, save_path=None):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    o2 = m["o2"]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 5.5))
    xs = [o2[0], 0] + [f["A"][0] for f in frames] + [f["B"][0] for f in frames]
    ys = [o2[1], 0] + [f["A"][1] for f in frames] + [f["B"][1] for f in frames]
    pad = 0.15 * (max(xs) - min(xs))
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect("equal")
    ax.set_title("Four-bar linkage")
    ax.plot([o2[0], 0], [o2[1], 0], "k--", lw=1)
    ax.plot([f["B"][0] for f in frames], [f["B"][1] for f in frames], ":", color="gray")
    ax.plot(*o2, "ks", ms=8)
    ax.plot(0, 0, "ks", ms=8)
    crank, = ax.plot([], [], "-o", lw=4, color="tab:red", label="crank a")
    coupler, = ax.plot([], [], "-o", lw=4, color="tab:blue", label="coupler b")
    rocker, = ax.plot([], [], "-o", lw=4, color="tab:green", label="rocker c")
    ax.legend(loc="upper left")

    curve = sorted((f["theta"], f["mu"]) for f in frames)
    ax2.plot([p[0] for p in curve], [p[1] for p in curve], color="tab:purple")
    ax2.axhspan(MU_MIN_DEG, MU_MAX_DEG, color="tab:green", alpha=0.1, label="safe band")
    ax2.set_xlabel("Crank angle (deg)")
    ax2.set_ylabel("Transmission angle mu (deg)")
    ax2.set_xlim(0, 360)
    ax2.set_ylim(0, 180)
    ax2.legend()
    marker, = ax2.plot([], [], "o", color="tab:purple")

    def update(i):
        f = frames[i]
        A, B = f["A"], f["B"]
        crank.set_data([o2[0], A[0]], [o2[1], A[1]])
        coupler.set_data([A[0], B[0]], [A[1], B[1]])
        rocker.set_data([0, B[0]], [0, B[1]])
        marker.set_data([f["theta"]], [f["mu"]])
        return crank, coupler, rocker, marker

    anim = FuncAnimation(fig, update, frames=len(frames), interval=20, blit=True)
    fig.tight_layout()
    if save_path:
        update(len(frames) // 8)
        fig.savefig(save_path, dpi=90)
    else:
        plt.show()
    return anim


if __name__ == "__main__":
    # Usage: python kinematics.py [crank_mm] [swing_deg] [Q] [--png out.png]
    argv = sys.argv[1:]
    if "--png" in argv:
        i = argv.index("--png")
        argv = argv[:i] + argv[i + 2:]
    args = [x for x in argv if not x.startswith("--")]
    a = float(args[0]) if len(args) > 0 else 20.0
    psi = float(args[1]) if len(args) > 1 else 60.0
    png = sys.argv[sys.argv.index("--png") + 1] if "--png" in sys.argv else None

    q = float(args[2]) if len(args) > 2 else 1.0
    mech = synthesize_quick_return(a, psi, q)
    frames = simulate(mech)
    passed = report(mech, frames, psi)
    export_json(mech, frames, passed)
    animate(mech, frames, save_path=png)
