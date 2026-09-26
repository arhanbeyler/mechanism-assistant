"""
Layer interference check (B8b).

The links sit on top of each other in layers. If a joint connects two NON-adjacent layers
(e.g. ground on layer 0, rocker on layer 3: the O4 shaft), the shaft passes through the layers between.
If those links sweep over the shaft during the cycle, the mechanism jams.

Link region: a web of width w (between the two ends) + bosses of radius R = D/2 at the ends.
Point-to-region distance = min(strip distance - w/2, distance to an end centre - R).
Clearance needed around the shaft: shaft radius + CLEARANCE.
"""

import math

from strength import LINK_ENDS, SHAFT_POINTS, section_with_bosses

CLEARANCE = 1.0          # mm, smallest gap between the shaft and a passing link

# Layer orders to try (bottom to top). In both, the A and B pins connect adjacent layers.
LAYER_ORDERS = [("Ground", "Crank", "Coupler", "Rocker"),     # the O4 shaft passes the crank and coupler layers
                ("Ground", "Rocker", "Coupler", "Crank")]     # the O2 shaft passes the rocker and coupler layers

JOINTS = [("Ground", "Crank", "O2"), ("Crank", "Coupler", "A"), ("Coupler", "Rocker", "B"), ("Rocker", "Ground", "O4")]


def _seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    s = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + s * dx), p[1] - (ay + s * dy))


def region_distance(p, a, b, w, R0, R1):
    """Distance of point p to the link region (web + bosses); negative = inside."""
    return min(_seg_dist(p, a, b) - w / 2, math.dist(p, a) - R0, math.dist(p, b) - R1)


def check_layers(mech, frames, section, order):
    """For the given layer order, the worst clearance of every shaft passing through middle layers.
    Returns [(joint, links passed, smallest clearance mm)]."""
    sec = section_with_bosses(section)
    d_pin, d_shaft = sec["hole"], sec["shaft"]
    layer = {name: i for i, name in enumerate(order)}
    out = []
    for l1, l2, p in JOINTS:
        lo, hi = sorted((layer[l1], layer[l2]))
        between = [order[i] for i in range(lo + 1, hi)]
        if not between:
            continue
        r_need = (d_shaft if p in SHAFT_POINTS else d_pin) / 2 + CLEARANCE
        worst = float("inf")
        for fr in frames:
            pts = {"O2": tuple(mech["o2"]), "O4": (0.0, 0.0), "A": tuple(fr["A"]), "B": tuple(fr["B"])}
            for name in between:
                s = sec["links"][name]
                e0, e1 = LINK_ENDS[name]
                dist = region_distance(pts[p], pts[e0], pts[e1], s["width"], s["boss0"] / 2, s["boss1"] / 2)
                worst = min(worst, dist - r_need)
        out.append((p, between, worst))
    return out


def choose_layer_order(mech, frames, section):
    """First layer order without interference. Returns (order, clearances, passed)."""
    best = None
    for order in LAYER_ORDERS:
        res = check_layers(mech, frames, section, order)
        margin = min((r[2] for r in res), default=float("inf"))
        if margin >= 0:
            return order, res, True
        if best is None or margin > best[2]:
            best = (order, res, margin)
    return best[0], best[1], False
