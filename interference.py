"""
Katman cakismasi kontrolu (B8b).

Uzuvlar ust uste katmanlarda durur. Bir mafsal iki KOMSU olmayan katmani baglarsa
(ornegin sabit uzuv katman 0, kol katman 3: O4 mili), mil aradaki katmanlardan gecer.
Aradaki uzuvlar cevrim boyunca bu milin ustunden gecerse mekanizma kilitlenir.

Uzuv bolgesi: w genisliginde govde (iki uc arasi) + uclarda R = D/2 yaricapli gobek.
Nokta-bolge mesafesi = min(serit mesafesi - w/2, uc merkezi mesafesi - R).
Mil etrafinda gereken bosluk: mil yaricapi + CLEARANCE.
"""

import math

from strength import LINK_ENDS, SHAFT_POINTS, section_with_bosses

CLEARANCE = 1.0          # mm, mil ile gecen uzuv arasinda en az bosluk

# Denenecek katman siralari (alttan uste). Ikisinde de A ve B pimleri komsu katmanlari baglar.
LAYER_ORDERS = [("Sabit", "Krank", "Biyel", "Kol"),     # O4 mili krank ve biyel katmanlarindan gecer
                ("Sabit", "Kol", "Biyel", "Krank")]     # O2 mili kol ve biyel katmanlarindan gecer

JOINTS = [("Sabit", "Krank", "O2"), ("Krank", "Biyel", "A"), ("Biyel", "Kol", "B"), ("Kol", "Sabit", "O4")]


def _seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    s = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + s * dx), p[1] - (ay + s * dy))


def region_distance(p, a, b, w, R0, R1):
    """p noktasinin uzuv bolgesine (govde + gobekler) mesafesi; negatif = icinde."""
    return min(_seg_dist(p, a, b) - w / 2, math.dist(p, a) - R0, math.dist(p, b) - R1)


def check_layers(mech, frames, section, order):
    """Verilen katman sirasi icin, ara katmanlardan gecen her milin en kotu boslugu.
    Dondurur: [(mafsal, gecilen uzuvlar, en kucuk bosluk mm)]."""
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
    """Cakismasiz ilk katman sirasi. Dondurur: (sira, bosluklar, gecti_mi)."""
    best = None
    for order in LAYER_ORDERS:
        res = check_layers(mech, frames, section, order)
        margin = min((r[2] for r in res), default=float("inf"))
        if margin >= 0:
            return order, res, True
        if best is None or margin > best[2]:
            best = (order, res, margin)
    return best[0], best[1], False
