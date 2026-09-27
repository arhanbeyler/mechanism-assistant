"""
B8c (preparation): print-in-place clearance coupon, to measure the clearance a given FDM printer needs before
the four-bar's pins are printed in place (D12).

Each spinner is a pin and a ring printed together, both standing on the bed, no supports: the pin has 45° cones
at its foot and under its head, the ring's hole follows them at a normal clearance c (so c on the cylinder and
c * sqrt(2) across on the cones).  The ring is captured, and turns only if the printer kept the gap open.
The smallest c whose ring turns freely is the printer's print-in-place clearance.  Units: mm.

Profiles are (r, z) polygons revolved about the z axis.
"""

import math

CLEARANCES = [0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5]
D_PIN = 6.0          # pin diameter
CONE = 1.5           # cone height (45°)
H = 8.0              # ring height (the pin's head starts here)
KNOB = 4.0           # the pin's knob above the ring, to hold it
WALL = 3.0           # ring wall outside the widest part of its hole
S2 = math.sqrt(2.0)


def pin_profile():
    r = D_PIN / 2
    return [(0.0, 0.0), (r + CONE, 0.0), (r, CONE), (r, H - CONE), (r + CONE, H), (r + CONE, H + KNOB), (0.0, H + KNOB)]


def ring_profile(c):
    """The ring's section: its hole is the pin's outline offset by c (normal to each face)."""
    r = D_PIN / 2
    z_lo = CONE + c * (S2 - 1)              # where the offset foot cone meets the offset cylinder
    z_hi = H - CONE - c * (S2 - 1)
    r_out = r + CONE + c * S2 + WALL
    return [(r + CONE + c * S2, 0.0), (r_out, 0.0), (r_out, H), (r + CONE + c * S2, H), (r + c, z_hi), (r + c, z_lo)]


def _seg_dist(p, a, b):
    ax, az = a
    bx, bz = b
    dx, dz = bx - ax, bz - az
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - az) * dz) / (dx * dx + dz * dz)))
    return math.hypot(p[0] - ax - t * dx, p[1] - az - t * dz)


def min_gap(c, n=40):
    """Smallest distance between the pin's and the ring's outlines (sampled), in the section plane."""
    P, R = pin_profile(), ring_profile(c)
    best = float("inf")
    for A, B in ((P, R), (R, P)):
        for i in range(len(A)):
            a, b = A[i], A[(i + 1) % len(A)]
            for k in range(n + 1):
                q = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                best = min(best, min(_seg_dist(q, B[j], B[(j + 1) % len(B)]) for j in range(len(B))))
    return best


def captured(c):
    """The ring cannot come off: its narrowest hole is smaller than the pin's foot and head."""
    return D_PIN / 2 + c < D_PIN / 2 + CONE


def coupon(clearances=CLEARANCES, pitch=None):
    """Spinners in a row: [{c, x, pin, ring}], x the centre on the bed."""
    pitch = pitch or 2 * (D_PIN / 2 + CONE + max(clearances) * S2 + WALL) + 8.0
    return {"type": "clearance_coupon", "spinners": [{"c": c, "x": i * pitch, "pin": pin_profile(), "ring": ring_profile(c)}
                                                     for i, c in enumerate(clearances)]}
