"""
F4d: Builds the single-dwell six-bar in Fusion 360 (runs inside Fusion only).

Links (fusion_lib.fourbar.make_link, one component each, in layers): ground bars O2-O4 and O2-O6 (grounded),
crank O2-A, the coupler as one straight three-hole bar through A, B and P (its ends are the outer two), rocker
O4-B, link 5 P-D and the output O6-D. Joints: Ground-Crank (O2) (the add-in's input), Crank-Coupler (A),
Coupler-Rocker (B), Rocker-Ground (O4), Coupler-Link 5 (P), Link 5-Output (D), Output-Ground 2 (O6).
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import GAP, hole_edge, make_link

SHAFTS = {"O2", "O4", "O6"}


def build(g, root, design=None):
    pts, section = g["points_mm"], g["section_mm"]
    d_pin, d_shaft = section["hole"], section.get("shaft", section["hole"])
    c0, c1 = g["coupler_ends"]
    mid = next(q for q in ("A", "B", "P") if q not in (c0, c1))
    dist = lambda u, v: math.hypot(pts[u][0] - pts[v][0], pts[u][1] - pts[v][1])
    links = [("Ground", "O2", "O4"), ("Ground 2", "O2", "O6"), ("Crank", "O2", "A"), ("Coupler", c0, c1),
             ("Rocker", "O4", "B"), ("Link 5", "P", "D"), ("Output", "O6", "D")]
    occs, secs, ends, z = {}, {}, {}, 0.0
    for name, s, e in links:
        base = section["links"].get(name.split(" ")[0], section["links"]["Coupler"])
        sec = dict(base, hole=d_pin, hole0=d_shaft if s in SHAFTS else d_pin, hole1=d_shaft if e in SHAFTS else d_pin)
        if name == "Coupler":
            sec["mid_holes"] = [dist(c0, mid)]
        occs[name], length = make_link(root, name, pts[s], pts[e], z, sec)
        occs[name].component.name = name
        secs[name], ends[name] = sec, (s, e, length)
        z += sec["thickness"] + GAP
    occs["Ground"].isGrounded = occs["Ground 2"].isGrounded = True

    def edge(link, point):
        s, e, length = ends[link]
        x = dist(s, point) if link == "Coupler" else (0.0 if point == s else length)
        return hole_edge(occs[link], x, secs[link])

    for l1, l2, p in [("Ground", "Crank", "O2"), ("Crank", "Coupler", "A"), ("Coupler", "Rocker", "B"),
                      ("Rocker", "Ground", "O4"), ("Coupler", "Link 5", "P"), ("Link 5", "Output", "D"),
                      ("Output", "Ground 2", "O6")]:
        geo = adsk.fusion.JointGeometry.createByCurve(edge(l1, p), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(occs[l1], occs[l2], geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        root.asBuiltJoints.add(ji).name = f"{l1}-{l2} ({p})"
    return occs
