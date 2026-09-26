"""
F4b: Builds the Watt II six-bar in Fusion 360 (runs inside Fusion only).

Links as in the four-bar (fusion_lib.fourbar.make_link, one component each, stacked in layers): the ground
as two bars (O2-O4 and O4-O6, both grounded), crank, coupler, rocker, the rocker's second arm (a separate bar
held to the rocker by a rigid joint at O4: together the ternary link), coupler 2 and the output. Joints:
Ground-Crank (O2) (the add-in's input), Crank-Coupler (A), Coupler-Rocker (B), Rocker-Ground (O4),
Rocker-Rocker arm (rigid), Rocker arm-Coupler 2 (A2), Coupler 2-Output (B2), Output-Ground 2 (O6).
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import GAP, hole_edge, make_link

LINKS = [("Ground", "O2", "O4"), ("Ground 2", "O4", "O6"), ("Crank", "O2", "A"), ("Coupler", "A", "B"),
         ("Rocker", "O4", "B"), ("Rocker arm", "O4", "A2"), ("Coupler 2", "A2", "B2"), ("Output", "O6", "B2")]
JOINTS = [("Ground", "Crank", "O2"), ("Crank", "Coupler", "A"), ("Coupler", "Rocker", "B"), ("Rocker", "Ground", "O4"),
          ("Rocker arm", "Coupler 2", "A2"), ("Coupler 2", "Output", "B2"), ("Output", "Ground 2", "O6")]
SHAFTS = {"O2", "O4", "O6"}


def build(g, root, design=None):
    pts, section = g["points_mm"], g["section_mm"]
    d_pin, d_shaft = section["hole"], section.get("shaft", section["hole"])
    occs, lengths, z = {}, {}, 0.0
    for name, s, e in LINKS:
        base = section["links"].get(name.split(" ")[0], section["links"]["Coupler"])
        sec = dict(base, hole=d_pin, hole0=d_shaft if s in SHAFTS else d_pin, hole1=d_shaft if e in SHAFTS else d_pin)
        occs[name], lengths[name] = make_link(root, name, pts[s], pts[e], z, sec)
        occs[name].component.name = name
        occs[name + "#sec"] = sec
        z += sec["thickness"] + GAP
    for name in ("Ground", "Ground 2"):
        occs[name].isGrounded = True

    ends = {name: (s, e) for name, s, e in LINKS}

    def edge(link, point):
        s, e = ends[link]
        return hole_edge(occs[link], 0.0 if point == s else lengths[link], occs[link + "#sec"])

    for l1, l2, p in JOINTS:
        geo = adsk.fusion.JointGeometry.createByCurve(edge(l1, p), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(occs[l1], occs[l2], geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        root.asBuiltJoints.add(ji).name = f"{l1}-{l2} ({p})"
    geo = adsk.fusion.JointGeometry.createByCurve(edge("Rocker arm", "O4"), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(occs["Rocker arm"], occs["Rocker"], geo)
    ji.setAsRigidJointMotion()
    root.asBuiltJoints.add(ji).name = "Rocker-Rocker arm (rigid)"
    return occs
