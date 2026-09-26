"""
F2: Builds the Hoeckens straight-line linkage in Fusion 360 (runs inside Fusion only).

Same parts as the four-bar (fusion_lib.fourbar: links as components in z layers, as-built revolute joints),
except that the coupler runs from A to the coupler point P with a middle hole at B (AB = b). The path of P
is drawn as a closed spline above the links, with its straight part as a line, so the crank's
"1 turn" button shows the point riding along it.
"""

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import GAP, SHAFT_POINTS, hole_edge, make_link, mm

LINKS = {"Ground": ("O2", "O4"), "Crank": ("O2", "A"), "Coupler": ("A", "P"), "Rocker": ("O4", "B")}
JOINTS = [("Ground", "Crank", "O2"), ("Crank", "Coupler", "A"), ("Coupler", "Rocker", "B"), ("Rocker", "Ground", "O4")]


def _path_sketch(root, mech, z):
    """Coupler-point path (closed fitted spline) and its straight part (line) at height z."""
    P = adsk.core.Point3D.create
    pin = root.constructionPlanes.createInput()
    pin.setByOffset(root.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    plane = root.constructionPlanes.add(pin)
    plane.name = "Path plane"
    plane.isLightBulbOn = False
    sk = root.sketches.add(plane)
    sk.name = "Coupler-point path"
    to = lambda p: sk.modelToSketchSpace(P(mm(p[0]), mm(p[1]), mm(z)))
    pts = adsk.core.ObjectCollection.create()
    for p in mech["path"]:
        pts.add(to(p))
    spline = sk.sketchCurves.sketchFittedSplines.add(pts)
    spline.isClosed = True
    a, b = mech["line"]
    sk.sketchCurves.sketchLines.addByTwoPoints(to(a), to(b))
    sk.arePointsShown = False                  # the spline's fit points and the closed-profile shading
    sk.areProfilesShown = False
    return sk


def build(mech, root, design=None):
    pts = mech["points_mm"]
    L = mech["lengths_mm"]
    section = mech["section_mm"]
    d_pin, d_shaft = section["hole"], section.get("shaft", section["hole"])
    hole_at = lambda p: d_shaft if p in SHAFT_POINTS else d_pin
    secs = {n: dict(section["links"][n], hole=d_pin, hole0=hole_at(s), hole1=hole_at(e))
            for n, (s, e) in LINKS.items()}
    secs["Coupler"]["mid_holes"] = [L["b"]]                 # B, between A (x = 0) and P (x = p)

    occs, lengths, z, z_path = {}, {}, 0.0, 0.0
    for name in mech["layer_order"]:
        s, e = LINKS[name]
        occs[name], lengths[name] = make_link(root, name, pts[s], pts[e], z, secs[name])
        z += secs[name]["thickness"] + GAP
        if name == "Coupler":
            z_path = z - GAP / 2                  # the path on top of the coupler's layer, where P is
    occs["Ground"].isGrounded = True

    def local_x(link, point):
        if link == "Coupler" and point == "B":
            return L["b"]
        return 0.0 if point == LINKS[link][0] else lengths[link]

    for l1, l2, p in JOINTS:
        geo = adsk.fusion.JointGeometry.createByCurve(hole_edge(occs[l1], local_x(l1, p), secs[l1]),
                                                      adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(occs[l1], occs[l2], geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        root.asBuiltJoints.add(ji).name = f"{l1}-{l2} ({p})"
    _path_sketch(root, mech, z_path)
    return occs
