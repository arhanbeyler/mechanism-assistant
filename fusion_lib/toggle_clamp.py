"""
F5: Builds the toggle clamp in Fusion 360 (runs inside Fusion only).

Layers (bottom to top): base plate (holds O2), handle, coupler, arm, top base plate (holds O4), so every
pin joins neighbouring layers; a post ties the two plates outside the links' sweep (toggle.spacer). The
handle is one link O2 -> grip with the coupler pin A in between; the arm runs O4 -> spindle with B in
between. Drawn locked (on the stop); the handle joint is limited to the stroke, so the add-in's buttons
open and close the clamp.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import GAP, hole_edge, make_link, mm

STOP_R = 4.0         # mm, stop post radius
SPINDLE_LEN = 15.0   # mm, pressing spindle beyond the arm


def _plate_sketch(comp, z, pts, w, name):
    """Obround bars between consecutive points (one outline per bar) on a plane at height z."""
    P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    plane = comp.constructionPlanes.add(pin)
    plane.isLightBulbOn = False
    sk = comp.sketches.add(plane)
    sk.name = name
    to = lambda x, y: sk.modelToSketchSpace(P(mm(x), mm(y), mm(z)))
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    for p0, p1 in zip(pts, pts[1:]):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        nx, ny = -uy * w / 2, ux * w / 2
        a = lines.addByTwoPoints(to(p0[0] + nx, p0[1] + ny), to(p1[0] + nx, p1[1] + ny))
        b = lines.addByTwoPoints(to(p0[0] - nx, p0[1] - ny), to(p1[0] - nx, p1[1] - ny))
        arcs.addByThreePoints(a.endSketchPoint, to(p1[0] + ux * w / 2, p1[1] + uy * w / 2), b.endSketchPoint)
        arcs.addByThreePoints(b.startSketchPoint, to(p0[0] - ux * w / 2, p0[1] - uy * w / 2), a.startSketchPoint)
    return sk, to


def _extrude_all(comp, sk, h, name):
    profs = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        profs.add(p)
    ext = comp.features.extrudeFeatures
    feat = ext.addSimple(profs, adsk.core.ValueInput.createByReal(mm(h)),
                         adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = feat.bodies.item(0)
    body.name = name
    return body


def _cylinder(comp, x, y, r, z0, h, name):
    P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    plane = comp.constructionPlanes.add(pin)
    plane.isLightBulbOn = False
    sk = comp.sketches.add(plane)
    sk.name = name
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(mm(x), mm(y), mm(z0))), mm(r))
    return _extrude_all(comp, sk, h, name)


def _limit(root, joint, occ, range_deg):
    """Joint limits over the stroke relative to the drawn (locked) pose; the joint's positive sense is
    measured with a small trial turn (as fourbar._limit_input_joint)."""
    motion = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    ang = lambda: math.atan2(occ.transform2.getCell(1, 0), occ.transform2.getCell(0, 0))
    a0 = ang()
    motion.rotationValue = math.radians(2)
    sign = 1.0 if math.remainder(ang() - a0, 2 * math.pi) > 0 else -1.0
    motion.rotationValue = 0.0
    lo, hi = sorted(sign * math.radians(v) for v in range_deg)
    lim = motion.rotationLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = lo
    lim.isMaximumValueEnabled = True
    lim.maximumValue = hi


def build(mech, root, design=None):
    p, L, sec = mech["points_mm"], mech["lengths_mm"], mech["section_mm"]
    d = sec["hole"]
    lk = lambda n, **kw: dict(sec["links"][n], hole=d, hole0=d, hole1=d, **kw)
    s_handle = lk("Crank", mid_holes=[L["a"]])
    s_coupler = lk("Coupler")
    s_arm = lk("Rocker", mid_holes=[L["c"]])
    t = {n: sec["links"][n]["thickness"] for n in sec["links"]}
    z_h = t["Ground"] + GAP
    z_c = z_h + t["Crank"] + GAP
    z_a = z_c + t["Coupler"] + GAP
    z_top = z_a + t["Rocker"] + GAP

    base = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    bc = base.component
    bc.name = "Base"
    wb = max(sec["links"]["Ground"]["width"], 2.5 * d)
    post, r_post = mech["post"], mech["post_r"]
    near = min((p["O2"], p["O4"]), key=lambda q: math.dist(q, post))
    far = p["O4"] if near is p["O2"] else p["O2"]
    chain = [far, near, post]
    sk, _ = _plate_sketch(bc, 0.0, chain, wb, "Bottom plate")
    _extrude_all(bc, sk, t["Ground"], "Bottom plate")
    sk, _ = _plate_sketch(bc, z_top, chain, wb, "Top plate")
    _extrude_all(bc, sk, t["Ground"], "Base top plate").opacity = 0.35        # see the linkage through it
    _cylinder(bc, post[0], post[1], r_post, t["Ground"], z_top - t["Ground"], "Post")
    # stop: just beyond the locked handle, on the side it was moving to
    O2, A = p["O2"], p["A"]
    th_l = math.atan2(A[1] - O2[1], A[0] - O2[0])
    sgn = 1.0 if mech["input_range_deg"][0] < 0 else -1.0       # the stroke ran towards the locked pose
    r_s = L["a"] + (L["L_handle"] - L["a"]) * 0.3
    off = (sec["links"]["Crank"]["width"] / 2 + STOP_R + 0.2) / r_s
    ang = th_l + sgn * off
    stop = (O2[0] + r_s * math.cos(ang), O2[1] + r_s * math.sin(ang))
    _cylinder(bc, stop[0], stop[1], STOP_R, t["Ground"], z_c - GAP - t["Ground"], "Stop")
    base.isGrounded = True

    handle, _ = make_link(root, "Handle", p["O2"], p["H"], z_h, s_handle)
    coupler, _ = make_link(root, "Coupler", p["A"], p["B"], z_c, s_coupler)
    arm, _ = make_link(root, "Arm", p["O4"], p["S"], z_a, s_arm)
    # spindle: in the arm's plane, square to the arm, pointing the way the arm closes (where the work is)
    sp = (L["L_arm"], mech["spindle_dir"] * SPINDLE_LEN)
    sk, _ = _plate_sketch(arm.component, 0.0, [(L["L_arm"], 0.0), sp], 0.8 * sec["links"]["Rocker"]["width"],
                          "Spindle")
    _extrude_all(arm.component, sk, t["Rocker"], "Spindle")

    joints = []
    for occ, x, other, name in ((handle, 0.0, base, "Ground-Handle (O2, limited)"),
                                (handle, L["a"], coupler, "Handle-Coupler (A)"),
                                (arm, L["c"], coupler, "Arm-Coupler (B)"),
                                (arm, 0.0, base, "Arm-Ground (O4)")):
        s_ = s_handle if occ is handle else s_arm
        geo = adsk.fusion.JointGeometry.createByCurve(hole_edge(occ, x, s_), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(occ, other, geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        j.name = name
        joints.append(j)
    _limit(root, joints[0], handle, mech["input_range_deg"])
    return handle
