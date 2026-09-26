"""
F1: Builds the slider-crank (piston) mechanism in Fusion 360 (runs inside Fusion only).

Frame as in slider_crank.py: crank pivot O2 at the origin, slide axis along +x at y = e.
Layers (z): base plate below z = 0 | crank | connecting rod | piston + guide rails.
Joints: Ground-Crank (O2) revolute, Crank-Coupler (A) revolute, Coupler-Piston (B) revolute, and a
Ground-Piston slider along the slide axis. The slider direction is checked with a trial slide (a custom
direction was once ignored by Fusion, see fusion_lib/cam.py). The add-in's turn buttons drive Ground-Crank;
Fusion's solver moves the rod and the piston.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import GAP, hole_edge, make_link, mm

PLATE_T = 3.0
RAIL_W = 4.0
CLEAR = 0.5


def _box(comp, x0, y0, x1, y1, z0, h, name, op=adsk.fusion.FeatureOperations.NewBodyFeatureOperation):
    P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    to = lambda x, y: sk.modelToSketchSpace(P(mm(x), mm(y), mm(z0)))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(to(x0, y0), to(x1, y1))
    body = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(h)),
                                                   op).bodies.item(0)
    body.name = name
    return body


def _hole(comp, body, x, y, z0, d, h):
    P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(mm(x), mm(y), mm(z0))), mm(d / 2))
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(h)))
    inp.participantBodies = [body]
    ext.add(inp)


def build(g, root, design=None):
    """g: the pipeline's fusion dict (a, b, e, x_min, x_max, points_mm, section_mm)."""
    a, b, e = g["a"], g["b"], g["e"]
    pts, section = g["points_mm"], g["section_mm"]
    d_pin, d_shaft = section["hole"], section.get("shaft", section["hole"])
    crank_sec = dict(section["links"]["Crank"], hole=d_pin, hole0=d_shaft, hole1=d_pin)
    rod_sec = dict(section["links"]["Coupler"], hole=d_pin, hole0=d_pin, hole1=d_pin)
    tc, tr = crank_sec["thickness"], rod_sec["thickness"]
    z1 = GAP
    z2 = z1 + tc + GAP
    z3 = z2 + tr + GAP
    Lb = max(12.0, 2.0 * rod_sec["boss1"])          # piston block length (x), height (y), thickness (z)
    H = max(10.0, 1.2 * rod_sec["boss1"])
    tp = max(6.0, tr)

    # ---- ground: base plate (O2 hole), guide rails around the piston, a post carrying the rails
    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    x_r0, x_r1 = g["x_min"] - Lb / 2 - 2.0, g["x_max"] + Lb / 2 + 3.0
    y_in = H / 2 + CLEAR
    R = a + crank_sec.get("boss0", crank_sec["width"]) / 2 + 5.0
    plate = _box(gc, -R, min(-R, e - y_in - RAIL_W - 5), x_r1 + 6 + 5, max(R, e + y_in + RAIL_W + 5), -PLATE_T, PLATE_T, "Base")
    _hole(gc, plate, 0.0, 0.0, -PLATE_T, d_shaft, PLATE_T)
    for s in (1, -1):
        y0, y1 = sorted((e + s * y_in, e + s * (y_in + RAIL_W)))
        _box(gc, x_r0, y0, x_r1, y1, z3, tp, f"Rail {'+' if s > 0 else '-'}")
    _box(gc, x_r1, e - y_in - RAIL_W, x_r1 + 6, e + y_in + RAIL_W, 0.0, z3 + tp, "Rail post")
    ground.isGrounded = True

    # ---- moving parts
    crank, _ = make_link(root, "Crank", pts["O2"], pts["A"], z1, crank_sec)
    rod, _ = make_link(root, "Coupler", pts["A"], pts["B"], z2, rod_sec)
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(mm(pts["B"][0]), mm(pts["B"][1]), mm(z3))
    piston = root.occurrences.addNewComponent(m)
    pc = piston.component
    pc.name = "Piston"
    block = _box(pc, -Lb / 2, -H / 2, Lb / 2, H / 2, 0.0, tp, "Piston")
    _hole(pc, block, 0.0, 0.0, 0.0, d_pin, tp)

    # ---- joints
    def revolute(o1, o2, edge, name):
        geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(o1, o2, geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        j.name = name
        return j

    revolute(ground, crank, hole_edge(crank, 0.0, crank_sec), "Ground-Crank (O2)")
    revolute(crank, rod, hole_edge(crank, a, crank_sec), "Crank-Coupler (A)")
    revolute(rod, piston, hole_edge(rod, b, rod_sec), "Coupler-Piston (B)")

    pin_edge = next(ed for ed in block.edges if isinstance(ed.geometry, (adsk.core.Circle3D, adsk.core.Arc3D))
                    and abs(ed.geometry.radius - mm(d_pin / 2)) < 1e-4 and abs(ed.geometry.center.z - mm(tp)) < 1e-4)
    for direction in (adsk.fusion.JointDirections.XAxisJointDirection, adsk.fusion.JointDirections.YAxisJointDirection):
        geo = adsk.fusion.JointGeometry.createByCurve(pin_edge.createForAssemblyContext(piston),
                                                      adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(ground, piston, geo)
        ji.setAsSliderJointMotion(direction)
        slider = root.asBuiltJoints.add(ji)
        slider.name = "Ground-Piston"
        # does a positive slide move the piston along world x? (trial slide, undone at once)
        motion = adsk.fusion.SliderJointMotion.cast(slider.jointMotion)
        crank_joint = [j for j in root.asBuiltJoints if j.name == "Ground-Crank (O2)"][0]
        crank_joint.isSuppressed = True            # free the loop for the trial
        p0 = piston.transform2.translation
        motion.slideValue = 0.05
        p1 = piston.transform2.translation
        motion.slideValue = 0.0
        crank_joint.isSuppressed = False
        if abs(p1.x - p0.x) > 0.04 and abs(p1.y - p0.y) < 1e-4:
            return {"crank": crank, "piston": piston}
        slider.deleteMe()
    raise RuntimeError("The piston slider joint does not move along the slide axis")
