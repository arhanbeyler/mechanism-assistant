"""
F6: Builds the ratchet wheel and pawl in Fusion 360 (runs inside Fusion only).

Frame as in ratchet.py: wheel centre at the origin, the pawl in the tooth space of tooth 0 (tip on +x), both
in the same layer (z = 0 .. b) on a base plate. The pawl rides the teeth when the wheel turns its free way;
no joint can express that, so the add-in turns the pawl from the engine's lift table (design attributes
"geneva_table" / "geneva_signs" with follower Ground-Pawl, step 0 = oscillating, forward = free direction).
"""

import json
import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import MARK_GROUP, mm
from fusion_lib.slider_crank import PLATE_T, _box, _hole

GAP = 0.5


def _sketch(comp, z):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    pl = comp.constructionPlanes.add(pin)
    pl.isLightBulbOn = False
    sk = comp.sketches.add(pl)
    return sk, (lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z))))


def _polygon(sk, to, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to(*pts[0]), to(*pts[1]))
    prev = first.endSketchPoint
    for p in pts[2:]:
        prev = lines.addByTwoPoints(prev, to(*p)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def _extrude_new(comp, sk, h, name):
    profs = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        profs.add(p)
    body = comp.features.extrudeFeatures.addSimple(profs, adsk.core.ValueInput.createByReal(mm(h)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def _obround(sk, to, p0, p1, w):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    nx, ny = -uy * w / 2, ux * w / 2
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    a = lines.addByTwoPoints(to(p0[0] + nx, p0[1] + ny), to(p1[0] + nx, p1[1] + ny))
    b = lines.addByTwoPoints(to(p0[0] - nx, p0[1] - ny), to(p1[0] - nx, p1[1] - ny))
    arcs.addByThreePoints(a.endSketchPoint, to(p1[0] + ux * w / 2, p1[1] + uy * w / 2), b.endSketchPoint)
    arcs.addByThreePoints(b.startSketchPoint, to(p0[0] - ux * w / 2, p0[1] - uy * w / 2), a.startSketchPoint)


def _revolute(root, ground, occ, body, x, y, name):
    edge = min((e for e in body.edges if isinstance(e.geometry, adsk.core.Circle3D)),
               key=lambda e: math.hypot(e.geometry.center.x - mm(x), e.geometry.center.y - mm(y)) + e.geometry.center.z)
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(occ),
                                                  adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(occ, ground, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def _sense(joint, occ):
    mo = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    ang = lambda: math.atan2(occ.transform2.getCell(1, 0), occ.transform2.getCell(0, 0))
    a0 = ang()
    mo.rotationValue = math.radians(2)
    s = 1 if math.remainder(ang() - a0, 2 * math.pi) > 0 else -1
    mo.rotationValue = 0.0
    return s


def build(mech, root, design=None):
    b, R, wp = mech["b"], mech["R"], mech["w_pawl"]
    pv, top = mech["pivot"], mech["top"]
    d_pin, d_sh = mech["d_pin"], mech["d_shaft"]
    boss = max(wp, 2.5 * d_pin)

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    ext = R + boss + 10
    plate = _box(gc, -ext, min(-ext, pv[1] - boss), ext + max(0.0, pv[0] - R) + boss, ext, -PLATE_T - GAP, PLATE_T, "Base")
    _hole(gc, plate, 0.0, 0.0, -PLATE_T - GAP, d_sh, PLATE_T)
    ground.isGrounded = True

    wheel = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    wc = wheel.component
    wc.name = "Ratchet"
    sk, to = _sketch(wc, 0.0)
    _polygon(sk, to, mech["outline"])
    wb = _extrude_new(wc, sk, b, f"Ratchet {len(mech['outline']) // 2} teeth")
    _hole(wc, wb, 0.0, 0.0, 0.0, d_sh, b)

    pawl = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    pc = pawl.component
    pc.name = "Pawl"
    sk, to = _sketch(pc, 0.0)
    _polygon(sk, to, mech["head"])
    _obround(sk, to, tuple(top), tuple(pv), min(wp, boss))
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(*pv), mm(boss / 2))
    pb = _extrude_new(pc, sk, b, "Pawl")
    _hole(pc, pb, pv[0], pv[1], 0.0, d_pin, b)

    jw = _revolute(root, ground, wheel, wb, 0.0, 0.0, "Ground-Ratchet")
    jp = _revolute(root, ground, pawl, pb, pv[0], pv[1], "Ground-Pawl")
    s_w, s_p = _sense(jw, wheel), _sense(jp, pawl)
    des = design or adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    table = [mech["lift_sign"] * v for v in mech["table"]]          # pawl turn (world, ccw +) per degree of wheel
    des.attributes.add(MARK_GROUP, "geneva_table", json.dumps(table))
    des.attributes.add(MARK_GROUP, "geneva_signs", json.dumps({"driver": s_w, "wheel": s_p, "step": 0,
                                                               "follower": "Ground-Pawl", "forward": s_w}))
    return wheel
