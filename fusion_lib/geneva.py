"""
F3: Builds the external Geneva mechanism in Fusion 360 (runs inside Fusion only).

Frame as in geneva.py: driver axis at the origin, wheel axis at (C, 0). Drawn pose: driver at 180° (mid-dwell,
pin and relief facing away from the wheel), wheel at rest with a locking arc facing the driver.
Layers: wheel, locking disk and the lower part of the pin in z = 0..t; the crank plate above (t + gap ..);
a base plate below. The wheel's motion is intermittent, which no motion link can express: the add-in
turns the wheel from the table the engine wrote (design attribute "geneva_table": wheel angle per degree
of driver turn), with the joints' measured positive senses ("geneva_signs").
"""

import json
import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import MARK_GROUP, mm

BASE_T = 4.0         # mm, base plate
MARGIN = 10.0        # mm, base plate beyond the parts


def _plane(comp, z):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    pl = comp.constructionPlanes.add(pin)
    pl.isLightBulbOn = False
    return pl


def _extrude(comp, sk, h, op, body=None, name=None):
    profs = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        profs.add(p)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(h)))
    if body is not None:
        inp.participantBodies = [body]           # a cut must touch only this body
    feat = ext.add(inp)
    if name and op == adsk.fusion.FeatureOperations.NewBodyFeatureOperation:
        feat.bodies.item(0).name = name
    return feat.bodies.item(0) if feat.bodies.count else None


def _sketch(comp, z, name):
    sk = comp.sketches.add(_plane(comp, z))
    sk.name = name
    return sk, (lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z))))


def _cylinder(comp, x, y, r, z0, h, name):
    sk, to = _sketch(comp, z0, name)
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(x, y), mm(r))
    return _extrude(comp, sk, h, adsk.fusion.FeatureOperations.NewBodyFeatureOperation, name=name)


def _obround(sk, to, p0, p1, w):
    """Slot outline between centres p0 and p1 (width w) in the sketch."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    nx, ny = -uy * w / 2, ux * w / 2
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    a = lines.addByTwoPoints(to(p0[0] + nx, p0[1] + ny), to(p1[0] + nx, p1[1] + ny))
    b = lines.addByTwoPoints(to(p0[0] - nx, p0[1] - ny), to(p1[0] - nx, p1[1] - ny))
    arcs.addByThreePoints(a.endSketchPoint, to(p1[0] + ux * w / 2, p1[1] + uy * w / 2), b.endSketchPoint)
    arcs.addByThreePoints(b.startSketchPoint, to(p0[0] - ux * w / 2, p0[1] - uy * w / 2), a.startSketchPoint)


def _joint(root, ground, occ, body, name):
    """Revolute joint about the axis of a vertical cylinder body (centre of its bottom circle)."""
    edge = min((e for e in body.edges if isinstance(e.geometry, adsk.core.Circle3D)),
               key=lambda e: e.geometry.center.z)
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(occ),
                                                  adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(ground, occ, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def _angle(occ):
    t = occ.transform2
    return math.atan2(t.getCell(1, 0), t.getCell(0, 0))


def _sense(joint, occ):
    """+1 if a positive joint value turns the part counter-clockwise (seen from +z), else -1."""
    motion = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    a0 = _angle(occ)
    motion.rotationValue = math.radians(2)
    turned = math.remainder(_angle(occ) - a0, 2 * math.pi)
    motion.rotationValue = 0.0
    return 1 if turned > 0 else -1


def build(mech, root, design=None):
    g, t, gap = mech["geometry"], mech["t"], mech["gap"]
    n, C, a = g["n"], g["C"], g["a"]
    NEW, CUT = adsk.fusion.FeatureOperations.NewBodyFeatureOperation, adsk.fusion.FeatureOperations.CutFeatureOperation

    # base plate
    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Base"
    sk, to = _sketch(gc, -gap - BASE_T, "Base")
    ext = max(a + g["d_pin"], g["r_lock"]) + MARGIN
    sk.sketchCurves.sketchLines.addTwoPointRectangle(to(-ext, -max(ext, g["R_o"] + MARGIN)),
                                                     to(C + g["R_o"] + MARGIN, max(ext, g["R_o"] + MARGIN)))
    _extrude(gc, sk, BASE_T, NEW, name="Base plate")
    ground.isGrounded = True

    # driver: locking disk with the relief facing the pin (pin at 180°), crank plate above, pin, shaft
    drv = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    dc = drv.component
    dc.name = "Driver"
    sk, to = _sketch(dc, 0, "Locking disk")
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(0, 0), mm(g["r_lock"]))
    disk = _extrude(dc, sk, t, NEW, name="Locking disk")
    sk, to = _sketch(dc, 0, "Relief")
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(-C, 0), mm(g["relief_r"]))
    _extrude(dc, sk, t, CUT, body=disk)
    w = max(2.5 * g["d_pin"], mech["shaft_driver"] + 6)
    sk, to = _sketch(dc, t + gap, "Crank plate")
    _obround(sk, to, (0.0, 0.0), (-a, 0.0), w)
    _extrude(dc, sk, t, NEW, name="Crank plate")
    _cylinder(dc, -a, 0, g["d_pin"] / 2, 0, t + gap, f"Pin Ø{g['d_pin']:g}")
    d_shaft = _cylinder(dc, 0, 0, mech["shaft_driver"] / 2, -gap, 2 * t + 2 * gap, f"Shaft Ø{mech['shaft_driver']:g}")

    # wheel at (C, 0), rest position: slots at pi + pi/n + k 2pi/n, locking arcs at pi + k 2pi/n
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(mm(C), 0, 0)
    whl = root.occurrences.addNewComponent(m)
    wc = whl.component
    wc.name = "Wheel"
    sk, to = _sketch(wc, 0, "Wheel disk")
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(0, 0), mm(g["R_o"]))
    wheel = _extrude(wc, sk, t, NEW, name=f"Wheel {n} slots")
    sk, to = _sketch(wc, 0, "Slots")
    for k in range(n):
        be = math.pi + math.pi / n + k * 2 * math.pi / n
        u = (math.cos(be), math.sin(be))
        r0, r1 = g["slot_r_min"], g["R_o"] + g["slot_w"]
        _obround(sk, to, (r0 * u[0], r0 * u[1]), (r1 * u[0], r1 * u[1]), g["slot_w"])
    _extrude(wc, sk, t, CUT, body=wheel)
    sk, to = _sketch(wc, 0, "Locking arcs")
    for k in range(n):
        ga = math.pi + k * 2 * math.pi / n
        sk.sketchCurves.sketchCircles.addByCenterRadius(to(C * math.cos(ga), C * math.sin(ga)), mm(g["r_arc"]))
    _extrude(wc, sk, t, CUT, body=wheel)
    w_shaft = _cylinder(wc, 0, 0, mech["shaft_wheel"] / 2, -gap, t + gap, f"Shaft Ø{mech['shaft_wheel']:g}")

    jd = _joint(root, ground, drv, d_shaft, "Ground-Driver")
    jw = _joint(root, ground, whl, w_shaft, "Ground-Wheel")
    signs = (_sense(jd, drv), _sense(jw, whl))
    des = design or adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    des.attributes.add(MARK_GROUP, "geneva_table", json.dumps(mech["table"]))
    des.attributes.add(MARK_GROUP, "geneva_signs", json.dumps({"driver": signs[0], "wheel": signs[1],
                                                               "step": 360.0 / n}))
    return drv
