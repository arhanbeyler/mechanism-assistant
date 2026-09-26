"""
H5a: Draws a flywheel in Fusion 360 (runs inside Fusion only): a steel disk on a shaft between two
pedestals on a base plate; the disk + shaft turn on the joint Ground-Driver (the add-in turns it).
"""

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

PLATE_T = 10.0
CLEAR = 10.0


def _cyl(comp, r, z0, h, name, x=0.0, y=0.0):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z0))), mm(r))
    body = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(h)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def _block(comp, x0, y0, x1, y1, z0, h, name):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    to = lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z0)))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(to(x0, y0), to(x1, y1))
    body = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(h)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def horizontal(x, y, z):
    """Local z (the axis) -> global x; local x -> global y; local y -> global z; origin at (x, y, z)."""
    mt = adsk.core.Matrix3D.create()
    for col, v in enumerate(((0, 1, 0), (0, 0, 1), (1, 0, 0))):
        for row in range(3):
            mt.setCell(row, col, v[row])
    mt.translation = adsk.core.Vector3D.create(mm(x), mm(y), mm(z))
    return mt


def pedestals(root, H, span, width, bore):
    """Ground: base plate and two pedestals with bores at height H, at local axis positions -span/2 and +span/2."""
    g = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = g.component
    c.name = "Ground"
    _block(c, -span / 2 - 30, -width / 2, span / 2 + 30, width / 2, 0, PLATE_T, "Base")
    for x in (-span / 2, span / 2):
        body = _block(c, x - 10, -bore * 1.5, x + 10, bore * 1.5, PLATE_T, H - PLATE_T + bore * 1.5, "Pedestal")
        pin = c.constructionPlanes.createInput()
        pin.setByOffset(c.yZConstructionPlane, adsk.core.ValueInput.createByReal(mm(x - 10)))
        hole = c.sketches.add(c.constructionPlanes.add(pin))
        hole.sketchCurves.sketchCircles.addByCenterRadius(
            hole.modelToSketchSpace(adsk.core.Point3D.create(mm(x - 10), 0, mm(H))), mm(bore / 2 + 0.5))
        ext = c.features.extrudeFeatures
        fi = ext.createInput(hole.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
        fi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(20)))
        fi.participantBodies = [body]
        ext.add(fi)
    g.isGrounded = True
    return g


def revolute(root, occ, ground, body, r, name):
    edge = next(e for e in body.edges if isinstance(e.geometry, adsk.core.Circle3D) and abs(e.geometry.radius - mm(r)) < 1e-4)
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(occ), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(occ, ground, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def build(g, root, design=None):
    D, t, bore = g["D"], g["t"], g["bore"]
    H = PLATE_T + D / 2 + CLEAR
    span = t + 80
    ground = pedestals(root, H, span, D + 20, bore)
    occ = root.occurrences.addNewComponent(horizontal(0, 0, H))
    c = occ.component
    c.name = "Flywheel"
    disk = _cyl(c, D / 2, -t / 2, t, f"Flywheel Ø{D:.0f}x{t:.0f}")
    shaft = _cyl(c, bore / 2, -span / 2 - 20, span + 40, "Shaft")
    revolute(root, occ, ground, shaft, bore / 2, "Ground-Driver")
    return occ
