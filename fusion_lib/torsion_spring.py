"""
H4: Draws a helical torsion spring in Fusion 360 (runs inside Fusion only).

Body: the wire's centre line as a helix spline (close wound, pitch 1.05 d so the sweep does not touch
itself), a circle swept along it; the two straight legs are extruded from the planes normal to the path at
its ends, along the helix tangent. With an arbor, a grey rod goes through the coils. Nothing moves.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

GAP = 1.05
PER_TURN = 24


def build(g, root, design=None):
    d, D, Nb, legs = g["d"], g["D"], g["Nb"], g["legs"]
    R = D / 2
    p = GAP * d
    P = adsk.core.Point3D.create
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Spring (torsion)"

    n = int(round(Nb * PER_TURN))
    col = adsk.core.ObjectCollection.create()
    for i in range(n + 1):
        a = 2 * math.pi * Nb * i / n
        col.add(P(mm(R * math.cos(a)), mm(R * math.sin(a)), mm(d / 2 + p * Nb * i / n)))
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "Wire axis (helix)"
    spline = sk.sketchCurves.sketchFittedSplines.add(col)
    path = comp.features.createPath(spline, False)

    def plane_at(t):
        pi = comp.constructionPlanes.createInput()
        pi.setByDistanceOnPath(path, adsk.core.ValueInput.createByReal(t))
        return comp.constructionPlanes.add(pi)

    start = plane_at(0.0)
    ps = comp.sketches.add(start)
    first = col.item(0)
    ps.sketchCurves.sketchCircles.addByCenterRadius(ps.modelToSketchSpace(first), mm(d / 2))
    si = comp.features.sweepFeatures.createInput(ps.profiles.item(0), path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = comp.features.sweepFeatures.add(si).bodies.item(0)
    body.name = f"Torsion spring d{d:g}"

    ext = comp.features.extrudeFeatures
    for t, pt, direction in ((0.0, first, adsk.fusion.ExtentDirections.NegativeExtentDirection),
                             (1.0, col.item(col.count - 1), adsk.fusion.ExtentDirections.PositiveExtentDirection)):
        pl = start if t == 0.0 else plane_at(1.0)
        lk = comp.sketches.add(pl)
        lk.name = "Leg"
        lk.sketchCurves.sketchCircles.addByCenterRadius(lk.modelToSketchSpace(pt), mm(d / 2))
        fi = ext.createInput(lk.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
        fi.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(mm(legs))), direction)
        fi.participantBodies = [body]
        ext.add(fi)
    occ.isGrounded = True

    if g.get("arbor"):
        a_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        a_occ.component.name = "Arbor"
        ak = a_occ.component.sketches.add(a_occ.component.xYConstructionPlane)
        ak.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(g["arbor"] / 2))
        a_occ.component.features.extrudeFeatures.addSimple(
            ak.profiles.item(0), adsk.core.ValueInput.createByReal(mm(p * Nb + d + 10)),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = "Arbor"
        a_occ.isGrounded = True
    return occ
