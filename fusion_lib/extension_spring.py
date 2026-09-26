"""
H4c: Draws an extension spring in Fusion 360 (runs inside Fusion only), free (closed coils).

The body: a close-wound helix spline (pitch 1.05 d so the sweep does not touch itself) with the wire swept along
it; each hook is a half circle of the coil radius in the axial plane through the wire's end (centred on the
axis at the end's height), swept too and joined.
Nothing moves.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

GAP = 1.05
PER_TURN = 24


def _sweep(comp, pts, d, name, body=None):
    P = adsk.core.Point3D.create
    sk = comp.sketches.add(comp.xYConstructionPlane)
    col = adsk.core.ObjectCollection.create()
    for x, y, z in pts:
        col.add(P(mm(x), mm(y), mm(z)))
    spline = sk.sketchCurves.sketchFittedSplines.add(col)
    path = comp.features.createPath(spline, False)
    pi = comp.constructionPlanes.createInput()
    pi.setByDistanceOnPath(path, adsk.core.ValueInput.createByReal(0.0))
    ps = comp.sketches.add(comp.constructionPlanes.add(pi))
    ps.sketchCurves.sketchCircles.addByCenterRadius(ps.modelToSketchSpace(col.item(0)), mm(d / 2))
    op = adsk.fusion.FeatureOperations.JoinFeatureOperation if body else adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    si = comp.features.sweepFeatures.createInput(ps.profiles.item(0), path, op)
    if body:
        si.participantBodies = [body]
    out = comp.features.sweepFeatures.add(si)
    if not body:
        out.bodies.item(0).name = name
        return out.bodies.item(0)
    return body


def build(g, root, design=None):
    d, D, Nb = g["d"], g["D"], g["Nb"]
    R, p = D / 2, GAP * d
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Spring (extension)"
    n = int(round(Nb * PER_TURN))
    helix = [(R * math.cos(2 * math.pi * Nb * i / n), R * math.sin(2 * math.pi * Nb * i / n), p * Nb * i / n)
             for i in range(n + 1)]
    body = _sweep(comp, helix, d, f"Extension spring d{d:g}")
    for (x, y, z), up in ((helix[0], -1), (helix[-1], 1)):
        a = math.atan2(y, x)
        ux, uy = math.cos(a), math.sin(a)                   # the axial plane through the wire's end
        # half circle of the coil radius centred on the axis at the end's height: it starts at the wire's end,
        # rises over the axis and comes down on the opposite side of the last coil (a full-loop hook)
        loop = [(R * math.cos(t) * ux, R * math.cos(t) * uy, z + up * R * math.sin(t))
                for t in [k * math.pi / 24 for k in range(25)]]
        body = _sweep(comp, loop, d, "Hook", body)
    occ.isGrounded = True
    return occ
