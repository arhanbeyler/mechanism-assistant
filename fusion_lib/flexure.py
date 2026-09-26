"""
I2: Draws a printed living hinge in Fusion 360 (runs inside Fusion only), flat as printed: two panels joined
by the thin strip flush with their bottom face (one body, printed in one go).
"""

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

PANEL = 25.0
PANEL_T = 3.0


def build(g, root, design=None):
    L, t, w = g["L"], g["t"], g["w"]
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Hinge"
    sk = comp.sketches.add(comp.xZConstructionPlane)
    to = lambda x, z: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), 0, mm(z)))
    pts = [(-L / 2 - PANEL, 0), (L / 2 + PANEL, 0), (L / 2 + PANEL, PANEL_T), (L / 2, PANEL_T), (L / 2, t),
           (-L / 2, t), (-L / 2, PANEL_T), (-L / 2 - PANEL, PANEL_T)]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to(*pts[0]), to(*pts[1]))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, to(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    ext = comp.features.extrudeFeatures
    fi = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    fi.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(w / 2)), False)
    ext.add(fi).bodies.item(0).name = f"Living hinge {L:g}x{t:g}"
    occ.isGrounded = True
    return occ
