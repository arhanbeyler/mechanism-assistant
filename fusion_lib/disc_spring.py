"""
H4b: Draws a disc spring stack in Fusion 360 (runs inside Fusion only), free (unloaded).

Every disc is revolved from its section, a parallelogram in the (r, z) plane (a cone of height h0 and thickness
t); discs in a group are nested (same way up), groups alternate (inner edges touch, then outer edges), on a
grey guide rod.
Animation: FRAMES frames from free to the working deflection s per disc (cone height h0 - s), shown in turn by
the add-in; the rod stays.
"""

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm
from fusion_lib.spring import frame_occurrences

FRAMES = 6


def _disc(comp, ro, ri, t, h0, z0, up, name):
    sk = comp.sketches.add(comp.xZConstructionPlane)
    to = lambda r, z: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(r), 0, mm(z)))
    if up:          # inner edge high: outer at z0, inner at z0 + h0
        pts = [(ro, z0), (ro, z0 + t), (ri, z0 + h0 + t), (ri, z0 + h0)]
    else:           # inner edge low
        pts = [(ri, z0), (ri, z0 + t), (ro, z0 + h0 + t), (ro, z0 + h0)]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to(*pts[0]), to(*pts[1]))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, to(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    ri_ = comp.features.revolveFeatures.createInput(sk.profiles.item(0), comp.zConstructionAxis,
                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri_.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    comp.features.revolveFeatures.add(ri_).bodies.item(0).name = name


def _frame(c, g, s):
    """The stack with every disc flattened by s (cone height h0 - s)."""
    ro, ri, t, np_, ns = g["De"] / 2, g["Di"] / 2, g["t"], g["np"], g["ns"]
    h = g["h0"] - s
    z = 0.0
    for k in range(ns):
        up = k % 2 == 0
        for j in range(np_):
            _disc(c, ro, ri, t, h, z + j * t, up, f"Disc {k + 1}.{j + 1}")
        z += h + np_ * t
    return f"{ns * s:.2f} mm"


def build(g, root, design=None):
    ri, t, h0, np_, ns = g["Di"] / 2, g["t"], g["h0"], g["np"], g["ns"]
    s = g.get("s") or 0.0
    occ = frame_occurrences(root, design, FRAMES, lambda c, f: _frame(c, g, s * f))[0]
    z = ns * (h0 + np_ * t)
    rod = g.get("rod") or max(2.0, 2 * ri - 1.0)
    r_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    r_occ.component.name = "Arbor"
    pin = r_occ.component.constructionPlanes.createInput()
    pin.setByOffset(r_occ.component.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(-5)))
    sk = r_occ.component.sketches.add(r_occ.component.constructionPlanes.add(pin))
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(adsk.core.Point3D.create(0, 0, mm(-5))), mm(rod / 2))
    r_occ.component.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(z + 10)),
                                                       adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = "Guide rod"
    r_occ.isGrounded = True
    return occ
