"""
H3: Draws a press fit in Fusion 360 (runs inside Fusion only): a steel shaft and a translucent hub on it,
with the shaft sticking out on both sides. Nothing moves (the fit is the joint).
"""

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm


def _cyl(comp, r_out, r_in, z0, h, name):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    c = sk.modelToSketchSpace(adsk.core.Point3D.create(0, 0, mm(z0)))
    sk.sketchCurves.sketchCircles.addByCenterRadius(c, mm(r_out))
    if r_in:
        sk.sketchCurves.sketchCircles.addByCenterRadius(c, mm(r_in))
        prof = next(p for p in sk.profiles if p.profileLoops.count == 2)
    else:
        prof = sk.profiles.item(0)
    body = comp.features.extrudeFeatures.addSimple(prof, adsk.core.ValueInput.createByReal(mm(h)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def build(g, root, design=None):
    d, D, L = g["d"], g["D_hub"], g["L"]
    stick = max(d, 20.0)
    hub = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    hub.component.name = "Hub"
    b = _cyl(hub.component, D / 2, d / 2, 0.0, L, f"Hub Ø{D:g} x {L:g}")
    b.opacity = 0.4
    hub.isGrounded = True
    shaft = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    shaft.component.name = "Shaft"
    _cyl(shaft.component, d / 2, 0, -stick, L + 2 * stick, f"Shaft Ø{d:g}")
    shaft.isGrounded = True
    return hub
