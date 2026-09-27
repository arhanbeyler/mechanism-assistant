"""
Rotation markers (runs inside Fusion only): a thin yellow radial bar on the face of a part that turns, so a
smooth disc (flywheel, pulley, clutch plate, shaft end) visibly turns when the add-in drives it.  The bar is a
separate body named "Marker" in the turning component; fourbar.paint colours it.  Visual only.
"""

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm


def marker(comp, r0, r1, z, width, height=1.5):
    """Bar from radius r0 to r1 along local +x (negative radii: along -x), on the plane z; height > 0 raises it
    towards +z, < 0 towards -z."""
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    sk.name = "Marker"
    to = lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z)))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(to(r0, -width / 2), to(r1, width / 2))
    body = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(height)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = "Marker"
    return body
