"""
H5b: Draws an engaged dry disc clutch in Fusion 360 (runs inside Fusion only).

Along the (horizontal) axis: the input's flywheel plate, then N/2 friction disks (output) with intermediate
plates (input) between them, then the input's pressure plate; the output shaft runs through the input
plates. Input (joint Ground-Driver, the add-in's input) and output (Ground-Driven) are linked 1:1: the clutch
is engaged. Faces touch, nothing overlaps.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.flywheel import PLATE_T, horizontal, pedestals, revolute
from fusion_lib.fourbar import mm
from fusion_lib.marker import marker

PLATE = 12.0          # flywheel / intermediate / pressure plate thickness
DISK = 8.0            # friction disk (lining + core)


def _ring(comp, r_out, r_in, z0, h, name):
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
    ro, ri, N, bore = g["ro"], g["ri"], g["N"], g["bore"]
    R_plate = ro + 10
    H = PLATE_T + R_plate + 10
    disks = N // 2
    length = PLATE + disks * DISK + (disks - 1) * PLATE + PLATE
    span = length + 120
    ground = pedestals(root, H, span, 2 * R_plate + 20, bore)

    inp = root.occurrences.addNewComponent(horizontal(0, 0, H))
    ic = inp.component
    ic.name = "Input"
    out = root.occurrences.addNewComponent(horizontal(0, 0, H))
    oc = out.component
    oc.name = "Output"

    z = -length / 2
    _ring(ic, R_plate, 0, z, PLATE, "Flywheel plate")
    shaft_in = _ring(ic, bore / 2, 0, -span / 2 - 20, span / 2 + 20 + z, "Shaft")
    z += PLATE
    for k in range(disks):
        _ring(oc, ro, bore / 2, z, DISK, f"Friction disk {k + 1}")
        z += DISK
        name = "Pressure plate" if k == disks - 1 else f"Intermediate plate {k + 1}"
        _ring(ic, R_plate, bore / 2 + 2, z, PLATE, name)
        z += PLATE
    shaft_out = _ring(oc, bore / 2, 0, -length / 2 + PLATE, span / 2 + 20 + length / 2 - PLATE, "Shaft")
    marker(ic, bore / 2 + 4, 0.9 * R_plate, length / 2, max(6.0, R_plate / 12))    # on the pressure plate
    marker(oc, 0, bore / 2 + 12, span / 2 + 20, 6.0, 3.0)                        # a flag on the output end
    marker(ic, 0, bore / 2 + 12, -span / 2 - 20, 6.0, -3.0)                      # and on the input end

    j_in = revolute(root, inp, ground, shaft_in, bore / 2, "Ground-Driver")
    j_out = revolute(root, out, ground, shaft_out, bore / 2, "Ground-Driven")
    mo = adsk.fusion.RevoluteJointMotion.cast(j_in.jointMotion)

    def ang(o):
        m = o.transform2
        return math.atan2(m.getCell(2, 1), m.getCell(1, 1))       # rotation about the global x axis

    for reverse in (False, True):
        li = root.motionLinks.createInput(j_in, j_out)
        li.valueOne = adsk.core.ValueInput.createByString("360 deg")
        li.valueTwo = adsk.core.ValueInput.createByString("360 deg")
        li.isReversed = reverse
        link = root.motionLinks.add(li)
        a0, b0 = ang(inp), ang(out)
        mo.rotationValue = 0.3
        same = abs((ang(inp) - a0) - (ang(out) - b0)) < 1e-6
        mo.rotationValue = 0.0
        if same:
            return inp
        link.deleteMe()
    raise RuntimeError("Clutch: the output does not follow the input")
