"""
E4: Draws the crank press in Fusion 360 (runs inside Fusion only): the slider-crank (fusion_lib.slider_crank)
and, under its base plate, the drive that makes the flywheel's part visible: the flywheel on its own fast shaft
with a pinion, meshing with a large gear on the crankshaft (the shaft comes up through the plate and the crank's
hole).  Motion links: the big gear turns with the crank, the flywheel at the gear stage's ratio, the other way.
The gear stage is drawn to show the motion (module 3, teeth from the engine's ratio); it is not sized: the press
engine sizes the crank, the rod and the flywheel.  The motor's belt to the flywheel is not drawn.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib import slider_crank as sc
from fusion_lib.fourbar import mm
from fusion_lib.gear import build_gear_body
from fusion_lib.marker import marker

GAP = 5.0
M = 3.0            # module of the drawn gear stage
WIDTH = 20.0       # its face width


def _cyl(c, r, z, h, name):
    pin = c.constructionPlanes.createInput()
    pin.setByOffset(c.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    sk = c.sketches.add(c.constructionPlanes.add(pin))
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(adsk.core.Point3D.create(0, 0, mm(z))), mm(r))
    b = c.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(h)),
                                             adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    b.name = name
    return b


def _revolute(root, occ, ground, body, r, name):
    edge = next(e for e in body.edges if isinstance(e.geometry, adsk.core.Circle3D) and abs(e.geometry.radius - mm(r)) < 1e-4)
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(occ), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(occ, ground, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def _link(root, j1, j2, deg2, reverse):
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = adsk.core.ValueInput.createByString("360 deg")
    li.valueTwo = adsk.core.ValueInput.createByString(f"{deg2} deg")
    li.isReversed = reverse
    return root.motionLinks.add(li)


def build(g, root, design=None):
    sc.build(g, root, design)
    D, t, bore = g["fly_D"], g["fly_t"], g["fly_bore"]
    d_shaft = g["section_mm"].get("shaft", g["section_mm"]["hole"])
    t_crank = g["section_mm"]["links"]["Crank"]["thickness"]
    ground = next(o for o in root.occurrences if o.component.name == "Ground")
    jc = next(j for j in root.asBuiltJoints if j.name == "Ground-Crank (O2)")

    # gear stage: the pinion's root must clear the flywheel shaft's bore
    z1 = max(17, math.ceil((bore + 2 * 2 * M + 2.5 * M) / M))
    z2 = max(z1 + 1, round(z1 * g["gear_ratio"]))
    C = M * (z1 + z2) / 2
    zg = -sc.PLATE_T - GAP - WIDTH                           # gears' lower face
    zf = zg - GAP - t                                        # flywheel's lower face

    # large gear on the crankshaft (O2 at the origin); a tooth space faces the pinion on the -x side
    mt = adsk.core.Matrix3D.create()
    mt.setToRotation(math.pi / z2 if z2 % 2 == 0 else 0.0, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    mt.translation = adsk.core.Vector3D.create(0, 0, mm(zg))
    big = root.occurrences.addNewComponent(mt)
    bc = big.component
    bc.name = "Gear (crankshaft)"
    build_gear_body(bc, M, z2, WIDTH, d_shaft).name = f"Gear z{z2} (drawn, not sized)"
    shaft = _cyl(bc, d_shaft / 2 - 0.2, 0, -zg + t_crank, "Shaft")    # up through the plate and the crank
    marker(bc, -(d_shaft / 2 + 3), -(M * z2 / 2 - 2.5 * M), WIDTH, max(6.0, M * z2 / 40))   # top face, to the pinion
    j_big = _revolute(root, big, ground, shaft, d_shaft / 2 - 0.2, "Ground-Gear")
    _link(root, jc, j_big, 360.0, False)

    # flywheel + pinion on the fast shaft at x = -C
    mt = adsk.core.Matrix3D.create()
    mt.translation = adsk.core.Vector3D.create(mm(-C), 0, mm(zg))
    occ = root.occurrences.addNewComponent(mt)
    c = occ.component
    c.name = "Flywheel"
    build_gear_body(c, M, z1, WIDTH, bore).name = f"Pinion z{z1}"
    _cyl(c, D / 2, zf - zg, t, f"Flywheel Ø{D:.0f}x{t:.0f}")
    fs = _cyl(c, bore / 2, zf - zg - 15, t + GAP + WIDTH + 25, "Shaft")
    marker(c, -(bore / 2 + 5), -0.9 * D / 2, zf - zg + t, max(6.0, D / 25))   # upper face, away from the gear
    jf = _revolute(root, occ, ground, fs, bore / 2, "Ground-Flywheel")
    _link(root, jc, jf, 360.0 * z2 / z1, True)
    return occ
