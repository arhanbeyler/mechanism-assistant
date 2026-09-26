"""
E4: Draws the crank press in Fusion 360 (runs inside Fusion only): the slider-crank (fusion_lib.slider_crank)
and, beside it under the base plate, the flywheel on its own fast shaft. A motion link turns the flywheel at the
gear ratio when the crank turns (the belt and the gears between them are not drawn).
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib import slider_crank as sc
from fusion_lib.fourbar import mm

GAP = 5.0


def build(g, root, design=None):
    sc.build(g, root, design)
    D, t, bore = g["fly_D"], g["fly_t"], g["fly_bore"]
    ground = next(o for o in root.occurrences if o.component.name == "Ground")
    x = -(g["a"] + g["section_mm"]["links"]["Crank"].get("boss0", 20) / 2 + D / 2 + 20)
    z0 = -sc.PLATE_T - GAP - t
    mt = adsk.core.Matrix3D.create()
    mt.translation = adsk.core.Vector3D.create(mm(x), 0, 0)
    occ = root.occurrences.addNewComponent(mt)
    c = occ.component
    c.name = "Flywheel"
    P = adsk.core.Point3D.create
    bodies = []
    for r, z, h, name in ((D / 2, z0, t, f"Flywheel Ø{D:.0f}x{t:.0f}"), (bore / 2, z0 - 15, t + 15 + sc.PLATE_T + GAP, "Shaft")):
        pin = c.constructionPlanes.createInput()
        pin.setByOffset(c.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
        sk = c.sketches.add(c.constructionPlanes.add(pin))
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(0, 0, mm(z))), mm(r))
        b = c.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(h)),
                                                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
        b.name = name
        bodies.append(b)
    edge = next(e for e in bodies[1].edges if isinstance(e.geometry, adsk.core.Circle3D))
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(occ), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(occ, ground, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    jf = root.asBuiltJoints.add(ji)
    jf.name = "Ground-Flywheel"
    jc = next(j for j in root.asBuiltJoints if j.name == "Ground-Crank (O2)")
    li = root.motionLinks.createInput(jc, jf)
    li.valueOne = adsk.core.ValueInput.createByString("360 deg")
    li.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 * g['gear_ratio']} deg")
    root.motionLinks.add(li)
    return occ
