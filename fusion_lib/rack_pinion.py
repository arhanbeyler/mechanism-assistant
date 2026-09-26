"""
G4: Builds the rack and pinion in Fusion 360 (runs inside Fusion only).

Pinion at the origin (involute teeth from fusion_lib.gear), turned so that a tooth points straight down; the
rack's pitch line is tangent to the pitch circle below it, with a tooth space centred under that tooth
(rack_pinion.rack_outline). Joints: Ground-Pinion (revolute, the add-in's input), Ground-Rack (slider along x,
direction checked by a trial slide) and a motion link: one turn = pi d of rack travel, its sense chosen by a
small-turn overlap test (as the power screw's).
"""

import math

import adsk.core
import adsk.fusion

import rack_pinion
from fusion_lib.fourbar import mm
from fusion_lib.gear import build_gear_body
from fusion_lib.slider_crank import PLATE_T, _box

GAP = 0.5


def _overlap(a_body, b_body):
    tb = adsk.fusion.TemporaryBRepManager.get()
    a, c = tb.copy(a_body), tb.copy(b_body)
    tb.booleanOperation(a, c, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    return (a.volume or 0.0) * 1000


def build(g, root, design=None):
    m, z, b, bore = g["m"], g["z"], g["b"], g["bore"]
    d = m * z
    L = g["rack_len"]
    back = g["back"]
    y_bot = -d / 2 - 1.25 * m - back

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    _box(gc, -L / 2 - 20, y_bot - 10, L / 2 + 20, d / 2 + m + 10, -PLATE_T - GAP, PLATE_T, "Base")
    ground.isGrounded = True

    mt = adsk.core.Matrix3D.create()
    mt.setToRotation(-math.pi / 2, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    pin = root.occurrences.addNewComponent(mt)
    pin.component.name = "Pinion"
    pb = build_gear_body(pin.component, m, z, b, bore)
    pb.name = f"Pinion z{z} m{m:g}"

    rack = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    rc = rack.component
    rc.name = "Rack"
    sk = rc.sketches.add(rc.xYConstructionPlane)
    sk.name = "Rack profile"
    pts = rack_pinion.rack_outline(m, d, L, back)
    P = lambda x, y: adsk.core.Point3D.create(mm(x), mm(y), 0)
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P(*pts[0]), P(*pts[1]))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    rb = rc.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(b)),
                                               adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    rb.name = f"Rack m{m:g}"

    edge = next(e for e in pb.edges if isinstance(e.geometry, (adsk.core.Circle3D, adsk.core.Arc3D))
                and abs(e.geometry.radius - mm(bore / 2)) < 1e-4 and abs(e.geometry.center.z) < 1e-6)
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(pin), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(pin, ground, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    jp = root.asBuiltJoints.add(ji)
    jp.name = "Ground-Pinion"

    corner = next(e for e in rb.edges if isinstance(e.geometry, adsk.core.Line3D))
    slider = None
    for direction in (adsk.fusion.JointDirections.XAxisJointDirection, adsk.fusion.JointDirections.YAxisJointDirection,
                      adsk.fusion.JointDirections.ZAxisJointDirection):
        g2 = adsk.fusion.JointGeometry.createByPoint(corner.startVertex.createForAssemblyContext(rack))
        ji = root.asBuiltJoints.createInput(rack, ground, g2)
        ji.setAsSliderJointMotion(direction)
        j = root.asBuiltJoints.add(ji)
        mo = adsk.fusion.SliderJointMotion.cast(j.jointMotion)
        p0 = rack.transform2.translation
        mo.slideValue = 0.05
        p1 = rack.transform2.translation
        mo.slideValue = 0.0
        if abs(abs(p1.x - p0.x) - 0.05) < 1e-4:
            j.name = "Ground-Rack"
            slider = j
            break
        j.deleteMe()
    if slider is None:
        raise RuntimeError("Rack: no slider along x")
    lim = adsk.fusion.SliderJointMotion.cast(slider.jointMotion).slideLimits
    lim.isMinimumValueEnabled, lim.minimumValue = True, -mm(g["travel"] / 2)
    lim.isMaximumValueEnabled, lim.maximumValue = True, mm(g["travel"] / 2)

    rot = adsk.fusion.RevoluteJointMotion.cast(jp.jointMotion)
    for sense in (1, -1):
        link = root.motionLinks.createInput(jp, slider)
        link.valueOne = adsk.core.ValueInput.createByString("360 deg")
        link.valueTwo = adsk.core.ValueInput.createByString(f"{sense * math.pi * d} mm")
        ml = root.motionLinks.add(link)
        rot.rotationValue = math.radians(360.0 / z / 2)          # half a tooth: the wrong sense collides
        v = _overlap(pin.bRepBodies.item(0), rack.bRepBodies.item(0))
        rot.rotationValue = 0.0
        if v < 1.0:
            return pin
        ml.deleteMe()
    raise RuntimeError("Rack and pinion: the rack does not follow the pinion either way")
