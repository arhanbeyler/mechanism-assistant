"""
G1a: Builds a helical gear pair in Fusion 360 (runs inside Fusion only).

Each gear: root disc + one tooth swept along the gear axis with a twist (the transverse involute profile
turned by F tan(psi) / r over the face width: an exact helicoid), patterned z times and joined to the disc,
then the shaft bore. The pinion is right hand, the gear left hand. Joints and the reversed motion link as
for the spur pair (fusion_lib.gear).
"""

import math

import adsk.core
import adsk.fusion

import gears
from fusion_lib.fourbar import mm
from fusion_lib.gear import GAP, PLATE_T, _bore_edge


def _profile(sketch, mt, z, phit, add):
    """Transverse tooth profile (axis +x), sunk into the root disc."""
    right, left, rf, ra = gears.involute_tooth(mt, z, n=10, phi=phit, add=add)
    r_in = rf - 0.3 * mt
    P = adsk.core.Point3D.create
    lines, splines = sketch.sketchCurves.sketchLines, sketch.sketchCurves.sketchFittedSplines
    a_r, a_l = math.atan2(right[0][1], right[0][0]), math.atan2(left[0][1], left[0][0])
    l1 = lines.addByTwoPoints(P(mm(r_in * math.cos(a_r)), mm(r_in * math.sin(a_r)), 0), P(mm(right[0][0]), mm(right[0][1]), 0))
    pts = adsk.core.ObjectCollection.create()
    pts.add(l1.endSketchPoint)
    for x, y in right[1:]:
        pts.add(P(mm(x), mm(y), 0))
    s1 = splines.add(pts)
    tip = lines.addByTwoPoints(s1.endSketchPoint, P(mm(left[-1][0]), mm(left[-1][1]), 0))
    pts = adsk.core.ObjectCollection.create()
    pts.add(tip.endSketchPoint)
    for x, y in reversed(left[:-1]):
        pts.add(P(mm(x), mm(y), 0))
    s2 = splines.add(pts)
    l2 = lines.addByTwoPoints(s2.endSketchPoint, P(mm(r_in * math.cos(a_l)), mm(r_in * math.sin(a_l)), 0))
    lines.addByTwoPoints(l2.endSketchPoint, l1.startSketchPoint)
    return min(sketch.profiles, key=lambda p: p.areaProperties().area) if sketch.profiles.count > 1 \
        else sketch.profiles.item(0), rf


def build_helical_body(comp, g, z, bore, hand):
    """One helical gear body (hand +1: right, -1: left)."""
    mn, psi = g["module"], math.radians(g["psi"])
    mt, phit = mn / math.cos(psi), math.radians(g["phi_t"])
    F = g["face_width"]
    r = z * mt / 2
    feats = comp.features
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "Tooth profile (transverse involute)"
    prof, rf = _profile(sk, mt, z, phit, math.cos(psi))

    disk = comp.sketches.add(comp.xYConstructionPlane)
    disk.name = "Root disc"
    disk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), mm(rf))
    body = feats.extrudeFeatures.addSimple(disk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(F)),
                                           adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)

    axis = comp.sketches.add(comp.xZConstructionPlane)
    axis.name = "Helix axis"
    to = lambda x, y, zz: axis.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(zz)))
    line = axis.sketchCurves.sketchLines.addByTwoPoints(to(0, 0, 0), to(0, 0, F))
    path = feats.createPath(line, False)
    si = feats.sweepFeatures.createInput(prof, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    twist = hand * F * math.tan(psi) / r
    si.twistAngle = adsk.core.ValueInput.createByReal(twist)
    tooth = feats.sweepFeatures.add(si).bodies.item(0)

    ents = adsk.core.ObjectCollection.create()
    ents.add(tooth)
    pin = feats.circularPatternFeatures.createInput(ents, comp.zConstructionAxis)
    pin.quantity = adsk.core.ValueInput.createByReal(z)
    pin.totalAngle = adsk.core.ValueInput.createByString("360 deg")
    pat = feats.circularPatternFeatures.add(pin)
    tools = adsk.core.ObjectCollection.create()
    tools.add(tooth)
    for b in pat.bodies:
        if b != tooth:
            tools.add(b)
    ci = feats.combineFeatures.createInput(body, tools)
    ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    feats.combineFeatures.add(ci)

    hole = comp.sketches.add(comp.xYConstructionPlane)
    hole.name = "Shaft bore"
    hole.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), mm(bore / 2))
    cut = feats.extrudeFeatures.createInput(hole.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(F)))
    cut.participantBodies = [body]
    feats.extrudeFeatures.add(cut)
    return body, twist


def build(g, root, design=None):
    z1, z2, F, a = g["z1"], g["z2"], g["face_width"], g["center_distance"]
    mn = g["module"]
    ra1, ra2 = g["d1"] / 2 + mn, g["d2"] / 2 + mn
    P = adsk.core.Point3D.create

    plate_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    plate = plate_occ.component
    plate.name = "Ground"
    sk = plate.sketches.add(plate.xYConstructionPlane)
    margin = 5.0
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        P(mm(-ra1 - margin), mm(-max(ra1, ra2) - margin), 0), P(mm(a + ra2 + margin), mm(max(ra1, ra2) + margin), 0))
    base = plate.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(PLATE_T)),
                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    base.name = "Ground"
    plate_occ.isGrounded = True

    zb = PLATE_T + GAP
    occs = {}
    for name, z, bore, x, phase, hand in (("Pinion", z1, g["bore1"], 0.0, 0.0, 1),
                                          ("Gear", z2, g["bore2"], a, math.pi / z2 if z2 % 2 == 0 else 0.0, -1)):
        mt = adsk.core.Matrix3D.create()
        mt.setToRotation(phase, adsk.core.Vector3D.create(0, 0, 1), P(0, 0, 0))
        mt.translation = adsk.core.Vector3D.create(mm(x), 0, mm(zb))
        occ = root.occurrences.addNewComponent(mt)
        occ.component.name = name
        body, _ = build_helical_body(occ.component, g, z, bore, hand)
        body.name = f"{name} z{z} ({'right' if hand > 0 else 'left'} hand)"
        occs[name] = (occ, bore)

    joints = {}
    for name in ("Pinion", "Gear"):
        occ, bore = occs[name]
        geo = adsk.fusion.JointGeometry.createByCurve(_bore_edge(occ, bore, F), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(plate_occ, occ, geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        j.name = f"Ground-{name}"
        joints[name] = j

    link = root.motionLinks.createInput(joints["Pinion"], joints["Gear"])
    link.valueOne = adsk.core.ValueInput.createByString("360 deg")
    link.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 * z1 / z2} deg")
    link.isReversed = True
    root.motionLinks.add(link)
    return occs
