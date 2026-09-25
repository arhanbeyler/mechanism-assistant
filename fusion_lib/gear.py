"""
C9: Duz disli ciftini Fusion 360'ta kurar (sadece Fusion icinde).

Her disli: dis dibi diski + tek disin evolvent profili (iki yan spline) -> dairesel coklama (z adet)
-> mil deligi. Iki disli taban plakasina doner mafsallarla baglanir; aralarinda disli oraninda,
ters yonlu motion link (pinyon bir tur -> cark z1/z2 tur).
"""

import math

import adsk.core
import adsk.fusion

import gears
from fusion_lib.fourbar import mm

PLATE_T = 3.0     # mm, taban plakasi
GAP = 0.5         # mm, plaka ile disli arasi


def _tooth_profile(sketch, m, z):
    """Tek dis (ekseni +x), dis dibi diskine gomulu kapali profil. Dondurur: profil."""
    right, left, rf, ra = gears.involute_tooth(m, z, n=10)
    r_in = rf - 0.3 * m                                     # diske gomulsun (birlesme icin)
    P = adsk.core.Point3D.create
    lines = sketch.sketchCurves.sketchLines
    splines = sketch.sketchCurves.sketchFittedSplines
    a_r = math.atan2(right[0][1], right[0][0])
    a_l = math.atan2(left[0][1], left[0][0])
    l1 = lines.addByTwoPoints(P(mm(r_in * math.cos(a_r)), mm(r_in * math.sin(a_r)), 0),
                              P(mm(right[0][0]), mm(right[0][1]), 0))
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
        else sketch.profiles.item(0)


def build_gear_body(comp, m, z, width, bore):
    """Bilesende tek disli govdesi. Dondurur: BRepBody."""
    ext = comp.features.extrudeFeatures
    dist = adsk.core.ValueInput.createByReal(mm(width))
    rf = m * z / 2 - 1.25 * m

    disk = comp.sketches.add(comp.xYConstructionPlane)
    disk.name = "Dis dibi"
    disk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), mm(rf))
    body = ext.addSimple(disk.profiles.item(0), dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)

    tooth_sk = comp.sketches.add(comp.xYConstructionPlane)
    tooth_sk.name = "Dis profili (evolvent)"
    tooth = ext.addSimple(_tooth_profile(tooth_sk, m, z), dist, adsk.fusion.FeatureOperations.JoinFeatureOperation)

    ents = adsk.core.ObjectCollection.create()
    ents.add(tooth)
    pin = comp.features.circularPatternFeatures.createInput(ents, comp.zConstructionAxis)
    pin.quantity = adsk.core.ValueInput.createByReal(z)
    pin.totalAngle = adsk.core.ValueInput.createByString("360 deg")
    pin.isSymmetric = False
    comp.features.circularPatternFeatures.add(pin)

    hole = comp.sketches.add(comp.xYConstructionPlane)
    hole.name = "Mil deligi"
    hole.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), mm(bore / 2))
    ext.addSimple(hole.profiles.item(0), dist, adsk.fusion.FeatureOperations.CutFeatureOperation)
    return body


def _bore_edge(occ, bore, z_top):
    body = occ.component.bRepBodies.item(0)
    for e in body.edges:
        g = e.geometry
        if isinstance(g, (adsk.core.Circle3D, adsk.core.Arc3D)) and abs(g.radius - mm(bore / 2)) < 1e-3 \
                and abs(g.center.x) < 1e-3 and abs(g.center.y) < 1e-3 and abs(g.center.z - mm(z_top)) < 1e-3:
            return e.createForAssemblyContext(occ)
    raise RuntimeError(f"{occ.component.name}: mil deligi kenari bulunamadi")


def build(g, root, design=None):
    """g: gears.design() ciktisi (+ type). root: temizlenmis kok bilesen."""
    m, z1, z2, b = g["module"], g["z1"], g["z2"], g["face_width"]
    a = g["center_distance"]
    ra1, ra2 = m * z1 / 2 + m, m * z2 / 2 + m
    P = adsk.core.Point3D.create

    # Taban plakasi (sabit): iki mil deligi
    plate_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    plate = plate_occ.component
    plate.name = "Sabit"
    sk = plate.sketches.add(plate.xYConstructionPlane)
    margin = 5.0
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        P(mm(-ra1 - margin), mm(-max(ra1, ra2) - margin), 0), P(mm(a + ra2 + margin), mm(max(ra1, ra2) + margin), 0))
    body = plate.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(PLATE_T)),
                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = "Sabit"
    holes = plate.sketches.add(plate.xYConstructionPlane)
    holes.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(g["bore1"] / 2))
    holes.sketchCurves.sketchCircles.addByCenterRadius(P(mm(a), 0, 0), mm(g["bore2"] / 2))
    cut = adsk.core.ObjectCollection.create()
    for p in holes.profiles:
        cut.add(p)
    plate.features.extrudeFeatures.addSimple(cut, adsk.core.ValueInput.createByReal(mm(PLATE_T)),
                                             adsk.fusion.FeatureOperations.CutFeatureOperation)
    plate_occ.isGrounded = True

    # Disliler: pinyon orijinde, cark (a, 0)'da; carkin bir dis boslugu pinyona baksin
    zb = PLATE_T + GAP
    occs = {}
    for name, z, bore, x, phase in (("Pinyon", z1, g["bore1"], 0.0, 0.0),
                                    ("Cark", z2, g["bore2"], a, math.pi / z2 if z2 % 2 == 0 else 0.0)):
        mt = adsk.core.Matrix3D.create()
        mt.setToRotation(phase, adsk.core.Vector3D.create(0, 0, 1), P(0, 0, 0))
        mt.translation = adsk.core.Vector3D.create(mm(x), 0, mm(zb))
        occ = root.occurrences.addNewComponent(mt)
        occ.component.name = name
        build_gear_body(occ.component, m, z, b, bore).name = name
        occs[name] = (occ, bore)

    joints = {}
    for name in ("Pinyon", "Cark"):
        occ, bore = occs[name]
        geo = adsk.fusion.JointGeometry.createByCurve(_bore_edge(occ, bore, b),
                                                      adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(plate_occ, occ, geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        j.name = f"Sabit-{name}"
        joints[name] = j

    link = root.motionLinks.createInput(joints["Pinyon"], joints["Cark"])
    link.valueOne = adsk.core.ValueInput.createByString("360 deg")
    link.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 * z1 / z2} deg")
    link.isReversed = True
    root.motionLinks.add(link)
    return occs
