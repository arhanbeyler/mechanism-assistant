"""
C11a: Disk cam + translating roller follower: Fusion 360 builder (runs inside Fusion only).

Cam profile (cams.profile) as a closed spline -> extrude -> shaft bore. Follower: roller + stem in one
part. The cam has a revolute joint (Ground-Cam), the follower a slider joint along +y (Ground-Follower).
Fusion cannot model the cam-follower contact with a joint, so the s(theta) table is written into the
document; while turning the cam the add-in places the follower from that table.
"""

import json

import adsk.core
import adsk.fusion

import cams
from fusion_lib.fourbar import MARK_GROUP, mm

PLATE_T = 3.0
GAP = 0.5


def _extrude(comp, profiles, t, op):
    col = adsk.core.ObjectCollection.create()
    for p in profiles:
        col.add(p)
    return comp.features.extrudeFeatures.addSimple(col, adsk.core.ValueInput.createByReal(mm(t)), op)


def _circle_edge(occ, x, y, r, z_top):
    for e in occ.component.bRepBodies.item(0).edges:
        g = e.geometry
        if isinstance(g, (adsk.core.Circle3D, adsk.core.Arc3D)) and abs(g.radius - mm(r)) < 1e-3 \
                and abs(g.center.x - mm(x)) < 1e-3 and abs(g.center.y - mm(y)) < 1e-3 and abs(g.center.z - mm(z_top)) < 1e-3:
            return e.createForAssemblyContext(occ)
    raise RuntimeError(f"{occ.component.name}: circular edge not found")


def build(d, root, design):
    P = adsk.core.Point3D.create
    NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
    JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
    Rb, Rf, h = d["Rb"], d["Rf"], d["h"]
    Rp = Rb + Rf
    t = max(6.0, round(Rf))                     # cam and roller thickness
    top = Rp + h + Rf + 25                      # top end of the follower stem (y)
    zc = PLATE_T + GAP

    # Base plate
    plate_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    plate = plate_occ.component
    plate.name = "Ground"
    sk = plate.sketches.add(plate.xYConstructionPlane)
    R = Rp + h + 5
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-R), mm(-R), 0), P(mm(R), mm(top), 0))
    _extrude(plate, [sk.profiles.item(0)], PLATE_T, NEW).bodies.item(0).name = "Ground"
    hs = plate.sketches.add(plate.xYConstructionPlane)
    hs.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(d["bore"] / 2))
    _extrude(plate, [hs.profiles.item(0)], PLATE_T, CUT)
    plate_occ.isGrounded = True

    # Cam
    mt = adsk.core.Matrix3D.create()
    mt.translation = adsk.core.Vector3D.create(0, 0, mm(zc))
    cam_occ = root.occurrences.addNewComponent(mt)
    cam = cam_occ.component
    cam.name = "Cam"
    ps = cam.sketches.add(cam.xYConstructionPlane)
    ps.name = "Cam profile"
    pts = cams.profile(d, n=360)
    col = adsk.core.ObjectCollection.create()
    first = None
    for i, (x, y) in enumerate(pts):
        if i == 0:
            first = ps.sketchPoints.add(P(mm(x), mm(y), 0))
            col.add(first)
        else:
            col.add(P(mm(x), mm(y), 0))
    col.add(first)                               # closed curve
    ps.sketchCurves.sketchFittedSplines.add(col)
    _extrude(cam, [ps.profiles.item(0)], t, NEW).bodies.item(0).name = "Cam"
    bs = cam.sketches.add(cam.xYConstructionPlane)
    bs.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(d["bore"] / 2))
    _extrude(cam, [bs.profiles.item(0)], t, CUT)

    # Follower: roller (centre y = Rp, s = 0) + stem
    fol_occ = root.occurrences.addNewComponent(mt)
    fol = fol_occ.component
    fol.name = "Follower"
    fs = fol.sketches.add(fol.xYConstructionPlane)
    fs.sketchCurves.sketchCircles.addByCenterRadius(P(0, mm(Rp), 0), mm(Rf))
    _extrude(fol, [fs.profiles.item(0)], t, NEW).bodies.item(0).name = "Follower"
    ss = fol.sketches.add(fol.xYConstructionPlane)
    ss.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-Rf / 2), mm(Rp), 0), P(mm(Rf / 2), mm(top), 0))
    _extrude(fol, [p for p in ss.profiles], t, JOIN)

    # Joints
    geo = adsk.fusion.JointGeometry.createByCurve(_circle_edge(cam_occ, 0, 0, d["bore"] / 2, t),
                                                  adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(plate_occ, cam_occ, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    root.asBuiltJoints.add(ji).name = "Ground-Cam"
    geo = adsk.fusion.JointGeometry.createByCurve(_circle_edge(fol_occ, 0, Rp, Rf, t),
                                                  adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(plate_occ, fol_occ, geo)
    # Slide direction: the Y axis of the joint geometry (roller circle, normal z). When a custom direction
    # (root axis or a body edge) was given, Fusion ignored it and slid along z.
    ji.setAsSliderJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
    slider = root.asBuiltJoints.add(ji)
    slider.name = "Ground-Follower"

    # Is it really along y, and is a positive slide +y or -y? Measured with a trial slide
    motion = adsk.fusion.SliderJointMotion.cast(slider.jointMotion)
    p0 = fol_occ.transform2.translation
    motion.slideValue = 0.1
    p1 = fol_occ.transform2.translation
    motion.slideValue = 0.0
    dy, dx, dz = p1.y - p0.y, p1.x - p0.x, p1.z - p0.z
    if abs(dy) < 0.09 or abs(dx) > 1e-4 or abs(dz) > 1e-4:
        raise RuntimeError(f"The follower does not slide along y (trial: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f} cm)")
    sign = 1.0 if dy > 0 else -1.0

    # s(theta) table for the animation (per degree, mm) and the slide sense
    for key in ("cam_table", "cam_slide_sign"):
        old = design.attributes.itemByName(MARK_GROUP, key)
        if old:
            old.deleteMe()
    design.attributes.add(MARK_GROUP, "cam_table", json.dumps(d["s_table"]))
    design.attributes.add(MARK_GROUP, "cam_slide_sign", str(sign))
    return cam_occ, fol_occ


