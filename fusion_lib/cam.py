"""
C11a: Disk kam + oteleyen makarali itici — Fusion 360 cizici (sadece Fusion icinde).

Kam profili (cams.profile) kapali spline -> extrude -> mil deligi. Itici: makara + govde tek parca.
Kam doner mafsal (Sabit-Kam), itici +y yonunde kayar mafsal (Sabit-Itici). Fusion'da kam-itici
temasi mafsalla kurulamadigi icin s(theta) tablosu belgeye yazilir; eklenti kami dondururken
iticiyi tablodan konumlandirir.
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
    raise RuntimeError(f"{occ.component.name}: dairesel kenar bulunamadi")


def build(d, root, design):
    P = adsk.core.Point3D.create
    NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
    JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
    Rb, Rf, h = d["Rb"], d["Rf"], d["h"]
    Rp = Rb + Rf
    t = max(6.0, round(Rf))                     # kam ve makara kalinligi
    top = Rp + h + Rf + 25                      # itici govdesinin ust ucu (y)
    zc = PLATE_T + GAP

    # Taban plakasi
    plate_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    plate = plate_occ.component
    plate.name = "Sabit"
    sk = plate.sketches.add(plate.xYConstructionPlane)
    R = Rp + h + 5
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-R), mm(-R), 0), P(mm(R), mm(top), 0))
    _extrude(plate, [sk.profiles.item(0)], PLATE_T, NEW).bodies.item(0).name = "Sabit"
    hs = plate.sketches.add(plate.xYConstructionPlane)
    hs.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(d["bore"] / 2))
    _extrude(plate, [hs.profiles.item(0)], PLATE_T, CUT)
    plate_occ.isGrounded = True

    # Kam
    mt = adsk.core.Matrix3D.create()
    mt.translation = adsk.core.Vector3D.create(0, 0, mm(zc))
    cam_occ = root.occurrences.addNewComponent(mt)
    cam = cam_occ.component
    cam.name = "Kam"
    ps = cam.sketches.add(cam.xYConstructionPlane)
    ps.name = "Kam profili"
    pts = cams.profile(d, n=360)
    col = adsk.core.ObjectCollection.create()
    first = None
    for i, (x, y) in enumerate(pts):
        if i == 0:
            first = ps.sketchPoints.add(P(mm(x), mm(y), 0))
            col.add(first)
        else:
            col.add(P(mm(x), mm(y), 0))
    col.add(first)                               # kapali egri
    ps.sketchCurves.sketchFittedSplines.add(col)
    _extrude(cam, [ps.profiles.item(0)], t, NEW).bodies.item(0).name = "Kam"
    bs = cam.sketches.add(cam.xYConstructionPlane)
    bs.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(d["bore"] / 2))
    _extrude(cam, [bs.profiles.item(0)], t, CUT)

    # Itici: makara (merkez y = Rp, s = 0) + govde
    fol_occ = root.occurrences.addNewComponent(mt)
    fol = fol_occ.component
    fol.name = "Itici"
    fs = fol.sketches.add(fol.xYConstructionPlane)
    fs.sketchCurves.sketchCircles.addByCenterRadius(P(0, mm(Rp), 0), mm(Rf))
    _extrude(fol, [fs.profiles.item(0)], t, NEW).bodies.item(0).name = "Itici"
    ss = fol.sketches.add(fol.xYConstructionPlane)
    ss.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-Rf / 2), mm(Rp), 0), P(mm(Rf / 2), mm(top), 0))
    _extrude(fol, [p for p in ss.profiles], t, JOIN)

    # Mafsallar
    geo = adsk.fusion.JointGeometry.createByCurve(_circle_edge(cam_occ, 0, 0, d["bore"] / 2, t),
                                                  adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(plate_occ, cam_occ, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    root.asBuiltJoints.add(ji).name = "Sabit-Kam"
    geo = adsk.fusion.JointGeometry.createByCurve(_circle_edge(fol_occ, 0, Rp, Rf, t),
                                                  adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(plate_occ, fol_occ, geo)
    # Kayma yonu: mafsal geometrisinin (makara dairesi, normali z) Y ekseni. Ozel yon (kok ekseni
    # ya da govde kenari) verildiginde Fusion onu yok sayip z yonunde kaydiriyordu.
    ji.setAsSliderJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
    slider = root.asBuiltJoints.add(ji)
    slider.name = "Sabit-Itici"

    # Gercekten y yonunde mi ve pozitif kayma +y mi -y mi? Deneme kaydirmasiyla olc
    motion = adsk.fusion.SliderJointMotion.cast(slider.jointMotion)
    p0 = fol_occ.transform2.translation
    motion.slideValue = 0.1
    p1 = fol_occ.transform2.translation
    motion.slideValue = 0.0
    dy, dx, dz = p1.y - p0.y, p1.x - p0.x, p1.z - p0.z
    if abs(dy) < 0.09 or abs(dx) > 1e-4 or abs(dz) > 1e-4:
        raise RuntimeError(f"Itici y yonunde kaymiyor (deneme: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f} cm)")
    sign = 1.0 if dy > 0 else -1.0

    # Animasyon icin s(theta) tablosu (derece basina, mm) ve kayma yonu
    for key in ("cam_table", "cam_slide_sign"):
        old = design.attributes.itemByName(MARK_GROUP, key)
        if old:
            old.deleteMe()
    design.attributes.add(MARK_GROUP, "cam_table", json.dumps(d["s_table"]))
    design.attributes.add(MARK_GROUP, "cam_slide_sign", str(sign))
    return cam_occ, fol_occ


