"""
C10: Basma yayini Fusion 360'ta kurar (sadece Fusion icinde).

API'de yeni "coil" olusturulamadigi icin: tel merkez cizgisi 3B spline (springs.helix_points,
konik yayda degisken yaricap, uclarda kapali sarimlar) -> yolun basinda yola dik duzlemde
tel kesiti (daire) -> sweep -> z = 0 ve z = L0 duzlemleriyle kesme (kare ve taslanmis uclar).

Animasyon: Fusion'da govdeler esnemez; yay FRAMES farkli sikisma miktarinda (0 -> calisma cokmesi)
ayri kareler olarak cizilir (her karede ust basma plakasi). Eklenti kareleri sirayla gorunur yapar.
"""

import adsk.core
import adsk.fusion

import springs
from fusion_lib.fourbar import MARK_GROUP, mm

END_GAP = 1.03    # kapali uc sarimlarinda hatve = 1.03 d (tam temas sweep'te kendine degen govde yapiyor)
FRAMES = 8
PLATE_T = 3.0


def _plate(comp, R, z0, t, name):
    P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(R))
    body = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(t)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def _spring_body(comp, geo, compression):
    """compression mm sikismis yay govdesi (taslanmis uclar). Dondurur: (govde, ust yuzey z'si)."""
    d, L0 = geo["d"], geo["L0"]
    P = adsk.core.Point3D.create
    g = dict(geo)
    g["d"] = d * END_GAP
    g["pitch"] = (L0 - 2 * g["d"]) / geo["Na"]       # gorsel: uc boslugu; L0 ve k degismez
    pts = springs.helix_points(g, per_turn=16, compression=compression)
    z_top = pts[-1][2]

    path_sk = comp.sketches.add(comp.xYConstructionPlane)
    path_sk.name = "Tel ekseni (helis)"
    col = adsk.core.ObjectCollection.create()
    for x, y, z in pts:
        col.add(P(mm(x), mm(y), mm(z)))
    spline = path_sk.sketchCurves.sketchFittedSplines.add(col)
    path = comp.features.createPath(spline, False)       # alt bilesende: bilesen baglaminda yol

    pl_in = comp.constructionPlanes.createInput()
    pl_in.setByDistanceOnPath(path, adsk.core.ValueInput.createByReal(0.0))
    prof_sk = comp.sketches.add(comp.constructionPlanes.add(pl_in))
    center = prof_sk.modelToSketchSpace(P(mm(pts[0][0]), mm(pts[0][1]), mm(pts[0][2])))
    prof_sk.sketchCurves.sketchCircles.addByCenterRadius(center, mm(d / 2))
    sw_in = comp.features.sweepFeatures.createInput(prof_sk.profiles.item(0), path,
                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = comp.features.sweepFeatures.add(sw_in).bodies.item(0)
    body.name = "Yay"

    R = max(geo["D1"], geo["D2"]) / 2 + d * 2
    for z0, direction in ((0.0, -1.0), (z_top, 1.0)):   # taslama
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
        sk = comp.sketches.add(comp.constructionPlanes.add(pin))
        sk.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-R), mm(-R), 0), P(mm(R), mm(R), 0))
        cut = comp.features.extrudeFeatures.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
        cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(direction * 2 * d)))
        cut.participantBodies = [body]     # SADECE bu karenin yayi: varsayilan, kesistigi tum govdeleri
        comp.features.extrudeFeatures.add(cut)   # kesiyordu (ust uste duran diger kareler ve taban)
    return body, z_top


def build(geo, root, design=None):
    """geo["show_plates"]: taban ve basma plakasi da cizilsin mi (varsayilan: sadece yay)."""
    R_plate = max(geo["D1"], geo["D2"]) / 2 + geo["d"] + 4
    plates = bool(geo.get("show_plates"))

    if plates:
        base = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        base.component.name = "Taban"
        _plate(base.component, R_plate, -PLATE_T, PLATE_T, "Taban")
        base.isGrounded = True

    frames = []
    for k in range(FRAMES):
        delta = geo["y_work"] * k / (FRAMES - 1)
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = f"Yay {k} ({delta:.1f} mm)"
        _, z_top = _spring_body(occ.component, geo, delta)
        if plates:
            _plate(occ.component, R_plate, z_top, PLATE_T, "Basma plakası")
        occ.isGrounded = True
        occ.isLightBulbOn = (k == 0)
        frames.append(occ)

    if design is not None:
        old = design.attributes.itemByName(MARK_GROUP, "spring_frames")
        if old:
            old.deleteMe()
        design.attributes.add(MARK_GROUP, "spring_frames", str(FRAMES))
    return frames
