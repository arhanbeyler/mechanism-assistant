"""
C10: Basma yayini Fusion 360'ta kurar (sadece Fusion icinde).

API'de yeni "coil" olusturulamadigi icin: tel merkez cizgisi 3B spline (springs.helix_points,
konik yayda degisken yaricap, uclarda kapali sarimlar) -> yolun basinda yola dik duzlemde
tel kesiti (daire) -> sweep -> z = 0 ve z = L0 duzlemleriyle kesme (kare ve taslanmis uclar).
"""

import adsk.core
import adsk.fusion

import springs
from fusion_lib.fourbar import mm

END_GAP = 1.03    # kapali uc sarimlarinda hatve = 1.03 d (tam temas sweep'te kendine degen govde yapiyor)


def build(geo, root):
    d, L0 = geo["d"], geo["L0"]
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Yay"
    P = adsk.core.Point3D.create

    # Gorsel model icin uc sarimlarinda kucuk bosluk; tasarim degerleri (L0, k) degismez
    g = dict(geo)
    g["d"] = d * END_GAP
    g["pitch"] = (L0 - 2 * g["d"]) / geo["Na"]
    pts = springs.helix_points(g, per_turn=20)
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
    plane = comp.constructionPlanes.add(pl_in)
    prof_sk = comp.sketches.add(plane)
    prof_sk.name = "Tel kesiti"
    center = prof_sk.modelToSketchSpace(P(mm(pts[0][0]), mm(pts[0][1]), mm(pts[0][2])))
    prof_sk.sketchCurves.sketchCircles.addByCenterRadius(center, mm(d / 2))

    sw_in = comp.features.sweepFeatures.createInput(prof_sk.profiles.item(0), path,
                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = comp.features.sweepFeatures.add(sw_in).bodies.item(0)
    body.name = "Yay"

    # Taslama: alt (z < 0) ve ust (z > z_top) kisimlari kes
    R = max(geo["D1"], geo["D2"]) / 2 + d * 2
    for z0, direction in ((0.0, -1.0), (z_top, 1.0)):
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
        cp = comp.constructionPlanes.add(pin)
        sk = comp.sketches.add(cp)
        sk.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-R), mm(-R), 0), P(mm(R), mm(R), 0))
        comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(direction * 2 * d)),
                                                adsk.fusion.FeatureOperations.CutFeatureOperation)
    occ.isGrounded = True
    return occ
