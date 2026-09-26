"""
C11b: Iki yatakli mili Fusion 360'ta kurar (sadece Fusion icinde).

Geometri shafts.layout/profile'dan (saf Python, test edilir): x ekseni boyunca donel profil -> revolve.
Gerilme yigilmasi olarak sadece hesaptaki ozellik cizilir: kama yuvasi (parmak freze cebi + DIN 6885
kama), segman kanali, keskin ya da yuvarlatilmis omuz (profilde radyus). Yataklar sabit bilesen;
"Sabit-Mil" donel mafsali eklentinin 1 tur / surekli dondurme dugmeleriyle calisir.
"""

import math

import adsk.core
import adsk.fusion

import shafts
from fusion_lib.fourbar import mm


def _profile_sketch(comp, lay):
    """Donel profil (XY duzlemi, y = yaricap) + eksen cizgisi. Dondurur: (profil, eksen)."""
    P = adsk.core.Point3D.create
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "Mil profili"
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    pts = shafts.profile(lay)
    prev = None                                   # onceki egrinin bitis noktasi (SketchPoint): kapali zincir
    first = None
    for i in range(1, len(pts)):
        item = pts[i]
        if item[0] == "arc":
            _, (cx, cy), (sx, sy), (ex, ey) = item
            rf = math.hypot(sx - cx, sy - cy)
            mid = P(mm(cx - rf / math.sqrt(2)), mm(cy - rf / math.sqrt(2)), 0)   # ic koseye dogru siskin
            a = arcs.addByThreePoints(prev, mid, P(mm(ex), mm(ey), 0))
            prev = a.endSketchPoint if a.startSketchPoint.geometry.isEqualTo(prev.geometry) else a.startSketchPoint
            continue
        if isinstance(pts[i - 1][0], str):        # yaydan sonraki nokta = yay sonu (zaten prev)
            continue
        x0, y0 = pts[i - 1]
        x1, y1 = item
        ln = lines.addByTwoPoints(prev or P(mm(x0), mm(y0), 0), P(mm(x1), mm(y1), 0))
        first = first or ln
        prev = ln.endSketchPoint
    axis = lines.addByTwoPoints(prev, first.startSketchPoint)   # eksen boyunca kapat
    return sk.profiles.item(0), axis


def _obround(comp, z, xc, length, b, name):
    """z yuksekliginde XY'ye paralel duzlemde x yonunde uzun kanal (kama / kama yuvasi) cizimi."""
    P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    sk.name = name
    to = lambda x, y: sk.modelToSketchSpace(P(mm(x), mm(y), mm(z)))
    x0, x1, hb = xc - length / 2 + b / 2, xc + length / 2 - b / 2, b / 2
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    top = lines.addByTwoPoints(to(x0, hb), to(x1, hb))
    bot = lines.addByTwoPoints(to(x0, -hb), to(x1, -hb))
    arcs.addByThreePoints(top.endSketchPoint, to(x1 + hb, 0), bot.endSketchPoint)
    arcs.addByThreePoints(bot.startSketchPoint, to(x0 - hb, 0), top.startSketchPoint)
    return sk.profiles.item(0)


def _extrude(comp, prof, dist_mm, op, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(dist_mm)))
    if body is not None:
        inp.participantBodies = [body]            # sadece mil kesilsin
    return ext.add(inp)


def _bearings(comp, lay, dx=0.0, dy=0.0, suffix=""):
    """Bearings (visual, grounded; catalogue sizes): width B at x = xc + dx, bore = seat diameter, outer Do.
    dx, dy: where the shaft's local frame sits in the assembly (reducer: several shafts)."""
    P = adsk.core.Point3D.create
    for i, (xc, B, Do) in enumerate(lay["bearings"], 1):
        x = xc + dx
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(mm(x)))
        sk = comp.sketches.add(comp.constructionPlanes.add(pin))
        c = sk.modelToSketchSpace(P(mm(x), mm(dy), 0))
        ri = lay["journal"] / 2 + 0.05            # seat diameter; must not touch the shaft
        sk.sketchCurves.sketchCircles.addByCenterRadius(c, mm(Do / 2))
        sk.sketchCurves.sketchCircles.addByCenterRadius(c, mm(ri))
        ring = min(sk.profiles, key=lambda p: abs(p.areaProperties().area - math.pi * (mm(Do / 2) ** 2 - mm(ri) ** 2)))
        ext = comp.features.extrudeFeatures
        inp = ext.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(B)), True)   # total B, centred on x
        name = f"Bearing {i}{suffix} ({lay['bearing_name']})" if lay["bearing_name"] else f"Bearing {i}{suffix}"
        ext.add(inp).bodies.item(0).name = name


def make_shaft(root, lay, name, matrix=None):
    """Shaft component: revolved profile + (keyseat) end-milled pocket and DIN 6885 key.
    Returns (occurrence, shaft body)."""
    occ = root.occurrences.addNewComponent(matrix or adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    prof, axis = _profile_sketch(comp, lay)
    rv = comp.features.revolveFeatures
    rin = rv.createInput(prof, axis, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    rin.setAngleExtent(False, adsk.core.ValueInput.createByReal(2 * math.pi))
    shaft = rv.add(rin).bodies.item(0)
    shaft.name = f"Shaft Ø{lay['d']:g}"
    feat = lay["feature"]
    if feat["kind"] == "keyseat":                 # end-milled keyseat + key
        r = lay["d"] / 2
        pocket = _obround(comp, r, feat["x"], feat["length"], feat["b"], "Keyseat")
        _extrude(comp, pocket, -feat["t1"], adsk.fusion.FeatureOperations.CutFeatureOperation, shaft)
        key = _obround(comp, r - feat["t1"], feat["x"], feat["length"], feat["b"], "Key")
        _extrude(comp, key, feat["h"], adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name =             f"Key {feat['b']}×{feat['h']}×{feat['length']:g} (DIN 6885)"
    return occ, shaft


def revolute_to(root, ground, occ, shaft, lay, name):
    """Revolute joint about the shaft axis: centre of the shaft's end circle at local x = 0."""
    edge = next(e for e in shaft.edges
                if isinstance(e.geometry, adsk.core.Circle3D) and abs(e.geometry.center.x) < 1e-6
                and abs(e.geometry.radius - mm(lay["segments"][0][2])) < 1e-6)
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(occ),
                                                  adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(ground, occ, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    joint = root.asBuiltJoints.add(ji)
    joint.name = name
    return joint


def build(g, root, design=None):
    """g: pipeline's fusion dict (d, feature, layout ...). Returns the shaft occurrence."""
    lay = g.get("layout") or shafts.layout(g["d"], g["feature"], g.get("span_mm"))   # engine's bearing / key
    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    ground.component.name = "Yataklar"
    _bearings(ground.component, lay)
    ground.isGrounded = True
    occ, shaft = make_shaft(root, lay, "Mil")
    revolute_to(root, ground, occ, shaft, lay, "Sabit-Mil")
    return occ
