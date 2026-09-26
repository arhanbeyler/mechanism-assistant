"""
H5b: Draws a rigid flange coupling in Fusion 360 (runs inside Fusion only).

Two flanged hubs (hub 2d x 1.5d with a flange 4d x 0.5d) meeting at z = 0 on two shafts, joined by N bolts
with nuts on the 3d bolt circle (clearance holes); the bolt and the nut are one component each, placed N
times (as in fusion_lib.bolted). Static (the joint is the coupling).
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.bolted import HEAD, _hexagon, _sketch_at, _thread
from fusion_lib.fourbar import mm


def _half(root, name, g, sign):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = occ.component
    c.name = name
    d, D, L, Df, t, Db = g["d"], g["D_hub"], g["L_hub"], g["D_flange"], g["t"], g["D_bolt"]
    sk = c.sketches.add(c.xZConstructionPlane)
    to = lambda r, z: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(r), 0, mm(z)))
    pts = [(d / 2, 0), (Df / 2, 0), (Df / 2, sign * t), (D / 2, sign * t), (D / 2, sign * L), (d / 2, sign * L)]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to(*pts[0]), to(*pts[1]))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, to(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    ri = c.features.revolveFeatures.createInput(sk.profiles.item(0), c.zConstructionAxis,
                                                adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    body = c.features.revolveFeatures.add(ri).bodies.item(0)
    body.name = f"{name} Ø{Df:g}"
    hole = g["bolt"]["d"] + (1.0 if g["bolt"]["d"] <= 12 else 2.0)
    hs, ht = _sketch_at(c, 0.0, "Bolt holes")
    for i in range(g["N"]):
        a = 2 * math.pi * i / g["N"]
        hs.sketchCurves.sketchCircles.addByCenterRadius(ht(Db / 2 * math.cos(a), Db / 2 * math.sin(a)), mm(hole / 2))
    col = adsk.core.ObjectCollection.create()
    for p in hs.profiles:
        if p.profileLoops.count == 1 and p.areaProperties().area < math.pi * (mm(hole)) ** 2:
            col.add(p)
    ext = c.features.extrudeFeatures
    fi = ext.createInput(col, adsk.fusion.FeatureOperations.CutFeatureOperation)
    fi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(sign * t)))
    fi.participantBodies = [body]
    ext.add(fi)
    sh, st = _sketch_at(c, -L - 30 if sign < 0 else 0.0, "Shaft")     # each shaft ends at the joint face
    sh.sketchCurves.sketchCircles.addByCenterRadius(st(0, 0), mm(d / 2))
    shaft = c.features.extrudeFeatures.addSimple(sh.profiles.item(0), adsk.core.ValueInput.createByReal(mm(L + 30)),
                                                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    shaft.name = "Shaft"
    occ.isGrounded = True
    return occ


def build(g, root, design=None):
    _half(root, "Hub A", g, -1)
    _half(root, "Hub B", g, 1)
    b, t, Db, N = g["bolt"], g["t"], g["D_bolt"], g["N"]
    d, L = b["d"], b["L"]

    def at(i, z):
        a = 2 * math.pi * i / N
        mt = adsk.core.Matrix3D.create()
        mt.translation = adsk.core.Vector3D.create(mm(Db / 2 * math.cos(a)), mm(Db / 2 * math.sin(a)), mm(z))
        return mt

    # the bolt's head sits on hub A's flange face (z = -t) pointing -z, the nut on hub B's (z = +t)
    flip = adsk.core.Matrix3D.create()
    flip.setToRotation(math.pi, adsk.core.Vector3D.create(1, 0, 0), adsk.core.Point3D.create(0, 0, 0))

    def at_bolt(i):
        m = at(i, -t)
        f = flip.copy()
        f.transformBy(m)
        return f

    bolt = root.occurrences.addNewComponent(at_bolt(0))
    bc = bolt.component
    bc.name = f"Bolt M{d}x{L}"
    sk, to = _sketch_at(bc, 0, "Head")
    _hexagon(sk, to, b["af"])
    body = bc.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(HEAD * d)),
                                                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    sk, to = _sketch_at(bc, -L, "Shank")
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(0, 0), mm(d / 2))
    fi = bc.features.extrudeFeatures.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    fi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(L)))
    fi.participantBodies = [body]
    bc.features.extrudeFeatures.add(fi)
    body.name = f"Bolt M{d}x{L}"
    _thread(bc, body, d, b["pitch"], min(2 * d + 6, L))
    nut = root.occurrences.addNewComponent(at(0, t + b["nut_h"]))
    nc = nut.component
    nc.name = f"Hex nut M{d}"
    sk, to = _sketch_at(nc, -b["nut_h"], "Nut")
    _hexagon(sk, to, b["af"])
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(0, 0), mm(d / 2))
    prof = next(p for p in sk.profiles if p.profileLoops.count == 2)
    nc.features.extrudeFeatures.addSimple(prof, adsk.core.ValueInput.createByReal(mm(b["nut_h"])),
                                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = f"Nut M{d}"
    bolt.isGrounded = nut.isGrounded = True
    for i in range(1, N):
        root.occurrences.addExistingComponent(bc, at_bolt(i)).isGrounded = True
        root.occurrences.addExistingComponent(nc, at(i, t + b["nut_h"])).isGrounded = True
    return bolt
