"""
H1: Draws a bolted joint in Fusion 360 (runs inside Fusion only).

Two round flanges (each half the grip) with clearance holes (ISO 273 medium) on the bolt circle; one bolt
component (hex head 0.7 d, shank, cosmetic ISO thread over the threaded length) and one hex nut component,
placed N times as occurrences of the same components. A static joint: no motion joints.
"""

import math

import adsk.core
import adsk.fusion

import bolted
from fusion_lib.fourbar import mm

HEAD = 0.7                      # hex head height, in d


def _hexagon(sk, to, af):
    R = af / math.sqrt(3)
    pts = [to(R * math.cos(math.pi / 6 + k * math.pi / 3), R * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for p in pts[2:]:
        prev = lines.addByTwoPoints(prev, p).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def _sketch_at(comp, z, name):
    if z:
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
        plane = comp.constructionPlanes.add(pin)
    else:
        plane = comp.xYConstructionPlane
    sk = comp.sketches.add(plane)
    sk.name = name
    return sk, (lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z))))


def _extrude(comp, prof, h, op=adsk.fusion.FeatureOperations.NewBodyFeatureOperation, bodies=None):
    ext = comp.features.extrudeFeatures
    fi = ext.createInput(prof, op)
    fi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(h)))
    if bodies:
        fi.participantBodies = bodies
    return ext.add(fi)


def _thread(comp, body, d, pitch, length):
    """Cosmetic ISO thread on the shank's cylindrical face (visual only: failures are ignored)."""
    try:
        face = next(f for f in body.faces if isinstance(f.geometry, adsk.core.Cylinder)
                    and abs(f.geometry.radius - mm(d / 2)) < 1e-4)
        q = comp.features.threadFeatures.threadDataQuery
        ttype = "ISO Metric profile"
        desig = next((s for s in q.allDesignations(ttype, q.allSizes(ttype)[0]) if s), None)
        size = next(s for s in q.allSizes(ttype) if abs(float(s) - d) < 1e-6)
        desig = next(s for s in q.allDesignations(ttype, size) if s.endswith(f"x{pitch:g}"))
        cls = q.allClasses(False, ttype, desig)[0]
        info = comp.features.threadFeatures.createThreadInfo(False, ttype, desig, cls)
        faces = adsk.core.ObjectCollection.create()
        faces.add(face)
        ti = comp.features.threadFeatures.createInput(faces, info)
        ti.isFullLength = False
        ti.threadLength = adsk.core.ValueInput.createByReal(mm(length))
        ti.isModeled = False
        ti.threadLocation = adsk.fusion.ThreadLocations.LowEndThreadLocation   # at the free end, not under the head
        comp.features.threadFeatures.add(ti)
    except Exception:
        pass


def build(g, root, design=None):
    d, N, L, grip, nut_h, af, bc, pitch = g["d"], g["N"], g["L"], g["grip"], g["nut_h"], g["af"], g["bolt_circle"], g["pitch"]
    hole = d + (1.0 if d <= 12 else 2.0 if d <= 24 else 3.0)         # ISO 273 medium clearance
    D = bc + 2 * af + 4
    New, Cut = adsk.fusion.FeatureOperations.NewBodyFeatureOperation, adsk.fusion.FeatureOperations.CutFeatureOperation

    plates = []
    for k, z0 in enumerate((0.0, grip / 2)):
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        c = occ.component
        c.name = f"Plate {k + 1}"
        sk, to = _sketch_at(c, z0, "Flange")
        sk.sketchCurves.sketchCircles.addByCenterRadius(to(0, 0), mm(D / 2))
        body = _extrude(c, sk.profiles.item(0), grip / 2).bodies.item(0)
        body.name = f"Flange {k + 1}"
        body.opacity = 0.45                               # the bolts and nuts stay visible
        hs, ht = _sketch_at(c, z0, "Bolt holes")
        for i in range(N):
            a = 2 * math.pi * i / N
            hs.sketchCurves.sketchCircles.addByCenterRadius(ht(bc / 2 * math.cos(a), bc / 2 * math.sin(a)), mm(hole / 2))
        col = adsk.core.ObjectCollection.create()
        for p in hs.profiles:
            col.add(p)
        _extrude(c, col, grip / 2, Cut, [body])
        occ.isGrounded = True
        plates.append(occ)

    def at(i, z):
        a = 2 * math.pi * i / N
        mt = adsk.core.Matrix3D.create()
        mt.translation = adsk.core.Vector3D.create(mm(bc / 2 * math.cos(a)), mm(bc / 2 * math.sin(a)), mm(z))
        return mt

    # occurrences are created where they belong (moving one afterwards does not take effect reliably)
    bolt_occ = root.occurrences.addNewComponent(at(0, grip))
    bc_ = bolt_occ.component
    bc_.name = f"Bolt M{d}x{L}"
    sk, to = _sketch_at(bc_, 0, "Head")
    _hexagon(sk, to, af)
    body = _extrude(bc_, sk.profiles.item(0), HEAD * d).bodies.item(0)
    sk, to = _sketch_at(bc_, -L, "Shank")
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(0, 0), mm(d / 2))
    _extrude(bc_, sk.profiles.item(0), L, adsk.fusion.FeatureOperations.JoinFeatureOperation, [body])
    body.name = f"Bolt M{d}x{L}"
    _thread(bc_, body, d, pitch, min(bolted.thread_length(L, d), L))

    nut_occ = root.occurrences.addNewComponent(at(0, 0.0))
    nc = nut_occ.component
    nc.name = f"Hex nut M{d}"
    sk, to = _sketch_at(nc, -nut_h, "Nut")
    _hexagon(sk, to, af)
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(0, 0), mm(d / 2))
    prof = next(p for p in sk.profiles if p.profileLoops.count == 2)
    _extrude(nc, prof, nut_h).bodies.item(0).name = f"Nut M{d}"

    bolt_occ.isGrounded = nut_occ.isGrounded = True
    for i in range(1, N):
        root.occurrences.addExistingComponent(bolt_occ.component, at(i, grip)).isGrounded = True
        root.occurrences.addExistingComponent(nut_occ.component, at(i, 0.0)).isGrounded = True
    return plates
