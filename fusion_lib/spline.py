"""
H3b: Draws a straight-sided spline in Fusion 360 (runs inside Fusion only): the shaft (minor-diameter core with
N teeth B wide up to the major diameter) and a translucent hub whose spline hole is 0.1 mm larger all round.
Nothing moves.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

CLEAR = 0.1


def _spline_profile(sk, N, r_minor, r_major, B, rot=0.0):
    """Closed outline: minor-circle arcs between the teeth and the teeth's straight flanks up to the major circle."""
    P = lambda x, y: adsk.core.Point3D.create(mm(x), mm(y), 0)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    pts = []
    for k in range(N):
        c = rot + 2 * math.pi * k / N
        u, v = (math.cos(c), math.sin(c)), (-math.sin(c), math.cos(c))
        h = B / 2
        # tooth k: flank points at the minor and major circles (straight flanks parallel to the tooth's axis)
        xm = math.sqrt(max(r_minor ** 2 - h * h, 0.0))
        xM = math.sqrt(r_major ** 2 - h * h)
        pts.append([(xm * u[0] - h * v[0], xm * u[1] - h * v[1]), (xM * u[0] - h * v[0], xM * u[1] - h * v[1]),
                    (xM * u[0] + h * v[0], xM * u[1] + h * v[1]), (xm * u[0] + h * v[0], xm * u[1] + h * v[1]), c])
    first = prev = None
    for k in range(N):
        a0, a1, a2, a3, c = pts[k]
        l1 = lines.addByTwoPoints(prev if prev else P(*a0), P(*a1))
        first = first or l1.startSketchPoint
        mid = (r_major * math.cos(c), r_major * math.sin(c))
        top = arcs.addByThreePoints(l1.endSketchPoint, P(*mid), P(*a2))
        e = top.endSketchPoint if top.endSketchPoint.geometry.distanceTo(P(*a2)) < 1e-6 else top.startSketchPoint
        l2 = lines.addByTwoPoints(e, P(*a3))
        nxt = pts[(k + 1) % N]
        cm = c + math.pi / N
        midm = (r_minor * math.cos(cm), r_minor * math.sin(cm))
        end = first if k == N - 1 else P(*nxt[0])
        root = arcs.addByThreePoints(l2.endSketchPoint, P(*midm), end)
        prev = root.endSketchPoint if (k < N - 1 and root.endSketchPoint.geometry.distanceTo(P(*nxt[0])) < 1e-6) \
            else root.startSketchPoint
    return sk


def build(g, root, design=None):
    N, d, D, B, L, Dh = g["N"], g["d"], g["D"], g["B"], g["L"], g["hub_od"]
    shaft = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    sc = shaft.component
    sc.name = "Shaft"
    pin = sc.constructionPlanes.createInput()
    pin.setByOffset(sc.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(-20)))
    sk = sc.sketches.add(sc.constructionPlanes.add(pin))
    _spline_profile(sk, N, d / 2, D / 2, B)
    prof = min(sk.profiles, key=lambda p: -p.areaProperties().area)
    sc.features.extrudeFeatures.addSimple(prof, adsk.core.ValueInput.createByReal(mm(L + 40)),
                                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = f"Spline shaft {N}x{d}x{D}"
    shaft.isGrounded = True
    hub = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    hc = hub.component
    hc.name = "Hub"
    sk = hc.sketches.add(hc.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), mm(Dh / 2))
    _spline_profile(sk, N, d / 2 + CLEAR, D / 2 + CLEAR, B + 2 * CLEAR)
    ring = max((p for p in sk.profiles if p.profileLoops.count == 2), key=lambda p: p.areaProperties().area)
    body = hc.features.extrudeFeatures.addSimple(ring, adsk.core.ValueInput.createByReal(mm(L)),
                                                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = f"Hub Ø{Dh:g} x {L:g}"
    body.opacity = 0.5
    hub.isGrounded = True
    return shaft
