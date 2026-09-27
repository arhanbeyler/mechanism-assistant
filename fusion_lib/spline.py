"""
H3b / H3c: Draws a spline in Fusion 360 (runs inside Fusion only): the shaft and a translucent hub.
Straight-sided (ISO 14): minor-diameter core with N teeth B wide up to the major diameter, the hub's hole 0.1 mm
larger all round.  Involute (DIN 5480): 30° involute flanks, the hub's tooth spaces wider by a small flank
clearance, its tips and roots at the standard's diameters (0.1 m / 0.2 m radial clearance).
A fixed hub stays; a sliding hub (use 'sliding' / 'sliding_loaded') is drawn as FRAMES frames along a shaft one hub
length longer, shown in turn by the add-in.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm
from fusion_lib.spring import frame_occurrences

CLEAR = 0.1
FRAMES = 6
FLANK_CLEAR = 0.06          # involute: added to each flank along the arc (about 0.05 mm normal to it)


def _involute_outline(sk, z, r_lo, r_hi, theta, n=6):
    """Closed outline of z teeth (tooth k on the angle 2 pi k / z): involute flanks from r_lo to r_hi, whose half
    angular thickness at radius rho is theta(rho), tip arcs on r_hi and root arcs on r_lo."""
    P = lambda r, a: adsk.core.Point3D.create(mm(r * math.cos(a)), mm(r * math.sin(a)), 0)
    arcs, splines = sk.sketchCurves.sketchArcs, sk.sketchCurves.sketchFittedSplines
    radii = [r_lo + (r_hi - r_lo) * i / n for i in range(n + 1)]

    def end_near(curve, pt):
        e = curve.endSketchPoint
        return e if e.geometry.distanceTo(pt) < 1e-6 else curve.startSketchPoint

    first = prev = None
    for k in range(z):
        c = 2 * math.pi * k / z
        pts = adsk.core.ObjectCollection.create()
        for i, r in enumerate(radii):
            pts.add(prev if (i == 0 and prev) else P(r, c - theta(r)))
        right = splines.add(pts)
        first = first or right.startSketchPoint
        tip = arcs.addByThreePoints(right.endSketchPoint, P(r_hi, c), P(r_hi, c + theta(r_hi)))
        pts = adsk.core.ObjectCollection.create()
        for i, r in enumerate(reversed(radii)):
            pts.add(end_near(tip, P(r_hi, c + theta(r_hi))) if i == 0 else P(r, c + theta(r)))
        left = splines.add(pts)
        cn = 2 * math.pi * (k + 1) / z
        nxt = P(r_lo, cn - theta(r_lo))
        root = arcs.addByThreePoints(left.endSketchPoint, P(r_lo, c + math.pi / z), first if k == z - 1 else nxt)
        prev = end_near(root, nxt) if k < z - 1 else None
    return sk


def _build_involute(g, root, design):
    z, dB, L = g["z"], g["dB"], g["L"]
    inv = lambda a: math.tan(a) - a
    a0 = math.radians(30.0)

    def theta(rho, extra=0.0):
        a = math.acos(min(1.0, g["db"] / (2 * rho)))
        return g["s"] / g["d"] + inv(a0) - inv(a) + extra / rho

    shaft = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    sc = shaft.component
    sc.name = "Shaft"
    pin = sc.constructionPlanes.createInput()
    pin.setByOffset(sc.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(-20)))
    sk = sc.sketches.add(sc.constructionPlanes.add(pin))
    _involute_outline(sk, z, g["df1"] / 2, g["da1"] / 2, theta)
    prof = max(sk.profiles, key=lambda p: p.areaProperties().area)
    name = f"W {dB:g} x {g['m']:g} x {z}"
    sc.features.extrudeFeatures.addSimple(prof, adsk.core.ValueInput.createByReal(mm(L + 40 + _travel(g))),
                                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = f"Spline shaft {name}"
    shaft.isGrounded = True
    r_hi = g["df2"] / 2
    while theta(r_hi, FLANK_CLEAR) * r_hi < 0.1:            # the hub's space must not close into a point
        r_hi -= 0.05
    _hubs(root, design, g, lambda sk: _involute_outline(sk, z, g["da2"] / 2, r_hi, lambda r: theta(r, FLANK_CLEAR)))
    return shaft


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


def _travel(g):
    return g["L"] if g.get("use") in ("sliding", "sliding_loaded") else 0.0


def _hubs(root, design, g, hole):
    """The hub (translucent) with its spline hole drawn by hole(sketch); a sliding hub as frames along the shaft."""
    L, Dh = g["L"], g["hub_od"]

    def make(comp, z0):
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
        sk = comp.sketches.add(comp.constructionPlanes.add(pin))
        sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), mm(Dh / 2))
        hole(sk)
        ring = max((p for p in sk.profiles if p.profileLoops.count == 2), key=lambda p: p.areaProperties().area)
        body = comp.features.extrudeFeatures.addSimple(ring, adsk.core.ValueInput.createByReal(mm(L)),
                                                       adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
        body.name = f"Hub Ø{Dh:g} x {L:g}"
        body.opacity = 0.5
        return f"{z0:.0f} mm"

    travel = _travel(g)
    if travel:
        return frame_occurrences(root, design, FRAMES, lambda c, f: make(c, travel * f), "Hub")
    hub = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    hub.component.name = "Hub"
    make(hub.component, 0.0)
    hub.isGrounded = True
    return [hub]


def build(g, root, design=None):
    if g.get("profile") == "involute":
        return _build_involute(g, root, design)
    N, d, D, B, L = g["N"], g["d"], g["D"], g["B"], g["L"]
    shaft = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    sc = shaft.component
    sc.name = "Shaft"
    pin = sc.constructionPlanes.createInput()
    pin.setByOffset(sc.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(-20)))
    sk = sc.sketches.add(sc.constructionPlanes.add(pin))
    _spline_profile(sk, N, d / 2, D / 2, B)
    prof = min(sk.profiles, key=lambda p: -p.areaProperties().area)
    sc.features.extrudeFeatures.addSimple(prof, adsk.core.ValueInput.createByReal(mm(L + 40 + _travel(g))),
                                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = f"Spline shaft {N}x{d}x{D}"
    shaft.isGrounded = True
    _hubs(root, design, g, lambda sk: _spline_profile(sk, N, d / 2 + CLEAR, D / 2 + CLEAR, B + 2 * CLEAR))
    return shaft
