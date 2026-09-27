"""
G3: Builds a roller chain drive or a V-belt drive in Fusion 360 (runs inside Fusion only).

The small wheel sits at the origin, the large one at (C, 0); the input one is the "Driver" (joint
Ground-Driver, the add-in's input), the other the "Driven". A motion link turns the driven wheel at the
tooth / diameter ratio in the same direction (the sense is checked by a trial turn).
Chain: sprockets with ANSI-style seats (seat arc + straight flanks, one gap cut and patterned); the chain
is drawn as its link-plate bands (inner + outer plate) on both sides of each tooth row, with pin heads,
and stays still while the sprockets turn (the plates are outside the teeth's width, so nothing collides).
Belt: sheaves revolved from their groove section (36 deg grooves), one belt per groove swept along the
pitch path; a belt looks the same when it moves, so it stays still too.
"""

import math

import adsk.core
import adsk.fusion

import chain
from fusion_lib.fourbar import mm
from fusion_lib.gear import _bore_edge
from fusion_lib.marker import marker

PLATE_T = 3.0            # base plate
GAP = 0.5
TOOTH_W = 0.88           # sprocket tooth width, share of the chain's inner width
GROOVE = math.radians(18.0)   # half the groove angle
GROOVE_CLEAR = 0.2       # belt side to groove wall
SHAFT_OUT = 10.0         # shaft beyond the wheel's top


def _plane(comp, z):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    return comp.constructionPlanes.add(pin)


def _sketch(comp, z, name):
    sk = comp.sketches.add(_plane(comp, z) if z else comp.xYConstructionPlane)
    sk.name = name
    sk.arePointsShown = False
    to = lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z)))
    return sk, to


def _loop(sk, to, r1, r2, C):
    """Closed open-drive loop (two arcs + two spans) around r1 at the origin and r2 at (C, 0)."""
    p = chain.open_path(r1, r2, C)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    top = lines.addByTwoPoints(to(*p["u1"]), to(*p["u2"]))
    a2 = arcs.addByThreePoints(top.endSketchPoint, to(*p["far2"]), to(*p["l2"]))
    bot = lines.addByTwoPoints(_end_near(a2, to(*p["l2"])), to(*p["l1"]))
    arcs.addByThreePoints(bot.endSketchPoint, to(*p["far1"]), top.startSketchPoint)
    return p


def _end_near(arc, pt):
    """Fusion orders an arc's ends by its own direction, not by how it was drawn: take the end at pt."""
    s, e = arc.startSketchPoint, arc.endSketchPoint
    return s if s.geometry.distanceTo(pt) < e.geometry.distanceTo(pt) else e


def _extrude(comp, profile, h, op, bodies=None):
    ext = comp.features.extrudeFeatures
    fi = ext.createInput(profile, op)
    fi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(h)))
    if bodies:
        fi.participantBodies = bodies
    return ext.add(fi)


def _disk(comp, r, z, h, name, op=adsk.fusion.FeatureOperations.NewBodyFeatureOperation, bodies=None):
    sk, to = _sketch(comp, z, name)
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(0, 0), mm(r))
    return _extrude(comp, sk.profiles.item(0), h, op, bodies)


def _shaft(comp, bore, z_bottom, top):
    body = _disk(comp, bore / 2, z_bottom, top - z_bottom, "Shaft").bodies.item(0)
    body.name = "Shaft"


def _sprocket(comp, g, N, bore, z_bottom):
    """Sprocket with one tooth row per strand; the rows stand on a hub below the chain plates."""
    p, Dr, W, pt, rows = g["p"], g["Dr"], g["W"], g["pt"], g["strands"]
    tw = TOOTH_W * W
    width = tw + (rows - 1) * pt
    gap = chain.tooth_gap(p, N, Dr)
    New, Join, Cut = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                      adsk.fusion.FeatureOperations.JoinFeatureOperation,
                      adsk.fusion.FeatureOperations.CutFeatureOperation)
    hub_r = gap["R"] - chain.PLATE_H * p / 2 - 1.0
    body = _disk(comp, hub_r if rows > 1 else gap["Ro"], 0, width, "Hub").bodies.item(0)
    if rows > 1:
        for k in range(rows):
            _disk(comp, gap["Ro"], k * pt, tw, f"Row {k + 1}", Join, [body])
    sk, to = _sketch(comp, 0, "Tooth gap (seat + flanks)")
    lines = sk.sketchCurves.sketchLines
    f1 = lines.addByTwoPoints(to(*gap["out_plus"]), to(*gap["seat_plus"]))
    arc = sk.sketchCurves.sketchArcs.addByThreePoints(f1.endSketchPoint, to(*gap["bottom"]), to(*gap["seat_minus"]))
    f2 = lines.addByTwoPoints(arc.endSketchPoint, to(*gap["out_minus"]))
    lines.addByTwoPoints(f2.endSketchPoint, f1.startSketchPoint)
    cut = _extrude(comp, sk.profiles.item(0), width, Cut, [body])
    ents = adsk.core.ObjectCollection.create()
    ents.add(cut)
    pin = comp.features.circularPatternFeatures.createInput(ents, comp.zConstructionAxis)
    pin.quantity = adsk.core.ValueInput.createByReal(N)
    pin.totalAngle = adsk.core.ValueInput.createByString("360 deg")
    comp.features.circularPatternFeatures.add(pin)
    _disk(comp, bore / 2, 0, width, "Shaft bore", Cut, [body])
    body.name = f"Sprocket z{N}"
    _shaft(comp, bore, z_bottom, width + SHAFT_OUT)
    r1 = gap["R"] - chain.PLATE_H * p / 2 - 2.0                  # inside the chain plates' reach
    if r1 > bore / 2 + 6:
        marker(comp, bore / 2 + 2, r1, width, max(4.0, 0.3 * p))
    return width, gap


def _chain(comp, g, R1, R2, C, z0):
    """Link-plate bands (inner + outer plate as one band) either side of every tooth row, plus pin heads."""
    p, W, pt, rows, Dr = g["p"], g["W"], g["pt"], g["strands"], g["Dr"]
    tw = TOOTH_W * W
    h = chain.PLATE_H * p / 2
    t = 2 * chain.PLATE_T * p if rows == 1 else min(2 * chain.PLATE_T * p, (pt - W) / 2 - 0.2)
    New, Join = adsk.fusion.FeatureOperations.NewBodyFeatureOperation, adsk.fusion.FeatureOperations.JoinFeatureOperation
    bands = []
    for k in range(rows):
        zc = z0 + tw / 2 + k * pt
        for side, zb in ((-1, zc - W / 2 - t), (1, zc + W / 2)):
            sk, to = _sketch(comp, zb, f"Plates row {k + 1} {'bottom' if side < 0 else 'top'}")
            _loop(sk, to, R1 + h, R2 + h, C)
            _loop(sk, to, R1 - h, R2 - h, C)
            ring = next(pr for pr in sk.profiles if pr.profileLoops.count == 2)
            body = _extrude(comp, ring, t, New).bodies.item(0)
            body.name = f"Plates {k + 1}{'a' if side < 0 else 'b'}"
            bands.append((zb, body))
    # pin heads on both outer faces, one per pitch along the pitch path
    path = chain.open_path(R1, R2, C)
    n = max(4, int(round(path["length"] / p)))
    pts = [_along(path, R1, R2, C, path["length"] * i / n) for i in range(n)]
    for zb, body, up in ((bands[0][0], bands[0][1], False), (bands[-1][0] + t, bands[-1][1], True)):
        sk, to = _sketch(comp, zb, "Pin heads")
        for x, y in pts:
            sk.sketchCurves.sketchCircles.addByCenterRadius(to(x, y), mm(0.3 * Dr))
        prof = adsk.core.ObjectCollection.create()
        for pr in sk.profiles:
            prof.add(pr)
        ext = comp.features.extrudeFeatures
        fi = ext.createInput(prof, Join)
        fi.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(mm(0.08 * p))),
                            adsk.fusion.ExtentDirections.PositiveExtentDirection if up
                            else adsk.fusion.ExtentDirections.NegativeExtentDirection)
        fi.participantBodies = [body]
        ext.add(fi)


def _along(path, r1, r2, C, s):
    """Point at arc length s along the loop: upper span, large arc, lower span, small arc."""
    b = math.asin((r2 - r1) / C)
    span = C * math.cos(b)
    a2, a1 = r2 * (math.pi + 2 * b), r1 * (math.pi - 2 * b)
    (x1, y1), (x2, y2) = path["u1"], path["u2"]
    if s < span:
        f = s / span
        return x1 + f * (x2 - x1), y1 + f * (y2 - y1)
    s -= span
    if s < a2:
        ang = math.pi / 2 + b - s / r2
        return C + r2 * math.cos(ang), r2 * math.sin(ang)
    s -= a2
    (x1, y1), (x2, y2) = path["l2"], path["l1"]
    if s < span:
        f = s / span
        return x1 + f * (x2 - x1), y1 + f * (y2 - y1)
    s -= span
    ang = -(math.pi / 2 + b) - s / r1
    return r1 * math.cos(ang), r1 * math.sin(ang)


def _sheave(comp, g, d, bore, z_bottom):
    """V-grooved sheave revolved from its section: groove top width a + 2 clearances at the rim."""
    b, a, e, n = g["thick"], g["top"], g["e"], g["belts"]
    f = (g["width"] - (n - 1) * e) / 2
    Ro = d / 2 + 0.3 * b
    wt, dep = a + 2 * GROOVE_CLEAR / math.cos(GROOVE), b + 3.0
    wb = wt - 2 * dep * math.tan(GROOVE)
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = "Sheave section"
    to = lambda r, z: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(r), 0, mm(z)))
    pts = [(bore / 2, 0), (Ro, 0)]
    for k in range(n):
        zk = f + k * e
        pts += [(Ro, zk - wt / 2), (Ro - dep, zk - wb / 2), (Ro - dep, zk + wb / 2), (Ro, zk + wt / 2)]
    pts += [(Ro, g["width"]), (bore / 2, g["width"])]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to(*pts[0]), to(*pts[1]))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, to(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    rev = comp.features.revolveFeatures
    ri = rev.createInput(sk.profiles.item(0), comp.zConstructionAxis, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    body = rev.add(ri).bodies.item(0)
    body.name = f"Sheave Ø{d:.0f}"
    _shaft(comp, bore, z_bottom, g["width"] + SHAFT_OUT)
    marker(comp, bore / 2 + 2, Ro - 3, g["width"], max(5.0, Ro / 10))
    return Ro, wt, f


def _belts(comp, g, r1, r2, C, z0, Ro1, wt, f):
    """One belt per groove: a trapezoid (36 deg sides, clear of the walls) swept along the pitch loop."""
    b, e = g["thick"], g["e"]
    top_half = wt / 2 - GROOVE_CLEAR / math.cos(GROOVE)
    bot_half = top_half - b * math.tan(GROOVE)
    for k in range(g["belts"]):
        zk = z0 + f + k * e
        sk, to = _sketch(comp, zk, f"Belt {k + 1} path")
        _loop(sk, to, r1, r2, C)
        prof = comp.sketches.add(comp.xZConstructionPlane)
        prof.name = f"Belt {k + 1} section"
        at = lambda x, z: prof.modelToSketchSpace(adsk.core.Point3D.create(mm(x), 0, mm(z)))
        q = [(-Ro1, zk - top_half), (-(Ro1 - b), zk - bot_half), (-(Ro1 - b), zk + bot_half), (-Ro1, zk + top_half)]
        lines = prof.sketchCurves.sketchLines
        l0 = lines.addByTwoPoints(at(*q[0]), at(*q[1]))
        l1 = lines.addByTwoPoints(l0.endSketchPoint, at(*q[2]))
        l2 = lines.addByTwoPoints(l1.endSketchPoint, at(*q[3]))
        lines.addByTwoPoints(l2.endSketchPoint, l0.startSketchPoint)
        path = comp.features.createPath(sk.sketchCurves.item(0), True)
        sw = comp.features.sweepFeatures
        si = sw.createInput(prof.profiles.item(0), path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        si.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
        sw.add(si).bodies.item(0).name = f"Belt {k + 1}"


def _angle(occ):
    m = occ.transform2
    return math.atan2(m.getCell(1, 0), m.getCell(0, 0))


def build(g, root, design=None):
    is_chain = g["type"] == "chain_drive"
    C = g["C"]
    small_in = g["small_is_input"]
    if is_chain:
        p = g["p"]
        R1, R2 = chain.pitch_diameter(p, g["N1"]) / 2, chain.pitch_diameter(p, g["N2"]) / 2
        Ro1, Ro2 = chain.outside_diameter(p, g["N1"]) / 2, chain.outside_diameter(p, g["N2"]) / 2
        t = 2 * chain.PLATE_T * p if g["strands"] == 1 else min(2 * chain.PLATE_T * p, (g["pt"] - g["W"]) / 2 - 0.2)
        z0 = PLATE_T + GAP + t + (1 - TOOTH_W) * g["W"] / 2 + 0.08 * p + 0.5   # lowest plates + pin heads clear
        ratio = g["N1"] / g["N2"]
    else:
        R1, R2 = g["d"] / 2, g["D"] / 2
        Ro1, Ro2 = R1 + 0.3 * g["thick"], R2 + 0.3 * g["thick"]
        z0 = PLATE_T + GAP
        ratio = g["d"] / g["D"]
    P = adsk.core.Point3D.create

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    sk = gc.sketches.add(gc.xYConstructionPlane)
    m = 10.0
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-Ro1 - m), mm(-Ro2 - m), 0), P(mm(C + Ro2 + m), mm(Ro2 + m), 0))
    for x, bore in ((0.0, g["bore1"]), (C, g["bore2"])):
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(mm(x), 0, 0), mm(bore / 2 + 0.5))
    plate = max(sk.profiles, key=lambda pr: pr.areaProperties().area)
    _extrude(gc, plate, PLATE_T, adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = "Base"
    ground.isGrounded = True

    occs = {}
    for key, x, N_or_d, bore in (("small", 0.0, g["N1"] if is_chain else g["d"], g["bore1"]),
                                 ("large", C, g["N2"] if is_chain else g["D"], g["bore2"])):
        mt = adsk.core.Matrix3D.create()
        mt.translation = adsk.core.Vector3D.create(mm(x), 0, mm(z0))
        occ = root.occurrences.addNewComponent(mt)
        occ.component.name = "Driver" if (key == "small") == small_in else "Driven"
        if is_chain:
            width, gap = _sprocket(occ.component, g, N_or_d, bore, -z0)
        else:
            Ro, wt, f = _sheave(occ.component, g, N_or_d, bore, -z0)
            width = g["width"]
        occs[key] = (occ, bore)

    band = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    band.component.name = "Chain" if is_chain else "Belt"
    if is_chain:
        _chain(band.component, g, R1, R2, C, z0)
    else:
        _belts(band.component, g, R1, R2, C, z0, Ro1, wt, f)
    band.isGrounded = True

    joints = {}
    for key in ("small", "large"):
        occ, bore = occs[key]
        geo = adsk.fusion.JointGeometry.createByCurve(_bore_edge(occ, bore, width),
                                                      adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(ground, occ, geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        j.name = f"Ground-{occ.component.name}"
        joints[key] = j

    drv, dvn = ("small", "large") if small_in else ("large", "small")
    per_turn = 360.0 * (ratio if small_in else 1 / ratio)     # driven degrees per driver turn
    mo = adsk.fusion.RevoluteJointMotion.cast(joints[drv].jointMotion)
    for reverse in (False, True):
        li = root.motionLinks.createInput(joints[drv], joints[dvn])
        li.valueOne = adsk.core.ValueInput.createByString("360 deg")
        li.valueTwo = adsk.core.ValueInput.createByString(f"{per_turn} deg")
        li.isReversed = reverse
        link = root.motionLinks.add(li)
        a_drv, a_dvn = _angle(occs[drv][0]), _angle(occs[dvn][0])
        mo.rotationValue = math.radians(10.0)
        d_drv = _angle(occs[drv][0]) - a_drv
        d_dvn = _angle(occs[dvn][0]) - a_dvn
        mo.rotationValue = 0.0
        if d_drv * d_dvn > 0 and abs(d_dvn / d_drv - per_turn / 360.0) < 1e-3:
            return occs
        link.deleteMe()
    raise RuntimeError("Drive: the driven wheel does not follow the driver in the same sense")
