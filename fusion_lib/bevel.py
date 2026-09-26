"""
G1c: Builds a straight bevel gear pair (shafts at 90 deg) in Fusion 360 (runs inside Fusion only).

Each gear is built with its cone apex at the component origin and its axis on z, the body towards -z.
Tooth: the virtual spur tooth (Tredgold: z / cos(gamma) teeth, module m) drawn on the back-cone plane at the
large end (perpendicular to the pitch-cone element, the virtual gear's centre on the axis), and the same
tooth scaled towards the apex at the small end; a loft between them has straight elements through the apex,
as a straight bevel tooth. The blank is revolved from its section (root cone, back and front faces, a web
and the bore). Tooth patterned z times and joined. The pinion stands on the base plate (vertical axis), the
gear lies with its axis along x; both apexes meet. Joints Ground-Pinion (input) / Ground-Gear, motion link
sense from a trial turn.
"""

import math

import adsk.core
import adsk.fusion

import bevel
import gears
from fusion_lib.fourbar import mm
from fusion_lib.gear import GAP, PLATE_T


def _overlap(b1, b2):
    tb = adsk.fusion.TemporaryBRepManager.get()
    a, c = tb.copy(b1), tb.copy(b2)
    tb.booleanOperation(a, c, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    try:
        return (a.volume or 0.0) * 1000
    except RuntimeError:
        return 0.0


def _section(m, g, F, bore):
    """Blank section in the (r, z) half plane: root cone, back face, web, bore, front face. Returns points."""
    Ao = m * g["z"] / (2 * math.sin(g["gamma"]))
    ga = g["gamma"]
    df = math.atan(1.25 * m / Ao)
    uf = (math.sin(ga - df), -math.cos(ga - df))
    k = (Ao - F) / Ao
    B = (Ao / math.cos(df) * uf[0], Ao / math.cos(df) * uf[1])
    Fr = (k * B[0], k * B[1])
    e1 = (math.cos(ga), math.sin(ga))                    # back / front planes' in-section direction
    web = 2.5 * m
    wf = min(web, (Fr[0] - bore / 2 - 0.5) / e1[0])
    B2 = (B[0] - web * e1[0], B[1] - web * e1[1])
    F2 = (Fr[0] - wf * e1[0], Fr[1] - wf * e1[1])
    return [Fr, B, B2, (bore / 2, B2[1]), (bore / 2, F2[1]), F2], Ao


def _plane_at(comp, P, gamma):
    """Construction plane through P (on the x-z half plane) perpendicular to the pitch element (sin g, 0, -cos g)."""
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(P[2])))
    off = comp.constructionPlanes.add(pin)
    sk = comp.sketches.add(off)
    sk.isVisible = False
    to = lambda x, y, z: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z)))
    line = sk.sketchCurves.sketchLines.addByTwoPoints(to(P[0], -5, P[2]), to(P[0], 5, P[2]))
    u = (math.sin(gamma), 0.0, -math.cos(gamma))
    for ang in (gamma, -gamma, math.pi - gamma, gamma - math.pi):
        pi = comp.constructionPlanes.createInput()
        pi.setByAngle(line, adsk.core.ValueInput.createByReal(ang), off)
        pl = comp.constructionPlanes.add(pi)
        n = pl.geometry.normal
        if abs(abs(n.x * u[0] + n.y * u[1] + n.z * u[2]) - 1) < 1e-6:
            return pl
        pl.deleteMe()
    raise RuntimeError("Bevel: back-cone plane not found")


def _tooth_sketch(comp, plane, pts3):
    """Closed tooth outline (right flank, tip, left flank, sunk base) from 3D points on the plane."""
    sk = comp.sketches.add(plane)
    to = lambda p: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(p[0]), mm(p[1]), mm(p[2])))
    right, left, base_r, base_l = pts3
    lines, splines = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchFittedSplines
    l1 = lines.addByTwoPoints(to(base_r), to(right[0]))
    col = adsk.core.ObjectCollection.create()
    col.add(l1.endSketchPoint)
    for p in right[1:]:
        col.add(to(p))
    s1 = splines.add(col)
    tip = lines.addByTwoPoints(s1.endSketchPoint, to(left[-1]))
    col = adsk.core.ObjectCollection.create()
    col.add(tip.endSketchPoint)
    for p in reversed(left[:-1]):
        col.add(to(p))
    s2 = splines.add(col)
    l2 = lines.addByTwoPoints(s2.endSketchPoint, to(base_l))
    lines.addByTwoPoints(l2.endSketchPoint, l1.startSketchPoint)
    return sk.profiles.item(0)


def build_bevel_body(comp, m, z, gamma, F, bore):
    feats = comp.features
    g = {"z": z, "gamma": gamma}
    sec, Ao = _section(m, g, F, bore)
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = "Blank section"
    to = lambda r, zz: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(r), 0, mm(zz)))
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to(*sec[0]), to(*sec[1]))
    prev = first.endSketchPoint
    for q in sec[2:]:
        prev = lines.addByTwoPoints(prev, to(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    ri = feats.revolveFeatures.createInput(sk.profiles.item(0), comp.zConstructionAxis,
                                           adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    body = feats.revolveFeatures.add(ri).bodies.item(0)

    zv = z / math.cos(gamma)
    right, left, rf, ra = gears.involute_tooth(m, zv, n=10)
    rv = m * zv / 2
    r_in = rf - 0.3 * m
    a_r, a_l = math.atan2(right[0][1], right[0][0]), math.atan2(left[0][1], left[0][0])
    base_r2, base_l2 = (r_in * math.cos(a_r), r_in * math.sin(a_r)), (r_in * math.cos(a_l), r_in * math.sin(a_l))
    Cv = (0.0, 0.0, -Ao / math.cos(gamma))
    e1 = (math.cos(gamma), 0.0, math.sin(gamma))

    def to3(p, k=1.0):
        X, Y = p
        return (k * (Cv[0] + X * e1[0]), k * Y, k * (Cv[2] + X * e1[2]))

    k = (Ao - F) / Ao
    profiles = []
    for kk in (1.0, k):
        Pc = to3((rv, 0.0), kk)
        plane = _plane_at(comp, Pc, gamma)
        profiles.append(_tooth_sketch(comp, plane, ([to3(p, kk) for p in right], [to3(p, kk) for p in left],
                                                    to3(base_r2, kk), to3(base_l2, kk))))
    li = feats.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    li.loftSections.add(profiles[0])
    li.loftSections.add(profiles[1])
    tooth = feats.loftFeatures.add(li).bodies.item(0)
    ents = adsk.core.ObjectCollection.create()
    ents.add(tooth)
    pin = feats.circularPatternFeatures.createInput(ents, comp.zConstructionAxis)
    pin.quantity = adsk.core.ValueInput.createByReal(z)
    pin.totalAngle = adsk.core.ValueInput.createByString("360 deg")
    pat = feats.circularPatternFeatures.add(pin)
    tools = adsk.core.ObjectCollection.create()
    tools.add(tooth)
    for b in pat.bodies:
        if b != tooth:
            tools.add(b)
    ci = feats.combineFeatures.createInput(body, tools)
    ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    feats.combineFeatures.add(ci)
    return body, sec


def _bore_edge_any(occ, body, bore):
    for e in body.edges:
        g = e.geometry
        if isinstance(g, adsk.core.Circle3D) and abs(g.radius - mm(bore / 2)) < 1e-4 \
                and abs(g.center.x) < 1e-5 and abs(g.center.y) < 1e-5:
            return e.createForAssemblyContext(occ)
    raise RuntimeError("Bevel: bore edge not found")


def build(g, root, design=None):
    m, z1, z2, F = g["m"], g["z1"], g["z2"], g["F"]
    g1, g2 = bevel.cones(z1, z2)
    r1, r2 = m * z1 / 2, m * z2 / 2
    P = adsk.core.Point3D.create

    secs = {name: _section(m, {"z": z, "gamma": gamma}, F, bore)[0]
            for name, z, gamma, bore in (("Pinion", z1, g1, g["bore1"]), ("Gear", z2, g2, g["bore2"]))}
    low_pinion = min(p[1] for p in secs["Pinion"])
    H = max(PLATE_T + GAP - low_pinion, PLATE_T + GAP + r2 + 1.5 * m) + 1.0
    far_gear = -min(p[1] for p in secs["Gear"])

    mp = adsk.core.Matrix3D.create()
    mp.translation = adsk.core.Vector3D.create(0, 0, mm(H))
    mg = adsk.core.Matrix3D.create()
    mg.setToRotation(math.pi / z2, adsk.core.Vector3D.create(0, 0, 1), P(0, 0, 0))   # a space faces the pinion
    mapping = adsk.core.Matrix3D.create()
    for col, v in enumerate(((0, 0, -1), (0, -1, 0), (-1, 0, 0))):      # local x, y, z -> global
        for row in range(3):
            mapping.setCell(row, col, v[row])
    mg.transformBy(mapping)                                             # turn about the own axis, then lay down
    mg.translation = adsk.core.Vector3D.create(0, 0, mm(H))

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    sk = gc.sketches.add(gc.xYConstructionPlane)
    w = r2 + 2 * m + 5
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-r1 - 2 * m - 5), mm(-w), 0), P(mm(far_gear + 5), mm(w), 0))
    gc.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(PLATE_T)),
                                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = "Base"
    ground.isGrounded = True

    occs = {}
    for name, z, gamma, bore, mt in (("Pinion", z1, g1, g["bore1"], mp), ("Gear", z2, g2, g["bore2"], mg)):
        occ = root.occurrences.addNewComponent(mt)
        occ.component.name = name
        body, sec = build_bevel_body(occ.component, m, z, gamma, F, bore)
        body.name = f"{name} z{z}"
        occs[name] = (occ, body, sec)

    joints = {}
    for name, bore in (("Pinion", g["bore1"]), ("Gear", g["bore2"])):
        occ, body, _ = occs[name]
        geo = adsk.fusion.JointGeometry.createByCurve(_bore_edge_any(occ, body, bore), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(occ, ground, geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        j.name = f"Ground-{name}"
        joints[name] = j

    mo = adsk.fusion.RevoluteJointMotion.cast(joints["Pinion"].jointMotion)
    pb, gb = occs["Pinion"][0].bRepBodies.item(0), occs["Gear"][0].bRepBodies.item(0)
    base = _overlap(pb, gb)
    for reverse in (False, True):
        li = root.motionLinks.createInput(joints["Pinion"], joints["Gear"])
        li.valueOne = adsk.core.ValueInput.createByString("360 deg")
        li.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 * z1 / z2} deg")
        li.isReversed = reverse
        link = root.motionLinks.add(li)
        mo.rotationValue = math.pi / z1 / 2                 # a quarter tooth: the wrong sense collides
        v = _overlap(occs["Pinion"][0].bRepBodies.item(0), occs["Gear"][0].bRepBodies.item(0))
        mo.rotationValue = 0.0
        if v < base + 0.5:
            return occs
        link.deleteMe()
    raise RuntimeError(f"Bevel: the gear does not follow the pinion either way (rest overlap {base:.2f} mm3)")
