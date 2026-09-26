"""
G1b: Builds a worm gear set in Fusion 360 (runs inside Fusion only).

Wheel: a helical gear (fusion_lib.helical) with helix angle = the worm's lead angle, same (right) hand,
transverse module = the worm's axial module, on the vertical axis at the origin.
Worm: an Archimedean (ZA) thread: its transverse section (worm.worm_section) swept along the worm axis with a
twist of 2 pi per lead (one sweep per start, patterned), joined to the core, with steel journals; the worm lies
along y at x = C, level with the wheel's mid-plane, a thread centred where it meets the wheel.
Joints Ground-Driver (worm, the add-in's input) and Ground-Wheel; the motion link turns the wheel N_W / N_G of a
turn per worm turn, its sense found by a trial turn (the wrong one cuts into the wheel).
"""

import math

import adsk.core
import adsk.fusion

import worm
from fusion_lib.fourbar import mm
from fusion_lib.gear import GAP, PLATE_T, _bore_edge
from fusion_lib.helical import build_helical_body

JOURNAL = 15.0          # journal length beyond the thread, each end
CLEAR = 0.02            # thread flank clearance, in modules (a little backlash) ...
CLEAR_LEAD = 0.08       # ... + this x sin(lead angle): an Archimedean thread departs more from the wheel's
                        # involute helicoid as the lead grows (3-start, 20.6 deg: 0.035 mm3 slivers without it)


def _overlap(b1, b2):
    tb = adsk.fusion.TemporaryBRepManager.get()
    a, c = tb.copy(b1), tb.copy(b2)
    tb.booleanOperation(a, c, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    try:
        return (a.volume or 0.0) * 1000
    except RuntimeError:
        return 0.0


def _worm_body(comp, g, Lw):
    m, NW, dW = g["m"], g["NW"], g["dW"]
    L = NW * math.pi * m
    feats = comp.features
    P = adsk.core.Point3D.create
    rot0 = -math.pi * Lw / L                              # the start at z = Lw/2 is centred on +x
    clear = (CLEAR + CLEAR_LEAD * math.sin(math.radians(g["lead_deg"]))) * m
    right, left, r_in, ra = worm.worm_section(m, NW, dW, rotate=rot0, clear=clear)
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "Thread section (ZA)"
    sp = sk.sketchCurves.sketchFittedSplines
    arcs = sk.sketchCurves.sketchArcs

    def spline(pts):
        col = adsk.core.ObjectCollection.create()
        for x, y in pts:
            col.add(P(mm(x), mm(y), 0))
        return sp.add(col)

    s1 = spline(right)
    # the section is symmetric about rot0: the arcs' middle points lie there (averaging atan2 angles fails
    # when the two ends straddle +-180 deg: the arc then goes the long way round)
    tip = arcs.addByThreePoints(s1.endSketchPoint, P(mm(ra * math.cos(rot0)), mm(ra * math.sin(rot0)), 0),
                                P(mm(left[-1][0]), mm(left[-1][1]), 0))
    far = tip.endSketchPoint if tip.endSketchPoint.geometry.distanceTo(s1.endSketchPoint.geometry) > 1e-6 \
        else tip.startSketchPoint
    col = adsk.core.ObjectCollection.create()
    col.add(far)
    for x, y in reversed(left[:-1]):
        col.add(P(mm(x), mm(y), 0))
    s2 = sp.add(col)
    arcs.addByThreePoints(s2.endSketchPoint, P(mm(r_in * math.cos(rot0)), mm(r_in * math.sin(rot0)), 0), s1.startSketchPoint)
    prof = max(sk.profiles, key=lambda p: p.areaProperties().area)

    axis = comp.sketches.add(comp.xZConstructionPlane)
    axis.name = "Worm axis"
    to = lambda z: axis.modelToSketchSpace(P(0, 0, mm(z)))
    path = feats.createPath(axis.sketchCurves.sketchLines.addByTwoPoints(to(0), to(Lw)), False)
    si = feats.sweepFeatures.createInput(prof, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    si.twistAngle = adsk.core.ValueInput.createByReal(2 * math.pi * Lw / L)
    thread = feats.sweepFeatures.add(si).bodies.item(0)
    tools = adsk.core.ObjectCollection.create()
    tools.add(thread)
    if NW > 1:
        ents = adsk.core.ObjectCollection.create()
        ents.add(thread)
        pin = feats.circularPatternFeatures.createInput(ents, comp.zConstructionAxis)
        pin.quantity = adsk.core.ValueInput.createByReal(NW)
        pin.totalAngle = adsk.core.ValueInput.createByString("360 deg")
        for b in feats.circularPatternFeatures.add(pin).bodies:
            if b != thread:
                tools.add(b)

    core = comp.sketches.add(comp.xYConstructionPlane)
    core.name = "Core"
    core.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(dW / 2 - 1.25 * m))
    body = feats.extrudeFeatures.addSimple(core.profiles.item(0), adsk.core.ValueInput.createByReal(mm(Lw)),
                                           adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    ci = feats.combineFeatures.createInput(body, tools)
    ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    feats.combineFeatures.add(ci)
    body.name = f"Worm {NW}-start (right hand)"

    jr = g["bore_worm"] / 2
    shaft = None
    for z0, h in ((-JOURNAL, JOURNAL), (Lw, JOURNAL)):
        pl = comp.constructionPlanes.createInput()
        pl.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
        s = comp.sketches.add(comp.constructionPlanes.add(pl))
        s.name = "Journal"
        s.sketchCurves.sketchCircles.addByCenterRadius(s.modelToSketchSpace(P(0, 0, mm(z0))), mm(jr))
        b = feats.extrudeFeatures.addSimple(s.profiles.item(0), adsk.core.ValueInput.createByReal(mm(h)),
                                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
        b.name = "Shaft"
        shaft = shaft or b
    return body, shaft


def build(g, root, design=None):
    m, NW, NG, dW = g["m"], g["NW"], g["NG"], g["dW"]
    lam = math.radians(g["lead_deg"])
    dG = m * NG
    C = (dW + dG) / 2
    raG, raW = dG / 2 + m, dW / 2 + m
    F = g["F_G"]
    Lw = worm.worm_length(m, NW, NG, dW)
    z0 = PLATE_T + GAP + max(0.0, raW - F / 2) + 1.0
    zmid = z0 + F / 2
    P = adsk.core.Point3D.create

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    sk = gc.sketches.add(gc.xYConstructionPlane)
    half_y = max(raG, Lw / 2 + JOURNAL) + 5
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-raG - 5), mm(-half_y), 0), P(mm(C + raW + 5), mm(half_y), 0))
    gc.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(PLATE_T)),
                                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = "Base"
    ground.isGrounded = True

    wg = {"module": m * math.cos(lam), "psi": g["lead_deg"], "phi_t": math.degrees(worm.PHI_X), "face_width": F}
    rG = dG / 2
    phase = -g.get("_hand", 1) * (F / 2) * math.tan(lam) / rG + math.pi / NG   # a tooth space faces the worm at mid-face
    mt = adsk.core.Matrix3D.create()
    mt.setToRotation(phase, adsk.core.Vector3D.create(0, 0, 1), P(0, 0, 0))
    mt.translation = adsk.core.Vector3D.create(0, 0, mm(z0))
    wheel = root.occurrences.addNewComponent(mt)
    wheel.component.name = "Wheel"
    wb, _ = build_helical_body(wheel.component, wg, NG, g["bore_wheel"], g.get("_hand", 1))
    wb.name = f"Wheel z{NG} (right hand)"

    # worm: local z -> global y, local x -> global -x (towards the wheel), local y -> global z
    mw = adsk.core.Matrix3D.create()
    mw.setCell(0, 0, -1); mw.setCell(1, 0, 0); mw.setCell(2, 0, 0)
    mw.setCell(0, 1, 0); mw.setCell(1, 1, 0); mw.setCell(2, 1, 1)
    mw.setCell(0, 2, 0); mw.setCell(1, 2, 1); mw.setCell(2, 2, 0)
    mw.translation = adsk.core.Vector3D.create(mm(C), mm(-Lw / 2), mm(zmid))
    wocc = root.occurrences.addNewComponent(mw)
    wocc.component.name = "Worm"
    wbody, shaft = _worm_body(wocc.component, g, Lw)

    edge = next(e for e in shaft.edges if isinstance(e.geometry, adsk.core.Circle3D))
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(wocc), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(wocc, ground, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    jw = root.asBuiltJoints.add(ji)
    jw.name = "Ground-Driver"
    geo = adsk.fusion.JointGeometry.createByCurve(_bore_edge(wheel, g["bore_wheel"], F), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(wheel, ground, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    jg = root.asBuiltJoints.add(ji)
    jg.name = "Ground-Wheel"

    mo = adsk.fusion.RevoluteJointMotion.cast(jw.jointMotion)
    wb_, gb_ = wocc.bRepBodies.item(0), wheel.bRepBodies.item(0)
    base = _overlap(wb_, gb_)
    if g.get("_nolink"):
        return wocc
    for reverse in (False, True):
        li = root.motionLinks.createInput(jw, jg)
        li.valueOne = adsk.core.ValueInput.createByString("360 deg")
        li.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 * NW / NG} deg")
        li.isReversed = reverse
        link = root.motionLinks.add(li)
        mo.rotationValue = math.pi / 2                       # a quarter worm turn: the wrong sense collides
        v = _overlap(wocc.bRepBodies.item(0), wheel.bRepBodies.item(0))
        mo.rotationValue = 0.0
        if v < base + 1.0:
            return wocc
        link.deleteMe()
    raise RuntimeError(f"Worm: the wheel does not follow the worm either way (rest overlap {base:.2f} mm3)")
