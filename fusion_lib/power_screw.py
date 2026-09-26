"""
G5: Builds the power screw in Fusion 360 (runs inside Fusion only).

A horizontal steel screw along x (axis at height H above the base) with a modelled ISO trapezoidal thread
(Fusion's thread library), plain journals at both ends in two end supports, and a nut block with the
matching internal thread sliding on the base. Joints: Ground-Screw (revolute, the add-in's input),
Ground-Nut (slider along x, direction checked by a trial slide) and a motion link: one turn = one lead.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

THREAD_TYPE = "ISO Metric Trapezoidal Threads"
PLATE_T = 4.0
GAP = 0.5
END = 12.0            # journal length at each end


def _cyl_x(comp, x0, length, r, z, name, op=adsk.fusion.FeatureOperations.NewBodyFeatureOperation, body=None):
    """Cylinder along +x from x0 (axis at y = 0, height z)."""
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(mm(x0)))
    pl = comp.constructionPlanes.add(pin)
    pl.isLightBulbOn = False
    sk = comp.sketches.add(pl)
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x0), 0, mm(z))), mm(r))
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(length)))
    if body is not None:
        inp.participantBodies = [body]
    feat = ext.add(inp)
    if op == adsk.fusion.FeatureOperations.NewBodyFeatureOperation:
        feat.bodies.item(0).name = name
        return feat.bodies.item(0)
    return feat


def _box(comp, x0, x1, y0, y1, z0, z1, name):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    pl = comp.constructionPlanes.add(pin)
    pl.isLightBulbOn = False
    sk = comp.sketches.add(pl)
    to = lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z0)))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(to(x0, y0), to(x1, y1))
    b = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(z1 - z0)),
                                                adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    b.name = name
    return b


def _thread(comp, body, radius, internal, designation):
    """Modelled thread on the cylindrical face of body with the given radius."""
    face = next(f for f in body.faces if isinstance(f.geometry, adsk.core.Cylinder)
                and abs(f.geometry.radius - mm(radius)) < 1e-4)
    tf = comp.features.threadFeatures
    classes = list(tf.threadDataQuery.allClasses(internal, THREAD_TYPE, designation))
    info = tf.createThreadInfo(internal, THREAD_TYPE, designation, classes[0])
    inp = tf.createInput(face, info)
    inp.isModeled = True
    inp.isFullLength = True
    return tf.add(inp)


def build(mech, root, design=None):
    d, P, lead = mech["d"], mech["P"], mech["l"]
    L_nut, travel = mech["nut_len"], mech["travel"]
    desig = mech["designation"]
    dj = d - P - 2.0                                          # journal: below the thread root
    H = d                                                     # axis height above the base top
    L_thr = travel + L_nut + 10.0
    wn = max(1.8 * d, d + 16)

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    x_end = 2 * END + L_thr
    _box(gc, -8, x_end + 8, -wn / 2 - 6, wn / 2 + 6, -PLATE_T, 0.0, "Base")
    for i, (xa, xb) in enumerate(((-6.0, END - 2), (END + L_thr + 2, x_end + 6))):
        blk = _box(gc, xa, xb, -wn / 2, wn / 2, 0.0, H + d / 2 + 4, f"Support {i + 1}")
        _cyl_x(gc, xa, xb - xa, dj / 2 + 0.2, H, "", adsk.fusion.FeatureOperations.CutFeatureOperation, blk)
    ground.isGrounded = True

    screw = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    sc = screw.component
    sc.name = "Screw"
    body = _cyl_x(sc, 0.0, END, dj / 2, H, "Journal 1")
    thr = _cyl_x(sc, END, L_thr, d / 2, H, f"Screw {desig}")
    _cyl_x(sc, END + L_thr, END, dj / 2, H, "Journal 2")
    _thread(sc, thr, d / 2, False, desig)

    nut = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    nc = nut.component
    nc.name = "Nut"
    # Fusion starts each modelled helix at its start face in the same way: the nut meshes when its start face is
    # a whole number of pitches from the screw thread's start (checked: zero overlap; off by P/4 it overlaps)
    x0 = END + P * round((L_thr - L_nut) / 2 / P)             # the middle of the thread, pitch-aligned
    nb = _box(nc, x0, x0 + L_nut, -wn / 2, wn / 2, GAP, H + wn / 2, f"Nut {desig}")
    _cyl_x(nc, x0, L_nut, (d - P) / 2, H, "", adsk.fusion.FeatureOperations.CutFeatureOperation, nb)
    _thread(nc, nb, (d - P) / 2, True, desig)

    # joints
    edge = next(e for e in body.edges if isinstance(e.geometry, adsk.core.Circle3D) and abs(e.geometry.center.x) < 1e-6)
    geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(screw), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(screw, ground, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    js = root.asBuiltJoints.add(ji)
    js.name = "Ground-Screw"
    nedge = next(e for e in nb.edges if isinstance(e.geometry, (adsk.core.Circle3D, adsk.core.Arc3D))
                 and abs(e.geometry.center.x - mm(x0)) < 1e-6)
    slider = None
    for direction in (adsk.fusion.JointDirections.ZAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection,
                      adsk.fusion.JointDirections.YAxisJointDirection):
        g2 = adsk.fusion.JointGeometry.createByCurve(nedge.createForAssemblyContext(nut), adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(nut, ground, g2)
        ji.setAsSliderJointMotion(direction)
        j = root.asBuiltJoints.add(ji)
        mo = adsk.fusion.SliderJointMotion.cast(j.jointMotion)
        p0 = nut.transform2.translation
        mo.slideValue = 0.05
        p1 = nut.transform2.translation
        mo.slideValue = 0.0
        if abs(abs(p1.x - p0.x) - 0.05) < 1e-4:
            j.name = "Ground-Nut"
            slider = (j, 1 if p1.x > p0.x else -1)
            break
        j.deleteMe()
    if slider is None:
        raise RuntimeError("Nut: no slider along the screw axis")
    lim = adsk.fusion.SliderJointMotion.cast(slider[0].jointMotion).slideLimits   # the nut stays on the thread
    lo, hi = -mm(travel / 2), mm(travel / 2)                 # from the middle, either way
    lim.isMinimumValueEnabled, lim.minimumValue = True, lo
    lim.isMaximumValueEnabled, lim.maximumValue = True, hi
    # which way the nut runs for a turn depends on the joint's axis sense: a quarter turn must keep the threads
    # apart (the wrong way puts them half a pitch into each other)
    tb = adsk.fusion.TemporaryBRepManager.get()
    rot = adsk.fusion.RevoluteJointMotion.cast(js.jointMotion)
    for sense in (1, -1):
        link = root.motionLinks.createInput(js, slider[0])
        link.valueOne = adsk.core.ValueInput.createByString("360 deg")
        link.valueTwo = adsk.core.ValueInput.createByString(f"{sense * lead * slider[1]} mm")
        ml = root.motionLinks.add(link)
        rot.rotationValue = math.radians(90)
        a = tb.copy(next(b for b in screw.bRepBodies if b.name.startswith("Screw")))
        c = tb.copy(nut.bRepBodies.item(0))
        tb.booleanOperation(a, c, adsk.fusion.BooleanTypes.IntersectionBooleanType)
        rot.rotationValue = 0.0
        if not a.volume or a.volume * 1000 < 1.0:
            return screw
        ml.deleteMe()
    raise RuntimeError("Power screw: the nut does not follow the thread either way")
