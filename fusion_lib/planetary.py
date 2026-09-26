"""
G2: Builds a planetary gear set in Fusion 360 (runs inside Fusion only).

Sun at the origin (joint Ground-Driver, the add-in's input), n planets on pins of a carrier plate above the
gears (joints Carrier-Planet k), the carrier turning about the origin (Ground-Carrier, the output) and a
fixed ring gear. The ring's tooth spaces are an external tooth's shape with a 1.25 m addendum (so its root
clears the planets' tips), cut once and patterned. Each gear is turned to its mesh phase
(planetary.phases). Motion links: sun -> carrier at 1/i, carrier -> every planet at Z_r/Z_p; the sense of
each link is found by a trial turn (the carrier must follow the sun; a planet must not cut into the ring).
"""

import math

import adsk.core
import adsk.fusion

import planetary
from fusion_lib.fourbar import mm
from fusion_lib.gear import _bore_edge, _tooth_profile, build_gear_body

PLATE_T = 3.0
GAP = 0.5
PIN_CLEAR = 0.2          # planet bore over the pin (diameter)


def _overlap(b1, b2):
    tb = adsk.fusion.TemporaryBRepManager.get()
    a, c = tb.copy(b1), tb.copy(b2)
    tb.booleanOperation(a, c, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    return (a.volume or 0.0) * 1000


def _angle(occ):
    m = occ.transform2
    return math.atan2(m.getCell(1, 0), m.getCell(0, 0))


def _disk(comp, r, z, h, op, bodies=None, x=0.0, y=0.0, name="Disk"):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    sk.name = name
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z))), mm(r))
    ext = comp.features.extrudeFeatures
    fi = ext.createInput(sk.profiles.item(0), op)
    fi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(h)))
    if bodies:
        fi.participantBodies = bodies
    return ext.add(fi)


def _ring_body(comp, m, Zr, b, R_out):
    ext = comp.features.extrudeFeatures
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "Ring blank"
    O = adsk.core.Point3D.create(0, 0, 0)
    sk.sketchCurves.sketchCircles.addByCenterRadius(O, mm(R_out))
    sk.sketchCurves.sketchCircles.addByCenterRadius(O, mm(m * Zr / 2 - m))
    ring = next(p for p in sk.profiles if p.profileLoops.count == 2)
    body = ext.addSimple(ring, adsk.core.ValueInput.createByReal(mm(b)),
                         adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    sp = comp.sketches.add(comp.xYConstructionPlane)
    sp.name = "Tooth space (internal)"
    ci = ext.createInput(_tooth_profile(sp, m, Zr, add=1.25), adsk.fusion.FeatureOperations.CutFeatureOperation)
    ci.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(b)))
    ci.participantBodies = [body]
    cut = ext.add(ci)
    ents = adsk.core.ObjectCollection.create()
    ents.add(cut)
    pi = comp.features.circularPatternFeatures.createInput(ents, comp.zConstructionAxis)
    pi.quantity = adsk.core.ValueInput.createByReal(Zr)
    pi.totalAngle = adsk.core.ValueInput.createByString("360 deg")
    comp.features.circularPatternFeatures.add(pi)
    body.name = f"Ring z{Zr}"
    return body


def _place(root, name, x, y, z, rot):
    mt = adsk.core.Matrix3D.create()
    mt.setToRotation(rot, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    mt.translation = adsk.core.Vector3D.create(mm(x), mm(y), mm(z))
    occ = root.occurrences.addNewComponent(mt)
    occ.component.name = name
    return occ


def _revolute(root, a, b, edge, name):
    geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(a, b, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def build(g, root, design=None):
    m, Zs, Zp, Zr, n, b = g["m"], g["Zs"], g["Zp"], g["Zr"], g["n"], g["b"]
    a = m * (Zs + Zp) / 2
    pin_d, bs, bc = g["pin"], g["bore_sun"], g["bore_carrier"]
    z0 = PLATE_T + GAP
    New, Join = adsk.fusion.FeatureOperations.NewBodyFeatureOperation, adsk.fusion.FeatureOperations.JoinFeatureOperation
    out, psi = planetary.phases(Zs, Zp, Zr, n)

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    base = _disk(gc, g["R_out"] + 8, 0, PLATE_T, New, name="Base").bodies.item(0)
    base.name = "Base"
    _disk(gc, bs / 2 + 0.5, 0, PLATE_T, adsk.fusion.FeatureOperations.CutFeatureOperation, [base], name="Sun shaft hole")
    ground.isGrounded = True

    ring = _place(root, "Ring", 0, 0, z0, psi)
    _ring_body(ring.component, m, Zr, b, g["R_out"])
    ring.isGrounded = True

    sun = _place(root, "Sun", 0, 0, z0, 0.0)
    build_gear_body(sun.component, m, Zs, b, bs).name = f"Sun z{Zs}"
    _disk(sun.component, bs / 2, -z0 - 2, b + z0 + 2, New, name="Shaft").bodies.item(0).name = "Shaft"

    planets = []
    for k, (th, phi) in enumerate(out):
        occ = _place(root, f"Planet {k + 1}", a * math.cos(th), a * math.sin(th), z0, phi)
        build_gear_body(occ.component, m, Zp, b, pin_d + PIN_CLEAR).name = f"Planet z{Zp}"
        planets.append(occ)

    carrier = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    cc = carrier.component
    cc.name = "Carrier"
    t = max(pin_d, 5.0)
    zc = z0 + b + GAP
    plate = _disk(cc, a + pin_d / 2 + 6, zc, t, New, name="Carrier plate").bodies.item(0)
    plate.name = "Carrier"
    plate.opacity = 0.45                                  # the planets stay visible under the plate
    _disk(cc, bc / 2, zc + t, 20.0, Join, [plate], name="Output shaft")
    for th, _ in out:
        _disk(cc, pin_d / 2, z0 - 0.0, zc - z0, Join, [plate], a * math.cos(th), a * math.sin(th), "Planet pin")

    js = _revolute(root, ground, sun, _bore_edge(sun, bs, b), "Ground-Driver")
    top = zc + t + 20.0
    edge = next(e for e in plate.edges if isinstance(e.geometry, adsk.core.Circle3D)
                and abs(e.geometry.radius - mm(bc / 2)) < 1e-4 and abs(e.geometry.center.z - mm(top)) < 1e-4)
    jc = _revolute(root, ground, carrier, edge, "Ground-Carrier")
    jp = [_revolute(root, occ, carrier, _bore_edge(occ, pin_d + PIN_CLEAR, b), f"Carrier-Planet {k + 1}")
          for k, occ in enumerate(planets)]

    ms = adsk.fusion.RevoluteJointMotion.cast(js.jointMotion)
    i = 1 + Zr / Zs
    for reverse in (False, True):
        li = root.motionLinks.createInput(js, jc)
        li.valueOne = adsk.core.ValueInput.createByString("360 deg")
        li.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 / i} deg")
        li.isReversed = reverse
        link = root.motionLinks.add(li)
        c0 = _angle(carrier)
        ms.rotationValue = math.radians(10.0)
        turned = _angle(carrier) - c0
        ms.rotationValue = 0.0
        if turned > 0 and abs(turned - math.radians(10.0) / i) < 1e-6:
            break
        link.deleteMe()
    else:
        raise RuntimeError("Planetary: the carrier does not follow the sun")

    trial = math.radians(180.0 / Zs)                       # half a sun tooth: a wrong planet sense collides
    ring_body, sun_body = ring.bRepBodies.item(0), sun.bRepBodies.item(0)
    for k, (occ, j) in enumerate(zip(planets, jp)):
        for reverse in (False, True):
            li = root.motionLinks.createInput(jc, j)
            li.valueOne = adsk.core.ValueInput.createByString("360 deg")
            li.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 * Zr / Zp} deg")
            li.isReversed = reverse
            link = root.motionLinks.add(li)
            ms.rotationValue = trial
            body = occ.bRepBodies.item(0)
            v = _overlap(body, ring_body) + _overlap(body, sun_body)
            ms.rotationValue = 0.0
            if v < 0.5:
                break
            link.deleteMe()
        else:
            raise RuntimeError(f"Planetary: planet {k + 1} does not roll on the ring either way")
    return sun
