"""
E1: Builds the single-stage reducer in Fusion 360 (runs inside Fusion only).

Frame: shaft axes along x, gear mid-plane at x = 0, input shaft on y = 0, output shaft on y = a.
Housing (grounded): two walls carrying the bearings + base plate (open top so the gears stay visible).
Gears: gear.build_gear_body (drawn in the XY plane, extruded along +z) placed with
    T(-b/2, y, 0) * Ry(90 deg) * Rz(phi):  local z -> world x, the gear-module layout (line of centres
    along +x) rotated so that the line of centres points along +y. The hub gets a keyway where the key is.
Each gear is rigidly joined to its shaft; the shafts turn in the housing (revolute joints) and a
reversed motion link at z1/z2 couples them (checked by gear-overlap volume in the tests).
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm
from fusion_lib.gear import build_gear_body
from fusion_lib.shaft import _bearings, make_shaft, revolute_to


def _matrix(rz, ry, t):
    """T(t) * Ry(ry) * Rz(rz)."""
    O, V = adsk.core.Point3D.create(0, 0, 0), adsk.core.Vector3D.create
    m = adsk.core.Matrix3D.create()
    m.setToRotation(rz, V(0, 0, 1), O)
    r2 = adsk.core.Matrix3D.create()
    r2.setToRotation(ry, V(0, 1, 0), O)
    m.transformBy(r2)                              # first Rz, then Ry
    m.translation = V(mm(t[0]), mm(t[1]), mm(t[2]))
    return m


def _housing(comp, h):
    """Two walls (bearing seats as holes) + base plate."""
    P = adsk.core.Point3D.create
    (y0, y1), (z0, z1), t, bt = h["y"], h["z"], h["t"], h["base_t"]
    ext = comp.features.extrudeFeatures
    for i, xw in enumerate(h["x_walls"], 1):
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(mm(xw)))
        sk = comp.sketches.add(comp.constructionPlanes.add(pin))
        to = lambda y, z: sk.modelToSketchSpace(P(mm(xw), mm(y), mm(z)))
        sk.sketchCurves.sketchLines.addTwoPointRectangle(to(y0, z0 + bt), to(y1, z1))
        for yc, od in h["seats"]:
            sk.sketchCurves.sketchCircles.addByCenterRadius(to(yc, 0), mm(od / 2))
        wall = max(sk.profiles, key=lambda p: p.areaProperties().area)     # the wall, not the seat discs
        inp = ext.createInput(wall, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(t)), True)
        ext.add(inp).bodies.item(0).name = f"Wall {i}"
    xl = h["x_walls"][1] + t / 2
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    to = lambda x, y: sk.modelToSketchSpace(P(mm(x), mm(y), mm(z0)))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(to(-xl, y0), to(xl, y1))
    ext.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(bt)),
                  adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0).name = "Base"


def _keyway(comp, body, bore, key, phi, width):
    """Hub keyway through the gear, on the side where the shaft key is (world +z).
    World +z in the gear's local frame: Rz(-phi) Ry(-90) (0, 0, 1) = (-cos phi, sin phi, 0)."""
    P = adsk.core.Point3D.create
    ux, uy = -math.cos(phi), math.sin(phi)         # radial direction of the key
    vx, vy = -uy, ux                               # across the key
    r0, r1, hb = bore / 2 - 1.0, bore / 2 + (key["h"] - key["t1"]) + 0.2, key["b"] / 2
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "Keyway"
    pts = [P(mm(r * ux + s * hb * vx), mm(r * uy + s * hb * vy), 0) for r, s in ((r0, -1), (r1, -1), (r1, 1), (r0, 1))]
    lines = sk.sketchCurves.sketchLines
    for a, b in zip(pts, pts[1:] + pts[:1]):
        lines.addByTwoPoints(a, b)
    prof = min(sk.profiles, key=lambda p: p.areaProperties().area)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(width)))
    inp.participantBodies = [body]
    ext.add(inp)


def build(r, root, design=None):
    """r: pipeline's fusion dict (reducer.design output). Returns {"in": occ, "out": occ}."""
    g, s1, s2, h = r["gear"], r["shaft_in"], r["shaft_out"], r["housing"]
    a, b, m = g["center_distance"], g["face_width"], g["module"]
    lay1, lay2 = s1["layout"], s2["layout"]
    xm1 = sum(x for x, _, _ in lay1["bearings"]) / 2
    xm2 = sum(x for x, _, _ in lay2["bearings"]) / 2

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    ground.component.name = "Housing"
    _housing(ground.component, h)
    _bearings(ground.component, lay1, dx=-xm1, dy=0.0, suffix=" in")
    _bearings(ground.component, lay2, dx=-xm2, dy=a, suffix=" out")
    ground.isGrounded = True

    # input shaft: local x -> world x - xm1; output shaft turned 180 deg about z (its drive end points to +x)
    occ1, body1 = make_shaft(root, lay1, "Input shaft", _matrix(0.0, 0.0, (-xm1, 0.0, 0.0)))
    occ2, body2 = make_shaft(root, lay2, "Output shaft", _matrix(math.pi, 0.0, (xm2, a, 0.0)))

    even = g["z2"] % 2 == 0
    joints = {}
    for name, z, bore, y, phase, s, occ_s in (("Pinion", g["z1"], s1["d"], 0.0, 0.0, s1, occ1),
                                              ("Gear", g["z2"], s2["d"], a, math.pi / g["z2"] if even else 0.0, s2, occ2)):
        phi = math.pi / 2 + phase
        occ = root.occurrences.addNewComponent(_matrix(phi, math.pi / 2, (-b / 2, y, 0.0)))
        occ.component.name = name
        body = build_gear_body(occ.component, m, z, b, bore)
        body.name = f"{name} z{z}"
        if s["key"]:
            _keyway(occ.component, body, bore, s["key"], phi, b)
        ji = root.asBuiltJoints.createInput(occ, occ_s, None)
        ji.setAsRigidJointMotion()
        root.asBuiltJoints.add(ji).name = f"{name} on shaft"
    joints["in"] = revolute_to(root, ground, occ1, body1, lay1, "Ground-Input")
    joints["out"] = revolute_to(root, ground, occ2, body2, lay2, "Ground-Output")

    link = root.motionLinks.createInput(joints["in"], joints["out"])
    link.valueOne = adsk.core.ValueInput.createByString("360 deg")
    link.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 * g['z1'] / g['z2']} deg")
    link.isReversed = REVERSED
    root.motionLinks.add(link)
    return {"in": occ1, "out": occ2}


# The output joint axis points the other way (shaft turned 180 deg about z), so equal-sign joint angles
# already mean opposite turning in the world: the link is NOT reversed. Measured in Fusion: with
# isReversed = True the teeth overlapped 445-658 mm^3; with False the overlap is 0 at 0-180 deg.
REVERSED = False
