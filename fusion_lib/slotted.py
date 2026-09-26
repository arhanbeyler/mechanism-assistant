"""
F4: Builds the Scotch yoke and the crank + slotted lever quick return in Fusion 360 (runs inside Fusion only).

Both use a slotted block that rides between two rails along x (the yoke / the ram); a pin drives it through
a vertical slot (pin-slot joint). The quick return adds the slotted lever between the crank and the ram.
Layers: base plate below z = 0 | crank (+ its pin, up) | lever (+ its tip pin, up) | block and rails.
No shaft passes a moving layer: the crank and lever pivots only go down to the base.
Pin-slot and slider directions are checked with a trial move (Fusion's joint axes follow the picked
geometry, see fusion_lib/slider_crank.py).
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import GAP, hole_edge, make_link, mm
from fusion_lib.slider_crank import PLATE_T, _box, _hole

CLEAR = 0.3          # mm, pin to slot (as slotted.CLEAR)
RAIL_W = 4.0
RAIL_CLEAR = 0.5


def _plane_sketch(comp, z):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    pl = comp.constructionPlanes.add(pin)
    pl.isLightBulbOn = False
    sk = comp.sketches.add(pl)
    return sk, (lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z))))


def _cut_slot(comp, body, p0, p1, w, z0, h):
    """Obround slot between centres p0, p1 (width w) cut through body."""
    sk, to = _plane_sketch(comp, z0)
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    nx, ny = -uy * w / 2, ux * w / 2
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    a = lines.addByTwoPoints(to(p0[0] + nx, p0[1] + ny), to(p1[0] + nx, p1[1] + ny))
    b = lines.addByTwoPoints(to(p0[0] - nx, p0[1] - ny), to(p1[0] - nx, p1[1] - ny))
    arcs.addByThreePoints(a.endSketchPoint, to(p1[0] + ux * w / 2, p1[1] + uy * w / 2), b.endSketchPoint)
    arcs.addByThreePoints(b.startSketchPoint, to(p0[0] - ux * w / 2, p0[1] - uy * w / 2), a.startSketchPoint)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(h)))
    inp.participantBodies = [body]
    ext.add(inp)


def _pin(comp, x, y, d, z0, h, name):
    sk, to = _plane_sketch(comp, z0)
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(x, y), mm(d / 2))
    body = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(h)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def _revolute(root, o1, o2, edge, name):
    geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(o1, o2, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def _block(root, gc, name, x0, y_lo, y_hi, w, t, z, slot, s, travel, x_post_min=0.0):
    """Slotted block (component name) at x0 between rails on the ground component gc.
    slot: (y0, y1) of the pin centre in the block; travel: (x_min, x_max) of the block centre;
    x_post_min: the rail post (which passes the lower layers) stays beyond the parts moving there.
    Returns (occurrence, block body)."""
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(mm(x0), 0, mm(z))
    occ = root.occurrences.addNewComponent(m)
    comp = occ.component
    comp.name = name
    body = _box(comp, -w / 2, y_lo, w / 2, y_hi, 0.0, t, name)
    _cut_slot(comp, body, (0.0, slot[0]), (0.0, slot[1]), s, 0.0, t)
    xr0, xr1 = travel[0] - w / 2 - 3, max(travel[1] + w / 2 + 3, x_post_min)
    for sgn, y in ((1, y_hi + RAIL_CLEAR), (-1, y_lo - RAIL_CLEAR)):
        y0, y1 = sorted((y, y + sgn * RAIL_W))
        _box(gc, xr0, y0, xr1, y1, z, t, f"Rail {'+' if sgn > 0 else '-'}")
    _box(gc, xr1, y_lo - RAIL_CLEAR - RAIL_W, xr1 + 6, y_hi + RAIL_CLEAR + RAIL_W, 0.0, z + t, "Rail post")
    return occ, body


def _slider(root, ground, occ, body):
    """Ground-block slider along world x (the joint axes follow the picked edge: tried both ways)."""
    edge = next(e for e in body.edges if isinstance(e.geometry, adsk.core.Arc3D))   # a slot end
    for direction in (adsk.fusion.JointDirections.XAxisJointDirection, adsk.fusion.JointDirections.YAxisJointDirection):
        geo = adsk.fusion.JointGeometry.createByCurve(edge.createForAssemblyContext(occ),
                                                      adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(occ, ground, geo)
        ji.setAsSliderJointMotion(direction)
        j = root.asBuiltJoints.add(ji)
        mo = adsk.fusion.SliderJointMotion.cast(j.jointMotion)
        p0 = occ.transform2.translation
        mo.slideValue = 0.05
        p1 = occ.transform2.translation
        mo.slideValue = 0.0
        if abs(p1.x - p0.x) > 0.04 and abs(p1.y - p0.y) < 1e-4:
            j.name = f"Ground-{occ.component.name}"
            return j
        j.deleteMe()
    raise RuntimeError(f"{occ.component.name}: no slider along x")


def _pin_slot(root, o_pin, o_slot, edge, name, check):
    """Pin-slot joint (pin on o_pin rotating about z, sliding along the slot); check() -> True if the
    mechanism then follows the engine (tried with both slide axes)."""
    for direction in (adsk.fusion.JointDirections.YAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection):
        geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(o_pin, o_slot, geo)
        ji.setAsPinSlotJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, direction)
        j = root.asBuiltJoints.add(ji)
        j.name = name
        if check():
            return j
        j.deleteMe()
    raise RuntimeError(f"{name}: the pin-slot joint does not follow the slot")


def _crank(root, g, z, O2, th0):
    sec = g["section_mm"]
    d, ds = sec["hole"], sec.get("shaft", sec["hole"])
    cs = dict(sec["links"]["Crank"], hole=d, hole0=ds, hole1=d)
    A = (O2[0] + g["r"] * math.cos(math.radians(th0)), O2[1] + g["r"] * math.sin(math.radians(th0)))
    occ, _ = make_link(root, "Crank", O2, A, z, cs)
    return occ, cs, A


def _turns_like(crank_joint, check_pose):
    """Turn the crank a little and compare with the engine; the joint is put back."""
    mo = adsk.fusion.RevoluteJointMotion.cast(crank_joint.jointMotion)
    ok = True
    for deg in (20, -35):
        mo.rotationValue = math.radians(deg)
        moved = abs(mo.rotationValue - math.radians(deg)) < 1e-3     # a locked loop does not turn at all
        ok = ok and moved and check_pose()
    mo.rotationValue = 0.0
    return ok


def build_yoke(g, root):
    r, sec = g["r"], g["section_mm"]
    d = sec["hole"]
    ys = sec["links"]["Yoke"]
    s = d + 2 * CLEAR
    w, t = ys["width"], ys["thickness"]
    bar = (w - s) / 2
    th0 = g["theta0"]
    tc = sec["links"]["Crank"]["thickness"]
    z_c, z_y = GAP, GAP + tc + GAP

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    H = r + s / 2 + bar
    ext = r + max(w / 2, sec["links"]["Crank"].get("boss1", 0) / 2) + 14
    plate = _box(gc, -ext, -H - 8, ext + 8, H + 8, -PLATE_T, PLATE_T, "Base")
    _hole(gc, plate, 0.0, 0.0, -PLATE_T, sec.get("shaft", d), PLATE_T)
    crank, cs, A = _crank(root, g, z_c, (0.0, 0.0), th0)
    _pin(crank.component, g["r"], 0.0, d, 0.0, tc + GAP + t, f"Pin Ø{d:g}")
    boss = max(cs.get("boss1", cs["width"]), cs["width"])
    yoke, body = _block(root, gc, "Yoke", A[0], -H, H, w, t, z_y, (-r, r), s, (-r, r), r + boss / 2 + 2)
    ground.isGrounded = True
    jc = _revolute(root, ground, crank, hole_edge(crank, 0.0, cs), "Ground-Crank (O2)")
    _slider(root, ground, yoke, body)

    sign = _sense(jc, crank)

    def pose_ok():
        th = math.radians(th0) + sign * adsk.fusion.RevoluteJointMotion.cast(jc.jointMotion).rotationValue
        return abs(yoke.transform2.translation.x * 10 - r * math.cos(th)) < 0.05

    _pin_slot(root, crank, yoke, hole_edge(crank, r, cs), "Crank-Yoke (pin in slot)", lambda: _turns_like(jc, pose_ok))
    return crank


def _sense(joint, occ):
    mo = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    ang = lambda: math.atan2(occ.transform2.getCell(1, 0), occ.transform2.getCell(0, 0))
    a0 = ang()
    mo.rotationValue = math.radians(2)
    s = 1 if math.remainder(ang() - a0, 2 * math.pi) > 0 else -1
    mo.rotationValue = 0.0
    return s


def build_quick_return(g, root):
    r, dd, L, sec = g["r"], g["d"], g["L"], g["section_mm"]
    d = sec["hole"]
    s = d + 2 * CLEAR
    ls = sec["links"]["Lever"]
    wl, tl = ls["width"], ls["thickness"]
    tc = sec["links"]["Crank"]["thickness"]
    z_c = GAP
    z_l = z_c + tc + GAP
    z_r = z_l + tl + GAP
    psi_max = math.radians(g["psi_max"])
    half = L * math.sin(psi_max)                          # stroke / 2
    y_lo_pin = L * math.cos(psi_max)

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    wr = max(3 * s, 24.0)                                 # ram width (x)
    y_lo, y_hi = y_lo_pin - s / 2 - 6, L + s / 2 + 6
    ext = half + wr / 2 + 16
    plate = _box(gc, -ext, -wl, ext, y_hi + 12, -PLATE_T, PLATE_T, "Base")
    _hole(gc, plate, 0.0, 0.0, -PLATE_T, d, PLATE_T)
    _hole(gc, plate, 0.0, dd, -PLATE_T, sec.get("shaft", d), PLATE_T)

    crank, cs, A = _crank(root, g, z_c, (0.0, dd), g["theta0"])
    _pin(crank.component, r, 0.0, d, 0.0, tc + GAP + tl, f"Pin Ø{d:g}")
    lever_sec = dict(ls, hole=d, hole0=d, hole1=d, boss0=max(wl, 2.5 * d), boss1=wl)
    lever, _ = make_link(root, "Lever", (0.0, 0.0), (0.0, L), z_l, lever_sec)
    lb = lever.component.bRepBodies.item(0)
    _cut_slot(lever.component, lb, (dd - r - CLEAR, 0.0), (dd + r + CLEAR, 0.0), s, 0.0, tl)
    _pin(lever.component, L, 0.0, d, 0.0, tl + GAP + tl, f"Pin Ø{d:g}")
    ram, body = _block(root, gc, "Ram", 0.0, y_lo, y_hi, wr, tl, z_r, (y_lo_pin - CLEAR, L + CLEAR), s, (-half, half),
                       half + wl / 2 + 2)
    ground.isGrounded = True

    _revolute(root, ground, lever, hole_edge(lever, 0.0, lever_sec), "Ground-Lever (O)")
    jc = _revolute(root, ground, crank, hole_edge(crank, 0.0, cs), "Ground-Crank (O2)")
    _slider(root, ground, ram, body)
    sign = _sense(jc, crank)

    def lever_ok():
        th = math.radians(g["theta0"]) + sign * adsk.fusion.RevoluteJointMotion.cast(jc.jointMotion).rotationValue
        P = (r * math.cos(th), dd + r * math.sin(th))
        want = math.atan2(P[0], P[1])                     # from +y towards +x
        t = lever.transform2                              # the lever's local x runs from O to the tip
        got = math.atan2(t.getCell(0, 0), t.getCell(1, 0))
        return abs(math.remainder(got - want, 2 * math.pi)) < 1e-3

    _pin_slot(root, crank, lever, hole_edge(crank, r, cs), "Crank-Lever (pin in slot)", lambda: _turns_like(jc, lever_ok))

    def ram_ok():
        th = math.radians(g["theta0"]) + sign * adsk.fusion.RevoluteJointMotion.cast(jc.jointMotion).rotationValue
        P = (r * math.cos(th), dd + r * math.sin(th))
        return abs(ram.transform2.translation.x * 10 - L * math.sin(math.atan2(P[0], P[1]))) < 0.05

    _pin_slot(root, lever, ram, hole_edge(lever, L, lever_sec), "Lever-Ram (pin in slot)", lambda: _turns_like(jc, ram_ok))
    return crank


def build(mech, root, design=None):
    return build_yoke(mech, root) if mech["type"] == "scotch_yoke" else build_quick_return(mech, root)
