"""
F4c: Builds the Whitworth quick return in Fusion 360 (runs inside Fusion only).

The crank (radius r) turns about O2 = (0, d) with d < r, so its circle surrounds the lever pivot O. As on real
Whitworth machines the crank is a RING turning round a fixed hub at O2 that carries the lever's pivot pin at O.
Layers: base plate | hub (ground) and crank ring (+ its pin, up) | slotted lever (two arms: the slot on one, the
rod pin on the other at R) | rod | ram (sliding along x, its pin down into the rod).
Joints: Ground-Crank (O2) (the add-in's input), Ground-Lever (O), Crank-Lever (pin in slot, trial-checked),
Lever-Rod, Rod-Ram, Ground-Ram (slider along x, trial-checked).
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import GAP, hole_edge, make_link, mm
from fusion_lib.slider_crank import PLATE_T, _box
from fusion_lib.slotted import CLEAR, _cut_slot, _pin, _pin_slot, _plane_sketch, _revolute, _turns_like

TH0 = 90.0            # drawn with the crank pin straight above O2


def _ring(comp, c, r_out, r_in, z0, h, name):
    sk, to = _plane_sketch(comp, z0)
    sk.sketchCurves.sketchCircles.addByCenterRadius(to(*c), mm(r_out))
    if r_in:
        sk.sketchCurves.sketchCircles.addByCenterRadius(to(*c), mm(r_in))
    prof = next(p for p in sk.profiles if p.profileLoops.count == (2 if r_in else 1))
    body = comp.features.extrudeFeatures.addSimple(prof, adsk.core.ValueInput.createByReal(mm(h)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def _circle_edge(occ, body, r, centre):
    for e in body.edges:
        g = e.geometry
        if isinstance(g, adsk.core.Circle3D) and abs(g.radius - mm(r)) < 1e-4 and \
                abs(g.center.x - mm(centre[0])) < 1e-4 and abs(g.center.y - mm(centre[1])) < 1e-4:
            return e.createForAssemblyContext(occ)
    raise RuntimeError("Whitworth: circular edge not found")


def build(g, root, design=None):
    r, dd, R, l, sec = g["r"], g["d"], g["R"], g["l"], g["section_mm"]
    dp = sec["hole"]
    tc = sec["links"]["Crank"]["thickness"]
    ls = sec["links"]["Lever"]
    wl, tl = ls["width"], ls["thickness"]
    tr = tl
    z_c = GAP
    z_l = z_c + tc + GAP
    z_r = z_l + tl + GAP
    z_m = z_r + tr + GAP
    O2 = (0.0, dd)
    rho_min, rho_max = r - dd, r + dd
    hub = dd + dp / 2 + 3.0
    xB = math.sqrt(l * l - R * R)
    P = lambda x, y: adsk.core.Point3D.create(mm(x), mm(y), 0)

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gc = ground.component
    gc.name = "Ground"
    _box(gc, -(r + 30), -(R + 25), xB + R + 60, dd + r + 25, -PLATE_T, PLATE_T, "Base")
    hub_body = _ring(gc, O2, hub, 0, 0.0, z_c + tc, "Hub")
    _pin(gc, 0.0, 0.0, dp, z_c + tc, z_l + tl - (z_c + tc), f"Lever pivot Ø{dp:g}")
    ground.isGrounded = True

    crank = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    cc = crank.component
    cc.name = "Crank"
    ring = _ring(cc, O2, r + 1.5 * dp, hub + CLEAR, z_c, tc, "Crank ring")
    pin_body = _pin(cc, 0.0, dd + r, dp, z_c, z_l + tl - z_c, f"Pin Ø{dp:g}")

    lever_sec = dict(ls, hole=dp, hole0=dp, hole1=dp, boss0=max(wl, 2.5 * dp), boss1=wl, mid_holes=[R])
    tip = rho_max + dp + 4
    lever, L_len = make_link(root, "Lever", (0.0, -R), (0.0, tip), z_l, lever_sec)
    lb = lever.component.bRepBodies.item(0)
    _cut_slot(lever.component, lb, (R + rho_min - CLEAR, 0.0), (R + rho_max + CLEAR, 0.0), dp + 2 * CLEAR, 0.0, tl)
    _pin(lever.component, 0.0, 0.0, dp, 0.0, tl + GAP + tr, f"Rod pin Ø{dp:g}")

    rod_sec = dict(ls, width=max(wl * 0.7, 2.5 * dp), hole=dp, hole0=dp, hole1=dp)
    rod, rod_len = make_link(root, "Rod", (0.0, -R), (xB, 0.0), z_r, rod_sec)

    mt = adsk.core.Matrix3D.create()
    mt.translation = adsk.core.Vector3D.create(mm(xB), 0, mm(z_m))
    ram = root.occurrences.addNewComponent(mt)
    rc = ram.component
    rc.name = "Ram"
    wr = max(4 * dp, 24.0)
    ram_body = _box(rc, -wr, -wr / 2, wr, wr / 2, 0.0, tl, "Ram")
    _pin(rc, 0.0, 0.0, dp, -(tr + GAP), tr + GAP, f"Pin Ø{dp:g}")

    jc = _revolute(root, ground, crank, _circle_edge(crank, ring, hub + CLEAR, O2), "Ground-Crank (O2)")
    _revolute(root, ground, lever, hole_edge(lever, R, lever_sec), "Ground-Lever (O)")
    mo = adsk.fusion.RevoluteJointMotion.cast(jc.jointMotion)

    def crank_angle():
        t = crank.transform2
        return math.radians(TH0) + math.atan2(t.getCell(1, 0), t.getCell(0, 0))

    def lever_ok():
        th = crank_angle()
        Pp = (r * math.cos(th), dd + r * math.sin(th))
        want = math.atan2(Pp[1], Pp[0])                        # the slot arm points at the crank pin
        t = lever.transform2
        got = math.atan2(t.getCell(1, 0), t.getCell(0, 0))     # local x: from the rod pin towards the slot arm
        return abs(math.remainder(got - want, 2 * math.pi)) < 1e-3

    top = next(e for e in pin_body.edges if isinstance(e.geometry, adsk.core.Circle3D))
    _pin_slot(root, crank, lever, top.createForAssemblyContext(crank), "Crank-Lever (pin in slot)",
              lambda: _turns_like(jc, lever_ok))
    _revolute(root, lever, rod, hole_edge(rod, 0.0, rod_sec), "Lever-Rod")
    _revolute(root, rod, ram, hole_edge(rod, rod_len, rod_sec), "Rod-Ram")

    def ram_ok():
        th = crank_angle()
        Pp = (r * math.cos(th), dd + r * math.sin(th))
        a = math.atan2(Pp[1], Pp[0]) + math.pi
        x = R * math.cos(a) + math.sqrt(l * l - (R * math.sin(a)) ** 2)
        return abs(ram.transform2.translation.x * 10 - x) < 0.05

    edge = _circle_edge(ram, rc.bRepBodies.item(1), dp / 2, (0.0, 0.0))    # local coordinates
    for direction in (adsk.fusion.JointDirections.XAxisJointDirection, adsk.fusion.JointDirections.YAxisJointDirection):
        geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(ram, ground, geo)
        ji.setAsSliderJointMotion(direction)
        js = root.asBuiltJoints.add(ji)
        js.name = "Ground-Ram"
        if _turns_like(jc, ram_ok):
            return crank
        js.deleteMe()
    raise RuntimeError("Whitworth: the ram does not follow the lever")
