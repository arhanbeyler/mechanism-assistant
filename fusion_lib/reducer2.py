"""
E2: Builds the two-stage reducer in Fusion 360 (runs inside Fusion only).

Frame as in reducer2.py: shaft axes along x in one plane (input y = 0, intermediate y = a1, output y = a1 + a2),
stage 1 at x1, stage 2 at x2, housing walls at -+span/2. Reuses the single-stage builder's pieces: housing,
gear placement matrix and hub keyway (fusion_lib.reducer), shafts and bearings (fusion_lib.shaft).
The three shafts are not turned (unlike the single stage's output shaft), so their joint axes point the same
way and each external mesh needs a reversed motion link; checked by the tooth-overlap test.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.gear import build_gear_body
from fusion_lib.reducer import _housing, _keyway, _matrix
from fusion_lib.shaft import _bearings, make_shaft, revolute_to

REVERSED = True


def build(r, root, design=None):
    (g1, g2), shafts_, h = r["stages"], r["shafts"], r["housing"]
    a1, a2 = g1["center_distance"], g2["center_distance"]
    ys = [0.0, a1, a1 + a2]
    x1, x2 = r["x_stages"]

    ground = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    ground.component.name = "Housing"
    _housing(ground.component, h)
    for i, (s, y) in enumerate(zip(shafts_, ys)):
        _bearings(ground.component, s["layout"], dx=-s["layout"]["shift"], dy=y, suffix=f" s{i + 1}")
    ground.isGrounded = True

    names = ("Input shaft", "Intermediate shaft", "Output shaft")
    occs = []
    for name, s, y in zip(names, shafts_, ys):
        occ, body = make_shaft(root, s["layout"], name, _matrix(0.0, 0.0, (-s["layout"]["shift"], y, 0.0)))
        occs.append((occ, body))

    # (component name, teeth, module, face, shaft index, x, key of that seat, phase)
    even = lambda z: math.pi / z if z % 2 == 0 else 0.0
    gear_list = [("Pinion 1", g1["z1"], g1["module"], g1["face_width"], 0, x1, shafts_[0]["keys"][0], 0.0),
                 ("Gear 2", g1["z2"], g1["module"], g1["face_width"], 1, x1, shafts_[1]["keys"][0], even(g1["z2"])),
                 ("Pinion 3", g2["z1"], g2["module"], g2["face_width"], 1, x2, shafts_[1]["keys"][1], 0.0),
                 ("Gear 4", g2["z2"], g2["module"], g2["face_width"], 2, x2, shafts_[2]["keys"][0], even(g2["z2"]))]
    for name, z, m, b, si, x, key, phase in gear_list:
        phi = math.pi / 2 + phase
        occ = root.occurrences.addNewComponent(_matrix(phi, math.pi / 2, (x - b / 2, ys[si], 0.0)))
        occ.component.name = name
        bore = shafts_[si]["d"]
        body = build_gear_body(occ.component, m, z, b, bore)
        body.name = f"{name} z{z}"
        if key:
            _keyway(occ.component, body, bore, key, phi, b)
        ji = root.asBuiltJoints.createInput(occ, occs[si][0], None)
        ji.setAsRigidJointMotion()
        root.asBuiltJoints.add(ji).name = f"{name} on shaft"

    joints = [revolute_to(root, ground, occ, body, s["layout"], jn)
              for (occ, body), s, jn in zip(occs, shafts_, ("Ground-Input", "Ground-Intermediate", "Ground-Output"))]
    for (ja, jb), (za, zb) in (((joints[0], joints[1]), (g1["z1"], g1["z2"])),
                               ((joints[1], joints[2]), (g2["z1"], g2["z2"]))):
        link = root.motionLinks.createInput(ja, jb)
        link.valueOne = adsk.core.ValueInput.createByString("360 deg")
        link.valueTwo = adsk.core.ValueInput.createByString(f"{360.0 * za / zb} deg")
        link.isReversed = REVERSED
        root.motionLinks.add(link)
    return [o for o, _ in occs]
