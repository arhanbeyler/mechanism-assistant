"""
I1: Draws a cantilever snap-fit in Fusion 360 (runs inside Fusion only), snapped in place.

The snap part (base, beam and hook) is one side profile in the x-z plane extruded by the width: beam along
+x (length l, thickness h), hook of height y at the tip with the insertion ramp towards the tip and the
retaining face behind it. The catch (the mating part) lies on the beam with its edge against the retaining
face. Nothing moves.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

BASE = 6.0
TOP = 0.5            # flat on top of the hook


def _profile_body(comp, pts, w, name):
    sk = comp.sketches.add(comp.xZConstructionPlane)
    to = lambda x, z: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), 0, mm(z)))
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to(*pts[0]), to(*pts[1]))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, to(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    ext = comp.features.extrudeFeatures
    fi = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    fi.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(w / 2)), False)
    body = ext.add(fi).bodies.item(0)
    body.name = name
    return body


def build(g, root, design=None):
    l, h, w, y = g["l"], g["h"], g["w"], g["y"]
    a_in, a_out = math.radians(g["alpha_in"]), math.radians(g["alpha_out"])
    x_ramp = l - y / math.tan(a_in)
    x_top = x_ramp - TOP
    x_r = x_top - (0.0 if g["alpha_out"] >= 89.9 else y / math.tan(a_out))
    snap = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    snap.component.name = "Snap"
    _profile_body(snap.component, [(-BASE, 0), (l, 0), (l, h), (x_ramp, h + y), (x_top, h + y), (x_r, h), (0, h),
                                   (0, BASE), (-BASE, BASE)], w, f"Snap beam {l:g}x{h:g}x{w:g}")
    snap.isGrounded = True
    catch = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    catch.component.name = "Catch"
    x0 = max(0.5, x_r - 10)
    body = _profile_body(catch.component, [(x0, h), (x_r, h), (x_r, h + y + 2), (x0, h + y + 2)], w + 10, "Catch")
    body.opacity = 0.5
    catch.isGrounded = True
    return snap
