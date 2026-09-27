"""
I1: Draws a cantilever snap-fit in Fusion 360 (runs inside Fusion only), snapped in place.

The snap part (base, beam and hook) is one side profile in the x-z plane extruded by the width: beam along
+x (length l, thickness h), hook of height y at the tip with the insertion ramp towards the tip and the
retaining face behind it. The catch (the mating part) lies on the beam with its edge against the retaining
face.
Animation: FRAMES frames of the catch sliding in from beyond the hook's tip, shown in turn by the add-in: the
beam bends down (cantilever shape, tip deflection d(u) = delta u^2 (3 - u) / 2) just enough for the hook to pass
under the catch, then springs back as it snaps in.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm
from fusion_lib.spring import frame_occurrences

BASE = 6.0
TOP = 0.5            # flat on top of the hook
FRAMES = 7
CLEAR = 0.05         # the drawn beam's straight segments cut the bent curve's chords: a little more bend


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


def geometry(g):
    l, h, y = g["l"], g["h"], g["y"]
    a_in, a_out = math.radians(g["alpha_in"]), math.radians(g["alpha_out"])
    x_ramp = l - y / math.tan(a_in)
    x_top = x_ramp - TOP
    x_r = x_top - (0.0 if g["alpha_out"] >= 89.9 else y / math.tan(a_out))
    return x_ramp, x_top, x_r, max(0.5, x_r - 10)


def hook_height(g, x):
    """The hook's height above the beam's top at x (0 off the hook)."""
    l, y = g["l"], g["y"]
    x_ramp, x_top, x_r, _ = geometry(g)
    if x <= x_r or x >= l:
        return 0.0
    if x < x_top:
        return y * (x - x_r) / (x_top - x_r)
    if x <= x_ramp:
        return y
    return y * (l - x) / (l - x_ramp)


def shape(u):
    return u * u * (3 - u) / 2


def deflection(g, s):
    """How far the beam's tip must bend down for the catch, shifted s beyond its place, to clear the hook."""
    x_ramp, x_top, x_r, x0 = geometry(g)
    xs = [x0 + s + (x_r - x0) * i / 40 for i in range(41)]
    return max((hook_height(g, x) / shape(x / g["l"]) for x in xs if 0 < x < g["l"]), default=0.0)


def _frame(comp, g, s):
    l, h, w, y = g["l"], g["h"], g["w"], g["y"]
    x_ramp, x_top, x_r, x0 = geometry(g)
    d = deflection(g, s)
    d += CLEAR if d > 0 else 0.0
    bend = lambda x, z: (x, z - d * shape(max(0.0, x) / l))
    pts = ([(-BASE, 0), (0, 0)] + [(l * i / 12, 0) for i in range(1, 12)]                 # beam split so it curves
           + [(l, 0), (l, h), (x_ramp, h + y), (x_top, h + y), (x_r, h)]
           + [(x_r * (1 - i / 10), h) for i in range(1, 10)] + [(0, h), (0, BASE), (-BASE, BASE)])
    _profile_body(comp, [bend(*p) for p in pts], w, f"Snap beam {l:g}x{h:g}x{w:g}")
    body = _profile_body(comp, [(x0 + s, h), (x_r + s, h), (x_r + s, h + y + 2), (x0 + s, h + y + 2)], w + 10, "Catch")
    body.opacity = 0.5
    return f"{s:.1f} mm to go"


def build(g, root, design=None):
    x_ramp, x_top, x_r, x0 = geometry(g)
    s_max = g["l"] - x0 + 1.0                       # the catch clear of the hook's tip
    # the catch slides in (the last step before 0 still on the hook's top), then snaps home
    steps = [s_max * (1 - k / (FRAMES - 2)) for k in range(FRAMES - 2)] + [0.3 * (x_top - x_r) + 0.2, 0.0]
    return frame_occurrences(root, design, FRAMES, lambda comp, f: _frame(comp, g, steps[round(f * (FRAMES - 1))]),
                             "Snap")[0]
