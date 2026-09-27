"""
I2: Draws a printed living hinge in Fusion 360 (runs inside Fusion only): two panels joined by the thin strip
flush with their bottom face (one body, printed in one go).
Animation: FRAMES frames from flat (as printed) to the design angle, shown in turn by the add-in: the strip in
pure bending is a circular arc of its own length (radius L / phi), each panel turns by phi / 2 with the strip's end.
"""

import math

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm
from fusion_lib.spring import frame_occurrences

PANEL = 25.0
PANEL_T = 3.0
FRAMES = 6
SEG = 16


def outline(L, t, phi):
    """Section (x, z) of the hinge folded by phi (rad), symmetric about x = 0, the strip's middle on z = 0."""
    if phi < 1e-6:
        return [(-L / 2 - PANEL, 0), (L / 2 + PANEL, 0), (L / 2 + PANEL, PANEL_T), (L / 2, PANEL_T), (L / 2, t),
                (-L / 2, t), (-L / 2, PANEL_T), (-L / 2 - PANEL, PANEL_T)]
    rho = L / phi                                   # neutral axis radius; centre above the strip
    cz = t / 2 + rho

    def at(s, c):
        """Point c above the strip's bottom at arc length s (|s| <= L/2) along the strip."""
        b = s / rho
        r = rho + t / 2 - c
        return (r * math.sin(b), cz - r * math.cos(b))

    def panel(sign, a, c):
        """Point of a panel: a beyond the strip's end along its tangent, c above its bottom face."""
        b = sign * phi / 2
        ex, ez = at(sign * L / 2, c)
        return (ex + sign * a * math.cos(b), ez + sign * a * math.sin(b))

    bottom = [at(-L / 2 + L * i / SEG, 0.0) for i in range(SEG + 1)]
    top = [at(L / 2 - L * i / SEG, t) for i in range(SEG + 1)]
    return ([panel(-1, PANEL, 0.0)] + bottom + [panel(1, PANEL, 0.0), panel(1, PANEL, PANEL_T), panel(1, 0.0, PANEL_T)]
            + top + [panel(-1, 0.0, PANEL_T), panel(-1, PANEL, PANEL_T)])


def _frame(comp, g, phi):
    L, t, w = g["L"], g["t"], g["w"]
    sk = comp.sketches.add(comp.xZConstructionPlane)
    to = lambda x, z: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), 0, mm(z)))
    pts = outline(L, t, phi)
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to(*pts[0]), to(*pts[1]))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, to(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    ext = comp.features.extrudeFeatures
    fi = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    fi.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(w / 2)), False)
    ext.add(fi).bodies.item(0).name = f"Living hinge {L:g}x{t:g}"
    return f"{math.degrees(phi):.0f}°"


def build(g, root, design=None):
    th = math.radians(g.get("theta_deg") or 0.0)
    if th and g["L"] / th + g["t"] / 2 <= PANEL_T:     # the drawn panels' ends would meet before the full angle
        th = g["L"] / (PANEL_T - g["t"] / 2) * 0.95
    return frame_occurrences(root, design, FRAMES, lambda comp, f: _frame(comp, g, th * f), "Hinge")[0]
