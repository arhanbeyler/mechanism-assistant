"""
H2: Draws a fillet-welded cantilever bar in Fusion 360 (runs inside Fusion only).

Local frame: the wall face is z = 0 (wall behind, z < 0), the bar b x d runs along +z; every weld line is a
triangular prism (legs h on the wall and on the bar) extruded along its edge. The three components (Wall,
Bar, Weld) are laid so that the bar sticks out horizontally from a vertical wall. Nothing moves.
"""

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

WALL_T = 12.0


def _placement():
    """local x -> global y, local y -> global z (up), local z -> global x (the bar's length)."""
    mt = adsk.core.Matrix3D.create()
    for col, v in enumerate(((0, 1, 0), (0, 0, 1), (1, 0, 0))):
        for row in range(3):
            mt.setCell(row, col, v[row])
    return mt


def _box(comp, x0, y0, x1, y1, z0, h, name):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    to = lambda x, y: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x), mm(y), mm(z0)))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(to(x0, y0), to(x1, y1))
    body = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(h)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def _bead(comp, plane, tri, half, name):
    """Triangle (three model points) on a plane through the origin, extruded symmetrically by +-half."""
    sk = comp.sketches.add(plane)
    to = lambda p: sk.modelToSketchSpace(adsk.core.Point3D.create(mm(p[0]), mm(p[1]), mm(p[2])))
    lines = sk.sketchCurves.sketchLines
    a = lines.addByTwoPoints(to(tri[0]), to(tri[1]))
    b = lines.addByTwoPoints(a.endSketchPoint, to(tri[2]))
    lines.addByTwoPoints(b.endSketchPoint, a.startSketchPoint)
    ext = comp.features.extrudeFeatures
    fi = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    fi.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(half)), False)     # half each side
    body = ext.add(fi).bodies.item(0)
    body.name = name


def build(g, root, design=None):
    b, d, h, L, pattern = g["b"], g["d"], g["h"], g["L"], g["pattern"]
    W = max(b, d) * 2.5 + 2 * h
    New = adsk.fusion.FeatureOperations.NewBodyFeatureOperation

    wall = root.occurrences.addNewComponent(_placement())
    wall.component.name = "Wall"
    body = _box(wall.component, -W / 2, -W / 2, W / 2, W / 2, -WALL_T, WALL_T, "Wall")
    wall.isGrounded = True

    bar = root.occurrences.addNewComponent(_placement())
    bar.component.name = "Bar"
    _box(bar.component, -b / 2, -d / 2, b / 2, d / 2, 0.0, L + d / 2, f"Bar {b:g}x{d:g}")
    bar.isGrounded = True

    weld = root.occurrences.addNewComponent(_placement())
    wc = weld.component
    wc.name = "Weld"
    if pattern in ("top_bottom", "all_around"):
        for s in (1, -1):                                   # along x, on the top / bottom faces (y = +-d/2)
            y = s * d / 2
            _bead(wc, wc.yZConstructionPlane, ((0, y, 0), (0, y + s * h, 0), (0, y, h)), b / 2, f"Weld {'top' if s > 0 else 'bottom'} h{h:g}")
    if pattern in ("sides", "all_around"):
        for s in (1, -1):                                   # along y, on the side faces (x = +-b/2)
            x = s * b / 2
            _bead(wc, wc.xZConstructionPlane, ((x, 0, 0), (x + s * h, 0, 0), (x, 0, h)), d / 2, f"Weld side h{h:g}")
    weld.isGrounded = True
    return bar
