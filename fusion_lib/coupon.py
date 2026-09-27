"""
B8c (preparation): draws the print-in-place clearance coupon (printfit.py) in Fusion 360 (runs inside Fusion only).

One component per spinner, placed in a row: the pin and the ring are revolved from printfit's (r, z) sections, the
ring has a tab with its clearance engraved on top.  Both stand on the bed (z = 0); print as is, no supports.
"""

import adsk.core
import adsk.fusion

from fusion_lib.fourbar import mm

TAB = (7.0, 14.0)        # tab beyond the ring's outer radius (length along +y, width along x)
ENGRAVE = 0.6


def _revolve(comp, section, name):
    sk = comp.sketches.add(comp.xZConstructionPlane)
    lines = sk.sketchCurves.sketchLines
    pts = [sk.modelToSketchSpace(adsk.core.Point3D.create(mm(r), 0, mm(z))) for r, z in section]
    first = prev = None
    for i in range(len(pts)):
        ln = lines.addByTwoPoints(prev or pts[0], first if i == len(pts) - 1 else pts[i + 1])
        first = first or ln.startSketchPoint
        prev = ln.endSketchPoint
    rin = comp.features.revolveFeatures.createInput(sk.profiles.item(0), comp.zConstructionAxis,
                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    rin.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    body = comp.features.revolveFeatures.add(rin).bodies.item(0)
    body.name = name
    return body


def _tab(comp, ring, r_out, h, c):
    """A rectangular tab joined to the ring, the clearance engraved on its top."""
    ext = comp.features.extrudeFeatures
    sk = comp.sketches.add(comp.xYConstructionPlane)
    y0, y1, w = r_out * 0.6, r_out + TAB[0], TAB[1] / 2
    sk.sketchCurves.sketchLines.addTwoPointRectangle(adsk.core.Point3D.create(mm(-w), mm(y0), 0),
                                                     adsk.core.Point3D.create(mm(w), mm(y1), 0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), mm(r_out))
    prof = max(sk.profiles, key=lambda p: p.areaProperties().centroid.y)      # the tab outside the ring
    tin = ext.createInput(prof, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    tin.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(h)))
    tin.participantBodies = [ring]
    ext.add(tin)
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(h)))
    tsk = comp.sketches.add(comp.constructionPlanes.add(pin))
    ti = tsk.sketchTexts.createInput2(f"{c:.2f}", mm(2.5))
    ti.setAsMultiLine(tsk.modelToSketchSpace(adsk.core.Point3D.create(mm(-w + 0.5), mm(r_out + 1), mm(h))),
                      tsk.modelToSketchSpace(adsk.core.Point3D.create(mm(w - 0.5), mm(y1 - 1), mm(h))),
                      adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                      adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
    text = tsk.sketchTexts.add(ti)
    cin = ext.createInput(text, adsk.fusion.FeatureOperations.CutFeatureOperation)
    cin.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(-ENGRAVE)))
    cin.participantBodies = [ring]
    ext.add(cin)


def build(g, root, design=None):
    first = None
    for s in g["spinners"]:
        m = adsk.core.Matrix3D.create()
        m.translation = adsk.core.Vector3D.create(mm(s["x"]), 0, 0)
        occ = root.occurrences.addNewComponent(m)
        comp = occ.component
        comp.name = f"Spinner {s['c']:.2f}"
        _revolve(comp, s["pin"], f"Pin {s['c']:.2f}")
        ring = _revolve(comp, s["ring"], f"Ring {s['c']:.2f}")
        r_out = max(r for r, z in s["ring"])
        h = max(z for r, z in s["ring"])
        _tab(comp, ring, r_out, h, s["c"])
        occ.isGrounded = True
        first = first or occ
    return first
