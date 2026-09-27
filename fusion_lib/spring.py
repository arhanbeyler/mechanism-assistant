"""
C10: Builds the compression spring in Fusion 360 (runs inside Fusion only).

The API cannot create a new "coil" feature, so: wire centre line as a 3D spline (springs.helix_points,
varying radius for a conical spring, closed coils at the ends) -> wire section (circle) on a plane normal
to the path at its start -> sweep -> cut with the planes z = 0 and z = L0 (squared and ground ends).

Animation: bodies cannot flex in Fusion; the spring is drawn as FRAMES separate frames at different
compressions (0 -> working deflection), optionally with a top plate in each. The add-in shows them in turn.
"""

import adsk.core
import adsk.fusion

import springs
from fusion_lib.fourbar import MARK_GROUP, mm

END_GAP = 1.03    # pitch of the closed end coils = 1.03 d (full contact makes the sweep touch itself)
FRAMES = 8
PLATE_T = 3.0


def _plate(comp, R, z0, t, name):
    P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
    sk = comp.sketches.add(comp.constructionPlanes.add(pin))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(R))
    body = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByReal(mm(t)),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    return body


def _spring_body(comp, geo, compression):
    """Spring body compressed by `compression` mm (ground ends). Returns (body, z of the top face)."""
    d, L0 = geo["d"], geo["L0"]
    P = adsk.core.Point3D.create
    g = dict(geo)
    g["d"] = d * END_GAP
    g["pitch"] = (L0 - 2 * g["d"]) / geo["Na"]       # visual: end gap; L0 and k do not change
    pts = springs.helix_points(g, per_turn=16, compression=compression)
    z_top = pts[-1][2]

    path_sk = comp.sketches.add(comp.xYConstructionPlane)
    path_sk.name = "Wire axis (helix)"
    col = adsk.core.ObjectCollection.create()
    for x, y, z in pts:
        col.add(P(mm(x), mm(y), mm(z)))
    spline = path_sk.sketchCurves.sketchFittedSplines.add(col)
    path = comp.features.createPath(spline, False)       # in a sub-component: path in the component context

    pl_in = comp.constructionPlanes.createInput()
    pl_in.setByDistanceOnPath(path, adsk.core.ValueInput.createByReal(0.0))
    prof_sk = comp.sketches.add(comp.constructionPlanes.add(pl_in))
    center = prof_sk.modelToSketchSpace(P(mm(pts[0][0]), mm(pts[0][1]), mm(pts[0][2])))
    prof_sk.sketchCurves.sketchCircles.addByCenterRadius(center, mm(d / 2))
    sw_in = comp.features.sweepFeatures.createInput(prof_sk.profiles.item(0), path,
                                                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = comp.features.sweepFeatures.add(sw_in).bodies.item(0)
    body.name = "Spring"

    R = max(geo["D1"], geo["D2"]) / 2 + d * 2
    for z0, direction in ((0.0, -1.0), (z_top, 1.0)):   # grinding
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z0)))
        sk = comp.sketches.add(comp.constructionPlanes.add(pin))
        sk.sketchCurves.sketchLines.addTwoPointRectangle(P(mm(-R), mm(-R), 0), P(mm(R), mm(R), 0))
        cut = comp.features.extrudeFeatures.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
        cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(direction * 2 * d)))
        cut.participantBodies = [body]     # ONLY this frame's spring: by default it cut every body it met
        comp.features.extrudeFeatures.add(cut)   # (the other frames on top of each other and the base)
    return body, z_top


def build(geo, root, design=None):
    """geo["show_plates"]: also draw the base and top plates (default: the spring only)."""
    R_plate = max(geo["D1"], geo["D2"]) / 2 + geo["d"] + 4
    plates = bool(geo.get("show_plates"))

    if plates:
        base = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        base.component.name = "Base"
        _plate(base.component, R_plate, -PLATE_T, PLATE_T, "Base")
        base.isGrounded = True

    frames = []
    for k in range(FRAMES):
        delta = geo["y_work"] * k / (FRAMES - 1)
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = f"Spring {k} ({delta:.1f} mm)"
        _, z_top = _spring_body(occ.component, geo, delta)
        if plates:
            _plate(occ.component, R_plate, z_top, PLATE_T, "Top plate")
        occ.isGrounded = True
        occ.isLightBulbOn = (k == 0)
        frames.append(occ)

    mark_frames(design, FRAMES)
    return frames


def mark_frames(design, n):
    """Tell the add-in that the design is n spring frames ("Spring 0 ..." ... "Spring n-1 ...") to show in turn."""
    if design is None:
        return
    old = design.attributes.itemByName(MARK_GROUP, "spring_frames")
    if old:
        old.deleteMe()
    design.attributes.add(MARK_GROUP, "spring_frames", str(n))


def frame_occurrences(root, design, n, make, part="Spring"):
    """n grounded frame components "<part> k (label)", only the first visible; make(comp, frac) draws one (frac 0 =
    free ... 1 = working) and returns its label. Marks the design for the add-in's frame animation."""
    occs = []
    for k in range(n):
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = f"{part} {k} ({make(occ.component, k / (n - 1))})"
        occ.isGrounded = True
        occ.isLightBulbOn = (k == 0)
        occs.append(occ)
    mark_frames(design, n)
    return occs
