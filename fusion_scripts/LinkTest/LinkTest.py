"""
First Fusion 360 test: draws a single mechanism link.

Link = a bar with two rounded ends + two pin holes, then extruded.
Note: inside the Fusion API every length is in CENTIMETRES. We think in mm and convert with mm().
"""

import math
import traceback

import adsk.core
import adsk.fusion

# --- Link dimensions (mm) ---
LINK_LENGTH = 100.0   # between the two hole centres
LINK_WIDTH = 16.0     # bar width
HOLE_DIAMETER = 5.0   # pin hole
THICKNESS = 5.0       # thickness (extrude)


def mm(value):
    """mm -> Fusion's internal unit (cm)."""
    return value / 10.0


def draw_link(sketch, length, width, hole_d):
    """Draw the link profile in the sketch: two lines + two arcs + two holes."""
    r = width / 2.0
    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs
    circles = sketch.sketchCurves.sketchCircles
    P = adsk.core.Point3D.create

    # Top and bottom edges
    top = lines.addByTwoPoints(P(0, mm(r), 0), P(mm(length), mm(r), 0))
    bottom = lines.addByTwoPoints(P(mm(length), mm(-r), 0), P(0, mm(-r), 0))

    # Right end: from the bottom, 180 degrees counter-clockwise
    arcs.addByCenterStartSweep(P(mm(length), 0, 0), bottom.startSketchPoint, math.pi)
    # Left end: from the top, 180 degrees counter-clockwise
    arcs.addByCenterStartSweep(P(0, 0, 0), top.startSketchPoint, math.pi)

    # Pin holes
    circles.addByCenterRadius(P(0, 0, 0), mm(hole_d / 2.0))
    circles.addByCenterRadius(P(mm(length), 0, 0), mm(hole_d / 2.0))


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Open a clean design document
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent

        sketch = root.sketches.add(root.xYConstructionPlane)
        sketch.name = "Link profile"
        draw_link(sketch, LINK_LENGTH, LINK_WIDTH, HOLE_DIAMETER)

        # The largest profile = the body without the holes (the holes are their own small profiles)
        body_profile = max(sketch.profiles, key=lambda p: p.areaProperties().area)

        extrudes = root.features.extrudeFeatures
        extrude = extrudes.addSimple(
            body_profile,
            adsk.core.ValueInput.createByReal(mm(THICKNESS)),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        )
        extrude.bodies.item(0).name = "Link"

        app.activeViewport.fit()
        ui.messageBox(
            f"Link created!\n\n"
            f"Hole spacing: {LINK_LENGTH:g} mm\n"
            f"Width: {LINK_WIDTH:g} mm\n"
            f"Hole diameter: {HOLE_DIAMETER:g} mm\n"
            f"Thickness: {THICKNESS:g} mm"
        )
    except Exception:
        if ui:
            ui.messageBox("Error:\n{}".format(traceback.format_exc()))
