"""
Ilk Fusion 360 testi: tek bir mekanizma uzvu (link) cizer.

Uzuv = iki ucu yuvarlatilmis cubuk + iki pim deligi, sonra extrude.
Dikkat: Fusion API icinde tum uzunluklar SANTIMETRE. Biz mm dusunup
mm() fonksiyonuyla ceviriyoruz.
"""

import math
import traceback

import adsk.core
import adsk.fusion

# --- Uzuv olculeri (mm) ---
LINK_LENGTH = 100.0   # iki delik merkezi arasi
LINK_WIDTH = 16.0     # cubuk genisligi
HOLE_DIAMETER = 5.0   # pim deligi
THICKNESS = 5.0       # kalinlik (extrude)


def mm(value):
    """mm -> Fusion'un ic birimi (cm)."""
    return value / 10.0


def draw_link(sketch, length, width, hole_d):
    """Sketch'e uzuv profilini ciz: iki cizgi + iki yay + iki delik."""
    r = width / 2.0
    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs
    circles = sketch.sketchCurves.sketchCircles
    P = adsk.core.Point3D.create

    # Ust ve alt kenar
    top = lines.addByTwoPoints(P(0, mm(r), 0), P(mm(length), mm(r), 0))
    bottom = lines.addByTwoPoints(P(mm(length), mm(-r), 0), P(0, mm(-r), 0))

    # Sag uc: alttan baslayip saat yonu tersine 180 derece
    arcs.addByCenterStartSweep(P(mm(length), 0, 0), bottom.startSketchPoint, math.pi)
    # Sol uc: ustten baslayip saat yonu tersine 180 derece
    arcs.addByCenterStartSweep(P(0, 0, 0), top.startSketchPoint, math.pi)

    # Pim delikleri
    circles.addByCenterRadius(P(0, 0, 0), mm(hole_d / 2.0))
    circles.addByCenterRadius(P(mm(length), 0, 0), mm(hole_d / 2.0))


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Temiz bir tasarim dokumani ac
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent

        sketch = root.sketches.add(root.xYConstructionPlane)
        sketch.name = "Uzuv profili"
        draw_link(sketch, LINK_LENGTH, LINK_WIDTH, HOLE_DIAMETER)

        # En buyuk alanli profil = deliksiz govde (delikler kendi kucuk profilleri)
        body_profile = max(sketch.profiles, key=lambda p: p.areaProperties().area)

        extrudes = root.features.extrudeFeatures
        extrude = extrudes.addSimple(
            body_profile,
            adsk.core.ValueInput.createByReal(mm(THICKNESS)),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        )
        extrude.bodies.item(0).name = "Uzuv"

        app.activeViewport.fit()
        ui.messageBox(
            f"Uzuv olusturuldu!\n\n"
            f"Delikler arasi: {LINK_LENGTH:g} mm\n"
            f"Genislik: {LINK_WIDTH:g} mm\n"
            f"Delik capi: {HOLE_DIAMETER:g} mm\n"
            f"Kalinlik: {THICKNESS:g} mm"
        )
    except Exception:
        if ui:
            ui.messageBox("Hata:\n{}".format(traceback.format_exc()))
