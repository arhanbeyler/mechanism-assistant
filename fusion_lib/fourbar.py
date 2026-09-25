"""
Dort cubuk mekanizmasini Fusion 360'ta kurar (sadece Fusion icinde calisir).

build(mech) -> mech: kinematics.to_fusion_dict() ciktisi.
4 uzvu ayri bilesen olarak cizer, dogru konuma yerlestirir ve 4 adet
Revolute (as-built) joint ile baglar. Sabit uzuv (ground) sabitlenir.
Uzuvlar birbirine carpmasin diye z yonunde katmanlara dizilir.
"""

import math

import adsk.core
import adsk.fusion

# Varsayilan kesit (mech['section_mm'] yoksa). Yuk verildiyse strength.py hesaplar.
DEFAULT_SECTION = {"width": 10.0, "thickness": 4.0, "hole": 4.0}
GAP = 0.5          # mm, katmanlar arasi bosluk

# uzuv adi: (baslangic noktasi, bitis noktasi, katman)
LINKS = {
    "Sabit": ("O2", "O4", 0),
    "Krank": ("O2", "A", 1),
    "Biyel": ("A", "B", 2),
    "Kol":   ("O4", "B", 3),
}

# (uzuv1, uzuv2, ortak nokta)
JOINTS = [
    ("Sabit", "Krank", "O2"),
    ("Krank", "Biyel", "A"),
    ("Biyel", "Kol", "B"),
    ("Kol", "Sabit", "O4"),
]


def mm(v):
    """mm -> Fusion'un ic birimi (cm)."""
    return v / 10.0


def draw_link_sketch(sketch, length, sec):
    """Yerel koordinatta (0,0)-(length,0) arasi uzuv profili: iki cizgi, iki yay, iki delik."""
    r = sec["width"] / 2.0
    hole_r = sec["hole"] / 2.0
    P = adsk.core.Point3D.create
    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs
    circles = sketch.sketchCurves.sketchCircles
    top = lines.addByTwoPoints(P(0, mm(r), 0), P(mm(length), mm(r), 0))
    bottom = lines.addByTwoPoints(P(mm(length), mm(-r), 0), P(0, mm(-r), 0))
    arcs.addByCenterStartSweep(P(mm(length), 0, 0), bottom.startSketchPoint, math.pi)
    arcs.addByCenterStartSweep(P(0, 0, 0), top.startSketchPoint, math.pi)
    circles.addByCenterRadius(P(0, 0, 0), mm(hole_r))
    circles.addByCenterRadius(P(mm(length), 0, 0), mm(hole_r))


def make_link(root, name, p_start, p_end, layer, sec):
    """Bir uzvu yeni bilesen olarak olusturur ve yerine koyar."""
    dx, dy = p_end[0] - p_start[0], p_end[1] - p_start[1]
    length = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)

    m = adsk.core.Matrix3D.create()
    m.setToRotation(angle, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    m.translation = adsk.core.Vector3D.create(mm(p_start[0]), mm(p_start[1]), mm(layer * (sec["thickness"] + GAP)))

    occ = root.occurrences.addNewComponent(m)
    comp = occ.component
    comp.name = name

    sketch = comp.sketches.add(comp.xYConstructionPlane)
    draw_link_sketch(sketch, length, sec)
    prof = max(sketch.profiles, key=lambda p: p.areaProperties().area)
    ext = comp.features.extrudeFeatures.addSimple(
        prof, adsk.core.ValueInput.createByReal(mm(sec["thickness"])),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.bodies.item(0).name = name
    return occ, length


def hole_edge(occ, local_x_mm, sec):
    """Uzvun yerel x konumundaki deligin ust dairesel kenari (montaj baglaminda)."""
    body = occ.component.bRepBodies.item(0)
    for edge in body.edges:
        geo = edge.geometry
        if isinstance(geo, adsk.core.Circle3D) \
                and abs(geo.radius - mm(sec["hole"] / 2)) < 1e-4 \
                and abs(geo.center.x - mm(local_x_mm)) < 1e-4 \
                and abs(geo.center.z - mm(sec["thickness"])) < 1e-4:
            return edge.createForAssemblyContext(occ)
    raise RuntimeError(f"{occ.component.name}: x={local_x_mm} mm'de delik kenari bulunamadi")


def build(mech, new_document=True):
    """Mekanizmayi kurar. Dondurur: olusturulan tasarim (Design)."""
    app = adsk.core.Application.get()
    if new_document:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    pts = mech["points_mm"]
    sec = mech.get("section_mm") or DEFAULT_SECTION

    occs, lengths = {}, {}
    for name, (s, e, layer) in LINKS.items():
        occs[name], lengths[name] = make_link(root, name, pts[s], pts[e], layer, sec)
    occs["Sabit"].isGrounded = True

    # Her uzvun hangi ucu hangi noktada: baslangic -> x=0, bitis -> x=L
    def local_x(link, point):
        s, _, _ = LINKS[link]
        return 0.0 if point == s else lengths[link]

    for l1, l2, p in JOINTS:
        geo = adsk.fusion.JointGeometry.createByCurve(
            hole_edge(occs[l1], local_x(l1, p), sec),
            adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(occs[l1], occs[l2], geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        joint = root.asBuiltJoints.add(ji)
        joint.name = f"{l1}-{l2} ({p})"

    app.activeViewport.fit()
    return design
