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

# Varsayilan kesit (mech['section_mm'] yoksa). Yuk verildiyse strength.py hesaplar:
# pim capi (A, B), mil capi (O2, O4) + uzuv basina genislik/kalinlik.
DEFAULT_SECTION = {"hole": 4.0, "shaft": 4.0, "links": {n: {"width": 10.0, "thickness": 4.0}
                                                        for n in ("Sabit", "Krank", "Biyel", "Kol")}}
SHAFT_POINTS = {"O2", "O4"}   # tahrik ve cikis milleri; A, B serbest pim
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


def build_link_body(comp, length, sec):
    """Bilesende uzuv govdesi: once dolu dis hat (w genisliginde govde + uclarda gobek, cap
    sec["boss0"]/["boss1"], yoksa w) extrude, sonra delikler (sec["hole0"]/["hole1"], yoksa
    sec["hole"]) ayri cizimden kesilir. (Govde delikten dar olunca tek cizimde bolge ayiklamak
    guvenilir degildi.) Dondurur: BRepBody."""
    w, t = sec["width"], sec["thickness"]
    d0 = sec.get("hole0", sec["hole"])
    d1 = sec.get("hole1", sec["hole"])
    R0 = max(w, sec.get("boss0", w)) / 2.0
    R1 = max(w, sec.get("boss1", w)) / 2.0
    P = adsk.core.Point3D.create
    dist = adsk.core.ValueInput.createByReal(mm(t))
    extrudes = comp.features.extrudeFeatures

    outline = comp.sketches.add(comp.xYConstructionPlane)
    outline.name = "Dis hat"
    outline.sketchCurves.sketchLines.addTwoPointRectangle(P(0, mm(-w / 2), 0), P(mm(length), mm(w / 2), 0))
    outline.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(R0))
    outline.sketchCurves.sketchCircles.addByCenterRadius(P(mm(length), 0, 0), mm(R1))
    solid = adsk.core.ObjectCollection.create()
    for prof in outline.profiles:
        solid.add(prof)
    body = extrudes.addSimple(solid, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)

    holes = comp.sketches.add(comp.xYConstructionPlane)
    holes.name = "Delikler"
    holes.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(d0 / 2))
    holes.sketchCurves.sketchCircles.addByCenterRadius(P(mm(length), 0, 0), mm(d1 / 2))
    cut = adsk.core.ObjectCollection.create()
    for prof in holes.profiles:
        cut.add(prof)
    extrudes.addSimple(cut, dist, adsk.fusion.FeatureOperations.CutFeatureOperation)
    return body


def make_link(root, name, p_start, p_end, z_mm, sec):
    """Bir uzvu yeni bilesen olarak olusturur ve z_mm yuksekligine yerlestirir.
    sec: {"width", "thickness", "hole"} (bu uzvun kesiti)."""
    dx, dy = p_end[0] - p_start[0], p_end[1] - p_start[1]
    length = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)

    m = adsk.core.Matrix3D.create()
    m.setToRotation(angle, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    m.translation = adsk.core.Vector3D.create(mm(p_start[0]), mm(p_start[1]), mm(z_mm))

    occ = root.occurrences.addNewComponent(m)
    comp = occ.component
    comp.name = name

    build_link_body(comp, length, sec).name = name
    return occ, length


def hole_edge(occ, local_x_mm, sec):
    """Uzvun yerel x konumundaki deligin ust dairesel kenari (montaj baglaminda)."""
    body = occ.component.bRepBodies.item(0)
    d = sec.get("hole0" if local_x_mm < 1e-9 else "hole1", sec["hole"])
    for edge in body.edges:
        geo = edge.geometry
        # Kisa uzuvlarda Fusion delik kenarini tam daire yerine yaylara bolebiliyor
        if isinstance(geo, (adsk.core.Circle3D, adsk.core.Arc3D)) \
                and abs(geo.radius - mm(d / 2)) < 1e-3 \
                and abs(geo.center.x - mm(local_x_mm)) < 1e-3 \
                and abs(geo.center.y) < 1e-3 \
                and abs(geo.center.z - mm(sec["thickness"])) < 1e-3:
            return edge.createForAssemblyContext(occ)
    raise RuntimeError(f"{occ.component.name}: x={local_x_mm} mm'de delik kenari bulunamadi")


MARK_GROUP, MARK_NAME = "MekanizmaAsistani", "fourbar"   # eklentinin cizdigi belgenin etiketi


def _find_marked_design(app):
    """Daha once bu eklentinin cizdigi (etiketli) acik belge; yoksa (None, None)."""
    for i in range(app.documents.count):
        doc = app.documents.item(i)
        try:
            des = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        except Exception:
            continue
        if des and des.attributes.itemByName(MARK_GROUP, MARK_NAME):
            return doc, des
    return None, None


def _clear(design):
    """Onceki mekanizmayi sil (sadece etiketli belgede cagrilir)."""
    root = design.rootComponent
    for ml in list(root.motionLinks):        # disli: once mafsallari baglayan motion link
        ml.deleteMe()
    for j in list(root.asBuiltJoints):
        j.deleteMe()
    for o in list(root.occurrences):
        o.deleteMe()


def build(mech, reuse=True):
    """Mekanizmayi kurar. reuse=True: eklentinin daha once cizdigi belge aciksa onu gunceller
    (yeni sekme acmaz); yoksa yeni belge acar. Dondurur: tasarim (Design)."""
    app = adsk.core.Application.get()
    doc, design = _find_marked_design(app) if reuse else (None, None)
    if doc:
        doc.activate()
        _clear(design)
    else:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.attributes.add(MARK_GROUP, MARK_NAME, "1")
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    for key, owner in (("cam_table", "cam"), ("cam_slide_sign", "cam"), ("spring_frames", "spring")):
        old = design.attributes.itemByName(MARK_GROUP, key)
        if old and mech.get("type") != owner:
            old.deleteMe()                    # onceki tasarimin animasyon verisi kalmasin
    other = {"spur_gear": "gear",             # C9: disli cifti
             "spring": "spring",              # C10: basma yayi (+ sikisma animasyonu kareleri)
             "cam": "cam",                    # C11a: kam + itici
             "shaft": "shaft"}.get(mech.get("type"))   # C11b: iki yatakli mil
    if other:
        import importlib
        importlib.import_module(f"fusion_lib.{other}").build(mech, root, design)
        app.activeViewport.fit()
        return design
    pts = mech["points_mm"]
    section = mech.get("section_mm") or DEFAULT_SECTION
    d_pin = section["hole"]
    d_shaft = section.get("shaft", d_pin)

    def hole_at(p):
        return d_shaft if p in SHAFT_POINTS else d_pin

    secs = {n: dict(section["links"][n], hole=d_pin, hole0=hole_at(s), hole1=hole_at(e))
            for n, (s, e, _) in LINKS.items()}

    # Katman sirasi motordan (interference.choose_layer_order); yoksa LINKS'teki varsayilan
    order = mech.get("layer_order") or [n for n, _ in sorted(LINKS.items(), key=lambda kv: kv[1][2])]
    occs, lengths, z = {}, {}, 0.0
    for name in order:
        s, e, _ = LINKS[name]
        occs[name], lengths[name] = make_link(root, name, pts[s], pts[e], z, secs[name])
        z += secs[name]["thickness"] + GAP
    occs["Sabit"].isGrounded = True

    # Her uzvun hangi ucu hangi noktada: baslangic -> x=0, bitis -> x=L
    def local_x(link, point):
        s, _, _ = LINKS[link]
        return 0.0 if point == s else lengths[link]

    for l1, l2, p in JOINTS:
        geo = adsk.fusion.JointGeometry.createByCurve(
            hole_edge(occs[l1], local_x(l1, p), secs[l1]),
            adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        ji = root.asBuiltJoints.createInput(occs[l1], occs[l2], geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        joint = root.asBuiltJoints.add(ji)
        joint.name = f"{l1}-{l2} ({p})"

    if mech.get("input_range_deg"):
        _limit_input_joint(root, occs["Krank"], mech["input_range_deg"])
    app.activeViewport.fit()
    return design


def _occ_angle(occ):
    t = occ.transform2
    return math.atan2(t.getCell(1, 0), t.getCell(0, 0))


def _limit_input_joint(root, crank_occ, range_deg):
    """Sinirli acili mekanizma (B9): giris mafsalina donme siniri koy. range_deg, cizilen konuma gore
    giris acisi araligi (theta artisi = saat yonu tersi). Mafsalin pozitif yonu kucuk bir deneme
    donusuyle olculur, sinirlar ona gore yerlestirilir."""
    joint = [j for j in root.asBuiltJoints if j.name.startswith("Sabit-Krank")][0]
    motion = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    a0 = _occ_angle(crank_occ)
    motion.rotationValue = math.radians(2)
    turned = (_occ_angle(crank_occ) - a0 + math.pi) % (2 * math.pi) - math.pi
    motion.rotationValue = 0.0
    sign = 1.0 if turned > 0 else -1.0
    lo, hi = sorted(sign * math.radians(v) for v in range_deg)
    limits = motion.rotationLimits
    limits.isMinimumValueEnabled = True
    limits.minimumValue = lo
    limits.isMaximumValueEnabled = True
    limits.maximumValue = hi
    joint.name = "Sabit-Krank (O2, sinirli)"
