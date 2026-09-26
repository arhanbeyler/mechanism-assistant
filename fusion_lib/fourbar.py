"""
Builds the four-bar linkage in Fusion 360 (runs inside Fusion only), and the shared document handling
used by every module (one tagged document is reused instead of opening new tabs).

build(mech) -> mech: kinematics.to_fusion_dict() output (+ section, layer order ...).
Draws the 4 links as separate components, places them and joins them with 4 revolute (as-built)
joints. The ground link is grounded. The links are stacked in z layers so that they do not collide.
"""

import math

import adsk.core
import adsk.fusion

# Default section (if mech['section_mm'] is missing). With a load, strength.py computes it:
# pin diameter (A, B), shaft diameter (O2, O4) + width / thickness per link.
DEFAULT_SECTION = {"hole": 4.0, "shaft": 4.0, "links": {n: {"width": 10.0, "thickness": 4.0}
                                                        for n in ("Ground", "Crank", "Coupler", "Rocker")}}
SHAFT_POINTS = {"O2", "O4"}   # drive and output shafts; A, B are free pins
GAP = 0.5          # mm, gap between layers

# link name: (start point, end point, layer)
LINKS = {
    "Ground": ("O2", "O4", 0),
    "Crank": ("O2", "A", 1),
    "Coupler": ("A", "B", 2),
    "Rocker": ("O4", "B", 3),
}

# (link 1, link 2, shared point)
JOINTS = [
    ("Ground", "Crank", "O2"),
    ("Crank", "Coupler", "A"),
    ("Coupler", "Rocker", "B"),
    ("Rocker", "Ground", "O4"),
]


def mm(v):
    """mm -> Fusion's internal unit (cm)."""
    return v / 10.0


def build_link_body(comp, length, sec):
    """Link body in the component: first the solid outline (web of width w + end bosses of diameter
    sec["boss0"]/["boss1"], else w) is extruded, then the holes (sec["hole0"]/["hole1"], else sec["hole"])
    are cut from a separate sketch. (Picking regions in one sketch was unreliable when the web was
    narrower than the hole.) sec["mid_holes"]: local x of extra pin holes along the link (F2: the
    straight-line coupler's B), each with a boss like the end's. Returns the BRepBody."""
    w, t = sec["width"], sec["thickness"]
    d0 = sec.get("hole0", sec["hole"])
    d1 = sec.get("hole1", sec["hole"])
    R0 = max(w, sec.get("boss0", w)) / 2.0
    R1 = max(w, sec.get("boss1", w)) / 2.0
    P = adsk.core.Point3D.create
    dist = adsk.core.ValueInput.createByReal(mm(t))
    extrudes = comp.features.extrudeFeatures

    outline = comp.sketches.add(comp.xYConstructionPlane)
    outline.name = "Outline"
    outline.sketchCurves.sketchLines.addTwoPointRectangle(P(0, mm(-w / 2), 0), P(mm(length), mm(w / 2), 0))
    outline.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(R0))
    outline.sketchCurves.sketchCircles.addByCenterRadius(P(mm(length), 0, 0), mm(R1))
    for x in sec.get("mid_holes", ()):
        outline.sketchCurves.sketchCircles.addByCenterRadius(P(mm(x), 0, 0), mm(R1))
    solid = adsk.core.ObjectCollection.create()
    for prof in outline.profiles:
        solid.add(prof)
    body = extrudes.addSimple(solid, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)

    holes = comp.sketches.add(comp.xYConstructionPlane)
    holes.name = "Holes"
    holes.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), mm(d0 / 2))
    holes.sketchCurves.sketchCircles.addByCenterRadius(P(mm(length), 0, 0), mm(d1 / 2))
    for x in sec.get("mid_holes", ()):
        holes.sketchCurves.sketchCircles.addByCenterRadius(P(mm(x), 0, 0), mm(sec["hole"] / 2))
    cut = adsk.core.ObjectCollection.create()
    for prof in holes.profiles:
        cut.add(prof)
    cin = extrudes.createInput(cut, adsk.fusion.FeatureOperations.CutFeatureOperation)
    cin.setDistanceExtent(False, dist)
    cin.participantBodies = [body]           # only this link (a cut hits every body it touches otherwise)
    extrudes.add(cin)
    return body


def make_link(root, name, p_start, p_end, z_mm, sec):
    """Creates one link as a new component and places it at height z_mm.
    sec: {"width", "thickness", "hole"} (this link's section)."""
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
    """Top circular edge of the link's hole at local x (in the assembly context)."""
    body = occ.component.bRepBodies.item(0)
    d = sec.get("hole0" if local_x_mm < 1e-9 else "hole1", sec["hole"])
    for edge in body.edges:
        geo = edge.geometry
        # On short links Fusion may split the hole edge into arcs instead of one full circle
        if isinstance(geo, (adsk.core.Circle3D, adsk.core.Arc3D)) \
                and abs(geo.radius - mm(d / 2)) < 1e-3 \
                and abs(geo.center.x - mm(local_x_mm)) < 1e-3 \
                and abs(geo.center.y) < 1e-3 \
                and abs(geo.center.z - mm(sec["thickness"])) < 1e-3:
            return edge.createForAssemblyContext(occ)
    raise RuntimeError(f"{occ.component.name}: no hole edge at x = {local_x_mm} mm")


# Tag of the document drawn by the add-in. Documents drawn before the add-in was renamed carry the old
# group; they are still found and moved to the new one (_retag).
MARK_GROUP, MARK_NAME = "MechanismAssistant", "fourbar"
LEGACY_GROUP = "MekanizmaAsistani"


def _find_marked_design(app):
    """The open document this add-in drew before (tagged); (None, None) if there is none."""
    for i in range(app.documents.count):
        doc = app.documents.item(i)
        try:
            des = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        except Exception:
            continue
        if des and (des.attributes.itemByName(MARK_GROUP, MARK_NAME) or des.attributes.itemByName(LEGACY_GROUP, MARK_NAME)):
            return doc, des
    return None, None


def _retag(design):
    """Old-group attributes of a document drawn before the rename -> the new group."""
    for att in list(design.attributes.itemsByGroup(LEGACY_GROUP)):
        if not design.attributes.itemByName(MARK_GROUP, att.name):
            design.attributes.add(MARK_GROUP, att.name, att.value)
        att.deleteMe()


def _clear(design):
    """Delete the previous mechanism (only called on the tagged document)."""
    root = design.rootComponent
    for ml in list(root.motionLinks):        # gears: first the motion link that ties the joints
        ml.deleteMe()
    for j in list(root.asBuiltJoints):
        j.deleteMe()
    for o in list(root.occurrences):
        o.deleteMe()
    for sk in list(root.sketches):           # root-level drawings (F2: coupler-point path) and their planes
        sk.deleteMe()
    for pl in list(root.constructionPlanes):
        pl.deleteMe()


# Appearances (same colours as the panel): component name -> Fusion Appearance Library appearance.
# Components named "Spring k ..." are the spring frames; bodies named "Key ..." get brass separately.
APPEARANCES = {"Ground": "Plastic - Matte (Gray)", "Base": "Plastic - Matte (Gray)", "Bearings": "Plastic - Matte (Gray)",
               "Housing": "Plastic - Matte (Gray)",
               "Crank": "Plastic - Matte (Red)", "Pinion": "Plastic - Matte (Red)", "Cam": "Plastic - Matte (Red)",
               "Coupler": "Plastic - Matte (Blue)", "Driver": "Plastic - Matte (Red)", "Wheel": "Plastic - Matte (Green)",
               "Handle": "Plastic - Matte (Red)", "Arm": "Plastic - Matte (Green)",
               "Ratchet": "Plastic - Matte (Green)", "Pawl": "Plastic - Matte (Red)",
               "Screw": "Steel - Satin", "Nut": "Brass - Matte", "Rack": "Plastic - Matte (Green)",
               "Driven": "Plastic - Matte (Green)", "Sun": "Plastic - Matte (Red)", "Planet": "Plastic - Matte (Green)",
               "Carrier": "Plastic - Matte (Blue)", "Ring": "Plastic - Matte (Gray)", "Worm": "Steel - Satin", "Bolt": "Steel - Satin", "Hex": "Steel - Satin",
               "Plate": "Plastic - Matte (Gray)",
               "Wall": "Plastic - Matte (Gray)", "Bar": "Steel - Satin", "Weld": "Coating - Black Oxide", "Disc": "Steel - Satin", "Rod": "Plastic - Matte (Blue)", "Link": "Plastic - Matte (Blue)", "Hub": "Plastic - Matte (Green)", "Arbor": "Plastic - Matte (Gray)", "Flywheel": "Plastic - Matte (Red)",
               "Input": "Plastic - Matte (Red)", "Snap": "Plastic - Matte (Red)", "Hinge": "Plastic - Matte (Blue)", "Catch": "Plastic - Matte (Gray)", "Output": "Plastic - Matte (Green)", "Chain": "Coating - Black Oxide", "Belt": "Rubber - Hard",
               "Yoke": "Plastic - Matte (Green)", "Ram": "Plastic - Matte (Green)", "Lever": "Plastic - Matte (Blue)",
               "Rocker": "Plastic - Matte (Green)", "Gear": "Plastic - Matte (Green)", "Follower": "Plastic - Matte (Green)",
               "Piston": "Plastic - Matte (Green)",
               "Shaft": "Steel - Satin", "Input shaft": "Steel - Satin", "Output shaft": "Steel - Satin",
               "Intermediate shaft": "Steel - Satin",
               "Spring": "Steel - Satin", "Key": "Brass - Matte"}


def _appearance(design, name):
    ap = design.appearances.itemByName(name)
    if ap:
        return ap
    lib = adsk.core.Application.get().materialLibraries.itemByName("Fusion Appearance Library")
    return design.appearances.addByCopy(lib.appearances.itemByName(name), name)


def paint(design):
    """Colours the drawn components (visual only: a problem here must not break the drawing)."""
    try:
        for occ in design.rootComponent.occurrences:
            name = occ.component.name
            key = name if name in APPEARANCES else name.split(" ")[0]    # "Spring 3 (...)", "Pinion 1", "Gear 4"
            if key in APPEARANCES:
                occ.appearance = _appearance(design, APPEARANCES[key])
            for body in occ.component.bRepBodies:
                if body.name.startswith("Key "):
                    body.appearance = _appearance(design, APPEARANCES["Key"])
                elif body.name == "Top plate":
                    body.appearance = _appearance(design, APPEARANCES["Base"])
                elif body.name.startswith(("Shaft", "Pin", "Spindle")) and name in ("Driver", "Driven", "Sun", "Flywheel", "Input", "Output", "Hub A", "Hub B", "Wheel", "Arm", "Crank", "Lever"):
                    body.appearance = _appearance(design, APPEARANCES["Shaft"])
    except Exception:
        pass


def build(mech, reuse=True):
    """Builds the mechanism. reuse=True: if the document drawn by the add-in is open it is updated
    (no new tab); otherwise a new document is opened. Returns the Design."""
    app = adsk.core.Application.get()
    doc, design = _find_marked_design(app) if reuse else (None, None)
    if doc:
        doc.activate()
        _retag(design)
        _clear(design)
    else:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.attributes.add(MARK_GROUP, MARK_NAME, "1")
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    for key, owner in (("cam_table", "cam"), ("cam_slide_sign", "cam"), ("spring_frames", "spring"),
                       ("geneva_table", "geneva"), ("geneva_signs", "geneva")):
        old = design.attributes.itemByName(MARK_GROUP, key)
        if old and mech.get("type") != owner:
            old.deleteMe()                    # no animation data of the previous design may remain
    other = {"spur_gear": "gear",             # C9: gear pair
             "spring": "spring",              # C10: compression spring (+ compression animation frames)
             "cam": "cam",                    # C11a: cam + follower
             "shaft": "shaft",                # C11b: two-bearing shaft
             "reducer": "reducer",            # E1: single-stage reducer (system)
             "slider_crank": "slider_crank",  # F1: slider-crank (piston)
             "reducer2": "reducer2",          # E2: two-stage reducer
             "straight_line": "straight_line",  # F2: Hoeckens straight-line linkage
             "geneva": "geneva",                # F3: Geneva (wheel driven from a table by the add-in)
             "toggle_clamp": "toggle_clamp",    # F5: toggle clamp
             "scotch_yoke": "slotted", "quick_return": "slotted",   # F4: slot mechanisms
             "ratchet": "ratchet",              # F6: ratchet and pawl
             "power_screw": "power_screw",      # G5: power screw
             "rack_pinion": "rack_pinion",      # G4: rack and pinion
             "chain_drive": "flex_drive", "belt_drive": "flex_drive",   # G3: chain / V-belt
             "planetary": "planetary",          # G2: planetary gear set
             "helical_gear": "helical",         # G1a: helical gear pair
             "worm_gear": "worm",               # G1b: worm gear set
             "bevel_gear": "bevel",             # G1c: bevel gear pair
             "bolted_joint": "bolted",          # H1: bolted joint
             "welded_joint": "welded",          # H2: welded bracket
             "press_fit": "press_fit",          # H3: press fit
             "torsion_spring": "torsion_spring",   # H4: torsion spring
             "flywheel": "flywheel", "clutch": "clutch",   # H5: flywheel, disc clutch
             "snap_fit": "snap_fit",            # I1: printed snap-fit
             "flexure": "flexure",              # I2: living hinge
             "press": "press",                  # E4: crank press (slider-crank + flywheel)
             "six_bar": "sixbar",               # F4b: Watt II six-bar
             "coupling": "coupling",            # H5b: flange coupling
             "disc_spring": "disc_spring",      # H4b: disc spring stack
             "extension_spring": "extension_spring",   # H4c: extension spring
             "spline": "spline",                # H3b: splined shaft
             "whitworth": "whitworth",          # F4c: Whitworth quick return
             "dwell": "dwell"}.get(mech.get("type"))   # F4d: dwell six-bar
    if other:
        import importlib
        importlib.import_module(f"fusion_lib.{other}").build(mech, root, design)
        paint(design)
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

    # Layer order from the engine (interference.choose_layer_order); else the default in LINKS
    order = mech.get("layer_order") or [n for n, _ in sorted(LINKS.items(), key=lambda kv: kv[1][2])]
    occs, lengths, z = {}, {}, 0.0
    for name in order:
        s, e, _ = LINKS[name]
        occs[name], lengths[name] = make_link(root, name, pts[s], pts[e], z, secs[name])
        z += secs[name]["thickness"] + GAP
    occs["Ground"].isGrounded = True

    # Which end of each link is at which point: start -> x = 0, end -> x = L
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
        _limit_input_joint(root, occs["Crank"], mech["input_range_deg"])
    paint(design)
    app.activeViewport.fit()
    return design


def _occ_angle(occ):
    t = occ.transform2
    return math.atan2(t.getCell(1, 0), t.getCell(0, 0))


def _limit_input_joint(root, crank_occ, range_deg):
    """Limited-angle mechanism (B9): joint limits on the input joint. range_deg is the input angle range
    relative to the drawn pose (theta grows counter-clockwise). The joint's positive sense is measured
    with a small trial turn and the limits are placed accordingly."""
    joint = [j for j in root.asBuiltJoints if j.name.startswith("Ground-Crank")][0]
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
    joint.name = "Ground-Crank (O2, limited)"
