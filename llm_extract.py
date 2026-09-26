"""
Natural language -> structured JSON (two stages).

The LLM invents NO dimension here. It only puts the facts stated in the user's sentence into a fixed
form (schema). Anything not in the sentence stays null. The code (pipeline.py), not the LLM, decides
what is missing or required. Sentences may be Turkish or English.

Two stages (with more modules, one long prompt made the 9B model miss details):
  1) Router: a short prompt that only picks the kind (four-bar / reducer / gears / spring / cam / shaft).
  2) Module extractor: a short prompt and schema specific to that module.
A new module = a new (prompt, schema) pair; the other prompts are not touched.

OpenAI-compatible API; switching to LM Studio only changes BASE_URL.
"""

import json
import sys
import time
import urllib.request

BASE_URL = "http://localhost:11434/v1"   # LM Studio: http://localhost:1234/v1
MODEL = "qwen3.5:9b"

COMMON = ("You extract design requirements from the user's text (usually Turkish) into JSON. "
          "Only use values the user explicitly states. Never invent or estimate numbers; use null when a "
          "value is not given. Convert units to mm, degrees and newtons (1 kg = 9.81 N). "
          "The degree sign '°' means the same as 'derece'. ")

MATERIAL = {"type": ["string", "null"], "enum": ["PLA", "PETG", "aluminum", "steel", None]}


def _schema(props):
    return {"type": "object", "properties": props, "required": list(props), "additionalProperties": False}


# ---------------------------------------------------------------- 1) router
ROUTER_SCHEMA = _schema({"kind": {"type": "string",
                                   "enum": ["fourbar", "slider_crank", "straight_line", "geneva", "toggle_clamp",
                                            "scotch_yoke", "quick_return", "ratchet", "power_screw", "rack_pinion",
                                            "chain_drive", "belt_drive", "planetary", "helical_gear", "worm_gear", "bevel_gear", "bolted_joint", "welded_joint", "press_fit", "torsion_spring", "flywheel", "clutch", "snap_fit", "flexure", "press", "path_synthesis", "coupling", "disc_spring", "extension_spring", "spline", "reducer", "spur_gear",
                                            "spring", "cam", "shaft", "unknown"]}})
ROUTER_PROMPT = (
    "Classify what the user wants to design. Answer only the kind. "
    "'shaft': sizing a rotating shaft / axle (mil, şaft, aks, 'mil çapı', torque and bending on a shaft, yorulma). "
    "'cam': a cam and follower (kam, eksantrik, itici, 'X mm kaldırsın', yükselme/bekleme/dönüş açıları). "
    "'spring': a compression spring (yay, basma yayı, konik yay). "
    "'slider_crank': a piston or slider moved back and forth by a crank (piston, pistonlu, krank-biyel, strok, "
    "stroke, 'ileri geri giden', reciprocating slider, 'X mm strok'). "
    "'flexure': a living hinge / flexure / compliant hinge that bends instead of turning on a pin (canlı "
    "menteşe, esnek menteşe, film menteşe, 'kapak katlanarak açılsın', living hinge, flexure). "
    "'snap_fit': a snap-fit / clip / latch that holds printed parts together by bending (geçmeli tırnak, "
    "klips, 'tık diye oturan', kapak tırnağı, snap-fit, snap hook, latch). "
    "'press': a whole crank / eccentric press machine to design (eksantrik pres, krank pres, 'X ton pres', zımba "
    "presi with stroke and strokes per minute, punch press, crank press); a flywheel alone is 'flywheel'. "
    "'flywheel': a flywheel that evens out the speed (volan, volant, 'devir dalgalanması', pres / zımba volanı, "
    "flywheel, energy storage wheel). "
    "'spline': a splined shaft / spline connection (kamalı mil, spline, 'kaydırmalı göbek', ISO 14). "
    "'extension_spring': a spring that is PULLED / stretched (çekme yayı, uzama yayı, 'çekince uzasın', "
    "extension spring, tension spring). "
    "'disc_spring': disc / Belleville / cup springs (tabak yay, çanak yay, Belleville, disc spring), "
    "usually a big force over a small deflection. "
    "'coupling': a coupling joining two shafts (kaplin, flanşlı kaplin, mil bağlantısı, 'iki mili bağla', "
    "rigid / flange coupling). "
    "'clutch': a friction disc clutch or disc brake (kavrama, debriyaj, disk fren, sürtünmeli kavrama, clutch, "
    "disc brake). "
    "'torsion_spring': a torsion spring that is twisted and pushes back with a moment (burulma yayı, "
    "kurmalı yay, 'mandal yayı', kapak / menteşe yayı, 'X derecede Y Nm', torsion spring). "
    "'press_fit': a hub / gear / pulley pressed or shrunk onto a shaft, or an ISO fit / tolerance question "
    "(sıkı geçme, pres geçme, sıcak geçirme, geçme toleransı, H7/p6, press fit, shrink fit, interference fit). "
    "'welded_joint': a welded connection, e.g. a bar or bracket welded to a wall carrying a load (kaynak, "
    "köşe kaynağı, kaynaklı konsol, 'duvara kaynatılmış', weld, fillet weld, welded bracket). "
    "'bolted_joint': bolts / screws holding parts together against a load pulling them apart (cıvata, civata, "
    "cıvatalı bağlantı, flanş cıvatası, 'kaç cıvata gerekir', bolted joint, bolt size, preload). "
    "'bevel_gear': a bevel gear pair turning the motion through an angle, usually 90 degrees (konik dişli, "
    "açılı dişli, 'dik açıyla aktarsın', bevel gear, mitre / miter gear, right-angle gear pair). "
    "'worm_gear': a worm and worm wheel (sonsuz vida, sonsuz vida-çark, salyangoz dişli, worm gear, worm drive, "
    "'kendini kilitlesin', self-locking gearing with a large ratio). "
    "'helical_gear': a helical gear pair on parallel shafts (helis dişli, helisel dişli, eğik dişli, helical "
    "gear); only a gear pair, not a gearbox. "
    "'planetary': a planetary / epicyclic gear set with sun, planets and a ring gear (planet dişli, gezegen "
    "dişli, planet redüktör, planeter, epicyclic, sun and planet); it is never 'reducer' or 'spur_gear'. "
    "'chain_drive': a roller chain and sprockets transmitting rotation (zincir, zincir dişli, zincirli aktarma, "
    "zincir-dişli, bisiklet zinciri, roller chain, sprocket). "
    "'belt_drive': a V-belt and pulleys transmitting rotation (kayış, V kayış, kayış-kasnak, kasnak, "
    "kayışlı aktarma, V-belt, pulley, sheave). A chain or belt is never 'spur_gear' or 'reducer'. "
    "'rack_pinion': a rack and pinion turning rotation into straight motion (kremayer, kremayer-pinyon, dişli kremayer, "
    "rack and pinion). "
    "'power_screw': a lead / power screw that moves or lifts a load along its axis (trapez vida, güç vidası, "
    "hareket vidası, kriko, vidalı kaldırıcı / pres, lead screw, jack, 'vida ile X kg kaldırsın'). "
    "'ratchet': a ratchet wheel and pawl that turns one way only (cırcır, mandal, tırnaklı çark, 'geri "
    "dönmesin', ratchet, pawl, 'tek yönlü dönsün'). "
    "'scotch_yoke': a Scotch yoke, a crank pin in a slot of a sliding yoke (İskoç boyunduruğu, kulisli kızak, "
    "'yuvalı boyunduruk', slotted yoke, 'saf harmonik ileri geri hareket'). "
    "'quick_return': a shaper-type quick return: a RAM moving a straight STROKE in mm, slow working stroke and "
    "fast return (vargel / shaper, kulisli kol, slotted lever, Whitworth, 'X mm strok, dönüş daha hızlı'). "
    "A crank with an output SWING angle in degrees that should return quickly ('krank, 50°, hızlı dönüşlü') "
    "is 'fourbar', not 'quick_return'. "
    "'toggle_clamp': a toggle / over-centre clamp that holds a workpiece (mafsallı / kelebek / toggle "
    "işkence, hızlı bağlama aparatı, 'kol ölü noktayı geçip kilitlensin', hold-down clamp). "
    "'geneva': intermittent rotation, a wheel that turns a step and stops (Cenevre / Malta haçı mekanizması, "
    "Geneva, indexing table / endeksleme tablası, 'X derece adım adım dönsün', 'dur-kalk dönen tabla'). "
    "'path_synthesis': a linkage whose point must pass through GIVEN POINTS with coordinates (şu noktalardan "
    "geçsin, (0,0), (40,10) ..., 'path through these points', path generation). "
    "'straight_line': a linkage whose POINT follows a straight line with no slide or guide (düz çizgi / doğrusal "
    "yol çizen mekanizma, 'bir nokta düz bir çizgide gitsin', straight-line linkage, Hoeckens, Chebyshev, Watt, "
    "yürüme mekanizması / walking linkage, coupler point path). "
    "'reducer': a complete gearbox with shafts, bearings and housing (redüktör, dişli kutusu, şanzıman, "
    "'motora bağlı', motor power in kW). "
    "'spur_gear': only a gear pair (dişli, dişli çifti, çevrim oranı, 'X kat yavaşlatsın' without a gearbox). "
    "'fourbar': a linkage / mechanism with links, crank, lever or swinging arm (krank, kol, biyel, dört çubuk, "
    "altı çubuk / six-bar / Watt for a large swing, salınım, 'kolu X derece çevirince'). 'unknown': anything else.")

# ---------------------------------------------------------------- 2) module extractors
FOURBAR = (_schema({
    "mechanism_type": {"type": "string", "enum": ["crank_rocker", "rocker_rocker"]},
    "crank_length_mm": {"type": ["number", "null"]},
    "input_swing_deg": {"type": ["number", "null"]},
    "output_swing_deg": {"type": ["number", "null"]},
    "quick_return": {"type": "boolean"},
    "time_ratio": {"type": ["number", "null"]},
    "max_load_N": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a four-bar linkage. "
    "mechanism_type: 'rocker_rocker' when the INPUT link only turns through a limited angle "
    "(two angles: first how far the input is turned — çevirince/döndürünce/dönünce/itince — then the output; "
    "e.g. '4 cm kolu 45° çevirince diğer kol 120° dönsün' -> rocker_rocker, input 45, output 120; "
    "'kapı gibi açılıp kapansın'); otherwise 'crank_rocker' (input crank turns fully; a single "
    "salınım/swing angle, e.g. '30 mm krank, 60° salınım' -> crank_rocker, output 60, input null). "
    "crank_length_mm: length of the INPUT link (krank, giriş kolu, or the 'kol' that is turned): "
    "'30 mm krank' -> 30, '40 mm giriş kolu' -> 40, '4 cm kolu' -> 40. "
    "output_swing_deg: output angle; 'salınım'/'sallansın' is always the output; a lone angle is the output. "
    "input_swing_deg: input rotation, only for rocker_rocker and only if given. "
    "max_load_N: load at the output tip ('uçta 5 kg' -> 49.05). "
    "quick_return: true only if asked ('hızlı dönüşlü', 'zaman oranı'). "
    "time_ratio: Q only if a number is given ('zaman oranı 1.2', 'Q=1.3'). "
    "speed_rpm: crank speed if given ('300 devirde dönen', '600 rpm')."))

GEAR = (_schema({
    "gear_ratio": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a spur gear pair. gear_ratio: speed reduction ratio ('1:3', '3 kat yavaş', 'oran 3', "
    "'devri 3 kat düşüren' -> 3). torque_Nm: input torque in N·m ('5 Nm' -> 5; 1 kg·cm = 0.0981 N·m; "
    "N·mm / 1000). speed_rpm: input speed if given."))

REDUCER = (_schema({
    "gear_ratio": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "power_kW": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
    "shaft_material": {"type": ["string", "null"], "enum": ["1020", "1045", None]},
}), COMMON + (
    "The user wants a gear reducer (gearbox; one or two stages, the engine picks the stages). gear_ratio: "
    "speed reduction ('1:4', '4 kat', 'oran 4', 'devri 12 kat düşüren' -> the number; 'iki kademeli' / "
    "'two-stage' is NOT the ratio). torque_Nm: input torque in N·m if given. power_kW: motor power in kW ('1.5 kW' -> 1.5; "
    "'750 W' -> 0.75; 1 HP = 0.746 kW). speed_rpm: input (motor) speed ('1450 devir', '1450 d/d', '1450 rpm'); a number followed by rpm, devir or "
    "d/d is ALWAYS speed_rpm, even when the ratio is also described with 'devri X kat düşüren'. "
    "material: gear material only if named. shaft_material: '1020' or '1045' only if named."))

SLIDER_CRANK = (_schema({
    "stroke_mm": {"type": ["number", "null"]},
    "crank_length_mm": {"type": ["number", "null"]},
    "rod_length_mm": {"type": ["number", "null"]},
    "offset_mm": {"type": ["number", "null"]},
    "max_load_N": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a slider-crank (piston) mechanism. stroke_mm: piston stroke ('60 mm strok', 'stroke 6 cm'). "
    "crank_length_mm: crank length only if given. rod_length_mm: connecting rod (biyel) length only if given. "
    "offset_mm: offset (kaçıklık, eksantriklik) of the slide axis only if given. max_load_N: force on the piston "
    "('pistona 500 N', '20 kg bassın' -> 196.2). speed_rpm: crank speed if given. material only if named."))

STRAIGHT_LINE = (_schema({
    "stroke_mm": {"type": ["number", "null"]},
    "max_load_N": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a straight-line linkage: a crank-driven four-bar whose coupler point moves along a straight "
    "line. stroke_mm: length of the straight path ('100 mm düz gitsin', 'straight line of 10 cm' -> 100). "
    "max_load_N: force on the moving point along the path ('20 N itsin', '2 kg' -> 19.62) if given. speed_rpm: "
    "crank speed if given. material only if named."))

GENEVA = (_schema({
    "slots": {"type": ["integer", "null"]},
    "step_deg": {"type": ["number", "null"]},
    "center_distance_mm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "load_mass_kg": {"type": ["number", "null"]},
    "load_diameter_mm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a Geneva mechanism (intermittent rotation). slots: number of slots / stations "
    "('6 yuvalı', '6 istasyonlu', '6 slot') if given. step_deg: the wheel's step angle ('60 derece adım', "
    "'90° indexing') if given. center_distance_mm: distance between the driver and wheel axes ('eksenler "
    "arası 100 mm', 'centre distance 10 cm' -> 100). speed_rpm: driver speed if given. load_mass_kg and "
    "load_diameter_mm: a table / plate carried on the wheel shaft ('2 kg, 300 mm çaplı tabla') if given. "
    "material only if named."))

SLOTTED = {
    "stroke_mm": {"type": ["number", "null"]},
    "max_load_N": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}
SCOTCH_YOKE = (_schema(dict(SLOTTED, crank_length_mm={"type": ["number", "null"]})), COMMON + (
    "The user wants a Scotch yoke. stroke_mm: stroke of the yoke ('60 mm strok'). crank_length_mm: crank radius "
    "only if given. max_load_N: force on the sliding yoke ('200 N', '10 kg' -> 98.1) if given. speed_rpm: crank "
    "speed if given. material only if named."))
QUICK_RETURN = (_schema(dict(SLOTTED, time_ratio={"type": ["number", "null"]}, whitworth={"type": "boolean"})), COMMON + (
    "The user wants a quick-return (shaper, slotted lever) mechanism. stroke_mm: ram stroke ('150 mm strok'). "
    "time_ratio: slow stroke time / fast stroke time ('dönüş 1.5 kat hızlı' -> 1.5, 'Q = 1.6') if given. "
    "max_load_N: cutting force on the ram if given. speed_rpm: crank speed if given. material only if named. "
    "whitworth: true only if Whitworth is named (the crank longer than the pivot distance, the lever turning "
    "full circles)."))

FLEX_DRIVE_FIELDS = {
    "power_kW": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "gear_ratio": {"type": ["number", "null"]},
    "shock": {"type": ["string", "null"], "enum": ["smooth", "moderate", "heavy", None]},
    "center_distance_mm": {"type": ["number", "null"]},
}
FLEX_DRIVE_TEXT = (
    "power_kW: transmitted power in kW ('2 kW' -> 2, '750 W' -> 0.75, 1 HP = 0.746 kW). torque_Nm: input torque "
    "in N·m only if given. speed_rpm: INPUT (driver / motor) speed ('1450 devir', '1450 rpm', '1450 d/d'). "
    "gear_ratio: input speed / output speed ('1:3', 'oran 3', '3 kat yavaşlatsın' -> 3; 'devri 2 katına "
    "çıkarsın' / 'speed up 2x' -> 0.5; '1400 devirden 560 devire' -> 2.5); null if not given. shock: 'smooth' "
    "(düzgün, darbesiz, fan, uniform), 'moderate' (orta darbeli, konveyör), 'heavy' (ağır darbeli, kırıcı, "
    "crusher) only if described, else null. center_distance_mm: distance between the shafts (eksenler arası, "
    "mil mesafesi, 'aradaki mesafe 400 mm') only if given.")
FLEXURE = (_schema({
    "spring_angle_deg": {"type": ["number", "null"]},
    "snap_width_mm": {"type": ["number", "null"]},
    "repeated": {"type": "boolean"},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a printed living hinge. spring_angle_deg: how far it bends / opens ('90 derece açılsın' -> 90, "
    "'tamamen katlansın' / 'fold flat' -> 180). snap_width_mm: hinge width if given. repeated: true if it opens "
    "and closes many times (sık açılıp kapanan, repeated). material only if named."))

SNAP = (_schema({
    "undercut_mm": {"type": ["number", "null"]},
    "hold_force_N": {"type": ["number", "null"]},
    "snap_width_mm": {"type": ["number", "null"]},
    "releasable": {"type": "boolean"},
    "repeated": {"type": "boolean"},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a cantilever snap-fit. undercut_mm: how far the hook sticks out / must bend ('1 mm tırnak', "
    "'1.5 mm undercut'). hold_force_N: force it must hold if given (N; kg x 9.81). snap_width_mm: tab width if given. "
    "releasable: true if it must come apart again (sökülebilir, açılabilir, releasable). repeated: true if it is "
    "snapped many times (sık açılıp kapanan, repeated). material only if named."))

SPLINE = (_schema({
    "torque_Nm": {"type": ["number", "null"]},
    "spline_use": {"type": ["string", "null"], "enum": ["fixed", "sliding", "sliding_loaded", None]},
}), COMMON + (
    "The user wants a splined shaft. torque_Nm: the torque in N·m. spline_use: 'fixed' (sabit göbek), 'sliding' "
    "(the hub slides along the shaft when unloaded, kayar göbek), 'sliding_loaded' (it slides while carrying "
    "torque, 'yük altında kayar') only if said."))

EXTENSION = (_schema({
    "spring_force_N": {"type": ["number", "null"]},
    "spring_deflection_mm": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants an extension spring. spring_force_N: the pulling force ('5 kg' -> 49.05, '50 N' -> 50). "
    "spring_deflection_mm: how far it stretches to reach that force ('20 mm uzasın', 'extends 20 mm')."))

DISC = (_schema({
    "spring_force_N": {"type": ["number", "null"]},
    "spring_deflection_mm": {"type": ["number", "null"]},
    "arbor_mm": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a disc (Belleville) spring stack. spring_force_N: the force in NEWTONS (N, kN are forces: "
    "'5 kN' -> 5000; only a mass is converted: '500 kg' -> 4905). spring_deflection_mm: the deflection at that "
    "force. arbor_mm: the guide rod / bolt it sits on, if given ('M12 cıvata üstünde' -> 12)."))

COUPLING = (_schema({
    "torque_Nm": {"type": ["number", "null"]},
    "shaft_diameter_mm": {"type": ["number", "null"]},
    "bolts": {"type": ["integer", "null"]},
}), COMMON + (
    "The user wants a flange coupling. torque_Nm: the torque in N·m. shaft_diameter_mm: shaft diameter if given "
    "(any Turkish case of 'mil': '30 mm mil', '30 mm milleri'). bolts: number of bolts if given."))

PATH = (_schema({
    "path_points": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}},
}), COMMON + (
    "The user wants a linkage whose point passes through given points. path_points: the points in the order "
    "given, each as [x, y] in mm ('(0,0), (40,10), (80,0)' -> [[0,0],[40,10],[80,0]]; cm x 10). Copy the numbers "
    "exactly."))

CRANK_PRESS = (_schema({
    "force_N": {"type": ["number", "null"]},
    "stroke_mm": {"type": ["number", "null"]},
    "working_stroke_mm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "speed_fluctuation": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a crank press. force_N: the press force in NEWTONS ('20 kN' -> 20000; a press rated in ton is a "
    "mass: '10 ton' -> 98100). stroke_mm: ram stroke. working_stroke_mm: the part of the stroke where the force acts "
    "if given ('son 6 mm'de kesme', 'the last 5 mm'). speed_rpm: strokes per minute ('dakikada 40 vuruş', '40 spm'). "
    "speed_fluctuation only if given."))

FLYWHEEL = (_schema({
    "energy_J": {"type": ["number", "null"]},
    "work_J": {"type": ["number", "null"]},
    "work_fraction": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "speed_fluctuation": {"type": ["number", "null"]},
    "application": {"type": ["string", "null"], "enum": ["press", "crusher", "machine_tool", "pump", "engine", "generator", None]},
    "outer_diameter_max_mm": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a flywheel. energy_J: the energy fluctuation per cycle in J if given ('500 J dalgalanma', "
    "'1 kJ' -> 1000). work_J: the work of ONE stroke of a press / punch / cutter if given instead ('zımba her "
    "vuruşta 1500 J iş yapar'). work_fraction: the share of the cycle that work takes ('devrin %10'unda' -> 0.1, "
    "'one sixth of a turn' -> 0.1667). speed_rpm: mean speed. speed_fluctuation: C_s if given ('%5 hız "
    "dalgalanması' -> 0.05, '±2 %' -> 0.04). application: 'press' (pres, zımba), 'crusher' (kırıcı), "
    "'machine_tool', 'pump', 'engine' (motor), 'generator' only if named. outer_diameter_max_mm: max diameter if "
    "given."))

CLUTCH = (_schema({
    "torque_Nm": {"type": ["number", "null"]},
    "power_kW": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "lining": {"type": ["string", "null"], "enum": ["molded", "woven", "sintered", None]},
    "outer_diameter_max_mm": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a disc clutch or brake. torque_Nm: torque in N·m if given. power_kW and speed_rpm if given "
    "instead. lining: 'molded', 'woven' (dokuma), 'sintered' (sinter, metal) only if named. "
    "outer_diameter_max_mm: max outside diameter if given."))

TORSION_SPRING = (_schema({
    "torque_Nm": {"type": ["number", "null"]},
    "spring_angle_deg": {"type": ["number", "null"]},
    "arbor_mm": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a torsion spring. torque_Nm: the moment in N·m at the working angle ('0.5 Nm' -> 0.5, "
    "'500 N·mm' -> 0.5, '5 kg·cm' -> 0.4905). spring_angle_deg: how far it is twisted at that moment ('90 derece "
    "kurulunca', 'at 120°'). arbor_mm: the rod / pin / shaft it sits on, if given ('8 mm milin üstünde')."))

PRESS = (_schema({
    "torque_Nm": {"type": ["number", "null"]},
    "shaft_diameter_mm": {"type": ["number", "null"]},
    "hub_diameter_mm": {"type": ["number", "null"]},
    "hub_length_mm": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a press fit of a hub on a shaft. torque_Nm: the torque it must transmit in N·m. "
    "shaft_diameter_mm: shaft / bore diameter if given, in any Turkish case of 'mil' ('40 mm mil', '40 mm mile', "
    "'40 mm milin', '40'lık mil', 'Ø40' -> 40). hub_diameter_mm: hub outer diameter if "
    "given. hub_length_mm: hub length / fit length if given ('50 mm boyunda göbek')."))

WELDED = (_schema({
    "force_N": {"type": ["number", "null"]},
    "arm_mm": {"type": ["number", "null"]},
    "bar_width_mm": {"type": ["number", "null"]},
    "bar_height_mm": {"type": ["number", "null"]},
    "weld_pattern": {"type": ["string", "null"], "enum": ["all_around", "top_bottom", "sides", None]},
}), COMMON + (
    "The user wants a welded bracket. force_N: the load in NEWTONS (N, kN are forces: '5 kN' -> 5000; only a mass "
    "is converted: '200 kg' -> 1962). arm_mm: how far from the weld / wall the load acts ('ucunda', "
    "'duvardan 150 mm', 'at 150 mm'). bar_height_mm and bar_width_mm: the bar's section if given ('40x20 lama' -> "
    "height 40, width 20; '30 mm kare' -> both 30). weld_pattern: 'all_around' (etrafı), 'top_bottom' (alttan ve "
    "üstten), 'sides' (yanlardan) only if said."))

BOLTED = (_schema({
    "force_N": {"type": ["number", "null"]},
    "bolts": {"type": ["integer", "null"]},
    "grip_mm": {"type": ["number", "null"]},
    "member_material": {"type": ["string", "null"], "enum": ["steel", "aluminum", "cast_iron", None]},
    "bolt_class": {"type": ["string", "null"], "enum": ["8.8", "10.9", "12.9", None]},
    "fluctuating": {"type": "boolean"},
}), COMMON + (
    "The user wants a bolted joint. force_N: the total load pulling the joint apart in NEWTONS. N and kN are "
    "already forces: '50 kN' -> 50000, '800 N' -> 800 (never multiply kN by 9.81). Only a MASS is converted: "
    "'300 kg' -> 2943, '2 ton' -> 19620. bolts: number of bolts if given. grip_mm: total "
    "clamped thickness if given ('iki 10 mm sac' / 'two 10 mm plates' -> 20). member_material: 'steel', "
    "'aluminum' (alüminyum), 'cast_iron' (döküm, pik) only if named. bolt_class: '8.8', '10.9', '12.9' only if "
    "named. fluctuating: true if the load varies / repeats / vibrates (değişken, titreşimli, tekrarlı, "
    "fluctuating, cyclic, fatigue)."))

BEVEL = (_schema({
    "gear_ratio": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a bevel gear pair. gear_ratio: speed reduction ('1:2', '2 kat', 'oran 2' -> 2; a mitre / "
    "'eşit' pair -> 1). torque_Nm: input torque in N·m. speed_rpm: input speed if given. material only if named."))

WORM = (_schema({
    "gear_ratio": {"type": ["number", "null"]},
    "output_torque_Nm": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "power_kW": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "self_locking": {"type": "boolean"},
}), COMMON + (
    "The user wants a worm gear set. gear_ratio: reduction ('1:30', '30 kat', 'oran 30' -> 30). output_torque_Nm: "
    "torque at the wheel / output if given ('çıkışta 50 Nm', 'output 50 N·m'). torque_Nm: torque at the worm / motor "
    "(input) if given. power_kW: motor power ('0.37 kW', '370 W' -> 0.37). speed_rpm: worm / motor speed. "
    "self_locking: true only if asked ('kendini kilitlesin', 'geri dönmesin', 'yük geri çevirmesin', 'self-locking')."))

HELICAL = (_schema({
    "gear_ratio": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "helix_deg": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a helical gear pair. gear_ratio: speed reduction ('1:3', '3 kat', 'oran 3' -> 3). torque_Nm: "
    "input torque in N·m. helix_deg: helix angle only if given ('20 derece helis', 'helix 25°'). speed_rpm: input "
    "speed if given. material only if named."))

PLANETARY = (_schema({
    "gear_ratio": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "power_kW": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "planets": {"type": ["integer", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a planetary gear set. gear_ratio: speed reduction ('1:5', '5 kat', 'oran 5' -> 5). "
    "torque_Nm: input torque in N·m if given. power_kW: power in kW if given ('750 W' -> 0.75). speed_rpm: input "
    "speed if given. planets: number of planet gears only if given ('3 planetli', 'four planets' -> 4). "
    "material only if named."))

CHAIN_DRIVE = (_schema(FLEX_DRIVE_FIELDS), COMMON + "The user wants a roller chain drive. " + FLEX_DRIVE_TEXT)
BELT_DRIVE = (_schema(FLEX_DRIVE_FIELDS), COMMON + "The user wants a V-belt drive. " + FLEX_DRIVE_TEXT)

RACK_PINION = (_schema({
    "force_N": {"type": ["number", "null"]},
    "travel_mm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a rack and pinion. force_N: the force on the rack in newtons; only a MASS in kg is multiplied "
    "by 9.81 ('20 kg' -> 196.2, '2 kN' -> 2000). travel_mm: rack travel / stroke if given. speed_rpm: pinion speed "
    "if given. material only if named."))

POWER_SCREW = (_schema({
    "force_N": {"type": ["number", "null"]},
    "travel_mm": {"type": ["number", "null"]},
    "screw_ends": {"type": ["string", "null"], "enum": ["supported", "jack", None]},
    "nut_material": {"type": ["string", "null"], "enum": ["bronze", "steel", "PLA", None]},
    "speed_rpm": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a power screw. force_N: the axial load in newtons. Only a MASS in kg is multiplied by 9.81 "
    "('500 kg kaldırsın' -> 4905); N and kN are already forces ('2 kN' -> 2000, '800 N' -> 800). "
    "travel_mm: stroke / lifting height if given. screw_ends: 'jack' for a car jack / screw jack with a free top "
    "(kriko), 'supported' if the screw is held at both ends, else null. nut_material: 'bronze' (bronz), 'steel', "
    "'PLA' only if named. speed_rpm: screw speed if given."))

RATCHET = (_schema({
    "torque_Nm": {"type": ["number", "null"]},
    "teeth": {"type": ["integer", "null"]},
    "outer_diameter_max_mm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a ratchet wheel and pawl. torque_Nm: the torque it must hold ('20 Nm tutsun'; a load on a "
    "drum: only if the torque itself is given). teeth: number of teeth ('24 dişli') if given. "
    "outer_diameter_max_mm: wheel diameter if given ('çapı 60 mm'). material only if named."))

TOGGLE_CLAMP = (_schema({
    "opening_deg": {"type": ["number", "null"]},
    "handle_travel_deg": {"type": ["number", "null"]},
    "arm_length_mm": {"type": ["number", "null"]},
    "clamp_force_N": {"type": ["number", "null"]},
    "handle_length_mm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a toggle clamp. opening_deg: how far the clamp arm opens ('90 derece açılsın'). "
    "handle_travel_deg: how far the handle turns if given. arm_length_mm: distance from the arm pivot to the "
    "pressing spindle / pad ('kol 60 mm', 'arm 6 cm'). clamp_force_N: holding / clamping force ('1000 N tutsun', "
    "'50 kg' -> 490.5) if given. handle_length_mm: handle length if given. material only if named."))

SPRING = (_schema({
    "spring_force_N": {"type": ["number", "null"]},
    "spring_deflection_mm": {"type": ["number", "null"]},
    "spring_shape": {"type": "string", "enum": ["helical", "conical"]},
    "spring_taper": {"type": ["number", "null"]},
    "outer_diameter_max_mm": {"type": ["number", "null"]},
    "spring_force_min_N": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a compression spring. spring_force_N: max load ('5 kg' -> 49.05, '100 N' -> 100). "
    "spring_deflection_mm: how much it compresses under that load ('20 mm sıkışan', '15 mm çöken'). "
    "spring_shape: 'conical' if konik/conical, else 'helical'. spring_taper: small/large diameter ratio only "
    "if given. outer_diameter_max_mm: max outer diameter if given ('dış çapı en fazla 12 mm', 'Ø20 deliğe girsin'). "
    "spring_force_min_N: null unless the force cycles between two values; then spring_force_N is the LARGER and "
    "this the smaller one ('50 ile 200 N arasında' -> spring_force_N 200, spring_force_min_N 50)."))

CAM = (_schema({
    "cam_rise_mm": {"type": ["number", "null"]},
    "rise_deg": {"type": ["number", "null"]},
    "dwell_top_deg": {"type": ["number", "null"]},
    "return_deg": {"type": ["number", "null"]},
    "motion_law": {"type": ["string", "null"], "enum": ["cycloidal", "harmonic", "poly345", None]},
    "roller_radius_mm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "follower_mass_kg": {"type": ["number", "null"]},
    "follower_load_N": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a disk cam with a translating roller follower. cam_rise_mm: follower lift/stroke "
    "('20 mm kaldırsın', 'strok 20'). rise_deg: cam angle for the rise ('120 derecede yükselsin'). "
    "dwell_top_deg: dwell at the top ('60 derece beklesin', 'üstte 60° bekleme'). return_deg: cam angle for the "
    "return/fall ('120 derecede insin'). motion_law: 'cycloidal' (sikloidal), 'harmonic' (basit harmonik), "
    "'poly345' (3-4-5 polinom) only if named. roller_radius_mm: roller radius if given. speed_rpm: cam speed "
    "('600 devirde', '600 rpm') if given. follower_mass_kg: mass of the follower in kg ('itici 200 g' -> 0.2) if "
    "given. follower_load_N: a load pressing the follower onto the cam if given."))

SHAFT = (_schema({
    "torque_Nm": {"type": ["number", "null"]},
    "bending_moment_Nm": {"type": ["number", "null"]},
    "radial_force_N": {"type": ["number", "null"]},
    "span_mm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "feature": {"type": ["string", "null"],
                "enum": ["keyseat", "sharp_shoulder", "round_shoulder", "ring_groove", "plain", None]},
    "shaft_material": {"type": ["string", "null"], "enum": ["1020", "1045", None]},
    "safety_n": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants to size a rotating shaft. torque_Nm: transmitted torque in N·m. bending_moment_Nm: "
    "bending moment in N·m if given. radial_force_N: a transverse force on the shaft ('2 kN' -> 2000) and "
    "span_mm: bearing distance ('yataklar arası 200 mm') if given. speed_rpm: shaft speed ('1450 devir', "
    "'1450 d/d', '1450 rpm') if given. feature: 'keyseat' (kama, kama yuvası), "
    "'sharp_shoulder' (keskin omuz), 'round_shoulder' (radyuslu/yuvarlatılmış omuz), 'ring_groove' (segman), "
    "'plain' (düz) only if named. shaft_material: '1020' or '1045' only if named. safety_n: required safety "
    "factor if given ('emniyet katsayısı 2.5')."))

MODULE_EXTRACTORS = {"fourbar": FOURBAR, "slider_crank": SLIDER_CRANK, "straight_line": STRAIGHT_LINE, "geneva": GENEVA, "toggle_clamp": TOGGLE_CLAMP, "scotch_yoke": SCOTCH_YOKE, "ratchet": RATCHET, "power_screw": POWER_SCREW, "rack_pinion": RACK_PINION,
                     "chain_drive": CHAIN_DRIVE, "planetary": PLANETARY, "helical_gear": HELICAL, "worm_gear": WORM, "bevel_gear": BEVEL, "bolted_joint": BOLTED, "welded_joint": WELDED, "press_fit": PRESS, "torsion_spring": TORSION_SPRING, "flywheel": FLYWHEEL, "press": CRANK_PRESS, "path_synthesis": PATH, "coupling": COUPLING, "disc_spring": DISC, "extension_spring": EXTENSION, "spline": SPLINE, "snap_fit": SNAP, "flexure": FLEXURE, "clutch": CLUTCH, "belt_drive": BELT_DRIVE,
                     "quick_return": QUICK_RETURN, "reducer": REDUCER, "spur_gear": GEAR, "spring": SPRING, "cam": CAM, "shaft": SHAFT}

# All fields (the pipeline expects one dict; fields of other modules are None)
ALL_FIELDS = {"mechanism_type": "unknown", "quick_return": False}
for _schema_, _ in MODULE_EXTRACTORS.values():
    for _k in _schema_["properties"]:
        ALL_FIELDS.setdefault(_k, None)
KIND_TO_TYPE = {"slider_crank": "slider_crank", "straight_line": "straight_line", "geneva": "geneva",
                "toggle_clamp": "toggle_clamp", "scotch_yoke": "scotch_yoke", "quick_return": "quick_return", "ratchet": "ratchet", "power_screw": "power_screw", "rack_pinion": "rack_pinion", "chain_drive": "chain_drive", "belt_drive": "belt_drive", "planetary": "planetary", "helical_gear": "helical_gear", "worm_gear": "worm_gear", "bevel_gear": "bevel_gear", "bolted_joint": "bolted_joint", "welded_joint": "welded_joint", "press_fit": "press_fit", "torsion_spring": "torsion_spring", "flywheel": "flywheel", "clutch": "clutch", "snap_fit": "snap_fit", "flexure": "flexure", "press": "press", "path_synthesis": "path_synthesis", "coupling": "coupling", "disc_spring": "disc_spring", "extension_spring": "extension_spring", "spline": "spline", "reducer": "reducer", "spur_gear": "spur_gear", "spring": "spring", "cam": "cam", "shaft": "shaft"}   # fourbar: the extractor decides


# ---------------------------------------------------------------- 3) edits of the current design (K1)
KIND_OF_TYPE = {"crank_rocker": "fourbar", "rocker_rocker": "fourbar", "slider_crank": "slider_crank",
                "straight_line": "straight_line", "geneva": "geneva", "toggle_clamp": "toggle_clamp",
                "scotch_yoke": "scotch_yoke", "quick_return": "quick_return", "ratchet": "ratchet", "power_screw": "power_screw", "rack_pinion": "rack_pinion", "chain_drive": "chain_drive", "belt_drive": "belt_drive", "planetary": "planetary", "helical_gear": "helical_gear", "worm_gear": "worm_gear", "bevel_gear": "bevel_gear", "bolted_joint": "bolted_joint", "welded_joint": "welded_joint", "press_fit": "press_fit", "torsion_spring": "torsion_spring", "flywheel": "flywheel", "clutch": "clutch", "snap_fit": "snap_fit", "flexure": "flexure", "press": "press", "path_synthesis": "path_synthesis", "coupling": "coupling", "disc_spring": "disc_spring", "extension_spring": "extension_spring", "spline": "spline", "reducer": "reducer", "spur_gear": "spur_gear",
                "spring": "spring", "cam": "cam", "shaft": "shaft"}
DESCRIPTION = {"fourbar": "four-bar linkage", "slider_crank": "slider-crank (piston) mechanism",
               "straight_line": "straight-line linkage (Hoeckens)", "geneva": "Geneva mechanism (intermittent rotation)",
               "toggle_clamp": "toggle clamp", "scotch_yoke": "Scotch yoke",
               "quick_return": "quick-return (shaper) mechanism", "ratchet": "ratchet and pawl", "power_screw": "power screw", "rack_pinion": "rack and pinion", "chain_drive": "roller chain drive", "belt_drive": "V-belt drive", "planetary": "planetary gear set", "helical_gear": "helical gear pair", "worm_gear": "worm gear set", "bevel_gear": "bevel gear pair", "bolted_joint": "bolted joint", "welded_joint": "welded bracket", "press_fit": "press fit", "torsion_spring": "torsion spring", "flywheel": "flywheel", "clutch": "disc clutch", "snap_fit": "snap-fit", "flexure": "living hinge", "press": "crank press", "path_synthesis": "path-generating four-bar", "coupling": "flange coupling", "disc_spring": "disc spring stack", "extension_spring": "extension spring", "spline": "splined shaft", "reducer": "single-stage gear reducer (gearbox)",
               "spur_gear": "spur gear pair", "spring": "compression spring", "cam": "disk cam with roller follower",
               "shaft": "rotating shaft"}
NOT_EDITABLE = {"mechanism_type", "quick_return", "spring_shape"}


def editable_fields(kind):
    """Fields of a module that an edit may change: (numeric fields, {text field: allowed values})."""
    props = MODULE_EXTRACTORS[kind][0]["properties"]
    numeric, choice = [], {}
    for k, v in props.items():
        if k in NOT_EDITABLE:
            continue
        if "enum" in v:
            choice[k] = [e for e in v["enum"] if e is not None]
        else:
            numeric.append(k)
    return numeric, choice


def _edit_schema(fields):
    return _schema({
        "is_edit": {"type": "boolean"},
        "changes": {"type": "array", "items": _schema({
            "field": {"type": "string", "enum": fields},
            "op": {"type": "string", "enum": ["set", "add", "multiply"]},
            "number": {"type": ["number", "null"]},
            "choice": {"type": ["string", "null"]},
        })},
    })


def extract_edit(user_text: str, prev_spec: dict) -> dict:
    """Is the message an edit of the current design, and which changes? Returns {"is_edit", "changes"}.
    The LLM only reports what the text says; pipeline.apply_edit checks the numbers and applies them."""
    kind = KIND_OF_TYPE.get(prev_spec.get("mechanism_type"))
    if kind is None:
        return {"is_edit": False, "changes": []}
    numeric, choice = editable_fields(kind)
    current = {k: prev_spec.get(k) for k in numeric + list(choice)}
    prompt = COMMON + (
        f"The current design is a {DESCRIPTION[kind]} with these values: {json.dumps(current)}. "
        "Decide whether the user's new message asks to CHANGE this design ('make the crank 40 mm', 'use aluminium', "
        "'yükü 10 kg yap', 'krankı 5 mm uzat', 'devri iki katına çıkar', 'alüminyum olsun', 'salınımı 70 derece yap') "
        "-> is_edit = true, or describes a NEW / different request (a new mechanism, a gear, a spring... with its own "
        "values) -> is_edit = false and changes = []. "
        "For each change: field from the list; op 'set' for a new absolute value, 'add' for 'X more / less' "
        "(negative number for less / shorter), 'multiply' for 'twice' (2), 'half' (0.5), 'three times' (3). "
        "number: ONLY an amount written in the message, converted to the field's unit (_mm: mm, _deg: degrees, "
        "_N: newtons with 1 kg = 9.81 N, _Nm: N·m, _rpm: rpm, _kW: kW); if the message gives no amount "
        "('biraz daha uzun', 'a bit longer') number = null. "
        f"choice: for these text fields only, one of the allowed values: {json.dumps(choice)}; otherwise null.")
    return _chat(prompt, user_text, _edit_schema(numeric + list(choice)), "edit")


def _chat(system, user_text, schema, name):
    body = {
        "model": MODEL,
        "messages": [{"role": "system", "content": system}, {"role": "user", "content": user_text}],
        "temperature": 0,
        "reasoning_effort": "none",
        "response_format": {"type": "json_schema", "json_schema": {"name": name, "schema": schema, "strict": True}},
    }
    req = urllib.request.Request(BASE_URL + "/chat/completions", data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as resp:
        reply = json.load(resp)
    return json.loads(reply["choices"][0]["message"]["content"])


def route(user_text: str) -> str:
    return _chat(ROUTER_PROMPT, user_text, ROUTER_SCHEMA, "route")["kind"]


def extract_spec(user_text: str) -> dict:
    """Two-stage extraction. Returns a dict with all fields (None where the module has no such field)."""
    kind = route(user_text)
    spec = dict(ALL_FIELDS)
    if kind not in MODULE_EXTRACTORS:
        return spec
    schema, prompt = MODULE_EXTRACTORS[kind]
    spec.update(_chat(prompt, user_text, schema, kind))
    if kind in KIND_TO_TYPE:
        spec["mechanism_type"] = KIND_TO_TYPE[kind]
    h, w = spec.get("bar_height_mm"), spec.get("bar_width_mm")
    if kind == "welded_joint" and h and w and h < w:      # a cantilever bar stands on its deep side
        spec["bar_height_mm"], spec["bar_width_mm"] = w, h
    return spec


if __name__ == "__main__":
    text = " ".join(sys.argv[1:]) or (
        "10 cm'lik bir krankla çıkış kolu 60 derece salınım yapsın, "
        "PETG'den basacağım, uçta yaklaşık 5 kg yük olacak."
    )
    print("Input :", text)
    t0 = time.time()
    spec = extract_spec(text)
    print(f"Time  : {time.time() - t0:.1f} s")
    print("Output:")
    print(json.dumps(spec, indent=2, ensure_ascii=False))
