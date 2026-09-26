"""
Design flow (prints nothing, only returns data):
    sentence -> LLM (JSON) -> synthesis / sizing -> independent simulation -> checks

Used by both the terminal (main.py) and the Fusion panel.
Standard library only: it also runs in Fusion's own Python.
"""

import math
import re

import cam_system
import cams
import coupler
import function_gen
import geneva
import gears
import interference
import kinematics as kin
import shafts
import slotted
import slider_crank
import springs
import toggle
import llm_extract
import power_screw
import rack_pinion
import ratchet
import reducer
import reducer2
import strength
from llm_extract import extract_spec


# C0: each module defines its own sliders; the panel builds them from here.
# (The four-bar has its own sliders in the panel: coloured swing band, input motion, etc.)
MODULES = {
    "rack_pinion": {
        "title": "Rack and pinion",
        "params": [
            {"key": "force_N", "label": "Force on the rack", "unit": "N", "min": 10, "max": 20000, "step": 10, "default": 500},
            {"key": "travel_mm", "label": "Travel", "unit": "mm", "min": 20, "max": 1000, "step": 10, "default": 200},
            {"key": "speed_rpm", "label": "Pinion speed", "unit": "rpm", "min": 0, "max": 1500, "step": 10, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "power_screw": {
        "title": "Power screw (Tr)",
        "params": [
            {"key": "force_N", "label": "Axial load", "unit": "N", "min": 100, "max": 50000, "step": 100, "default": 5000},
            {"key": "travel_mm", "label": "Travel", "unit": "mm", "min": 20, "max": 1000, "step": 10, "default": 200},
            {"key": "screw_ends", "label": "Ends", "default": "supported",
             "options": [["supported", "Both supported (C = 1)"], ["jack", "Jack, free top (C = 0.25)"]]},
            {"key": "nut_material", "label": "Nut", "default": "bronze",
             "options": [["bronze", "Bronze"], ["steel", "Steel"], ["PLA", "PLA (3D print)"]]},
            {"key": "speed_rpm", "label": "Screw speed", "unit": "rpm", "min": 0, "max": 1500, "step": 10, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "ratchet": {
        "title": "Ratchet and pawl",
        "params": [
            {"key": "torque_Nm", "label": "Holding torque", "unit": "N·m", "min": 0.5, "max": 200, "step": 0.5, "default": 5},
            {"key": "teeth", "label": "Teeth", "unit": "", "min": 8, "max": 60, "step": 1, "default": 24},
            {"key": "outer_diameter_max_mm", "label": "Wheel diameter", "unit": "mm", "min": 0, "max": 300, "step": 5,
             "default": 0, "zero_label": "sized"},
        ],
    },
    "scotch_yoke": {
        "title": "Scotch yoke",
        "params": [
            {"key": "stroke_mm", "label": "Stroke", "unit": "mm", "min": 10, "max": 200, "step": 1, "default": 60},
            {"key": "max_load_N", "label": "Slide force", "unit": "N", "min": 0, "max": 2000, "step": 10, "default": 0,
             "zero_label": "none"},
            {"key": "speed_rpm", "label": "Crank speed", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "quick_return": {
        "title": "Quick return (shaper)",
        "params": [
            {"key": "stroke_mm", "label": "Stroke", "unit": "mm", "min": 20, "max": 400, "step": 5, "default": 150},
            {"key": "time_ratio", "label": "Time ratio Q", "unit": "", "min": 1.1, "max": 2.5, "step": 0.05,
             "default": 1.5},
            {"key": "max_load_N", "label": "Cutting force", "unit": "N", "min": 0, "max": 3000, "step": 10, "default": 0,
             "zero_label": "none"},
            {"key": "speed_rpm", "label": "Crank speed", "unit": "rpm", "min": 0, "max": 200, "step": 5, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "toggle_clamp": {
        "title": "Toggle clamp",
        "params": [
            {"key": "opening_deg", "label": "Arm opening", "unit": "°", "min": 20, "max": 150, "step": 5, "default": 90},
            {"key": "handle_travel_deg", "label": "Handle travel", "unit": "°", "min": 30, "max": 170, "step": 5,
             "default": 90},
            {"key": "arm_length_mm", "label": "Arm (to spindle)", "unit": "mm", "min": 20, "max": 300, "step": 5,
             "default": 60},
            {"key": "clamp_force_N", "label": "Holding force", "unit": "N", "min": 0, "max": 5000, "step": 50,
             "default": 0, "zero_label": "none"},
        ],
    },
    "geneva": {
        "title": "Geneva (intermittent rotation)",
        "params": [
            {"key": "slots", "label": "Slots", "unit": "", "min": 3, "max": 12, "step": 1, "default": 6},
            {"key": "center_distance_mm", "label": "Centre distance", "unit": "mm", "min": 30, "max": 300, "step": 5,
             "default": 100},
            {"key": "speed_rpm", "label": "Driver speed", "unit": "rpm", "min": 0, "max": 300, "step": 5, "default": 0,
             "zero_label": "not given"},
            {"key": "load_mass_kg", "label": "Table mass", "unit": "kg", "min": 0, "max": 20, "step": 0.1, "default": 0,
             "zero_label": "none"},
            {"key": "load_diameter_mm", "label": "Table diameter", "unit": "mm", "min": 0, "max": 500, "step": 10,
             "default": 0, "zero_label": "none"},
        ],
    },
    "straight_line": {
        "title": "Straight-line linkage",
        "params": [
            {"key": "stroke_mm", "label": "Straight path", "unit": "mm", "min": 20, "max": 300, "step": 5, "default": 100},
            {"key": "max_load_N", "label": "Force on the point", "unit": "N", "min": 0, "max": 300, "step": 5,
             "default": 0, "zero_label": "none"},
            {"key": "speed_rpm", "label": "Crank speed", "unit": "rpm", "min": 0, "max": 300, "step": 5, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "slider_crank": {
        "title": "Slider-crank (piston)",
        "params": [
            {"key": "stroke_mm", "label": "Stroke", "unit": "mm", "min": 10, "max": 200, "step": 1, "default": 60},
            {"key": "offset_mm", "label": "Offset", "unit": "mm", "min": 0, "max": 40, "step": 1, "default": 0,
             "zero_label": "in-line"},
            {"key": "max_load_N", "label": "Piston force", "unit": "N", "min": 0, "max": 2000, "step": 10, "default": 0,
             "zero_label": "none"},
            {"key": "speed_rpm", "label": "Crank speed", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "reducer": {
        "title": "Reducer (system)",
        "params": [
            {"key": "torque_Nm", "label": "Input torque", "unit": "N·m", "min": 1, "max": 400, "step": 1, "default": 20},
            {"key": "speed_rpm", "label": "Input speed", "unit": "rpm", "min": 100, "max": 3000, "step": 10, "default": 1450},
            {"key": "gear_ratio", "label": "Ratio (> 6: two stages)", "unit": "", "min": 1.2, "max": 30, "step": 0.1,
             "default": 3},
            {"key": "shaft_material", "label": "Shaft steel", "default": "1045",
             "options": [["1045", "AISI 1045 CD"], ["1020", "AISI 1020 CD"]]},
            {"key": "feature", "label": "Gear on shaft", "default": "keyseat",
             "options": [["keyseat", "Key"], ["ring_groove", "Retaining ring"], ["round_shoulder", "Rounded shoulder"]]},
        ],
    },
    "spur_gear": {
        "title": "Spur gears",
        "params": [
            {"key": "gear_ratio", "label": "Ratio", "unit": "", "min": 1, "max": 8, "step": 0.1, "default": 3},
            {"key": "torque_Nm", "label": "Input torque", "unit": "N·m", "min": 0.1, "max": 50, "step": 0.1, "default": 2},
            {"key": "speed_rpm", "label": "Input speed", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "spring": {
        "title": "Compression spring",
        "params": [
            {"key": "spring_force_N", "label": "Max force", "unit": "N", "min": 1, "max": 500, "step": 1, "default": 100},
            {"key": "spring_deflection_mm", "label": "Deflection at that force", "unit": "mm", "min": 1, "max": 100, "step": 1, "default": 20},
            {"key": "spring_taper", "label": "Taper (D₂/D₁)", "unit": "", "min": 0.3, "max": 1.0, "step": 0.05,
             "default": 1.0, "one_label": "cylindrical"},
            {"key": "outer_diameter_max_mm", "label": "Max outer diameter", "unit": "mm", "min": 0, "max": 100, "step": 1,
             "default": 0, "zero_label": "free"},
            {"key": "show_plates", "label": "Fusion view", "default": "no",
             "options": [["no", "Spring only"], ["yes", "With plates (compression)"]]},
        ],
    },
    "cam": {
        "title": "Cam (roller follower)",
        "params": [
            {"key": "cam_rise_mm", "label": "Lift (stroke)", "unit": "mm", "min": 2, "max": 60, "step": 1, "default": 20},
            {"key": "rise_deg", "label": "Rise angle", "unit": "°", "min": 20, "max": 200, "step": 5, "default": 120},
            {"key": "dwell_top_deg", "label": "Dwell at top", "unit": "°", "min": 0, "max": 180, "step": 5, "default": 60},
            {"key": "return_deg", "label": "Return angle", "unit": "°", "min": 20, "max": 200, "step": 5, "default": 120},
            {"key": "motion_law", "label": "Motion law", "default": "cycloidal",
             "options": [["cycloidal", "Cycloidal"], ["harmonic", "Simple harmonic"], ["poly345", "3-4-5 polynomial"]]},
            {"key": "speed_rpm", "label": "Cam speed", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "not given"},
            {"key": "follower_mass_kg", "label": "Follower mass", "unit": "kg", "min": 0, "max": 3, "step": 0.05,
             "default": 0, "zero_label": "none"},
        ],
    },
    "shaft": {
        "title": "Shaft (fatigue)",
        "params": [
            {"key": "torque_Nm", "label": "Torque", "unit": "N·m", "min": 1, "max": 500, "step": 1, "default": 100},
            {"key": "bending_moment_Nm", "label": "Bending moment", "unit": "N·m", "min": 0, "max": 500, "step": 1, "default": 60},
            {"key": "feature", "label": "Stress raiser", "default": "keyseat",
             "options": [["keyseat", "Keyseat"], ["sharp_shoulder", "Sharp shoulder"], ["round_shoulder", "Rounded shoulder"],
                         ["ring_groove", "Retaining-ring groove"], ["plain", "Plain shaft"]]},
            {"key": "shaft_material", "label": "Steel", "default": "1045",
             "options": [["1045", "AISI 1045 CD"], ["1020", "AISI 1020 CD"]]},
            {"key": "safety_n", "label": "Fatigue safety", "unit": "", "min": 1.2, "max": 4, "step": 0.1, "default": 2},
            {"key": "speed_rpm", "label": "Speed", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 300,
             "zero_label": "not given"},
            {"key": "span_mm", "label": "Bearing span", "unit": "mm", "min": 0, "max": 400, "step": 10, "default": 0,
             "zero_label": "auto"},
        ],
    },
}


def max_feasible_swing(a, psi_start):
    """If the wanted swing is not feasible: the largest swing meeting the design target (45-135, compact)."""
    for psi in range(int(psi_start) - 1, 0, -1):
        m = kin.synthesize_crank_rocker(a, psi)
        if m["worst_dev"] <= 45.0:
            return psi
    return None


def max_feasible_q(a, psi, q_max=1.5, step=0.05):
    """Largest time ratio meeting the transmission-angle design target (45-135). None if there is none."""
    q = q_max
    while q > 1.0 + 1e-9:
        if kin.synthesize_quick_return(a, psi, q)["worst_dev"] <= 45.0:
            return round(q, 2)
        q -= step
    return None


def swing_band(Q=1.0, step=5, n=720):
    """
    Coloured bands of the swing slider: [(psi, level), ...]
      2: transmission angle within 45-135 (a compact solution exists)
      1: passes the 40-140 limit (but the mechanism may be large)
      0: fails
    Scale-free: the angles do not depend on the crank length (computed with a = 10).
    """
    band = []
    for psi in range(step, 180, step):
        m = (kin.synthesize_crank_rocker(10.0, psi) if Q <= 1.0
             else kin.synthesize_quick_return(10.0, psi, Q, n=n))
        if m["worst_dev"] <= 45.0:
            level = 2
        elif kin.MU_MIN_DEG <= m["mu_min"] and m["mu_max"] <= kin.MU_MAX_DEG:
            level = 1
        else:
            level = 0
        band.append((psi, level))
    return band


_INPUT_TURN = re.compile(
    r"(\d+(?:[.,]\d+)?)\s*(?:°|derece|derecelik)\s*(?:\S+\s+)?"
    r"(?:çevir|döndür|dönün|döndüğ|itin|it(?:ti)?ğ|hareket ettir|oynat)", re.IGNORECASE)


# Speed backstop: "1500 rpm", "1450 devirli", "960 d/d" -> speed_rpm when the LLM leaves it empty
_SPEED = re.compile(r"(\d+(?:[.,]\d+)?)\s*(?:rpm|d/d|dev/dak|devir(?:de|li)?)(?![a-zçğıöşü])", re.IGNORECASE)
_SPEED_KINDS = ("reducer", "spur_gear", "shaft", "slider_crank", "cam", "straight_line", "geneva", "scotch_yoke",
                "quick_return", "power_screw", "rack_pinion")


def input_turn_backstop(text, spec):
    """Safety net: a Turkish "X° cevirince / dondurunce / itince" ("when turned / pushed by X degrees")
    pattern means the input arm turns through a limited angle. If the LLM misses it (e.g. written as
    '45°'), the number is read DIRECTLY from the user's sentence (not invented). The speed (rpm) is read
    the same way. Returns (spec, note or None)."""
    sp = _SPEED.search(text)
    if sp and spec.get("speed_rpm") is None and spec.get("mechanism_type") in _SPEED_KINDS:
        spec = dict(spec, speed_rpm=float(sp.group(1).replace(",", ".")))
    m = _INPUT_TURN.search(text)
    if not m or spec.get("input_swing_deg") is not None:
        return spec, None
    angle = float(m.group(1).replace(",", "."))
    if not 0 < angle < 360:
        return spec, None
    fixed = dict(spec, mechanism_type="rocker_rocker", input_swing_deg=angle,
                 quick_return=False, time_ratio=None)
    return fixed, f"Read from the sentence: the input arm turns {angle:g}° (limited-angle mechanism)."


# ---------------------------------------------------------------- K1: edits of the current design
_NUMBER = re.compile(r"\d+(?:[.,]\d+)?")
# factors written as words ("twice", "iki katına", "yarısı") for 'multiply' edits
_WORD_FACTORS = {2.0: ("twice", "double", "iki kat", "2 kat"), 3.0: ("three times", "triple", "üç kat", "3 kat"),
                 0.5: ("half", "yarı", "yarısı", "yarıya")}
_UNIT_FACTORS = (1.0, 10.0, 9.81, 1000.0, 0.1, 0.001)   # as written, cm -> mm, kg -> N, kN -> N / m -> mm, ...


def _number_in_text(value, text, op):
    """The LLM's number must come from the user's text (possibly unit-converted) - never invented."""
    nums = [float(n.replace(",", ".")) for n in _NUMBER.findall(text)]
    if any(abs(abs(value) - n * f) <= 1e-6 * max(1.0, n * f) for n in nums for f in _UNIT_FACTORS):
        return True
    low = text.lower()
    return op == "multiply" and any(abs(value - k) < 1e-9 and any(w in low for w in words)
                                    for k, words in _WORD_FACTORS.items())


def _field_label(key):
    for mod in MODULES.values():
        for p in mod["params"]:
            if p["key"] == key:
                return p["label"].lower()
    return {"crank_length_mm": "input arm / crank length", "input_swing_deg": "input angle",
            "output_swing_deg": "output swing", "time_ratio": "time ratio Q", "max_load_N": "tip load",
            "power_kW": "motor power", "radial_force_N": "radial force", "roller_radius_mm": "roller radius",
            "material": "material"}.get(key, key.replace("_", " "))


def apply_edit(prev_spec, edit, text):
    """Applies the LLM's change list to the previous spec. Returns (new spec, note) or (None, question).
    Every number is checked against the user's text; a change without an amount is asked about, not guessed."""
    spec, done = dict(prev_spec), []
    for c in edit.get("changes", []):
        key, op, num, choice = c["field"], c["op"], c.get("number"), c.get("choice")
        label = _field_label(key)
        if choice is not None and num is None:
            spec[key] = choice
            done.append(f"{label} → {choice}")
            continue
        if num is None:
            return None, f"How much should the {label} change? Please give a value, e.g. \"make the {label} 40\"."
        if not _number_in_text(num, text, op):
            return None, f"I could not find the value {num:g} for the {label} in your message; please write it out."
        old = spec.get(key)
        if op != "set" and old is None:
            return None, f"The current design has no {label} to change; please give the new value."
        new = num if op == "set" else old + num if op == "add" else old * num
        spec[key] = round(new, 6)
        done.append(f"{label} {'' if old is None else f'{old:g} → '}{new:g}")
    if not done:
        return None, "I understood this as a change, but not what to change. Which value should be different?"
    if spec.get("mechanism_type") in ("crank_rocker", "rocker_rocker"):
        if spec.get("input_swing_deg") is not None:          # the input angle decides the four-bar kind
            full = spec["input_swing_deg"] >= 360
            spec["mechanism_type"] = "crank_rocker" if full else "rocker_rocker"
            if full:
                spec["input_swing_deg"] = None
        if spec.get("time_ratio") and spec["time_ratio"] > 1:
            spec["quick_return"] = True
    return spec, "Changed: " + "; ".join(done) + "."


def design_from_text(text, on_status=None, prev_spec=None):
    """
    Design from a sentence. Returns dict(status, message, spec, ...)
      status: "ok" | "failed" (did not pass the checks) | "missing" | "unsupported"
    on_status(str): optional progress callback.
    prev_spec: the current design's spec (panel); the sentence may be an edit of it (K1).
    """
    say = on_status or (lambda s: None)
    if prev_spec and prev_spec.get("mechanism_type") in llm_extract.KIND_OF_TYPE:
        say("Is this a change to the current design?")
        edit = llm_extract.extract_edit(text, prev_spec)
        if edit.get("is_edit"):
            spec, note = apply_edit(prev_spec, edit, text)
            if spec is None:
                return {"status": "missing", "message": note, "spec": prev_spec, "text": text,
                        "mechanism_type": prev_spec.get("mechanism_type")}
            result = design_from_spec(spec, on_status)
            result["message"] = note + " " + result.get("message", "")
            result.update(text=text, edit=True)
            return result
    say("The LLM is reading the sentence...")
    spec, note = input_turn_backstop(text, extract_spec(text))
    result = design_from_spec(spec, on_status)
    if note and "message" in result:
        result["message"] = note + " " + result["message"]
    result["text"] = text
    return result


def design_from_params(crank_length_mm=30, output_swing_deg=60, time_ratio=1.0,
                       max_load_N=None, material=None, input_swing_deg=360, kind="fourbar",
                       gear_ratio=None, torque_Nm=None, speed_rpm=None,
                       spring_force_N=None, spring_deflection_mm=None, spring_taper=None, outer_diameter_max_mm=None,
                       cam_rise_mm=None, rise_deg=None, dwell_top_deg=None, return_deg=None, motion_law=None,
                       bending_moment_Nm=None, feature=None, shaft_material=None, safety_n=None, show_plates=None,
                       span_mm=None, power_kW=None, stroke_mm=None, offset_mm=None, rod_length_mm=None,
                       follower_mass_kg=None, follower_load_N=None, slots=None, center_distance_mm=None,
                       load_mass_kg=None, load_diameter_mm=None, opening_deg=None, handle_travel_deg=None,
                       arm_length_mm=None, clamp_force_N=None, handle_length_mm=None, teeth=None,
                       force_N=None, travel_mm=None, screw_ends=None, nut_material=None):
    """For the sliders: design straight from numbers, no LLM.
    kind: "fourbar", "reducer", "spur_gear", "spring", "cam" or "shaft" (C0: module choice).
    input_swing_deg >= 360: full-turn input (crank-rocker); less: limited-angle input arm (B9)."""
    if kind == "rack_pinion":
        return design_from_spec({"mechanism_type": "rack_pinion", "force_N": force_N, "travel_mm": travel_mm,
                                 "speed_rpm": speed_rpm or None, "material": material})
    if kind == "power_screw":
        return design_from_spec({"mechanism_type": "power_screw", "force_N": force_N, "travel_mm": travel_mm,
                                 "screw_ends": screw_ends, "nut_material": nut_material, "speed_rpm": speed_rpm or None})
    if kind == "ratchet":
        return design_from_spec({"mechanism_type": "ratchet", "torque_Nm": torque_Nm, "teeth": teeth,
                                 "outer_diameter_max_mm": outer_diameter_max_mm or None, "material": material})
    if kind in ("scotch_yoke", "quick_return"):
        return design_from_spec({"mechanism_type": kind, "stroke_mm": stroke_mm,
                                 "time_ratio": time_ratio if time_ratio and time_ratio > 1 else None,
                                 "max_load_N": max_load_N or None, "speed_rpm": speed_rpm or None,
                                 "material": material})
    if kind == "toggle_clamp":
        return design_from_spec({"mechanism_type": "toggle_clamp", "opening_deg": opening_deg,
                                 "handle_travel_deg": handle_travel_deg, "arm_length_mm": arm_length_mm,
                                 "clamp_force_N": clamp_force_N or None, "handle_length_mm": handle_length_mm,
                                 "material": material})
    if kind == "geneva":
        return design_from_spec({"mechanism_type": "geneva", "slots": slots, "center_distance_mm": center_distance_mm,
                                 "speed_rpm": speed_rpm or None, "load_mass_kg": load_mass_kg or None,
                                 "load_diameter_mm": load_diameter_mm or None, "material": material})
    if kind == "straight_line":
        return design_from_spec({"mechanism_type": "straight_line", "stroke_mm": stroke_mm,
                                 "max_load_N": max_load_N or None, "speed_rpm": speed_rpm or None,
                                 "material": material})
    if kind == "slider_crank":
        return design_from_spec({"mechanism_type": "slider_crank", "stroke_mm": stroke_mm, "offset_mm": offset_mm,
                                 "rod_length_mm": rod_length_mm, "max_load_N": max_load_N or None,
                                 "speed_rpm": speed_rpm or None, "material": material})
    if kind == "reducer":
        return design_from_spec({"mechanism_type": "reducer", "torque_Nm": torque_Nm, "power_kW": power_kW,
                                 "speed_rpm": speed_rpm, "gear_ratio": gear_ratio, "material": material,
                                 "shaft_material": shaft_material, "feature": feature, "safety_n": safety_n})
    if kind == "shaft":
        return design_from_spec({"mechanism_type": "shaft", "torque_Nm": torque_Nm,
                                 "bending_moment_Nm": bending_moment_Nm, "feature": feature,
                                 "shaft_material": shaft_material, "safety_n": safety_n,
                                 "speed_rpm": speed_rpm or None, "span_mm": span_mm or None})
    if kind == "cam":
        return design_from_spec({"mechanism_type": "cam", "cam_rise_mm": cam_rise_mm, "rise_deg": rise_deg,
                                 "dwell_top_deg": dwell_top_deg, "return_deg": return_deg,
                                 "motion_law": motion_law or "cycloidal", "roller_radius_mm": None,
                                 "speed_rpm": speed_rpm or None, "follower_mass_kg": follower_mass_kg or None,
                                 "follower_load_N": follower_load_N or None})
    if kind == "spring":
        return design_from_spec({"mechanism_type": "spring", "spring_force_N": spring_force_N,
                                 "spring_deflection_mm": spring_deflection_mm, "spring_taper": spring_taper,
                                 "outer_diameter_max_mm": outer_diameter_max_mm or None, "material": material,
                                 "show_plates": show_plates})
    if kind == "spur_gear":
        return design_from_spec({"mechanism_type": "spur_gear", "gear_ratio": gear_ratio,
                                 "torque_Nm": torque_Nm, "speed_rpm": speed_rpm or None, "material": material})
    rr = input_swing_deg < 360
    return design_from_spec({
        "mechanism_type": "rocker_rocker" if rr else "crank_rocker", "crank_length_mm": crank_length_mm,
        "output_swing_deg": output_swing_deg, "quick_return": (not rr) and time_ratio > 1.0,
        "time_ratio": time_ratio if (not rr and time_ratio > 1.0) else None,
        "input_swing_deg": input_swing_deg if rr else None,
        "max_load_N": max_load_N or None, "material": material})


def design_from_spec(spec, on_status=None):
    """Design from a structured request (LLM output or sliders)."""
    say = on_status or (lambda s: None)
    result = {"spec": spec}

    mtype = spec["mechanism_type"]
    if mtype == "slider_crank":
        return _slider_crank(result, spec, say)
    if mtype == "straight_line":
        return _straight_line(result, spec, say)
    if mtype == "geneva":
        return _geneva(result, spec, say)
    if mtype == "toggle_clamp":
        return _toggle_clamp(result, spec, say)
    if mtype == "scotch_yoke":
        return _scotch_yoke(result, spec, say)
    if mtype == "ratchet":
        return _ratchet(result, spec, say)
    if mtype == "power_screw":
        return _power_screw(result, spec, say)
    if mtype == "rack_pinion":
        return _rack_pinion(result, spec, say)
    if mtype == "quick_return":
        return _quick_return(result, spec, say)
    if mtype == "reducer":
        return _reducer(result, spec, say)
    if mtype == "spur_gear":
        return _spur_gear(result, spec, say)
    if mtype == "spring":
        return _spring(result, spec, say)
    if mtype == "cam":
        return _cam(result, spec, say)
    if mtype == "shaft":
        return _shaft(result, spec, say)
    if mtype not in ("crank_rocker", "rocker_rocker"):
        return dict(result, status="unsupported",
                    message="Supported: four-bar linkages (crank-rocker or limited-angle input), straight-line "
                            "linkage, Geneva, toggle clamp, slider-crank, Scotch yoke, quick return, ratchet, power screw, rack and pinion, reducers (one or two stages), spur gears, compression springs, "
                            "cams and shafts. This request is not one of them.")
    rr = mtype == "rocker_rocker"
    required = ((("crank_length_mm", "input arm length"), ("input_swing_deg", "input arm turning angle"),
                 ("output_swing_deg", "output arm turning angle")) if rr else
                (("crank_length_mm", "crank length"), ("output_swing_deg", "swing angle")))
    missing = [name for key, name in required if spec.get(key) is None]
    if missing:
        return dict(result, status="missing",
                    message=f"Missing for the design: {', '.join(missing)}.")

    a, psi = spec["crank_length_mm"], spec["output_swing_deg"]
    if not 0 < psi < 180:
        return dict(result, status="missing",
                    message=f"The swing angle must be between 0 and 180° (given: {psi:g}°).")
    notes = []
    if rr:
        t_in = spec["input_swing_deg"]
        if not 0 < t_in < 360:
            return dict(result, status="missing",
                        message=f"The input angle must be between 0 and 360° (given: {t_in:g}°).")
        say("Function generator synthesis (Freudenstein)...")
        fg = _pick_function_generator(a, t_in, psi, spec)
        if fg is None:
            return dict(result, status="failed",
                        message=f"No suitable four-bar found for input {t_in:g}° → output {psi:g}°.")
        mech, frames = function_gen.frames_for(fg)
        mr = function_gen.measure_range(frames)
        grashof_ok, grashof_kind = kin.grashof(mech["a"], mech["b"], mech["c"], mech["d"])
        mu_ok = kin.MU_MIN_DEG <= mr["mu_min"] and mr["mu_max"] <= kin.MU_MAX_DEG
        in_ok = abs(mr["in_swing"] - t_in) < 0.5
        out_ok = abs(abs(mr["out_swing"]) - psi) <= 1.5
        err_ok = fg["err_deg"] <= 1.0
        passed = mu_ok and in_ok and out_ok and err_ok
        Q = 1.0
        meas = {"swing": abs(mr["out_swing"]), "in_swing": mr["in_swing"], "Q": 1.0,
                "mu_min": mr["mu_min"], "mu_max": mr["mu_max"]}
        checks = [
            {"name": "Mechanism type", "ok": True, "detail": grashof_kind},
            {"name": "Input rotation (simulation)", "ok": in_ok,
             "detail": f"{mr['in_swing']:.1f}° (target {t_in:g}°)"},
            {"name": "Output rotation (simulation)", "ok": out_ok,
             "detail": f"{abs(mr['out_swing']):.1f}° (target {psi:g}°)"},
            {"name": "Structural error (linearity)", "ok": err_ok,
             "detail": f"at most {fg['err_deg']:.2f}° (limit 1°)"},
            {"name": "Transmission angle", "ok": mu_ok,
             "detail": f"{mr['mu_min']:.1f}° – {mr['mu_max']:.1f}° "
                       f"(limits {kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°)"},
        ]
    return _finish(result, spec, notes, say, mech, frames, meas, checks, passed, a, psi,
                   Q if rr else None, rr) if rr else _crank_rocker(result, spec, notes, say, a, psi)


def _reducer(result, spec, say):
    """E1: single-stage reducer (gear pair + two shafts + bearings + keys + housing), all coupled.
    Required: ratio, input speed, and input torque or motor power (nothing is invented)."""
    result["mechanism_type"] = "reducer"
    T, P, rpm, ratio = spec.get("torque_Nm"), spec.get("power_kW"), spec.get("speed_rpm"), spec.get("gear_ratio")
    notes = []
    if T is None and P is not None and rpm:
        T = 9550.0 * P / rpm
        notes.append(f"Input torque from motor power: T = 9550 P / n = {T:.1f} N·m.")
    missing = [n for v, n in ((ratio, "ratio"), (rpm, "input speed (rpm)"), (T, "input torque or motor power"))
               if v is None]
    if missing:
        return dict(result, status="missing", message=f"Missing for the reducer: {', '.join(missing)}.")
    ratio = float(ratio)
    if ratio < 1:
        ratio = 1 / ratio
    if not 1 < ratio <= reducer2.MAX_RATIO:
        return dict(result, status="missing",
                    message=f"The reducer covers ratios of 1–{reducer2.MAX_RATIO:g} (given: {ratio:g}).")
    mat_key = spec.get("material") if spec.get("material") in strength.MATERIALS else None
    if mat_key is None:
        mat_key = "steel"
        notes.append("Gear material not given: steel assumed.")
    feature = spec.get("feature") or "keyseat"
    shaft_mat = spec.get("shaft_material") or "1045"
    n = float(spec.get("safety_n") or 2.0)
    if ratio > reducer2.SINGLE_MAX:
        return _reducer2(result, spec, say, float(T), float(rpm), ratio, mat_key, shaft_mat, n, notes)
    say("Reducer: gear pair -> shafts -> bearings -> keys -> housing (iterating until they fit)...")
    r, r_checks = reducer.design(float(T) * 1000.0, float(rpm), ratio, mat_key, shaft_mat, feature, n)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0, "detail": c["detail"]}
              for c in r_checks]
    result.update(checks=checks, notes=notes, target={"torque_Nm": float(T), "speed_rpm": float(rpm), "ratio": ratio})
    if r is None:
        bad = [c["name"] for c in checks if not c["ok"]]
        return dict(result, status="failed", message=" ".join(notes + [
            f"No reducer passes all checks (failing: {', '.join(bad)}). Try a stronger material, a smaller "
            f"torque or a different ratio."]))
    g, s1, s2 = r["gear"], r["shaft_in"], r["shaft_out"]
    notes.append("Losses neglected (output torque = ratio × input torque): conservative for the output side.")
    result.update(
        reducer=r, fusion={"type": "reducer", "passed_checks": True, **r},
        status="ok",
        message=" ".join(notes + [
            f"All checks passed. Gears m {g['module']:g}, z {g['z1']}/{g['z2']} (i = {g['ratio']:.3f}), "
            f"face {g['face_width']:g} mm, centre distance {g['center_distance']:g} mm. Shafts Ø{s1['d']:g} / "
            f"Ø{s2['d']:g} mm on bearings {s1['bearing']['name']} / {s2['bearing']['name']}. Output "
            f"{r['T2'] / 1000:.1f} N·m at {r['rpm2']:.0f} rpm ({r['power_W'] / 1000:.2f} kW)."]))
    return result


def _slider_crank(result, spec, say):
    """F1: slider-crank (piston). Required: the stroke (or the crank length of an in-line slider-crank)."""
    result["mechanism_type"] = "slider_crank"
    S, a_in, e = spec.get("stroke_mm"), spec.get("crank_length_mm"), float(spec.get("offset_mm") or 0.0)
    notes = []
    if S is None and a_in is not None:
        if e:
            return dict(result, status="missing", message="With an offset, please give the stroke (mm).")
        S = 2.0 * a_in
        notes.append(f"Stroke from the crank: S = 2a = {S:g} mm.")
    if S is None:
        return dict(result, status="missing", message="Missing for the slider-crank: stroke (mm) or crank length.")
    S = float(S)
    say("Slider-crank synthesis (stroke, rod ratio, rod angle)...")
    g = slider_crank.synthesize(S, e, spec.get("rod_length_mm"))
    if g is None:
        return dict(result, status="failed", message=f"No crank turns fully with a {S:g} mm stroke"
                    + (f", {e:g} mm offset" if e else "") + " and this rod. Suggestion: a longer rod or less offset.")
    frames = slider_crank.simulate(g["a"], g["b"], g["e"])
    m = slider_crank.measure(frames)
    checks = [
        {"name": "Stroke (simulation)", "ok": abs(m["stroke"] - S) < 0.1, "detail": f"{m['stroke']:.2f} mm (target {S:g})"},
        {"name": "Rod / crank ratio", "ok": g["ratio"] >= slider_crank.RATIO_MIN - 1e-9,
         "detail": f"{g['ratio']:.2f} (at least {slider_crank.RATIO_MIN:g} for smooth running)"},
        {"name": "Rod angle (side load)", "ok": m["phi_max"] <= slider_crank.PHI_MAX + 1e-9,
         "detail": f"{m['phi_max']:.1f}° (at most {slider_crank.PHI_MAX:g}°)"},
    ]
    if e:
        checks.append({"name": "Time ratio Q (offset)", "ok": True, "detail": f"{m['Q']:.3f} (quick return from the offset)"})
    passed = all(c["ok"] for c in checks)
    section, strength_ok, F = strength.DEFAULT_SECTION, True, spec.get("max_load_N")
    mat_key = spec.get("material")
    if passed and F:
        if mat_key not in strength.MATERIALS:
            mat_key = "PLA"
            notes.append("Material not given: PLA (3D print) assumed.")
        mat = strength.MATERIALS[mat_key]
        say("Strength and section sizing...")
        forc = slider_crank.forces(frames, g["a"], float(F))
        sec, s_checks = slider_crank.size(forc, g["a"], g["b"], mat)
        strength_ok = sec is not None
        section = sec or section
        worst = {}
        for c in s_checks:
            if c["group"] not in worst or c["util"] > worst[c["group"]]["util"]:
                worst[c["group"]] = c
        checks += [{"name": f"{k}: {c['name']}", "ok": c["util"] <= 1.0,
                    "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for k, c in worst.items()]
        result["strength"] = {"material": mat["name"], "safety_factor": mat["n"], "load_N": float(F),
                              "rod_force_max_N": forc["F_ab_max"], "input_torque_max_Nmm": forc["M_crank_max"]}
    elif passed:
        notes.append("No piston force given, so no strength calculation: default section (10×4 mm, Ø4 pin) used.")
    section = strength.section_with_bosses({"hole": section["hole"], "shaft": section.get("shaft", section["hole"]),
                                            "links": {n: section["links"][n] for n in ("Crank", "Coupler")}})
    motion = None
    if spec.get("speed_rpm"):
        v, acc = slider_crank.kinematics_at(frames, float(spec["speed_rpm"]))
        motion = {"rpm": float(spec["speed_rpm"]), "v_max": max(abs(x) for x in v), "a_max": max(abs(x) for x in acc)}
        checks.append({"name": "Piston speed / acceleration", "ok": True,
                       "detail": f"v_max {motion['v_max']:.2f} m/s, a_max {motion['a_max']:.0f} m/s² at {motion['rpm']:g} rpm"})
    k = len(frames) // 8                                      # drawn pose: 45° past the outer dead centre
    x0, phi0 = frames[k]["x"], math.radians(frames[k]["phi"])
    th0 = math.radians(frames[k]["theta"])
    pts = {"O2": [0.0, 0.0], "A": [g["a"] * math.cos(th0), g["a"] * math.sin(th0)], "B": [x0, g["e"]]}
    curve = [(round(f["theta"], 1), round(f["x"] - m["x_min"], 3)) for f in frames[::4]]
    ok = passed and strength_ok
    result.update(slider=dict(g, **m, motion=motion), checks=checks, notes=notes, x_curve=curve,
                  section_mm=section, target={"stroke": S, "offset": e},
                  fusion={"type": "slider_crank", "passed_checks": ok, **g, "x_min": m["x_min"], "x_max": m["x_max"],
                          "points_mm": pts, "section_mm": section})
    if not passed:
        msg = "The design does not meet the checks above."
    elif not strength_ok:
        msg = (f"No {strength.MATERIALS[mat_key]['name']} section carries {F:g} N. "
               "Suggestion: a stronger material or a smaller force.")
    else:
        msg = (f"All checks passed. Crank {g['a']:.2f} mm, rod {g['b']:g} mm (ratio {g['ratio']:.2f})"
               + (f", offset {e:g} mm" if e else "") + f", stroke {m['stroke']:.1f} mm, rod angle ≤ {m['phi_max']:.1f}°.")
        if "strength" in result:
            st = result["strength"]
            msg += (f" Max rod force {st['rod_force_max_N']:.0f} N, max crank torque "
                    f"{st['input_torque_max_Nmm'] / 1000:.2f} N·m.")
    result.update(status="ok" if ok else "failed", message=" ".join(notes + [msg]))
    return result


def _straight_line(result, spec, say):
    """F2: Hoeckens straight-line linkage. Required: the length of the straight path."""
    result["mechanism_type"] = "straight_line"
    S = spec.get("stroke_mm")
    if not S:
        return dict(result, status="missing",
                    message="Missing for the straight-line linkage: the length of the straight path (mm).")
    S, F, rpm = float(S), float(spec.get("max_load_N") or 0.0), spec.get("speed_rpm")
    notes = []
    mat_key = spec.get("material")
    if F and mat_key not in strength.MATERIALS:
        mat_key = "PLA"
        notes.append("Material not given: PLA (3D print) assumed.")
    mat = strength.MATERIALS.get(mat_key or "PLA", strength.MATERIALS["PLA"])
    say("Hoeckens linkage: coupler curve, straight part, forces...")
    d, s_checks = coupler.design_straight(S, mat, F, rpm)
    m, seg = d["mech"], d["segment"]
    motion = [c for c in s_checks if c["group"] == "Motion"]
    checks = [{"name": c["name"], "ok": c["util"] <= 1.0 + 1e-9, "detail": c["detail"]} for c in motion]
    strength_ok = True
    if F:
        say("Strength and section sizing...")
        strength_ok = d["section"] is not None
        worst = {}
        for c in s_checks:
            if c["group"] != "Motion" and (c["group"] not in worst or c["util"] > worst[c["group"]]["util"]):
                worst[c["group"]] = c
        checks += [{"name": f"{k}: {c['name']}", "ok": c["util"] <= 1.0,
                    "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for k, c in worst.items()]
        section = d["section"] or strength.DEFAULT_SECTION
        result["strength"] = {"material": mat["name"], "safety_factor": mat["n"], "load_N": F,
                              "input_torque_max_Nmm": d["T_max"], "coupler_moment_Nmm": d["forces"]["M_coupler"]}
    else:
        section = strength.DEFAULT_SECTION
        notes.append("No force on the point given, so no strength calculation: default section (10×4 mm, Ø4 pin).")
    section = strength.section_with_bosses(section)
    # Layers: the coupler runs A -> P; frames moved to the four-bar frame of interference.py (O4 at the origin)
    sh = lambda p: (p[0] - m["d"], p[1])
    lay_frames = [{"A": sh(f["A"]), "B": sh(f["B"]), "P": sh(f["P"])} for f in d["frames"][::2]]
    order, clearances, layers_ok = interference.choose_layer_order(
        {"o2": (-m["d"], 0.0)}, lay_frames, section, dict(strength.LINK_ENDS, Coupler=("A", "P")))
    margin = min((c[2] for c in clearances), default=0.0)
    checks.append({"name": "Layers (a shaft passes a moving link)", "ok": layers_ok,
                   "detail": f"smallest gap {margin + interference.CLEARANCE:.1f} mm "
                             f"(needed {interference.CLEARANCE:g})"})
    passed = all(c["ok"] for c in checks)
    fr0 = d["frames"][(seg["i0"] + seg["count"] // 3) % len(d["frames"])]     # drawn pose: P on the line
    r3 = lambda p: [round(p[0], 3), round(p[1], 3)]
    pts = {"O2": [0.0, 0.0], "O4": [m["d"], 0.0], "A": r3(fr0["A"]), "B": r3(fr0["B"]), "P": r3(fr0["P"])}
    (cx, cy), u = seg["centroid"], seg["direction"]
    ends = [r3((cx - u[0] * seg["length"] / 2, cy - u[1] * seg["length"] / 2)),
            r3((cx + u[0] * seg["length"] / 2, cy + u[1] * seg["length"] / 2))]
    path = [r3(f["P"]) for f in d["frames"][::4]]
    lift = max(abs(coupler._cross(u, (f["P"][0] - cx, f["P"][1] - cy))) for f in d["frames"])   # step height
    lengths = {k: round(m[k], 3) for k in ("a", "b", "c", "d", "p")}
    info = {**lengths, "stroke": seg["length"], "deviation": seg["deviation"], "fraction": seg["fraction"],
            "theta0": seg["theta0"], "theta1": seg["theta1"], "speed_variation": d["speed_variation"], "lift": lift,
            "v_mean_mm_s": d["v_mean_mm_s"], "mu_work": d["mu_work"], "mu_all": d["mu_all"], "T_max": d["T_max"],
            "path": path, "line": ends, "points": pts}
    ok = passed and strength_ok
    result.update(straight=info, checks=checks, notes=notes, section_mm=section, target={"stroke": S},
                  fusion={"type": "straight_line", "passed_checks": ok, "lengths_mm": lengths, "points_mm": pts,
                          "section_mm": section, "layer_order": list(order), "path": path, "line": ends})
    if not strength_ok:
        msg = f"No {mat['name']} section carries {F:g} N. Suggestion: a stronger material or a smaller force."
    elif not passed:
        msg = "The design does not meet the checks above."
    else:
        msg = (f"All checks passed. Hoeckens linkage: crank {m['a']:g} mm, coupler {m['b']:g} mm with the point "
               f"{m['p']:g} mm from the crank pin, rocker {m['c']:g} mm, ground {m['d']:g} mm. The point runs "
               f"{seg['length']:.1f} mm straight (within {seg['deviation']:.2f} mm) for "
               f"{seg['fraction'] * 360:.0f}° of the crank turn, then lifts off and returns fast; its speed along "
               f"the line varies by ±{d['speed_variation'] * 50:.0f}%.")
        if rpm:
            msg += f" Mean speed on the line {d['v_mean_mm_s']:.0f} mm/s at {rpm:g} rpm."
        if F:
            msg += f" Max crank torque {d['T_max'] / 1000:.2f} N·m."
    result.update(status="ok" if ok else "failed", message=" ".join(notes + [msg]))
    return result


def _geneva(result, spec, say):
    """F3: external Geneva (intermittent rotation). Required: number of slots (or the step angle) and the centre
    distance. With the driver speed and a load (mass + diameter of a disk on the wheel shaft) the pin and the
    wheel thickness are sized."""
    result["mechanism_type"] = "geneva"
    n, step, C = spec.get("slots"), spec.get("step_deg"), spec.get("center_distance_mm")
    notes = []
    if n is None and step:
        k = 360.0 / float(step)
        if abs(k - round(k)) > 1e-6:
            return dict(result, status="missing",
                        message=f"A {step:g}° step does not divide the turn; a Geneva wheel needs 360 / n.")
        n = round(k)
        notes.append(f"Step {step:g}°: n = 360 / {step:g} = {n} slots.")
    missing = [name for v, name in ((n, "number of slots (or step angle)"), (C, "centre distance (mm)")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the Geneva mechanism: {', '.join(missing)}.")
    n, C = int(n), float(C)
    if not geneva.N_MIN <= n <= geneva.N_MAX:
        return dict(result, status="failed",
                    message=f"{n} slots: a Geneva wheel needs {geneva.N_MIN}..{geneva.N_MAX} slots.")
    m_kg, D_mm, rpm = spec.get("load_mass_kg"), spec.get("load_diameter_mm"), spec.get("speed_rpm")
    if m_kg and not D_mm:
        return dict(result, status="missing",
                    message="The load's inertia needs its diameter too (a disk of that mass and diameter).")
    J = 0.5 * m_kg * (D_mm / 2000) ** 2 if m_kg else 0.0
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "PLA"
        notes.append("Material not given: PLA (3D print) assumed.")
    mat = strength.MATERIALS[mat_key]
    if not (J and rpm):
        notes.append("No speed + load given, so no strength sizing: pin ~ crank / 5, 5 mm thick.")
    say("Geneva: tangential entry geometry, index motion, pin and slot loads...")
    d, s_checks = geneva.design(n, C, mat, rpm, J, geneva.DENSITY[mat_key])
    g, ld = d["geometry"], d["loads"]
    checks = [{"name": c["name"], "ok": c["util"] <= 1.0 + 1e-9,
               "detail": c["detail"] + (f" · use {c['util'] * 100:.0f}%" if c["group"] in ("Pin", "Slot") else "")}
              for c in s_checks]
    checks.insert(1, {"name": "Index / dwell (driver turn)", "ok": True,
                      "detail": f"{d['index_deg']:.0f}° moving, {d['dwell_deg']:.0f}° at rest"})
    ok = all(c["ok"] for c in checks)
    # table for Fusion: wheel rotation (deg, world, ccw +) for each degree of driver turn from the drawn pose
    # (driver at 180°: mid-dwell, pin away from the wheel)
    al, st = math.degrees(g["alpha"]), 360.0 / n
    table = []
    for k in range(360):
        th = 180 + k - 360                                  # driver angle measured from the next index centre
        if th < -al:
            table.append(0.0)
        elif th > al:
            table.append(-st)
        else:
            table.append(round(math.degrees(geneva.wheel_angle(g, math.radians(th))), 4))
    moving = [(round(th, 1), round(math.degrees(p), 3)) for th, p, _, _ in d["motion"][::4] if th <= al + 1e-9]
    curve = moving + [(round(moving[0][0] + 360, 1), -st)]            # the index, then the dwell (flat)
    om = ld["omega"]
    info = {"n": n, "C": C, "a": g["a"], "R_o": g["R_o"], "d_pin": g["d_pin"], "t": d["t"], "r_lock": g["r_lock"],
            "slot_w": g["slot_w"], "step": st, "index_deg": d["index_deg"], "dwell_deg": d["dwell_deg"],
            "w_ratio_max": d["w_ratio_max"], "acc_max": d["acc_max"], "rpm": rpm, "J": d["J"],
            "alpha_max": d["acc_max"] * om * om if rpm else None, "T_wheel": ld["T_wheel"], "F_pin": ld["F_pin"],
            "T_driver": ld["T_driver"], "shaft_wheel": d["shaft_wheel"], "shaft_driver": d["shaft_driver"],
            "curve": curve}
    result.update(geneva=info, checks=checks, notes=notes, target={"slots": n, "center": C},
                  fusion={"type": "geneva", "passed_checks": ok, "geometry": g, "t": d["t"], "gap": geneva.GAP,
                          "shaft_wheel": d["shaft_wheel"], "shaft_driver": d["shaft_driver"], "table": table})
    if not ok:
        msg = "The design does not meet the checks above."
        if n == 3:
            msg += " Three slots give very high accelerations; 4 to 8 slots are usual."
    else:
        msg = (f"All checks passed. {n}-slot Geneva, centre distance {C:g} mm: crank {g['a']:.1f} mm with a "
               f"Ø{g['d_pin']:g} pin, wheel Ø{2 * g['R_o']:.1f} mm, {d['t']:g} mm thick. The wheel turns {st:g}° while "
               f"the driver turns {d['index_deg']:.0f}°, then rests for {d['dwell_deg']:.0f}°; the pin enters and "
               f"leaves the slots tangentially (no impact). Peak wheel speed {d['w_ratio_max']:.2f}× the driver's.")
        if rpm and J:
            msg += (f" At {rpm:g} rpm: peak wheel acceleration {info['alpha_max']:.0f} rad/s², pin force "
                    f"{ld['F_pin']:.0f} N, driver torque {ld['T_driver'] / 1000:.2f} N·m.")
    result.update(status="ok" if ok else "failed", message=" ".join(notes + [msg]))
    return result


_CLAMP_NAMES = (("Crank", "Handle link"), ("Coupler", "Coupler"), ("Rocker", "Arm"), ("Ground", "Base"))


def _toggle_clamp(result, spec, say):
    """F5: toggle clamp. Required: arm opening angle and spindle distance (arm length). Handle travel and handle
    length have defaults (reported); with a holding force the pins and links are sized."""
    result["mechanism_type"] = "toggle_clamp"
    psi, L = spec.get("opening_deg"), spec.get("arm_length_mm")
    missing = [n for v, n in ((psi, "arm opening angle"), (L, "arm length to the spindle (mm)")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the toggle clamp: {', '.join(missing)}.")
    psi, L = float(psi), float(L)
    notes = []
    travel = spec.get("handle_travel_deg")
    if not travel:
        travel = 90.0
        notes.append("Handle travel not given: 90° assumed.")
    Lh = spec.get("handle_length_mm")
    if not Lh:
        Lh = 2 * L
        notes.append(f"Handle length not given: {Lh:g} mm (twice the arm).")
    F = float(spec.get("clamp_force_N") or 0.0)
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "steel" if F else "PLA"
        notes.append(f"Material not given: {strength.MATERIALS[mat_key]['name']} assumed.")
    mat = strength.MATERIALS[mat_key]
    if not 10 <= psi <= 150 or not 20 <= travel <= 170:
        return dict(result, status="failed", message="The opening must be 10..150° and the handle travel 20..170°.")
    say("Toggle clamp: dead-centre synthesis, over-centre lock, forces...")
    d, s_checks = toggle.design(psi, float(travel), L, mat, F, float(Lh))
    if d is None:
        return dict(result, status="failed", message=s_checks + " Suggestion: more handle travel or less opening.")
    m, fr, an = d["mech"], d["frames"], d["analysis"]

    def rename(text):
        for old, new in _CLAMP_NAMES:
            text = text.replace(old, new)
        return text

    checks = [{"name": c["name"], "ok": c["util"] <= 1.0 + 1e-9, "detail": c["detail"]}
              for c in s_checks if c["group"] == "Motion"]
    strength_ok = True
    if F:
        say("Pins and links for the holding force...")
        strength_ok = d["section"] is not None
        worst = {}
        for c in s_checks:
            if c["group"] != "Motion" and (c["group"] not in worst or c["util"] > worst[c["group"]]["util"]):
                worst[c["group"]] = c
        checks += [{"name": rename(f"{k}: {c['name']}"), "ok": c["util"] <= 1.0,
                    "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for k, c in worst.items()]
    else:
        notes.append("No holding force given, so no strength sizing: default section (10×4 mm, Ø4 pins).")
    section = d["section"] or strength.DEFAULT_SECTION
    section = {"hole": section["hole"], "shaft": section["hole"],
               "links": {n: dict(section["links"][n], boss0=max(section["links"][n]["width"], 2.5 * section["hole"]),
                                 boss1=max(section["links"][n]["width"], 2.5 * section["hole"]))
                         for n in ("Crank", "Coupler", "Rocker", "Ground")}}
    k_dc = an["k_dc"]
    F_ref = F or 100.0
    curve = [(round(f["th"], 1), round(toggle.handle_torque(m, f, F_ref, L) / Lh, 2)) for f in fr[1:k_dc:4]]
    locked = fr[-1]
    r3 = lambda p: [round(p[0], 3), round(p[1], 3)]
    unit = lambda p, q: ((q[0] - p[0]) / math.dist(p, q), (q[1] - p[1]) / math.dist(p, q))
    uh, ua = unit(m["O2"], locked["A"]), unit((0.0, 0.0), locked["B"])
    pts = {"O2": r3(m["O2"]), "O4": [0.0, 0.0], "A": r3(locked["A"]), "B": r3(locked["B"]),
           "H": r3((m["O2"][0] + Lh * uh[0], m["O2"][1] + Lh * uh[1])), "S": r3((L * ua[0], L * ua[1]))}
    # two base plates (bottom holds O2, top holds O4): every pin joins neighbouring layers; a post ties them
    r_post = max(4.0, section["hole"])
    post, gap = toggle.spacer(m, fr, L, Lh, section, r_post)
    checks.append({"name": "Base post clears the moving links", "ok": gap >= toggle.SPACER_GAP,
                   "detail": f"smallest gap {gap:.1f} mm over the stroke (needed {toggle.SPACER_GAP:g})"})
    total = m["travel"] + toggle.OVERCENTRE
    ok = all(c["ok"] for c in checks) and strength_ok
    info = {"a": m["a"], "b": m["b"], "c": m["c"], "d": m["d"], "L_arm": L, "L_handle": Lh, "opening": psi,
            "travel": m["travel"], "overcentre": toggle.OVERCENTRE, "backoff_deg": an["backoff_deg"],
            "backoff_mm": an["backoff_mm"], "F": F, "F_ref": F_ref, "F_AB": an["F_AB"], "R_O4": an["R_O4"],
            "T_stop": an["T_stop"], "mu_locked": an["mu_locked"], "curve": curve, "points": pts,
            "open": {"A": r3(fr[0]["A"]), "B": r3(fr[0]["B"])}, "post": r3(post)}
    result.update(clamp=info, checks=checks, notes=notes, section_mm=section, target={"opening": psi},
                  fusion={"type": "toggle_clamp", "passed_checks": ok, "points_mm": pts, "section_mm": section,
                          "lengths_mm": {"a": m["a"], "b": m["b"], "c": m["c"], "d": m["d"], "L_arm": L,
                                         "L_handle": Lh}, "post": r3(post), "post_r": r_post,
                          "spindle_dir": -1 if fr[0]["psi"] > an["psi_dc"] else 1,
                          "input_range_deg": sorted([0.0, -m["sign"] * total])})
    if not strength_ok:
        msg = f"No {mat['name']} section holds {F:g} N. Suggestion: a stronger material or a smaller force."
    elif not ok:
        msg = "The design does not meet the checks above. Suggestion: more handle travel or less opening."
    else:
        msg = (f"All checks passed. Toggle clamp: arm opens {psi:g}° for {m['travel']:g}° of handle travel; "
               f"handle link {m['a']:.1f} mm, coupler {m['b']:.1f} mm, arm link {m['c']:.1f} mm (spindle at {L:g} mm), "
               f"base {m['d']:.1f} mm. The handle goes {toggle.OVERCENTRE:g}° past the dead centre onto a stop, so a "
               f"push on the spindle locks it harder; there the arm lifts {an['backoff_mm']:.2f} mm, which the "
               f"clamping squeeze must exceed.")
        if F:
            msg += (f" Holding {F:g} N: coupler {an['F_AB']:.0f} N, arm pivot {an['R_O4']:.0f} N, "
                    f"stop torque {an['T_stop'] / 1000:.2f} N·m.")
    result.update(status="ok" if ok else "failed", message=" ".join(notes + [msg]))
    return result


def _slot_checks(s_checks):
    """Worst check per group (the card shows one line each)."""
    worst = {}
    for c in s_checks:
        if c["group"] not in worst or c["util"] > worst[c["group"]]["util"]:
            worst[c["group"]] = c
    return [{"name": f"{k}: {c['name']}", "ok": c["util"] <= 1.0, "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"}
            for k, c in worst.items()]


def _slot_material(spec, F, notes):
    mat_key = spec.get("material")
    if F and mat_key not in strength.MATERIALS:
        mat_key = "PLA"
        notes.append("Material not given: PLA (3D print) assumed.")
    return strength.MATERIALS.get(mat_key or "PLA", strength.MATERIALS["PLA"])


def _scotch_yoke(result, spec, say):
    """F4: Scotch yoke. Required: the stroke (or the crank length)."""
    result["mechanism_type"] = "scotch_yoke"
    S, a = spec.get("stroke_mm"), spec.get("crank_length_mm")
    notes = []
    if not S and a:
        S = 2.0 * a
        notes.append(f"Stroke from the crank: S = 2r = {S:g} mm.")
    if not S:
        return dict(result, status="missing", message="Missing for the Scotch yoke: stroke (mm) or crank length.")
    S, F, rpm = float(S), float(spec.get("max_load_N") or 0.0), spec.get("speed_rpm")
    mat = _slot_material(spec, F, notes)
    say("Scotch yoke: harmonic motion, slot force, sizing...")
    d, s_checks = slotted.yoke_design(S, mat, F)
    r = d["r"]
    checks = [{"name": "Stroke", "ok": True, "detail": f"{S:g} mm = 2 × crank {r:g} mm (exactly harmonic: x = r cos θ)"}]
    ok = True
    if F:
        checks += _slot_checks(s_checks)
        ok = d["section"] is not None
        result["strength"] = {"material": mat["name"], "load_N": F, "input_torque_max_Nmm": d["T_max"]}
    else:
        notes.append("No slide force given, so no strength calculation: default section (10×4 mm, Ø4 pin).")
    section = d["section"] or {"hole": 4.0, "shaft": 4.0, "links": {"Crank": {"width": 10.0, "thickness": 4.0},
                                                                   "Yoke": {"width": 14.0, "thickness": 4.0}}}
    motion = None
    if rpm:
        w = rpm * 2 * math.pi / 60
        motion = {"rpm": float(rpm), "v_max": r * w / 1000, "a_max": r * w * w / 1000,
                  "a_max_slider_crank": r * w * w * (1 + 1 / slider_crank.RATIO_MIN) / 1000}
        checks.append({"name": "Speed / acceleration", "ok": True,
                       "detail": f"v_max {motion['v_max']:.2f} m/s, a_max {motion['a_max']:.1f} m/s² "
                                 f"(a slider-crank with b = 3r: {motion['a_max_slider_crank']:.1f} m/s²)"})
    th0 = math.radians(90)                     # drawn with the crank square to the slide: joint axes along the slot
    curve = [(round(f["theta"], 1), round(f["x"] + r, 3)) for f in d["frames"][::4]]
    result.update(yoke={"r": r, "stroke": S, "motion": motion, "T_max": d.get("T_max")}, checks=checks, notes=notes,
                  section_mm=section, x_curve=curve, target={"stroke": S},
                  fusion={"type": "scotch_yoke", "passed_checks": ok, "r": r, "theta0": 90.0,
                          "points_mm": {"O2": [0.0, 0.0], "A": [r * math.cos(th0), r * math.sin(th0)]},
                          "section_mm": section})
    if not ok:
        msg = f"No {mat['name']} section carries {F:g} N. Suggestion: a stronger material or a smaller force."
    else:
        msg = (f"All checks passed. Scotch yoke: crank {r:g} mm, stroke {S:g} mm, pure harmonic motion; the pin "
               f"runs in a {2 * r + section['hole']:.0f} mm slot square to the slide.")
        if F:
            msg += f" Max crank torque {d['T_max'] / 1000:.2f} N·m."
    result.update(status="ok" if ok else "failed", message=" ".join(notes + [msg]))
    return result


def _quick_return(result, spec, say):
    """F4: crank and slotted lever quick return (shaper). Required: the stroke; time ratio defaults to 1.5."""
    result["mechanism_type"] = "quick_return"
    S = spec.get("stroke_mm")
    if not S:
        return dict(result, status="missing", message="Missing for the quick-return mechanism: stroke (mm).")
    notes = []
    Q = spec.get("time_ratio")
    if not Q:
        Q = 1.5
        notes.append("Time ratio not given: Q = 1.5 assumed (shapers use about 1.4..1.8).")
    S, Q, F, rpm = float(S), float(Q), float(spec.get("max_load_N") or 0.0), spec.get("speed_rpm")
    if not 1.05 <= Q <= 3.0:
        return dict(result, status="failed", message=f"Time ratio {Q:g}: this linkage gives 1.05..3.")
    mat = _slot_material(spec, F, notes)
    say("Crank and slotted lever: geometry from the stroke and the time ratio, forces...")
    d, s_checks = slotted.lever_design(S, Q, mat, F)
    m = d["measure"]
    checks = [{"name": "Stroke (simulation)", "ok": abs(m["stroke"] - S) < 0.1, "detail": f"{m['stroke']:.2f} mm (target {S:g})"},
              {"name": "Time ratio (simulation)", "ok": abs(m["Q"] - Q) < 0.02,
               "detail": f"{m['Q']:.3f} (target {Q:g}): {max(m['to_minus'], m['to_plus']):.0f}° slow, "
                         f"{min(m['to_minus'], m['to_plus']):.0f}° fast"}]
    ok = all(c["ok"] for c in checks)
    if F:
        checks += _slot_checks(s_checks)
        ok = ok and d["section"] is not None
        result["strength"] = {"material": mat["name"], "load_N": F, "input_torque_max_Nmm": d["T_max"]}
    else:
        notes.append("No cutting force given, so no strength calculation: default section (10×4 mm, Ø4 pin).")
    section = d["section"] or {"hole": 4.0, "shaft": 4.0, "links": {"Crank": {"width": 10.0, "thickness": 4.0},
                                                                   "Lever": {"width": 14.0, "thickness": 4.0}}}
    speeds = None
    if rpm:
        t_turn = 60.0 / rpm
        slow, fast = max(m["to_minus"], m["to_plus"]), min(m["to_minus"], m["to_plus"])
        speeds = {"rpm": float(rpm), "v_cut": S / (t_turn * slow / 360) / 1000, "v_return": S / (t_turn * fast / 360) / 1000}
        checks.append({"name": "Mean speeds", "ok": True,
                       "detail": f"cutting {speeds['v_cut']:.2f} m/s, return {speeds['v_return']:.2f} m/s at {rpm:g} rpm"})
    x0 = min(f["x"] for f in d["frames"])
    curve = [(round(f["theta"], 1), round(f["x"] - x0, 3)) for f in d["frames"][::4]]
    th0 = 90.0                                                  # drawn with the crank pin at the top
    result.update(lever={k: d[k] for k in ("r", "d", "L", "Q", "stroke", "slow_direction")} | {"Q_meas": m["Q"],
                  "slow_deg": max(m["to_minus"], m["to_plus"]), "speeds": speeds, "T_max": d.get("T_max"),
                  "N_max": d.get("N_max")}, checks=checks, notes=notes, section_mm=section, x_curve=curve,
                  target={"stroke": S, "Q": Q},
                  fusion={"type": "quick_return", "passed_checks": ok, "r": d["r"], "d": d["d"], "L": d["L"],
                          "theta0": th0, "psi_max": math.degrees(math.asin(d["r"] / d["d"])), "section_mm": section})
    if not ok:
        msg = (f"No {mat['name']} section carries {F:g} N. Suggestion: a stronger material or a smaller force."
               if F and d["section"] is None else "The design does not meet the checks above.")
    else:
        msg = (f"All checks passed. Crank {d['r']:.1f} mm at {d['d']:.1f} mm above the lever pivot, lever {d['L']:.1f} mm: "
               f"stroke {m['stroke']:.1f} mm, time ratio {m['Q']:.2f} (the slow stroke runs towards {d['slow_direction']} "
               f"with the crank turning counter-clockwise).")
        if F:
            msg += f" Max crank torque {d['T_max'] / 1000:.2f} N·m, max pin force on the lever {d['N_max']:.0f} N."
    result.update(status="ok" if ok else "failed", message=" ".join(notes + [msg]))
    return result


def _ratchet(result, spec, say):
    """F6: ratchet and pawl. Required: the holding torque. Teeth default to 24; the diameter is sized unless given."""
    result["mechanism_type"] = "ratchet"
    T = spec.get("torque_Nm")
    if not T:
        return dict(result, status="missing", message="Missing for the ratchet: the torque it must hold (N·m).")
    notes = []
    z = spec.get("teeth")
    if not z:
        z = 24
        notes.append("Number of teeth not given: 24 assumed (15° of backlash).")
    z = int(z)
    if not 8 <= z <= 60:
        return dict(result, status="failed", message=f"{z} teeth: this module designs 8..60.")
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "PLA"
        notes.append("Material not given: PLA (3D print) assumed.")
    mat = strength.MATERIALS[mat_key]
    D = spec.get("outer_diameter_max_mm")
    say("Ratchet: hook angle from friction, tooth / pawl / pin sizing, the pawl's ride over the teeth...")
    d, s_checks = ratchet.design(float(T) * 1000, mat_key, z, float(D) if D else None)
    g = d["geometry"]
    worst = {}
    for c in s_checks:
        if c["group"] not in worst or c["util"] > worst[c["group"]]["util"]:
            worst[c["group"]] = c
    checks = [{"name": c["name"], "ok": c["util"] <= 1.0,
               "detail": c["detail"] + (f" · use {c['util'] * 100:.0f}%" if k != "Motion" else "")}
              for k, c in worst.items()]
    ok = all(c["ok"] for c in checks)
    pose = ratchet.pawl_pose(g, d["lp"])
    head, top = ratchet.pawl_points(g, pose, d["w_pawl"])
    table, lift_sign = ratchet.ride_table(g, pose, head)
    r3 = lambda p: [round(p[0], 3), round(p[1], 3)]
    info = {"z": z, "R": g["R"], "h": g["h"], "pitch": g["pitch"], "hook": g["alpha"], "b": d["b"], "lp": d["lp"],
            "w_pawl": d["w_pawl"], "d_pin": d["d_pin"], "d_shaft": d["d_shaft"], "F": d["F"], "mu": d["mu"],
            "backlash": d["backlash_deg"], "T": float(T), "lift_max": max(table),
            "outline": [r3(p) for p in ratchet.outline(g)], "head": [r3(p) for p in head], "top": r3(top),
            "pivot": r3(pose["pivot"])}
    result.update(ratchet=info, checks=checks, notes=notes, target={"torque": float(T)},
                  fusion={"type": "ratchet", "passed_checks": ok, "outline": info["outline"], "head": info["head"],
                          "top": info["top"], "pivot": info["pivot"], "b": d["b"], "w_pawl": d["w_pawl"],
                          "d_pin": d["d_pin"], "d_shaft": d["d_shaft"], "R": g["R"], "table": table,
                          "lift_sign": lift_sign})
    if not ok:
        msg = "The design does not meet the checks above. Suggestion: fewer teeth, a larger diameter or a stronger material."
    else:
        msg = (f"All checks passed. Ratchet Ø{2 * g['R']:g} mm, {z} teeth {g['h']:.1f} mm high, {d['b']:g} mm thick, "
               f"hook angle {g['alpha']:.1f}° (friction angle {math.degrees(math.atan(d['mu'])):.1f}°) so the pawl pulls "
               f"itself in; the tooth takes {d['F']:.0f} N at {T:g} N·m. It turns freely counter-clockwise and "
               f"may slip back up to {360 / z:.1f}° before a tooth catches; the pawl lifts {max(table):.1f}° over each tooth.")
    result.update(status="ok" if ok else "failed", message=" ".join(notes + [msg]))
    return result


def _power_screw(result, spec, say):
    """G5: power screw (Tr). Required: the axial load. Travel sets the column length; ends: both supported or
    a jack with a free top (C = 1 / 0.25)."""
    result["mechanism_type"] = "power_screw"
    F = spec.get("force_N")
    if not F:
        return dict(result, status="missing", message="Missing for the power screw: the axial load (N or kg).")
    notes = []
    F = float(F)
    L = spec.get("travel_mm")
    if not L:
        L = 200.0
        notes.append("Travel not given: 200 mm assumed (it sets the column length).")
    L = float(L)
    jack = (spec.get("screw_ends") or "supported") == "jack"
    C = 0.25 if jack else 1.0
    nut_key = spec.get("nut_material") or "bronze"
    if nut_key not in power_screw.NUT:
        nut_key = "bronze"
    if not spec.get("nut_material"):
        notes.append("Nut material not given: bronze assumed (steel screw on a bronze nut).")
    say("Power screw: Tr size, self-locking, torques, column and thread checks...")
    d, s_checks = power_screw.design(F, nut_key, 2.0, L, C, "ball")
    if d is None:
        worst = max(s_checks, key=lambda c: c["util"])
        return dict(result, status="failed", checks=[{"name": c["name"], "ok": c["util"] <= 1, "detail": c["detail"]}
                                                      for c in s_checks],
                    message=f"No Tr size up to Tr80 carries {F:g} N here ({worst['name']}). Suggestion: a shorter "
                            "travel or supported ends.")
    th = d["thread"]
    checks = [{"name": c["name"], "ok": c["util"] <= 1.0,
               "detail": c["detail"] + (f" · use {c['util'] * 100:.0f}%" if not c["name"].startswith("Self") else "")}
              for c in s_checks]
    rpm = spec.get("speed_rpm")
    motion = None
    if rpm:
        v = rpm * th["l"] / 60                                  # mm/s
        motion = {"rpm": float(rpm), "v_mm_s": v, "P_W": (d["T_R"] + d["T_c"]) / 1000 * rpm * 2 * math.pi / 60}
        checks.append({"name": "Speed and power", "ok": True,
                       "detail": f"{v:.1f} mm/s at {rpm:g} rpm, {motion['P_W']:.0f} W to raise"})
    info = {"d": th["d"], "P": th["P"], "l": th["l"], "dm": th["dm"], "dr": th["dr"], "n_t": d["n_t"],
            "nut_len": d["nut_len"], "T_R": d["T_R"], "T_L": d["T_L"], "T_c": d["T_c"], "efficiency": d["efficiency"],
            "F": F, "travel": L, "C": C, "nut": d["nut"]["name"], "motion": motion}
    result.update(screw=info, checks=checks, notes=notes, target={"force": F},
                  fusion={"type": "power_screw", "passed_checks": True, "d": th["d"], "P": th["P"], "l": th["l"],
                          "nut_len": max(d["nut_len"], th["d"]), "travel": L,
                          "designation": f"TR{th['d']:g}x{th['P']:g}"})
    msg = (f"All checks passed. Tr{th['d']:g}×{th['P']:g} steel screw with a {d['nut_len']:g} mm {nut_key} "
           f"nut ({d['n_t']} threads): raising {F:g} N takes {d['T_R'] / 1000:.2f} N·m (+{d['T_c'] / 1000:.2f} at a ball thrust "
           f"bearing), efficiency {100 * d['efficiency']:.0f} %; it holds the load by itself (self-locking even lubricated).")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _rack_pinion(result, spec, say):
    """G4: rack and pinion. Required: the force on the rack; travel defaults to 200 mm (reported)."""
    result["mechanism_type"] = "rack_pinion"
    F = spec.get("force_N")
    if not F:
        return dict(result, status="missing", message="Missing for the rack and pinion: the force on the rack (N or kg).")
    notes = []
    S = spec.get("travel_mm")
    if not S:
        S = 200.0
        notes.append("Travel not given: 200 mm assumed.")
    F, S, rpm = float(F), float(S), spec.get("speed_rpm")
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "steel"
        notes.append("Material not given: steel assumed.")
    mat = strength.MATERIALS[mat_key]
    say("Rack and pinion: module, teeth, face width, contact ratio...")
    d, s_checks = rack_pinion.design(F, S, mat, rpm)
    checks = [{"name": c["name"], "ok": c["util"] <= 1.0, "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"}
              for c in s_checks]
    if d is None:
        return dict(result, status="failed", checks=checks,
                    message=f"No pinion up to module {gears.MODULES[-1]:g} carries {F:g} N in {mat['name']}.")
    motion = None
    if rpm:
        motion = {"rpm": float(rpm), "v_mm_s": d["per_turn"] * rpm / 60}
        checks.append({"name": "Rack speed", "ok": True, "detail": f"{motion['v_mm_s']:.0f} mm/s at {rpm:g} rpm"})
    info = dict(d, F=F, travel=S, material=mat["name"], motion=motion)
    result.update(rack=info, checks=checks, notes=notes, target={"force": F},
                  fusion={"type": "rack_pinion", "passed_checks": True, "m": d["module"], "z": d["z"],
                          "b": d["face_width"], "bore": d["bore"], "back": d["back"], "rack_len": d["rack_len"],
                          "travel": S})
    msg = (f"All checks passed. Pinion m {d['module']:g}, {d['z']} teeth (Ø{d['d']:g} mm), rack {d['rack_len']:.0f} mm long, "
           f"face {d['face_width']:g} mm: the rack moves {d['per_turn']:.1f} mm per pinion turn ({d['turns']:.2f} turns for "
           f"{S:g} mm); holding {F:g} N takes {d['T'] / 1000:.2f} N·m on a Ø{d['bore']:g} steel shaft.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _reducer2(result, spec, say, T, rpm, ratio, mat_key, shaft_mat, n, notes):
    """E2: two-stage reducer (ratio above one stage's practical limit)."""
    notes.append(f"Ratio {ratio:g} > {reducer2.SINGLE_MAX:g}: two stages.")
    say("Two-stage reducer: ratio split -> gears -> three shafts -> bearings -> housing...")
    r, r_checks = reducer2.design(T * 1000.0, rpm, ratio, mat_key, shaft_mat, n)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0, "detail": c["detail"]} for c in r_checks]
    result.update(checks=checks, notes=notes, target={"torque_Nm": T, "speed_rpm": rpm, "ratio": ratio})
    if r is None:
        bad = [c["name"] for c in checks if not c["ok"]]
        return dict(result, status="failed", message=" ".join(notes + [
            f"No two-stage reducer passes all checks (failing: {', '.join(bad[:6])}). Try a stronger material or "
            "a smaller torque."]))
    g1, g2 = r["stages"]
    s = r["shafts"]
    notes.append("Losses neglected (output torque = ratio × input torque): conservative for the output side.")
    result.update(
        reducer2=r, fusion={"type": "reducer2", "passed_checks": True, **r}, status="ok",
        message=" ".join(notes + [
            f"All checks passed. Stages {g1['ratio']:.3f} × {g2['ratio']:.3f} = {r['ratio']:.2f}; modules "
            f"{g1['module']:g} / {g2['module']:g}, centre distances {g1['center_distance']:g} / "
            f"{g2['center_distance']:g} mm. Shafts Ø{s[0]['d']:g} / Ø{s[1]['d']:g} / Ø{s[2]['d']:g} mm on bearings "
            f"{' / '.join(x['bearing']['name'] for x in s)}. Output {r['T'][2] / 1000:.1f} N·m at "
            f"{r['rpm'][2]:.0f} rpm ({r['power_W'] / 1000:.2f} kW)."]))
    return result


def _spur_gear(result, spec, say):
    """C9: spur gear pair. Required: ratio and input torque (the torque is never invented)."""
    missing = [name for key, name in (("gear_ratio", "ratio"), ("torque_Nm", "input torque (N·m)"))
               if spec.get(key) is None]
    if missing:
        return dict(result, status="missing", mechanism_type="spur_gear",
                    message=f"Missing for the gears: {', '.join(missing)}.")
    ratio, T1 = float(spec["gear_ratio"]), float(spec["torque_Nm"]) * 1000.0
    if ratio < 1:
        ratio = 1 / ratio                     # 1:3 and 3:1 are the same pair; the pinion is always the input
    if not 1 <= ratio <= 8:
        return dict(result, status="missing", mechanism_type="spur_gear",
                    message=f"A single spur stage needs a ratio of 1–8 (given: {ratio:g}).")
    notes = []
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "PLA"
        notes.append("Material not given: PLA (3D print) assumed.")
    mat = strength.MATERIALS[mat_key]
    say("Gear synthesis (Lewis bending, contact ratio)...")
    g, g_checks = gears.design(ratio, T1, mat, spec.get("speed_rpm"))
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0 or c["group"] == "Contact",
               "detail": c["detail"] + (f" · use {c['util'] * 100:.0f}%" if c["group"] != "Contact" else "")}
              for c in g_checks]
    result.update(mechanism_type="spur_gear", checks=checks, notes=notes)
    if g is None:
        return dict(result, status="failed",
                    message=" ".join(notes + [f"No {mat['name']} gear pair carries {T1 / 1000:g} N·m input torque. "
                                              f"Suggestion: a stronger material or a smaller torque."]))
    checks.insert(0, {"name": "Ratio", "ok": g["ratio_err_pct"] <= 2.0,
                      "detail": f"{g['z2']}/{g['z1']} = {g['ratio']:.3f} (target {ratio:g}, off by {g['ratio_err_pct']:.1f}%)"})
    checks.insert(1, {"name": "Undercut", "ok": g["z1"] >= gears.z_min(),
                      "detail": f"z₁ = {g['z1']} ≥ {gears.z_min():.1f}"})
    result.update(
        gear=g, target={"ratio": ratio, "torque_Nm": T1 / 1000},
        fusion={"type": "spur_gear", "passed_checks": True, **g},
        status="ok",
        message=" ".join(notes + [
            f"All checks passed. Module {g['module']:g}, z = {g['z1']}/{g['z2']}, "
            f"face width {g['face_width']:g} mm, centre distance {g['center_distance']:g} mm, "
            f"shafts Ø{g['bore1']:g}/Ø{g['bore2']:g} mm (steel). Output torque {T1 * g['ratio'] / 1000:.2f} N·m."]))
    return result


def _spring(result, spec, say):
    """C10: compression spring (cylindrical or conical). Required: force and deflection (never invented)."""
    result["mechanism_type"] = "spring"
    missing = [name for key, name in (("spring_force_N", "max force"),
                                      ("spring_deflection_mm", "deflection at that force (mm)"))
               if spec.get(key) is None]
    if missing:
        return dict(result, status="missing", message=f"Missing for the spring: {', '.join(missing)}.")
    F, y = float(spec["spring_force_N"]), float(spec["spring_deflection_mm"])
    notes = []
    taper = spec.get("spring_taper")
    if taper is None and spec.get("spring_shape") == "conical":
        taper = 0.5
        notes.append("Taper not given: the small end is half of the large end (D₂/D₁ = 0.5).")
    taper = min(1.0, max(0.3, float(taper))) if taper else 1.0
    if spec.get("material") == "steel" or spec.get("material") is None:
        mat = springs.SPRING_MATERIALS["music_wire"]
    else:
        mat = springs.SPRING_MATERIALS["music_wire"]
        notes.append("Springs are made of steel wire (plastic / aluminium springs are not suitable): music wire assumed.")
    say("Spring synthesis (Shigley: Bergsträsser, solid height, buckling)...")
    geo, s_checks = springs.design(F, y, mat, taper, spec.get("outer_diameter_max_mm") or None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": c["detail"] + (f" · use {c['util'] * 100:.0f}%" if c["group"] in ("Stress", "Buckling") else "")}
              for c in s_checks]
    result.update(checks=checks, notes=notes, target={"force_N": F, "deflection_mm": y, "taper": taper})
    if geo is None:
        return dict(result, status="failed", message=" ".join(notes + [
            f"No spring found for {F:g} N / {y:g} mm. Suggestion: allow more deflection, relax the outer "
            f"diameter limit or lower the force."]))
    kind = "conical" if taper < 1 else "cylindrical"
    result.update(
        spring=dict(geo, material=mat["name"]),
        fusion={"type": "spring", "passed_checks": True, **geo, "show_plates": spec.get("show_plates") == "yes"},
        status="ok",
        message=" ".join(notes + [
            f"All checks passed ({kind}, {mat['name']}). Wire Ø{geo['d']:g} mm, mean diameter "
            + (f"{geo['D1']:.1f} → {geo['D2']:.1f}" if taper < 1 else f"{geo['D1']:.1f}")
            + f" mm, {geo['Na']:g} active + 2 end coils, free length {geo['L0']:.1f} mm, k = {geo['k']:.2f} N/mm."]))
    return result


def _shaft(result, spec, say):
    """C11b: rotating shaft, DE-Goodman. Required: torque; bending moment or (radial force + bearing span)."""
    result["mechanism_type"] = "shaft"
    notes = []
    T = spec.get("torque_Nm")
    M = spec.get("bending_moment_Nm")
    if M is None and spec.get("radial_force_N") is not None and spec.get("span_mm") is not None:
        M = spec["radial_force_N"] * spec["span_mm"] / 1000.0 / 4.0
        notes.append(f"Bending moment from the force at mid-span of the two-bearing shaft: "
                     f"M = F·L/4 = {M:.1f} N·m (simply supported beam).")
    missing = [n for v, n in ((T, "torque (N·m)"), (M, "bending moment or force + bearing span")) if v is None]
    if missing:
        return dict(result, status="missing", message=f"Missing for the shaft: {', '.join(missing)}.")
    feature = spec.get("feature") or "keyseat"
    if not spec.get("feature"):
        notes.append("Stress raiser not given: keyseat assumed (K_f = 2.14, K_fs = 3.0).")
    mat = spec.get("shaft_material") or "1045"
    if not spec.get("shaft_material"):
        notes.append("Steel not given: cold-drawn AISI 1045 assumed.")
    n = float(spec.get("safety_n") or 2.0)
    rpm = spec.get("speed_rpm")
    if not rpm:
        notes.append("Speed not given: bearing life and critical speed were not checked.")
    span = spec.get("span_mm")
    say("Shaft design (DE-Goodman fatigue, bearings, key, deflection)...")
    g, s_checks = shafts.design(float(M) * 1000.0, float(T) * 1000.0, mat, feature, n, span=span,
                                rpm=float(rpm) if rpm else None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0, "detail": c["detail"]} for c in s_checks]
    result.update(checks=checks, notes=notes,
                  target={"torque_Nm": float(T), "bending_moment_Nm": float(M), "n": n})
    if g is None:
        bad = [c["name"] for c in checks if not c["ok"]]
        return dict(result, status="failed", message=" ".join(notes + [
            f"No standard diameter works (failing: {', '.join(bad)}). "
            "Suggestion: lower the load, shorten the bearing span or pick a stronger steel."]))
    g["span_mm"] = span
    fname = shafts.FEATURES[feature][0]
    if g["raised"]:
        notes.append(f"Ø{g['d_min']:.2f} mm was enough at the hub; a thinner shaft failed "
                     f"({', '.join(g['raised_by']).lower()}), so the diameter was raised to Ø{g['d']:g} mm.")
    if rpm:
        notes.append("The critical speed includes only the shaft's own mass, not the gear / pulley on the hub.")
    b = g["bearing"]
    result.update(
        shaft=g, fusion={"type": "shaft", "passed_checks": True, **g},
        status="ok",
        message=" ".join(notes + [
            f"All checks passed. Shaft Ø{g['d']:g} mm ({g['material']}, {fname}), bearings "
            f"{b['name']} (Ø{b['bore']}×Ø{b['OD']}×{b['B']}). Fatigue safety n_f = {g['nf']:.2f}, "
            f"yield safety n_y = {g['ny']:.2f}."]))
    return result


LAW_NAMES = {"cycloidal": "cycloidal", "harmonic": "simple harmonic", "poly345": "3-4-5 polynomial"}


def _cam(result, spec, say):
    """C11a: disk cam + roller follower. Required: lift and rise angle (never invented)."""
    result["mechanism_type"] = "cam"
    missing = [name for key, name in (("cam_rise_mm", "lift (stroke, mm)"), ("rise_deg", "rise angle"))
               if spec.get(key) is None]
    if missing:
        return dict(result, status="missing", message=f"Missing for the cam: {', '.join(missing)}.")
    notes = []
    h, b1 = float(spec["cam_rise_mm"]), float(spec["rise_deg"])
    b3 = spec.get("return_deg")
    if b3 is None:
        b3 = b1
        notes.append(f"Return angle not given: same as the rise ({b1:g}°).")
    b2 = float(spec.get("dwell_top_deg") or 0.0)
    law_ = spec.get("motion_law") or "cycloidal"
    if not spec.get("motion_law"):
        notes.append("Motion law not given: cycloidal chosen (no infinite jerk).")
    say("Cam synthesis (pressure angle, curvature)...")
    d, c_checks = cams.design(h, b1, b2, float(b3), law_, spec.get("roller_radius_mm"))
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": c["detail"]} for c in c_checks]
    result.update(checks=checks, notes=notes,
                  target={"rise": h, "rise_deg": b1, "dwell_top_deg": b2, "return_deg": float(b3), "law": law_})
    if d is None:
        return dict(result, status="failed", message=" ".join(notes + [
            "No suitable cam found. Suggestion: larger rise / return angles, and keep their sum below 360°."]))
    table = cams.svaj(h, b1, b2, float(b3), law_, n=360)
    msg = (f"All checks passed ({LAW_NAMES[law_]}). Base circle R_b = {d['Rb']:g} mm, roller "
           f"R = {d['Rf']:g} mm, lift {h:g} mm: {b1:g}° rise, {b2:g}° dwell, {float(b3):g}° return, "
           f"{d['dwell_bottom']:g}° dwell at the bottom.")
    rpm, mass = spec.get("speed_rpm"), spec.get("follower_mass_kg")
    if rpm and mass:                                  # E3: the follower spring is designed with the cam
        say("Follower spring (jump check)...")
        sysd, s_checks = cam_system.design(d, cams.svaj(h, b1, b2, float(b3), law_, n=720), float(rpm), float(mass),
                                           float(spec.get("follower_load_N") or 0.0))
        checks += [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0, "detail": c["detail"]}
                   for c in s_checks]
        if sysd is None:
            return dict(result, status="failed", cam=d, message=" ".join(notes + [
                f"The cam works, but no spring keeps a {mass:g} kg follower on it at {rpm:g} rpm. Suggestion: lower "
                "speed, a lighter follower, or longer rise / return angles (gentler motion)."]))
        sp = sysd["spring"]
        result["cam_spring"] = sysd
        notes.append("Gravity and damping neglected in the jump check.")
        msg += (f" Follower spring: wire Ø{sp['d']:g} mm, mean Ø{sp['D1']:.1f} mm, {sp['Na']:g} active coils, "
                f"k = {sysd['k']:.2f} N/mm, preload {sysd['preload_N']:.1f} N; smallest contact force "
                f"{sysd['contact_min']:.1f} N at {rpm:g} rpm.")
    elif rpm or mass:
        notes.append("For the follower spring both the cam speed and the follower mass are needed.")
    result.update(
        cam=d, cam_curve=[(round(t, 1), round(s, 3)) for t, s, _, _ in table[::2]],
        fusion={"type": "cam", "passed_checks": True, **d, "s_table": [round(s, 4) for _, s, _, _ in table]},
        status="ok", message=" ".join(notes + [msg]))
    return result


def _pick_function_generator(a, t_in, psi, spec):
    """The first kinematic candidate that also passes strength (if a load is given) and layer interference.
    If none does, the kinematically best one (the checks then show why)."""
    cands = function_gen.candidates(a, t_in, psi)
    if not cands:
        return function_gen.synthesize(a, t_in, psi)   # best candidate, even though it misses the kinematic target
    F, mat_key = spec.get("max_load_N"), spec.get("material")
    mat = strength.MATERIALS.get(mat_key) or strength.MATERIALS["PLA"]
    for fg in cands:
        mech, frames = function_gen.frames_for(fg)
        section = strength.DEFAULT_SECTION
        if F:
            forces = strength.analyze_forces(mech, frames, F)
            section, _ = strength.size_links(forces, {k: mech[k] for k in "abcd"}, mat)
            if section is None:
                continue
        if interference.choose_layer_order(mech, frames, section)[2]:
            return fg
    return cands[0]


def _crank_rocker(result, spec, notes, say, a, psi):
    # Time ratio Q: the user's value if given; if "quick return" is asked without a value the ENGINE
    # picks it (the LLM never invents numbers); if not mentioned, Q = 1.
    Q = spec.get("time_ratio")
    if Q is not None:
        if not 1.0 <= Q <= 3.0:
            return dict(result, status="missing",
                        message=f"The time ratio Q must be between 1 and 3 (given: {Q:g}).")
    elif spec.get("quick_return"):
        say("Searching for a feasible time ratio (Q)...")
        Q = max_feasible_q(a, psi)
        if Q:
            notes.append(f"Q not given: the largest value keeping the transmission angle within 45°–135° "
                         f"was chosen: Q = {Q:g}.")
        else:
            Q = 1.0
            notes.append(f"At {psi:g}° swing a quick return spoils the transmission angle; "
                         f"designed without quick return (Q = 1).")
    else:
        Q = 1.0

    say("Kinematic synthesis and checks...")
    mech = kin.synthesize_quick_return(a, psi, Q)
    frames = kin.simulate(mech)
    meas = kin.measure(frames)
    grashof_ok, grashof_kind = kin.grashof(mech["a"], mech["b"], mech["c"], mech["d"])
    mu_ok = kin.MU_MIN_DEG <= meas["mu_min"] and meas["mu_max"] <= kin.MU_MAX_DEG
    swing_ok = abs(meas["swing"] - psi) < 0.5
    q_ok = abs(meas["Q"] - Q) < 0.03
    passed = grashof_ok and mu_ok and swing_ok and q_ok

    checks = [
        {"name": "Grashof", "ok": grashof_ok, "detail": grashof_kind},
        {"name": "Swing (simulation)", "ok": swing_ok,
         "detail": f"{meas['swing']:.2f}° (target {psi:g}°)"},
        {"name": "Transmission angle", "ok": mu_ok,
         "detail": f"{meas['mu_min']:.1f}° – {meas['mu_max']:.1f}° "
                   f"(limits {kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°)"},
    ]
    if Q > 1.0:
        checks.append({"name": "Time ratio Q (simulation)", "ok": q_ok,
                       "detail": f"{meas['Q']:.2f} (target {Q:g})"})
    return _finish(result, spec, notes, say, mech, frames, meas, checks, passed, a, psi, Q, False)


def _finish(result, spec, notes, say, mech, frames, meas, checks, passed, a, psi, Q, rr):
    """Shared by both four-bar kinds: strength, layer interference, Fusion data, message."""
    # ---- strength: with a load the sections are sized; without one nothing is invented
    section, strength_ok = strength.DEFAULT_SECTION, True
    F, mat_key = spec.get("max_load_N"), spec.get("material")
    if passed and F:
        if mat_key not in strength.MATERIALS:
            mat_key = "PLA"
            notes.append("Material not given: PLA (3D print) assumed.")
        mat = strength.MATERIALS[mat_key]
        say("Strength and section sizing...")
        forces = strength.analyze_forces(mech, frames, F)
        sec, s_checks = strength.size_links(forces, {k: mech[k] for k in "abcd"}, mat)
        strength_ok = sec is not None
        if sec:
            section = sec
        worst = {}
        for c in s_checks:                      # most critical check per group
            if c["group"] not in worst or c["util"] > worst[c["group"]]["util"]:
                worst[c["group"]] = c
        for g, c in worst.items():
            checks.append({"name": f"{g}: {c['name']}", "ok": c["util"] <= 1.0,
                           "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"})
        result["strength"] = {
            "material": mat["name"], "safety_factor": mat["n"], "load_N": F,
            "coupler_force_max_N": forces["F_ab_max"],
            "input_torque_max_Nmm": forces["M_crank_max"],
            "checks": s_checks,
        }
    elif passed:
        notes.append("No load given, so no strength calculation: "
                     "default section (10×4 mm, Ø4 pin) used.")
    section = strength.section_with_bosses(section)
    result["section_mm"] = section

    # ---- layer interference: a shaft through the middle layers must not hit the moving links
    order, clearances, layers_ok = interference.choose_layer_order(mech, frames, section)
    for p, between, margin in clearances:
        checks.append({"name": f"Layer interference ({p} shaft)", "ok": margin >= 0,
                       "detail": f"passes through the {', '.join(between)} layer; smallest clearance "
                                 f"{margin + interference.CLEARANCE:.1f} mm (needed {interference.CLEARANCE:g})"})
    if passed and not layers_ok:
        notes.append("In no layer order does the shaft clear the moving links.")

    fusion = kin.to_fusion_dict(mech, frames, passed and strength_ok and layers_ok)
    fusion["section_mm"] = section
    fusion["layer_order"] = list(order)
    if rr:
        # Fusion draws frame len//8; relative to it the input arm moves within this range (joint limits)
        k = len(frames) // 8
        fusion["input_range_deg"] = [frames[0]["theta"] - frames[k]["theta"],
                                     frames[-1]["theta"] - frames[k]["theta"]]
        th0 = frames[0]["theta"]
        mu_curve = [(round(f["theta"] - th0, 1), round(f["mu"], 2)) for f in frames]
        mu_axis = [0, round(frames[-1]["theta"] - th0, 1)]
    else:
        mu_curve = sorted((round(f["theta"], 1), round(f["mu"], 2)) for f in frames[::3])
        mu_axis = [0, 360]
    result.update(
        mechanism_type="rocker_rocker" if rr else "crank_rocker",
        lengths={k: mech[k] for k in "abcd"},
        target={"swing": psi, "Q": Q, **({"input_swing": spec["input_swing_deg"]} if rr else {})},
        measured=meas,
        checks=checks,
        notes=notes,
        mu_curve=mu_curve,
        mu_axis=mu_axis,
        fusion=fusion,
    )

    if passed and not strength_ok:
        passed = False
        msg = (f"No {strength.MATERIALS[mat_key]['name']} section carries {F:g} N "
               f"(width ≤ {strength.MAX_WIDTH} mm). Suggestion: a stronger material or a smaller load.")
    elif passed and not layers_ok:
        passed = False
        msg = ("The mechanism collides with itself: a shaft through a middle layer is in the way of the "
               "turning links. Suggestion: a two-sided ground link (top and bottom) or different dimensions.")
    elif passed:
        msg = "All checks passed."
        if "strength" in result:
            parts = ", ".join(f"{n} {s['width']:g}×{s['thickness']:g}"
                              for n, s in section["links"].items())
            msg += (f" Sections (mm): {parts}; pin Ø{section['hole']:g} mm, "
                    f"shaft Ø{section.get('shaft', section['hole']):g} mm (steel). "
                    f"Max drive torque {result['strength']['input_torque_max_Nmm'] / 1000:.2f} N·m.")
    elif rr:
        msg = (f"For input {spec['input_swing_deg']:g}° → output {psi:g}° the accuracy (≤ 1°) or the "
               f"transmission angle ({kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°) cannot be met. Suggestion: bring "
               f"the input and output angles closer or reduce the output angle.")
    else:
        msg = (f"With a {psi:g}° swing" + (f" and Q = {Q:g}" if Q > 1 else "") +
               f" the transmission angle limits ({kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°) cannot be met.")
        if Q > 1:
            q_best = max_feasible_q(a, psi)
            msg += (f" Suggestion: Q of at most {q_best:g}." if q_best
                    else " Suggestion: drop the quick return (Q = 1).")
        else:
            best = max_feasible_swing(a, psi)
            if best:
                msg += f" Suggestion: a swing of at most {best}°."
                result["suggestion_swing"] = best
    result.update(status="ok" if passed else "failed",
                  message=" ".join(notes + [msg]))
    return result
