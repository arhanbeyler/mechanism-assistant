"""
Design flow (prints nothing, only returns data):
    sentence -> LLM (JSON) -> synthesis / sizing -> independent simulation -> checks

Used by both the terminal (main.py) and the Fusion panel.
Standard library only: it also runs in Fusion's own Python.
"""

import math
import re

import bevel
import bolted
import cam_system
import cams
import dynamics
import clutch
import chain
import coupling
import disc_spring
import dwell
import extension_spring
import coupler
import fits
import flywheel
import flexure
import function_gen
import geneva
import gears
import helical
import interference
import kinematics as kin
import shafts
import sixbar
import slotted
import slider_crank
import spline
import din5480
import snapfit
import springs
import toggle
import torsion_spring
import welded
import whitworth
import worm
import llm_extract
import planetary
import pathsynth
import power_screw
import press
import rack_pinion
import ratchet
import reducer
import reducer2
import strength
import vbelt
from llm_extract import extract_spec


# C0: each module defines its own sliders; the panel builds them from here.
# (The four-bar has its own sliders in the panel: coloured swing band, input motion, etc.)
MODULES = {
    "dwell": {
        "title": "Dwell mechanism (six-bar)",
        "params": [
            {"key": "crank_length_mm", "label": "Crank", "unit": "mm", "min": 10, "max": 100, "step": 1, "default": 30},
            {"key": "dwell_deg", "label": "Dwell (crank °)", "unit": "°", "min": 30, "max": 150, "step": 5, "default": 90},
            {"key": "output_swing_deg", "label": "Output swing", "unit": "°", "min": 10, "max": 90, "step": 5, "default": 40},
        ],
    },
    "spline": {
        "title": "Splined shaft (ISO 14 / DIN 5480)",
        "params": [
            {"key": "torque_Nm", "label": "Torque", "unit": "N·m", "min": 5, "max": 3000, "step": 5, "default": 100},
            {"key": "spline_profile", "label": "Profile", "default": "straight",
             "options": [["straight", "Straight-sided (ISO 14)"], ["involute", "Involute (DIN 5480)"]]},
            {"key": "spline_use", "label": "Hub", "default": "fixed",
             "options": [["fixed", "Fixed"], ["sliding", "Slides when unloaded"], ["sliding_loaded", "Slides under load"]]},
        ],
    },
    "extension_spring": {
        "title": "Extension spring",
        "params": [
            {"key": "spring_force_N", "label": "Force", "unit": "N", "min": 1, "max": 500, "step": 1, "default": 50},
            {"key": "spring_deflection_mm", "label": "Extension", "unit": "mm", "min": 2, "max": 200, "step": 1, "default": 20},
        ],
    },
    "disc_spring": {
        "title": "Disc spring stack",
        "params": [
            {"key": "spring_force_N", "label": "Force", "unit": "N", "min": 100, "max": 100000, "step": 100, "default": 5000},
            {"key": "spring_deflection_mm", "label": "Deflection", "unit": "mm", "min": 0.2, "max": 30, "step": 0.1, "default": 2},
            {"key": "arbor_mm", "label": "Guide rod Ø", "unit": "mm", "min": 0, "max": 60, "step": 1, "default": 0,
             "zero_label": "none"},
        ],
    },
    "coupling": {
        "title": "Flange coupling",
        "params": [
            {"key": "torque_Nm", "label": "Torque", "unit": "N·m", "min": 5, "max": 5000, "step": 5, "default": 100},
            {"key": "shaft_diameter_mm", "label": "Shaft Ø", "unit": "mm", "min": 0, "max": 100, "step": 1, "default": 0,
             "zero_label": "sized"},
            {"key": "bolts", "label": "Bolts", "unit": "", "min": 0, "max": 12, "step": 1, "default": 0, "zero_label": "4 / 6"},
        ],
    },
    "press": {
        "title": "Crank press (system)",
        "params": [
            {"key": "force_N", "label": "Press force", "unit": "N", "min": 1000, "max": 200000, "step": 1000, "default": 20000},
            {"key": "stroke_mm", "label": "Stroke", "unit": "mm", "min": 10, "max": 200, "step": 5, "default": 60},
            {"key": "working_stroke_mm", "label": "Working part", "unit": "mm", "min": 0, "max": 50, "step": 1, "default": 6,
             "zero_label": "10 %"},
            {"key": "speed_rpm", "label": "Strokes / min", "unit": "", "min": 5, "max": 200, "step": 5, "default": 40},
        ],
    },
    "flexure": {
        "title": "Living hinge (printed)",
        "params": [
            {"key": "spring_angle_deg", "label": "Bend angle", "unit": "°", "min": 5, "max": 180, "step": 5, "default": 90},
            {"key": "material", "label": "Material", "default": "PETG", "options": [["PETG", "PETG"], ["PLA", "PLA"]]},
            {"key": "repeated", "label": "Use", "default": "no", "options": [["no", "A few times"], ["yes", "Many cycles"]]},
        ],
    },
    "snap_fit": {
        "title": "Snap-fit (printed)",
        "params": [
            {"key": "undercut_mm", "label": "Undercut", "unit": "mm", "min": 0.3, "max": 5, "step": 0.1, "default": 1.0},
            {"key": "material", "label": "Material", "default": "PLA", "options": [["PLA", "PLA"], ["PETG", "PETG"]]},
            {"key": "hold_force_N", "label": "Hold", "unit": "N", "min": 0, "max": 200, "step": 1, "default": 0,
             "zero_label": "any"},
            {"key": "releasable", "label": "Type", "default": "no", "options": [["no", "Permanent"], ["yes", "Releasable"]]},
        ],
    },
    "flywheel": {
        "title": "Flywheel",
        "params": [
            {"key": "energy_J", "label": "Energy fluctuation", "unit": "J", "min": 10, "max": 20000, "step": 10, "default": 500},
            {"key": "speed_rpm", "label": "Mean speed", "unit": "rpm", "min": 50, "max": 3000, "step": 10, "default": 300},
            {"key": "speed_fluctuation", "label": "C_s", "unit": "", "min": 0.005, "max": 0.3, "step": 0.005, "default": 0.05},
            {"key": "outer_diameter_max_mm", "label": "Max diameter", "unit": "mm", "min": 0, "max": 2000, "step": 10,
             "default": 0, "zero_label": "free"},
        ],
    },
    "clutch": {
        "title": "Disc clutch",
        "params": [
            {"key": "torque_Nm", "label": "Torque", "unit": "N·m", "min": 1, "max": 2000, "step": 1, "default": 100},
            {"key": "lining", "label": "Lining", "default": "molded",
             "options": [["molded", "Molded"], ["woven", "Woven"], ["sintered", "Sintered metal"]]},
            {"key": "outer_diameter_max_mm", "label": "Max diameter", "unit": "mm", "min": 0, "max": 600, "step": 5,
             "default": 0, "zero_label": "free"},
        ],
    },
    "torsion_spring": {
        "title": "Torsion spring",
        "params": [
            {"key": "torque_Nm", "label": "Moment", "unit": "N·m", "min": 0.05, "max": 20, "step": 0.05, "default": 0.5},
            {"key": "spring_angle_deg", "label": "At angle", "unit": "°", "min": 10, "max": 720, "step": 5, "default": 90},
            {"key": "arbor_mm", "label": "Arbor Ø", "unit": "mm", "min": 0, "max": 50, "step": 1, "default": 0,
             "zero_label": "none"},
        ],
    },
    "press_fit": {
        "title": "Press fit (ISO fits)",
        "params": [
            {"key": "torque_Nm", "label": "Torque", "unit": "N·m", "min": 1, "max": 5000, "step": 1, "default": 200},
            {"key": "shaft_diameter_mm", "label": "Shaft Ø", "unit": "mm", "min": 0, "max": 120, "step": 1, "default": 40,
             "zero_label": "sized"},
            {"key": "hub_diameter_mm", "label": "Hub Ø", "unit": "mm", "min": 0, "max": 300, "step": 5, "default": 0,
             "zero_label": "2 d"},
            {"key": "hub_length_mm", "label": "Hub length", "unit": "mm", "min": 0, "max": 250, "step": 5, "default": 0,
             "zero_label": "sized"},
        ],
    },
    "welded_joint": {
        "title": "Welded bracket",
        "params": [
            {"key": "force_N", "label": "Load", "unit": "N", "min": 100, "max": 100000, "step": 100, "default": 5000},
            {"key": "arm_mm", "label": "Arm", "unit": "mm", "min": 10, "max": 1000, "step": 5, "default": 100},
            {"key": "bar_height_mm", "label": "Bar height", "unit": "mm", "min": 0, "max": 200, "step": 5, "default": 0,
             "zero_label": "sized"},
            {"key": "bar_width_mm", "label": "Bar width", "unit": "mm", "min": 0, "max": 100, "step": 5, "default": 0,
             "zero_label": "sized"},
            {"key": "weld_pattern", "label": "Weld", "default": "all_around",
             "options": [["all_around", "All around"], ["top_bottom", "Top and bottom"], ["sides", "Both sides"]]},
        ],
    },
    "bolted_joint": {
        "title": "Bolted joint",
        "params": [
            {"key": "force_N", "label": "Separating load", "unit": "N", "min": 1000, "max": 500000, "step": 1000, "default": 50000},
            {"key": "grip_mm", "label": "Clamped thickness", "unit": "mm", "min": 5, "max": 150, "step": 1, "default": 20},
            {"key": "bolts", "label": "Bolts", "unit": "", "min": 0, "max": 20, "step": 1, "default": 0, "zero_label": "4 first"},
            {"key": "member_material", "label": "Plates", "default": "steel",
             "options": [["steel", "Steel"], ["aluminum", "Aluminium"], ["cast_iron", "Cast iron"]]},
            {"key": "bolt_class", "label": "Class", "default": "8.8", "options": [["8.8", "8.8"], ["10.9", "10.9"], ["12.9", "12.9"]]},
            {"key": "fluctuating", "label": "Load", "default": "no",
             "options": [["no", "Static"], ["yes", "Fluctuating 0..P"]]},
        ],
    },
    "bevel_gear": {
        "title": "Bevel gear pair",
        "params": [
            {"key": "gear_ratio", "label": "Ratio", "unit": "", "min": 1, "max": 6, "step": 0.1, "default": 2},
            {"key": "torque_Nm", "label": "Input torque", "unit": "N·m", "min": 0.5, "max": 300, "step": 0.5, "default": 10},
            {"key": "speed_rpm", "label": "Input speed", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "worm_gear": {
        "title": "Worm gear",
        "params": [
            {"key": "gear_ratio", "label": "Ratio", "unit": "", "min": 5, "max": 100, "step": 1, "default": 30},
            {"key": "output_torque_Nm", "label": "Output torque", "unit": "N·m", "min": 1, "max": 2000, "step": 1, "default": 50},
            {"key": "speed_rpm", "label": "Worm speed", "unit": "rpm", "min": 50, "max": 3000, "step": 50, "default": 1400},
            {"key": "self_locking", "label": "Self-locking", "default": "no",
             "options": [["no", "Not needed"], ["yes", "Required"]]},
        ],
    },
    "helical_gear": {
        "title": "Helical gear pair",
        "params": [
            {"key": "gear_ratio", "label": "Ratio", "unit": "", "min": 1, "max": 8, "step": 0.1, "default": 3},
            {"key": "torque_Nm", "label": "Input torque", "unit": "N·m", "min": 0.5, "max": 500, "step": 0.5, "default": 10},
            {"key": "helix_deg", "label": "Helix angle", "unit": "°", "min": 5, "max": 45, "step": 1, "default": 20},
            {"key": "speed_rpm", "label": "Input speed", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "not given"},
        ],
    },
    "planetary": {
        "title": "Planetary gear set",
        "params": [
            {"key": "gear_ratio", "label": "Ratio", "unit": "", "min": 3, "max": 12, "step": 0.1, "default": 5},
            {"key": "torque_Nm", "label": "Input torque", "unit": "N·m", "min": 0.5, "max": 500, "step": 0.5, "default": 10},
            {"key": "speed_rpm", "label": "Input speed", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "not given"},
            {"key": "planets", "label": "Planets", "default": "auto",
             "options": [["auto", "Lightest"], [3, "3"], [4, "4"], [5, "5"]]},
        ],
    },
    "chain_drive": {
        "title": "Roller chain drive",
        "params": [
            {"key": "power_kW", "label": "Power", "unit": "kW", "min": 0.1, "max": 50, "step": 0.1, "default": 1.5},
            {"key": "speed_rpm", "label": "Input speed", "unit": "rpm", "min": 10, "max": 3000, "step": 10, "default": 1400},
            {"key": "gear_ratio", "label": "Speed ratio", "unit": "", "min": 0.2, "max": 6, "step": 0.1, "default": 2.5},
            {"key": "shock", "label": "Load", "default": "moderate",
             "options": [["smooth", "Smooth"], ["moderate", "Moderate shock"], ["heavy", "Heavy shock"]]},
            {"key": "center_distance_mm", "label": "Centre distance", "unit": "mm", "min": 0, "max": 2000, "step": 10,
             "default": 0, "zero_label": "auto"},
        ],
    },
    "belt_drive": {
        "title": "V-belt drive",
        "params": [
            {"key": "power_kW", "label": "Power", "unit": "kW", "min": 0.1, "max": 50, "step": 0.1, "default": 1.5},
            {"key": "speed_rpm", "label": "Input speed", "unit": "rpm", "min": 10, "max": 3000, "step": 10, "default": 1400},
            {"key": "gear_ratio", "label": "Speed ratio", "unit": "", "min": 0.2, "max": 6, "step": 0.1, "default": 2.5},
            {"key": "shock", "label": "Load", "default": "moderate",
             "options": [["smooth", "Smooth"], ["moderate", "Moderate shock"], ["heavy", "Heavy shock"]]},
            {"key": "center_distance_mm", "label": "Centre distance", "unit": "mm", "min": 0, "max": 2000, "step": 10,
             "default": 0, "zero_label": "auto"},
        ],
    },
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
            {"key": "whitworth", "label": "Arrangement", "default": "no",
             "options": [["no", "Crank + slotted lever"], ["yes", "Whitworth (lever turns fully)"]]},
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
            {"key": "spring_force_min_N", "label": "Cycles down to", "unit": "N", "min": 0, "max": 500, "step": 1,
             "default": 0, "zero_label": "static"},
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
_SPEED_KINDS = ("crank_rocker", "reducer", "spur_gear", "shaft", "slider_crank", "cam", "straight_line", "geneva", "scotch_yoke",
                "quick_return", "power_screw", "rack_pinion", "chain_drive", "belt_drive", "planetary", "helical_gear", "worm_gear", "bevel_gear")


_FORCE = re.compile(r"(\d+(?:[.,]\d+)?)\s*(kN|N)(?![a-zçğıöşü·])", re.IGNORECASE)


def force_backstop(text, spec):
    """Safety net: a force written as 'X kN' / 'X N' is already a force. If the LLM treated it as a mass
    (X * 9.81 or X * 9810), the value is read directly from the sentence. Returns (spec, note or None)."""
    F = spec.get("force_N")
    if not F:
        return spec, None
    for num, unit in _FORCE.findall(text):
        v = float(num.replace(",", "."))
        right = v * (1000.0 if unit.lower() == "kn" else 1.0)
        if abs(F - right) < 1e-6 * right:
            return spec, None
        if any(abs(F - right * k) < 1e-3 * right * k for k in (9.81, 9.80665)):
            return dict(spec, force_N=right), f"Read from the sentence: {num} {unit} is a force ({right:g} N)."
    return spec, None


_OUT_TORQUE = re.compile(r"(\d+(?:[.,]\d+)?)\s*N\s*[·.*]?\s*m\s+(?:output|çıkış)|(?:output|çıkış\w*)\s+(?:torque\s+|torku\s+)?"
                         r"(\d+(?:[.,]\d+)?)\s*N\s*[·.*]?\s*m", re.IGNORECASE)


def output_torque_backstop(text, spec):
    """Safety net for the worm gear: a torque written right next to 'output' / 'çıkış' that the LLM dropped is read
    directly from the sentence (it never fills a missing number itself)."""
    if spec.get("mechanism_type") != "worm_gear" or spec.get("output_torque_Nm") or spec.get("torque_Nm"):
        return spec, None
    m = _OUT_TORQUE.search(text)
    if not m:
        return spec, None
    v = float((m.group(1) or m.group(2)).replace(",", "."))
    return dict(spec, output_torque_Nm=v), f"Read from the sentence: {v:g} N·m output torque."


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
    spec, f_note = force_backstop(text, spec)
    spec, t_note = output_torque_backstop(text, spec)
    note = " ".join(n for n in (note, f_note, t_note) if n) or None
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
                       force_N=None, travel_mm=None, screw_ends=None, nut_material=None, shock=None, planets=None, helix_deg=None, output_torque_Nm=None,
                       self_locking=None, grip_mm=None, bolts=None, member_material=None, bolt_class=None,
                       fluctuating=None, arm_mm=None, bar_width_mm=None, bar_height_mm=None, weld_pattern=None,
                       shaft_diameter_mm=None, hub_diameter_mm=None, hub_length_mm=None, spring_angle_deg=None,
                       arbor_mm=None, spring_force_min_N=None, energy_J=None, speed_fluctuation=None, lining=None,
                       undercut_mm=None, hold_force_N=None, releasable=None, repeated=None,
                       working_stroke_mm=None, path_points=None, spline_use=None, spline_profile=None, whitworth=None, preview=False,
                       dwell_deg=None):
    """For the sliders: design straight from numbers, no LLM.
    kind: "fourbar", "reducer", "spur_gear", "spring", "cam" or "shaft" (C0: module choice).
    input_swing_deg >= 360: full-turn input (crank-rocker); less: limited-angle input arm (B9)."""
    if kind == "dwell":
        return design_from_spec({"mechanism_type": "dwell", "crank_length_mm": crank_length_mm, "dwell_deg": dwell_deg,
                                 "output_swing_deg": output_swing_deg, "_preview": bool(preview)})
    if kind == "spline":
        return design_from_spec({"mechanism_type": "spline", "torque_Nm": torque_Nm, "spline_use": spline_use,
                                 "spline_profile": spline_profile})
    if kind == "extension_spring":
        return design_from_spec({"mechanism_type": "extension_spring", "spring_force_N": spring_force_N,
                                 "spring_deflection_mm": spring_deflection_mm})
    if kind == "disc_spring":
        return design_from_spec({"mechanism_type": "disc_spring", "spring_force_N": spring_force_N,
                                 "spring_deflection_mm": spring_deflection_mm, "arbor_mm": arbor_mm or None})
    if kind == "coupling":
        return design_from_spec({"mechanism_type": "coupling", "torque_Nm": torque_Nm,
                                 "shaft_diameter_mm": shaft_diameter_mm or None, "bolts": bolts or None})
    if kind == "path_synthesis":
        return design_from_spec({"mechanism_type": "path_synthesis", "path_points": path_points})
    if kind == "press":
        return design_from_spec({"mechanism_type": "press", "force_N": force_N, "stroke_mm": stroke_mm,
                                 "working_stroke_mm": working_stroke_mm or None, "speed_rpm": speed_rpm})
    if kind == "flexure":
        return design_from_spec({"mechanism_type": "flexure", "spring_angle_deg": spring_angle_deg, "material": material,
                                 "repeated": repeated in (True, "yes")})
    if kind == "snap_fit":
        return design_from_spec({"mechanism_type": "snap_fit", "undercut_mm": undercut_mm, "material": material,
                                 "hold_force_N": hold_force_N or None, "releasable": releasable in (True, "yes")})
    if kind == "flywheel":
        return design_from_spec({"mechanism_type": "flywheel", "energy_J": energy_J, "speed_rpm": speed_rpm,
                                 "speed_fluctuation": speed_fluctuation, "outer_diameter_max_mm": outer_diameter_max_mm or None})
    if kind == "clutch":
        return design_from_spec({"mechanism_type": "clutch", "torque_Nm": torque_Nm, "lining": lining,
                                 "outer_diameter_max_mm": outer_diameter_max_mm or None})
    if kind == "torsion_spring":
        return design_from_spec({"mechanism_type": "torsion_spring", "torque_Nm": torque_Nm,
                                 "spring_angle_deg": spring_angle_deg, "arbor_mm": arbor_mm or None})
    if kind == "press_fit":
        return design_from_spec({"mechanism_type": "press_fit", "torque_Nm": torque_Nm,
                                 "shaft_diameter_mm": shaft_diameter_mm or None, "hub_diameter_mm": hub_diameter_mm or None,
                                 "hub_length_mm": hub_length_mm or None})
    if kind == "welded_joint":
        return design_from_spec({"mechanism_type": "welded_joint", "force_N": force_N, "arm_mm": arm_mm,
                                 "bar_width_mm": bar_width_mm or None, "bar_height_mm": bar_height_mm or None,
                                 "weld_pattern": weld_pattern})
    if kind == "bolted_joint":
        return design_from_spec({"mechanism_type": "bolted_joint", "force_N": force_N, "grip_mm": grip_mm,
                                 "bolts": bolts or None, "member_material": member_material, "bolt_class": bolt_class,
                                 "fluctuating": fluctuating in (True, "yes")})
    if kind == "bevel_gear":
        return design_from_spec({"mechanism_type": "bevel_gear", "gear_ratio": gear_ratio, "torque_Nm": torque_Nm,
                                 "speed_rpm": speed_rpm or None, "material": material})
    if kind == "worm_gear":
        return design_from_spec({"mechanism_type": "worm_gear", "gear_ratio": gear_ratio, "speed_rpm": speed_rpm,
                                 "output_torque_Nm": output_torque_Nm, "torque_Nm": torque_Nm, "power_kW": power_kW,
                                 "self_locking": self_locking in (True, "yes")})
    if kind == "helical_gear":
        return design_from_spec({"mechanism_type": "helical_gear", "gear_ratio": gear_ratio, "torque_Nm": torque_Nm,
                                 "helix_deg": helix_deg, "speed_rpm": speed_rpm or None, "material": material})
    if kind == "planetary":
        return design_from_spec({"mechanism_type": "planetary", "gear_ratio": gear_ratio, "torque_Nm": torque_Nm,
                                 "power_kW": power_kW, "speed_rpm": speed_rpm or None, "material": material,
                                 "planets": int(planets) if str(planets) in ("3", "4", "5") else None})
    if kind in ("chain_drive", "belt_drive"):
        return design_from_spec({"mechanism_type": kind, "power_kW": power_kW, "torque_Nm": torque_Nm,
                                 "speed_rpm": speed_rpm, "gear_ratio": gear_ratio, "shock": shock,
                                 "center_distance_mm": center_distance_mm or None})
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
                                 "material": material, "whitworth": whitworth in (True, "yes")})
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
                                 "show_plates": show_plates, "spring_force_min_N": spring_force_min_N or None})
    if kind == "spur_gear":
        return design_from_spec({"mechanism_type": "spur_gear", "gear_ratio": gear_ratio,
                                 "torque_Nm": torque_Nm, "speed_rpm": speed_rpm or None, "material": material})
    rr = input_swing_deg < 360
    return design_from_spec({
        "mechanism_type": "rocker_rocker" if rr else "crank_rocker", "crank_length_mm": crank_length_mm,
        "output_swing_deg": output_swing_deg, "quick_return": (not rr) and time_ratio > 1.0,
        "time_ratio": time_ratio if (not rr and time_ratio > 1.0) else None,
        "input_swing_deg": input_swing_deg if rr else None,
        "max_load_N": max_load_N or None, "material": material, "speed_rpm": speed_rpm or None,
        "_preview": bool(preview)})


# numeric fields that may be zero (0 is meaningful) and the one that may also be negative
_ZERO_OK = {"offset_mm", "bending_moment_Nm", "spring_force_min_N", "radial_force_N", "dwell_top_deg", "speed_rpm"}
_NEGATIVE_OK = {"offset_mm"}


def _sanitize(spec):
    """A zero or negative value where it has no meaning (a force, a length, a ratio) is dropped, so the module asks
    for it instead of computing with it; non-finite numbers too. Returns (clean spec, note or None)."""
    clean, dropped = dict(spec), []
    for k, v in spec.items():
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            continue
        if not math.isfinite(v) or (v < 0 and k not in _NEGATIVE_OK) or (v == 0 and k not in _ZERO_OK | _NEGATIVE_OK):
            clean[k] = None
            dropped.append(f"{_field_label(k)} = {v:g}")
    return clean, ("Ignored (zero or negative where that has no meaning): " + ", ".join(dropped) + ".") if dropped else None


def design_from_spec(spec, on_status=None):
    """Design from a structured request (LLM output or sliders). Inputs are sanitised first; a numeric failure
    inside a module (values far outside what its method covers) comes back as a failed design with the reason,
    never as an exception."""
    spec, note = _sanitize(spec)
    try:
        result = _design_from_spec(spec, on_status)
    except (ValueError, ZeroDivisionError, OverflowError, TypeError, StopIteration, IndexError, KeyError) as e:
        result = {"spec": spec, "mechanism_type": spec.get("mechanism_type"), "status": "failed",
                  "message": f"These values are outside what the {spec.get('mechanism_type', '').replace('_', ' ')} "
                             f"method covers ({type(e).__name__}: {e}). Please check the numbers."}
    if note:
        result["message"] = note + " " + result.get("message", "")
    return result


def _design_from_spec(spec, on_status=None):
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
    if mtype in ("chain_drive", "belt_drive"):
        return _flex_drive(result, spec, say)
    if mtype == "planetary":
        return _planetary(result, spec, say)
    if mtype == "helical_gear":
        return _helical_gear(result, spec, say)
    if mtype == "worm_gear":
        return _worm_gear(result, spec, say)
    if mtype == "bevel_gear":
        return _bevel_gear(result, spec, say)
    if mtype == "bolted_joint":
        return _bolted_joint(result, spec, say)
    if mtype == "welded_joint":
        return _welded_joint(result, spec, say)
    if mtype == "press_fit":
        return _press_fit(result, spec, say)
    if mtype == "torsion_spring":
        return _torsion_spring(result, spec, say)
    if mtype == "flywheel":
        return _flywheel(result, spec, say)
    if mtype == "snap_fit":
        return _snap_fit(result, spec, say)
    if mtype == "flexure":
        return _flexure(result, spec, say)
    if mtype == "press":
        return _press(result, spec, say)
    if mtype == "path_synthesis":
        return _path_synthesis(result, spec, say)
    if mtype == "coupling":
        return _coupling(result, spec, say)
    if mtype == "disc_spring":
        return _disc_spring(result, spec, say)
    if mtype == "extension_spring":
        return _extension_spring(result, spec, say)
    if mtype == "spline":
        return _spline(result, spec, say)
    if mtype == "dwell":
        return _dwell(result, spec, say)
    if mtype == "clutch":
        return _clutch(result, spec, say)
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
                            "linkage, Geneva, toggle clamp, slider-crank, Scotch yoke, quick return, ratchet, power screw, rack and pinion, chain and V-belt drives, planetary gear sets, helical, bevel and worm gears, bolted and welded joints, press fits, torsion springs, flywheels, disc clutches, printed snap-fits and living hinges, crank presses, path synthesis through given points, flange couplings, disc and extension springs, splines, dwell mechanisms, reducers (one or two stages), spur gears, compression springs, "
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
    notes = []
    if not rr and 180 <= psi <= 300 and not (spec.get("time_ratio") or 1) > 1:
        six = _six_bar(result, spec, notes, say, a, psi)          # F4b: beyond any four-bar's swing
        if six:
            return six
    if not 0 < psi < 180:
        return dict(result, status="missing",
                    message=f"The swing angle must be between 0 and 180° (given: {psi:g}°)"
                            + (" and no six-bar reaches it either." if not rr and psi <= 300 else "."))
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
    if spec.get("whitworth") in (True, "yes"):
        return _whitworth(result, spec, say, S, Q, F, rpm, mat, notes)
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


def _whitworth(result, spec, say, S, Q, F, rpm, mat, notes):
    """F4c: Whitworth quick return (the crank longer than the pivot distance: the lever turns full circles)."""
    result["mechanism_type"] = "whitworth"
    notes.append(f"Whitworth proportions: rod arm R = S/2, rod {whitworth.ROD:g} R, crank {whitworth.CRANK:g} R (reported).")
    say("Whitworth: crank, full-turn slotted lever, rod and ram; simulated...")
    d, s_checks = whitworth.design(S, Q, mat, F)
    m = d["measure"]
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": c["detail"] + (f" · use {c['util'] * 100:.0f}%" if c["group"] != "Motion" else "")} for c in s_checks]
    ok = all(c["ok"] for c in checks) and (not F or d["section"] is not None)
    section = d["section"] or {"hole": 5.0, "shaft": 8.0, "links": {"Crank": {"width": 12.0, "thickness": 5.0},
                                                                   "Lever": {"width": 16.0, "thickness": 5.0}}}
    speeds = None
    if rpm:
        t_turn = 60.0 / rpm
        slow = max(m["to_minus"], 360 - m["to_minus"])
        speeds = {"rpm": float(rpm), "v_cut": S / (t_turn * slow / 360) / 1000, "v_return": S / (t_turn * (360 - slow) / 360) / 1000}
        checks.append({"name": "Mean speeds", "ok": True,
                       "detail": f"cutting {speeds['v_cut']:.2f} m/s, return {speeds['v_return']:.2f} m/s at {rpm:g} rpm"})
    x0 = min(f["x"] for f in d["frames"])
    curve = [(round(f["theta"], 1), round(f["x"] - x0, 3)) for f in d["frames"][::8]]
    result.update(whitworth={k: d[k] for k in ("r", "d", "R", "l", "alpha", "Q", "stroke", "slow_direction")} |
                  {"Q_meas": m["Q"], "stroke_meas": m["stroke"], "rod_angle_max": m["rod_angle_max"], "speeds": speeds,
                   "T_max": d.get("T_max"), "rho_min": m["rho_min"], "rho_max": m["rho_max"]},
                  checks=checks, notes=notes, section_mm=section, x_curve=curve, target={"stroke": S, "Q": Q},
                  fusion={"type": "whitworth", "passed_checks": ok, "r": d["r"], "d": d["d"], "R": d["R"], "l": d["l"],
                          "section_mm": section})
    if not ok:
        msg = "The design does not meet the checks above."
    else:
        msg = (f"All checks passed. Whitworth: crank {d['r']:.1f} mm turning about a point {d['d']:.1f} mm from the lever "
               f"pivot, so the slotted lever turns full circles; its {d['R']:.1f} mm arm drives the ram through a "
               f"{d['l']:.0f} mm rod: stroke {m['stroke']:.1f} mm, time ratio {m['Q']:.2f}.")
        if F:
            msg += f" Max crank torque {d['T_max'] / 1000:.2f} N·m."
    result.update(status="ok" if ok else "failed", message=" ".join(notes + [msg]))
    return result


_DWELL_CACHE = {}


def _dwell(result, spec, say):
    """F4d: single-dwell six-bar. Required: the crank, the dwell (crank degrees) and the output swing."""
    result["mechanism_type"] = "dwell"
    a, dw, sw = spec.get("crank_length_mm"), spec.get("dwell_deg"), spec.get("output_swing_deg")
    missing = [n for v, n in ((a, "crank length"), (dw, "dwell (degrees of the crank turn)"), (sw, "output swing (°)"))
               if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the dwell mechanism: {', '.join(missing)}.")
    a, dw, sw = float(a), float(dw), float(sw)
    if not (30 <= dw <= 150 and 10 <= sw <= 90):
        return dict(result, status="missing", message="Dwell 30–150° of the crank and a swing of 10–90° are covered.")
    notes = ["Default sections (no load given); the coupler point lies on the coupler's line."]
    key = (round(a, 3), round(dw, 3), round(sw, 3))
    if key not in _DWELL_CACHE:
        if spec.get("_preview"):
            result["six_bar_pending"] = True
            return dict(result, status="failed", notes=notes, checks=[],
                        message="Press Draw in Fusion to synthesize the dwell mechanism (about half a minute).")
        say("Dwell six-bar: searching a coupler curve with a circular arc, then the output dyad...")
        _DWELL_CACHE[key] = dwell.design(a, dw, sw)
    m, frames = _DWELL_CACHE[key]
    if m is None:
        return dict(result, status="failed", message="No dwell linkage found for these values.")
    me = m["measured"]
    checks = [{"name": "Dwell (simulation)", "ok": me["dwell_motion"] <= 1.0,
               "detail": f"the output moves {me['dwell_motion']:.2f}° while the crank turns {dw:g}°"},
              {"name": "Output swing (simulation)", "ok": abs(me["swing"] - sw) <= 2.0, "detail": f"{me['swing']:.1f}° (target {sw:g}°)"},
              {"name": "Transmission angle, four-bar", "ok": me["mu1_min"] >= dwell.MU_MIN and me["mu1_max"] <= 180 - dwell.MU_MIN,
               "detail": f"{me['mu1_min']:.1f}° – {me['mu1_max']:.1f}°"},
              {"name": "Transmission angle, output dyad", "ok": me["mu6_min"] >= dwell.MU_MIN and me["mu6_max"] <= 180 - dwell.MU_MIN,
               "detail": f"{me['mu6_min']:.1f}° – {me['mu6_max']:.1f}°"}]
    ok = all(c["ok"] for c in checks)
    section = strength.section_with_bosses(strength.DEFAULT_SECTION)
    f0 = frames[0]
    r3 = lambda p: [round(p[0], 3), round(p[1], 3)]
    pts = {"O2": [0.0, 0.0], "O4": [round(m["d"], 3), 0.0], "O6": r3(m["O6"]), "A": r3(f0["A"]), "B": r3(f0["B"]),
           "P": r3(f0["P"]), "D": r3(f0["D"])}
    order = sorted((("A", 0.0), ("B", m["b"]), ("P", m["p"])), key=lambda t: t[1])
    info = {k: round(m[k], 2) for k in ("a", "b", "c", "d", "p", "r5", "c6")}
    info.update(dwell=dw, swing=me["swing"], dwell_motion=me["dwell_motion"], mu1=[me["mu1_min"], me["mu1_max"]],
                mu6=[me["mu6_min"], me["mu6_max"]], points=pts,
                phi_curve=[(round(f["theta"], 1), round(f["phi"], 2)) for f in frames[::4]])
    result.update(dwell=info, checks=checks, notes=notes, section_mm=section, target={"dwell": dw, "swing": sw},
                  fusion={"type": "dwell", "passed_checks": ok, "points_mm": pts, "section_mm": section,
                          "coupler_ends": [order[0][0], order[-1][0]]})
    msg = (f"All checks passed. A four-bar (crank {a:g}, coupler {m['b']:.1f}, rocker {m['c']:.1f}, ground {m['d']:.1f} mm) "
           f"whose coupler point {m['p']:.1f} mm along the coupler runs on a circular arc (radius {m['r5']:.1f} mm) for "
           f"{dw:g}° of the crank; a {m['r5']:.1f} mm link from that point to the arc's centre drives the {m['c6']:.1f} mm "
           f"output: it stands still (±{me['dwell_motion'] / 2:.2f}°) during the arc and swings {me['swing']:.1f}° otherwise."
           if ok else "The best dwell linkage found does not meet the checks above: a longer dwell or a smaller swing is easier.")
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
    if d is None:
        return dict(result, status="failed", notes=notes, checks=[{"name": c["name"], "ok": False, "detail": c["detail"]}
                                                                  for c in s_checks],
                    message=" ".join(notes + [s_checks[0]["detail"] + ": a larger diameter or fewer teeth."]))
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


def _flex_drive(result, spec, say):
    """G3: roller chain or V-belt drive. Required: input speed, power (or torque) and the speed ratio;
    the load's shock class defaults to moderate (reported), the centre distance to the engine's default."""
    kind = spec["mechanism_type"]
    result["mechanism_type"] = kind
    what = "chain drive" if kind == "chain_drive" else "V-belt drive"
    T, P, rpm, ratio = spec.get("torque_Nm"), spec.get("power_kW"), spec.get("speed_rpm"), spec.get("gear_ratio")
    notes = []
    if P is None and T is not None and rpm:
        P = float(T) * float(rpm) / 9550.0
        notes.append(f"Power from the torque: P = T n / 9550 = {P:.2f} kW.")
    missing = [n for v, n in ((rpm, "input speed (rpm)"), (P, "power or torque"), (ratio, "speed ratio"))
               if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the {what}: {', '.join(missing)}.")
    P_W, rpm, ratio = float(P) * 1000.0, float(rpm), float(ratio)
    if not 1 / 6 <= ratio <= 6:
        return dict(result, status="missing", message=f"One {what} covers speed ratios up to 6 (given: {ratio:g}).")
    shock = spec.get("shock")
    table = chain.SERVICE if kind == "chain_drive" else vbelt.SERVICE
    if shock not in table:
        shock = "moderate"
        notes.append(f"Load type not given: moderate shock assumed (service factor {table[shock]:g}).")
    Ks = table[shock]
    C = spec.get("center_distance_mm")
    if kind == "chain_drive":
        say("Chain drive: pitch, strands, sprocket teeth, chain length...")
        d, s_checks = chain.design(P_W, rpm, ratio, Ks, C)
    else:
        say("V-belt drive: section, sheave diameters, belt length, number of belts...")
        d, s_checks = vbelt.design(P_W, rpm, ratio, Ks, C)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if d is None:
        bad = [c["name"] for c in checks if not c["ok"]]
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            f"No {what} passes all checks (failing: {', '.join(bad) or 'no candidate'})."]))
    if not C:
        notes.append(f"Centre distance not given: {d['C']:.0f} mm from the engine's default and the standard length.")
    n_out = rpm / d["ratio"]
    info = dict(d, P=P_W, rpm_in=rpm, rpm_out=n_out, shock=shock)
    fusion = {"type": kind, "passed_checks": True, "C": d["C"], "bore1": d["bore1"], "bore2": d["bore2"],
              "small_is_input": d["small_is_input"]}
    if kind == "chain_drive":
        fusion.update(p=d["p"], Dr=d["Dr"], W=d["W"], pt=d["pt"], strands=d["strands"], N1=d["N1"], N2=d["N2"])
        msg = (f"All checks passed. {'Single' if d['strands'] == 1 else ('Double' if d['strands'] == 2 else 'Triple')}"
               f"-strand chain no. {d['no']} (ISO {d['iso']}{'' if d['strands'] == 1 else '-' + str(d['strands'])}, "
               f"pitch {d['p']:.2f} mm), sprockets {d['N1']}/{d['N2']} teeth (Ø{d['D1']:.0f} / Ø{d['D2']:.0f} mm), "
               f"{d['links']} links, centre distance {d['C']:.1f} mm. Output {n_out:.0f} rpm; chain speed "
               f"{d['V']:.1f} m/s, pull {d['F']:.0f} N.")
    else:
        fusion.update(d=d["d"], D=d["D"], belts=d["belts"], e=d["e"], top=d["top"], thick=d["thick"],
                      width=d["width"])
        msg = (f"All checks passed. {d['belts']} × {d['belt']} V-belt{'s' if d['belts'] > 1 else ''} on sheaves "
               f"Ø{d['d']:.0f} / Ø{d['D']:.0f} mm, centre distance {d['C']:.0f} mm. Output {n_out:.0f} rpm; belt "
               f"speed {d['V']:.1f} m/s, safety {d['n_fs']:.2f}; set the initial tension to {d['Fi']:.0f} N per belt.")
    result.update(drive=info, checks=checks, notes=notes, fusion=fusion, target={"power_W": P_W, "rpm": rpm},
                  status="ok", message=" ".join(notes + [msg]))
    return result


def _planetary(result, spec, say):
    """G2: planetary gear set (sun in, carrier out, ring fixed). Required: ratio and input torque (or power + speed)."""
    result["mechanism_type"] = "planetary"
    T, P, rpm, ratio = spec.get("torque_Nm"), spec.get("power_kW"), spec.get("speed_rpm"), spec.get("gear_ratio")
    notes = []
    if T is None and P is not None and rpm:
        T = 9550.0 * P / rpm
        notes.append(f"Input torque from the power: T = 9550 P / n = {T:.1f} N·m.")
    missing = [n for v, n in ((ratio, "ratio"), (T, "input torque (or power and speed)")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the planetary set: {', '.join(missing)}.")
    ratio = float(ratio)
    if ratio < 1:
        ratio = 1 / ratio
    if not 3 <= ratio <= 12:
        return dict(result, status="missing",
                    message=f"A planetary stage with a fixed ring covers ratios 3–12 (given: {ratio:g}).")
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "steel"
        notes.append("Gear material not given: steel assumed.")
    mat = strength.MATERIALS[mat_key]
    n_pl = spec.get("planets")
    n_pl = int(n_pl) if n_pl in planetary.N_PLANETS else None
    say("Planetary set: teeth for the ratio, assembly and neighbour conditions, module, face width...")
    d, s_checks = planetary.design(ratio, float(T) * 1000.0, mat, rpm, n_pl)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if d is None:
        bad = [c["name"] for c in checks if not c["ok"]]
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            f"No planetary set passes all checks (failing: {', '.join(bad) or 'no tooth combination'})."]))
    notes.append(f"Load sharing between the {d['n']} planets taken as K_γ = {d['K_gamma']:g}; losses neglected.")
    info = dict(d, material=mat["name"], rpm=rpm)
    result.update(planetary=info, checks=checks, notes=notes, target={"ratio": ratio, "torque_Nm": float(T)},
                  fusion={"type": "planetary", "passed_checks": True, "m": d["module"], "Zs": d["Zs"], "Zp": d["Zp"],
                          "Zr": d["Zr"], "n": d["n"], "b": d["face_width"], "pin": d["pin"],
                          "bore_sun": d["bore_sun"], "bore_carrier": d["bore_carrier"], "R_out": d["R_out"]})
    at_rpm = f" at {rpm / d['ratio']:.0f} rpm" if rpm else ""
    msg = (f"All checks passed. Sun {d['Zs']}, {d['n']} planets of {d['Zp']}, ring {d['Zr']} teeth, module {d['module']:g}, "
           f"face {d['face_width']:g} mm: i = {d['ratio']:.3f} (ring Ø{2 * d['R_out']:.0f} mm). Output "
           f"{d['T_out'] / 1000:.1f} N·m{at_rpm}; "
           f"planet pins Ø{d['pin']:g}, carrier shaft Ø{d['bore_carrier']:g}.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _helical_gear(result, spec, say):
    """G1a: helical gear pair. Required: ratio and input torque; the helix angle defaults to 20 deg (reported)."""
    result["mechanism_type"] = "helical_gear"
    missing = [name for key, name in (("gear_ratio", "ratio"), ("torque_Nm", "input torque (N·m)")) if not spec.get(key)]
    if missing:
        return dict(result, status="missing", message=f"Missing for the helical gears: {', '.join(missing)}.")
    ratio, T1 = float(spec["gear_ratio"]), float(spec["torque_Nm"]) * 1000.0
    if ratio < 1:
        ratio = 1 / ratio
    if not 1 <= ratio <= 8:
        return dict(result, status="missing", message=f"A single helical stage needs a ratio of 1–8 (given: {ratio:g}).")
    notes = []
    psi = spec.get("helix_deg")
    if not psi:
        psi = helical.PSI_DEFAULT
        notes.append(f"Helix angle not given: {psi:g}° assumed.")
    psi = float(psi)
    if not 5 <= psi <= 45:
        return dict(result, status="missing", message=f"Helix angles of 5–45° are covered (given: {psi:g}°).")
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "steel"
        notes.append("Material not given: steel assumed.")
    mat = strength.MATERIALS[mat_key]
    rpm = spec.get("speed_rpm")
    say("Helical gears: normal module, virtual teeth, contact ratios...")
    g, g_checks = helical.design(ratio, T1, mat, rpm, psi)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in g_checks]
    result.update(checks=checks, notes=notes)
    if g is None:
        return dict(result, status="failed", message=" ".join(notes + [
            f"No {mat['name']} helical pair carries {T1 / 1000:g} N·m. Suggestion: a stronger material or a smaller torque."]))
    checks.insert(0, {"name": "Ratio", "ok": g["ratio_err_pct"] <= 2.0,
                      "detail": f"{g['z2']}/{g['z1']} = {g['ratio']:.3f} (target {ratio:g})"})
    result.update(helical=dict(g, material=mat["name"], rpm=rpm), target={"ratio": ratio, "torque_Nm": T1 / 1000},
                  fusion={"type": "helical_gear", "passed_checks": True, **g}, status="ok",
                  message=" ".join(notes + [
                      f"All checks passed. Normal module {g['module']:g}, helix {psi:g}° (pinion right hand, gear left), "
                      f"z = {g['z1']}/{g['z2']}, face {g['face_width']:g} mm, centre distance {g['center_distance']:.2f} mm, "
                      f"shafts Ø{g['bore1']:g}/Ø{g['bore2']:g} mm. Axial thrust {g['Wa']:.0f} N on the bearings; output "
                      f"{T1 * g['ratio'] / 1000:.2f} N·m."]))
    return result


def _worm_gear(result, spec, say):
    """G1b: worm gear set. Required: ratio, worm speed and a torque (output, or input / power: then the output
    torque follows the efficiency, iterated because the efficiency depends on the design)."""
    result["mechanism_type"] = "worm_gear"
    ratio, rpm = spec.get("gear_ratio"), spec.get("speed_rpm")
    T_out, T_in, P = spec.get("output_torque_Nm"), spec.get("torque_Nm"), spec.get("power_kW")
    notes = []
    if T_in is None and P is not None and rpm:
        T_in = 9550.0 * P / rpm
        notes.append(f"Worm torque from the power: {T_in:.2f} N·m.")
    missing = [n for v, n in ((ratio, "ratio"), (rpm, "worm speed (rpm)"), (T_out or T_in, "a torque (output or input) or power"))
               if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the worm gear: {', '.join(missing)}.")
    ratio, rpm = float(ratio), float(rpm)
    if ratio < 1:
        ratio = 1 / ratio
    if not 5 <= ratio <= 100:
        return dict(result, status="missing", message=f"A worm set covers ratios of 5–100 (given: {ratio:g}).")
    lock = bool(spec.get("self_locking"))
    casting = spec.get("casting") if spec.get("casting") in worm.CASTING else "sand"
    say("Worm gear: starts, module, diameter quotient, AGMA rating, efficiency...")
    if T_out:
        d, s_checks = worm.design(ratio, float(T_out) * 1000.0, rpm, lock, casting)
    else:
        eff = 0.7
        for _ in range(4):                                  # the output torque depends on the efficiency
            d, s_checks = worm.design(ratio, float(T_in) * 1000.0 * ratio * eff, rpm, lock, casting)
            if d is None or abs(d["efficiency"] - eff) < 1e-3:
                break
            eff = d["efficiency"]
        if d:
            notes.append(f"Output torque = input × ratio × efficiency = {d['T_out'] / 1000:.1f} N·m.")
    notes.append(f"Steel worm, {worm.CASTING[casting]} wheel, application factor K_a = {worm.KA:g} (assumed).")
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if d is None:
        why = (" A self-locking worm needs one start and at least 21 wheel teeth, so the ratio must be at least 21."
               if lock and ratio < worm.NG_MIN else "")
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            f"No worm set passes all checks.{why}"]))
    info = dict(d, rpm_in=rpm, self_locking_asked=lock)
    result.update(worm=info, checks=checks, notes=notes, target={"ratio": ratio},
                  fusion={"type": "worm_gear", "passed_checks": True, "m": d["module"], "NW": d["NW"], "NG": d["NG"],
                          "dW": d["dW"], "lead_deg": d["lead_deg"], "bore_worm": d["bore_worm"],
                          "bore_wheel": d["bore_wheel"], "F_G": d["F_G"]})
    lock_txt = ("self-locking (the load cannot turn the worm)" if d["self_locking"]
                else "NOT self-locking (the load can drive the worm back)")
    msg = (f"All checks passed. {d['NW']}-start worm Ø{d['dW']:g} mm (lead angle {d['lead_deg']:.2f}°), wheel {d['NG']} teeth "
           f"(Ø{d['dG']:g} mm), module {d['module']:g}, centre distance {d['C']:.1f} mm; efficiency {100 * d['efficiency']:.0f}%, "
           f"{lock_txt}. Output {d['T_out'] / 1000:.1f} N·m at {d['rpm_out']:.1f} rpm; input {d['P_in']:.0f} W, "
           f"{d['P_loss']:.0f} W becomes heat.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _bevel_gear(result, spec, say):
    """G1c: straight bevel gear pair, shafts at 90 deg. Required: ratio and input torque."""
    result["mechanism_type"] = "bevel_gear"
    missing = [name for key, name in (("gear_ratio", "ratio"), ("torque_Nm", "input torque (N·m)")) if not spec.get(key)]
    if missing:
        return dict(result, status="missing", message=f"Missing for the bevel gears: {', '.join(missing)}.")
    ratio, T1 = float(spec["gear_ratio"]), float(spec["torque_Nm"]) * 1000.0
    if ratio < 1:
        ratio = 1 / ratio
    if not 1 <= ratio <= 6:
        return dict(result, status="missing", message=f"A bevel pair covers ratios of 1–6 (given: {ratio:g}).")
    notes = []
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "steel"
        notes.append("Material not given: steel assumed.")
    mat = strength.MATERIALS[mat_key]
    rpm = spec.get("speed_rpm")
    say("Bevel gears: pitch cones, virtual (Tredgold) teeth, Lewis at the mean section...")
    g, g_checks = bevel.design(ratio, T1, mat, rpm)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in g_checks]
    result.update(checks=checks, notes=notes)
    if g is None:
        return dict(result, status="failed", message=" ".join(notes + [
            f"No {mat['name']} bevel pair carries {T1 / 1000:g} N·m. Suggestion: a stronger material or a smaller torque."]))
    checks.insert(0, {"name": "Ratio", "ok": g["ratio_err_pct"] <= 2.0,
                      "detail": f"{g['z2']}/{g['z1']} = {g['ratio']:.3f} (target {ratio:g})"})
    result.update(bevel=dict(g, material=mat["name"], rpm=rpm), target={"ratio": ratio, "torque_Nm": T1 / 1000},
                  fusion={"type": "bevel_gear", "passed_checks": True, "m": g["module"], "z1": g["z1"], "z2": g["z2"],
                          "F": g["face_width"], "bore1": g["bore1"], "bore2": g["bore2"]},
                  status="ok",
                  message=" ".join(notes + [
                      f"All checks passed. Module {g['module']:g} (large end), z = {g['z1']}/{g['z2']}, pitch cones "
                      f"{g['gamma1_deg']:.1f}° / {g['gamma2_deg']:.1f}°, face {g['face_width']:g} mm, cone distance "
                      f"{g['Ao']:.1f} mm, shafts Ø{g['bore1']:g}/Ø{g['bore2']:g} mm at 90°. Thrust on the pinion "
                      f"{g['Wa1']:.0f} N, on the gear {g['Wa2']:.0f} N; output {T1 * g['ratio'] / 1000:.2f} N·m."]))
    return result


def _bolted_joint(result, spec, say):
    """H1: bolted tension joint. Required: the separating load; grip 20 mm, steel members, class 8.8 and 4 bolts
    are defaults (reported)."""
    result["mechanism_type"] = "bolted_joint"
    P = spec.get("force_N")
    if not P:
        return dict(result, status="missing", message="Missing for the bolted joint: the load pulling the joint apart (N or kg).")
    notes = []
    grip = spec.get("grip_mm")
    if not grip:
        grip = 20.0
        notes.append("Clamped thickness not given: 20 mm (two 10 mm plates) assumed.")
    mem = spec.get("member_material") if spec.get("member_material") in bolted.MEMBERS else None
    if mem is None:
        mem = "steel"
        notes.append("Plate material not given: steel assumed.")
    cls = spec.get("bolt_class") if spec.get("bolt_class") in bolted.CLASSES else None
    if cls is None:
        cls = "8.8"
        notes.append("Bolt class not given: 8.8 assumed.")
    N = spec.get("bolts")
    N = int(N) if N else None
    if N is None:
        notes.append(f"Number of bolts not given: {bolted.N_DEFAULT} tried first.")
    fl = bool(spec.get("fluctuating"))
    say("Bolted joint: stiffnesses, preload, load / separation / fatigue factors...")
    a, s_checks = bolted.design(float(P), float(grip), N, mem, cls, fl)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if a is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            f"No M5–M36 class {cls} bolts pass (failing: {', '.join(c['name'] for c in checks if not c['ok'])})."]))
    info = dict(a, P=float(P), grip=float(grip), fluctuating=fl)
    result.update(bolted=info, checks=checks, notes=notes, target={"force_N": float(P)},
                  fusion={"type": "bolted_joint", "passed_checks": True, "d": a["d"], "N": a["N"], "L": a["L"],
                          "grip": float(grip), "nut_h": a["nut_h"], "af": a["af"], "bolt_circle": a["bolt_circle"],
                          "pitch": a["pitch"]})
    msg = (f"All checks passed. {a['N']} × M{a['d']}×{a['L']} class {cls} bolts with nuts on a Ø{a['bolt_circle']:.0f} mm "
           f"bolt circle; tighten to {a['T'] / 1000:.1f} N·m (preload {a['Fi'] / 1000:.1f} kN each, K = {bolted.K_TORQUE:g}). "
           f"Joint constant C = {a['C']:.3f}: each bolt feels only {100 * a['C']:.0f}% of its {a['P_bolt']:.0f} N share.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _welded_joint(result, spec, say):
    """H2: fillet-welded cantilever bar. Required: the load and its arm; the bar is sized when not given, the
    weld runs all around by default, E70 electrode (both reported)."""
    result["mechanism_type"] = "welded_joint"
    P, L = spec.get("force_N"), spec.get("arm_mm")
    missing = [n for v, n in ((P, "load (N or kg)"), (L, "distance of the load from the weld (arm)")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the welded joint: {', '.join(missing)}.")
    notes = []
    b, d = spec.get("bar_width_mm"), spec.get("bar_height_mm")
    if b and not d:
        d = b
    if d and not b:
        b = d
    pattern = spec.get("weld_pattern") if spec.get("weld_pattern") in welded.PATTERNS else None
    if pattern is None:
        pattern = "all_around"
        notes.append("Weld layout not given: all around the bar.")
    notes.append("E70 electrode, S235 bar and wall (AISC allowables).")
    say("Welded joint: weld group as lines, throat shear, bar bending...")
    a, s_checks = welded.design(float(P), float(L), float(b) if b else None, float(d) if d else None, pattern)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if a is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            f"No fillet up to {welded.LEGS[-1]} mm passes (failing: {', '.join(c['name'] for c in checks if not c['ok'])})."]))
    if not (b and d):
        notes.append(f"Bar not given: {a['b']:g} × {a['d']:g} mm flat bar sized for the bending.")
    result.update(welded=a, checks=checks, notes=notes, target={"force_N": float(P)},
                  fusion={"type": "welded_joint", "passed_checks": True, "b": a["b"], "d": a["d"], "h": a["h"],
                          "L": a["L"], "pattern": pattern})
    msg = (f"All checks passed. {a['h']:g} mm fillet weld ({welded.PATTERNS[pattern]}, {a['weld_length']:.0f} mm long, "
           f"throat {a['throat']:.1f} mm) on a {a['b']:g} × {a['d']:g} mm bar: τ = {a['tau']:.0f} MPa of {a['tau_all']:.0f} allowed; "
           f"the bar sees σ = {a['sigma_bar']:.0f} MPa at the wall.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _press_fit(result, spec, say):
    """H3: press (shrink) fit of a hub on a shaft, ISO 286 H7 fits. Required: the torque; the shaft is sized in
    torsion when not given, the hub 2 d x 1..2 d (reported)."""
    result["mechanism_type"] = "press_fit"
    T = spec.get("torque_Nm")
    if not T:
        return dict(result, status="missing", message="Missing for the press fit: the torque it must transmit (N·m).")
    notes = []
    d = spec.get("shaft_diameter_mm")
    if not d:
        d = chain.shaft_bore(float(T) * 1000.0)
        if d is None:
            return dict(result, status="failed", message="No shaft up to Ø100 carries this torque.")
        notes.append(f"Shaft not given: Ø{d:g} mm from torsion.")
    d = float(d)
    if not 3 < d <= fits.TABLE[-1][0]:
        return dict(result, status="missing", message=f"Fits are tabulated for Ø3–{fits.TABLE[-1][0]} mm (given: {d:g}).")
    D_hub, L = spec.get("hub_diameter_mm"), spec.get("hub_length_mm")
    notes.append(f"Hub and shaft C45 (quenched and tempered), friction μ = {fits.MU:g} (dry, conservative).")
    say("Press fit: H7 fits k6..s6, Lamé pressure, torque at the smallest and stress at the largest interference...")
    a, s_checks = fits.design(float(T) * 1000.0, d, float(D_hub) if D_hub else None, float(L) if L else None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if a is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            f"No standard H7 press fit on Ø{d:g} both carries {float(T):g} N·m at its smallest interference and keeps "
            f"the hub elastic at its largest: use a key or a spline, a larger shaft, or a tighter-toleranced fit."]))
    hl, hh, sl, sh = a["limits"]
    result.update(press_fit=a, checks=checks, notes=notes, target={"torque_Nm": float(T)},
                  fusion={"type": "press_fit", "passed_checks": True, "d": d, "D_hub": a["D_hub"], "L": a["L"]})
    msg = (f"All checks passed. Ø{d:g} H7/{a['fit']} ({a['description']}): hole +{hh / 1000:.3f}/+{hl / 1000:.3f}, shaft "
           f"+{sh / 1000:.3f}/+{sl / 1000:.3f} mm, interference {a['i_min']}–{a['i_max']} µm. Hub Ø{a['D_hub']:g} × {a['L']:g} mm "
           f"holds {a['T_min'] / 1000:.0f} N·m even at the smallest interference; heat the hub by about {a['dT_assembly']:.0f} °C to assemble.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _torsion_spring(result, spec, say):
    """H4: helical torsion spring. Required: the moment and the angle it is reached at."""
    result["mechanism_type"] = "torsion_spring"
    M, th = spec.get("torque_Nm"), spec.get("spring_angle_deg")
    missing = [n for v, n in ((M, "moment (N·m or N·mm)"), (th, "angle at that moment (°)")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the torsion spring: {', '.join(missing)}.")
    M, th = float(M) * 1000.0, float(th)
    if not 10 <= th <= 720:
        return dict(result, status="missing", message=f"Working angles of 10–720° are covered (given: {th:g}°).")
    notes = ["Music wire, straight legs (2 D long unless given), loaded to close the coils."]
    arbor = spec.get("arbor_mm")
    say("Torsion spring: wire, coil diameter, body turns, inner-fibre stress, arbor clearance...")
    s, s_checks = torsion_spring.design(M, th, "music_wire", float(arbor) if arbor else None, spec.get("leg_mm") or None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if s is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            f"No torsion spring found for {M / 1000:g} N·m at {th:g}°."]))
    result.update(torsion=s, checks=checks, notes=notes, target={"M": M, "theta": th},
                  fusion={"type": "torsion_spring", "passed_checks": True, "d": s["d"], "D": s["D"], "Nb": s["Nb"],
                          "legs": s["legs"], "theta_deg": s["theta_deg"], "arbor": s["arbor"]})
    msg = (f"All checks passed. Wire Ø{s['d']:g} mm, coil Ø{s['D']:.1f} mm (OD {s['OD']:.1f}), {s['Nb']:g} body turns, legs "
           f"{s['legs']:.0f} mm: {s['k_deg'] * 1000 / 1000:.1f} N·mm/°, {M / 1000:g} N·m at {s['theta_deg']:.0f}° (asked {th:g}°); "
           f"stress {s['sigma']:.0f} of {s['Sy']:.0f} MPa; the inside diameter closes from {s['D'] - s['d']:.1f} to {s['ID_wound']:.1f} mm.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _flywheel(result, spec, say):
    """H5a: flywheel. Required: speed and the energy fluctuation (or the work of one stroke and the share of
    the cycle it takes); C_s from the application or 0.05 (reported)."""
    result["mechanism_type"] = "flywheel"
    rpm, dE = spec.get("speed_rpm"), spec.get("energy_J")
    notes = []
    work, frac = spec.get("work_J"), spec.get("work_fraction")
    if not dE and work:
        frac = float(frac) if frac else 0.1
        if not spec.get("work_fraction"):
            notes.append("Share of the cycle the work takes not given: 10 % assumed (a punch).")
        dE = flywheel.press_energy(float(work), frac)
        notes.append(f"Energy the flywheel gives up: {float(work):g} J × (1 − {frac:g}) = {dE:.0f} J.")
    missing = [n for v, n in ((rpm, "speed (rpm)"), (dE, "energy fluctuation (J) or the work per stroke")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the flywheel: {', '.join(missing)}.")
    Cs = spec.get("speed_fluctuation")
    app = spec.get("application")
    if not Cs:
        Cs = flywheel.CS_TYPICAL.get(app, 0.05)
        notes.append(f"Speed fluctuation C_s = {Cs:g} ({'typical for ' + app.replace('_', ' ') if app in flywheel.CS_TYPICAL else 'not given'}).")
    say("Flywheel: inertia from the energy fluctuation, disk size, rotating stress...")
    info, s_checks = flywheel.analyse(float(dE), float(rpm), float(Cs), spec.get("outer_diameter_max_mm") or None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    result.update(flywheel=info, checks=checks, notes=notes, target={"dE": float(dE)},
                  fusion={"type": "flywheel", "passed_checks": True, "D": info["D"], "t": info["t"], "bore": info["bore"]})
    if not all(c["ok"] for c in checks):
        return dict(result, status="failed", message=" ".join(notes + [
            f"A Ø{info['D']:.0f} mm disk would have to be {info['t']:.0f} mm thick: allow a larger diameter or more speed fluctuation."]))
    msg = (f"All checks passed. I = {info['I']:.3f} kg·m²: steel disk Ø{info['D']:.0f} × {info['t']:.0f} mm, {info['mass']:.1f} kg; "
           f"the speed stays within {info['rpm_min']:.0f}–{info['rpm_max']:.0f} rpm; rim speed {info['rim_speed']:.1f} m/s, "
           f"stress {info['sigma']:.1f} MPa.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _clutch(result, spec, say):
    """H5b: dry disc clutch / brake (uniform wear). Required: the torque (or power and speed)."""
    result["mechanism_type"] = "clutch"
    T, P, rpm = spec.get("torque_Nm"), spec.get("power_kW"), spec.get("speed_rpm")
    notes = []
    if not T and P and rpm:
        T = 9550.0 * float(P) / float(rpm)
        notes.append(f"Torque from the power: {T:.1f} N·m.")
    if not T:
        return dict(result, status="missing", message="Missing for the clutch: the torque (or power and speed).")
    lining = spec.get("lining") if spec.get("lining") in clutch.LININGS else None
    if lining is None:
        lining = "molded"
        notes.append("Lining not given: molded (f 0.35, 1.0 MPa).")
    notes.append(f"Service factor {clutch.K_SERVICE:g} on the torque.")
    say("Disc clutch: uniform wear, optimum inner radius, faces...")
    c, s_checks = clutch.design(float(T) * 1000.0, lining, spec.get("outer_diameter_max_mm") or None)
    checks = [{"name": f"{x['group']}: {x['name']}", "ok": x["util"] <= 1.0,
               "detail": f"{x['detail']} · use {x['util'] * 100:.0f}%"} for x in s_checks]
    if c is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "Even 6 friction faces do not fit that diameter: allow a larger clutch."]))
    bore = chain.shaft_bore(float(T) * 1000.0)
    result.update(clutch=dict(c, bore=bore), checks=checks, notes=notes, target={"torque_Nm": float(T)},
                  fusion={"type": "clutch", "passed_checks": True, "ro": c["ro"], "ri": c["ri"], "N": c["N"],
                          "bore": bore or 20.0})
    msg = (f"All checks passed. {'Single' if c['N'] == 2 else str(c['N'] // 2)}-plate clutch, friction faces Ø{2 * c['ro']:.0f} / "
           f"Ø{2 * c['ri']:.0f} mm ({c['N']} faces, {c['lining']}): holds {c['T_cap'] / 1000:.0f} N·m with a clamp force of "
           f"{c['F'] / 1000:.2f} kN.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _snap_fit(result, spec, say):
    """I1: cantilever snap-fit for printing. Required: the undercut (how far the hook must bend)."""
    result["mechanism_type"] = "snap_fit"
    y = spec.get("undercut_mm")
    if not y:
        return dict(result, status="missing", message="Missing for the snap-fit: the undercut (hook height, mm).")
    notes = []
    mat = spec.get("material") if spec.get("material") in snapfit.STRAIN else None
    if mat is None:
        mat = "PLA"
        notes.append("Material not given: PLA assumed (print the beam flat, along the layers).")
    releasable = bool(spec.get("releasable"))
    a_out = 45.0 if releasable else 90.0
    notes.append(f"Ramp 30°, retaining face {a_out:g}° ({'can be pulled apart' if releasable else 'permanent'}); friction μ {snapfit.MU:g}.")
    say("Snap-fit: root strain, deflection and mating forces...")
    d, s_checks = snapfit.design(float(y), mat, spec.get("hold_force_N") or None, spec.get("snap_width_mm") or None,
                                 30.0, a_out, bool(spec.get("repeated")))
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if d is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "No printable beam up to 80 mm does it: a smaller undercut or a tougher material (PETG)."]))
    result.update(snap_fit=d, checks=checks, notes=notes, target={"undercut": float(y)},
                  fusion={"type": "snap_fit", "passed_checks": True, "l": d["l"], "h": d["h"], "w": d["w"], "y": d["y"],
                          "alpha_in": d["alpha_in"], "alpha_out": d["alpha_out"]})
    hold = "it cannot be pulled out (locked)" if d["locked"] else f"it pulls out at {d['W_out']:.1f} N"
    msg = (f"All checks passed. Beam {d['l']:g} × {d['h']:g} × {d['w']:g} mm (length × thickness × width) with a {d['y']:g} mm hook: "
           f"root strain {100 * d['eps']:.2f} % of {100 * d['eps_allow']:.1f} %; it snaps in with {d['W_in']:.1f} N and {hold}.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _flexure(result, spec, say):
    """I2: printed living hinge / flexure. Required: the angle it bends through."""
    result["mechanism_type"] = "flexure"
    th = spec.get("spring_angle_deg")
    if not th:
        return dict(result, status="missing", message="Missing for the living hinge: the angle it must bend through (°).")
    notes = []
    mat = spec.get("material") if spec.get("material") in flexure.STRAIN else None
    if mat is None:
        mat = "PETG"
        notes.append("Material not given: PETG assumed (tougher than PLA for hinges; print the strip flat).")
    w = float(spec.get("snap_width_mm") or 10.0)
    if not spec.get("snap_width_mm"):
        notes.append("Width not given: 10 mm.")
    cyclic = bool(spec.get("repeated"))
    say("Living hinge: strip strain, length and thickness, stiffness...")
    d, s_checks = flexure.design(float(th), mat, w, cyclic)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if d is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "No printable strip up to 80 mm bends that far within the elastic range."]))
    result.update(flexure=d, checks=checks, notes=notes, target={"theta": float(th)},
                  fusion={"type": "flexure", "passed_checks": True, "L": d["L"], "t": d["t"], "w": d["w"]})
    msg = (f"All checks passed. Strip {d['L']:g} × {d['t']:g} mm, {d['w']:g} mm wide: {d['theta_deg']:g}° gives "
           f"{100 * d['eps']:.2f} % strain (limit {100 * d['eps_allow']:.2f} %) and takes {d['M']:.1f} N·mm "
           f"({d['k_Nmm_deg']:.3f} N·mm/°); its pivot wanders about {d['pivot_shift']:.0f} mm along the strip.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _press(result, spec, say):
    """E4: crank press system. Required: the press force, the stroke and the strokes per minute; the working
    stroke defaults to 10 % of the stroke (reported)."""
    result["mechanism_type"] = "press"
    F, S, spm = spec.get("force_N"), spec.get("stroke_mm"), spec.get("speed_rpm")
    missing = [n for v, n in ((F, "press force (N / kN / ton)"), (S, "stroke (mm)"), (spm, "strokes per minute")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the press: {', '.join(missing)}.")
    F, S, spm = float(F), float(S), float(spm)
    notes = []
    s_w = spec.get("working_stroke_mm")
    if not s_w:
        s_w = 0.1 * S
        notes.append(f"Working stroke not given: the force acts over the last {s_w:g} mm (10 % of the stroke).")
    Cs = spec.get("speed_fluctuation")
    if not Cs:
        notes.append(f"Speed fluctuation C_s = {press.CS_PRESS:g} (typical for presses).")
    notes.append(f"Motor 1450 rpm, 4:1 V-belt to the flywheel shaft, drive efficiency {press.ETA:g}; inertia and "
                 "friction of the ram left out of the torque.")
    say("Press: slider-crank, torque curve, energy per stroke, flywheel, motor...")
    d, s_checks = press.design(F, S, float(s_w), spm, Cs, 1450.0, "steel", spec.get("outer_diameter_max_mm") or None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if d is None or d["section"] is None or not all(c["ok"] for c in checks):
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "The press does not pass (see the failing checks): a longer stroke or a smaller force."]))
    fw = d["flywheel"]
    section = strength.section_with_bosses({"hole": d["section"]["hole"], "shaft": d["section"]["shaft"],
                                            "links": d["section"]["links"]})
    g = slider_crank.synthesize(S, 0.0)
    frames = slider_crank.simulate(g["a"], g["b"], 0.0)
    m = slider_crank.measure(frames)
    k = len(frames) // 8
    th0 = math.radians(frames[k]["theta"])
    pts = {"O2": [0.0, 0.0], "A": [g["a"] * math.cos(th0), g["a"] * math.sin(th0)], "B": [frames[k]["x"], 0.0]}
    result.update(press=dict({k2: v for k2, v in d.items() if k2 not in ("section",)}, section=section),
                  checks=checks, notes=notes, target={"force_N": F},
                  fusion={"type": "press", "passed_checks": True, **g, "x_min": m["x_min"], "x_max": m["x_max"],
                          "points_mm": pts, "section_mm": section, "fly_D": fw["D"], "fly_t": fw["t"],
                          "fly_bore": fw["bore"], "gear_ratio": d["gear_ratio"]})
    msg = (f"All checks passed. Crank {d['a']:g} mm, rod {d['b']:g} mm (stroke {S:g} mm); {F / 1000:g} kN over the last "
           f"{d['s_w']:g} mm is {d['W']:.0f} J per stroke. Peak crank torque {d['T_max'] / 1000:.0f} N·m, mean "
           f"{d['T_mean'] / 1000:.1f} N·m: with a Ø{fw['D']:.0f} × {fw['t']:.0f} mm flywheel ({fw['mass']:.0f} kg) at "
           f"{d['flywheel_rpm']:.0f} rpm a {d['P_motor'] / 1000:.2f} kW motor is enough (without it "
           f"{d['P_no_flywheel'] / 1000:.1f} kW); gears {d['gear_ratio']:.1f}:1 to the crankshaft.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _path_synthesis(result, spec, say):
    """F2b: a crank-rocker whose coupler point passes through the given points (path generation, no timing)."""
    result["mechanism_type"] = "path_synthesis"
    raw = spec.get("path_points") or []
    targets = [(float(q[0]), float(q[1])) for q in raw if isinstance(q, (list, tuple)) and len(q) == 2]
    if len(targets) < 3:
        return dict(result, status="missing",
                    message="Missing for path synthesis: at least 3 points (x, y in mm) the point should pass through.")
    if len(targets) > 8:
        return dict(result, status="missing", message="Path synthesis takes 3 to 8 points.")
    notes = ["Coupler point on the coupler's line (a straight three-hole bar); no timing asked for."]
    say(f"Path synthesis: searching crank-rockers through {len(targets)} points (Nelder-Mead, many starts)...")
    x, val, scale = pathsynth.synthesize(targets)
    r = pathsynth.report(x, targets)
    tol = max(0.2, 0.005 * scale)
    checks = [{"name": "Passes through the points", "ok": r["max_dev"] <= tol,
               "detail": f"largest miss {r['max_dev']:.3f} mm (limit {tol:.2f} mm); crank at "
                         + ", ".join(f"{t:.0f}°" for t in r["theta_at"])},
              {"name": "Crank turns fully (Grashof)", "ok": r["grashof"],
               "detail": f"a {r['a']:.1f}, b {r['b']:.1f}, c {r['c']:.1f}, d {r['d']:.1f} mm"},
              {"name": "Transmission angle", "ok": pathsynth.MU_MIN <= r["mu_min"] and r["mu_max"] <= 180 - pathsynth.MU_MIN,
               "detail": f"{r['mu_min']:.0f}° – {r['mu_max']:.0f}° (limits {pathsynth.MU_MIN:g}–{180 - pathsynth.MU_MIN:g}°)"}]
    section = strength.section_with_bosses(strength.DEFAULT_SECTION)
    O4 = r["O4"]
    sh = lambda p: (p[0] - O4[0], p[1] - O4[1])
    lay_frames = [{"A": sh(f["A"]), "B": sh(f["B"]), "P": sh(f["P"])} for f in r["frames"][::8]]
    order_pts = sorted((("A", 0.0), ("B", r["b"]), ("P", r["p"])), key=lambda t: t[1])
    ends = (order_pts[0][0], order_pts[-1][0])
    order, clearances, layers_ok = interference.choose_layer_order(
        {"o2": sh(r["O2"])}, lay_frames, section, dict(strength.LINK_ENDS, Coupler=ends))
    margin = min((c[2] for c in clearances), default=0.0)
    checks.append({"name": "Layers (a shaft passes a moving link)", "ok": layers_ok,
                   "detail": f"smallest gap {margin + interference.CLEARANCE:.1f} mm (needed {interference.CLEARANCE:g})"})
    passed = all(c["ok"] for c in checks)
    k = min(range(len(r["frames"])), key=lambda i: abs(r["frames"][i]["theta"] - r["theta_at"][0]))
    fr = r["frames"][k]
    r3 = lambda p: [round(p[0], 3), round(p[1], 3)]
    pts = {"O2": r3(r["O2"]), "O4": r3(O4), "A": r3(fr["A"]), "B": r3(fr["B"]), "P": r3(fr["P"])}
    path = [r3(p) for p in r["path"][::6]]
    lengths = {k2: round(r[k2], 3) for k2 in ("a", "b", "c", "d", "p")}
    info = dict(lengths, targets=[list(t) for t in targets], deviations=r["deviations"], max_dev=r["max_dev"],
                theta_at=r["theta_at"], mu_min=r["mu_min"], mu_max=r["mu_max"], path=path, points=pts)
    result.update(pathsynth=info, checks=checks, notes=notes, section_mm=section,
                  target={"points": [list(t) for t in targets]},
                  fusion={"type": "straight_line", "passed_checks": passed, "lengths_mm": lengths, "points_mm": pts,
                          "section_mm": section, "layer_order": list(order), "path": path, "line": None,
                          "targets": [list(t) for t in targets], "coupler_ends": list(ends)})
    if passed:
        msg = (f"All checks passed. Crank {r['a']:.1f} mm, coupler {r['b']:.1f} mm with the point {r['p']:.1f} mm from "
               f"the crank pin along it, rocker {r['c']:.1f} mm, ground {r['d']:.1f} mm; the point passes all "
               f"{len(targets)} points within {r['max_dev']:.3f} mm.")
    else:
        msg = "The best linkage found does not meet the checks above: move the points closer together or give fewer."
    result.update(status="ok" if passed else "failed", message=" ".join(notes + [msg]))
    return result


def _coupling(result, spec, say):
    """H5b: rigid flange coupling. Required: the torque; the shaft is sized in torsion when not given."""
    result["mechanism_type"] = "coupling"
    T = spec.get("torque_Nm")
    if not T:
        return dict(result, status="missing", message="Missing for the coupling: the torque (N·m).")
    notes = [f"Service factor {coupling.K_SERVICE:g}; steel hubs, class 8.8 bolts carrying the torque by friction (μ {coupling.MU:g})."]
    d = spec.get("shaft_diameter_mm")
    say("Flange coupling: key, bolts (friction), hub and flange...")
    c, s_checks = coupling.design(float(T) * 1000.0, float(d) if d else None, spec.get("bolts") or None)
    checks = [{"name": f"{x['group']}: {x['name']}", "ok": x["util"] <= 1.0,
               "detail": f"{x['detail']} · use {x['util'] * 100:.0f}%"} for x in s_checks]
    if c is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "The coupling does not pass (see the checks): usually the shaft is too thin for the torque."]))
    if not d:
        notes.append(f"Shaft not given: Ø{c['d']:g} mm from torsion.")
    result.update(coupling=c, checks=checks, notes=notes, target={"torque_Nm": float(T)},
                  fusion={"type": "coupling", "passed_checks": True, **{k: c[k] for k in ("d", "D_hub", "L_hub", "D_bolt",
                                                                                        "D_flange", "t", "N", "bolt")}})
    k, b = c["key"], c["bolt"]
    msg = (f"All checks passed. Flange coupling for a Ø{c['d']:g} shaft: hubs Ø{c['D_hub']:g} × {c['L_hub']:g}, flanges "
           f"Ø{c['D_flange']:g} × {c['t']:g} mm, {c['N']} × M{b['d']}×{b['L']} bolts on Ø{c['D_bolt']:g} (friction holds "
           f"{b['T_fric'] / 1000:.0f} N·m), key {k['b']}×{k['h']}×{k['length']:g}.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _disc_spring(result, spec, say):
    """H4b: disc (Belleville) spring stack. Required: force and the deflection at that force."""
    result["mechanism_type"] = "disc_spring"
    F, y = spec.get("spring_force_N"), spec.get("spring_deflection_mm")
    missing = [n for v, n in ((F, "force"), (y, "deflection at that force (mm)")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the disc spring: {', '.join(missing)}.")
    notes = ["Spring steel 51CrV4, DIN 2093 series proportions, static load."]
    rod = spec.get("arbor_mm")
    say("Disc springs: DIN 2092 force curve, stacking in series / parallel...")
    b, s_checks = disc_spring.design(float(F), float(y), float(rod) if rod else None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if b is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "No disc stack does it: allow more deflection (more discs in series) or a larger diameter."]))
    result.update(disc_spring=b, checks=checks, notes=notes, target={"force_N": float(F), "deflection_mm": float(y)},
                  fusion={"type": "disc_spring", "passed_checks": True, **{k: b[k] for k in ("De", "Di", "t", "h0", "np", "ns")},
                          "rod": rod})
    stack = (f"{b['ns']} in series" if b["np"] == 1 else f"{b['ns']} groups of {b['np']} nested discs in series")
    msg = (f"All checks passed. Disc springs {b['De']:g} × {b['Di']:g} × {b['t']:g} mm (series {b['series']}, h0 {b['h0']:g}), "
           f"{stack}: {b['y']:.2f} mm at {float(F):g} N, stress {b['sigma_om']:.0f} MPa; stack {b['L0']:.1f} mm free, "
           f"{b['L_loaded']:.1f} mm loaded.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _extension_spring(result, spec, say):
    """H4c: extension spring with full-loop hooks. Required: the force and how far it extends to reach it."""
    result["mechanism_type"] = "extension_spring"
    F, y = spec.get("spring_force_N"), spec.get("spring_deflection_mm")
    missing = [n for v, n in ((F, "force"), (y, "extension at that force (mm)")) if not v]
    if missing:
        return dict(result, status="missing", message=f"Missing for the extension spring: {', '.join(missing)}.")
    notes = ["Music wire, close-wound with initial tension (middle of the preferred range), full-loop hooks."]
    say("Extension spring: initial tension, rate, body and hook stresses...")
    s, s_checks = extension_spring.design(float(F), float(y))
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if s is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "No extension spring found: allow more extension or less force."]))
    result.update(extension=s, checks=checks, notes=notes, target={"force_N": float(F), "extension_mm": float(y)},
                  fusion={"type": "extension_spring", "passed_checks": True, "d": s["d"], "D": s["D"], "Nb": s["Nb"]})
    msg = (f"All checks passed. Wire Ø{s['d']:g} mm, coil Ø{s['D']:.1f} mm, {s['Nb']:g} body coils, free length {s['L0']:.1f} mm "
           f"inside the hooks; it starts to open at {s['Fi']:.1f} N (initial tension), k = {s['k']:.2f} N/mm, "
           f"{s['F']:.1f} N at {float(y):g} mm. The hooks decide: bending {s['sigma_A']:.0f} MPa.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _spline(result, spec, say):
    """H3b / H3c: straight-sided (ISO 14) or involute (DIN 5480) spline. Required: the torque; the use (fixed /
    sliding) defaults to fixed, the profile to straight-sided."""
    result["mechanism_type"] = "spline"
    T = spec.get("torque_Nm")
    if not T:
        return dict(result, status="missing", message="Missing for the spline: the torque (N·m).")
    use = spec.get("spline_use") if spec.get("spline_use") in spline.P_ALLOW else None
    notes = [f"Service factor {spline.K_SERVICE:g}, steel shaft and hub (C45), 75 % of the teeth carrying."]
    if use is None:
        use = "fixed"
        notes.append("Use not given: a fixed hub (100 MPa on the flanks).")
    if spec.get("spline_profile") == "involute":
        return _spline_involute(result, spec, say, T, use, notes)
    say("Spline: ISO 14 sizes, flank pressure, hub length...")
    s, s_checks = spline.design(float(T) * 1000.0, use)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if s is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "No ISO 14 spline in the table carries it: a larger shaft or a fixed hub."]))
    result.update(spline=s, checks=checks, notes=notes, target={"torque_Nm": float(T)},
                  fusion={"type": "spline", "passed_checks": True, "profile": "straight",
                          **{k: s[k] for k in ("N", "d", "D", "B", "L", "hub_od")}})
    msg = (f"All checks passed. ISO 14 {s['series']} series {s['N']} × {s['d']} × {s['D']} (teeth {s['B']:g} mm wide), hub "
           f"{s['L']:g} mm long: the flanks carry {s['cap'] / 1000:.0f} N·m for {s['Td'] / 1000:.0f} N·m.")
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _spline_involute(result, spec, say, T, use, notes):
    """H3c: involute spline (DIN 5480), the same flank-pressure rule as ISO 14."""
    say("Spline: DIN 5480 sizes, flank pressure, hub length...")
    notes = notes + ["Module by the diameter as in the common DIN 5480 sizes; roots approximate (they follow the tool)."]
    s, s_checks = din5480.design(float(T) * 1000.0, use)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for c in s_checks]
    if s is None:
        return dict(result, status="failed", checks=checks, notes=notes, message=" ".join(notes + [
            "No DIN 5480 spline up to W 200 carries it: a fixed hub or a coupling."]))
    result.update(spline=s, checks=checks, notes=notes, target={"torque_Nm": float(T)},
                  fusion={"type": "spline", "passed_checks": True, "profile": "involute",
                          **{k: s[k] for k in ("dB", "m", "z", "d", "db", "s", "da1", "df1", "da2", "df2", "L", "hub_od")}})
    msg = (f"All checks passed. DIN 5480 {s['name']} (profile shift x = {s['x']:+.2f}), hub {s['L']:g} mm long: "
           f"the flanks carry {s['cap'] / 1000:.0f} N·m for {s['Td'] / 1000:.0f} N·m.")
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
    F_min = spec.get("spring_force_min_N")
    F_min = float(F_min) if F_min is not None and 0 <= float(F_min) < F else None
    geo, s_checks = springs.design(F, y, mat, taper, spec.get("outer_diameter_max_mm") or None, F_min)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": c["detail"] + (f" · use {c['util'] * 100:.0f}%" if c["group"] in ("Stress", "Buckling") else "")}
              for c in s_checks]
    result.update(checks=checks, notes=notes, target={"force_N": F, "deflection_mm": y, "taper": taper, "F_min": F_min})
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
    if not passed and Q == 1.0 and psi > 90:
        six = _six_bar(result, spec, notes, say, a, psi)          # F4b: two four-bars in series
        if six:
            return six
    return _finish(result, spec, notes, say, mech, frames, meas, checks, passed, a, psi, Q, False)


_SIX_CACHE = {}


def _six_bar(result, spec, notes, say, a, psi):
    """F4b: Watt II six-bar for a swing a four-bar cannot give with a good transmission angle."""
    say("Watt six-bar: crank-rocker + function generator in series, simulated as a whole...")
    key = (round(float(a), 3), round(float(psi), 3))
    if key not in _SIX_CACHE:
        if spec.get("_preview"):                      # slider previews: 4-7 s each would pile up while dragging
            notes.append("A Watt six-bar can give this swing: press Draw in Fusion to synthesize it (a few seconds).")
            result["six_bar_pending"] = True          # the panel keeps Draw enabled
            return None
        _SIX_CACHE[key] = sixbar.synthesize(a, psi)
    m, frames = _SIX_CACHE[key]
    if m is None or not m["ok"]:
        return None
    me = m["measured"]
    checks = [{"name": "Output swing (six-bar simulation)", "ok": True, "detail": f"{me['swing']:.1f}° (target {psi:g}°)"},
              {"name": "Transmission angle, first loop", "ok": True, "detail": f"{me['mu1_min']:.1f}° – {me['mu1_max']:.1f}°"},
              {"name": "Transmission angle, second loop", "ok": True, "detail": f"{me['mu2_min']:.1f}° – {me['mu2_max']:.1f}°"}]
    section = strength.DEFAULT_SECTION
    F, mat_key = spec.get("max_load_N"), spec.get("material")
    torque = None
    if F:
        if mat_key not in strength.MATERIALS:
            mat_key = "PLA"
            notes.append("Material not given: PLA (3D print) assumed.")
        mat = strength.MATERIALS[mat_key]
        f1, f2, F1 = sixbar.forces(m, frames, float(F))
        forc = dict(f1, F_ab_max=max(f1["F_ab_max"], f2["F_ab_max"]), M_rocker=max(f1["M_rocker"], f2["M_rocker"]))
        lengths = {"a": m["m1"]["a"], "b": max(m["m1"]["b"], m["b2"]), "c": max(m["m1"]["c"], m["c2"], m["a2"]),
                   "d": max(m["m1"]["d"], m["d2"])}
        sec, s_checks = strength.size_links(forc, lengths, mat)
        if sec is None:
            return dict(result, status="failed", checks=checks, notes=notes, mechanism_type="six_bar",
                        message=" ".join(notes + [f"No {mat['name']} section carries {F:g} N on the six-bar."]))
        section = sec
        worst = {}
        for c in s_checks:
            if c["group"] not in worst or c["util"] > worst[c["group"]]["util"]:
                worst[c["group"]] = c
        checks += [{"name": f"{g}: {c['name']}", "ok": c["util"] <= 1.0,
                    "detail": f"{c['detail']} · use {c['util'] * 100:.0f}%"} for g, c in worst.items()]
        torque = f1["M_crank_max"]
    section = strength.section_with_bosses(section)
    f0 = frames[0]
    r3 = lambda p: [round(p[0], 3), round(p[1], 3)]
    pts = {"O2": r3(m["m1"]["o2"]), "O4": [0.0, 0.0], "O6": r3(m["O6"]), "A": r3(f0["A"]), "B": r3(f0["B"]),
           "A2": r3(f0["A2"]), "B2": r3(f0["B2"])}
    L = {"a": m["m1"]["a"], "b": m["m1"]["b"], "c": m["m1"]["c"], "d": m["m1"]["d"],
         "a2": m["a2"], "b2": m["b2"], "c2": m["c2"], "d2": m["d2"]}
    info = dict({k: round(v, 2) for k, v in L.items()}, psi1=m["psi1"], swing=me["swing"], points=pts,
                mu1=[me["mu1_min"], me["mu1_max"]], mu2=[me["mu2_min"], me["mu2_max"]], torque_Nmm=torque,
                phi_curve=[(round(f["theta"], 1), round(f["phi"], 2)) for f in frames[::6]])
    result.update(mechanism_type="six_bar", sixbar=info, checks=checks, notes=notes, section_mm=section,
                  target={"swing": psi},
                  fusion={"type": "six_bar", "passed_checks": True, "points_mm": pts, "section_mm": section})
    msg = (f"A four-bar cannot swing {psi:g}° with the transmission angle in limits, so this is a Watt six-bar: a "
           f"crank-rocker (crank {L['a']:g}, coupler {L['b']:.1f}, rocker {L['c']:.1f}, ground {L['d']:.1f} mm, "
           f"{m['psi1']}° swing) drives, through the rocker's second arm ({L['a2']:.1f} mm), a second four-bar "
           f"(coupler {L['b2']:.1f}, output {L['c2']:.1f}, ground {L['d2']:.1f} mm): output swing {me['swing']:.1f}°.")
    if torque:
        msg += f" Max crank torque {torque / 1000:.2f} N·m."
    result.update(status="ok", message=" ".join(notes + [msg]))
    return result


def _finish(result, spec, notes, say, mech, frames, meas, checks, passed, a, psi, Q, rr):
    """Shared by both four-bar kinds: strength, layer interference, Fusion data, message."""
    # ---- strength: with a load the sections are sized; without one nothing is invented
    section, strength_ok = strength.DEFAULT_SECTION, True
    F, mat_key = spec.get("max_load_N"), spec.get("material")
    rpm = spec.get("speed_rpm") if not rr else None
    if mat_key not in strength.MATERIALS:
        mat_key = None
    dyn = None
    if passed and F:
        if mat_key is None:
            mat_key = "PLA"
            notes.append("Material not given: PLA (3D print) assumed.")
        mat = strength.MATERIALS[mat_key]
        say("Strength and section sizing...")
        forces = strength.analyze_forces(mech, frames, F)
        sec, s_checks = strength.size_links(forces, {k: mech[k] for k in "abcd"}, mat)
        if sec and rpm:
            # K2: at speed the pins also carry the links' inertia; if that exceeds the static forces the
            # sections are sized again for the larger ones (the masses change, so the dynamics is redone)
            say("Dynamics at speed (matrix method): pin forces and drive torque...")
            dyn = dynamics.analyse(mech, frames, sec["links"], dynamics.DENSITY[mat_key], float(rpm), float(F))
            if dyn["F_pin_max"] > forces["F_ab_max"] or abs(dyn["T_max"]) * 1000 > forces["M_crank_max"]:
                forces = dict(forces, F_ab_max=max(forces["F_ab_max"], dyn["F_pin_max"]),
                              M_crank_max=max(forces["M_crank_max"], abs(dyn["T_max"]) * 1000))
                sec2, s_checks2 = strength.size_links(forces, {k: mech[k] for k in "abcd"}, mat)
                if sec2:
                    sec, s_checks = sec2, s_checks2
                    dyn = dynamics.analyse(mech, frames, sec["links"], dynamics.DENSITY[mat_key], float(rpm), float(F))
                    notes.append(f"Sections sized for the dynamic forces at {float(rpm):g} rpm (larger than the static ones).")
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
    if passed and rpm and dyn is None:
        say("Dynamics at speed (matrix method): pin forces and drive torque...")
        dyn = dynamics.analyse(mech, frames, section["links"], dynamics.DENSITY[mat_key or "PLA"], float(rpm), 0.0)
    if dyn:
        I_fly = flywheel.inertia(dyn["dE"], float(rpm), 0.05) if dyn["dE"] > 1e-9 else 0.0
        step = max(1, len(dyn["T"]) // 72)
        th0 = dyn["theta"][0]
        result["dynamics"] = {
            "rpm": float(rpm), "T_max": dyn["T_max"], "T_mean": dyn["T_mean"], "dE": dyn["dE"],
            "F_pin_max": dyn["F_pin_max"], "F_ground_max": dyn["F_ground_max"], "shake_max": dyn["shake_max"],
            "masses_g": {k: 1000 * v for k, v in dyn["masses"].items()}, "I_flywheel": I_fly,
            "curve": sorted(((round((t - th0) % 360, 1), round(T, 4)) for t, T in zip(dyn["theta"][::step], dyn["T"][::step]))),
        }
        checks.append({"name": f"Dynamics at {float(rpm):g} rpm", "ok": True,
                       "detail": f"drive torque {dyn['T_max']:.2f} N·m peak / {dyn['T_mean']:.2f} mean, pin force "
                                 f"{dyn['F_pin_max']:.0f} N, shaking force {dyn['shake_max']:.1f} N; a flywheel of "
                                 f"{I_fly * 1e4:.1f} kg·cm² keeps the speed within ±2.5 %"})
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
