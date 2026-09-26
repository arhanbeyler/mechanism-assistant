"""
LLM evaluation set: run it whenever a prompt changes, so that fixing one case does not break another
(example: fixing "45° cevirince" once broke "30 mm krank, 60° salinim").
The sentences are mostly Turkish (the user's language) plus a few English ones (the panel examples).

Usage (in the project folder, with Ollama running):  python -u tools/llm_eval.py
The LLM output and the code safety nets (pipeline.input_turn_backstop, force_backstop) are tested together.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from llm_extract import extract_spec  # noqa: E402
from pipeline import force_backstop, input_turn_backstop  # noqa: E402

# (sentence, expected fields)
CASES = [
    ("30 mm krank, 60° salınım", dict(mechanism_type="crank_rocker", crank_length_mm=30, output_swing_deg=60)),
    ("30 mm krank, 60 derece salınım", dict(mechanism_type="crank_rocker", crank_length_mm=30, output_swing_deg=60)),
    ("4 cm krank, 50°, hızlı dönüşlü", dict(mechanism_type="crank_rocker", crank_length_mm=40, quick_return=True)),
    ("30 mm krank tam tur dönsün, kol 60 derece sallansın", dict(mechanism_type="crank_rocker", output_swing_deg=60)),
    ("30 mm krank, 60 derece salınım, uçta 5 kg yük, PETG", dict(mechanism_type="crank_rocker", material="PETG")),
    ("4 cm kolu 45° çevirince diğer kol 120° dönsün, PETG, 2 kg",
     dict(mechanism_type="rocker_rocker", crank_length_mm=40, input_swing_deg=45, output_swing_deg=120)),
    ("4 cm kolu 45 derece çevirince diğer kol 120 derece dönsün",
     dict(mechanism_type="rocker_rocker", input_swing_deg=45, output_swing_deg=120)),
    ("40 mm giriş kolu 60° dönünce çıkış kolu 90° dönsün",
     dict(mechanism_type="rocker_rocker", crank_length_mm=40, input_swing_deg=60, output_swing_deg=90)),
    ("kolu 30° itince diğer taraf 75° açılsın, kol 50 mm",
     dict(mechanism_type="rocker_rocker", input_swing_deg=30, output_swing_deg=75)),
    ("bana bir dört çubuk yap, 40 derece sallansın", dict(crank_length_mm=None, output_swing_deg=40)),
    ("1:3 dişli çifti, giriş torku 2 N·m, PLA", dict(mechanism_type="spur_gear", gear_ratio=3, torque_Nm=2, material="PLA")),
    ("devri 3 kat düşüren bir redüktör, 5 Nm tork, 1500 rpm, çelik",
     dict(mechanism_type="reducer", gear_ratio=3, torque_Nm=5, speed_rpm=1500, material="steel")),
    ("1.5 kW 1450 devirli motora bağlı 1:4 dişli kutusu",
     dict(mechanism_type="reducer", gear_ratio=4, power_kW=1.5, speed_rpm=1450, torque_Nm=None)),
    ("devri 12 kat düşüren iki kademeli redüktör, 10 Nm, 1450 rpm",
     dict(mechanism_type="reducer", gear_ratio=12, torque_Nm=10, speed_rpm=1450)),
    ("two-stage gearbox, 1:20, 0.75 kW motor at 1400 rpm",
     dict(mechanism_type="reducer", gear_ratio=20, power_kW=0.75, speed_rpm=1400)),
    ("60 mm stroklu, pistona 500 N gelen krank-biyel mekanizması",
     dict(mechanism_type="slider_crank", stroke_mm=60, max_load_N=500)),
    ("strok 80 mm, 10 mm kaçıklık, 1200 devir piston", dict(mechanism_type="slider_crank", stroke_mm=80, offset_mm=10,
                                                            speed_rpm=1200)),
    ("25 mm krank ile ileri geri giden bir piston", dict(mechanism_type="slider_crank", crank_length_mm=25)),
    ("krankla dönen, bir noktası 100 mm düz bir çizgide giden mekanizma",
     dict(mechanism_type="straight_line", stroke_mm=100)),
    ("500 kg kaldıracak trapez vidalı kriko, 300 mm kaldırma", dict(mechanism_type="power_screw", force_N=4905,
                                                                    travel_mm=300, screw_ends="jack")),
    ("lead screw moving a 2 kN load over 400 mm at 300 rpm, bronze nut",
     dict(mechanism_type="power_screw", force_N=2000, travel_mm=400, speed_rpm=300, nut_material="bronze")),
    ("50 kN değişken yükü taşıyacak cıvatalı flanş, iki 20 mm çelik sac, 6 cıvata",
     dict(mechanism_type="bolted_joint", force_N=50000, grip_mm=40, bolts=6, fluctuating=True)),
    ("how many 10.9 bolts to hold a 2 ton cast iron cover", dict(mechanism_type="bolted_joint", force_N=19620,
                                                                 bolt_class="10.9", member_material="cast_iron")),
    ("konik dişli çifti, oran 2, 10 Nm giriş torku", dict(mechanism_type="bevel_gear", gear_ratio=2, torque_Nm=10)),
    ("mitre gears transmitting 15 N·m at 600 rpm through a right angle",
     dict(mechanism_type="bevel_gear", gear_ratio=1, torque_Nm=15, speed_rpm=600)),
    ("1:30 sonsuz vida, çıkışta 50 Nm, 1400 devir, kendini kilitlesin",
     dict(mechanism_type="worm_gear", gear_ratio=30, output_torque_Nm=50, speed_rpm=1400, self_locking=True)),
    ("worm drive, ratio 40, 0.37 kW motor at 1400 rpm", dict(mechanism_type="worm_gear", gear_ratio=40, power_kW=0.37,
                                                            speed_rpm=1400, self_locking=False)),
    ("helis dişli çifti, oran 3, 10 Nm, 25 derece helis açısı", dict(mechanism_type="helical_gear", gear_ratio=3,
                                                                     torque_Nm=10, helix_deg=25)),
    ("helical gears 1:4 with 50 N·m input at 900 rpm", dict(mechanism_type="helical_gear", gear_ratio=4, torque_Nm=50,
                                                           speed_rpm=900)),
    ("1:5 planet dişli, giriş torku 20 Nm, 3 planetli", dict(mechanism_type="planetary", gear_ratio=5, torque_Nm=20,
                                                              planets=3)),
    ("epicyclic gear set, ratio 6, 0.75 kW at 1400 rpm", dict(mechanism_type="planetary", gear_ratio=6, power_kW=0.75,
                                                              speed_rpm=1400)),
    ("1450 devirli 3 kW motordan zincirle 1:3 aktarma, konveyör", dict(mechanism_type="chain_drive", power_kW=3,
                                                                     speed_rpm=1450, gear_ratio=3, shock="moderate")),
    ("roller chain drive, 200 N·m at 100 rpm, eksenler arası 600 mm, ratio 2",
     dict(mechanism_type="chain_drive", torque_Nm=200, speed_rpm=100, gear_ratio=2, center_distance_mm=600)),
    ("V kayış kasnak, 1.5 kW 1400 devir, fan devri 560", dict(mechanism_type="belt_drive", power_kW=1.5,
                                                              speed_rpm=1400, gear_ratio=2.5)),
    ("V-belt drive for a 7.5 kW motor at 1460 rpm driving a crusher at 730 rpm",
     dict(mechanism_type="belt_drive", power_kW=7.5, speed_rpm=1460, gear_ratio=2, shock="heavy")),
    ("kremayer dişli, 800 N itecek, 250 mm hareket", dict(mechanism_type="rack_pinion", force_N=800, travel_mm=250)),
    ("rack and pinion pushing 50 kg over 300 mm at 60 rpm",
     dict(mechanism_type="rack_pinion", force_N=490.5, travel_mm=300, speed_rpm=60)),
    ("20 Nm tutan cırcır mekanizması, 24 dişli, çelik", dict(mechanism_type="ratchet", torque_Nm=20, teeth=24,
                                                             material="steel")),
    ("ratchet and pawl that holds 5 N·m, PETG", dict(mechanism_type="ratchet", torque_Nm=5, material="PETG")),
    ("60 mm stroklu İskoç boyunduruğu, 300 devir", dict(mechanism_type="scotch_yoke", stroke_mm=60, speed_rpm=300)),
    ("scotch yoke with a 40 mm crank and 200 N on the yoke", dict(mechanism_type="scotch_yoke", crank_length_mm=40,
                                                                 max_load_N=200)),
    ("vargel tezgahı için hızlı geri dönüş mekanizması, 200 mm strok, dönüş 1.6 kat hızlı, 500 N kesme kuvveti",
     dict(mechanism_type="quick_return", stroke_mm=200, time_ratio=1.6, max_load_N=500)),
    ("quick return mechanism, 150 mm stroke", dict(mechanism_type="quick_return", stroke_mm=150)),
    ("90 derece açılan mafsallı işkence, kol 60 mm, 1000 N tutsun, çelik",
     dict(mechanism_type="toggle_clamp", opening_deg=90, arm_length_mm=60, clamp_force_N=1000, material="steel")),
    ("toggle clamp with an 80 mm arm that opens 100 degrees", dict(mechanism_type="toggle_clamp", opening_deg=100,
                                                                  arm_length_mm=80)),
    ("6 istasyonlu Cenevre mekanizması, eksenler arası 100 mm", dict(mechanism_type="geneva", slots=6,
                                                                     center_distance_mm=100)),
    ("90 derece adım adım dönen tabla, merkez mesafesi 80 mm, 30 devir, 2 kg 300 mm çaplı tabla",
     dict(mechanism_type="geneva", step_deg=90, center_distance_mm=80, speed_rpm=30, load_mass_kg=2,
          load_diameter_mm=300)),
    ("Geneva drive with 8 slots, 120 mm centre distance, aluminium",
     dict(mechanism_type="geneva", slots=8, center_distance_mm=120, material="aluminum")),
    ("Hoeckens mekanizması, 8 cm doğrusal yol, 60 devir", dict(mechanism_type="straight_line", stroke_mm=80,
                                                              speed_rpm=60)),
    ("straight-line linkage with a 150 mm straight path, pushing 10 N, PETG",
     dict(mechanism_type="straight_line", stroke_mm=150, max_load_N=10, material="PETG")),
    ("20 mm kaldıran, 120 derecede yükselen kam, 600 devirde dönecek, itici 200 gram",
     dict(mechanism_type="cam", cam_rise_mm=20, rise_deg=120, speed_rpm=600, follower_mass_kg=0.2)),
    # English: the panel's example sentences
    ("30 mm crank, 60° swing, 5 kg at the tip, PETG",
     dict(mechanism_type="crank_rocker", crank_length_mm=30, output_swing_deg=60, material="PETG")),
    ("1.5 kW 1450 rpm motor, 1:4 gearbox", dict(mechanism_type="reducer", gear_ratio=4, power_kW=1.5, speed_rpm=1450)),
    ("turn a 4 cm arm 45° and the other arm turns 120°",
     dict(mechanism_type="rocker_rocker", crank_length_mm=40, input_swing_deg=45, output_swing_deg=120)),
    ("a crank-driven point that moves 100 mm in a straight line",
     dict(mechanism_type="straight_line", stroke_mm=100)),
    ("piston with a 50 mm stroke and a 150 mm connecting rod", dict(mechanism_type="slider_crank", stroke_mm=50,
                                                                    rod_length_mm=150)),
    ("çevrim oranı 2.5 olan dişli", dict(mechanism_type="spur_gear", gear_ratio=2.5, torque_Nm=None)),
    ("100 N yükte 20 mm sıkışan bir basma yayı", dict(mechanism_type="spring", spring_force_N=100, spring_deflection_mm=20)),
    ("5 kg taşıyan, 15 mm çöken konik yay", dict(mechanism_type="spring", spring_shape="conical", spring_deflection_mm=15)),
    ("dış çapı en fazla 12 mm olan yay, 30 N'da 10 mm sıkışsın",
     dict(mechanism_type="spring", spring_force_N=30, spring_deflection_mm=10, outer_diameter_max_mm=12)),
    ("iticiyi 20 mm kaldıran bir kam, 120 derecede yükselsin, 60 derece beklesin, 120 derecede insin",
     dict(mechanism_type="cam", cam_rise_mm=20, rise_deg=120, dwell_top_deg=60, return_deg=120)),
    ("15 mm stroklu sikloidal kam, 90° yükselme", dict(mechanism_type="cam", cam_rise_mm=15, rise_deg=90,
                                                       motion_law="cycloidal")),
    ("100 Nm tork ve 60 Nm eğilme taşıyan kama yuvalı mil çapı", dict(mechanism_type="shaft", torque_Nm=100,
                                                                       bending_moment_Nm=60, feature="keyseat")),
    ("yataklar arası 200 mm, ortadan 2 kN kuvvet gelen, 50 Nm tork ileten 1020 çelik mil",
     dict(mechanism_type="shaft", radial_force_N=2000, span_mm=200, torque_Nm=50, shaft_material="1020")),
    ("1450 devirde dönen, 30 Nm tork ve 40 Nm eğilme taşıyan segmanlı mil",
     dict(mechanism_type="shaft", speed_rpm=1450, torque_Nm=30, bending_moment_Nm=40, feature="ring_groove")),
]


# K1 edits: (current design, message, expected) with expected = spec fields after the edit,
# "ask" (no amount given: the engine must ask) or "new" (a new request, not an edit)
FB = {"mechanism_type": "crank_rocker", "crank_length_mm": 30, "output_swing_deg": 60, "input_swing_deg": None,
      "quick_return": False, "time_ratio": None, "max_load_N": None, "material": "PLA"}
RED = {"mechanism_type": "reducer", "gear_ratio": 3, "torque_Nm": 20, "power_kW": None, "speed_rpm": 1450,
       "material": "steel", "shaft_material": "1045"}
SL = {"mechanism_type": "straight_line", "stroke_mm": 100, "max_load_N": None, "speed_rpm": None, "material": None}
EDIT_CASES = [
    (FB, "krankı 40 mm yap", dict(crank_length_mm=40, output_swing_deg=60)),
    (FB, "alüminyum olsun", dict(material="aluminum", crank_length_mm=30)),
    (FB, "uçtaki yük 10 kg olsun", dict(max_load_N=98.1)),
    (FB, "salınımı 70 dereceye çıkar", dict(output_swing_deg=70)),
    (FB, "krankı 5 mm uzat", dict(crank_length_mm=35)),
    (FB, "biraz daha uzun olsun", "ask"),
    (FB, "make the crank 45 mm", dict(crank_length_mm=45)),
    (FB, "1:3 dişli çifti, giriş torku 2 N·m", "new"),
    (RED, "oranı 4 yap", dict(gear_ratio=4, torque_Nm=20)),
    (RED, "devri iki katına çıkar", dict(speed_rpm=2900)),
    (RED, "use 1020 steel for the shafts", dict(shaft_material="1020")),
    (SL, "düz yolu 150 mm yap, 15 N itsin", dict(stroke_mm=150, max_load_N=15)),
]


def _same(a, b):
    return a == b or (isinstance(b, (int, float)) and isinstance(a, (int, float)) and abs(a - b) < 1e-6)


def run_edits():
    import pipeline
    fails = 0
    for prev, text, want in EDIT_CASES:
        t0 = time.time()
        edit = pipeline.llm_extract.extract_edit(text, prev)
        if want == "new":
            bad = {"is_edit": (edit["is_edit"], False)} if edit["is_edit"] else {}
        else:
            spec, note = pipeline.apply_edit(prev, edit, text) if edit["is_edit"] else (None, "not an edit")
            if want == "ask":
                bad = {} if spec is None and edit["is_edit"] else {"expected a question": (edit, note)}
            elif spec is None:
                bad = {"edit": (edit, note)}
            else:
                bad = {k: (spec.get(k), v) for k, v in want.items() if not _same(spec.get(k), v)}
        fails += bool(bad)
        print(f"{'OK  ' if not bad else 'FAIL'} ({time.time() - t0:.1f} s) [edit] {text}" + (f"\n      {bad}" if bad else ""))
    return fails


def main():
    fails = 0
    for text, want in CASES:
        t0 = time.time()
        spec, _ = force_backstop(text, input_turn_backstop(text, extract_spec(text))[0])
        bad = {k: (spec.get(k), v) for k, v in want.items()
               if not (spec.get(k) == v or (isinstance(v, (int, float)) and isinstance(spec.get(k), (int, float))
                                             and abs(spec.get(k) - v) < 1e-6))}
        fails += bool(bad)
        print(f"{'OK  ' if not bad else 'FAIL'} ({time.time() - t0:.1f} s) {text}" + (f"\n      {bad}" if bad else ""))
    edit_fails = run_edits()
    total = len(CASES) + len(EDIT_CASES)
    print(f"\n{total - fails - edit_fails}/{total} correct")
    return fails + edit_fails


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
