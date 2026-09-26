"""
LLM evaluation set: run it whenever a prompt changes, so that fixing one case does not break another
(example: fixing "45° cevirince" once broke "30 mm krank, 60° salinim").
The sentences are mostly Turkish (the user's language) plus a few English ones (the panel examples).

Usage (in the project folder, with Ollama running):  python -u tools/llm_eval.py
The LLM output and the code safety net (pipeline.input_turn_backstop) are tested together.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from llm_extract import extract_spec  # noqa: E402
from pipeline import input_turn_backstop  # noqa: E402

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
    # English: the panel's example sentences
    ("30 mm crank, 60° swing, 5 kg at the tip, PETG",
     dict(mechanism_type="crank_rocker", crank_length_mm=30, output_swing_deg=60, material="PETG")),
    ("1.5 kW 1450 rpm motor, 1:4 gearbox", dict(mechanism_type="reducer", gear_ratio=4, power_kW=1.5, speed_rpm=1450)),
    ("turn a 4 cm arm 45° and the other arm turns 120°",
     dict(mechanism_type="rocker_rocker", crank_length_mm=40, input_swing_deg=45, output_swing_deg=120)),
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


def main():
    fails = 0
    for text, want in CASES:
        t0 = time.time()
        spec, _ = input_turn_backstop(text, extract_spec(text))
        bad = {k: (spec.get(k), v) for k, v in want.items()
               if not (spec.get(k) == v or (isinstance(v, (int, float)) and isinstance(spec.get(k), (int, float))
                                             and abs(spec.get(k) - v) < 1e-6))}
        fails += bool(bad)
        print(f"{'OK  ' if not bad else 'FAIL'} ({time.time() - t0:.1f} s) {text}" + (f"\n      {bad}" if bad else ""))
    print(f"\n{len(CASES) - fails}/{len(CASES)} correct")
    return fails


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
