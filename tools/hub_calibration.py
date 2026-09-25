"""
Gobek esnekligi kalibrasyonu (B7g).

Model: kucuk bir delikten (yaricap r) sabitlenen plakanin donme rijitligi k = C E t r^2.
Gobek kaynakli uc sehimi: delta_hub = F L^2 / (C E t r^2).
FEA (fea_local) ile:  delta_hub = delta_FEA - delta_egilme - delta_kayma  ->  C = F L^2 / (E t r^2 delta_hub)
C sabit cikarsa model fiziksel olarak dogru; sonuc strength.py'deki sehim kontrolune eklenir.

Calistirma (proje klasorunde):  python -u tools/hub_calibration.py
Not: E sonucu olceklemez (delta ~ 1/E), bu yuzden tek malzemeyle tarama yeterli.
"""

import itertools
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fea_local  # noqa: E402

E, NU, F, T = 2000.0, 0.38, 50.0, 3.0

rows = []
for L, w, d in itertools.product((40, 60, 100), (9, 15, 29), (3, 4, 6)):
    if w < 2 * d + 2:                       # delik kenarinda en az 1 mm et
        continue
    t0 = time.time()
    r = fea_local.compare(L, w, T, d, F, E, NU)
    hand = r["hand"]
    d_hub = r["fea"]["tip_delta"] - hand["delta"]
    C = F * L ** 2 / (E * T * (d / 2) ** 2 * d_hub) if d_hub > 0 else float("nan")
    row = {"L": L, "w": w, "d": d, "fea": r["fea"]["tip_delta"], "eb": hand["delta_bending"],
           "shear": hand["delta_shear"], "hub": d_hub, "C": C, "s": round(time.time() - t0, 1)}
    rows.append(row)
    print(f"L={L:3d} w={w:2d} d={d}: FEA {row['fea']:.4f}  EB {row['eb']:.4f}  kayma {row['shear']:.4f}  "
          f"gobek {d_hub:.4f} mm  ->  C = {C:.3f}   ({row['s']} s)", flush=True)

out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output", "hub_calibration.json")
os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out, "w", encoding="utf-8") as fh:
    json.dump(rows, fh, indent=1)
print("kaydedildi:", out)
