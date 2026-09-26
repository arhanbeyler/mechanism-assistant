"""
Hub compliance calibration (B7g).

Model: a plate held at a small hole (radius r) has rotational stiffness k = C E t r^2.
Tip deflection due to the hub: delta_hub = F L^2 / (C E t r^2).
With the FEA (fea_local):  delta_hub = delta_FEA - delta_bending - delta_shear  ->  C = F L^2 / (E t r^2 delta_hub)
If C comes out constant the model is physically right; the result feeds the deflection check in strength.py.

Run (in the project folder):  python -u tools/hub_calibration.py
Note: E does not change C (delta ~ 1/E), so a sweep with one material is enough.
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
    if w < 2 * d + 2:                       # at least 1 mm of material beside the hole
        continue
    t0 = time.time()
    r = fea_local.compare(L, w, T, d, F, E, NU)
    hand = r["hand"]
    d_hub = r["fea"]["tip_delta"] - hand["delta"]
    C = F * L ** 2 / (E * T * (d / 2) ** 2 * d_hub) if d_hub > 0 else float("nan")
    row = {"L": L, "w": w, "d": d, "fea": r["fea"]["tip_delta"], "eb": hand["delta_bending"],
           "shear": hand["delta_shear"], "hub": d_hub, "C": C, "s": round(time.time() - t0, 1)}
    rows.append(row)
    print(f"L={L:3d} w={w:2d} d={d}: FEA {row['fea']:.4f}  EB {row['eb']:.4f}  shear {row['shear']:.4f}  "
          f"hub {d_hub:.4f} mm  ->  C = {C:.3f}   ({row['s']} s)", flush=True)

out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output", "hub_calibration.json")
os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out, "w", encoding="utf-8") as fh:
    json.dump(rows, fh, indent=1)
print("saved:", out)
