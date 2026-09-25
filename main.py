"""
Tek komut (terminal surumu): dogal dil -> LLM -> sentez -> kontrol -> mechanism.json

Kullanim:
    python main.py "30 mm krankli, 60 derece salinan bir mekanizma"
Sonra Fusion'da DortCubuk script'ini calistir (ya da Mekanizma Asistani panelini kullan).
"""

import json
import os
import sys

import kinematics as kin
from pipeline import design_from_text

ROOT = os.path.dirname(os.path.abspath(__file__))


def main(text):
    print(f"\n\"{text}\"")
    r = design_from_text(text, on_status=lambda s: print("  ..." + s))
    print(f"  LLM -> {r['spec']}")

    if "lengths" in r:
        L = r["lengths"]
        print("\n  " + "  ".join(f"{k} = {L[k]:.2f} mm" for k in "abcd"))
        for c in r["checks"]:
            print(f"  {'✓' if c['ok'] else '✗'} {c['name']}: {c['detail']}")

        path = os.path.join(ROOT, "output", "mechanism.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(r["fusion"], fh, indent=2)
        print(f"  Kaydedildi: {path}")

    print(f"\n{'✓' if r['status'] == 'ok' else '✗'} {r['message']}")
    return {"ok": 0, "missing": 1, "unsupported": 1, "failed": 2}[r["status"]]


if __name__ == "__main__":
    sentence = " ".join(sys.argv[1:]) or input("Mekanizmayi tarif edin: ")
    sys.exit(main(sentence))
