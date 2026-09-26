"""
Regression smoke test in Fusion: builds every module's default design (the slider defaults, plus a four-bar)
through the add-in's local bridge, one request per module, and reports ok / design status / error.

Usage (Fusion running with the MechanismAssistant add-in, from the repository root):
    python tools/fusion_smoke.py              every module
    python tools/fusion_smoke.py geneva cam   only these
Each build replaces the drawing in the add-in's document.
"""

import json
import os
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import pipeline  # noqa: E402

URL = "http://127.0.0.1:8765/exec"

CODE = '''
import time
pipeline, fourbar = reload_modules()
kind = {kind!r}
try:
    if kind == "fourbar":
        r = pipeline.design_from_params(crank_length_mm=30, output_swing_deg=60)
    else:
        kw = {{p["key"]: p["default"] for p in pipeline.MODULES[kind]["params"]}}
        kw = {{a: (None if v == 0 else v) for a, v in kw.items()}}
        r = pipeline.design_from_params(kind=kind, **kw)
    if r["status"] != "ok":
        print(kind, "design", r["status"])
    else:
        t = time.time()
        des = fourbar.build(r["fusion"])
        print(kind, "ok", des.rootComponent.occurrences.count, "occurrences", round(time.time() - t), "s")
except Exception as e:
    print(kind, "ERROR", str(e)[:200])
'''


def run(kind, token):
    req = urllib.request.Request(URL, data=CODE.format(kind=kind).encode(), headers={"X-Token": token}, method="POST")
    with urllib.request.urlopen(req, timeout=300) as resp:
        d = json.loads(resp.read())
    return (d.get("value") or d.get("error", "")).strip()


def main():
    token = open(os.path.join(ROOT, ".bridge_token")).read().strip()
    kinds = sys.argv[1:] or ["fourbar"] + list(pipeline.MODULES)
    bad = 0
    for k in kinds:
        line = run(k, token)
        print(line)
        bad += " ok " not in f" {line} "
    print(f"\n{len(kinds) - bad}/{len(kinds)} built")
    return bad


if __name__ == "__main__":
    sys.exit(main())
