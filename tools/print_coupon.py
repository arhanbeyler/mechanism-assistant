"""
B8c (preparation): builds the print-in-place clearance coupon (printfit.py) in Fusion through the add-in's local
bridge and exports it as one STL for the slicer.

Usage (Fusion running with the MechanismAssistant add-in, from the repository root):
    python tools/print_coupon.py                   clearances 0.15 ... 0.5 mm -> output/clearance_coupon.stl
    python tools/print_coupon.py 0.2 0.25 0.3      only these
Print it as it lies (every part stands on the bed, no supports), then try to turn each ring by its tab while
holding the pin's knob: the smallest clearance whose ring turns freely is the one for print-in-place pins.
"""

import json
import os
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URL = "http://127.0.0.1:8765/exec"

CODE = '''
import os
pipeline, fourbar = reload_modules()
import importlib, printfit
importlib.reload(printfit)
cs = {cs!r} or printfit.CLEARANCES
des = fourbar.build(printfit.coupon(cs))
path = os.path.join(ROOT, "output", "clearance_coupon.stl")
os.makedirs(os.path.dirname(path), exist_ok=True)
opts = des.exportManager.createSTLExportOptions(des.rootComponent, path)
opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
des.exportManager.execute(opts)
print("exported", path, "clearances", cs)
'''


def main():
    token = open(os.path.join(ROOT, ".bridge_token")).read().strip()
    cs = [float(a) for a in sys.argv[1:]]
    req = urllib.request.Request(URL, data=CODE.format(cs=cs).encode(), headers={"X-Token": token}, method="POST")
    with urllib.request.urlopen(req, timeout=300) as resp:
        d = json.loads(resp.read())
    print((d.get("value") or d.get("error", "")).strip())


if __name__ == "__main__":
    main()
