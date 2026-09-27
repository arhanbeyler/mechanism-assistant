"""
Animated GIFs of the drawn designs for the README: builds a module's default design in Fusion through the add-in's
local bridge, turns its input joint step by step (a limited joint between its limits) or shows its frames in
turn (springs, snap-fit, living hinge), saves a viewport image per step and joins them with Pillow.

Usage (Fusion running with the MechanismAssistant add-in, from the repository root):
    python tools/make_gifs.py                    the README set -> docs/img/anim/<name>.gif
    python tools/make_gifs.py fourbar spring     only these
Each build replaces the drawing in the add-in's document.  Cams, Geneva wheels and ratchets are driven from
tables by the add-in itself and are not in the set.
"""

import json
import os
import sys
import urllib.request

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import pipeline  # noqa: E402

URL = "http://127.0.0.1:8765/exec"
OUT = os.path.join(ROOT, "docs", "img", "anim")
TMP = os.path.join(ROOT, "output", "gif_frames")
SET = ["fourbar", "slider_crank", "dwell", "quick_return", "planetary", "worm_gear", "press", "spring",
       "torsion_spring", "snap_fit", "flexure"]
STEPS = 30
SIZE = (720, 480)

CODE = '''
import math, os
pipeline, fourbar = reload_modules()
kind, steps, tmp, size = {kind!r}, {steps}, {tmp!r}, {size!r}
if kind == "fourbar":
    r = pipeline.design_from_params(crank_length_mm=30, output_swing_deg=60)
else:
    kw = {{p["key"]: p["default"] for p in pipeline.MODULES[kind]["params"]}}
    kw = {{a: (None if v == 0 else v) for a, v in kw.items()}}
    r = pipeline.design_from_params(kind=kind, **kw)
des = fourbar.build(r["fusion"])
root = des.rootComponent
root.isJointsFolderLightBulbOn = False
vp = app.activeViewport
cam = vp.camera
cam.isSmoothTransition = False
cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType     # view extents then mean what they say
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
vp.camera = cam
vp.fit()
os.makedirs(tmp, exist_ok=True)
INPUT = ("Ground-Pinion", "Ground-Cam", "Ground-Shaft", "Ground-Input", "Ground-Driver", "Ground-Ratchet", "Ground-Screw")
joint = next((j for j in root.asBuiltJoints if j.name.startswith(("Ground-Crank", "Ground-Handle")) or j.name in INPUT), None)
frames = sorted([o for o in root.occurrences if o.component.name.split(" ")[1:2] and o.component.name.split(" ")[1].isdigit()
                 and o.component.name.split(" ")[2:3] and o.component.name.split(" ")[2].startswith("(")],
                key=lambda o: int(o.component.name.split(" ")[1]))


def frame_all(boxes):
    """Fit the camera to every box (the whole motion): a temporary sketch holds the union's corners during the fit."""
    lo = [min(b.minPoint.x for b in boxes), min(b.minPoint.y for b in boxes), min(b.minPoint.z for b in boxes)]
    hi = [max(b.maxPoint.x for b in boxes), max(b.maxPoint.y for b in boxes), max(b.maxPoint.z for b in boxes)]
    sk = root.sketches.add(root.xYConstructionPlane)
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            for z in (lo[2], hi[2]):
                sk.sketchPoints.add(sk.modelToSketchSpace(adsk.core.Point3D.create(x, y, z)))
    vp.fit()
    sk.deleteMe()


names = []
if frames:
    for o in frames:
        o.isLightBulbOn = True
    frame_all([root.boundingBox])
    seq = list(range(len(frames))) + list(range(len(frames) - 2, 0, -1))
    for i, k in enumerate(seq):
        for j, o in enumerate(frames):
            o.isLightBulbOn = (j == k)
        vp.refresh()
        names.append(os.path.join(tmp, f"{{kind}}_{{i:03d}}.png"))
        vp.saveAsImageFile(names[-1], *size)
    for j, o in enumerate(frames):
        o.isLightBulbOn = (j == 0)
elif joint:
    mo = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    lim = mo.rotationLimits
    a0 = mo.rotationValue
    if lim.isMinimumValueEnabled and lim.isMaximumValueEnabled:
        lo, hi = lim.minimumValue, lim.maximumValue
        vals = [lo + (hi - lo) * (1 - math.cos(2 * math.pi * i / steps)) / 2 for i in range(steps)]
    else:
        vals = [a0 + 2 * math.pi * i / steps for i in range(steps)]
    boxes = []
    for v in vals[::3]:
        mo.rotationValue = v
        boxes += [o.boundingBox for o in root.occurrences]
    mo.rotationValue = vals[0]
    frame_all(boxes)
    for i, v in enumerate(vals):
        mo.rotationValue = v
        vp.refresh()
        names.append(os.path.join(tmp, f"{{kind}}_{{i:03d}}.png"))
        vp.saveAsImageFile(names[-1], *size)
    mo.rotationValue = a0
root.isJointsFolderLightBulbOn = True
print(json.dumps(names))
'''


def capture(kind, token):
    code = "import json\n" + CODE.format(kind=kind, steps=STEPS, tmp=TMP, size=SIZE)
    req = urllib.request.Request(URL, data=code.encode(), headers={"X-Token": token}, method="POST")
    with urllib.request.urlopen(req, timeout=300) as resp:
        d = json.loads(resp.read())
    if d.get("error"):
        raise RuntimeError(d["error"][-400:])
    return json.loads(d["value"].strip().splitlines()[-1])


def to_gif(kind, names, frame_ms):
    frames = [Image.open(n).convert("RGB") for n in names]
    frames = [f.resize((SIZE[0] * 2 // 3, SIZE[1] * 2 // 3), Image.LANCZOS) for f in frames]
    pal = frames[0].quantize(colors=128, method=Image.Quantize.MEDIANCUT)
    frames = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{kind}.gif")
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=frame_ms, loop=0, optimize=True)
    for n in names:
        os.remove(n)
    return path


def main():
    token = open(os.path.join(ROOT, ".bridge_token")).read().strip()
    for kind in sys.argv[1:] or SET:
        names = capture(kind, token)
        if not names:
            print(kind, "nothing moves")
            continue
        path = to_gif(kind, names, 70 if len(names) >= STEPS else 140)
        print(kind, len(names), "frames ->", os.path.relpath(path, ROOT), f"{os.path.getsize(path) // 1024} kB")


if __name__ == "__main__":
    main()
