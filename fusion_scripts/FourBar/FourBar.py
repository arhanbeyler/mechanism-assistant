"""
Builds the four-bar linkage from output/mechanism.json (written by main.py).
The drawing code is shared: fusion_lib/fourbar.py (the panel uses the same code).
"""

import importlib
import json
import os
import sys
import traceback

import adsk.core

# realpath: resolves the shortcut (junction) in Fusion's script folder -> the project folder
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import fusion_lib.fourbar as fourbar  # noqa: E402


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        importlib.reload(fourbar)  # the latest code without restarting Fusion

        with open(os.path.join(ROOT, "output", "mechanism.json"), encoding="utf-8") as fh:
            mech = json.load(fh)
        fourbar.build(mech)

        L = mech["lengths_mm"]
        warning = "" if mech.get("passed_checks") else \
            "WARNING: this design did NOT pass the kinematic checks!\n\n"
        ui.messageBox(
            warning + "Four-bar linkage built!\n\n"
            f"Crank a = {L['a']:.2f} mm\nCoupler b = {L['b']:.2f} mm\n"
            f"Rocker c = {L['c']:.2f} mm\nGround d = {L['d']:.2f} mm\n\n"
            "To try it: drag the crank with the mouse,\n"
            "or right-click the 'Ground-Crank (O2)' joint -> Animate Joint."
        )
    except Exception:
        if ui:
            ui.messageBox("Error:\n{}".format(traceback.format_exc()))
