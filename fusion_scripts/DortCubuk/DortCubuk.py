"""
output/mechanism.json dosyasindan (main.py uretir) dort cubuk mekanizmasini kurar.
Cizim kodu ortak modulde: fusion_lib/fourbar.py (panel de ayni kodu kullanir).
"""

import importlib
import json
import os
import sys
import traceback

import adsk.core

# realpath: Fusion script klasorundeki kisayolu (junction) cozer -> proje klasoru
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import fusion_lib.fourbar as fourbar  # noqa: E402


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        importlib.reload(fourbar)  # kod degistiyse Fusion'u yeniden baslatmadan guncel hali

        with open(os.path.join(ROOT, "output", "mechanism.json"), encoding="utf-8") as fh:
            mech = json.load(fh)
        fourbar.build(mech)

        L = mech["lengths_mm"]
        warning = "" if mech.get("passed_checks") else \
            "UYARI: Bu tasarim kinematik kontrollerden GECMEDI!\n\n"
        ui.messageBox(
            warning + "Dort cubuk mekanizmasi kuruldu!\n\n"
            f"Krank a = {L['a']:.2f} mm\nBiyel b = {L['b']:.2f} mm\n"
            f"Kol c = {L['c']:.2f} mm\nSabit d = {L['d']:.2f} mm\n\n"
            "Denemek icin: Krank'i fareyle tutup surukle,\n"
            "ya da 'Sabit-Krank (O2)' joint'ine sag tik -> Animate Joint."
        )
    except Exception:
        if ui:
            ui.messageBox("Hata:\n{}".format(traceback.format_exc()))
