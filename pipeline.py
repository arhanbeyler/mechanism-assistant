"""
Tasarim akisi (ekrana yazdirmadan, sadece veri dondurur):
    cumle -> LLM (JSON) -> kinematik sentez -> simulasyon -> kontroller

Hem terminal (main.py) hem Fusion paneli bunu kullanir.
Sadece standart kutuphane: Fusion'in kendi Python'unda da calisir.
"""

import re

import cams
import function_gen
import gears
import interference
import kinematics as kin
import shafts
import springs
import reducer
import strength
from llm_extract import extract_spec


# C0: moduller kendi kaydiricilarini tanimlar; panel bunlardan otomatik kurar.
# (Dort cubuk panelde ozel kaydiricilarla gelir: renkli salinim bolgesi, giris hareketi vb.)
MODULES = {
    "reducer": {
        "title": "Reducer (system)",
        "params": [
            {"key": "torque_Nm", "label": "Input torque", "unit": "N·m", "min": 1, "max": 400, "step": 1, "default": 20},
            {"key": "speed_rpm", "label": "Input speed", "unit": "rpm", "min": 100, "max": 3000, "step": 10, "default": 1450},
            {"key": "gear_ratio", "label": "Ratio", "unit": "", "min": 1.2, "max": 6, "step": 0.1, "default": 3},
            {"key": "shaft_material", "label": "Shaft steel", "default": "1045",
             "options": [["1045", "AISI 1045 CD"], ["1020", "AISI 1020 CD"]]},
            {"key": "feature", "label": "Gear on shaft", "default": "keyseat",
             "options": [["keyseat", "Key"], ["ring_groove", "Retaining ring"], ["round_shoulder", "Rounded shoulder"]]},
        ],
    },
    "spur_gear": {
        "title": "Düz dişli",
        "params": [
            {"key": "gear_ratio", "label": "Çevrim oranı", "unit": "", "min": 1, "max": 8, "step": 0.1, "default": 3},
            {"key": "torque_Nm", "label": "Giriş torku", "unit": "N·m", "min": 0.1, "max": 50, "step": 0.1, "default": 2},
            {"key": "speed_rpm", "label": "Giriş devri", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "belirtilmedi"},
        ],
    },
    "spring": {
        "title": "Basma yayı",
        "params": [
            {"key": "spring_force_N", "label": "En büyük kuvvet", "unit": "N", "min": 1, "max": 500, "step": 1, "default": 100},
            {"key": "spring_deflection_mm", "label": "Bu kuvvette çökme", "unit": "mm", "min": 1, "max": 100, "step": 1, "default": 20},
            {"key": "spring_taper", "label": "Konik oranı (D₂/D₁)", "unit": "", "min": 0.3, "max": 1.0, "step": 0.05,
             "default": 1.0, "one_label": "silindirik"},
            {"key": "outer_diameter_max_mm", "label": "Dış çap sınırı", "unit": "mm", "min": 0, "max": 100, "step": 1,
             "default": 0, "zero_label": "serbest"},
            {"key": "show_plates", "label": "Fusion görünümü", "default": "no",
             "options": [["no", "Sadece yay"], ["yes", "Plakalarla (sıkışma)"]]},
        ],
    },
    "cam": {
        "title": "Kam (makaralı itici)",
        "params": [
            {"key": "cam_rise_mm", "label": "Kalkma (strok)", "unit": "mm", "min": 2, "max": 60, "step": 1, "default": 20},
            {"key": "rise_deg", "label": "Yükselme açısı", "unit": "°", "min": 20, "max": 200, "step": 5, "default": 120},
            {"key": "dwell_top_deg", "label": "Üstte bekleme", "unit": "°", "min": 0, "max": 180, "step": 5, "default": 60},
            {"key": "return_deg", "label": "Dönüş açısı", "unit": "°", "min": 20, "max": 200, "step": 5, "default": 120},
            {"key": "motion_law", "label": "Hareket yasası", "default": "cycloidal",
             "options": [["cycloidal", "Sikloidal"], ["harmonic", "Basit harmonik"], ["poly345", "3-4-5 polinom"]]},
        ],
    },
    "shaft": {
        "title": "Mil (yorulma)",
        "params": [
            {"key": "torque_Nm", "label": "Tork", "unit": "N·m", "min": 1, "max": 500, "step": 1, "default": 100},
            {"key": "bending_moment_Nm", "label": "Eğilme momenti", "unit": "N·m", "min": 0, "max": 500, "step": 1, "default": 60},
            {"key": "feature", "label": "Gerilme yığılması", "default": "keyseat",
             "options": [["keyseat", "Kama yuvası"], ["sharp_shoulder", "Keskin omuz"], ["round_shoulder", "Yuvarlatılmış omuz"],
                         ["ring_groove", "Segman kanalı"], ["plain", "Düz mil"]]},
            {"key": "shaft_material", "label": "Çelik", "default": "1045",
             "options": [["1045", "AISI 1045 CD"], ["1020", "AISI 1020 CD"]]},
            {"key": "safety_n", "label": "Yorulma emniyeti", "unit": "", "min": 1.2, "max": 4, "step": 0.1, "default": 2},
            {"key": "speed_rpm", "label": "Devir", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 300,
             "zero_label": "verilmedi"},
            {"key": "span_mm", "label": "Rulman aralığı", "unit": "mm", "min": 0, "max": 400, "step": 10, "default": 0,
             "zero_label": "otomatik"},
        ],
    },
}


def max_feasible_swing(a, psi_start):
    """Istenen salinim saglanamiyorsa, tasarim hedefini (45-135, kompakt) gecen en buyuk salinim."""
    for psi in range(int(psi_start) - 1, 0, -1):
        m = kin.synthesize_crank_rocker(a, psi)
        if m["worst_dev"] <= 45.0:
            return psi
    return None


def max_feasible_q(a, psi, q_max=1.5, step=0.05):
    """Iletim acisi tasarim hedefini (45-135) saglayan en buyuk zaman orani. Yoksa None."""
    q = q_max
    while q > 1.0 + 1e-9:
        if kin.synthesize_quick_return(a, psi, q)["worst_dev"] <= 45.0:
            return round(q, 2)
        q -= step
    return None


def swing_band(Q=1.0, step=5, n=720):
    """
    Salinim kaydiricisinin renkli bolgeleri: [(psi, seviye), ...]
      2: iletim acisi 45-135 icinde (kompakt cozum var)
      1: 40-140 sinirinda gecer (ama mekanizma buyuk olabilir)
      0: gecmez
    Olcekten bagimsiz: acilar krank boyuna bagli degil (a=10 ile hesaplanir).
    """
    band = []
    for psi in range(step, 180, step):
        m = (kin.synthesize_crank_rocker(10.0, psi) if Q <= 1.0
             else kin.synthesize_quick_return(10.0, psi, Q, n=n))
        if m["worst_dev"] <= 45.0:
            level = 2
        elif kin.MU_MIN_DEG <= m["mu_min"] and m["mu_max"] <= kin.MU_MAX_DEG:
            level = 1
        else:
            level = 0
        band.append((psi, level))
    return band


_INPUT_TURN = re.compile(
    r"(\d+(?:[.,]\d+)?)\s*(?:°|derece|derecelik)\s*(?:\S+\s+)?"
    r"(?:çevir|döndür|dönün|döndüğ|itin|it(?:ti)?ğ|hareket ettir|oynat)", re.IGNORECASE)


# Speed backstop: "1500 rpm", "1450 devirli", "960 d/d" -> speed_rpm when the LLM leaves it empty
_SPEED = re.compile(r"(\d+(?:[.,]\d+)?)\s*(?:rpm|d/d|dev/dak|devir(?:de|li)?)(?![a-zçğıöşü])", re.IGNORECASE)
_SPEED_KINDS = ("reducer", "spur_gear", "shaft")


def input_turn_backstop(text, spec):
    """Guvenlik agi: "X° cevirince/dondurunce/itince" kalibi varsa giris kolu sinirli acida doner.
    LLM bunu kacirirsa (ornegin '45°' yaziminda) sayi DOGRUDAN kullanicinin cumlesinden okunur
    (uydurma degil). Ayni sekilde devir (rpm) de cumleden okunur. Dondurur: (spec, not ya da None)."""
    sp = _SPEED.search(text)
    if sp and spec.get("speed_rpm") is None and spec.get("mechanism_type") in _SPEED_KINDS:
        spec = dict(spec, speed_rpm=float(sp.group(1).replace(",", ".")))
    m = _INPUT_TURN.search(text)
    if not m or spec.get("input_swing_deg") is not None:
        return spec, None
    angle = float(m.group(1).replace(",", "."))
    if not 0 < angle < 360:
        return spec, None
    fixed = dict(spec, mechanism_type="rocker_rocker", input_swing_deg=angle,
                 quick_return=False, time_ratio=None)
    return fixed, f"Giriş kolunun {angle:g}° döndüğü cümleden okundu (sınırlı açılı mekanizma)."


def design_from_text(text, on_status=None):
    """
    Cumleden tasarim. Dondurur: dict(status, message, spec, ...)
      status: "ok" | "failed" (kontrollerden gecmedi) | "missing" | "unsupported"
    on_status(str): ilerleme bildirimi icin istege bagli geri cagirma.
    """
    say = on_status or (lambda s: None)
    say("LLM cümleyi yorumluyor...")
    spec, note = input_turn_backstop(text, extract_spec(text))
    result = design_from_spec(spec, on_status)
    if note and "message" in result:
        result["message"] = note + " " + result["message"]
    result["text"] = text
    return result


def design_from_params(crank_length_mm=30, output_swing_deg=60, time_ratio=1.0,
                       max_load_N=None, material=None, input_swing_deg=360, kind="fourbar",
                       gear_ratio=None, torque_Nm=None, speed_rpm=None,
                       spring_force_N=None, spring_deflection_mm=None, spring_taper=None, outer_diameter_max_mm=None,
                       cam_rise_mm=None, rise_deg=None, dwell_top_deg=None, return_deg=None, motion_law=None,
                       bending_moment_Nm=None, feature=None, shaft_material=None, safety_n=None, show_plates=None,
                       span_mm=None, power_kW=None):
    """Kaydiricilar icin: LLM'siz, dogrudan sayilardan tasarim.
    kind: "fourbar", "spur_gear", "spring" ya da "cam" (C0: modul secimi).
    input_swing_deg >= 360: giris tam tur (krank-kol); daha az: sinirli acida giris kolu (B9)."""
    if kind == "reducer":
        return design_from_spec({"mechanism_type": "reducer", "torque_Nm": torque_Nm, "power_kW": power_kW,
                                 "speed_rpm": speed_rpm, "gear_ratio": gear_ratio, "material": material,
                                 "shaft_material": shaft_material, "feature": feature, "safety_n": safety_n})
    if kind == "shaft":
        return design_from_spec({"mechanism_type": "shaft", "torque_Nm": torque_Nm,
                                 "bending_moment_Nm": bending_moment_Nm, "feature": feature,
                                 "shaft_material": shaft_material, "safety_n": safety_n,
                                 "speed_rpm": speed_rpm or None, "span_mm": span_mm or None})
    if kind == "cam":
        return design_from_spec({"mechanism_type": "cam", "cam_rise_mm": cam_rise_mm, "rise_deg": rise_deg,
                                 "dwell_top_deg": dwell_top_deg, "return_deg": return_deg,
                                 "motion_law": motion_law or "cycloidal", "roller_radius_mm": None})
    if kind == "spring":
        return design_from_spec({"mechanism_type": "spring", "spring_force_N": spring_force_N,
                                 "spring_deflection_mm": spring_deflection_mm, "spring_taper": spring_taper,
                                 "outer_diameter_max_mm": outer_diameter_max_mm or None, "material": material,
                                 "show_plates": show_plates})
    if kind == "spur_gear":
        return design_from_spec({"mechanism_type": "spur_gear", "gear_ratio": gear_ratio,
                                 "torque_Nm": torque_Nm, "speed_rpm": speed_rpm or None, "material": material})
    rr = input_swing_deg < 360
    return design_from_spec({
        "mechanism_type": "rocker_rocker" if rr else "crank_rocker", "crank_length_mm": crank_length_mm,
        "output_swing_deg": output_swing_deg, "quick_return": (not rr) and time_ratio > 1.0,
        "time_ratio": time_ratio if (not rr and time_ratio > 1.0) else None,
        "input_swing_deg": input_swing_deg if rr else None,
        "max_load_N": max_load_N or None, "material": material})


def design_from_spec(spec, on_status=None):
    """Yapilandirilmis istekten (LLM ciktisi ya da kaydiricilar) tasarim."""
    say = on_status or (lambda s: None)
    result = {"spec": spec}

    mtype = spec["mechanism_type"]
    if mtype == "reducer":
        return _reducer(result, spec, say)
    if mtype == "spur_gear":
        return _spur_gear(result, spec, say)
    if mtype == "spring":
        return _spring(result, spec, say)
    if mtype == "cam":
        return _cam(result, spec, say)
    if mtype == "shaft":
        return _shaft(result, spec, say)
    if mtype not in ("crank_rocker", "rocker_rocker"):
        return dict(result, status="unsupported",
                    message="Şu an sadece dört çubuk mekanizmaları destekleniyor "
                            "(krank-kol ya da sınırlı açıda çalışan giriş kolu).")
    rr = mtype == "rocker_rocker"
    required = ((("crank_length_mm", "giriş kolu uzunluğu"), ("input_swing_deg", "giriş kolunun dönme açısı"),
                 ("output_swing_deg", "çıkış kolunun dönme açısı")) if rr else
                (("crank_length_mm", "krank uzunluğu"), ("output_swing_deg", "salınım açısı")))
    missing = [name for key, name in required if spec.get(key) is None]
    if missing:
        return dict(result, status="missing",
                    message=f"Tasarım için eksik bilgi: {', '.join(missing)}.")

    a, psi = spec["crank_length_mm"], spec["output_swing_deg"]
    if not 0 < psi < 180:
        return dict(result, status="missing",
                    message=f"Salınım açısı 0-180° arasında olmalı (verilen: {psi:g}°).")
    notes = []
    if rr:
        t_in = spec["input_swing_deg"]
        if not 0 < t_in < 360:
            return dict(result, status="missing",
                        message=f"Giriş açısı 0-360° arasında olmalı (verilen: {t_in:g}°).")
        say("Fonksiyon üreteci sentezi (Freudenstein)...")
        fg = _pick_function_generator(a, t_in, psi, spec)
        if fg is None:
            return dict(result, status="failed",
                        message=f"Giriş {t_in:g}° → çıkış {psi:g}° için uygun dört çubuk bulunamadı.")
        mech, frames = function_gen.frames_for(fg)
        mr = function_gen.measure_range(frames)
        grashof_ok, grashof_kind = kin.grashof(mech["a"], mech["b"], mech["c"], mech["d"])
        mu_ok = kin.MU_MIN_DEG <= mr["mu_min"] and mr["mu_max"] <= kin.MU_MAX_DEG
        in_ok = abs(mr["in_swing"] - t_in) < 0.5
        out_ok = abs(abs(mr["out_swing"]) - psi) <= 1.5
        err_ok = fg["err_deg"] <= 1.0
        passed = mu_ok and in_ok and out_ok and err_ok
        Q = 1.0
        meas = {"swing": abs(mr["out_swing"]), "in_swing": mr["in_swing"], "Q": 1.0,
                "mu_min": mr["mu_min"], "mu_max": mr["mu_max"]}
        checks = [
            {"name": "Mekanizma türü", "ok": True, "detail": grashof_kind},
            {"name": "Giriş dönmesi (simülasyon)", "ok": in_ok,
             "detail": f"{mr['in_swing']:.1f}° (hedef {t_in:g}°)"},
            {"name": "Çıkış dönmesi (simülasyon)", "ok": out_ok,
             "detail": f"{abs(mr['out_swing']):.1f}° (hedef {psi:g}°)"},
            {"name": "Yapısal hata (doğrusallık)", "ok": err_ok,
             "detail": f"en fazla {fg['err_deg']:.2f}° (sınır 1°)"},
            {"name": "İletim açısı", "ok": mu_ok,
             "detail": f"{mr['mu_min']:.1f}° – {mr['mu_max']:.1f}° "
                       f"(sınır {kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°)"},
        ]
    return _finish(result, spec, notes, say, mech, frames, meas, checks, passed, a, psi,
                   Q if rr else None, rr) if rr else _crank_rocker(result, spec, notes, say, a, psi)


def _reducer(result, spec, say):
    """E1: single-stage reducer (gear pair + two shafts + bearings + keys + housing), all coupled.
    Required: ratio, input speed, and input torque or motor power (nothing is invented)."""
    result["mechanism_type"] = "reducer"
    T, P, rpm, ratio = spec.get("torque_Nm"), spec.get("power_kW"), spec.get("speed_rpm"), spec.get("gear_ratio")
    notes = []
    if T is None and P is not None and rpm:
        T = 9550.0 * P / rpm
        notes.append(f"Input torque from motor power: T = 9550 P / n = {T:.1f} N·m.")
    missing = [n for v, n in ((ratio, "ratio"), (rpm, "input speed (rpm)"), (T, "input torque or motor power"))
               if v is None]
    if missing:
        return dict(result, status="missing", message=f"Missing for the reducer: {', '.join(missing)}.")
    ratio = float(ratio)
    if ratio < 1:
        ratio = 1 / ratio
    if not 1 < ratio <= 8:
        return dict(result, status="missing", message=f"A single spur stage needs a ratio of 1–8 (given: {ratio:g}).")
    mat_key = spec.get("material") if spec.get("material") in strength.MATERIALS else None
    if mat_key is None:
        mat_key = "steel"
        notes.append("Gear material not given: steel assumed.")
    feature = spec.get("feature") or "keyseat"
    shaft_mat = spec.get("shaft_material") or "1045"
    n = float(spec.get("safety_n") or 2.0)
    say("Reducer: gear pair -> shafts -> bearings -> keys -> housing (iterating until they fit)...")
    r, r_checks = reducer.design(float(T) * 1000.0, float(rpm), ratio, mat_key, shaft_mat, feature, n)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0, "detail": c["detail"]}
              for c in r_checks]
    result.update(checks=checks, notes=notes, target={"torque_Nm": float(T), "speed_rpm": float(rpm), "ratio": ratio})
    if r is None:
        bad = [c["name"] for c in checks if not c["ok"]]
        return dict(result, status="failed", message=" ".join(notes + [
            f"No reducer passes all checks (failing: {', '.join(bad)}). Try a stronger material, a smaller "
            f"torque or a different ratio."]))
    g, s1, s2 = r["gear"], r["shaft_in"], r["shaft_out"]
    notes.append("Losses neglected (output torque = ratio × input torque): conservative for the output side.")
    result.update(
        reducer=r, fusion={"type": "reducer", "passed_checks": True, **r},
        status="ok",
        message=" ".join(notes + [
            f"All checks passed. Gears m {g['module']:g}, z {g['z1']}/{g['z2']} (i = {g['ratio']:.3f}), "
            f"face {g['face_width']:g} mm, centre distance {g['center_distance']:g} mm. Shafts Ø{s1['d']:g} / "
            f"Ø{s2['d']:g} mm on bearings {s1['bearing']['name']} / {s2['bearing']['name']}. Output "
            f"{r['T2'] / 1000:.1f} N·m at {r['rpm2']:.0f} rpm ({r['power_W'] / 1000:.2f} kW)."]))
    return result


def _spur_gear(result, spec, say):
    """C9: duz disli cifti. Gerekli: cevrim orani ve giris torku (tork uydurulmaz)."""
    missing = [name for key, name in (("gear_ratio", "çevrim oranı"), ("torque_Nm", "giriş torku (N·m)"))
               if spec.get(key) is None]
    if missing:
        return dict(result, status="missing", mechanism_type="spur_gear",
                    message=f"Dişli tasarımı için eksik bilgi: {', '.join(missing)}.")
    ratio, T1 = float(spec["gear_ratio"]), float(spec["torque_Nm"]) * 1000.0
    if ratio < 1:
        ratio = 1 / ratio                     # 1:3 ve 3:1 ayni cift; pinyon her zaman giris
    if not 1 <= ratio <= 8:
        return dict(result, status="missing", mechanism_type="spur_gear",
                    message=f"Tek kademe düz dişli için oran 1–8 arasında olmalı (verilen: {ratio:g}).")
    notes = []
    mat_key = spec.get("material")
    if mat_key not in strength.MATERIALS:
        mat_key = "PLA"
        notes.append("Malzeme belirtilmediği için PLA (3B baskı) varsayıldı.")
    mat = strength.MATERIALS[mat_key]
    say("Dişli sentezi (Lewis eğilme, kavrama oranı)...")
    g, g_checks = gears.design(ratio, T1, mat, spec.get("speed_rpm"))
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0 or c["group"] == "Temas",
               "detail": c["detail"] + (f" · kullanım %{c['util'] * 100:.0f}" if c["group"] != "Temas" else "")}
              for c in g_checks]
    result.update(mechanism_type="spur_gear", checks=checks, notes=notes)
    if g is None:
        return dict(result, status="failed",
                    message=" ".join(notes + [f"{T1 / 1000:g} N·m giriş torku için {mat['name']} ile uygun dişli "
                                              f"bulunamadı. Öneri: daha dayanıklı malzeme ya da daha küçük tork."]))
    checks.insert(0, {"name": "Çevrim oranı", "ok": g["ratio_err_pct"] <= 2.0,
                      "detail": f"{g['z2']}/{g['z1']} = {g['ratio']:.3f} (hedef {ratio:g}, fark %{g['ratio_err_pct']:.1f})"})
    checks.insert(1, {"name": "Alttan kesme", "ok": g["z1"] >= gears.z_min(),
                      "detail": f"z₁ = {g['z1']} ≥ {gears.z_min():.1f}"})
    result.update(
        gear=g, target={"ratio": ratio, "torque_Nm": T1 / 1000},
        fusion={"type": "spur_gear", "passed_checks": True, **g},
        status="ok",
        message=" ".join(notes + [
            f"Tasarım tüm kontrollerden geçti. Modül {g['module']:g}, z = {g['z1']}/{g['z2']}, "
            f"diş genişliği {g['face_width']:g} mm, eksen mesafesi {g['center_distance']:g} mm, "
            f"miller Ø{g['bore1']:g}/Ø{g['bore2']:g} mm (çelik). Çıkış torku {T1 * g['ratio'] / 1000:.2f} N·m."]))
    return result


def _spring(result, spec, say):
    """C10: basma yayi (silindirik ya da konik). Gerekli: kuvvet ve cokme (uydurulmaz)."""
    result["mechanism_type"] = "spring"
    missing = [name for key, name in (("spring_force_N", "en büyük kuvvet"),
                                      ("spring_deflection_mm", "bu kuvvetteki çökme (mm)"))
               if spec.get(key) is None]
    if missing:
        return dict(result, status="missing", message=f"Yay tasarımı için eksik bilgi: {', '.join(missing)}.")
    F, y = float(spec["spring_force_N"]), float(spec["spring_deflection_mm"])
    notes = []
    taper = spec.get("spring_taper")
    if taper is None and spec.get("spring_shape") == "conical":
        taper = 0.5
        notes.append("Konik oranı belirtilmediği için küçük uç çapı büyük ucun yarısı alındı (D₂/D₁ = 0.5).")
    taper = min(1.0, max(0.3, float(taper))) if taper else 1.0
    if spec.get("material") == "steel" or spec.get("material") is None:
        mat = springs.SPRING_MATERIALS["music_wire"]
    else:
        mat = springs.SPRING_MATERIALS["music_wire"]
        notes.append("Yaylar çelik telden yapılır (plastik/alüminyum yay uygun değil); müzik teli varsayıldı.")
    say("Yay sentezi (Shigley: Bergsträsser, tam sıkışma, burkulma)...")
    geo, s_checks = springs.design(F, y, mat, taper, spec.get("outer_diameter_max_mm") or None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": c["detail"] + (f" · kullanım %{c['util'] * 100:.0f}" if c["group"] in ("Gerilme", "Burkulma") else "")}
              for c in s_checks]
    result.update(checks=checks, notes=notes, target={"force_N": F, "deflection_mm": y, "taper": taper})
    if geo is None:
        return dict(result, status="failed", message=" ".join(notes + [
            f"{F:g} N / {y:g} mm için uygun yay bulunamadı. Öneri: çökmeyi artırın, dış çap sınırını gevşetin "
            f"ya da kuvveti azaltın."]))
    kind = "konik" if taper < 1 else "silindirik"
    result.update(
        spring=dict(geo, material=mat["name"]),
        fusion={"type": "spring", "passed_checks": True, **geo, "show_plates": spec.get("show_plates") == "yes"},
        status="ok",
        message=" ".join(notes + [
            f"Tasarım tüm kontrollerden geçti ({kind}, {mat['name']}). Tel Ø{geo['d']:g} mm, ortalama çap "
            + (f"{geo['D1']:.1f} → {geo['D2']:.1f}" if taper < 1 else f"{geo['D1']:.1f}")
            + f" mm, {geo['Na']:g} aktif + 2 uç sarım, serbest boy {geo['L0']:.1f} mm, k = {geo['k']:.2f} N/mm."]))
    return result


def _shaft(result, spec, say):
    """C11b: donen mil, DE-Goodman. Gerekli: tork; egilme momenti ya da (radyal kuvvet + yatak araligi)."""
    result["mechanism_type"] = "shaft"
    notes = []
    T = spec.get("torque_Nm")
    M = spec.get("bending_moment_Nm")
    if M is None and spec.get("radial_force_N") is not None and spec.get("span_mm") is not None:
        M = spec["radial_force_N"] * spec["span_mm"] / 1000.0 / 4.0
        notes.append(f"Eğilme momenti, iki yataklı milin ortasına etkiyen kuvvetten hesaplandı: "
                     f"M = F·L/4 = {M:.1f} N·m (basit mesnetli kiriş varsayımı).")
    missing = [n for v, n in ((T, "tork (N·m)"), (M, "eğilme momenti ya da kuvvet + yatak aralığı")) if v is None]
    if missing:
        return dict(result, status="missing", message=f"Mil tasarımı için eksik bilgi: {', '.join(missing)}.")
    feature = spec.get("feature") or "keyseat"
    if not spec.get("feature"):
        notes.append("Gerilme yığılması belirtilmediği için kama yuvası varsayıldı (K_f = 2.14, K_fs = 3.0).")
    mat = spec.get("shaft_material") or "1045"
    if not spec.get("shaft_material"):
        notes.append("Çelik belirtilmediği için AISI 1045 soğuk çekilmiş varsayıldı.")
    n = float(spec.get("safety_n") or 2.0)
    rpm = spec.get("speed_rpm")
    if not rpm:
        notes.append("Devir verilmediği için rulman ömrü ve kritik hız kontrol edilmedi.")
    span = spec.get("span_mm")
    say("Mil tasarımı (DE-Goodman yorulma, rulman, kama, sehim)...")
    g, s_checks = shafts.design(float(M) * 1000.0, float(T) * 1000.0, mat, feature, n, span=span,
                                rpm=float(rpm) if rpm else None)
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0, "detail": c["detail"]} for c in s_checks]
    result.update(checks=checks, notes=notes,
                  target={"torque_Nm": float(T), "bending_moment_Nm": float(M), "n": n})
    if g is None:
        bad = [c["name"] for c in checks if not c["ok"]]
        return dict(result, status="failed", message=" ".join(notes + [
            f"Standart çaplar içinde uygun mil yok (geçmeyen: {', '.join(bad)}). "
            "Öneri: yükü azaltın, rulman aralığını kısaltın ya da daha dayanıklı çelik seçin."]))
    g["span_mm"] = span
    fname = shafts.FEATURES[feature][0]
    if g["raised"]:
        notes.append(f"Göbek kesiti için Ø{g['d_min']:.2f} mm yeterliydi; daha ince milde geçmeyen kontrol "
                     f"({', '.join(g['raised_by']).lower()}) yüzünden çap Ø{g['d']:g} mm'ye büyütüldü.")
    if rpm:
        notes.append("Kritik hıza sadece milin kütlesi katıldı; göbekteki dişli/kasnak kütlesi dahil değil.")
    b = g["bearing"]
    result.update(
        shaft=g, fusion={"type": "shaft", "passed_checks": True, **g},
        status="ok",
        message=" ".join(notes + [
            f"Tasarım tüm kontrollerden geçti. Mil Ø{g['d']:g} mm ({g['material']}, {fname}), rulmanlar "
            f"{b['name']} (Ø{b['bore']}×Ø{b['OD']}×{b['B']}). Yorulma emniyeti n_f = {g['nf']:.2f}, "
            f"akma emniyeti n_y = {g['ny']:.2f}."]))
    return result


LAW_NAMES = {"cycloidal": "sikloidal", "harmonic": "basit harmonik", "poly345": "3-4-5 polinom"}


def _cam(result, spec, say):
    """C11a: disk kam + makarali itici. Gerekli: kalkma ve yukselme acisi (uydurulmaz)."""
    result["mechanism_type"] = "cam"
    missing = [name for key, name in (("cam_rise_mm", "kalkma (strok, mm)"), ("rise_deg", "yükselme açısı"))
               if spec.get(key) is None]
    if missing:
        return dict(result, status="missing", message=f"Kam tasarımı için eksik bilgi: {', '.join(missing)}.")
    notes = []
    h, b1 = float(spec["cam_rise_mm"]), float(spec["rise_deg"])
    b3 = spec.get("return_deg")
    if b3 is None:
        b3 = b1
        notes.append(f"Dönüş açısı belirtilmediği için yükselmeyle aynı alındı ({b1:g}°).")
    b2 = float(spec.get("dwell_top_deg") or 0.0)
    law_ = spec.get("motion_law") or "cycloidal"
    if not spec.get("motion_law"):
        notes.append("Hareket yasası belirtilmediği için sikloidal seçildi (sonsuz sarsıntı yok).")
    say("Kam sentezi (basınç açısı, eğrilik)...")
    d, c_checks = cams.design(h, b1, b2, float(b3), law_, spec.get("roller_radius_mm"))
    checks = [{"name": f"{c['group']}: {c['name']}", "ok": c["util"] <= 1.0,
               "detail": c["detail"]} for c in c_checks]
    result.update(checks=checks, notes=notes,
                  target={"rise": h, "rise_deg": b1, "dwell_top_deg": b2, "return_deg": float(b3), "law": law_})
    if d is None:
        return dict(result, status="failed", message=" ".join(notes + [
            "Uygun kam bulunamadı. Öneri: yükselme/dönüş açılarını büyütün ya da açıların toplamını 360°'nin altında tutun."]))
    table = cams.svaj(h, b1, b2, float(b3), law_, n=360)
    result.update(
        cam=d, cam_curve=[(round(t, 1), round(s, 3)) for t, s, _, _ in table[::2]],
        fusion={"type": "cam", "passed_checks": True, **d, "s_table": [round(s, 4) for _, s, _, _ in table]},
        status="ok",
        message=" ".join(notes + [
            f"Tasarım tüm kontrollerden geçti ({LAW_NAMES[law_]}). Temel daire R_b = {d['Rb']:g} mm, makara "
            f"R = {d['Rf']:g} mm, kalkma {h:g} mm: {b1:g}° yükselme, {b2:g}° bekleme, {float(b3):g}° dönüş, "
            f"{d['dwell_bottom']:g}° altta bekleme."]))
    return result


def _pick_function_generator(a, t_in, psi, spec):
    """Kinematik adaylar arasindan, (yuk verildiyse) mukavemetten ve katman cakismasindan gecen ilki.
    Hicbiri gecmezse kinematik olarak en iyisi (kontroller nedenini gosterir)."""
    cands = function_gen.candidates(a, t_in, psi)
    if not cands:
        return function_gen.synthesize(a, t_in, psi)   # kinematik hedefi de tutmayan en iyi aday
    F, mat_key = spec.get("max_load_N"), spec.get("material")
    mat = strength.MATERIALS.get(mat_key) or strength.MATERIALS["PLA"]
    for fg in cands:
        mech, frames = function_gen.frames_for(fg)
        section = strength.DEFAULT_SECTION
        if F:
            forces = strength.analyze_forces(mech, frames, F)
            section, _ = strength.size_links(forces, {k: mech[k] for k in "abcd"}, mat)
            if section is None:
                continue
        if interference.choose_layer_order(mech, frames, section)[2]:
            return fg
    return cands[0]


def _crank_rocker(result, spec, notes, say, a, psi):
    # Zaman orani Q: kullanici verdiyse o; "hizli donus" deyip vermediyse MOTOR secer
    # (LLM asla sayi uydurmaz); hic bahsetmediyse Q = 1.
    Q = spec.get("time_ratio")
    if Q is not None:
        if not 1.0 <= Q <= 3.0:
            return dict(result, status="missing",
                        message=f"Zaman oranı Q, 1 ile 3 arasında olmalı (verilen: {Q:g}).")
    elif spec.get("quick_return"):
        say("Uygun zaman oranı (Q) aranıyor...")
        Q = max_feasible_q(a, psi)
        if Q:
            notes.append(f"Q belirtilmediği için, iletim açısını 45°–135° içinde tutan "
                         f"en büyük değer seçildi: Q = {Q:g}.")
        else:
            Q = 1.0
            notes.append(f"{psi:g}° salınımda hızlı dönüş iletim açısını bozuyor; "
                         f"hızlı dönüşsüz (Q = 1) tasarlandı.")
    else:
        Q = 1.0

    say("Kinematik sentez ve kontroller...")
    mech = kin.synthesize_quick_return(a, psi, Q)
    frames = kin.simulate(mech)
    meas = kin.measure(frames)
    grashof_ok, grashof_kind = kin.grashof(mech["a"], mech["b"], mech["c"], mech["d"])
    mu_ok = kin.MU_MIN_DEG <= meas["mu_min"] and meas["mu_max"] <= kin.MU_MAX_DEG
    swing_ok = abs(meas["swing"] - psi) < 0.5
    q_ok = abs(meas["Q"] - Q) < 0.03
    passed = grashof_ok and mu_ok and swing_ok and q_ok

    checks = [
        {"name": "Grashof", "ok": grashof_ok, "detail": grashof_kind},
        {"name": "Salınım (simülasyon)", "ok": swing_ok,
         "detail": f"{meas['swing']:.2f}° (hedef {psi:g}°)"},
        {"name": "İletim açısı", "ok": mu_ok,
         "detail": f"{meas['mu_min']:.1f}° – {meas['mu_max']:.1f}° "
                   f"(sınır {kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°)"},
    ]
    if Q > 1.0:
        checks.append({"name": "Zaman oranı Q (simülasyon)", "ok": q_ok,
                       "detail": f"{meas['Q']:.2f} (hedef {Q:g})"})
    return _finish(result, spec, notes, say, mech, frames, meas, checks, passed, a, psi, Q, False)


def _finish(result, spec, notes, say, mech, frames, meas, checks, passed, a, psi, Q, rr):
    """Iki mekanizma turu icin ortak: mukavemet, katman cakismasi, Fusion verisi, mesaj."""
    # ---- mukavemet: yuk verildiyse kesit boyutlandirilir, verilmediyse uydurulmaz
    section, strength_ok = strength.DEFAULT_SECTION, True
    F, mat_key = spec.get("max_load_N"), spec.get("material")
    if passed and F:
        if mat_key not in strength.MATERIALS:
            mat_key = "PLA"
            notes.append("Malzeme belirtilmediği için PLA (3B baskı) varsayıldı.")
        mat = strength.MATERIALS[mat_key]
        say("Mukavemet hesabı ve kesit boyutlandırma...")
        forces = strength.analyze_forces(mech, frames, F)
        sec, s_checks = strength.size_links(forces, {k: mech[k] for k in "abcd"}, mat)
        strength_ok = sec is not None
        if sec:
            section = sec
        worst = {}
        for c in s_checks:                      # grup basina en kritik kontrol
            if c["group"] not in worst or c["util"] > worst[c["group"]]["util"]:
                worst[c["group"]] = c
        for g, c in worst.items():
            checks.append({"name": f"{g}: {c['name']}", "ok": c["util"] <= 1.0,
                           "detail": f"{c['detail']} · kullanım %{c['util'] * 100:.0f}"})
        result["strength"] = {
            "material": mat["name"], "safety_factor": mat["n"], "load_N": F,
            "coupler_force_max_N": forces["F_ab_max"],
            "input_torque_max_Nmm": forces["M_crank_max"],
            "checks": s_checks,
        }
    elif passed:
        notes.append("Yük verilmediği için mukavemet hesabı yapılmadı; "
                     "varsayılan kesit (10×4 mm, Ø4 pim) kullanıldı.")
    section = strength.section_with_bosses(section)
    result["section_mm"] = section

    # ---- katman cakismasi: ara katmanlardan gecen mil, hareketli uzuvlara carpmamali
    order, clearances, layers_ok = interference.choose_layer_order(mech, frames, section)
    for p, between, margin in clearances:
        checks.append({"name": f"Katman çakışması ({p} mili)", "ok": margin >= 0,
                       "detail": f"{', '.join(between)} katmanından geçiyor; en az boşluk "
                                 f"{margin + interference.CLEARANCE:.1f} mm (gereken {interference.CLEARANCE:g})"})
    if passed and not layers_ok:
        notes.append("Hiçbir katman sırasında mil, hareketli uzuvlardan kurtulamıyor.")

    fusion = kin.to_fusion_dict(mech, frames, passed and strength_ok and layers_ok)
    fusion["section_mm"] = section
    fusion["layer_order"] = list(order)
    if rr:
        # Fusion cizimi kare len//8'de; giris kolu o konuma gore bu aralikta hareket eder (mafsal siniri)
        k = len(frames) // 8
        fusion["input_range_deg"] = [frames[0]["theta"] - frames[k]["theta"],
                                     frames[-1]["theta"] - frames[k]["theta"]]
        th0 = frames[0]["theta"]
        mu_curve = [(round(f["theta"] - th0, 1), round(f["mu"], 2)) for f in frames]
        mu_axis = [0, round(frames[-1]["theta"] - th0, 1)]
    else:
        mu_curve = sorted((round(f["theta"], 1), round(f["mu"], 2)) for f in frames[::3])
        mu_axis = [0, 360]
    result.update(
        mechanism_type="rocker_rocker" if rr else "crank_rocker",
        lengths={k: mech[k] for k in "abcd"},
        target={"swing": psi, "Q": Q, **({"input_swing": spec["input_swing_deg"]} if rr else {})},
        measured=meas,
        checks=checks,
        notes=notes,
        mu_curve=mu_curve,
        mu_axis=mu_axis,
        fusion=fusion,
    )

    if passed and not strength_ok:
        passed = False
        msg = (f"{F:g} N yük için {strength.MATERIALS[mat_key]['name']} ile uygun kesit bulunamadı "
               f"(genişlik ≤ {strength.MAX_WIDTH} mm). Öneri: daha dayanıklı malzeme ya da daha küçük yük.")
    elif passed and not layers_ok:
        passed = False
        msg = ("Mekanizma kendi kendine çarpıyor: ara katmandan geçen mil, dönen uzuvların yolunda. "
               "Öneri: sabit uzvu iki taraflı (üst ve alt) yapmak ya da ölçüleri değiştirmek.")
    elif passed:
        msg = "Tasarım tüm kontrollerden geçti."
        if "strength" in result:
            parts = ", ".join(f"{n} {s['width']:g}×{s['thickness']:g}"
                              for n, s in section["links"].items())
            msg += (f" Kesitler (mm): {parts}; pim Ø{section['hole']:g} mm, "
                    f"mil Ø{section.get('shaft', section['hole']):g} mm (çelik). "
                    f"En büyük tahrik torku {result['strength']['input_torque_max_Nmm'] / 1000:.2f} N·m.")
    elif rr:
        msg = (f"Giriş {spec['input_swing_deg']:g}° → çıkış {psi:g}° için doğruluk (≤ 1°) ya da iletim açısı "
               f"({kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°) sağlanamadı. Öneri: giriş ve çıkış açılarını "
               f"birbirine yaklaştırın ya da çıkış açısını küçültün.")
    else:
        msg = (f"{psi:g}° salınım" + (f", Q = {Q:g}" if Q > 1 else "") +
               f" ile iletim açısı sınırları ({kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°) sağlanamıyor.")
        if Q > 1:
            q_best = max_feasible_q(a, psi)
            msg += (f" Öneri: Q'yu en fazla {q_best:g} yapın." if q_best
                    else " Öneri: hızlı dönüşten vazgeçin (Q = 1).")
        else:
            best = max_feasible_swing(a, psi)
            if best:
                msg += f" Öneri: salınımı en fazla {best}° yapın."
                result["suggestion_swing"] = best
    result.update(status="ok" if passed else "failed",
                  message=" ".join(notes + [msg]))
    return result
