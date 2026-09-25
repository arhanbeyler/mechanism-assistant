"""
Tasarim akisi (ekrana yazdirmadan, sadece veri dondurur):
    cumle -> LLM (JSON) -> kinematik sentez -> simulasyon -> kontroller

Hem terminal (main.py) hem Fusion paneli bunu kullanir.
Sadece standart kutuphane: Fusion'in kendi Python'unda da calisir.
"""

import re

import function_gen
import gears
import interference
import kinematics as kin
import strength
from llm_extract import extract_spec


# C0: moduller kendi kaydiricilarini tanimlar; panel bunlardan otomatik kurar.
# (Dort cubuk panelde ozel kaydiricilarla gelir: renkli salinim bolgesi, giris hareketi vb.)
MODULES = {
    "spur_gear": {
        "title": "Düz dişli",
        "params": [
            {"key": "gear_ratio", "label": "Çevrim oranı", "unit": "", "min": 1, "max": 8, "step": 0.1, "default": 3},
            {"key": "torque_Nm", "label": "Giriş torku", "unit": "N·m", "min": 0.1, "max": 50, "step": 0.1, "default": 2},
            {"key": "speed_rpm", "label": "Giriş devri", "unit": "rpm", "min": 0, "max": 3000, "step": 50, "default": 0,
             "zero_label": "belirtilmedi"},
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


def input_turn_backstop(text, spec):
    """Guvenlik agi: "X° cevirince/dondurunce/itince" kalibi varsa giris kolu sinirli acida doner.
    LLM bunu kacirirsa (ornegin '45°' yaziminda) sayi DOGRUDAN kullanicinin cumlesinden okunur
    (uydurma degil). Dondurur: (spec, not ya da None)."""
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
                       gear_ratio=None, torque_Nm=None, speed_rpm=None):
    """Kaydiricilar icin: LLM'siz, dogrudan sayilardan tasarim.
    kind: "fourbar" ya da "spur_gear" (C0: modul secimi).
    input_swing_deg >= 360: giris tam tur (krank-kol); daha az: sinirli acida giris kolu (B9)."""
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
    if mtype == "spur_gear":
        return _spur_gear(result, spec, say)
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
