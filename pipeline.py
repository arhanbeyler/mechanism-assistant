"""
Tasarim akisi (ekrana yazdirmadan, sadece veri dondurur):
    cumle -> LLM (JSON) -> kinematik sentez -> simulasyon -> kontroller

Hem terminal (main.py) hem Fusion paneli bunu kullanir.
Sadece standart kutuphane: Fusion'in kendi Python'unda da calisir.
"""

import kinematics as kin
from llm_extract import extract_spec


def max_feasible_swing(a, psi_start):
    """Istenen salinim saglanamiyorsa, tasarim hedefini (45-135, kompakt) gecen en buyuk salinim."""
    for psi in range(int(psi_start) - 1, 0, -1):
        m = kin.synthesize_crank_rocker(a, psi)
        if m["worst_dev"] <= 45.0:
            return psi
    return None


def design_from_text(text, on_status=None):
    """
    Dondurur: dict(status, message, spec, ...)
      status: "ok" | "failed" (kontrollerden gecmedi) | "missing" | "unsupported"
    on_status(str): ilerleme bildirimi icin istege bagli geri cagirma.
    """
    say = on_status or (lambda s: None)

    say("LLM cümleyi yorumluyor...")
    spec = extract_spec(text)
    result = {"text": text, "spec": spec}

    if spec["mechanism_type"] != "crank_rocker":
        return dict(result, status="unsupported",
                    message="Şu an sadece krank-kol (dört çubuk) mekanizması destekleniyor.")

    missing = [name for key, name in (("crank_length_mm", "krank uzunluğu"),
                                      ("output_swing_deg", "salınım açısı"))
               if spec[key] is None]
    if missing:
        return dict(result, status="missing",
                    message=f"Tasarım için eksik bilgi: {', '.join(missing)}.")

    a, psi = spec["crank_length_mm"], spec["output_swing_deg"]
    if not 0 < psi < 180:
        return dict(result, status="missing",
                    message=f"Salınım açısı 0-180° arasında olmalı (verilen: {psi:g}°).")

    say("Kinematik sentez ve kontroller...")
    mech = kin.synthesize_crank_rocker(a, psi)
    frames = kin.simulate(mech)
    meas = kin.measure(frames)
    grashof_ok, grashof_kind = kin.grashof(mech["a"], mech["b"], mech["c"], mech["d"])
    mu_ok = kin.MU_MIN_DEG <= meas["mu_min"] and meas["mu_max"] <= kin.MU_MAX_DEG
    passed = grashof_ok and mu_ok

    result.update(
        lengths={k: mech[k] for k in "abcd"},
        target={"swing": psi, "Q": mech.get("Q", 1.0)},
        measured=meas,
        checks=[
            {"name": "Grashof", "ok": grashof_ok, "detail": grashof_kind},
            {"name": "Salınım (simülasyon)", "ok": abs(meas["swing"] - psi) < 0.5,
             "detail": f"{meas['swing']:.2f}° (hedef {psi:g}°)"},
            {"name": "İletim açısı", "ok": mu_ok,
             "detail": f"{meas['mu_min']:.1f}° – {meas['mu_max']:.1f}° "
                       f"(sınır {kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°)"},
        ],
        mu_curve=sorted((round(f["theta"], 1), round(f["mu"], 2)) for f in frames[::3]),
        fusion=kin.to_fusion_dict(mech, frames, passed),
    )

    if passed:
        result.update(status="ok", message="Tasarım tüm kontrollerden geçti.")
    else:
        best = max_feasible_swing(a, psi)
        msg = (f"{psi:g}° salınım, iletim açısı sınırlarıyla "
               f"({kin.MU_MIN_DEG:g}–{kin.MU_MAX_DEG:g}°) sağlanamıyor.")
        if best:
            msg += f" Öneri: salınımı en fazla {best}° yapın."
        result.update(status="failed", message=msg, suggestion_swing=best)
    return result
