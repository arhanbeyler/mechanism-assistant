"""
Tek komut: dogal dil -> LLM (JSON) -> kinematik sentez -> kontrol -> mechanism.json

Kullanim:
    python main.py "30 mm krankli, 60 derece salinan bir mekanizma"
Sonra Fusion'da DortCubuk script'ini calistir.
"""

import sys

import kinematics as kin
from llm_extract import extract_spec


def max_feasible_swing(a, psi_start):
    """Istenen salinim saglanamiyorsa, tasarim hedefini (45-135, kompakt) gecen en buyuk salinim."""
    for psi in range(int(psi_start) - 1, 0, -1):
        m = kin.synthesize_crank_rocker(a, psi)
        if m["worst_dev"] <= 45.0:
            return psi
    return None


def main(text):
    print(f"\n[1/3] LLM cumleyi yorumluyor...\n      \"{text}\"")
    spec = extract_spec(text)
    print(f"      -> {spec}")

    if spec["mechanism_type"] != "crank_rocker":
        print("\nSu an sadece krank-kol (dort cubuk) mekanizmasi destekleniyor.")
        return 1

    missing = [name for key, name in (("crank_length_mm", "krank uzunlugu"),
                                      ("output_swing_deg", "salinim acisi"))
               if spec[key] is None]
    if missing:
        print(f"\nTasarim icin eksik bilgi: {', '.join(missing)}. Lutfen cumleye ekleyin.")
        return 1

    a, psi = spec["crank_length_mm"], spec["output_swing_deg"]
    if not 0 < psi < 180:
        print(f"\nSalinim acisi 0-180 derece arasinda olmali (verilen: {psi}).")
        return 1

    print("\n[2/3] Kinematik sentez ve kontroller...")
    mech = kin.synthesize_crank_rocker(a, psi)
    frames = kin.simulate(mech)
    passed = kin.report(mech, frames, psi)

    if not passed:
        best = max_feasible_swing(a, psi)
        print(f"\n✗ {psi:g} derece salinim, iletim acisi sinirlariyla "
              f"({kin.MU_MIN_DEG:g}-{kin.MU_MAX_DEG:g} derece) saglanamiyor.")
        if best:
            print(f"  Oneri: salinimi en fazla {best} derece yapin.")
        print("  Dosya yine de kaydedildi ama bu tasarim onaylanmadi.")

    print("\n[3/3] Fusion icin kayit")
    kin.export_json(mech, frames, passed)
    kin.animate(mech, frames, save_path="output/preview.png")
    print("  Onizleme: output/preview.png")
    print("\nSimdi Fusion'da: Shift+S -> DortCubuk -> Run")
    return 0 if passed else 2


if __name__ == "__main__":
    sentence = " ".join(sys.argv[1:]) or input("Mekanizmayi tarif edin: ")
    sys.exit(main(sentence))
