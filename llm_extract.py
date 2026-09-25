"""
Dogal dil -> yapilandirilmis JSON (iki asamali).

LLM burada hicbir olcu UYDURMAZ. Sadece kullanicinin cumlesinde gecen
bilgileri sabit bir forma (sema) yerlestirir. Cumlede olmayan bilgi null kalir.
Hangi bilginin eksik/zorunlu oldugunu LLM degil, kod (pipeline.py) belirler.

Iki asama (moduller cogaldikca tek uzun istem 9B modelde ayrintilari kaciriyordu):
  1) Yonlendirici: kisa istemle sadece tur (dort cubuk / disli / yay / bilinmiyor).
  2) Modul cikarici: o module ozel kisa istem ve sema.
Yeni modul = yeni bir (istem, sema) ciftidir; digerlerinin istemlerine dokunulmaz.

OpenAI uyumlu API kullaniyoruz; LM Studio'ya gecmek icin sadece BASE_URL degisir.
"""

import json
import sys
import time
import urllib.request

BASE_URL = "http://localhost:11434/v1"   # LM Studio icin: http://localhost:1234/v1
MODEL = "qwen3.5:9b"

COMMON = ("You extract design requirements from the user's text (usually Turkish) into JSON. "
          "Only use values the user explicitly states. Never invent or estimate numbers; use null when a "
          "value is not given. Convert units to mm, degrees and newtons (1 kg = 9.81 N). "
          "The degree sign '°' means the same as 'derece'. ")

MATERIAL = {"type": ["string", "null"], "enum": ["PLA", "PETG", "aluminum", "steel", None]}


def _schema(props):
    return {"type": "object", "properties": props, "required": list(props), "additionalProperties": False}


# ---------------------------------------------------------------- 1) yonlendirici
ROUTER_SCHEMA = _schema({"kind": {"type": "string", "enum": ["fourbar", "spur_gear", "spring", "cam", "unknown"]}})
ROUTER_PROMPT = (
    "Classify what the user wants to design. Answer only the kind. "
    "'cam': a cam and follower (kam, eksantrik, itici, 'X mm kaldırsın', yükselme/bekleme/dönüş açıları). "
    "'spring': a spring (yay, basma yayı, konik yay). "
    "'spur_gear': gears or a speed reducer (dişli, dişli çifti, redüktör, çevrim oranı, 'X kat yavaşlatsın'). "
    "'fourbar': a linkage / mechanism with links, crank, lever or swinging arm (krank, kol, biyel, dört çubuk, "
    "salınım, 'kolu X derece çevirince'). 'unknown': anything else.")

# ---------------------------------------------------------------- 2) modul cikaricilar
FOURBAR = (_schema({
    "mechanism_type": {"type": "string", "enum": ["crank_rocker", "rocker_rocker"]},
    "crank_length_mm": {"type": ["number", "null"]},
    "input_swing_deg": {"type": ["number", "null"]},
    "output_swing_deg": {"type": ["number", "null"]},
    "quick_return": {"type": "boolean"},
    "time_ratio": {"type": ["number", "null"]},
    "max_load_N": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a four-bar linkage. "
    "mechanism_type: 'rocker_rocker' when the INPUT link only turns through a limited angle "
    "(two angles: first how far the input is turned — çevirince/döndürünce/dönünce/itince — then the output; "
    "e.g. '4 cm kolu 45° çevirince diğer kol 120° dönsün' -> rocker_rocker, input 45, output 120; "
    "'kapı gibi açılıp kapansın'); otherwise 'crank_rocker' (input crank turns fully; a single "
    "salınım/swing angle, e.g. '30 mm krank, 60° salınım' -> crank_rocker, output 60, input null). "
    "crank_length_mm: length of the INPUT link (krank, giriş kolu, or the 'kol' that is turned): "
    "'30 mm krank' -> 30, '40 mm giriş kolu' -> 40, '4 cm kolu' -> 40. "
    "output_swing_deg: output angle; 'salınım'/'sallansın' is always the output; a lone angle is the output. "
    "input_swing_deg: input rotation, only for rocker_rocker and only if given. "
    "max_load_N: load at the output tip ('uçta 5 kg' -> 49.05). "
    "quick_return: true only if asked ('hızlı dönüşlü', 'zaman oranı'). "
    "time_ratio: Q only if a number is given ('zaman oranı 1.2', 'Q=1.3')."))

GEAR = (_schema({
    "gear_ratio": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a spur gear pair. gear_ratio: speed reduction ratio ('1:3', '3 kat yavaş', 'oran 3', "
    "'devri 3 kat düşüren' -> 3). torque_Nm: input torque in N·m ('5 Nm' -> 5; 1 kg·cm = 0.0981 N·m; "
    "N·mm / 1000). speed_rpm: input speed if given."))

SPRING = (_schema({
    "spring_force_N": {"type": ["number", "null"]},
    "spring_deflection_mm": {"type": ["number", "null"]},
    "spring_shape": {"type": "string", "enum": ["helical", "conical"]},
    "spring_taper": {"type": ["number", "null"]},
    "outer_diameter_max_mm": {"type": ["number", "null"]},
    "material": MATERIAL,
}), COMMON + (
    "The user wants a compression spring. spring_force_N: max load ('5 kg' -> 49.05, '100 N' -> 100). "
    "spring_deflection_mm: how much it compresses under that load ('20 mm sıkışan', '15 mm çöken'). "
    "spring_shape: 'conical' if konik/conical, else 'helical'. spring_taper: small/large diameter ratio only "
    "if given. outer_diameter_max_mm: max outer diameter if given ('dış çapı en fazla 12 mm', 'Ø20 deliğe girsin')."))

CAM = (_schema({
    "cam_rise_mm": {"type": ["number", "null"]},
    "rise_deg": {"type": ["number", "null"]},
    "dwell_top_deg": {"type": ["number", "null"]},
    "return_deg": {"type": ["number", "null"]},
    "motion_law": {"type": ["string", "null"], "enum": ["cycloidal", "harmonic", "poly345", None]},
    "roller_radius_mm": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants a disk cam with a translating roller follower. cam_rise_mm: follower lift/stroke "
    "('20 mm kaldırsın', 'strok 20'). rise_deg: cam angle for the rise ('120 derecede yükselsin'). "
    "dwell_top_deg: dwell at the top ('60 derece beklesin', 'üstte 60° bekleme'). return_deg: cam angle for the "
    "return/fall ('120 derecede insin'). motion_law: 'cycloidal' (sikloidal), 'harmonic' (basit harmonik), "
    "'poly345' (3-4-5 polinom) only if named. roller_radius_mm: roller radius if given."))

MODULE_EXTRACTORS = {"fourbar": FOURBAR, "spur_gear": GEAR, "spring": SPRING, "cam": CAM}

# Tum alanlar (pipeline tek bir sozluk bekler; modulde olmayanlar None)
ALL_FIELDS = {"mechanism_type": "unknown", "quick_return": False}
for _schema_, _ in MODULE_EXTRACTORS.values():
    for _k in _schema_["properties"]:
        ALL_FIELDS.setdefault(_k, None)
KIND_TO_TYPE = {"spur_gear": "spur_gear", "spring": "spring", "cam": "cam"}   # fourbar: cikarici tur secer


def _chat(system, user_text, schema, name):
    body = {
        "model": MODEL,
        "messages": [{"role": "system", "content": system}, {"role": "user", "content": user_text}],
        "temperature": 0,
        "reasoning_effort": "none",
        "response_format": {"type": "json_schema", "json_schema": {"name": name, "schema": schema, "strict": True}},
    }
    req = urllib.request.Request(BASE_URL + "/chat/completions", data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as resp:
        reply = json.load(resp)
    return json.loads(reply["choices"][0]["message"]["content"])


def route(user_text: str) -> str:
    return _chat(ROUTER_PROMPT, user_text, ROUTER_SCHEMA, "route")["kind"]


def extract_spec(user_text: str) -> dict:
    """Iki asamali cikarim. Dondurur: tum alanlari iceren sozluk (modulde olmayanlar None)."""
    kind = route(user_text)
    spec = dict(ALL_FIELDS)
    if kind not in MODULE_EXTRACTORS:
        return spec
    schema, prompt = MODULE_EXTRACTORS[kind]
    spec.update(_chat(prompt, user_text, schema, kind))
    if kind in KIND_TO_TYPE:
        spec["mechanism_type"] = KIND_TO_TYPE[kind]
    return spec


if __name__ == "__main__":
    text = " ".join(sys.argv[1:]) or (
        "10 cm'lik bir krankla çıkış kolu 60 derece salınım yapsın, "
        "PETG'den basacağım, uçta yaklaşık 5 kg yük olacak."
    )
    print("Girdi :", text)
    t0 = time.time()
    spec = extract_spec(text)
    print(f"Sure  : {time.time() - t0:.1f} s")
    print("Cikti :")
    print(json.dumps(spec, indent=2, ensure_ascii=False))
