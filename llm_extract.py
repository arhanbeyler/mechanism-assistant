"""
Natural language -> structured JSON (two stages).

The LLM invents NO dimension here. It only puts the facts stated in the user's sentence into a fixed
form (schema). Anything not in the sentence stays null. The code (pipeline.py), not the LLM, decides
what is missing or required. Sentences may be Turkish or English.

Two stages (with more modules, one long prompt made the 9B model miss details):
  1) Router: a short prompt that only picks the kind (four-bar / reducer / gears / spring / cam / shaft).
  2) Module extractor: a short prompt and schema specific to that module.
A new module = a new (prompt, schema) pair; the other prompts are not touched.

OpenAI-compatible API; switching to LM Studio only changes BASE_URL.
"""

import json
import sys
import time
import urllib.request

BASE_URL = "http://localhost:11434/v1"   # LM Studio: http://localhost:1234/v1
MODEL = "qwen3.5:9b"

COMMON = ("You extract design requirements from the user's text (usually Turkish) into JSON. "
          "Only use values the user explicitly states. Never invent or estimate numbers; use null when a "
          "value is not given. Convert units to mm, degrees and newtons (1 kg = 9.81 N). "
          "The degree sign '°' means the same as 'derece'. ")

MATERIAL = {"type": ["string", "null"], "enum": ["PLA", "PETG", "aluminum", "steel", None]}


def _schema(props):
    return {"type": "object", "properties": props, "required": list(props), "additionalProperties": False}


# ---------------------------------------------------------------- 1) router
ROUTER_SCHEMA = _schema({"kind": {"type": "string",
                                   "enum": ["fourbar", "reducer", "spur_gear", "spring", "cam", "shaft", "unknown"]}})
ROUTER_PROMPT = (
    "Classify what the user wants to design. Answer only the kind. "
    "'shaft': sizing a rotating shaft / axle (mil, şaft, aks, 'mil çapı', torque and bending on a shaft, yorulma). "
    "'cam': a cam and follower (kam, eksantrik, itici, 'X mm kaldırsın', yükselme/bekleme/dönüş açıları). "
    "'spring': a spring (yay, basma yayı, konik yay). "
    "'reducer': a complete gearbox with shafts, bearings and housing (redüktör, dişli kutusu, şanzıman, "
    "'motora bağlı', motor power in kW). "
    "'spur_gear': only a gear pair (dişli, dişli çifti, çevrim oranı, 'X kat yavaşlatsın' without a gearbox). "
    "'fourbar': a linkage / mechanism with links, crank, lever or swinging arm (krank, kol, biyel, dört çubuk, "
    "salınım, 'kolu X derece çevirince'). 'unknown': anything else.")

# ---------------------------------------------------------------- 2) module extractors
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

REDUCER = (_schema({
    "gear_ratio": {"type": ["number", "null"]},
    "torque_Nm": {"type": ["number", "null"]},
    "power_kW": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "material": MATERIAL,
    "shaft_material": {"type": ["string", "null"], "enum": ["1020", "1045", None]},
}), COMMON + (
    "The user wants a single-stage gear reducer (gearbox). gear_ratio: speed reduction ('1:4', '4 kat', "
    "'oran 4' -> 4). torque_Nm: input torque in N·m if given. power_kW: motor power in kW ('1.5 kW' -> 1.5; "
    "'750 W' -> 0.75; 1 HP = 0.746 kW). speed_rpm: input (motor) speed ('1450 devir', '1450 d/d', '1450 rpm'); a number followed by rpm, devir or "
    "d/d is ALWAYS speed_rpm, even when the ratio is also described with 'devri X kat düşüren'. "
    "material: gear material only if named. shaft_material: '1020' or '1045' only if named."))

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

SHAFT = (_schema({
    "torque_Nm": {"type": ["number", "null"]},
    "bending_moment_Nm": {"type": ["number", "null"]},
    "radial_force_N": {"type": ["number", "null"]},
    "span_mm": {"type": ["number", "null"]},
    "speed_rpm": {"type": ["number", "null"]},
    "feature": {"type": ["string", "null"],
                "enum": ["keyseat", "sharp_shoulder", "round_shoulder", "ring_groove", "plain", None]},
    "shaft_material": {"type": ["string", "null"], "enum": ["1020", "1045", None]},
    "safety_n": {"type": ["number", "null"]},
}), COMMON + (
    "The user wants to size a rotating shaft. torque_Nm: transmitted torque in N·m. bending_moment_Nm: "
    "bending moment in N·m if given. radial_force_N: a transverse force on the shaft ('2 kN' -> 2000) and "
    "span_mm: bearing distance ('yataklar arası 200 mm') if given. speed_rpm: shaft speed ('1450 devir', "
    "'1450 d/d', '1450 rpm') if given. feature: 'keyseat' (kama, kama yuvası), "
    "'sharp_shoulder' (keskin omuz), 'round_shoulder' (radyuslu/yuvarlatılmış omuz), 'ring_groove' (segman), "
    "'plain' (düz) only if named. shaft_material: '1020' or '1045' only if named. safety_n: required safety "
    "factor if given ('emniyet katsayısı 2.5')."))

MODULE_EXTRACTORS = {"fourbar": FOURBAR, "reducer": REDUCER, "spur_gear": GEAR, "spring": SPRING, "cam": CAM, "shaft": SHAFT}

# All fields (the pipeline expects one dict; fields of other modules are None)
ALL_FIELDS = {"mechanism_type": "unknown", "quick_return": False}
for _schema_, _ in MODULE_EXTRACTORS.values():
    for _k in _schema_["properties"]:
        ALL_FIELDS.setdefault(_k, None)
KIND_TO_TYPE = {"reducer": "reducer", "spur_gear": "spur_gear", "spring": "spring", "cam": "cam", "shaft": "shaft"}   # fourbar: the extractor decides


# ---------------------------------------------------------------- 3) edits of the current design (K1)
KIND_OF_TYPE = {"crank_rocker": "fourbar", "rocker_rocker": "fourbar", "reducer": "reducer", "spur_gear": "spur_gear",
                "spring": "spring", "cam": "cam", "shaft": "shaft"}
DESCRIPTION = {"fourbar": "four-bar linkage", "reducer": "single-stage gear reducer (gearbox)",
               "spur_gear": "spur gear pair", "spring": "compression spring", "cam": "disk cam with roller follower",
               "shaft": "rotating shaft"}
NOT_EDITABLE = {"mechanism_type", "quick_return", "spring_shape"}


def editable_fields(kind):
    """Fields of a module that an edit may change: (numeric fields, {text field: allowed values})."""
    props = MODULE_EXTRACTORS[kind][0]["properties"]
    numeric, choice = [], {}
    for k, v in props.items():
        if k in NOT_EDITABLE:
            continue
        if "enum" in v:
            choice[k] = [e for e in v["enum"] if e is not None]
        else:
            numeric.append(k)
    return numeric, choice


def _edit_schema(fields):
    return _schema({
        "is_edit": {"type": "boolean"},
        "changes": {"type": "array", "items": _schema({
            "field": {"type": "string", "enum": fields},
            "op": {"type": "string", "enum": ["set", "add", "multiply"]},
            "number": {"type": ["number", "null"]},
            "choice": {"type": ["string", "null"]},
        })},
    })


def extract_edit(user_text: str, prev_spec: dict) -> dict:
    """Is the message an edit of the current design, and which changes? Returns {"is_edit", "changes"}.
    The LLM only reports what the text says; pipeline.apply_edit checks the numbers and applies them."""
    kind = KIND_OF_TYPE.get(prev_spec.get("mechanism_type"))
    if kind is None:
        return {"is_edit": False, "changes": []}
    numeric, choice = editable_fields(kind)
    current = {k: prev_spec.get(k) for k in numeric + list(choice)}
    prompt = COMMON + (
        f"The current design is a {DESCRIPTION[kind]} with these values: {json.dumps(current)}. "
        "Decide whether the user's new message asks to CHANGE this design ('make the crank 40 mm', 'use aluminium', "
        "'yükü 10 kg yap', 'krankı 5 mm uzat', 'devri iki katına çıkar', 'alüminyum olsun', 'salınımı 70 derece yap') "
        "-> is_edit = true, or describes a NEW / different request (a new mechanism, a gear, a spring... with its own "
        "values) -> is_edit = false and changes = []. "
        "For each change: field from the list; op 'set' for a new absolute value, 'add' for 'X more / less' "
        "(negative number for less / shorter), 'multiply' for 'twice' (2), 'half' (0.5), 'three times' (3). "
        "number: ONLY an amount written in the message, converted to the field's unit (_mm: mm, _deg: degrees, "
        "_N: newtons with 1 kg = 9.81 N, _Nm: N·m, _rpm: rpm, _kW: kW); if the message gives no amount "
        "('biraz daha uzun', 'a bit longer') number = null. "
        f"choice: for these text fields only, one of the allowed values: {json.dumps(choice)}; otherwise null.")
    return _chat(prompt, user_text, _edit_schema(numeric + list(choice)), "edit")


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
    """Two-stage extraction. Returns a dict with all fields (None where the module has no such field)."""
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
    print("Input :", text)
    t0 = time.time()
    spec = extract_spec(text)
    print(f"Time  : {time.time() - t0:.1f} s")
    print("Output:")
    print(json.dumps(spec, indent=2, ensure_ascii=False))
