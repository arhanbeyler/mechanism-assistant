"""
Dogal dil -> yapilandirilmis JSON.

LLM burada hicbir olcu UYDURMAZ. Sadece kullanicinin cumlesinde gecen
bilgileri sabit bir forma (sema) yerlestirir. Cumlede olmayan bilgi null kalir.
Hangi bilginin eksik/zorunlu oldugunu LLM degil, kod (pipeline.py) belirler.

OpenAI uyumlu API kullaniyoruz; LM Studio'ya gecmek icin sadece BASE_URL degisir.
"""

import json
import sys
import time
import urllib.request

BASE_URL = "http://localhost:11434/v1"   # LM Studio icin: http://localhost:1234/v1
MODEL = "qwen3.5:9b"

# LLM'in doldurmak zorunda oldugu form
SPEC_SCHEMA = {
    "type": "object",
    "properties": {
        "mechanism_type": {"type": "string", "enum": ["crank_rocker", "rocker_rocker", "spur_gear", "unknown"]},
        "gear_ratio": {"type": ["number", "null"]},
        "torque_Nm": {"type": ["number", "null"]},
        "speed_rpm": {"type": ["number", "null"]},
        "crank_length_mm": {"type": ["number", "null"]},
        "input_swing_deg": {"type": ["number", "null"]},
        "output_swing_deg": {"type": ["number", "null"]},
        "quick_return": {"type": "boolean"},
        "time_ratio": {"type": ["number", "null"]},
        "max_load_N": {"type": ["number", "null"]},
        "material": {"type": ["string", "null"], "enum": ["PLA", "PETG", "aluminum", "steel", None]},
    },
    "required": ["mechanism_type", "gear_ratio", "torque_Nm", "speed_rpm", "crank_length_mm",
                 "input_swing_deg", "output_swing_deg", "quick_return", "time_ratio", "max_load_N", "material"],
    "additionalProperties": False,
}

SYSTEM_PROMPT = (
    "You extract mechanism design requirements from the user's text (usually Turkish) into JSON. "
    "Only use values the user explicitly states. Never invent or estimate numbers; "
    "use null when a value is not given. Convert units to mm, degrees and newtons (1 kg = 9.81 N). "
    "mechanism_type: 'spur_gear' for a gear pair / reducer (dişli, dişli çifti, redüktör, çevrim oranı, "
    "'3 kat yavaşlatsın'); then gear_ratio = the speed reduction ratio (e.g. '1:3', '3 kat yavaş', 'oran 3' -> 3), "
    "torque_Nm = input torque in N·m (convert N·mm or kg·cm: 1 kg·cm = 0.0981 N·m), speed_rpm = input speed if given. "
    "Otherwise: 'crank_rocker' when the input (crank) rotates continuously / full turns "
    "(default when the input motion is not described); 'rocker_rocker' when the INPUT link only turns "
    "through a limited angle (e.g. 'giriş kolu 60 derece dönünce çıkış 90 derece dönsün', "
    "'kolu 45 derece çevirince', '4 cm kolu 45° çevirince diğer kol 120° dönsün' -> rocker_rocker, "
    "input_swing_deg 45, output_swing_deg 120; 'kapı gibi açılıp kapansın'). "
    "Two angles where the first is how far the input is turned (çevirince/döndürünce/dönünce/itince) "
    "always mean rocker_rocker. The degree sign '°' means the same as 'derece'. "
    "But a crank with a single swing/salınım angle is crank_rocker: '30 mm krank, 60° salınım' -> "
    "crank_rocker, output_swing_deg 60, input_swing_deg null. 'salınım'/'sallansın' is always the OUTPUT. "
    "input_swing_deg: the input link's rotation angle, only for rocker_rocker and only if given as a number. "
    "crank_length_mm: the length of the INPUT link — the link that is driven/turned by the user or motor; "
    "it may be called 'krank', 'giriş kolu', or simply the 'kol' that is 'çevrilen'/'döndürülen' "
    "(e.g. '4 cm kolu 45 derece çevirince' -> 40). "
    "For a four-bar mechanism, a single angle the user gives without saying what it is "
    "(e.g. '50 derece', '50°') is the output swing (output_swing_deg). "
    "quick_return: true only if the user asks for a quick-return motion "
    "(e.g. 'hızlı dönüşlü', 'geri dönüşü daha hızlı', 'zaman oranı'), otherwise false. "
    "time_ratio: the time ratio Q only if the user gives a number for it "
    "(e.g. 'zaman oranı 1.2', 'Q=1.3', 'dönüş 1.25 kat hızlı'), otherwise null."
)


def extract_spec(user_text: str) -> dict:
    body = {
        "model": MODEL,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_text},
        ],
        "temperature": 0,
        "reasoning_effort": "none",
        "response_format": {
            "type": "json_schema",
            "json_schema": {"name": "mechanism_spec", "schema": SPEC_SCHEMA, "strict": True},
        },
    }
    req = urllib.request.Request(
        BASE_URL + "/chat/completions",
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=300) as resp:
        reply = json.load(resp)
    return json.loads(reply["choices"][0]["message"]["content"])


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
