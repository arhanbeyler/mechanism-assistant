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
        "mechanism_type": {"type": "string", "enum": ["crank_rocker", "unknown"]},
        "crank_length_mm": {"type": ["number", "null"]},
        "output_swing_deg": {"type": ["number", "null"]},
        "quick_return": {"type": "boolean"},
        "time_ratio": {"type": ["number", "null"]},
        "max_load_N": {"type": ["number", "null"]},
        "material": {"type": ["string", "null"], "enum": ["PLA", "PETG", "aluminum", "steel", None]},
    },
    "required": ["mechanism_type", "crank_length_mm", "output_swing_deg",
                 "quick_return", "time_ratio", "max_load_N", "material"],
    "additionalProperties": False,
}

SYSTEM_PROMPT = (
    "You extract mechanism design requirements from the user's text (usually Turkish) into JSON. "
    "Only use values the user explicitly states. Never invent or estimate numbers; "
    "use null when a value is not given. Convert units to mm, degrees and newtons (1 kg = 9.81 N). "
    "For a four-bar mechanism, an angle the user gives without saying what it is "
    "(e.g. '50 derece', '50°') is the output rocker swing (output_swing_deg). "
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
