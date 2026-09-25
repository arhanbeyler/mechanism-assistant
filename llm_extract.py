"""
Ilk deneme: dogal dil -> yapilandirilmis JSON.

LLM burada hicbir olcu UYDURMAZ. Sadece kullanicinin cumlesinde gecen
bilgileri sabit bir forma (sema) yerlestirir. Cumlede olmayan bilgi null kalir.

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
        "max_load_N": {"type": ["number", "null"]},
        "material": {"type": ["string", "null"], "enum": ["PLA", "PETG", "aluminum", "steel", None]},
        "missing_info": {"type": "array", "items": {"type": "string"}},
    },
    "required": ["mechanism_type", "crank_length_mm", "output_swing_deg",
                 "max_load_N", "material", "missing_info"],
    "additionalProperties": False,
}

SYSTEM_PROMPT = (
    "You extract mechanism design requirements from the user's text into JSON. "
    "Only use values the user explicitly states. Never invent or estimate numbers; "
    "use null when a value is not given. Convert units to mm, degrees and newtons. "
    "List any missing required information in missing_info (in Turkish)."
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
