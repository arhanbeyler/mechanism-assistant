"""Every module survives random slider values: ok / failed / missing, never an exception, always JSON-safe."""

import json
import random

import pytest

import pipeline

KINDS = [k for k in pipeline.MODULES if k != "dwell"]          # dwell synthesizes (seconds): covered in test_dwell


def _sample(mod, rnd):
    kw = {}
    for p in mod["params"]:
        if p.get("options"):
            kw[p["key"]] = rnd.choice(p["options"])[0]
            continue
        lo, hi, st = p["min"], p["max"], p["step"]
        v = lo + st * rnd.randint(0, int(round((hi - lo) / st)))
        if rnd.random() < 0.25:
            v = rnd.choice([lo, hi])
        kw[p["key"]] = None if (p.get("zero_label") and v == 0) else round(v, 6)
    return kw


@pytest.mark.parametrize("kind", KINDS)
def test_random_slider_values(kind):
    rnd = random.Random(sum(map(ord, kind)))                   # stable (str hash changes per run)
    for _ in range(8):
        kw = _sample(pipeline.MODULES[kind], rnd)
        r = pipeline.design_from_params(kind=kind, preview=True, **kw)
        assert r.get("status") in ("ok", "failed", "missing"), (kw, r.get("status"))
        json.dumps(r, allow_nan=False)
