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


def test_zero_or_negative_values_are_asked_for_not_used():
    r = pipeline.design_from_spec({"mechanism_type": "spur_gear", "gear_ratio": 0, "torque_Nm": -5, "material": "PLA"})
    assert r["status"] == "missing" and "Ignored" in r["message"]


def test_absurd_values_fail_with_a_reason():
    r = pipeline.design_from_spec({"mechanism_type": "press", "force_N": 400, "stroke_mm": 1e6, "working_stroke_mm": 1,
                                   "speed_rpm": 50})
    assert r["status"] == "failed" and "outside what" in r["message"]


def test_every_module_is_in_one_group():
    import pipeline
    keys = [k for _, ks in pipeline.MODULE_GROUPS for k in ks]
    assert len(keys) == len(set(keys)) == len(pipeline.MODULES)
    assert all(m["group"] for m in pipeline.MODULES.values())
