---
name: new-module
description: Add a new machine-element / mechanism module to Mechanism Assistant end to end (engine, tests, pipeline, LLM extractor, eval, panel card, Fusion builder, verification, docs). Use when the user asks for a new module or a roadmap item that is a new design type.
disable-model-invocation: true
---

# New module: $ARGUMENTS

Follow the module pattern in CLAUDE.md, in order. Do not skip a step; tick each one off.

1. **Engine** `<name>.py`: a textbook method (Shigley / Norton / the standard), docstring with the formulas and units.
   Defaults allowed only if reported. `design(...)` returns `(result dict, checks)`, checks as
   `{"group", "name", "util", "detail"}`; `None` + a failing check when nothing fits.
2. **Tests** `tests/test_<name>.py`: reproduce a published example (cite it), the "smallest that works" choice,
   one failing case. `python -m pytest -q tests/test_<name>.py`.
3. **pipeline.py**: `_<name>(result, spec, say)` (missing values -> status "missing", notes for defaults),
   `MODULES` entry + `MODULE_GROUPS`, `design_from_params` kwarg, dispatch, the "unsupported" text.
   Zero / negative inputs: `_ZERO_OK` / `_NEGATIVE_OK` only if meaningful.
4. **llm_extract.py**: router enum + one-line description with Turkish words, extractor schema + prompt with
   Turkish examples, `MODULE_EXTRACTORS`, `KIND_TO_TYPE`, `KIND_OF_TYPE`, `DESCRIPTION`.
5. **tools/llm_eval.py**: at least one Turkish and one English case; run the subset, then the full eval.
6. **palette.html**: a card via `simpleCard(...)`, add `|| d.<key>` to both showResult and showPreview conditions.
7. **fusion_lib/<name>.py** builder + dispatch and `APPEARANCES` in `fusion_lib/fourbar.py`. If it moves: an input
   joint named like `Ground-Driver` or frames via `fusion_lib.spring.frame_occurrences`.
8. **Verify in Fusion** (warn the user first: Fusion freezes while building): `python tools/fusion_smoke.py <kind>`,
   an overlap check over the motion, a screenshot to `docs/img/<name>.png`.
9. **Docs**: README (gallery row in the right group, module table row, test / eval counts, engine list) and
   ROADMAP (tick the item, write the findings). Then commit and push.
