# Mechanism Assistant: notes for Claude Code

A sentence (English or Turkish) goes to a local LLM (Ollama, qwen3.5:9b) that only fills a JSON schema;
deterministic Python engines size and verify the design; a Fusion 360 add-in draws it.

## Core rule
- **The LLM never produces a number.** It only extracts values written in the sentence. Engines may use
  defaults, but every default must be reported in the result's `notes` / message.
- Every engine result is re-checked by an independent method (simulation, a textbook example in the tests).

## Conventions
- The repo is entirely in **English** (code, comments, UI, docs, commits). Turkish input sentences must keep working.
- Units: mm, N, N·mm (engines), N·m / rpm / kW at the interface. Match the surrounding code's style and comment density.
- Commit and push after each finished item; update README (counts, tables) and ROADMAP with the findings.

## Adding a module (the pattern every module follows)
1. Engine `<name>.py` + `tests/test_<name>.py` (reproduce a textbook example).
2. `pipeline.py`: `_<name>(result, spec, say)`, a `MODULES` entry (slider params) and its place in
   `MODULE_GROUPS`, a `design_from_params` kwarg + branch, dispatch in `_design_from_spec`, the "unsupported" text.
3. `llm_extract.py`: router enum + text, extractor `(_schema({...}), COMMON + "...")`, `MODULE_EXTRACTORS`,
   `KIND_TO_TYPE`, `KIND_OF_TYPE`, `DESCRIPTION`.
4. `tools/llm_eval.py`: cases (Turkish and English).
5. `fusion_addins/MechanismAssistant/palette.html`: a card (`simpleCard`) and `|| d.<key>` in showResult / showPreview.
6. `fusion_lib/<name>.py` builder + dispatch and `APPEARANCES` in `fusion_lib/fourbar.py`. Something that moves
   gets a revolute input joint (see `INPUT_JOINTS` in the add-in) or frames (`fusion_lib.spring.frame_occurrences`).
7. Verify in Fusion through the bridge, screenshot to `docs/img`, README + ROADMAP, commit, push.

## Commands
- Tests: `python -m pytest -q` (~2 min).
- LLM eval: `python tools/llm_eval.py` (Ollama running; ~12 min).
- Fusion (add-in running): `python tools/fusion_smoke.py [kinds]`, `python tools/make_gifs.py [kinds]`,
  `python tools/print_coupon.py`. The bridge is `POST http://127.0.0.1:8765/exec` with header `X-Token`
  (from `.bridge_token`), 120 s per request. **Fusion freezes while it builds: warn the user before long runs.**
- Add-in code changes need a Fusion restart; `fusion_lib` / engines reload on each build.

## Fusion API pitfalls (learned)
- Create the grounded Ground occurrence first; create occurrences at their final transform (setting it later is unreliable).
- Fusion orders a 3-point arc's ends itself (pick the end near the point); don't average atan2 angles across ±180°.
- Temporary BRep `volume` can raise on an empty result: wrap it. `setSymmetricExtent(d, True)` means d in total.
- A thread's location must be set at creation; occurrence bodies are proxies (use the component's bodies for local geometry).
- A sweep path that jumps at its start fails with "body would intersect itself". Joint limits and motion-link senses:
  check them with a trial move and an overlap test.
