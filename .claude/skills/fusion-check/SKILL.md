---
name: fusion-check
description: Regression check of Mechanism Assistant before a commit or a demo - unit tests, the LLM evaluation and every Fusion build through the bridge - with a short report. Use when the user asks to check that everything still works.
disable-model-invocation: true
---

# Full check

1. `python -m pytest -q` and report the count (README states it: keep them equal).
2. If Ollama is running: `python tools/llm_eval.py` in the background (about 12 minutes); report failures with
   what the model returned.
3. If Fusion is running with the add-in (the bridge answers on 127.0.0.1:8765): **tell the user Fusion will be
   busy for about 10 minutes**, then `python tools/fusion_smoke.py` and report `N/N built` and any error.
4. Summarise in Turkish for the user: what passed, what failed, what to look at. Do not fix anything unasked.
