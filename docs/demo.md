# Demo video: shot list (~2 minutes)

**Recording:** on Windows `Win + Alt + R` (Xbox Game Bar) records the Fusion window; press it again to stop.
Before recording, start Ollama and send one sentence to warm the model up (the first request is slow).
The panel is a floating window; place it below Fusion so that the model view stays on top.

| # | Time | What to do | On screen / caption |
|---|---|---|---|
| 1 | 0:00–0:10 | Empty Fusion + panel | "Describe a mechanism in plain language; a local AI reads it, Python engines size and check it, Fusion draws it." |
| 2 | 0:10–0:35 | Type `30 mm crank, 60° swing, 5 kg at the tip, PETG` → Send | Result card: link lengths, transmission-angle chart, ✓ checks. The coloured four-bar in Fusion. |
| 3 | 0:35–0:45 | **▶ Loop** | The mechanism turns. "The swing was also confirmed by Fusion's own solver: 55/55°." |
| 4 | 0:45–1:00 | Drag the swing slider from 60 to 120 | It enters the red band: ✗ and the reason (transmission angle). "It does not draw an infeasible design; it says why." |
| 5 | 1:00–1:15 | Type `turn a 4 cm arm 45° and the other arm turns 120°, PETG, 2 kg` → draw → **⟳ 1 turn** | A limited-angle four-bar swinging back and forth. |
| 6 | 1:15–1:30 | Type `1.5 kW 1450 rpm motor, 1:4 gearbox` → **▶ Loop** | Housing, bearings, keyed shafts and gears; the output turns 4× slower. "Gear forces become shaft loads automatically." |
| 7 | 1:30–1:40 | Type: **Compression spring**, taper 0.5, view "With plates" → draw → 1 turn | The conical spring compresses and releases. |
| 8 | 1:40–1:50 | Type: **Cam** → draw → Loop | The follower tracks the cam profile. |
| 9 | 1:50–2:00 | Type: **Shaft**, stress raiser: keyseat → retaining-ring groove | The diameter grows (K_t 2.14 → 5.0). Draw: the stepped, keyed shaft on its bearings. |
| 10 | 2:00–2:10 | `python -m pytest` in a terminal (optional) | "143 tests passed." Last frame: the image table in the README. |

**Tips**
- Press "Home" (the house icon) before each scene so the camera stays the same.
- Cut long waits in editing (the LLM takes ~5 s).
- Captions are easier than a voice-over: open the recording in Clipchamp (built into Windows) and add the captions above.
