# Roadmap

**Project:** AI-Driven Autonomous Kinematic Synthesis and Parametric CAD Generation System

**Core rule:** the LLM only understands the request and turns it into JSON; it produces no dimension.
Deterministic engines compute the numbers, an independent simulation verifies them, Fusion draws.

```
Sentence → LLM (Ollama, local) → engines + checks → Fusion 360
```

## A. Infrastructure
- [x] **A1. Git.** Version control, every step committed
- [x] **A2. Tests.** `python -m pytest`: formulas and simulations are checked automatically
- [x] **A3. Engines without NumPy.** Fusion's own Python has no NumPy
- [x] **A4. Fusion panel and bridge.** Chat + result card in a panel; through the bridge Claude can run code in Fusion and take screenshots

## B. Finishing the four-bar
- [x] **B5. Q from the sentence.** Understands "make it quick-return"
- [x] **B6. Tighter LLM schema.** No needless "missing information" warnings
- [x] **B7. Strength.** Sections from load and material (buckling, net section, bending, pin shear / bearing), safety factor, drive torque
- [x] **B7b. A section per link.** Common pin diameter, width / thickness per link; 32 % less material than one shared section
- [x] **B7c. Parameter sliders.** Sliders computed live in the panel, feasibility colours on the swing slider; "1 turn" and continuous turning in Fusion
- [x] **B7d. Stress concentration (Kt) and deflection.** Roark Kt (brittle materials, axial stress); L/200 deflection limit (cantilever bending / axial stretch)
- [x] **B7e. Verification with Fusion FEA (qualitative).** Al 6061 rocker as a cantilever, 49 N: FEA n = 2–4 at the root, hand calculation n ≈ 3.6 ✓. The Simulation API does not work in preview, the solve went to the cloud, results could not be read as numbers
- [x] **B7f. Local, automatic FEA.** gmsh + scikit-fem (fully local); FEA vs hand calculation for the rocker and the crank; an "FEA" button in the panel
- [x] **B7g. Hub compliance.** Model δ_hub = P·L²/(C·E·t·r²), C calibrated with FEA (21 geometries, C = 6–9, independent of L); C = 6 on the safe side
- [x] **B7h. Separate shaft and pin diameters.** O2/O4 steel shafts (torsion + shear), A/B pins; Fusion draws the right hole at each end. PETG 5 kg: coupler 25 → 13 mm, volume −12 %
- [x] **B8a. Local bosses.** Every hole in its own boss (D = 2.5·d), the web only as thick as the checks need; boss and web sections checked separately; solid extrude + hole cut in Fusion. PETG volume −29 %, Al −59 %
- [x] **B8b. Layer interference.** A shaft through a middle layer must stay at least 1 mm from the moving links (web + boss) over the cycle; the layer order changes if needed, otherwise the design is rejected
- [ ] **B8c. Pins and clearances (print-in-place).** Pin / head geometry, radial and axial clearance (at print time, with D12)
- [x] **B9. Four-bars without a full-turn input.** Freudenstein function generator (3 Chebyshev precision points, θ₀/φ₀ sweep, structural error ≤ 1°); first candidate passing strength + interference; LLM "rocker_rocker"; "Input motion" in the panel; joint limits and back-and-forth motion in Fusion

## C. New modules
- [x] **C0. Module-specific sliders.** pipeline.MODULES (name, unit, range); the panel builds its "Type" selector and sliders from it; a design from the chat switches to the right module
- [x] **C9. Spur gears.** Lewis bending, contact ratio, undercut, Hertz (info), steel shaft and hub; involute teeth, circular pattern, motion link in Fusion (tooth overlap 0, ratio verified)
- [x] **C10. Helical and conical springs.** Shigley: Bergsträsser, S_ut = A/dᵐ, no yield when solid, buckling; conical k = G·d⁴/(16·Nₐ(R₁+R₂)(R₁²+R₂²)); helix sweep + ground ends in Fusion
- [x] **L1. Two-stage LLM.** Router (kind) + short module-specific extractor; one long prompt made the 9B model miss details; llm_eval 16/16
- [x] **C11a. Cam.** Cycloidal / harmonic / 3-4-5; pressure angle ≤ 30°, no undercut, ρ ≥ 2·R_roller; smallest base circle; cam + roller follower + s(θ) animation in Fusion (contact gap 0 and overlap 0 at 8 angles)
- [x] **C11b. Shaft.** Shigley ch. 7, DE-Goodman fatigue (Marin factors, k_b iteration, Table 7-1 K_t), von Mises yield; rounded up to a standard diameter. Two-bearing shaft in Fusion: only the analysed stress raiser is drawn (DIN 6885 key + end-milled keyseat / retaining-ring groove / D/d = 1.5 shoulder, r/d 0.02 or 0.1). Length, diameter and volume match the calculation for all 5 kinds; turns on its joint
- [x] **C11c. Shaft completed.** Catalogue bearings (62/63 series, shoulder rule, L10 ≥ 20 000 h), torque-carrying bearing seat (sharp shoulder, DE-Goodman), DIN 6885 key length (shear + crushing, ≤ 1.5·d), numerical deflection / slope of the stepped shaft (Table 7-2), Rayleigh critical speed (≥ 2× running speed); a failing check moves to the next standard diameter. Bearing seats + catalogue-size bearings in Fusion

## E. Systems (connecting modules)
In a system one module's output is another's input (forces, diameters and masses are never typed in twice).
- [x] **E1. Single-stage reducer.** Motor (power, speed) → gear pair → two shafts (gear forces → M, T; gear mass → critical speed) → bearings → keys → housing. The constraints (gear bore = shaft diameter, key ≤ face width, bearings side by side in one wall, common span) are iterated until nothing changes. Housing + bearings + keyed shafts + gears with keyways in Fusion; tooth overlap 0 at 6 angles, ratio 1/3 verified
- [x] **E2. Two-stage reducer.** Ratios above 6 are split over two stages (lightest gear volume around √i); a multi-load shaft module (loads in two planes, reactions, numerical M/EI deflection, Rayleigh speed with several masses, a key per gear seat) sizes all three shafts; gear bores, keys, bearing seats, housing web and the common span are iterated together, plus the clearances only a combination can break (gear 2 vs output shaft, gear 4 vs input shaft). Fusion: housing, three shafts with keys, four keyed gears, two motion links; tooth overlap 0 in both stages at 5 angles, intermediate / output angles match the ratios. Example: 10 N·m, 1450 rpm, i = 12 → 3.83 × 3.14, shafts Ø17 / 25 / 32, bearings 6201 / 6303 / 6205
- [x] **E3. Cam + spring assembly.** With cam speed and follower mass: required spring force = max(F_min, 1.3·(−m·ẍ) − F_ext) over the turn; preload swept, the spring module designs each candidate, the lightest that still passes with its real (rounded) rate wins; contact force stays > 0 (recomputed in the tests); impossible speeds are rejected with a reason
- [ ] **E4. Press.** Slider-crank + flywheel (energy fluctuation from the torque curve) + motor
- [ ] **E5. Folding platform** (senior-project candidate). Four-bar + counterbalance spring + beam / plate + pins

## F. New mechanisms (motion)
- [x] **F1. Slider-crank (piston).** Stroke → crank (exact formula, offset by bisection), rod = shortest whole mm with b/a ≥ 3 and rod angle ≤ 20°; x, v, a over the cycle; crank torque by virtual work (checked against the moment of the rod force); crank and rod sized with the four-bar strength checks; Fusion: base + rails + crank + rod + piston with a slider joint, the solver's piston position equals x(θ) at 36 angles (0.000 mm)
- [x] **F2. Coupler curves and a straight-line linkage.** Coupler-point curve of any four-bar (point at distance p, angle γ on the coupler); straight part = longest run of the curve within 0.2 % of its length from its best-fit line (absolute limit iterated to a fixed point: the relative limit is not monotone). Hoeckens linkage scaled to the asked straight path (100 mm → crank 22.5 mm, 100.4 mm straight within 0.19 mm over 212° of the crank turn, speed ±28 %, lift 20 mm on return). Load at the point: coupler statics (rocker = two-force member) give the crank torque, equal to virtual work; the coupler is checked as an overhanging beam. Fusion: the coupler runs A → P with a hole at B, the path is drawn as a spline with its straight part; the drawn point stays within 0.0023 mm of the engine curve over a full turn
- [ ] **F2b. Path synthesis.** "This point should pass through these points": precision-point / optimisation synthesis of a four-bar coupler curve; Chebyshev lambda and Watt linkages as alternatives
- [x] **F3. Geneva mechanism.** External Geneva, n = 3..12 slots from the slot count or the step angle; tangential entry (a = C·sin(π/n)); wheel angle φ(θ), closed-form speed and acceleration checked against numerical derivatives; load inertia (a disk of given mass and diameter + the wheel) gives the pin force, driver torque and shaft sizes; pin ≥ a/8 and wheel thickness ≥ pin (proportions, not only strength). Locking disk relief sized by sweeping the wheel's outline over the index (a plain rim-radius circle left the rim 0.1–0.5 mm proud a few degrees off mid-index; found in Fusion, now a test). Fusion: base, driver (locking disk + crank plate + pin), slotted wheel; the add-in turns the wheel from the φ(θ) table. Finding: Fusion rounds joint angles to 0.1°, so incremental updates drifted 0.4° per turn and collided; the add-in now keeps the exact angle and sets it absolutely (zero overlap over two turns)
- [x] **F4. Scotch yoke and crank + slotted lever quick return.** Yoke: harmonic motion, slotted-bar sizing. Quick return: geometry from stroke and Q, statics = virtual work, cutting / return speeds. Fusion: slotted blocks on rails, pin-slot joints whose slide axes are found by a trial turn. Findings: joint axes follow the picked edge's frame, so the crank is drawn square to the slot; a locked loop does not turn at all, so the trial must also check that the crank really moved (a first version passed a locked joint); the rail post must stay outside the crank's swept circle
- [ ] **F4b. Six-bar (Watt / Stephenson)**, Whitworth (crank longer than the pivot distance)
- [x] **F5. Toggle clamp.** Dead-centre synthesis (O2, A, B in line at the clamped pose), 3° over centre onto a stop; transmission angle strict only where the load builds up (last 20° of travel), ≥ 20° elsewhere; handle force curve (statics = virtual work, 0 at the dead centre); arm back-off past the centre reported (the squeeze must exceed it). Finding: a 4-link loop stacked in layers always has one pin crossing a swept layer (all 24 orders failed), so the base is two plates (bottom holds O2, top holds O4) tied by a post placed clear of the sweep, as real clamps are built. Fusion: zero overlap over the stroke, arm swing 90.0°
- [x] **F6. Ratchet and pawl.** Undercut saw teeth, pawl pivot on the tangent at the contact, self-engagement tan(hook) > μ; tooth / pawl / pin / shaft sizing; the pawl's lift over each tooth from a collision search (plain bisection failed near the root corner: more lift is not always clearer). Fusion: wheel and pawl in one layer, the add-in's table follower (generalised from the Geneva: follower joint, oscillating table, free direction) makes the pawl ride and click. Findings: an undercut tip overhangs a radial line, so the pawl's front must follow the face line past the tip; the deep corner is placed where both clearance-offset lines meet

## G. Power transmission
- [ ] **G1. Helical and bevel gears, worm gear** (self-locking)
- [ ] **G2. Planetary gear set**
- [ ] **G3. Belt and pulley (V, timing), chain and sprocket**
- [ ] **G4. Rack and pinion**
- [ ] **G5. Power screw (trapezoidal).** Raising torque, self-locking, efficiency

## H. Machine elements
- [ ] **H1. Bolted joint.** Preload, separation, fatigue (Shigley ch. 8)
- [ ] **H2. Welded joint**
- [ ] **H3. Splined shaft, fits (H7/k6)**
- [ ] **H4. Torsion, extension and disc (Belleville) springs; spring fatigue (Zimmerli)**
- [ ] **H5. Coupling, brake / clutch, flywheel**

## I. For 3D printing
- [ ] **I1. Snap-fit**
- [ ] **I2. Flexure / living hinge**
- [ ] Print-ready pin clearances (B8c)

## L. Repository
- [x] **L2. The whole repository in English.** Code, comments, panel texts, messages, ROADMAP, docs (Turkish sentences are still understood)
- [x] **L3. Rename the add-in folder / id** to `MechanismAssistant` (re-registered in Fusion once; documents drawn under the old tag group are still found and re-tagged)

## K. Usability
- [x] **K1. Edits in the chat.** "Make the crank 40 mm", "use aluminium", "krankı 5 mm uzat", "devri iki katına çıkar": the LLM lists the changes (field, set / add / multiply, amount), the engine applies them to the design in the panel and redesigns. Every amount must appear in the message (unit conversions allowed); "a bit longer" gets a question, not a guess. llm_eval 36/36 (11 edit cases)
- [ ] **K2. Torque curve and dynamic forces for the four-bar** (also needed for the flywheel)

## D. Outside world
- [ ] **D12. Physical print** (a friend's printer)
- [ ] **D13. README and demo video** (portfolio). README + Fusion images done (parts in the panel colours); the video is to be recorded: shot list in `docs/demo.md`
- [ ] **D14. Later:** FreeCAD / AutoCAD support, a "free mode" (unverified, clearly labelled)

## Findings (for the report)
- **For Q = 1** the relation is `c = a / sin(ψ/2)`. There is a trade-off between compactness and transmission angle.
  As the coupler gets longer, μ_min approaches 90 − ψ/2. So for ψ > 90° the 45° target cannot be met.
- **Quick return (Q > 1) makes the transmission angle worse.** For ψ = 90°, μ_min is
  34° / 30° / 27° at Q = 1.1 / 1.2 / 1.3. For a large swing the answer is a gear stage or a six-bar linkage.
- **Synthesis alone is not reliable.** The first quick-return version said "110° ✓" while the simulation measured 47.7°
  (the extreme positions had fallen on different assembly branches). This is why every design is verified by an independent simulation.
- **Three-way agreement:** for a 35 mm crank and 55° swing, the formula (Norton), an independent simulation (55.00°) and
  Fusion's assembly solver (crank joint turned through the bridge, 54.99°) give the same result.
- **Coupler force F_AB = F / sin μ:** as the transmission angle falls the coupler and pin loads rise (μ = 45° → 1.41F).
  The strength calculation shows in numbers why the transmission-angle check is needed.
- **Wrong intuition, right engine:** the assumption "a short crank should get a smaller section" was wrong. The crank is bent by the
  drive torque (needs width), while the ground link carries only axial load (buckling, needs thickness).
  5 kg / PETG example: crank 21×3, coupler 13×3, rocker 24×3, ground 11×4 mm.
- **Stiffness governs in aluminium:** in the 5 kg example the rocker's stress utilisation is 57 % while its deflection utilisation is 94 %.
  In metal the part is sized by the deflection limit, not by breaking. In PETG the deflection limit widened the rocker from 24 to 29 mm.
- **Local FEA (gmsh + scikit-fem, P2 tetrahedra) verification:** the mid-section stress agrees with the hand calculation within 0.1–5 %
  (beam theory confirmed). The deflection is larger than the ideal clamp: Al 9×3 rocker +13 %, PETG 29×3 rocker +311 %.
  Reason: a wide, stiff link rotates around a small (Ø4) hole (hub compliance). Mesh convergence was checked.
- **Hub model (FEA calibration):** a link held at a hole has rotational stiffness k = C·E·t·r². Over 21 FEA runs
  C is independent of the length (±3 %, L² scaling confirmed) and 5.96–9.17 for d/w ≤ 0.3. C = 6 on the safe side.
  Effect: for a 40 mm link, 29 mm wide, on a Ø3 pin the hub deflection is 11 times the beam bending.
- **A hidden unsafe design caught:** once shaft torsion was checked, the Ø3 common pin of an earlier aluminium design turned out to reach
  τ ≈ 555 MPa at 2.94 N·m output torque (even steel breaks). Torque-carrying joints and free pins must be treated
  separately. PETG example (shaft Ø12, pin Ø4): model deflection 24–35 % on the safe side of the FEA, stress within 6–11 %.
- **FEA with bosses:** the model deflection is 24–54 % above the FEA (the boss stiffens the area around the shaft; the model is on the safe side),
  stress within 1–12 %. A thick shaft only enlarges its own boss: the PETG coupler and ground link came down to a 6 mm web.
- **Function generator (Freudenstein):** 40 mm input arm, 60° input → 90° output: simulation 89.0°, Fusion solver 89.4°
  (structural error < 1°, zero at the precision points). The mechanisms found are mostly non-Grashof (triple rockers): the input does not turn fully.
  "Most compact" produced links too short for their bosses; the choice was changed to "balanced ratio".
- **Gear verification (Fusion):** ratio 3 / 2 N·m / PLA → m = 2, z = 18/54, b = 16 mm. At five turning positions the overlap volume
  between pinion and gear is 0 mm³ (correct involute phase); pinion 90° → gear −30° (ratio and direction correct).
- **Spring verification (Fusion):** 100 N / 20 mm → Ø1.6 wire, D = 9.6 mm, 15 coils; in Fusion length 50.2 mm (L₀ = 50.2), outer diameter 11.2 mm.
  Conical spring (50 N / 15 mm, D₂/D₁ = 0.5): length 30.71 mm, outer diameter 16.1 mm, same as the design. The conical rate formula
  was checked coil by coil with a numerical integral (test).
- **One long LLM prompt does not scale:** adding the spring fields broke an older four-bar sentence (15/16).
  The two-stage structure (router + module extractor) gave 16/16 and, with shorter prompts, stayed at ~6 s.
- **Cam verification (Fusion):** 20 mm lift, 120/60/120° cycloidal → R_b computed; with the cam turned to 8 angles and the follower
  placed from the s(θ) table, the smallest roller-cam distance is 0.000 mm and the overlap 0.00 mm³ (exact contact).
  Turning the other way gives 1.8–15.5 mm gaps / 640 mm³ overlap: the test discriminates. A bug found on the way: given a custom
  direction (root axis) the slider joint ignored it and slid along z; the follower seemed not to move at all.
- **Shaft: the critical section may be the bearing seat, not the hub.** 60 N·m bending + 100 N·m torque, plain shaft: Ø20.6 mm is enough
  at the hub, but the torque-carrying Ø20 bearing seat (sharp shoulder, K_f 2.7/2.2) reaches n_f = 2 only with a Ø28 body
  (the bearing shoulder needs Ø25.4). Deflection (16 µm) and slope (0.35 mrad) are far below the limits. The numerical deflection
  matches the closed form within 1e-4; the Rayleigh critical speed is 0.7 % above the exact one (expected: an upper bound).
- **A hidden clash in the system:** while building the reducer in Fusion, the gear's bore cut also cut the shaft running through it
  and split it in two (in Fusion a cut hits every body it touches by default). A bug invisible in the gear module alone appeared
  once the modules were combined; every cut is now limited to its own body.
