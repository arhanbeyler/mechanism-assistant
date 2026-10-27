# Mechanism Assistant: solver-grounded mechanism design in Fusion 360

Describe a machine element in plain language and get a **sized, checked and drawn** Fusion 360 assembly
that you can rotate. Everything runs locally: a local LLM (Ollama), deterministic Python engines and local FEA.

**Website:** [arhanbeyler.github.io/github-slideshow](https://arhanbeyler.github.io/github-slideshow/), a scroll-driven
tour in which the planetary gear set, four-bar, slider-crank, Geneva and cam turn in 3D as you scroll, built from
the engines' default designs.

> *"turn a 4 cm arm 45° and the other arm turns 120°, PETG, 2 kg"*
> → Freudenstein function generator, strength and interference checks → four-bar in Fusion with joint limits
>
> *"1.5 kW 1450 rpm motor, 1:4 gearbox"*
> → gear pair, both shafts, bearings, keys and housing sized together → a reducer you can turn in Fusion

Sentences can be written in English or Turkish.

#### In motion (Fusion's own solver turning the drawn assemblies; `tools/make_gifs.py`)

| Four-bar | Slider-crank | Dwell six-bar |
|---|---|---|
| ![four-bar](docs/img/anim/fourbar.gif) | ![slider-crank](docs/img/anim/slider_crank.gif) | ![dwell](docs/img/anim/dwell.gif) |
| **Quick return** | **Planetary gear set** | **Worm gear** |
| ![quick return](docs/img/anim/quick_return.gif) | ![planetary](docs/img/anim/planetary.gif) | ![worm gear](docs/img/anim/worm_gear.gif) |
| **Crank press (flywheel drive)** | **Compression spring** | **Torsion spring** |
| ![crank press](docs/img/anim/press.gif) | ![spring](docs/img/anim/spring.gif) | ![torsion spring](docs/img/anim/torsion_spring.gif) |
| **Snap-fit** | **Living hinge** | |
| ![snap-fit](docs/img/anim/snap_fit.gif) | ![living hinge](docs/img/anim/flexure.gif) | |

#### Linkages

| Four-bar (crank-rocker / rocker-rocker) | Slider-crank (piston) | Straight-line linkage (Hoeckens) |
|---|---|---|
| ![four-bar](docs/img/fourbar.png) | ![slider-crank](docs/img/slider_crank.png) | ![straight-line](docs/img/straight_line.png) |
| **Toggle clamp (over-centre)** | **Scotch yoke** | **Quick return (shaper)** |
| ![toggle clamp](docs/img/toggle_clamp.png) | ![scotch yoke](docs/img/scotch_yoke.png) | ![quick return](docs/img/quick_return.png) |
| **Whitworth quick return** | **Watt six-bar (large swing)** | **Dwell six-bar** |
| ![Whitworth](docs/img/whitworth.png) | ![six-bar](docs/img/six_bar.png) | ![dwell](docs/img/dwell.png) |
| **Path synthesis (through given points)** | **Crank press (system)** |  |
| ![path synthesis](docs/img/path_synthesis.png) | ![crank press](docs/img/press.png) |  |

#### Intermittent motion and cams

| Geneva (intermittent rotation) | Ratchet and pawl | Disk cam + roller follower |
|---|---|---|
| ![geneva](docs/img/geneva.png) | ![ratchet](docs/img/ratchet.png) | ![cam](docs/img/cam.png) |

#### Gears

| Spur gear pair | Helical gear pair | Bevel gear pair |
|---|---|---|
| ![gear](docs/img/gear.png) | ![helical gears](docs/img/helical_gear.png) | ![bevel gears](docs/img/bevel_gear.png) |
| **Worm gear (self-locking)** | **Rack and pinion** | **Planetary gear set** |
| ![worm gear](docs/img/worm_gear.png) | ![rack and pinion](docs/img/rack_pinion.png) | ![planetary](docs/img/planetary.png) |
| **Single-stage reducer (system)** | **Two-stage reducer (system)** |  |
| ![reducer](docs/img/reducer.png) | ![two-stage reducer](docs/img/reducer2.png) |  |

#### Chain and belt drives

| Roller chain drive | V-belt drive |  |
|---|---|---|
| ![chain drive](docs/img/chain_drive.png) | ![V-belt drive](docs/img/belt_drive.png) |  |

#### Shafts, screws and couplings

| Two-bearing shaft (fatigue) | Splined shaft (ISO 14) | Involute spline (DIN 5480) |
|---|---|---|
| ![shaft](docs/img/shaft.png) | ![spline](docs/img/spline.png) | ![involute spline](docs/img/spline_din5480.png) |
| **Press fit (ISO H7 fits)** | **Flange coupling** | **Disc clutch** |
| ![press fit](docs/img/press_fit.png) | ![flange coupling](docs/img/coupling.png) | ![disc clutch](docs/img/clutch.png) |
| **Flywheel** | **Power screw (Tr, modelled thread)** |  |
| ![flywheel](docs/img/flywheel.png) | ![power screw](docs/img/power_screw.png) |  |

#### Bolted and welded joints

| Bolted joint | Welded bracket |  |
|---|---|---|
| ![bolted joint](docs/img/bolted_joint.png) | ![welded bracket](docs/img/welded_joint.png) |  |

#### Springs

| Conical compression spring | Extension spring | Torsion spring |
|---|---|---|
| ![spring](docs/img/spring.png) | ![extension spring](docs/img/extension_spring.png) | ![torsion spring](docs/img/torsion_spring.png) |
| **Disc spring stack** |  |  |
| ![disc springs](docs/img/disc_spring.png) |  |  |

#### 3D printing

| Snap-fit (printed) | Living hinge (printed) |  |
|---|---|---|
| ![snap-fit](docs/img/snap_fit.png) | ![living hinge](docs/img/flexure.png) |  |

## Core rule: the LLM never produces a number

Existing "LLM → CAD" tools let the model write geometry code directly. The results look plausible but nothing
checks them. This project splits the work into three parts:

```mermaid
flowchart LR
    A[Sentence] --> B["Local LLM (Ollama, qwen3.5 9B)<br/>intent → strict JSON only"]
    B --> C["Deterministic engines<br/>synthesis + sizing"]
    C --> D["Independent checks<br/>simulation, strength, interference, local FEA"]
    D -->|pass| E["Fusion 360 add-in<br/>components, joints, animation"]
    D -->|fail| F["Explain why + nearest feasible value"]
```

* **LLM:** It only fills a JSON schema (strict output, two stages: a router picks the module, then a short
  module-specific extractor fills its fields). Missing values are reported, never guessed. A regex backstop
  catches input angles and speeds the model misses, a force written in N / kN that the model took for a mass, and a worm's output torque the model dropped. The evaluation set is `tools/llm_eval.py` (122/122, incl. 15
  edits of an existing design).
* **Edits:** once a design is on screen, "make the crank 40 mm", "use aluminium" or "krankı 5 mm uzat" change it.
  The LLM lists the changes; every amount must appear in the message, so "a bit longer" gets a question, not a guess.
* **Engines:** Textbook methods (Norton, Shigley) in plain Python. Every result is re-verified by a method that
  is *independent* of the one that produced it. For example, synthesis is checked by position-loop simulation.
* **Fusion:** The add-in only draws what the engines decided. It reuses one tagged document instead of opening
  new tabs.

## Modules

#### Linkages

| Module | Design method | Checks |
|---|---|---|
| Four-bar, full rotation | Crank-rocker Q = 1 (c = a / sin(ψ/2)), quick-return Q > 1 (Norton) | Grashof, transmission angle 40–140° (synthesis targets 45°), branch consistency, simulated swing and Q; with a crank speed (K2): drive torque, pin and shaking forces with the links' inertia by Norton's matrix method (checked against the energy method and, at a slow crank, against the statics; the mean torque equals the load's work per turn), sections resized when the dynamic forces are larger, the flywheel that keeps the speed within ±2.5 %, torque curve on the panel |
| Four-bar, limited input | Freudenstein function generator, 3 Chebyshev precision points | Structural error ≤ 1°, transmission angle over the range, Fusion joint limits |
| Watt six-bar | Chosen automatically when a four-bar cannot give the swing (over 90°, up to 300°): a crank-rocker drives, through its rocker's second arm, a Freudenstein function generator | The whole six-bar simulated over a crank turn (both loops, same branches): swing within 1°, both transmission angles 40–140°, every link buildable (≥ 12 mm, ratio ≤ 10); stage 2's load moved to stage 1 by virtual work; in Fusion (8 links, a rigid joint makes the ternary link) the solver's output swing equals the engine's (149.43° vs 149.42°, 181.0° vs 181.0°) |
| Dwell six-bar | Crank, dwell (degrees of the crank turn) and output swing: a four-bar whose coupler point runs on a circular arc during the dwell, a link of the arc's radius to its centre, and the output rocker | Arc found by optimisation (Kasa circle fit, trivial arcs of A or B excluded, the point must leave the circle elsewhere); the whole six-bar simulated: the output moves 0.03° while the crank turns 90°, swing and both transmission angles checked; in Fusion the solver gives 0.035° and 40.06° (engine 40.06°) |
| Slider-crank (piston) | Stroke given; crank from the exact stroke formula (with offset), rod the shortest whole mm with b/a ≥ 3 and rod angle ≤ 20° | Simulated stroke, rod ratio, rod angle (side load), quick-return Q with offset; crank torque by virtual work T = F·|dx/dθ| (tested against the rod-force moment); crank and rod sized like the four-bar links; piston speed / acceleration; in Fusion the solver's piston position equals x(θ) at 36 angles |
| Straight-line linkage | Hoeckens four-bar (a : b : c : d = 1 : 2.5 : 2.5 : 2, point at 2 × AB) scaled to the asked straight path; the straight part is found on the simulated coupler curve (longest run within 0.2 % of its length from its best-fit line), not taken from a table | Straight length, deviation, crank-rotation share and speed variation on the line, lift on return, transmission angle on the working stroke; crank torque from coupler statics equals virtual work; links sized with the coupler also bending (point overhangs B); layer clearance; in Fusion the drawn point stays within 0.003 mm of the engine curve over a full turn |
| Path synthesis | 3–8 points (x, y) the coupler point must pass through; a crank-rocker with the point on the coupler's line (a three-hole bar) | Nelder–Mead from 40 seeded starts on the squared distances to the coupler curve with penalties for Grashof, transmission angle 30–150° and link ratios; the report re-measures the misses on a finer curve; in Fusion the solver-driven point, traced over a turn, passes the points within 0.2 mm (1° steps) |
| Toggle clamp | Arm opening, spindle distance (handle travel, handle length defaulted and reported); dead-centre synthesis (handle link and coupler in line at the clamped pose) swept and refined for the transmission angle, then 3° over centre onto a stop | Opening (simulation), arm back-off past the dead centre, transmission angle near the lock (40..140°) and over the stroke (≥ 20°); handle torque from statics equals virtual work and falls to 0 at the dead centre; locked-pose statics (coupler, arm pivot, stop torque), steel pins, arm bending at B; split base plates + post clear of the sweep; in Fusion zero overlap over the stroke, arm swing exactly as asked |
| Scotch yoke | Stroke (or crank); crank r = S/2, pin in a slot square to the slide | Exactly harmonic x = r cos θ (a_max against a slider-crank's), crank torque F·r, slotted yoke bar in bending + slot bearing, pins, shaft; in Fusion the yoke follows r cos θ exactly over a turn, no overlap |
| Quick return (shaper) | Stroke and time ratio Q: β = 180/(Q+1), r/d = cos β, lever L = S/(2 cos β), pin travel kept 15 % short of the lever tip | Simulated stroke and Q, slow-stroke direction, cutting / return speeds, crank torque from statics = virtual work, slotted lever bending, pins, shaft (up to Ø30); in Fusion two pin-slot joints, the ram follows L sin ψ exactly over a turn, no overlap |
| Whitworth quick return | Stroke and time ratio, 'Whitworth' named (or chosen on the panel): the crank longer than the pivot distance, so the slotted lever turns full circles and drives the ram through a rod | Q = (180 + 2α)/(180 − 2α), sin α = d/r; stroke, Q and the slot's radius range from the simulation (tested); rod angle; crank torque by virtual work; in Fusion the crank is a ring round a fixed hub that carries the lever pivot (as on the real machines), stroke 200.0 mm and Q 1.50 measured from the solver, no overlap over a turn |
| **Crank press (system)** | Press force, stroke, the part of the stroke it acts over (10 % if not given), strokes per minute: the slider-crank, its torque curve by virtual work, the energy per stroke, the flywheel on the fast shaft (4:1 belt, gears to the crank) and the motor, all from one torque curve | Work per stroke = F s_w (the integral of the torque, tested), the motor supplies the mean power while the flywheel covers the fluctuation (the same flywheel on the crankshaft would need the ratio² more inertia, tested), crank and rod sized for the working-zone forces with larger steel pins; in Fusion a pinion on the flywheel's shaft drives a large gear on the crankshaft (drawn to show the motion, not sized): zero overlap, the flywheel turns at the ratio the other way (9.05× at the defaults) |
| Link sizing | Per-link width/thickness for load and material (PLA, PETG, Al 6061, steel), local bosses D = 2.5d | Euler buckling, net section, bending, pin shear and bearing, Roark K_t, L/200 deflection incl. FEA-calibrated hub compliance, steel shaft torsion, layer interference ≥ 1 mm |

#### Intermittent motion and cams

| Module | Design method | Checks |
|---|---|---|
| Geneva | Slots (or step angle) and centre distance; tangential pin entry a = C·sin(π/n); wheel with slots and locking arcs, driver with locking disk whose relief is sized by sweeping the wheel outline over the index; with speed and a load (disk mass + diameter) the pin and wheel thickness are sized | Index / dwell split, closed-form ω and α (tested against numerical derivatives), peak ω = λ/(1−λ), pin force from J·α, pin shear and bending, slot bearing, both shafts, lock engaged < 1° after the pin leaves, material under the slots, rim vs driver shaft; point-sampled interference test; in Fusion zero overlap over two turns driven by the add-in |
| Ratchet and pawl | Holding torque (teeth, diameter optional); saw teeth with an undercut locking face, pawl pivot on the tangent at the contact; smallest wheel that passes, face width ≤ one pitch | Self-engagement tan(hook) > μ (hook = friction angle + 5°), tooth bending / shear / face bearing, pawl buckling and compression, pivot pin and shaft (steel); the pawl's ride over the teeth found by collision search (least lift, 0.3° margin); in Fusion 0.2 mm clearance at rest and the pawl riding in contact, no overlap |
| Disk cam | Cycloidal, harmonic or 3-4-5 motion; smallest base circle | Pressure angle ≤ 30°, no undercut, ρ ≥ 2·R_roller |
| **Cam + follower spring (system)** | With speed and follower mass: the follower's acceleration sets the spring force needed against jump (1.3 × inertia); the preload is swept and the spring module designs the lightest spring that also passes with its real (rounded) rate | Contact force > 0 over the whole turn (recomputed from the motion table), all spring checks at the real top force |

#### Gears

| Module | Design method | Checks |
|---|---|---|
| Spur gears | Lewis bending (Shigley Y table, Barth K_v), involute teeth | Contact ratio ≥ 1.2, undercut z ≥ 17, Hertz contact stress (info), shaft and hub |
| Helical gears | Ratio, input torque, helix angle (default 20°, reported); normal module, opposite hands; the lightest pair that passes | Shigley's helical geometry reproduced (d, p_x, φ_t), Lewis in the normal plane with the virtual tooth number z/cos³ψ, hobbed K_v, transverse contact ratio ≥ 1.2 and face contact ratio ≥ 1 (the helix overlaps), no undercut 2cosψ/sin²φ_t, axial thrust reported, shafts and hubs; in Fusion each tooth is swept along the axis with a twist (an exact helicoid), teeth in contact with zero gap and no overlap while turning |
| Bevel gears | Ratio 1–6, input torque, straight teeth, shafts at 90°; the lightest pair that passes | Pitch cones and one apex, Tredgold's virtual teeth z/cos γ for Lewis (at the mean section), contact ratio and undercut, face ≤ A_o/3 and 10 m, forces W_t, W_r = W_t tan φ cos γ, W_a = W_t tan φ sin γ (Shigley's example reproduced: 406 → 140 / 46.7 lbf; swapped on the gear), shafts and hubs; in Fusion the virtual tooth on the back cone lofted to its copy scaled towards the apex (straight elements through the apex), contact with no overlap while turning |
| Worm gear | Ratio 5–100, worm speed, output torque (or input torque / power: the output follows the efficiency, iterated), self-locking on request; starts 1–4, standard module, diameter quotient q; the smallest set that passes | Shigley 15-4 / AGMA: wheel rating C_s d_G^0.8 F_e C_m C_v, sliding speed, friction curve, efficiency (tested against the force ratio), worm and wheel forces, self-locking with a 40 % margin on the static friction (λ ≲ 5.5°), worm not too slender, N_G ≥ 21, shafts; Shigley example 15-4 reproduced (λ 4.764°, f 0.0178, e 0.818, W_Gt 1050 lbf, (W_t)all 1456 lbf); in Fusion an Archimedean thread (its transverse section swept with a twist of 2π per lead) on a helical wheel, zero overlap over a worm turn |
| Rack and pinion | Force on the rack, travel (default 200 mm, reported), speed optional; the lightest pinion (z 18..40, standard module, face 8..12 m) + rack that passes | Pinion torque F·d/2, travel π·d per turn, Lewis bending of pinion (Y(z)) and rack (Y = 0.485), Barth K_v, contact ratio with a rack ≥ 1.2, no undercut, steel shaft and hub web; in Fusion a motion-linked pinion and rack, zero tooth overlap over 10 teeth and rack travel exactly π·d·θ/360 |
| Planetary gear set | Ratio (3–12, sun in, carrier out, ring fixed), input torque or power, planets 3–5 (or the lightest); tooth numbers that give the ratio within 2 % AND assemble ((Z_s + Z_r)/n whole) AND clear their neighbours; the smallest ring that passes | Willis ratio (tested against pitch-point velocities), ring reaction from statics, load sharing K_γ (assumed, reported), Lewis bending of sun, planet as an idler (×0.7) and ring, both contact ratios (internal formula tends to the rack's), steel planet pins as cantilevers, sun and carrier shafts; mesh phases computed for every planet; in Fusion the carrier turns at exactly 1/i and each planet by the Willis relation, contact only (< 0.001 mm³) |
| **Reducer (system)** | Gear pair → gear forces and masses → both shafts → bearings → keys → housing, iterated until the constraints agree (gear bore = shaft, key ≤ face width, bearings side by side, common span) | All of the above + housing web; in Fusion, tooth overlap 0 mm³ at 6 angles and the output turns at exactly 1/i |
| **Two-stage reducer (system)** | Ratios above 6: the ratio is split over two stages (lightest gear set near √i); three shafts, the intermediate one carries two gears (loads in two planes, reactions, M/EI deflection and Rayleigh speed with several masses); gear bores, key lengths, bearing seats and the common span iterated together | Per shaft: fatigue / yield at every seat, keys, bearing life, slopes and deflections at each gear; gear 2 clears the output shaft, gear 4 clears the input shaft, material between bearing seats; in Fusion, tooth overlap 0 in both stages at 5 angles, both ratios exact |

#### Chain and belt drives

| Module | Design method | Checks |
|---|---|---|
| Roller chain drive | Power (or torque), input speed, speed ratio, load type (service factor, default moderate shock, reported), centre distance optional; ANSI / ISO 606 chains no. 25–160, 1–3 strands, small sprocket 17–25 teeth; the smallest sprocket pair that passes | Rating per strand from Shigley's link-plate and roller-bushing formulas (reproduce Table 17-20), multi-strand factor, chain pull vs breaking load (≥ 7), whole even number of links and the exact centre distance, wrap ≥ 120°, chordal speed variation, steel shafts and hub web; in Fusion the plates clear the teeth and the driven sprocket turns at exactly N1/N2 |
| V-belt drive | Same inputs; classical A / B / C sections, standard belt lengths; the smallest sheave pair (with the real sheave width) that passes, at most 5 belts | Shigley 17-4: tabulated rating, angle-of-contact K1 and length K2 factors, number of belts and n_fs, belt tensions and the initial tension to set, belt life (Table 17-17), belt speed ≤ 25 m/s, centre distance, steel shafts; Shigley example 17-4 reproduced (F1 = 64.4 lbf, F_i = 32.2 lbf); in Fusion each belt sits 0.2 mm off both groove walls |

#### Shafts, screws and couplings

| Module | Design method | Checks |
|---|---|---|
| Shaft | DE-Goodman fatigue with Marin factors, k_b iteration, Table 7-1 K_t; catalogue ball bearings; DIN 6885 key; diameter raised until every check passes | Fatigue n_f and yield at the hub and at the torque-carrying bearing seat, L10 life ≥ 20 000 h, key shear and crushing, slope ≤ 1 mrad and deflection ≤ 0.13 mm (numerical integration on the stepped shaft), Rayleigh critical speed ≥ 2× running speed |
| Splined shaft | Torque, fixed or sliding hub (flank pressure 100 / 40 / 10 MPa); the smallest ISO 14 straight-sided spline (light / medium series) with a hub ≤ 1.5 D | Flank bearing with 75 % of the teeth carrying (formula tested), shaft torsion on the minor diameter, hub length; in Fusion the splined shaft and a hub with a 0.1 mm clearance spline hole (measured); a sliding hub is shown sliding one hub length along the shaft |
| Involute spline (DIN 5480) | Torque, use as above, 'involute' / 'DIN 5480' in the sentence or the panel's Profile choice; the smallest W d_B × m × z with a hub ≤ 1.5 d_B | Sizes by the standard's rules (profile shift −0.05 … 0.45 fixes z: W 25×1.25×18, W 30×2×14, W 100×3×32 reproduced), tip / root diameters, 30° involute; the same flank-pressure rule, shaft torsion on the root, tip not pointed. Fusion: involute shaft and hub, zero overlap, 0.05 mm flank gap, cross-section equal to the integrated involute outline (839.4 mm²) |
| Press fit | Torque, shaft diameter (sized in torsion when not given), hub size optional; the loosest H7 fit (k6 … s6) and the shortest, slimmest hub that pass | ISO 286 limits (Shigley's H7/s6 48 mm example reproduced), Lamé pressure (same-material closed form tested), torque at the SMALLEST interference ×2, hub von Mises at the LARGEST, assembly temperature; says honestly when no standard fit works (small shafts: use a key or spline) |
| Flange coupling | Torque (service factor 1.5), shaft (sized in torsion when not given), bolts optional; the usual proportions (hub 2d × 1.5d, bolt circle 3d, flange 4d × 0.5d) | Key (DIN 6885) shear, crushing and its fit in the hub; bolts carrying the torque by friction μ N F_i r (the next smaller bolt tested to slip); hub torsion, flange shear; says when the shaft is too thin; in Fusion both hubs, shafts, bolts with a cosmetic thread and nuts, turning together as one rigid group on two bearing blocks |
| Disc clutch | Torque (or power and speed), lining, max diameter optional; service factor 1.3 | Shigley 16-5 uniform wear: T = N π f p_a r_i (r_o² − r_i²) (both forms tested equal), the best r_i = r_o/√3 found numerically, clamp force; more plates when the diameter is limited; in Fusion flywheel, friction disks, intermediate and pressure plates, input and output linked 1:1 (engaged) |
| Flywheel | Energy fluctuation (or the work of one press stroke and the share of the cycle it takes), mean speed, C_s (from the application, reported), max diameter optional | I = ΔE/(C_s ω²) (tested: the energy between the speed limits is ΔE), steel disk t = 0.1 D, rotating-disk stress, rim speed, speed range; in Fusion the disk turns on its shaft between pedestals |
| Power screw | Axial load, travel, ends (both supported / jack), nut material; smallest ISO trapezoidal size that passes AND self-locks, nut at least one diameter long | Raising / lowering torque (Shigley 8-1 reproduced: 15.94 N·m, −0.47 N·m), self-locking with lubricated friction, efficiency, body von Mises, Euler / Johnson column, thread bearing, root bending (first thread 38 %), nut thread shear; in Fusion a modelled Tr thread, the nut moves one lead per turn with zero thread overlap |

#### Bolted and welded joints

| Module | Design method | Checks |
|---|---|---|
| Bolted joint | Separating load (static or fluctuating), clamped thickness, plate material, bolt class, number of bolts (4 tried first); the smallest ISO metric bolt that passes, with nut, standard length | Shigley ch. 8: bolt stiffness from its threaded and plain lengths, member stiffness (Wileman; within 8 % of the frustum model), joint constant C (tested against the two-spring model), preload 0.75 F_p and tightening torque, yield / load / separation factors, Goodman fatigue (the load line meets the Goodman line at n_f, tested); in Fusion two flanges with clearance holes, hex bolts with a cosmetic ISO thread at the free end and nuts |
| Welded bracket | Load and its arm, bar section (sized in bending when not given), weld all around / top and bottom / sides; the smallest fillet leg that passes | Shigley ch. 9: welds as lines (unit area and second moment, tested by integrating along the lines), primary and bending shear on the throat combined, AISC 0.30 S_ut (E70), AISC minimum leg for the thickness, bar bending 0.6 S_y; in Fusion the wall, the bar and triangular fillet beads touching both |

#### Springs

| Module | Design method | Checks |
|---|---|---|
| Compression spring | Cylindrical or conical, Shigley ch. 10 | Bergsträsser, S_ut = A/dᵐ, solid-safe, buckling, conical rate k = Gd⁴ / (16Nₐ(R₁+R₂)(R₁²+R₂²)); fatigue for a cycling force (Zimmerli + Gerber, the Gerber point tested on the curve) |
| Extension spring | Force and the extension to reach it; the lightest music-wire spring with full-loop hooks | Shigley 10-10: initial tension from the preferred stress range, N_a = N_b + G/E, force = F_i + k y (tested), body torsion and the hooks' bending (A) and torsion (B), which govern; in Fusion a close-wound helix with loop hooks |
| Torsion spring | Moment at a working angle, arbor optional; the lightest music-wire spring in whole quarter body turns | Shigley 10-12: inner-fibre stress with K_i, S_y = 0.78 S_ut, rate d⁴E/(10.8 D N_a) with the legs' share, winding-down of the coil onto the arbor; Shigley example 10-8 reproduced (M 7.55 lbf·in, k' 29.2 lbf·in/turn, 93°); in Fusion a close-wound helix sweep with straight legs |
| Disc spring stack | Force and the deflection at it, guide rod optional; the lightest stack of DIN 2093-proportioned discs (series A / B / C), nested and in series | DIN 2092 force and σ_OM (the 40 mm catalogue discs reproduced: 6.5 / 2.6 / 1.0 kN), working deflection ≤ 0.75 h0, stress ≤ 1400 MPa (51CrV4), stack deflection within 10 %, free and loaded length; in Fusion the discs revolved as cones, alternating, on a guide rod |

#### 3D printing

| Module | Design method | Checks |
|---|---|---|
| Snap-fit (printed) | Undercut, PLA / PETG, hold force optional, permanent or releasable, repeated use; the shortest printable cantilever | Bayer / Covestro cantilever method: root strain 1.5 y h / l² within the material's allowance (tested against beam theory), deflection force, snap-in and pull-out forces over the ramp and the retaining face (a square face locks); in Fusion the snap and the catch against its hook, touching |
| Living hinge (printed) | Bend angle, PLA / PETG, a few or many cycles; the shortest, thinnest printable strip | A strip in pure bending: strain t θ/(2L) (tested as t/2 over the radius) kept elastic (S_y/E, halved for many cycles), stiffness E I / L (tested as E I / ρ), moment at the full bend, the pivot's wander along the strip |

In Fusion: **⟳ 1 turn** and **▶ continuous** buttons drive the input joint (gear motion link, cam follower from
the s(θ) table, Geneva wheel from its φ(θ) table, ratchet pawl riding the teeth, rocker-rocker oscillation within limits). Springs (compression, extension, torsion, disc), the snap-fit (the catch slides in, the beam bends just enough for the hook to pass, then snaps back) and the living hinge (flat as printed, folding to its angle as a circular arc) are animated as pre-built frames. For a drawing with nothing that moves (bolted and welded joints, fits, a fixed spline hub) the two buttons are hidden; the flange coupling turns as one rigid group. The panel's Type list is grouped as in the tables above.
Sliders in the panel re-run the engine live and colour the swing slider by feasibility.

## Verification highlights

* **Three-way agreement:** For a 35 mm crank and 55° swing, the formula, the independent simulation (55.00°) and
  Fusion's own assembly solver (54.99°, measured through the bridge) agree. Function generator 60° → 90°:
  simulation 89.0°, Fusion 89.4°.
* **Synthesis alone is not trustworthy:** The first quick-return version reported "110° ✓" while simulation
  measured 47.7°, because the extreme positions fell on different assembly branches. This is why every design is
  simulated.
* **Local FEA (gmsh + scikit-fem, P2 tetrahedra, mesh convergence checked):** Stress agrees with the hand
  calculation within 0.1–12%. Deflection was up to 4× the ideal cantilever for wide links on small holes. A hub
  compliance model δ = P·L² / (C·E·t·r²) was fitted over 21 FEA runs (C = 6–9, independent of L; C = 6 is used
  as the safe side).
* **A hidden failure caught:** Once shaft torsion was checked, an earlier aluminium design turned out to put
  τ ≈ 555 MPa on a Ø3 common pin. Torque-carrying joints now get separate steel shafts.
* **Geometry checks in Fusion:** Gear teeth overlap 0 mm³ at 5 positions and the ratio is confirmed. Cam–roller
  gap is 0.000 mm with 0 mm³ overlap at 8 angles. Spring length and diameter match the design. Shaft length,
  diameter and volume match the layout for all 5 notch types.
* **Fusion's solver agrees with the engines:** Watt six-bar output swing 149.43° vs 149.42° and 181.0° vs 181.0°;
  the path-synthesis point traced through its target points within 0.2 mm; Whitworth stroke 200.0 mm and Q 1.50;
  planetary carrier ratio and planet rotation exact; worm, helical, bevel, chain, belt and rack drives turn with no
  tooth overlap.
* **Textbook examples reproduced (tests):** Shigley's V-belt example 17-4, worm example 15-4, torsion spring
  example 10-8, power screw example 8-1, helical geometry, bevel forces, the H7/s6 48 mm fit, chain ratings
  (Table 17-20) and the DIN 2093 disc-spring catalogue forces.
* **Independent checks inside the engines:** the four-bar's dynamic torque (matrix method) equals the energy method
  and, at a slow crank, the statics; press work per stroke equals F·s; the Willis ratio matches pitch-point
  velocities; weld line properties match integration along the lines.

More findings are in [ROADMAP.md](ROADMAP.md).

## Setup (Windows)

1. **Ollama** with the model: `ollama pull qwen3.5:9b`. The model runs 100% on a 12 GB GPU at about 4–6 s per
   request. Any OpenAI-compatible server works, for example LM Studio (change `BASE_URL` in `llm_extract.py`).
2. **Python 3.11+** for tests and local FEA: `pip install numpy scipy gmsh scikit-fem meshio pytest`.
   The engines themselves use only the standard library, because Fusion's embedded Python has no NumPy.
3. **Fusion 360 add-in:** Link the add-in folder into Fusion's AddIns directory, then start it from
   *Utilities → Add-Ins*:
   ```bat
   mklink /J "%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\MechanismAssistant" "<repo>\fusion_addins\MechanismAssistant"
   ```
   The add-in finds the repository through the junction.
4. For the **FEA button**, write the path of the Python from step 2 into a file named `.local_python` in the
   repository root.

## Usage

* **In Fusion:** Use the *Mechanism Assistant* panel. Type a sentence or pick a type and use the sliders, then
  press *Draw in Fusion*, *1 turn* / *Loop* and *FEA*.
* **Terminal:** `python main.py "30 mm crank, 60° swing, 5 kg at the tip, PETG"` writes `output/mechanism.json`
  (the *FourBar* script in `fusion_scripts/` draws it).
* **Tests:** `python -m pytest`. This runs 398 tests on formulas, simulation, sizing, FEA and geometry.
* **LLM evaluation:** `python tools/llm_eval.py` (requires Ollama running).
* **Fusion smoke test:** `python tools/fusion_smoke.py` builds every module's default design in Fusion through the add-in's bridge (36/36 build); create an empty `.bridge_enabled` file in the repository root first and restart the add-in.

Idea and direction: Arhan Beyler. Code written together with Claude (Anthropic) as the coding partner.

## Repository layout

```
pipeline.py          sentence/params → spec → module engine → checks → Fusion dict (MODULES = slider definitions)
llm_extract.py       two-stage LLM (router + per-module JSON-schema extractors)
kinematics.py        four-bar synthesis (Q = 1, quick-return), simulation, measurement
function_gen.py      Freudenstein function generator (limited-input four-bar)
strength.py          link, pin and shaft sizing; hub compliance; K_t; deflection
interference.py      layer order and shaft-to-link clearance over the cycle
fea_local.py         gmsh + scikit-fem link model (with hub) vs hand calculation
module engines (one per element), for example:
  gears.py helical.py bevel.py worm.py planetary.py rack_pinion.py        gears
  chain.py vbelt.py power_screw.py coupling.py clutch.py flywheel.py       power transmission
  springs.py torsion_spring.py extension_spring.py disc_spring.py         springs
  bolted.py welded.py fits.py spline.py din5480.py shafts.py                          joints and shafts
  coupler.py pathsynth.py sixbar.py dwell.py geneva.py toggle.py slotted.py whitworth.py ratchet.py slider_crank.py cams.py
                                                                           linkages and mechanisms
  dynamics.py press.py snapfit.py flexure.py                               dynamics, systems, printing
shaft_multi.py       shaft with several gear loads (two planes, deflection, critical speed)
reducer.py reducer2.py cam_system.py   systems: modules sized together until their constraints agree
fusion_lib/          Fusion builders (one per module) + shared document handling
fusion_addins/       the add-in: palette (HTML), animation, local HTTP bridge used during development
tests/               pytest suite
tools/               hub-compliance calibration, LLM evaluation, Fusion smoke test
```

## Limitations

* The scope is deliberate: each module has one textbook method (Shigley, Norton, DIN / ISO tables it quotes).
  Anything else gets "unsupported" instead of a guess.
* Inertia is included for the four-bar at a given crank speed (K2); the other mechanisms are sized quasi-statically.
  Plastics use a single yield value with no creep or anisotropy from printing.
* Chains, belts and some couplings are drawn static (a chain or a belt looks the same when it moves); gears and
  linkages move with Fusion joints.
* The FEA covers the most loaded link as a cantilever with its hub. It is not a full-assembly contact analysis.
* The development bridge (127.0.0.1, token-protected) executes Python inside Fusion. It is off by default and
  starts only when an empty file `.bridge_enabled` exists in the repository root (needed for the tools below).

Planned next (after v1.0): see [ROADMAP.md](ROADMAP.md): print-in-place pins after a physical print, a demo video, the folding platform.

## License

MIT, see [LICENSE](LICENSE).
