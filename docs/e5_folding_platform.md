# E5: folding platform (proposal, to be decided)

A wall- or frame-mounted platform that folds from horizontal (in use) to vertical (stowed), moved by a four-bar and
balanced by a spring so a person can fold it with a small force. Everything below is built from modules that
already exist and are verified; nothing is started until the questions at the end are answered.

## How the existing engines would be combined

| Step | Engine (existing) | What it gives |
|---|---|---|
| Motion | `function_gen.py` (limited-input four-bar, Freudenstein) or a crank-rocker | the platform turns 90° (or the asked angle) while a handle / lever turns through its own angle; transmission angle checked over the whole fold |
| Gravity torque | statics along the simulated motion (as `strength.analyze_forces`, `dynamics.py` for speed) | the moment the platform's weight (and a user load) makes about the pivot at every angle |
| Counterbalance | `torsion_spring.py` or `extension_spring.py` (with `springs.py` fatigue) | a spring whose moment follows the gravity moment as closely as possible; the residual (unbalanced) torque curve is what the user feels |
| Structure | `strength.py` link sizing, `welded.py` / `bolted.py` for the brackets, `fits.py` / pins | links, platform beam in bending (L/200), pins and the wall brackets |
| Drawing | `fusion_lib` four-bar + spring + plate builders | the folding assembly, turnable in Fusion (the add-in's 1 turn / Loop) |

The design check that matters most: **residual torque over the fold** (spring vs gravity) and the **locked
positions** (a stop or an over-centre lock at 0° and 90°, as the toggle clamp does).

## Questions for Arhan

1. Platform: size (L × W), mass, and the user load on it (static? a person standing on it?).
2. Fold angle (90°?), and where it is mounted: wall, vehicle, machine frame?
3. Moved by hand (a handle force limit, e.g. 50 N?) or by a motor / actuator?
4. Counterbalance: a torsion spring at the pivot, extension springs on the linkage, or a gas spring (needs maker data)?
5. Locking: stops only, an over-centre toggle, or a pin?
6. Material and making: steel / aluminium profiles, welded or bolted; 3D-printed model at scale for the demo?
7. Any standard to follow (e.g. a safety factor from the course or the company)?
