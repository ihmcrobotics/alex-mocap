# CLAUDE.md — working notes for `alex-mocap`

Context that is expensive to rediscover. `FRAMEWORK.md` is the authoritative spec and
`RUNNING.md` is the operator's manual — read both before changing anything. This file records
**what was decided, what was found the hard way, and what is still open.**

---

## What this is

Whole-body CoM ground truth from OptiTrack, to validate the pelvis pose/velocity estimator in
`/home/llibshutz/alex/invariant-estimation` (a Python InEKF). Java 17 + Gradle, built on Euclid,
Mecano, EJML and SCS2.

Pipeline stages are named `F1`–`F11` and gates `G1`–`G4`, matching `FRAMEWORK.md` sections.

## Branch and PR state

| Branch | PR | Contents |
|---|---|---|
| `pr1` | [#1](https://github.com/ihmcrobotics/alex-mocap/pull/1) → `main` | `core`, `registration`, `mocap`, G1 + CLI |
| `pr2` | [#2](https://github.com/ihmcrobotics/alex-mocap/pull/2) → `pr1` | `model` (F1), `frames` (F8), `calibration` (F2–F5, A′), G2, G4 |
| `pr3` | [#3](https://github.com/ihmcrobotics/alex-mocap/pull/3) → `pr2` | `runtime` (F6–F10), `postprocess` (SG, F11), `scs2`, `ReplayRunner` |
| `pr4-alex-demo` | none | the real-Alex leg-marker demonstration |

**They are stacked.** Merge in order or GitHub retargets. PR4 has deliberately not been opened.

Reports live in `.claude-reports/` on `pr4-alex-demo`: the overnight markdown report, an HTML
visual summary, and the plan it was built from.

## Running it

```bash
# one file, from IntelliJ — generate → calibrate → replay → open SCS2
src/test/java/us/ihmc/alexMocap/AlexLegDemo.java        (main; test scope on purpose)

./gradlew clean build --rerun-tasks                     # 166 tests, no display needed
```

`AlexLegDemo --no-visualize | --degenerate | --out <dir>`. Test scope because it depends on
`RobotCaptures`, which invents mocap data — that must never reach the shipping jar.

---

## Departures from FRAMEWORK.md — decisions that are Lucas's to confirm

Each was found by building the thing and hitting the contradiction. Sections **§3 and §19 were
amended**; **§9/§18.1 were not** — the code departs and says so.

### §3 — link frame is `parentJoint.getFrameAfterJoint()`, not `getBodyFixedFrame()` *(amended)*

Mecano's body-fixed frame is the link's **centre-of-mass** frame (it appears in the tree as
`l_thighCoM`). Both conventions are self-consistent for the calibration — `^i p_ij` is solved for,
so a constant offset is absorbed and `J` is identical. §12 and §14 decide it: both presuppose
`^i c_i ≠ 0`, which is identically zero in the CoM frame, silently deleting the link-CoM term from
the error budget.

**Caveat found later on the real model:** `URDFTools` calls `transformAllFramesToZUp()` (default
on), which rewrites link-frame orientations wherever a joint has non-zero `<origin rpy>`. So the
javadoc claim that a calibrated `^i p̂_ij` is *"directly comparable to a CAD marker position"* is
**false for Alex above the legs**. True for the legs only because they all have `rpy="0 0 0"`.
**Open: reword or scope that claim.**

### §3 — `b` is the frame *after* the `SixDoFJoint` SCS2 injects *(amended)*

SCS2 does not instantiate the URDF root as the tree root. It creates a synthetic `rootBody` and
attaches the URDF root beneath a `SixDoFJoint` that appears in no URDF. Taking the synthetic root's
frame as `b` puts that floating joint inside every `^b T_i`, breaking §0's claim that `^b T_i(q)`
depends on joint angles alone — silently, since at identity it looks right.
`RobotModelHandleTest` moves the base to 20 random poses and requires no `^b T_i` to change.

### §19 — SCS2 rule narrowed to the *visualizer* *(amended)*

The old rule was enforced by **nothing**: `PackageDependencyTest` only scanned
`us/ihmc/alexMocap/` names, so it was blind to `import us.ihmc.scs2.*`. It now scans
`us/ihmc/scs2/`, `javafx/` and `us/ihmc/yoVariables/`. Only `model` may use `scs2-definition`
(headless), only the `scs2` package may use SCS2 beyond it or JavaFX/YoVariables.

### §9 / §18.1 — refuse on `σ₂`, **not** `σ₃` *(NOT amended — open decision)*

| cluster shape | `σ₂` | `σ₃` | pose determined? |
|---|---|---|---|
| generic | > 0 | > 0 | yes |
| **coplanar** (flat link face) | > 0 | ≈ 0 | **yes** |
| collinear | ≈ 0 | ≈ 0 | no |

Three non-collinear points fix a 6-DOF pose, so coplanar is fine — §2 itself calls a near-planar
cluster *"the realistic case, markers on a flat link face"*, which makes §2 and §18.1 inconsistent.
Refusing on `σ₃` rejects good data: the toy's `l_shank` cluster is near-coplanar with nominal
`σ₃ = 3.1e-08 m²` (0.17 mm out-of-plane, **below** the 0.3 mm noise) and was refused in **one frame
in six** of a healthy replay. `σ₃` is still logged per §9.

**Decision needed: amend §9/§18.1, or revert the code to σ₃.**

---

## Findings worth not rediscovering

### Layout accuracy is NOT `σ/√K` — the gauge cluster sets it

PR_PLAN's 0.2 mm target came from F4's averaging and is unreachable by any implementation. Every
capture's base pose comes from registering the pelvis markers, so it carries angular error
`σ/(√N·r_perp)` (FRAMEWORK §1's own scaling), multiplied by the lever arm to each link.

Measured, σ = 0.3 mm, RMS over all markers, 3 seeds:

| K | all noisy | gauge cluster clean |
|---|---|---|
| 30 | 0.383 mm | 0.0885 mm |
| 480 | 0.188 mm | 0.0226 mm |
| ratio | **2.04×** | **3.92× (=√16)** |

Clean the gauge and it is textbook `1/√K`. Noisy, it is 4.3× worse *and* the exponent breaks,
because the reference shape comes from **one capture** and that noise never averages.

**Levers, in order:** widen the pelvis bracket (error ∝ `1/r_perp`, verified 0.06 m → 3.00 mm,
0.20 m → 0.91 mm) ≫ lower σ ≫ more captures (16× bought 2×).

**On Alex specifically:** at §1's recommended 140 mm bracket and §17's 0.3 mm noise, held-out RMS is
**2.86 mm against the 2.2 mm TALOS bar**, on synthetic data with a perfect URDF.
`held-out ≈ 2.86 mm · (σ/0.3) · (140/spread)`. **Need ≥182 mm at σ=0.3, or σ≤0.23 at 140 mm.**

### Identifiability must be designed in — it is not detectable afterwards

`Δ → Δ·G` for a translation `g` shifts every layout by `R_i(q)ᵀg`. That is an **exact symmetry**
whenever `g` lies along an axis every marked link merely rotates about.

- **Toy:** hip and knee both pitched about y → `g ∥ y` free. A′ hit `J = 7.6e-29` (machine zero) and
  landed 13 mm from truth. Fixed by making the hip roll about x.
- **Alex:** hip is X-then-Z-then-Y, so pelvis + a *single* thigh already pins `Δ`. But pelvis +
  both `HIP_X_LINK` **is** degenerate (parallel `(1 0 0)` axes): in-sample RMS **0.112 mm**, exit 0,
  G2 passes, `isFullySolved()` true — and the layout is **57.3 mm wrong**, displaced along x.
- **`σ₃` cannot separate them**: 8.79e-4 vs 2.21e-3 for an identifiable set (factor 2.5), while two
  *both-identifiable* sets differ by 26.

### Other measured facts

- **CoM vs Mecano's `CenterOfMassCalculator`: 1.1e-15 m** on FK-consistent poses. Two code paths
  sharing nothing. This is the check that the loop closes.
- **A leg-only marker set leaves 58.45 % of Alex's 91.5 kg posed by FK, not markers.**
  `TORSO_LINK` alone is 22.21 kg (24 %) chained through one `SPINE_Z` joint — which is *also* a
  filter state. One torso cluster takes it to 34.18 %. Highest-leverage addition after the gauge.
- **F11 on real inertials:** mass 4.90 / link-CoM 2.73 / mocap 0.164 mm. CAD dominates 33×;
  perfect mocap would buy **1.00×**. Weigh the robot.
- **§17's 0.0037 m/s is a pelvis-*pose*-noise figure**, not marker noise: F6 registers 4 markers
  into one pose, so `σ/√N` reaches the differentiator. Fed raw 0.93 mm the same window gives
  0.0067 m/s and misses PR_PLAN's 0.005 target.
- **G4 shows no asymmetry** under a systematic bias. An i.i.d. split carries the bias equally in
  both halves; held-out detects *overfitting*, and 90 parameters against 1680 observations have
  none. Absolute level is the signal, not the ratio.
- **G2 correlates signed components, not magnitude.** Magnitude is V-shaped in the joint angle, so
  Pearson reads ~0 however strong the dependence.

### The failure mode this project has

**Every bug found here was a small residual with a wrong answer, never a loud one.** The gauge
freedom at `J = 7.6e-29`; a 2 m local minimum reached monotonically; a forgotten F8 correction
biasing every CoM 7 mm low. Be wary of any green number not paired with a conditioning number.

---

## SCS2 behaviours that cost real time

All verified against `scs2 17-0.30.0`.

1. **`simplifyKinematics` (default on)** merges all fixed joints. Alex: 141 links/140 joints in the
   file → **29 joints / 30 links**, mass preserved at 91.5126 kg.
2. **`transformAllFramesToZUp` (default on)** rewrites link-frame orientations. See §3 caveat above.
3. **`toGeometryDefinition` returns `null`** for geometry SCS2 has no class for — `<capsule>` is an
   SDF/IHMC extension and its set is box/cylinder/sphere/mesh. The null is *stored*, not thrown, and
   only explodes in `SimulationSession.addRobot` → `CollisionTools.toFrameShape3D`, which
   dereferences it inside its own "unhandled geometry type" warning. `RobotDefinition.newInstance`
   never touches geometry, which is why the whole test suite passes and only the visualizer breaks.
4. **An unresolvable mesh leaves `ModelFileGeometryDefinition.getFileName()` null**, dying one line
   earlier in the same method. Same symptom, different mechanism.
5. **`URDFTools.loadURDFModel(File, dirs)` passes a NULL ClassLoader**, and `SDFTools`' resolver
   consults nothing without one — so *every* `package://` mesh fails regardless of the directories
   supplied. Use the `InputStream` overload with a real loader. Verified directly against
   `tryToConvertToPath`: same URI and directory, resolves with a loader, fails without.
6. **`SessionVisualizer.startSessionVisualizer` does NOT block.** It returns once the toolkit is up.
   Returning from there lets `main` reach `System.exit` and kill the window. Use
   `controls.waitUntilVisualizerFullyUp()` … `controls.waitUntilVisualizerDown()`.
7. **Mecano's `CenterOfMassCalculator` cannot be fed measured poses** (§12 asks for this). It
   computes from the tree's own frames, and independently measured link poses are generally
   consistent with *no* joint configuration — which is the signal the method exists to expose. F9
   writes the sum out directly; Mecano is the **test oracle** instead.

## The Alex URDF

`/home/llibshutz/alex/invariant-estimation/assets/alex_with_imus.urdf`, vendored byte-identically to
`src/test/resources/us/ihmc/alexMocap/model/alex.urdf`. **It loads unmodified** — no sanitizer.

Root `PELVIS_LINK`. Legs, per side: `HIP_X`(x) → `*_HIP_X_LINK`, `HIP_Z`(z) → `*_HIP_Z_LINK`,
`HIP_Y`(y) → `*_THIGH`, `KNEE_Y`(y) → `*_SHIN`, `ANKLE_Y`(y) → `*_ANKLE_Y_LINK`,
`ANKLE_X`(x) → `*_FOOT`. Note `THIGH`/`SHIN`/`FOOT` have **no** `_LINK` suffix.

Masses: PELVIS 7.161; per side HIP_X 2.332, HIP_Z 0.761, THIGH 8.281, SHIN 6.338, ANKLE_Y 0.050,
FOOT 0.810; TORSO 22.21.

**The estimator's actual states are 9 joints:** `SPINE_Z` + both `HIP_X/Z/Y` + `KNEE_Y`. Ankles are
measured inputs, not estimated. "The legs" is a good approximation but `SPINE_Z` is in there.

---

## OPEN — in progress when this was written

### The visualizer renders no meshes (uncommitted work on `pr4-alex-demo`)

Fixed and verified so far: the two NPEs (items 3–4 above), mesh *resolution* (item 5 — dropped
visuals went 56 → 24, the remaining 24 being ability-hand meshes genuinely absent from disk), and
the blocking lifecycle (item 6 — headless run now blocks, exit 124, vs exiting 0 before).

**Still broken:** JavaFX itself fails to import every `.obj`:

```
JavaFXVisualTools: Could not import model file:
  file:/home/llibshutz/alex/alex-mocap/yoGraphicResources/alex_V1_description/meshes/legs/Pelvis.obj
  NullPointerException: Cannot read the array length because "array" is null
```

264 of these. Three things wrong with that path, and it is a **different resolution layer** from the
one already fixed:

- root is `<cwd>/yoGraphicResources/`, which does not exist;
- the `alex_virtual_description/` segment has been dropped;
- **case differs** — `alex_V1_description` vs the actual `alex_v1_description`.

Actual file: `…/assets/alex_virtual_description/alex_v1_description/meshes/legs/Pelvis.obj`.

Next step: find what maps SDFTools' resolved absolute path into `yoGraphicResources/…` — likely
JavaFX-side resource handling in the session visualizer — and whether the case change is ours or
theirs. Consequence today: the window opens and works, the CoM sphere and pelvis triad draw, but the
robot has no visible shapes.

### Also open

- **Nobody has confirmed a window renders on a machine with a display.** JavaFX initialises here
  (FXML loads, "Linking YoVariables") but this box is headless.
- `.idea/` is untracked and unignored on every branch.
- G3 (`VolumeDistortionGate`) is still an empty placeholder — it needs a physical two-marker artifact.

---

## Conventions

- **Fixed seeds, loose thresholds, the measured value stated in a comment.** Never write a threshold
  you have not seen a measurement for; never loosen one to make a test pass without understanding
  why it moved.
- **No mocking of core logic.** Generate data by FK from a real URDF; add seeded Gaussian noise.
- **Primitives report numbers and decide nothing** (§2, §9). Gates and estimators decide.
- **Unset is NaN, never zero.** Zero is a legal angle, position and tilt.
- **`σ` values are m², not m** — mean-squared spreads. A 120 mm cluster reads `σ ≈ 0.003`.
- Everything stays headless-testable. JavaFX lives only in `scs2`, enforced by a test.
- Commit messages carry the *why*, especially for anything that contradicts a spec section.
