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
| `mocap-sdk-model` | none | PR5: `sim`, SCS2 ground-truth track, SDK meshes, version alignment |

**They are stacked.** Merge in order or GitHub retargets. PR4 has deliberately not been opened.

Reports live in `.claude-reports/` on `pr4-alex-demo`: the overnight markdown report, an HTML
visual summary, and the plan it was built from.

### Where this repository lives

`~/workspaces/mocap/alex-mocap`, inside an IHMC **repository group** (`isProjectGroup=true`,
`compositeSearchHeight=0`) alongside `alex`, `ihmc-alex-sdk`, `ihmc_hands_ros2` and
`ihmc-open-robotics-software`. It was moved there from `~/alex/alex-mocap` in August 2026 so that
the SCS2 track can consume it through a composite build. Nothing should hard-code either path —
`AlexSdkModels` walks up to find its siblings, which is why.

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

### Four markers per cluster is not "three plus a spare" (measured, PR5)

At a **12 %** independent per-marker drop rate over the seven-cluster leg set, **126 of 200 frames
are refused — 63 %**. The naive arithmetic predicts 41 %: a four-marker cluster falls below three
with probability `1 − 0.88⁴ − 4·0.12·0.88³ = 7.3 %`, so `1 − 0.927⁷ ≈ 41 %`.

The gap is the `σ₂` guard. Losing one marker of four does not leave a comfortable three-point
cluster — it removes a quarter of the constellation's spread, and `LinkPoseEstimator`'s threshold is
`DEFAULT_SIGMA2_FRACTION = 0.25` of nominal `σ₂`. The ~6 % between the two numbers are clusters that
had **enough markers and still could not be trusted**.

Consequences: an occlusion budget computed from `MarkerCluster.MINIMUM_MARKERS` alone will be
optimistic; five markers on the limbs buys more than the count suggests. And this is the *friendly*
case — the simulated camera's occlusion is memoryless and per-marker, where a real dropout takes the
same markers for many consecutive frames.

**A refused marked link also orphans its unmarked descendants.** `KinematicChainCoupler` binds each
unmarked link to its nearest marked ancestor **once, at construction**, so refusing `LEFT_SHIN` also
refuses `LEFT_ANKLE_Y_LINK`: 6.39 kg leaves the sum, not 6.34. Losing one marked link costs that
link *and everything unmarked beneath it*, which is not readable off the marked set. The CoM goes
NaN rather than becoming the CoM of a robot missing a shin — marked links are never silently
substituted by FK, which is the conservative branch and the right one.

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

### The replay drew the robot at the origin (fixed, PR5)

`GroundTruthSessionVisualizer.setRobotConfiguration` set **joint angles only**, leaving the floating
joint at identity. The CoM sphere and pelvis triad are in measured world coordinates, so they
appeared where the robot really was — `(1.00, 1.99, 1.41)` on the demo's capture set, a suspended
gantry pose about **2.4 m from the origin** — while the robot itself was drawn at `(0, 0, 0)` with
its legs below the grid. It reads as "the robot is floating in the air in a weird pose".

Nothing threw, no number was wrong, and all 195 tests passed. Only the picture was wrong, which is
this project's failure mode exactly. `VisualizerPlacementTest` now pins it on the real SCS2 `Robot`.

Two details worth keeping: the floating joint is found **by type** (`FloatingJointBasics`), not by
name or by `getChildrenJoints().get(0)`, because the latter would silently pick a real URDF joint
and place the robot by bending its hip. And a refused pelvis (NaN pose) **holds the last good pose**
— assigning NaN puts it in the frame tree permanently, so one bad capture would blank the rest of
the replay; the dropout is still visible through the CoM sphere vanishing.

Not a bug, and asked about twice now: **the legs really do take strange postures.** The captures are
a calibration sweep, drawn uniformly across each joint's full URDF range per FRAMEWORK.md §1, and
that excursion is the signal A′ fits. `AlexLegDemo --excursion <f>` narrows it for legibility at a
measured cost (in-sample RMS 2.047 mm at 0.3 versus 1.910 mm at 1.0).

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

### It is Alex **V2**, and it matches the SDK exactly (measured, PR5)

The Python asset is not a separate model. Compared link-by-link against `ihmc-alex-sdk`:

| against | shared lower-body links | mass mismatches |
|---|---|---|
| `alex_v1_description/urdf/alex_v1.lowerBody.urdf` | 23 | **1** (`TORSO_LINK` 22.21 vs 11.478) |
| `alex_v2_description/urdf/alex_v2.lowerBody.urdf` | 23 | **0 — identical** |

Every actuated leg-joint origin and axis is identical too; the only textual differences are tabs
versus spaces. So the vendored URDF is the SDK's V2 lower body, and **every mass figure recorded in
this file is a V2 figure**.

That matters because `AlexFlatGroundWalkingTrack` builds `AlexVersion.getPhysicalRealityVersion()`,
which returns `AlexV2Version.*` for both Alex001 and Alex002. The simulation and the mocap chain are
therefore on the same model by construction — the SCS2 track needed no re-derivation of any number
here. Had it been V1, `TORSO_LINK` alone would have moved by 10.7 kg, and torso mass is the single
largest FK-posed term in the legs-only marker set.

**Two things the Python copy has that the SDK does not.** Calibrated IMU mounting: its
`*_HIP_X_IMU_JOINT` carry measured rpy (`-1.5803397 -0.0019308 1.570796` left) where the SDK has
nominal (`-1.570796 0 1.570796`). And ability hands — Alex001 is `CYCLOID_FOREARMS` and Alex002 is
`NUB_FOREARMS`, so the hand configuration in the vendored file matches neither robot. Neither
affects the CoM work: IMU links carry no mass and are not marked, and the hands are far above the
leg marker set. But the file is **not** redundant with the SDK, so do not delete it as a duplicate.

---

## OPEN

### The visualizer meshes — resolution fixed, rendering still unconfirmed

**What changed (PR5).** The mesh root is now named independently of the URDF:
`GroundTruthSessionVisualizer.show(urdf, resourceDirectories, …)`, `ReplayRunner --mesh-dir`
(repeatable), and `AlexSdkModels` to find `ihmc-alex-sdk` without an absolute path.

The real root cause was never the NPEs or the null ClassLoader — those were already fixed. It was
that `package://` resolved against **exactly one** directory, the URDF's own parent, which forced a
choice between a model whose bytes are pinned by sha256 and a robot that draws. The demo took the
second, pointing at a copy of the URDF in the Python estimator's checkout. When that copy moved, the
meshes went with it.

**Verified headlessly** (`MeshResolutionTest`): with `<sdk>/alex-models/` as the root, 29 of the
vendored URDF's 37 `package://` references resolve — every leg, pelvis and torso mesh. The other 8
are ability-hand hulls in `ihmc_hands_ros2/meshes/`. `getFileName()` now returns an **absolute path
to a file that exists**, e.g.
`…/ihmc-alex-sdk/alex-models/alex_virtual_description/alex_v1_description/meshes/legs/Pelvis.obj`.

**Why that probably — but not certainly — kills the JavaFX failure.** The old error was

```
file:/home/llibshutz/alex/alex-mocap/yoGraphicResources/alex_V1_description/meshes/legs/Pelvis.obj
```

which is a *relative* path resolved against `<cwd>/yoGraphicResources/`, with the
`alex_virtual_description/` segment dropped and the case changed. That shape is what you get when
`getFileName()` holds a relative string and something downstream prefixes a staging directory. With
an absolute path to an existing file there is nothing left to prefix. **This has not been observed
on a display, so treat it as diagnosed rather than closed.** (An earlier commit message here
overstated it as "closes".)

**Note the casing is still unexplained.** Nothing on disk is spelled `alex_V1_description` — not the
SDK, not the Python assets, not the URDF text. `AlexURDFParameters` in the `alex` repo has that
spelling in a *comment* only. If bare meshes ever come back, that is the thread to pull.

### Also open

- **Nobody has confirmed a window renders on a machine with a display.** JavaFX initialises here
  (FXML loads, "Linking YoVariables") but this box is headless. This is now the only thing standing
  between "meshes resolve" and "meshes draw".
- **The SCS2 track is written but not compiled.** `integration/AlexMocapGroundTruthTrack.java` plus
  `integration/README.md`; it needs three edits in the `alex` repo (an `includeBuild`, a dependency,
  a file copy) that were deliberately not applied. APIs were checked with `javap` against
  17-0.33.2, but nothing has been through a compiler.
- ~~`.idea/` is untracked and unignored on every branch.~~ Ignored as of PR5, along with `.claude/`.
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
