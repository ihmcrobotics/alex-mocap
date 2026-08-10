# Running `alex-mocap`

## Setup

Nothing to install beyond a JDK. The Gradle wrapper (8.11) is checked in, and the build
declares a Java 17 toolchain, so Gradle picks JDK 17 regardless of your default `java`.

- Requires a JDK 17 on the machine (`/usr/lib/jvm/java-17-openjdk-amd64` here).
- Your shell `java` may be 21; that is fine, the toolchain overrides it.

Verify with:

```bash
./gradlew javaToolchains
```

## Build

```bash
./gradlew build          # compile + jar + tests
./gradlew compileJava    # compile only
./gradlew dependencies --configuration compileClasspath   # inspect resolved deps
```

The configuration cache is deliberately **not** enabled in `gradle.properties` — it breaks
IDE import (see *IDE / LSP* below). `org.gradle.caching=true` (the build cache) is on
instead; it has no such interaction. Pass `--configuration-cache` per invocation if you
want it for a CLI build.

## IDE / LSP

The project imports as a plain Gradle project — open the repo root and let jdtls (or
Eclipse/Buildship, or IntelliJ) run the import. Nothing to configure by hand. `.project`,
`.classpath`, `.settings/`, and `bin/` are generated per machine and are gitignored; do
not commit them.

To sanity-check the model the IDE will receive, without an IDE:

```bash
printf "allprojects { apply plugin: 'eclipse' }\n" > /tmp/ec.gradle
./gradlew -I /tmp/ec.gradle eclipse      # must succeed
grep javanature .project                 # must be present
```

If the LSP shows no symbols, reset the jdtls workspace and reopen:

```bash
rm -rf ~/.cache/nvim/jdtls/alex-mocap/workspace
```

## Entry points

`us.ihmc.alexMocap.CalibrationRunner` runs a pre-flight gate over a captured mocap log.

```bash
./gradlew installDist
./build/install/alex-mocap/bin/alex-mocap --gate g1 --input capture.csv --sigma 0.0003

# or without installing:
./gradlew run --args="--gate g1 --input capture.csv --sigma 0.0003"
```

`--sigma` is the **measured** per-axis mocap noise in metres, and there is deliberately no
default — see the caveats below. `--help` prints the full option list.

Exit codes: `0` every gate passed, `1` a gate failed **or could not be fully evaluated**,
`2` usage or I/O error.

Sample output on a capture where a pelvis marker's mount slipped 2 mm partway through:

```
input    capture.csv
markers  8
clusters PELVIS(4), L_THIGH(4)
sigma    0.3000 mm per axis (measured)

G1 -- rigidity: inter-marker distances within a cluster must be constant to within mocap noise
  12 pairs over 600 frames, sigma 0.3000 mm, threshold 0.9000 mm (3.0 sigma; noise floor 0.4243 mm = sqrt(2) sigma)

  STATUS         SUBJECT                                      MEASURED    THRESHOLD  SAMPLES
  FAIL           PELVIS: PELVIS_1-PELVIS_2                   0.9924 mm    0.9000 mm      600
                 std 0.9924 mm over a 120.8 mm baseline (floor 0.4243 mm)
  FAIL           PELVIS: PELVIS_1-PELVIS_3                   1.0174 mm    0.9000 mm      600
                 std 1.0174 mm over a 121.0 mm baseline (floor 0.4243 mm)

  10 passed, 2 failed, 0 not evaluated

  G1: FAIL
```

Passing checks are summarised by count rather than listed: forty green rows is how a red one
gets missed.

### `--calibrate` — run A′ (PR2)

Needs three inputs: a mocap CSV, an encoder CSV, and the URDF.

```bash
./build/install/alex-mocap/bin/alex-mocap \
    --calibrate \
    --input    capture.csv \
    --encoders encoders.csv \
    --urdf     robot.urdf \
    --sigma    0.0003 \
    --world-tilt 0.08 \
    --output   calibration.json
```

`--gauge <link>` defaults to the URDF root link, which is what `Δ = ^c T_b` is defined
against. `--world-tilt` is in **degrees** and is recorded in provenance; omit it and the
result records NaN and the run warns, which is the honest encoding of "nobody measured it".

Sample output (30 synthetic captures at σ = 0.3 mm, toy 6-DOF URDF):

```
urdf     toy6dof.urdf  (sha256 2db166ba5bd031b6...)
captures 30
clusters pelvis(4), l_thigh(4), l_shank(4), l_foot(4), r_thigh(4), r_shank(4), r_foot(4), gauge=pelvis
skew     worst |mocap - encoder| = 0.000 ms over 30 captures

A' calibration report
  captures            30 usable of 30 (reference capture 0)
  observations        840
  iterations          71 (converged)
  J after bootstrap   3.114601e+00 m^2
  J final             2.950952e-03 m^2
  in-sample RMS       1.8743 mm  (NOT an accuracy claim; see G4)
  monotone            yes
  gauge worst sigma3  1.230385e-04 m^2
  base step sigma3    9.891633e-03 m^2
  per-marker in-sample residuals
    link         marker           K_ij   rms (mm)   max (mm)
    pelvis       pelvis_M0          30     0.4107     0.6905
    ...
    l_shank      l_shank_M0         30     1.6796     3.4646
    l_foot       l_foot_M0          30     2.7932     6.7967

G2 -- Bootstrap spread: ...
  No marker's per-capture spread exceeds 3.0σ; nothing indicts the model.
  all 28 checks passed
  G2: PASS
```

**Read the per-marker residual column, not the summary RMS.** The gradient down it —
0.41 mm at the pelvis, 2.79 mm at the foot, on data whose marker noise is 0.3 mm everywhere
— is not a bug and not a bad fit. It is the gauge cluster's angular error multiplied by the
lever arm out to each link, and it is the dominant error term in this whole pipeline. See
*Gotchas* below.

The encoder CSV mirrors the mocap one — header row is the schema, joint names travel with
the data so a permuted column order fails at the URDF boundary instead of silently
producing a plausible calibration at the wrong configuration:

```
# alex-mocap encoder log, format 1
timestamp_ns,l_hip,l_knee,l_ankle,r_hip,r_knee,r_ankle
1000000000,0.1042,0.8813,-0.2210,0.0455,1.4400,0.1188
```

Rows are paired with mocap rows **by index** — that is the only thing two independently
written logs agree on — and the worst timestamp skew is printed. FRAMEWORK §18.3: a
mispairing is valid mocap plus valid encoders at the wrong configuration, and nothing else
about it looks wrong.

### `ReplayRunner` — the runtime pass (PR3)

Consumes a logged capture plus a `CalibrationResult`, runs F6 → F7 → F9 → F10, and writes
`com.csv`, `pelvis.csv` and `conditioning.csv`.

```bash
java -cp "build/install/alex-mocap/lib/*" us.ihmc.alexMocap.ReplayRunner \
    --input       capture.csv \
    --encoders    encoders.csv \
    --urdf        robot.urdf \
    --calibration calibration.json \
    --output-directory out/ \
    --world-tilt  0.08 \
    --velocity --error-budget
```

Sample output (the same 30 synthetic captures, after `--calibrate`):

```
links        7 total, 7 marked, 0 chained
chained mass 0.000 of 28.000 kg on encoders (FRAMEWORK.md section 10)
world tilt   TiltMeasurement[PRECISION_LEVEL, θ=0.0800°, --world-tilt]

Conditioning over 30 frames (30 with every link accepted, 100.0%)
  link          accept%    below 3     worst s3      mean s3   visible-count histogram
  pelvis         100.0%          0    1.264e-04    1.299e-04   0:0 1:0 2:0 3:0 4:30
  l_thigh        100.0%          0    4.452e-06    5.301e-06   0:0 1:0 2:0 3:0 4:30
  l_shank        100.0%          0    9.863e-10    2.307e-08   0:0 1:0 2:0 3:0 4:30
  ...

velocity second pass (FRAMEWORK.md section 13)
  window          SGDifferentiator[21 samples (0.105 s), degree 2, noise gain 7.21 /s]
  edge samples    10 at each end are NaN; a centred window has no value there
  expected noise  0.00216 m/s at sigma = 0.3000 mm
  ContactNet bar  0.0844 / 0.0254 m/s

CoM error budget (FRAMEWORK.md section 14)
  mass error x lever arm        4.098 mm   (CAD)
  link-CoM error                4.033 mm   (CAD)
  pose error (mocap)            0.242 mm   <- the only one this pipeline controls
  dominant term              mass
  perfect mocap would buy    1.00x
  NOTE: the CoM is as good as the URDF, not as good as the mocap.
```

Exit codes: `0` every frame produced a CoM, `1` **at least one frame was refused**, `2`
usage or I/O error. A refused link means there is no CoM for that frame — not a slightly
worse one — so it is a failure rather than a footnote.

Two things in that output are worth reading rather than skimming. `l_shank`'s worst `σ₃` is
`9.9e-10` against `1e-4` for the others: that cluster is near-**coplanar**, which is fine and
is why the refusal guard is `σ₂` (see *Gotchas*). And the error budget prints §14's
conclusion outright — the CAD terms are 17× the mocap term, so perfect mocap would buy 1.00×.

## The SCS2 mocap ground-truth track (PR5)

Alex **walking**, with a simulated marker set on the legs, and the mocap-derived CoM drawn
against the simulation's real one. This is the live demo; `AlexLegDemo` below is the offline
replay it grew out of.

It lives in the **`alex` repository**, next to `AlexFlatGroundWalkingTrack`, because it needs
`AlexRobotModel` and the walking controller. The library half is finished here and tested; the
track itself is in **`integration/`** and is **not yet applied** — see `integration/README.md`
for the three edits (one `includeBuild`, one dependency, one file copy).

Once wired:

```bash
export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
cd ~/workspaces/mocap/alex && ../gradlew compileJava
# then green-arrow us.ihmc.alex.simulation.AlexMocapGroundTruthTrack from IntelliJ
```

`-Dmocap.occlusion=0.12` turns on occlusion; leave it off unless you want to see refusals.

**On screen:** 28 marker spheres (four per link, coloured per cluster, yellow = pelvis gauge),
a **gold** sphere at the mocap CoM and a **green** one at the simulation's. They overlap when
things are right. Plot `mocapMocapMinusActualComMagnitude`, and
`mocapMocapMinusActualComMean` beside it — a floor in the mean is a *bias*, not noise.

**Two caveats, both encoded in the code rather than left to the reader.** The runtime is given
the *planted* layout, so this is F6–F9 with calibration error set to zero (a calibrated layout
adds ~2.86 mm on Alex at a 140 mm gauge bracket). And the mocap chain shares the simulation's
URDF, so link masses and link-CoM offsets agree by construction — which F11 measured as the
*dominant* real-world terms (mass 4.90 mm / link-CoM 2.73 mm / mocap 0.164 mm). Weigh the robot.

### Using it as a library

The pieces are usable without the walking track:

| class | what it does |
|---|---|
| `sim.MarkerConstellation` | draws a randomised marker layout on chosen links |
| `sim.SimulatedMocapCamera` | projects markers from live link poses; noise + occlusion |
| `scs2.SimulatedMocapGroundTruth` | the whole chain, one `update()` per tick |
| `scs2.MocapMarkerYoVariables` | the marker cloud as YoVariables + spheres |
| `scs2.GroundTruthComparisonYoVariables` | mocap vs actual CoM, with running mean/sd |

`SimulatedMocapGroundTruth` **owns and mutates** the `RobotModelHandle` it is given — it poses
it from the encoders every tick. Give it its own instance, never the one your simulation poses.

---

## Demonstration: the real Alex model

### Run it — one command, or one green arrow

```bash
export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
./gradlew alexLegDemo                                  # opens the SCS2 window
./gradlew alexLegDemo --args="--no-visualize"          # headless / over SSH
./gradlew alexLegDemo --args="--degenerate"
./gradlew alexLegDemo --args="--sweep 0.6"             # wider leg swing about the rest pose
./gradlew alexLegDemo --args="--full-range"            # every joint across its whole URDF range
./gradlew alexLegDemo --args="--strict"                # stop on a gate failure instead of replaying
```

**A failed gate no longer stops the run.** With markers scattered over each link, G2 currently
fails on magnitude while reporting that it found no structure — see CLAUDE.md, *"Scattered
markers work; G2's expected-spread model does not cope"*. Stopping there would deny you the
result exactly when you most want it, so the demo prints a loud banner and replays anyway. The
**exit code is still non-zero**, so nothing automated mistakes it for a pass; `--strict` restores
the old behaviour.

Two numbers get printed that only a synthetic session can produce:

- **layout recovery** against the planted `^i p_ij` — currently 1.078 mm RMS, 1.676 mm worst.
  This is the calibration on its own, and it is *not* the report's in-sample RMS (5.2 mm), which
  also carries the base-pose fit residual and says so.
- **pelvis drift** — reconstructed vs planted base pose, which is what the ghost draws: currently
  **1.727 mm RMS, 3.926 mm max**. Tilt-corrected, so the 0.08° floor tilt (worth ~2 mm at gantry
  height) is not being reported as error.

Or open `src/test/java/us/ihmc/alexMocap/AlexLegDemo.java` in IntelliJ and hit the green arrow
on `main`. (The Gradle task exists because the demo is in **test** scope — `run` uses the main
source set, so from a terminal there was previously no way to launch it at all.)

That is the whole thing: it invents a leg-only marker session on the real Alex model, writes
it out as CSV, calibrates it, replays it, and **opens SCS2**.

Last verified end-to-end: 60/60 frames with every link accepted, error budget printing
mass 4.904 mm / link-CoM 2.727 mm / mocap 0.164 mm, and meshes resolving from
`~/workspaces/mocap/ihmc-alex-sdk/alex-models`.

```
AlexLegDemo                    generate -> calibrate -> replay -> open the SCS2 window
AlexLegDemo --no-visualize     the same without the window (headless / over SSH)
AlexLegDemo --degenerate       the hip-X-only set: a perfect-looking, 57 mm-wrong fit
AlexLegDemo --out /some/dir    write somewhere other than build/alex-demo
```

Everything lands in `build/alex-demo/`: `calibration.json`, `com.csv`, `pelvis.csv`,
`pelvisVelocity.csv`, `conditioning.csv`.

**In the SCS2 window** you get Alex at each logged configuration, a gold sphere at the
whole-body CoM, the pelvis coordinate system, and every conditioning variable available to
plot.

You get three things overlaid:

| what | how it is drawn |
|---|---|
| the robot at **truth** | solid, at the planted base pose |
| the **mocap reconstruction** | translucent cyan **ghost** |
| the 28 **markers** | 20 mm spheres, coloured per cluster (green = pelvis gauge) |
| the CoM | small gold sphere, inside the pelvis |

The markers stand off the **outside** of each segment — 0.12 m from a limb's centre of mass,
0.18 m for the pelvis outrigger — because that is where markers go and because a cluster at the
link centre of mass is drawn inside the mesh and cannot be seen at all. They are drawn at 20 mm
radius rather than a real marker's 6 mm so the set reads at a glance; do not measure distances
off the picture.

On a healthy leg marker set the ghost sits on top of the solid robot — that *is* the result.
Run `--degenerate` and it walks ~56 mm off along x: the same number the report prints, except
you can see it.

The robot hangs at `(0, 0, 1.4)`, directly above the origin triad, with its legs swinging
±0.45 rad (26°) about the straight-legged rest pose and its feet kept at least 0.10 m apart so
they do not cross. Successive captures are independent draws, so the replay *steps* between
poses rather than sweeping through them.

Note the **height** is deliberate. `RobotCaptures` itself still defaults to `(1.0, 2.0, 1.4)`,
off-origin on purpose: code that quietly assumes the base is at identity gives exactly the right
answer when it is, which is how the visualizer came to draw the robot at the origin for a whole
PR unnoticed. Keeping `z = 1.4` in the demo means that fault would still be obvious here.

**Why not the full URDF range.** FRAMEWORK.md §1 asks for as much joint excursion as the robot
has, and that excursion is what makes `Δ` identifiable — but taken literally on Alex it is
absurd. `HIP_Y` is `[-150°, +45°]` and `KNEE_Y` is `[0°, 140°]`, so independent uniform draws
fold one thigh against the chest while the other kicks forward. Measured, feet relative to the
pelvis:

| sweep | feet below pelvis | stance width |
|---|---|---|
| rest pose (all zeros) | 0.890 m | 0.240 m |
| **default, rest ±0.45 rad** | **≥ 0.733 m** | ≤ 0.932 m |
| range midpoint | 0.683 m | 0.774 m |
| full URDF range | **0.18 – 0.67 m** | up to 0.73 m |

Note the range *midpoint* is `HIP_Y = -52.5°, KNEE_Y = +70°` — a deep tuck — so simply narrowing
the sweep about the midpoint converges on a squat, not on a hanging robot. That is why the knob
is `--sweep` (a half-range about **rest**) and not a fraction of the range.

It costs about 4 %: in-sample RMS **1.987 mm** against **1.910 mm** for the full range, still
monotone-convergent with G2 passing. `--full-range` restores the old behaviour.
`CapturePostureTest` pins the geometry both ways. It **replays** rather than simulates — the session is built with
`newDoNothingPhysicsEngineFactory()`, so nothing integrates the robot forward and overwrites
the poses being inspected. Press play; the timeline is the capture index.

To plot the numbers that matter, open the YoVariable search and add `gtCom*`,
`gt*Sigma3SquaredMetres` and `gt*VisibleMarkers`.

Two notes. The demo lives in **test** scope because it depends on `RobotCaptures`, which
invents mocap data — that has no business on the shipping classpath. And nothing here touches
a camera: marker positions come from forward kinematics plus Gaussian noise, so this shows
that the code recovers an answer it was given, not that Alex's real geometry is good enough.

### What it demonstrates

Everything above is measured on the toy 6-DOF URDF, and every accuracy claim there carries
the caveat *"this tests the solver, not the robot"*. `AlexLegDemoTest` and
`AlexLegDemoCliTest` (PR4) point the same shipping classes at the URDF the Python InEKF uses
— `assets/alex_with_imus.urdf`, vendored byte-identically to
`src/test/resources/us/ihmc/alexMocap/model/alex.urdf` — with markers on the legs.

**It loads unmodified.** 141 links and 140 joints in the file; SCS2 merges the 111 fixed
joints and leaves **29 joints / 30 links**, root `PELVIS_LINK`, **91.512588 kg**, mass
preserved exactly. None of the feared hazards blocks it: the 11 `<capsule>` collisions, 17
`<gazebo>` blocks and 8 `<mimic>` elements on `type="fixed"` joints are all off the path from
URDF to Mecano tree, and although 70 links carry no `<inertial>` and 19 declare mass 0 with
zero inertia, all of them sit below fixed joints and are merged away. Every one of the 30
surviving links has a finite, strictly positive mass and a finite `^i c_i`, which
`testTheRealModelLoads` asserts as a tripwire.

You will see roughly a hundred `SDFTools: Unable to resolve the path: package://abilityHand/…`
lines on stderr. Those are the 17 missing hand meshes. They are cosmetic — visuals never
reach the Mecano tree — and there is no flag to silence them.

### Running it

```bash
./gradlew test --tests '*AlexLegDemo*'                          # quiet
./gradlew test --tests '*AlexLegDemo*' -Dalex.demo.verbose=true # with the tables
```

The verbose flag is forwarded to the test JVM explicitly in `build.gradle.kts`. Gradle's
`Test` task does not inherit the launching JVM's system properties, so without that
forwarding the flag is accepted and silently does nothing.

Both CLIs run on the real model with no new flags and no `--cluster` override — naming
markers `<LINK>_M<j>` makes the "prefix before the last underscore" convention produce exact
URDF link names. Real output, 40 captures at σ = 0.3 mm, 7 clusters:

```
$ ./build/install/alex-mocap/bin/alex-mocap --calibrate \
      --input capture.csv --encoders encoders.csv --urdf alex.urdf \
      --sigma 0.0003 --world-tilt 0.08 --output calibration.json

urdf     alex.urdf  (sha256 e453edccdfbc6a86...)
captures 40
clusters PELVIS_LINK(4), LEFT_THIGH(4), RIGHT_THIGH(4), LEFT_SHIN(4), RIGHT_SHIN(4), LEFT_FOOT(4), RIGHT_FOOT(4), gauge=PELVIS_LINK
skew     worst |mocap - encoder| = 0.000 ms over 40 captures

A' calibration report
  captures            40 usable of 40 (reference capture 0)
  observations        1120
  iterations          45 (converged)
  J after bootstrap   3.773248e+00 m^2
  J final             4.342767e-03 m^2
  in-sample RMS       1.9691 mm  (NOT an accuracy claim; see G4)
  monotone            yes
  gauge worst sigma3  4.843234e-05 m^2
  base step sigma3    4.805252e-02 m^2
  per-marker in-sample residuals
    link         marker           K_ij   rms (mm)   max (mm)
    PELVIS_LINK  PELVIS_LINK_M0     40     0.3533     0.6571
    ...
    LEFT_THIGH   LEFT_THIGH_M0      40     1.3140     3.3231
    ...
    LEFT_SHIN    LEFT_SHIN_M0       40     2.2147     4.5432
    ...
    LEFT_FOOT    LEFT_FOOT_M0       40     2.8151     6.1760
    RIGHT_FOOT   RIGHT_FOOT_M3      40     2.8584     6.9973

  G2: PASS
```

```
$ java -cp "build/install/alex-mocap/lib/*" us.ihmc.alexMocap.ReplayRunner \
      --input capture.csv --encoders encoders.csv --urdf alex.urdf \
      --calibration calibration.json --output-directory out/ \
      --world-tilt 0.08 --velocity --error-budget

links        30 total, 7 marked, 23 chained
chained mass 53.493 of 91.513 kg on encoders (FRAMEWORK.md section 10)
world tilt   TiltMeasurement[PRECISION_LEVEL, θ=0.0800°, --world-tilt]

Conditioning over 40 frames (40 with every link accepted, 100.0%)
  link          accept%    below 3     worst s3      mean s3   visible-count histogram
  PELVIS_LINK    100.0%          0    4.890e-05    5.048e-05   0:0 1:0 2:0 3:0 4:40
  LEFT_THIGH     100.0%          0    5.439e-06    6.102e-06   0:0 1:0 2:0 3:0 4:40
  RIGHT_THIGH    100.0%          0    2.833e-05    3.009e-05   0:0 1:0 2:0 3:0 4:40
  LEFT_SHIN      100.0%          0    2.883e-05    3.075e-05   0:0 1:0 2:0 3:0 4:40
  RIGHT_SHIN     100.0%          0    3.300e-06    4.025e-06   0:0 1:0 2:0 3:0 4:40
  LEFT_FOOT      100.0%          0    9.157e-06    1.025e-05   0:0 1:0 2:0 3:0 4:40
  RIGHT_FOOT     100.0%          0    1.826e-05    1.975e-05   0:0 1:0 2:0 3:0 4:40

CoM error budget (FRAMEWORK.md section 14)
  total mass                 91.513 kg
  assumed uncertainties      mass 5.0%, link CoM 5.0 mm/axis, pose 0.300 mm/axis
  mass error x lever arm        4.690 mm   (CAD)
  link-CoM error                2.727 mm   (CAD)
  pose error (mocap)            0.164 mm   <- the only one this pipeline controls
  total (quadrature)            5.428 mm
  dominant term              mass
  perfect mocap would buy    1.00x
```

**The two lines that actually matter are `chained mass 53.493 of 91.513 kg` and the
per-marker residual column climbing 0.35 mm → 2.86 mm.** Everything else in that output is
the toy's story repeated with bigger numbers.

The first says that with the pelvis and six leg links marked, **58.45% of Alex is on
encoders, not on markers.** `TORSO_LINK` alone is 22.21 kg — 24.3% of the robot — chained off
the pelvis through one `SPINE_Z` joint, the joint FRAMEWORK §1 refuses to make the gauge
because "under suspension the spine joint carries the full load in tension with off-axis
deflection the URDF does not model". Everything above the torso — neck, head and both arms,
**24.996 kg** — chains through that same joint, and `KinematicChainCoupler` attributes all of
it to `PELVIS_LINK`, because that is the nearest *marked* ancestor. The remaining 6.287 kg is
the four hip stubs and two ankle stubs. Exactly:
`22.210 + 24.996 + 6.287 = 53.493 kg`.

**Adding one torso cluster takes the chained fraction from 58.45% to 34.18%.** Note what that
does and does not buy: it removes `TORSO_LINK`'s own 22.21 kg from the chained set, and the
24.996 kg above it stays chained — but it now hangs off a *measured* torso instead of off the
pelvis through the one unmodelled joint FRAMEWORK §1 says not to trust. The error path
shortens by the joint that matters. It is the highest-leverage marker decision available
after the gauge itself.

The second is the gauge cluster's angular error times the lever arm out to each link, and on
Alex it is worse than on the toy for a purely geometric reason: pelvis origin to foot is
0.89 m here against roughly 0.6 m there. See the next section.

### Watch out for — the real Alex model

1. **A 140 mm gauge bracket is not enough on Alex.** At FRAMEWORK §1's recommended 120–150 mm
   and §17's 0.3 mm target noise, held-out marker RMS is **2.86 mm — above the TALOS 2.2 mm
   bar §15 names as the target — on synthetic data with a perfect URDF and nothing wrong but
   mocap noise.** Measured over a 5× range of bracket widths and three noise levels
   (3-seed means, K = 40, held-out RMS in mm):

   | gauge spread | σ = 0.93 mm | σ = 0.30 mm | σ = 0.10 mm |
   |---|---|---|---|
   | 60 mm | 20.44 | 6.56 | 2.18 |
   | 140 mm | 8.87 | **2.86** | 0.95 |
   | 200 mm | 6.34 | 2.04 | 0.68 |
   | 300 mm | 4.44 | 1.43 | 0.48 |

   That is §1's `σ/(√N·r_perp)` to within a couple of percent in both variables — linear in σ
   (0.93/0.30 = 3.10 against a measured 8.87/2.86 = 3.10) and inverse in the spread
   (140/60 = 2.33 against 6.56/2.86 = 2.29). Collected:

   ```
   held-out RMS  ≈  2.86 mm · (σ / 0.3 mm) · (140 mm / gauge spread)
   ```

   **To clear 2.2 mm on Alex you need a bracket of at least ~182 mm at σ = 0.3 mm, or
   σ ≤ 0.23 mm at 140 mm.** §1's recommendation has no lever arm in it. Pinned by
   `testHeldOutResidualMissesTheTalosBarAtTheRecommendedBracketWidth`.

2. **SCS2 rewrites the URDF's link frames, and it is on by default.**
   `URDFTools.toRobotDefinition` calls `RobotDefinition.transformAllFramesToZUp()` —
   **default `true`**, alongside `simplifyKinematics` — which walks the tree zeroing the
   rotation of every joint's `transformToParent` and compensating by rotating, in place: the
   joint axis, the inertia pose, the moment of inertia, and every child joint's transform.
   Kinematics and physics are exactly preserved; **the identity of the link frame is not.**
   For any link below a joint with a non-zero `<origin rpy>` — on Alex, everything from the
   shoulders outward — SCS2's frame is the URDF's frame rotated by the accumulated joint
   rotation.

   The consequence: `RobotModelHandle`'s javadoc promises that "a calibrated `^i p̂_ij`
   printed by this pipeline is directly comparable to a CAD marker position". **That does not
   hold for Alex's arms.** `LEFT_SHOULDER_Y_LINK`'s `^i c_i` comes back as
   `(-0.00264, +0.09735, +0.07277)` where the URDF's `<inertial><origin>` says
   `(-0.00264, 0.12135, -0.006824)`; the difference is exactly `R_x(0.698132)`, its parent
   joint's declared `rpy`. It *does* hold for the legs, where every joint declares
   `rpy="0 0 0"` — which is the set this demonstration marks. Nothing is numerically wrong:
   `^i p_ij` is solved for, `^i c_i` and `^b T_i` are consistent with each other, and F9/F11
   are unaffected. It is a *reading* hazard, and it is invisible on the toy URDF, which
   declares `rpy="0 0 0"` on all six joints.

   `setTransformToZUp(false)` on `URDFParserProperties` would turn it off. That is a
   specification change to §3, not an implementation one, so it is not done here.

3. **A green G2 on Alex is not "no joint offset".** At σ = 0.3 mm, a 0.5° `LEFT_HIP_Y` offset
   does **not** fire G2 — the worst affected marker (`LEFT_FOOT`) spreads 3.59 mm against
   2.08 mm expected, a ratio of 1.7 against a 3σ threshold. Alex's gauge-driven floor at the
   target noise is simply larger than the fault. The *indictment column is still correct*
   (`LEFT_THIGH` names `LEFT_HIP_Y` at r = 0.60); only the verdict is green. At σ = 0.05 mm it
   fires hard. G2's sensitivity to a joint offset scales with the gauge cluster's angular
   accuracy, not with the number of captures — the same lever as item 1.

4. **G2 fires on both branches on a one-branch fault.** On the toy the unaffected branch
   showed nothing. On Alex it shows about half: at σ = 0.05 mm with a 0.5° `LEFT_HIP_Y`
   offset, `LEFT_SHIN` 1.47 mm / `RIGHT_SHIN` 0.76 mm and `LEFT_FOOT` 2.29 mm /
   `RIGHT_FOOT` 1.08 mm. G2 is handed the **solved** `Δ`, and A′ absorbs part of a
   one-branch fault into `Δ`, which is global. The localisation claim that survives on a real
   robot is the **mirror comparison**, not "the other branch is clean".

5. **The hip-X-only marked set is exactly degenerate, and nothing catches it.** Marking only
   `PELVIS_LINK` and both `*_HIP_X_LINK`: `PELVIS_LINK` does not rotate relative to the base
   and `*_HIP_X_LINK`'s orientation is `R_x(q)` about an axis that is **the same `(1 0 0)` on
   both sides**, so a translation along `x` is an exact symmetry of `J`. At a 200-iteration
   cap the fit reports `J = 3.02e-6 m²` and an in-sample RMS of **0.112 mm on data with no
   noise in it**, writes a fully-solved `CalibrationResult`, exits 0, and passes G2 — and the
   layout is **57.3 mm wrong**, every marker displaced by the same 55.9 mm vector along `x`.
   `σ₃` does not detect it: 8.79e-4 m² here against 2.21e-3 m² for the identifiable
   pelvis+thighs set (a factor of 2.5), where the factor separating two sets that are *both*
   identifiable is 26. **There is no threshold on `σ₃` that separates "degenerate" from
   "merely awkward".** This is PR2's toy finding reproduced at Alex scale;
   `AlexLegDemoCliTest` pins the operational half of it.

6. **Alex's joint ranges are narrower than the toy's, and the base step feels it.** Hip-X is
   70° against the toy's 183°; hip-Z 80°, ankle-X 50°. Rank survives that, conditioning does
   not. At a 5% excursion fraction the base step's `σ₃` falls only 7.6× (4.86e-2 → 6.42e-3)
   while the layout error goes from 0.31 mm to **34.96 mm** and A′ hits its iteration cap.
   **Read `isConverged()`, not `σ₃`** — the conditioning number moves by less than an order of
   magnitude for a hundredfold loss of accuracy.

7. **Both knees declare `lower="0"`**, so a rest angle of zero sits exactly on the limit.
   `RobotCaptures` reports that rather than clamping silently, because a generator that
   silently clamps gives the same answer as one that silently does not, and that is the
   situation in which a genuinely out-of-range rest angle goes unnoticed.

8. **Report base pose position and rotation separately.** At σ = 0.3 mm, K = 30 the base
   position error is 0.84 mm and the base *rotation* error is 7.63 mrad (0.437°) — and the
   rotation is the one that matters, because 0.44° at the pelvis is 6.8 mm at a foot 0.89 m
   away. It is also, unchanged, the frame-to-frame noise on the runtime pelvis *orientation*
   F10 hands to the EKF comparison: F6 is single-frame with no averaging (§9). The toy's
   helper took the max of the two, which hid this.

### The visualizer

```bash
java -cp "build/install/alex-mocap/lib/*" us.ihmc.alexMocap.ReplayRunner ... --visualize
```

Opens SCS2 on the computed trajectory: the robot at each logged configuration, a gold CoM
sphere, the pelvis coordinate system, and every conditioning variable available to plot.
Needs a display. It replays rather than simulates — the session is built with
`newDoNothingPhysicsEngineFactory()`, so nothing integrates the robot forward and overwrites
the poses being inspected.

**It has no test, by design** (PR_PLAN: "if a JavaFX window does not appear you will know
within seconds"). It is also the only class in the project that touches JavaFX, and
`PackageDependencyTest` enforces that so the rest stays runnable over SSH.

#### Meshes: `--mesh-dir`

`package://` resolves against a directory whose *name* is the authority in the URI, and it does
not have to sit beside the URDF. On Alex it does not — the URDF here is a vendored copy (pinned
by sha256 in the provenance, so CI needs nothing external) and the meshes are in
`ihmc-alex-sdk/alex-models/`. So the roots are named separately:

```bash
... --visualize --mesh-dir ~/workspaces/mocap/ihmc-alex-sdk/alex-models \
                --mesh-dir ~/workspaces/mocap/ihmc_hands_ros2/meshes
```

`AlexLegDemo` finds both automatically via `AlexSdkModels`, which walks up from the working
directory looking for a sibling `ihmc-alex-sdk` (override with `-Dalex.sdk.dir` or
`$ALEX_SDK_DIR`). Omitting `--mesh-dir` falls back to the URDF's own directory — the old
behaviour, still right for a model that ships its meshes with it.

Measured on the vendored Alex URDF: **29 of its 37 `package://` references resolve under
`alex-models/`** — every leg, pelvis and torso mesh. The remaining 8 are ability-hand hulls,
which live in `ihmc_hands_ros2/meshes/`; without that second root the hands simply do not draw.

An unresolvable mesh **does not throw**. It leaves `ModelFileGeometryDefinition.getFileName()`
null, survives `RobotDefinition.newInstance()` (which never looks at geometry), and only becomes
an NPE inside `SimulationSession.addRobot`. So the whole headless suite passes with every mesh
broken — which is exactly what happened before. `MeshResolutionTest` counts resolved references
instead, and skips (rather than fails) when the SDK is not beside this checkout.

The G3 gate (`VolumeDistortionGate`) is still an empty placeholder — it needs a rigid
two-marker artifact carried through the volume, which is a hardware procedure rather than a
software one.

## What is implemented

### `us.ihmc.alexMocap.core` — the data model

The types every other package speaks in. Depends on Euclid and nothing else, not even the
other packages here.

| Type | Holds | Mutable? |
|---|---|---|
| `MarkerId` | marker name + dense index | immutable |
| `MarkerObservation` | one marker's world position + visibility | reusable |
| `MocapFrame` | timestamp + one observation per marker | reusable |
| `MarkerCluster` | link name + member markers | immutable |
| `EncoderSample` | timestamp + `q` + joint order | reusable |
| `Capture` | a paired `MocapFrame` + `EncoderSample` | reusable |
| `ClusterLayout` | `^i p̂_ij` per marker + `K_ij` | mutable (A′ overwrites) |
| `CalibrationResult` | layouts + `Δ` + provenance | mutable |
| `CalibrationResultIO` | JSON read/write | static |
| `GroundTruthSample` | CoM + pelvis pose + per-link `σ₃`/visible/refusal | reusable |

The split is deliberate: configuration types are immutable, and anything the 200 Hz loop
touches is preallocated and overwritten. `MarkerId.createDenseSet(...)` builds the session
marker set once; every `MocapFrame` shares it and addresses observations by
`MarkerId.getIndex()`, so a per-frame marker lookup is an array read.

`CalibrationResult` lives here rather than in `calibration` because it is the one object the
offline calibrator and the runtime loop both touch. Anywhere else and one of those packages
imports the other, which FRAMEWORK.md §19 forbids.

### Watch out for — `core`

1. **Marker indices are only meaningful within one marker set.** Two sets built separately
   assign index 1 to different markers. `MocapFrame.get(MarkerId)` verifies identity and
   throws rather than trusting the index, but the fix is to build the set once with
   `createDenseSet` and pass that same list everywhere.
2. **Unset means NaN, everywhere.** An invisible marker, an unsolved layout, an unmeasured
   world tilt, a fresh `GroundTruthSample` — all NaN, never zero. Zero is a legal joint
   angle, a legal position, and a legal tilt; NaN is the only value that cannot be mistaken
   for a measurement. Don't "helpfully" default any of them.
3. **Timestamps are `long` nanoseconds in whatever epoch the source defines.** The type does
   not care which epoch; it cares that mocap and encoders share one. Read
   `Capture.getTimestampSkewNanoseconds()` — a mispaired capture is valid mocap plus valid
   encoders at the wrong configuration, and nothing else about it looks wrong
   (FRAMEWORK §18.3).
4. **`K_ij` travels with the layout for a reason.** A marker seen in 3 of 30 captures has a
   position ~3× noisier than one seen in all 30, and the position itself does not say so.
   Check `ClusterLayout.getMinimumObservationCount()` before trusting a layout.
5. **`GroundTruthSample` has no velocity field and must not grow one.** FRAMEWORK §13:
   velocity is an offline second pass over the logged poses. A runtime velocity slot would
   get filled by differencing, at ~0.13 m/s of noise against a 0.025 m/s estimator. A test
   asserts by reflection that no accessor here names one.
6. **`CalibrationResultIO.read(path, markerSet)` is the one to use in a pipeline.**
   `readWithDenseMarkerSet` invents indices from file order and is for inspection tools only
   — its `MarkerId`s are not interchangeable with a running session's.

### `us.ihmc.alexMocap.mocap` — getting frames in

| Type | Role |
|---|---|
| `MocapSource` | interface: `read(frame)` + `isFinished()` |
| `MocapFrameRecorder` | writes a CSV log |
| `CsvReplayMocapSource` | reads back exactly what the recorder wrote |
| `MarkerLabeling` | Motive streaming id → `MarkerId` |
| `NatNetMocapSource` | live capture, consumer end (see below) |

The recorder/replay pair is the whole point: capture once at the gantry, then re-run every
gate and the whole calibration in CI off the file with no cameras in the room. Nothing in
this repository's tests needs hardware.

Log format — one header comment, one schema row, one line per frame:

```
# alex-mocap frame log, format 1
# invisible markers are recorded as NaN
timestamp_ns,PELVIS_1_x,PELVIS_1_y,PELVIS_1_z,PELVIS_2_x,PELVIS_2_y,PELVIS_2_z
1000000000,0.0612,-0.0344,0.0891,NaN,NaN,NaN
```

Visibility is encoded in the coordinates, not a separate flag: a visible marker always has a
finite position (`MarkerObservation.setVisible` rejects anything else), so there is no second
column that can disagree with the first three. NaN also plots as a gap in every tool you'll
open this with, where 0.0 would plot as the marker jumping to the world origin.

### Watch out for — `mocap`

1. **Write the loop as `while (!source.isFinished()) { if (source.read(frame)) … }`.** The
   shorter `while (source.read(frame))` works perfectly on a replay and then exits on the
   first dropped packet against live capture. "Nothing right now" and "nothing ever again"
   are different states; that's why the interface has two methods.
2. **`NatNetMocapSource` keeps only the latest frame.** If Motive delivers faster than you
   read, the older frame is overwritten and `getDroppedFrameCount()` increments. That's
   right for a control loop and *wrong for logging* — **check the drop count after any
   capture you intend to calibrate from.** A log with holes still produces a confident
   calibration, just from fewer captures than you think.
3. **`CsvReplayMocapSource.open(path, markerSet)` is the one to use in a pipeline.**
   `openWithHeaderMarkerSet` derives indices from column order, for inspection tools only.
   A name or ordering mismatch is refused — rebinding by column position would assign
   measurements to the wrong markers and produce poses that are wrong and look healthy.
4. **`MarkerLabeling` cannot catch a marker assigned to the wrong link.** G1 catches a label
   swap *within* a cluster, because trading places changes inter-marker distances. A thigh
   marker labelled as a shank marker gives you a clean calibration of the wrong thing.
   That assignment is taken on trust from Motive's rigid-body definitions (FRAMEWORK §21.5).
   Do check `getUnfedMarkers()` at startup — a marker no Motive id feeds can never become
   visible, so its cluster can never reach three.
5. **`readAll()` is for tests and short captures.** It holds every frame in memory.

### `NatNetMocapSource` — what is and isn't implemented

**Implemented and tested:** the handoff between the NatNet callback thread and the control
thread — labelling, latest-frame semantics, dropped-frame and unlabelled-marker accounting,
thread safety, and the guarantee that a marker Motive stops reporting comes back explicitly
not-visible rather than holding its last position.

**Not implemented:** the NatNet wire protocol. This class does not open a socket. It is
*driven* — a NatNet client calls `onFrameReceived(timestampNs, motiveIds, positionsXYZ,
count)` and this class does the rest. The intended client is `us.ihmc.mocap`'s in
ihmc-open-robotics-software (which is why FRAMEWORK §19 nests this project under
`us.ihmc.alexMocap` — to avoid a split package with it), and that artifact is not on this
build's classpath.

There is deliberately **no `connect()` method that throws**. A method that compiles and fails
at runtime reads as working code; the seam should be visible at the call site. Wiring it is a
short adapter in whichever module has the client.

Per PR_PLAN this class gets no unit test for the connection itself — a mocked NatNet client
tests the mock.

#### Manual smoke test

1. Start Motive, load the session calibration, confirm the rigid bodies are defined and
   streaming (Data Streaming pane → NatNet enabled, note the transmission type and interface).
2. Build a `MarkerLabeling` from Motive's streaming ids to your marker names. Log
   `getUnfedMarkers()` — it must be empty before you go further.
3. Wire the client's per-frame callback to `onFrameReceived` and construct
   `NatNetMocapSource`.
4. Record 60 s at rest into a CSV via `MocapFrameRecorder`. Then check, in order:
   - `getFramesReceived()` ≈ 60 × the Motive rate. Materially short means packets are being
     dropped on the wire, not by this class.
   - `getDroppedFrameCount()` is 0. Non-zero while recording means the recorder is not
     keeping up.
   - `getUnlabelledMarkerCount()` is steady and small. If it is comparable to your labelled
     marker count, the labelling ids are wrong.
   - Every cluster shows its full marker count visible for the whole 60 s.
5. Replay the CSV through `CsvReplayMocapSource` and confirm the frame count matches. From
   here on, nothing needs the cameras.
6. That 60 s of static data is also the per-axis σ measurement FRAMEWORK §20.1 asks for, and
   every threshold downstream is expressed in terms of it. Take the per-axis standard
   deviation of each marker's reconstructed position and check the anisotropy — the weak axis
   will point toward the missing camera side.

### `us.ihmc.alexMocap.gates` — pre-flight checks

`Gate` / `GateResult` / `GateRunner`, plus **G1 `RigidityGate`**. G2, G3 and G4 are empty
placeholders (PR2).

A gate runs *before* the thing it protects and answers pass/fail — the opposite of the rule
for primitives, which report numbers and decide nothing. G1 is the one that isolates mounting
and labelling from every modelling question: it consumes raw mocap and nothing else, so a
failure indicts the mount or the label and cannot be anything else. **Run it first, always.**

`gates` may not import `mocap` (FRAMEWORK §19), so G1 takes pushed frames via `accumulate`
and the CLI does the streaming. Accumulation is allocation-free and single-pass, so a 60 s
capture at 200 Hz costs nothing to hold.

### Watch out for — `gates`

1. **`σ₃` … sorry, `σ` here is per-axis position noise, and it must be measured.** No
   default anywhere: not in `RigidityGate`, not in the CLI. The wand residual is an average
   over the whole lab and is not a substitute (FRAMEWORK §17, §20.1). Get it from the 60 s
   static capture in the NatNet smoke test above.
2. **The real margin is 2.1×, not 3×.** A perfectly rigid pair still shows a spread of
   `√2·σ` — two independent noisy points, each contributing σ along the baseline — so the 3σ
   threshold sits `3/√2 = 2.12×` above the noise floor. `getNoiseFloor()` reports the floor;
   the report prints both. Know this before tightening the constant.
3. **G1's sensitivity depends on the *shape* of the fault, not just its size.** A step of
   amplitude `a` (mount slips once) has std `a/2`; a linear creep of the same total travel
   has std `a/√12`, which is 1.7× smaller. At σ = 0.3 mm the detection threshold is about
   **1.7 mm for a slip** and about **3.1 mm for a creep**. So a green G1 does *not* rule out
   a slowly loosening mount below ~3 mm of travel. Both numbers are asserted in
   `RigidityGateTest`.
4. **And on geometry.** Only the component of a movement *along* a pair's baseline changes
   that pair's distance. A marker shifting perpendicular to a baseline is invisible to that
   pair however far it goes — which is why the cluster is checked pairwise rather than by one
   aggregate number, and why non-collinear geometry is a real requirement rather than
   pedantry.
5. **INCOMPLETE is not PASS, and it exits 1.** A pair is measurable only in frames where both
   its markers were visible; below `--min-samples` (default 100) it is reported
   `NOT_EVALUATED`. With only two states, a cluster whose markers never appear together would
   produce the most confident possible green, because nothing contradicted it.
6. **Clusters are inferred from marker names** by the prefix before the last underscore, so
   `PELVIS_1, PELVIS_2, …` become the `PELVIS` cluster. That is a convention, not a fact —
   pass `--cluster <name>=<m1,m2,…>` when the naming doesn't match the mounting. Inference
   throws rather than guessing if a name has no underscore or a group has fewer than three
   markers.
7. **G1 cannot catch a marker assigned to the wrong link** — only swaps *within* a cluster,
   which change inter-marker distances. FRAMEWORK §21.5.

### `us.ihmc.alexMocap.model` — F1, the FK reference

`URDFLoader` (URDF → Mecano tree, plus a SHA-256 for provenance) and `RobotModelHandle`
(`setConfiguration`, `updateFrames`, `packLinkToBase` = `^b T_i(q)`).

This answers **where the model says the link is**, from the URDF and encoders alone. The
other object — where the link *actually* is — comes from a marker cluster through
`RigidBodyRegistration`. FRAMEWORK §0 exists to keep those two apart; they live in separate
types for the same reason.

### Watch out for — `model`

1. **SCS2 inserts a `SixDoFJoint` you did not ask for.** The instantiated tree is
   `rootBody → [6-DoF] → pelvis → …`, where `rootBody` is synthetic and appears in no URDF.
   The base frame `b` is the frame *after* that joint. Taking the synthetic root's frame
   instead is the obvious reading and is wrong: every `^b T_i` would then contain the
   floating joint, and §0's load-bearing claim that `^b T_i(q)` depends on joint angles
   alone would stop holding — silently, since at identity it looks correct.
   `RobotModelHandleTest` moves the base to 20 random poses and requires no `^b T_i` to move.
2. **`getBodyFixedFrame()` is the centre-of-mass frame, not the URDF link frame.** It comes
   out of the tree named `l_thighCoM`. FRAMEWORK §3 said to read it; that instruction has
   been amended, and this package uses `parentJoint.getFrameAfterJoint()`. Both conventions
   are self-consistent for the calibration itself, but §12 and §14 presuppose a non-zero
   `^i c_i`, which is identically zero in the CoM frame — it would delete the link-CoM term
   from the error budget.
3. **`getLinkNames()` excludes the synthetic root**, so a `MarkerCluster` naming a link that
   does not exist fails loudly rather than matching scaffolding.
4. **Prefer `setConfiguration(EncoderSample)` to `setQ(double[])`** for anything from outside
   the process. Only the former checks joint names.

### `us.ihmc.alexMocap.frames` — F8 and the pelvis triad

`TiltMeasurement`, `GravityAlignedWorldFrame`, `PelvisFrameTriad`, `FrameNames`.

F8 is a **frame node, not a correction function** (§11). A correction applied at call sites
can be forgotten at call sites, and forgetting it produces no error — just a CoM ~7 mm low
at 0.5° of tilt. The tree is `Wg → W`, gravity-aligned as the parent of Motive's tilted
world, so `changeFrame(Wg)` *is* the correction and cannot be half-applied.

### Watch out for — `frames`

1. **`TiltMeasurement` has no zero-argument constructor and no silent default.** The only
   untilted instance is `assumedLevel(note)`, which demands a written justification, reports
   `isMeasured() == false`, and prints `ASSUMED_LEVEL` everywhere. §11: measured, never
   assumed.
2. **It carries a direction, not just §11's scalar `θ`.** A 0.5° tilt toward +x and one
   toward +y give the same height error and completely different horizontal CoM. The
   correction is the *minimal* rotation carrying measured-up onto +z — a tilt constrains two
   DOF, not three, so it must not invent heading.
3. **Frames are instance fields, not statics.** A session constant is not a process constant;
   a test suite covers several tilts. Pass a `nameSuffix` when constructing more than one —
   Euclid rejects duplicate frame names under one parent.
4. **`setBaseToImu` requires you to say whether the transform is verified.** There is no
   overload that omits it. The `b → imu` offset is an unverified CAD number and getting it
   wrong costs `ω × r` = 0.1 m/s at 1 rad/s and 0.1 m, which reads as an estimator
   regression rather than as a bookkeeping error (§13).

### `us.ihmc.alexMocap.calibration` — F2–F5 and the A′ loop

| Type | Role |
|---|---|
| `CaptureSet` | the `K` captures + marker set, joint order, clusters, gauge |
| `BaseInitializer` | F2: `Δ = I`, and gauge-cluster tracking → `^W T_c^(k)` |
| `BootstrapSolver` | F3, the "software T-pose" — F4 over one capture |
| `MarkerLayoutSolver` | F4, the marker step: an unweighted mean |
| `BaseAlignmentSolver` | F5, the base step: one Procrustes over everything |
| `AlternatingCalibrator` | A′, plus convergence |
| `CalibrationReport` | objective trace + per-marker residuals |

### Watch out for — `calibration`

1. **`^W T_c^(k)` is computed once, before the loop, and never updated.** §7 says these come
   from "F6 applied to the pelvis cluster", which invites recomputing them each iteration
   from the freshly solved pelvis layout. Don't. A′'s monotonicity argument is that each step
   exactly minimises *a fixed objective*; moving `^W T_c^(k)` between iterations changes `J`
   itself, and a decreasing sequence of values of different functions means nothing.
2. **The cluster shape is centred on the gauge markers' centroid.** This looks cosmetic and
   is not — see *Gotchas*.
3. **The default iteration cap is 500, not §8's 50.** A′ converges linearly. On real data the
   tolerance stops it in a few tens of iterations; on noiseless or weakly-conditioned data
   `J` keeps falling geometrically and 50 stops it about 1 mm short, reporting a small `J`
   and looking converged. Always read `isConverged()`.
4. **`Δ` is convention-dependent and not comparable across runs** that chose a different
   reference capture. What is comparable is `^W T_c^(k) · Δ`, the base pose.
5. **In-sample RMS is not an accuracy claim.** The report says so in the line itself.

### `gates` — G2 and G4 (PR2)

`BootstrapSpreadGate` (G2) and `HeldOutResidualGate` (G4). Neither imports `calibration`
(§19); both take their inputs at construction, so G2 can run before the calibrator exists.

6. **G2 takes `Δ` as an input, and that changes how a failure reads.** The back-projection is
   not `Δ`-free: a wrong `Δ` can only make the spread *larger*, never smaller. So G2 at time
   zero with `Δ = I` is **conservative** — it can cry wolf, it cannot miss a real systematic
   error. For the diagnosis of *which* assumption is wrong, pass the solved `Δ`. The CLI does.
7. **G2's expected spread is per marker, not per session**, because it must include the
   gauge-cluster term scaled by that marker's lever arm. A gate built on `σ√3` alone fires on
   clean data.
8. **G2 is blind to an offset on a terminal joint.** It is absorbed wholesale into the last
   link's layout, and is genuinely unidentifiable from marker data. Sensitivity grows with
   depth below the offending joint; the link immediately below it barely moves.
9. **G2 cannot separate "joint offset" from "elasticity" by correlation.** Elastic deflection
   enters through gravitational torque, which is itself a function of joint angle, so both
   correlate strongly. The reliable discriminator is spatial: an offset raises spread on one
   branch, elasticity on every loaded branch at once.
10. **G4 does not show the asymmetry PR_PLAN describes.** See *Gotchas*.

### `us.ihmc.alexMocap.runtime` — F6, F7, F9, F10

| Type | Role |
|---|---|
| `MeasuredLinkPoses` | per-link pose + source + conditioning + refusal reason |
| `LinkPoseEstimator` | F6: register the layout against live markers. Owns the refusal policy |
| `KinematicChainCoupler` | F7: chain unmarked links from a marked ancestor |
| `CenterOfMassGroundTruth` | F9: the mass-weighted sum, in `Wg` |
| `PelvisStateExtractor` | F10: pelvis pose. **No velocity accessor** |

### Watch out for — `runtime`

1. **The refusal guard is `σ₂`, not `σ₃`.** FRAMEWORK §9 and §18.1 both say `σ₃`. Refusing
   on `σ₃` rejects the normal case — see *Gotchas*.
2. **A chained pose and a measured pose are not interchangeable, and the transform does not
   say which.** Read `MeasuredLinkPoses.getSource()`. `KinematicChainCoupler.getChainedMass()`
   is the number §10's trade is actually made against: if a large fraction of the robot is
   chained, the CoM is an FK result wearing a mocap costume.
3. **A refused ancestor takes everything below it.** One near-collinear pelvis cluster
   removes every chained link from that frame. That is correct, and it means a single bad
   cluster can cost a whole frame's CoM.
4. **`CenterOfMassGroundTruth.compute` returning `false` means there is NO CoM**, and it packs
   NaN. It does not return the CoM of a lighter robot. Check the return value;
   `getMissingMass()` tells you the size of what was unmeasurable, in kilograms.
5. **F9 applies the F8 tilt correction itself.** Do not apply it again at the call site.
6. **`PelvisStateExtractor` must never grow a velocity accessor.** A reflection test enforces
   it against names *and* return types. If you need velocity, it is `postprocess`, offline.

### `us.ihmc.alexMocap.postprocess` — the second pass and the budget

| Type | Role |
|---|---|
| `SGDifferentiator` | centred Savitzky-Golay first derivative, zero-lag by construction |
| `PelvisTwistEstimator` | linear + angular velocity over a logged pose trajectory |
| `COMErrorBudget` | F11's three terms |
| `ErrorBudgetReport` | which term dominates, and what perfect mocap would buy |

Depends on `core` and nothing else — F11 is arithmetic on arrays the caller assembles, so it
does not reach for a robot model to do it.

### Watch out for — `postprocess`

1. **The differentiator's input is the pelvis *pose* noise, not the marker noise.** F6
   registers four markers into one pose, so `σ_pose ≈ σ/√N`. FRAMEWORK §17's 0.0037 m/s only
   comes out that way; fed the raw 0.93 mm the same window gives 0.0067 m/s. Use
   `getNoiseGain() × σ_pose` to choose a window rather than guessing.
2. **Edges are NaN and must stay NaN.** A one-sided window at the ends would reintroduce
   exactly the lag the filter exists to avoid, at the samples nobody scrutinises.
3. **Degree 2 buys nothing over degree 1 for a first derivative.** The quadratic term is even
   and cannot influence an odd coefficient. The useful choices are 2 and 3.
4. **§14's first term assumes `Σδm_i = 0`** — the total pinned by a scale reading.
   `packShiftFromMassErrors` checks it and throws otherwise; use
   `packExactShiftFromMassErrors` for the unpinned case.
5. **Angular velocity comes from `vee(skew(Ṙ Rᵀ))`.** Watch `getWorstNonSkewResidual()`: a
   large value means the pose log is noisy enough that the rotations are not staying on SO(3)
   through the filter, which is a data problem, not a filter one.

### `us.ihmc.alexMocap.scs2` — telemetry and the view

`GroundTruthYoVariables` (CoM, pelvis pose, per-link `σ₃`/visible/accepted),
`ConditioningMonitor` (session histograms — headless, and what §20.4 sends you to the gantry
to produce), `GroundTruthYoGraphics`, `GroundTruthSessionVisualizer`.

### Watch out for — `scs2`

1. **This is the only package allowed to touch JavaFX or YoVariables**, enforced by
   `PackageDependencyTest` against the compiled classes.
2. **`ConditioningMonitor` reports a histogram, not a minimum**, because §20.4's question is
   *how often* — a cluster that dips to two markers for one frame in a thousand is a different
   mounting problem from one that spends a third of the sweep there, and a minimum reports
   them identically.
3. **There is no velocity YoVariable, deliberately.** One named `pelvisLinearVelocity` would
   be plotted against the estimator and would show 0.13 m/s of differencing noise.

### `us.ihmc.alexMocap.registration` — the primitive

The registration primitive of FRAMEWORK.md §2, consumed by F5, F6, G1 and G4. Two classes:

- `RigidBodyRegistration` — Umeyama closed-form pose from point correspondences. Stateful,
  **not thread safe** (owns preallocated EJML scratch), allocation-free in steady state.
  One instance per caller.
- `RegistrationResult` — pose plus the conditioning numbers (`σ₁ ≥ σ₂ ≥ σ₃`, correspondence
  count, reflection-corrected flag). Reports numbers, decides nothing.

Occlusion needs no special handling: an unseen marker is a correspondence you never add.

### Watch out for — `registration`

1. **`σ₃` is comparable across marker *counts*, not across marker *geometries*.** `H` is
   normalised by `L`, so occluding a marker no longer drops `σ₃` just because the sum got
   shorter. It does still move `σ₃`, because losing a marker genuinely changes the cluster's
   shape — and no normalisation can or should hide that. So `σ₃` is a fair conditioning
   number frame to frame, but a drop in it does **not** by itself mean "a marker occluded".
   Log the visible count alongside it (FRAMEWORK §9) and read the two together.
2. **`σ` has units of length², not length.** They are mean-squared spreads. A cluster with
   120 mm spread reports `σ ≈ 0.003`, not `0.12`. Pick refusal thresholds accordingly — this
   is the easiest way to set one off by three orders of magnitude.
3. **`compute` returning `true` is not a quality claim.** Three collinear markers give a
   perfectly well-formed rotation and `σ₂ = σ₃ = 0`. Nothing in the transform betrays it.
   Check `σ₃` — that is the whole of FRAMEWORK §18.1.
4. **Don't test `σ₃ == 0`.** With real noise a collinear cluster reports `σ₃` strictly
   positive but ~4 orders below `σ₁`. Compare the *ratio*, or compare against your measured
   per-axis noise.
5. **`wasReflectionCorrected()` firing is normal, not a fault.** On a near-planar cluster
   the sign of the raw `U Vᵀ` is a coin flip; it fires on ~50% of frames. It is a diagnostic
   for *how planar your cluster is*, not an error flag.
6. **One instance per caller, and it is not thread safe.** It owns preallocated EJML scratch.
   Sharing one across two solvers or two threads silently corrupts both.
7. **Allocation-free only once capacity is reached.** `addCorrespondence` grows by doubling,
   which allocates. Size the constructor to your worst case — markers per cluster for F6,
   `links × markers × captures` for F5.
8. **No frame checking.** `addCorrespondence` takes bare `Point3DReadOnly` and will happily
   accept a source point in the wrong frame. Frame safety is F8's job (FRAMEWORK §11), and
   it lives above this class, not in it.

```java
RigidBodyRegistration registration = new RigidBodyRegistration(markerCount);
RegistrationResult result = new RegistrationResult();

registration.clear();
for (int j = 0; j < markerCount; j++)
   if (visible[j])
      registration.addCorrespondence(layoutInLinkFrame[j], measuredInWorld[j]);

if (registration.compute(result))
   ... // result.getTransform() is ^W T̂_i; result.getSigma3() says whether to believe it
```

## Dependencies

Declared in `gradle/libs.versions.toml`, consumed in `build.gradle.kts`:

| Module | Version | Scope |
|---|---|---|
| `us.ihmc:euclid` | 0.22.5 | `api` |
| `us.ihmc:euclid-frame` | 0.22.5 | `api` |
| `us.ihmc:euclid-geometry` | 0.22.5 | `api` |
| `us.ihmc:mecano` | 17-0.19.2 | `api` |
| `us.ihmc:scs2-definition` | 17-0.30.0 | `implementation` |
| `us.ihmc:ihmc-yovariables` | 0.13.6 | `implementation` |
| `us.ihmc:scs2-session-visualizer-jfx` | 17-0.30.0 | `implementation` |
| `org.ejml:ejml-core` | 0.39 | `implementation` |
| `org.ejml:ejml-ddense` | 0.39 | `implementation` |
| `org.junit.jupiter:junit-jupiter` | 5.10.2 | `testImplementation` |

The `application` plugin provides `./gradlew run` and `./gradlew installDist`. `run` sets
`isIgnoreExitValue = true` so a gate failure surfaces as the gate's own table and exit code
rather than a Gradle stack trace on top of it; `installDist`'s launcher propagates the real
code (verified: 0 / 1 / 2).

Euclid and mecano are `api` because their types (`RigidBodyTransform`, `ReferenceFrame`,
`RigidBodyBasics`) show up in this package's own public signatures.

EJML is `implementation` on purpose. No EJML type appears in a public signature —
`RegistrationResult` reports doubles and a `RigidBodyTransform` — and keeping it off the
api surface is what stops a caller reaching past `RigidBodyRegistration` and writing a
second SVD. FRAMEWORK.md §2: *"There must be exactly one implementation."*

`0.39` is not a new version in the graph: euclid already pulls `ejml-core:0.39` and mecano
pulls `ejml-ddense:0.39`. It is declared explicitly so that `registration` does not depend
on mecano in order to get an SVD.

**`scs2-definition` is the headless half of SCS2** — `URDFTools`, `RobotDefinition`, and
`RobotDefinition.newInstance()` handing back a Mecano tree. It pulls **no JavaFX**, and its
POM pins euclid 0.22.5 and mecano 17-0.19.2, exactly the versions already declared, so
adding it upgrades nothing. `scs2-session-visualizer-jfx` is a separate artifact and is
deliberately not declared until PR3.

It is `implementation`, not `api`, for the same reason as EJML: `URDFLoader` takes a `Path`
and returns a Mecano `RigidBodyBasics`, so no SCS2 type reaches any signature anywhere in
the project.

**`scs2-session-visualizer-jfx` drags JavaFX 17.0.8** and arrives with PR3. It resolves fine
on Linux and adds nothing to the test runtime, because nothing in the suite touches it — no
test initialises a toolkit, and `PackageDependencyTest` confines both JavaFX and YoVariables
to the `scs2` package by scanning compiled classes. That is the rule that keeps every other
package runnable on a machine with no display.

FRAMEWORK §19 used to say "nothing outside `scs2` imports `scs2`", and **nothing enforced
it** — `PackageDependencyTest` only scanned `us/ihmc/alexMocap/` names, so it was blind to
`import us.ihmc.scs2.*` and any package could have taken the dependency without a test
moving. It now scans `us/ihmc/scs2/` too: only `model` may use `scs2-definition`, and only
the `scs2` package may use SCS2 beyond it. The rule in §19 was narrowed to match, since its
stated motive was headless-testability and `scs2-definition` is headless.

## Gotchas

**Do not enable `org.gradle.configuration-cache` in `gradle.properties`.** Buildship —
which jdtls, Eclipse, and IntelliJ all use to import Gradle projects — injects
`apply plugin: "eclipse"` through an init script. That plugin's `GenerateEclipseClasspath`
task holds a `SourceSetContainer`, which the configuration cache cannot serialize, so the
IDE's model request fails. The failure is silent from the editor's side: the project
imports with the Gradle nature but no Java nature, and the LSP reports
`Error in Java Model (code 969): alex-mocap does not exist` in
`~/.cache/nvim/jdtls/alex-mocap/workspace/.metadata/.log` while showing you an
apparently-fine, symbol-free buffer. `./gradlew build` stays green throughout.

**The package directory must be a legal Java identifier.** This package originally lived
in `src/main/java/us/ihmc/alex-mocap/`; hyphens are not valid in Java identifiers, so
`us.ihmc.alex-mocap` is not a package any compiler or LSP can form. It went unnoticed
because every file was 0 bytes — an empty compilation unit has no `package` line to
reject, so `compileJava` passed. The directory is now `alexMocap` (camelCase, matching
IHMC house style: `us.ihmc.robotDataLogger`, `us.ihmc.commonWalkingControlModules`). Keep
the artifact/repo name `alex-mocap`; only the Java package segment is constrained.

**`artifactory.ihmc.us` is not reachable off-VPN, and fails silently.** Every path under
it returns HTTP **200 with a zero-byte body** — including paths that do not exist. It does
not 404. If you add it as a Gradle repository and are off-VPN, you get confusing
"corrupt/empty POM" style failures rather than a clean "not found". This build therefore
resolves purely from `mavenCentral()`.

**`mecano 17-0.19.3` is not on Maven Central.** It may be in your local Gradle cache
(`~/.gradle/caches/modules-2/files-2.1/us.ihmc/mecano/`) from another IHMC project built
on-VPN, which makes it look available. The newest release on Central is `17-0.19.2`, which
is what this build pins. If you need 0.19.3, you must be on VPN with artifactory added.

**euclid version is upgraded past mecano's declared one.** `mecano 17-0.19.2` declares
`euclid 0.22.3`; this build pins `0.22.5`, so Gradle resolves `0.22.3 -> 0.22.5` for all
three euclid modules. Patch-level and binary compatible. Confirm with the `dependencies`
command above if you bump either version.

**The `17-` prefix in mecano's version is the Java flavor, not a major version.** Version
ordering is `17-0.19.1` < `17-0.19.2`; do not read it as semver.

### Layout accuracy is not `σ/√K`. It is set by the gauge cluster.

This is the single most useful thing PR2 turned up, and it changes what hardware work is
worth doing.

PR_PLAN derived its 0.2 mm target from F4's averaging: "theoretical is `σ/√K ≈ 0.055 mm`".
That term is real and correct, and it is **not** the dominant one, so no implementation can
hit a threshold derived from it.

Every capture's base pose comes from registering four pelvis markers, so it carries an
angular error of order `σ / (√N · r_perp)` — which is exactly the scaling FRAMEWORK §1
states in the sentence asking for an outrigger bracket. That angle is then multiplied by the
lever arm out to each link. Measured, at σ = 0.3 mm, RMS layout error over all markers,
three seeds:

| K | all markers noisy | gauge cluster noiseless |
|---|---|---|
| 30 | 0.383 mm | 0.0885 mm |
| 480 | 0.188 mm | 0.0226 mm |
| ratio | **2.04×** | **3.92× (= √16)** |

Take the noise off the four pelvis markers and everything downstream behaves exactly as §6
predicts — 0.0885 mm at K=30 against a 0.055 mm floor, improving as a clean `1/√K`. Put it
back and the error is 4.3× worse *and* the exponent breaks, because two things are going on:

1. **Per-capture gauge noise** does average as `1/√K`, but it enters amplified by the lever
   arm, which is why the per-marker residual table climbs from pelvis to foot.
2. **Reference-shape noise does not average at all.** `BaseInitializer` defines the cluster's
   shape from one capture's marker positions, so that capture's noise is baked into the
   frame definition for the whole session. This is what bends the exponent to about 0.25.

Practical consequences, in order of leverage:

- **Widen the pelvis cluster.** Error scales as `1/r_perp`, verified: 0.06 m → 3.00 mm,
  0.10 → 1.80, 0.14 → 1.29, 0.20 → 0.91. §1's outrigger bracket is the cheapest accuracy
  you can buy in this project.
- **Lower σ** (the §20 camera work). Linear.
- **More captures is the weakest lever** — 16× the captures bought 2×.
- **Item 2 above is removable and is not done.** Run A′, rebuild the gauge shape from the
  converged pelvis layout (averaged over all K, so `√K` quieter than any single capture),
  then run A′ again with that new fixed `^W T_c^(k)`. Monotonicity survives because each run
  minimises its own fixed objective. It is left out because it turns A′ into a two-stage
  method, which is a specification decision rather than an implementation one.

### Two things PR_PLAN expects that the algebra does not give

**G2 does not correlate with "that specific joint's excursion".** Writing the chain as
`T = J_1 … J_j(q_j) … J_n` and injecting `q_j → q_j + δ`:

```
(T_model)⁻¹ T_true  =  [J_{j+1} … J_n]⁻¹ · R(δ) · [J_{j+1} … J_n]
```

Everything above `j` cancels, and so does `J_j`'s own fixed offset. So the back-projection
error depends on the joints **strictly below** `j`, not on `q_j`. Three consequences: markers
above the fault show nothing; the link immediately below shows nothing either (empty
bracket, constant error, absorbed into the layout); and sensitivity accumulates with depth.
G2 therefore localises to *the branch and the depth*, which is more information than a joint
name — but **an offset on a terminal joint is invisible**, and correctly so, since it is
unidentifiable from marker data.

Related: correlate the deviation **magnitude** against a joint angle and you find nothing.
The signed displacement is roughly linear in the joint angle, so its magnitude is V-shaped
and its Pearson correlation is ~0 however strong the dependence. Measured on a 0.5° `l_hip`
injection, magnitudes reported "indicts nothing" on markers failing the spread test by 3.2σ.
G2 correlates signed components.

**G4 does not show "held-out blows up while in-sample stays low".** Measured on that same
injection, in-sample and held-out RMS are within 3% — both raised together, sixfold. An
i.i.d. split samples the same configuration distribution in both halves, so a systematic
bias is present equally in each. Held-out validation detects **overfitting**, and this fit
has 90 parameters against 1680 observations. The absolute level is the signal here, not the
ratio. Getting the literal asymmetry would need the held-out split to cover configurations
the training split did not — a better G4 design, and not implemented.

### Refuse on `σ₂`, not `σ₃`. `σ₃ ≈ 0` is the normal case.

FRAMEWORK §9 and §18.1 both say to log `σ₃` and refuse below a threshold on it. Logging it is
right; **refusing on it rejects good data.** Writing the marker cloud's covariance eigenvalues
as `λ₁ ≥ λ₂ ≥ λ₃`:

| shape | `σ₂` | `σ₃` | pose determined? |
|---|---|---|---|
| generic | > 0 | > 0 | yes |
| **coplanar** — markers on a flat link face | > 0 | ≈ 0 | **yes** |
| collinear | ≈ 0 | ≈ 0 | no |

Three non-collinear points already fix a 6-DOF pose, so a plane is entirely sufficient. §2
says as much in passing — "a noisy or near-planar cluster — the realistic case, markers on a
flat link face" — which makes the two sections quietly inconsistent. What genuinely destroys
a pose is collinearity, and that is `σ₂`.

The cost of conflating them is not theoretical. The toy's `l_shank` cluster came out
near-coplanar with a nominal `σ₃` of `3.1e-08 m²` — an out-of-plane extent of 0.17 mm, *below
the 0.3 mm mocap noise*. Its `σ₃` then fluctuated with the noise and refused roughly **one
frame in six** of a replay where every pose was fine. Its `σ₂` was four orders of magnitude
larger and perfectly steady. You can see both in the `ReplayRunner` conditioning table above.

`σ₃` is still computed and logged every frame per §9 — it measures how planar a cluster is,
which is worth watching because that is where the Umeyama reflection guard earns its keep.

### F9 cannot be Mecano's `CenterOfMassCalculator`, but Mecano is the right oracle

§12 says "Mecano's `CenterOfMassCalculator` implements this. Do not reimplement it; feed it
the measured poses." It computes from the tree's own frames, which are a function of a joint
configuration — and independently measured link poses are in general consistent with **no**
joint configuration. That inconsistency is the signal the whole method exists to expose;
feeding the calculator a joint configuration instead would make F9 an FK result measuring the
URDF rather than the robot.

So the sum is written out directly, and Mecano's calculator is the **test oracle**: on data
where the measured poses *are* FK-consistent, the two must agree to `1e-9`. They do. That is
stronger as a test than it would have been as an implementation, because the two paths share
no code.

### §17's 0.0037 m/s is a pose-noise figure, not a marker-noise one

F6 registers four pelvis markers into one pose, so what reaches the differentiator is
`σ/√N ≈ 0.47 mm`, not the raw 0.93 mm. That distinction is invisible in §13's prose and is a
factor of two in the headline number: fed pose noise, a 0.1 s centred window at 200 Hz gives
0.0034 m/s and meets PR_PLAN's 0.005 target; fed raw marker noise it gives 0.0067 m/s and does
not. Use `SGDifferentiator.getNoiseGain() × σ_pose` to size a window rather than guessing.

## Tests

JUnit 5 (`5.10.2`), run on the JUnit Platform.

```bash
./gradlew test                                   # all of it, ~1 s
./gradlew test --tests '*testReflectionGuard*'   # one test
./gradlew test --rerun-tasks                     # ignore the build cache
```

Reports land in `build/reports/tests/test/index.html`; machine-readable results in
`build/test-results/test/*.xml`. `testLogging` is configured to dump full stack traces on
failure, so a CI log is enough to diagnose without fetching the report.

Currently 168 tests, no external resources, no display, no hardware. Two URDFs are checked in
— the toy 6-DOF one and the real Alex one — and nothing else is read from outside the repo:

| Class | Covers |
|---|---|
| `registration.RigidBodyRegistrationTest` | exact recovery, reflection guard, rank deficiency, singular-value ordering, count normalisation, `σ/√N` noise scaling, below-minimum NaN, allocation-free, capacity growth |
| `core.CoreDataTypesTest` | dense marker sets, cross-marker-set rejection, NaN-when-unset, visible counts, capture skew, joint-order checking, `K_ij` bookkeeping, no-velocity-by-reflection, allocation-free frame loop |
| `core.CalibrationResultIOTest` | exact JSON round trip incl. byte-identical rewrite, `Δ` over 1000 random transforms, NaN survival, unknown-marker rejection, format version, parse errors |
| `mocap.CsvRoundTripTest` | exact CSV round trip incl. visibility over 200 frames at 20% occlusion, self-describing header, marker-set and ordering mismatch, corrupt lines, untimestamped frames, comments |
| `mocap.MocapSourceTest` | sparse Motive ids, labelling bijection, unfed markers, no stale positions between live frames, dropped-frame accounting, concurrent producer/consumer, allocation-free live handoff |
| `gates.RigidityGateTest` | rigid cluster passes, 2 mm slip fails and names the marker, slip-vs-creep sensitivity, perpendicular-baseline blindness, label swap, rigid-body motion, unevaluated≠pass, threshold algebra, allocation-free |
| `CalibrationRunnerTest` | the CLI end to end over real files: exit 0/1/2, incomplete exits non-zero, sigma required, cluster inference and override, usage errors |
| `model.RobotModelHandleTest` | URDF load, joint order, **`^b T_i` invariance under 20 random base poses**, link frame vs CoM frame, FK against hand-computed URDF arithmetic, permuted joint order rejected, parse errors carry a reason |
| `frames.FramesTest` | §11's 7 mm at 0.5°, correction carries up onto +z without inventing heading, `changeFrame` applies it, assumed-level must justify itself, 90° tilt rejected, pelvis triad composes in one order, `ω × r` = 0.1 m/s |
| `calibration.PlantAndRecoverTest` | noiseless K=5 exact to 4.6e-15 m, realistic and today's-lab noise, **J monotone across every half-step over 4 seeds**, convergence and determinism, 20% occlusion with `K_ij` bookkeeping, never-seen marker stays NaN, frozen legs leave the base step ill-conditioned, the gauge-cluster scaling law and error decomposition |
| `gates.GateInjectionTest` | G2 passes clean, fires on a 0.5° offset and localises to the affected branch, is blind to a terminal-joint offset, fires on elastic deflection across both branches; G4 clean vs injected |
| `CalibrateCliTest` | `--calibrate` end to end over two CSVs and a URDF, provenance round trip, unmeasured tilt warned about, gauge defaults to the base link, permuted joint order rejected, encoder CSV bit-exact round trip |
| `runtime.RuntimeGroundTruthTest` | **CoM vs Mecano's own `CenterOfMassCalculator` to 1e-9**, F7 chaining exact with only the pelvis marked, refused-ancestor propagation, F8 injection and correction at the CoM, occlusion refusal, near-collinear refusal on `σ₂`, near-coplanar cluster accepted |
| `runtime.PelvisStateExtractorTest` | no velocity or twist by reflection (names *and* return types), pose reported in `Wg`, refusal does not keep the last good pose |
| `postprocess.SGDifferentiatorTest` | antisymmetric kernel, exact on polynomials to its own degree, degree-2 is the least-squares slope, velocity RMS under 0.005 m/s, **zero lag with a causal filter of equal width as control**, NaN edges |
| `postprocess.COMErrorBudgetTest` | 1% mass perturbation against the closed form pinned and unpinned, link-CoM and pose terms, §14's conclusion that CAD dominates, and the flip side once inertials are measured |
| `ReplayRunnerTest` | both CLIs end to end: capture → `--calibrate` → replay → CoM trajectory, NaN velocity edges, refused frame exits non-zero, unmeasured tilt visible |
| `PackageDependencyTest` | the FRAMEWORK.md §19 dependency table, by scanning compiled class files; SCS2 containment against the external library; JavaFX and YoVariables confined to `scs2` |
| `AlexLegDemoTest` | **the real 91.5 kg Alex model**: 29 joints / 30 links / finite inertials after the fixed-joint merge, SCS2's link-frame rewrite, noiseless exactness, σ = 0.3 mm and σ = 0.93 mm recovery, monotonicity, **the hip-X-only degeneracy**, narrow-sweep conditioning collapse, **58.45% chained mass**, CoM vs Mecano to 1e-9 on 91.5 kg, G2 clean / fired / floored, **the 140 mm bracket missing the TALOS bar** |
| `AlexLegDemoCliTest` | both shipping CLIs unchanged on the real model, cluster inference producing exact URDF link names with no `--cluster`, and the degenerate marked set exiting 0 while being 56 mm wrong |

### Reading the tests

Two conventions carried from `PR_PLAN.md`, worth knowing before you change one:

- **Fixed seeds, loose thresholds.** Every randomised test seeds its `Random` explicitly and
  asserts a threshold several times the theoretical value, with that value stated in a
  comment. A test that flakes gets disabled and a disabled test is worse than none.
- **The guards are load-bearing, and were checked by breaking them.** Every one of these was
  mutated and the intended test confirmed to fail:

  | Mutation | Caught by |
  |---|---|
  | determinant repair removed (Arun, not Umeyama) | `NotARotationMatrixException` |
  | `sortDescending()` skipped | `σ₁` returns as the *smallest* singular value |
  | `1/L` dropped from `H` | count normalisation: 25.0% shift |
  | `double[4]` allocated in `compute` | 480,000 bytes |
  | `MocapFrame` trusts the index instead of verifying identity | cross-marker-set test |
  | invisible marker keeps its last position | NaN test |
  | NaN written as `0.0` instead of `null` | never-observed and world-tilt tests |
  | `Δ` written as the full 4×4 | `Δ` round trip |
  | allocation escaping from `MocapFrame.clear()` | 320,000 bytes |
  | live source skips `staging.clear()` (stale positions persist) | no-carry-over test |
  | CSV reader ignores header/marker-set mismatch | mismatch test |
  | partially-NaN triple treated as occluded | corrupt-line test |
  | dropped frames not counted | drop-count and concurrency tests |
  | recorder accepts untimestamped frames | untimestamped test |
  | NOT_EVALUATED counted as a pass | unevaluated-pair test |
  | incomplete gate exits zero | CLI incomplete test |
  | naive sum-of-squares variance instead of Welford | three G1 tests |
  | occluded pairs silently skipped | unevaluated-pair test |
  | CLI gives sigma a silent default | sigma-required test |
  | `registration → core` import added | both dependency tests |

  EJML really does return singular values unordered — the sort is not a theoretical concern.
  If you loosen a threshold, re-run the matching mutation.

### The allocation tests

`AllocationMeasurement.assertAllocationFree` measures with `com.sun.management.ThreadMXBean`
`getThreadAllocatedBytes`, runs the batch five times after warmup, and asserts the
**minimum** is zero. Not the mean, and not a single window: the JIT recompiles on its own
schedule and charges that bookkeeping to the running thread, so single-window readings of
allocation-free code look like `2792, 0, 104, 0, 0`. The minimum is exact rather than
approximate — if the batch allocated one byte per iteration, no window could read zero.

It validates its own meter first, so a zero is never vacuous. No JVM flags are needed:
HotSpot's counter includes the in-progress TLAB.

**What it proves.** What the JVM actually allocates — which is the quantity that matters,
since a scalar-replaced allocation costs the collector nothing. It is *not* a proof that the
source contains no `new`: injecting `new double[2]` into `MocapFrame.clear()` is invisible
here because escape analysis removes it, while the same array assigned to a static field
shows up instantly. Keep hot-path methods small and don't lean on that deliberately.

If the registration one fails after a dependency bump, suspect EJML: the zero depends on
`SvdImplicitQrDecompose_DDRM` reusing its internals across same-size `decompose` calls, and
on `getU`/`getV` reshaping rather than reallocating a preallocated 3×3.
