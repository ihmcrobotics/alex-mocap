# BRINGUP.md — standing up a real OptiTrack session with Alex

**Ordered by lead time, not by workflow order.** The failure mode this guide is designed against is
not a bug: it is arriving at the lab with a bracket that is 140 mm wide, taking a perfect capture,
getting a green calibration, and being 2.9 mm wrong forever. Software cannot recover any of the
decisions in the first two sections. Read those before anyone touches the robot.

Where things live, so this file does not repeat them:

| Document | What is in it |
|---|---|
| [`FRAMEWORK.md`](FRAMEWORK.md) | the spec. F1–F11, the gates G1–G5, §17's operating numbers, §20's hardware sequence. |
| [`RUNNING.md`](RUNNING.md) | the operator's manual. Build, CLI options, sample output, every "Watch out for". |
| [`CLAUDE.md`](CLAUDE.md) | what was decided, what was found the hard way, what is still open. |
| [`../ihmc-alex-sdk/alex-ros2/alex_msgs/MOCAP_STREAMING.md`](../ihmc-alex-sdk/alex-ros2/alex_msgs/MOCAP_STREAMING.md) | **the publisher contract.** Topic, QoS, domain, message fields, provenance bytes, the labelling file, network. Hand this to whoever writes the Motive bridge. |

This file is the decision order. It links; it does not duplicate.

---

## 1. Before you order anything

Two things here are fabricated or physically committed, and neither is fixable in software.

### 1.1 The pelvis gauge bracket: **>= 182 mm spread**, not FRAMEWORK §1's 140 mm

Every capture's base pose comes from registering the pelvis cluster, so that cluster's angular
error `sigma / (sqrt(N) * r_perp)` multiplies the lever arm out to every other link. That term is
the dominant one in the whole pipeline, and **it does not average away with more captures** — the
reference shape comes from one capture.

Measured on Alex, K = 40, 3-seed means, held-out RMS (`RUNNING.md`, *Watch out for — the real Alex
model*, item 1):

| gauge spread | sigma = 0.93 mm | sigma = 0.30 mm | sigma = 0.10 mm |
|---|---|---|---|
| 60 mm | 20.44 | 6.56 | 2.18 |
| **140 mm** (§1's recommendation) | 8.87 | **2.86** | 0.95 |
| 200 mm | 6.34 | 2.04 | 0.68 |
| 300 mm | 4.44 | 1.43 | 0.48 |

```
held-out RMS  ~  2.86 mm * (sigma / 0.3 mm) * (140 mm / gauge spread)
```

The bar is TALOS's cross-validated **2.2 mm** (`FRAMEWORK.md` §15, `HeldOutResidualGate.TALOS_CROSS_VALIDATED_RMS_METERS = 2.2e-3`).

**So: >= 182 mm of spread at sigma = 0.3 mm, or sigma <= 0.23 mm at 140 mm.** A 140 mm bracket
locks in 2.86 mm on *synthetic data with a perfect URDF and nothing wrong but mocap noise*. Real
hardware only adds to that.

More captures is not the lever. Going from K = 30 to K = 480 — **16x the data** — bought
**2.04x** (`CLAUDE.md`, *Layout accuracy is NOT sigma/sqrt(K)*). Widen the gauge >> lower sigma >>
take more captures. Build the bracket wide, rigid, non-collinear, asymmetric, four markers, and
build it now, because it is the only item on this page with a fabrication queue in front of it.

Scattering limb markers is fine — noiseless layout recovery is exact either way, and scattering
costs ~3x in conditioning at equal K, bought back by 200 captures. **Do not scatter the pelvis.**
A scattered pelvis cluster has 0.148 m mean radius against a bracket's 0.165 m, and that difference
propagates to every link.

### 1.2 Which links get markers — you are choosing a mass fraction

With the pelvis and six leg links marked (7 marked, 23 chained), **53.493 of 91.513 kg = 58.45 %
of Alex is posed by forward kinematics, not by markers.** That mass rides on encoder values and on
the URDF, not on the measurement.

| decision | FK-posed fraction |
|---|---|
| legs only (pelvis + 6 leg links) | **58.45 %** |
| legs + one torso cluster | **34.18 %** |

`TORSO_LINK` alone is **22.21 kg (24.3 %)**, chained off the pelvis through one `SPINE_Z` joint —
the joint `FRAMEWORK.md` §1 explicitly refuses to make the gauge, because under suspension the
spine carries the full load in tension with off-axis deflection the URDF does not model. The
24.996 kg above the torso stays chained either way, but a torso cluster moves it off that one
unmodelled joint. **It is the highest-leverage marker decision after the gauge itself.**

Order enough markers and brackets for the torso even if you are not sure yet.

### 1.3 Identifiability has to be designed in. Nothing downstream checks it.

**Pelvis + both `HIP_X_LINK` is exactly degenerate**, and every indicator you would look at is
green:

| indicator | value on the degenerate set |
|---|---|
| in-sample RMS | **0.112 mm** — on data with *no noise in it* |
| `J` at the 200-iteration cap | 3.02e-6 m^2 |
| exit code | 0 |
| G2 | **PASS** |
| `CalibrationResult.isFullySolved()` | **true** |
| actual layout error | **57.3 mm** — every marker displaced by the same 55.9 mm vector along x |

The cause: `PELVIS_LINK` does not rotate relative to the base, and both `*_HIP_X_LINK` rotate about
the *same* `(1 0 0)` axis, so a translation along x is an exact symmetry of `J`.

`sigma_3` does not separate it: **8.79e-4 m^2** for the degenerate set against **2.21e-3 m^2** for
the identifiable pelvis+thighs set — a factor of 2.5 — where the factor separating two sets that
are *both* identifiable is **26**. There is no threshold on `sigma_3` that works.

> **Open gap.** There is no code in `src/main/java` that checks marker-set identifiability.
> `isFullySolved()` only asks whether every marker got a finite position. Nothing computes the
> rank of the symmetry group of the marked set. Until something does, **this is a human check on a
> whiteboard before the markers are ordered.**

The rule of thumb that holds on Alex: the hip is X-then-Z-then-Y, so pelvis + a *single* thigh
already pins `Delta`. Marking a full leg is safe. Marking a symmetric pair of links that rotate
about parallel axes is not.

---

## 2. Before markers go on the robot

### 2.1 Marker names are load-bearing

Cluster inference takes **everything before the LAST underscore**, and that prefix must be the
**exact URDF link name** (`CalibrationRunner.inferClusters`).

| marker name | inferred cluster | outcome |
|---|---|---|
| `PELVIS_LINK_M0` | `PELVIS_LINK` | correct |
| `PELVIS_M0` | `PELVIS` | **not a link — the run fails at startup** |
| `PELVIS` (no underscore) | — | throws: "has no `<cluster>_<n>` prefix" |

Failing at startup is the good case. Alex's leg links are `*_HIP_X_LINK`, `*_HIP_Z_LINK`,
`*_THIGH`, `*_SHIN`, `*_ANKLE_Y_LINK`, `*_FOOT` — note **`THIGH`/`SHIN`/`FOOT` have no `_LINK`
suffix**. Root is `PELVIS_LINK`.

`--cluster <name>=<m1,m2,...>` overrides inference when the naming does not match the mounting.
Use it rather than renaming markers to fit a convention.

**>= 3 markers per cluster (`MarkerCluster.MINIMUM_MARKERS`), 4 recommended
(`RECOMMENDED_MINIMUM_MARKERS`).** Three is the algebraic minimum for a pose; the fourth is what
G1 checks and what survives an occlusion. Four is not "three plus a spare": at a 12 % per-marker
drop rate over a seven-cluster leg set, **126 of 200 frames are refused — 63 %**, against the 41 %
naive arithmetic predicts, because losing one of four also removes a quarter of the constellation's
spread and trips `LinkPoseEstimator`'s `DEFAULT_SIGMA2_FRACTION = 0.25` guard. **Five on the limbs
buys more than the count suggests.**

The labelling file (motive id -> marker name) is the publisher's deliverable. Format and rules:
see [MOCAP_STREAMING.md](../ihmc-alex-sdk/alex-ros2/alex_msgs/MOCAP_STREAMING.md), *The labelling
file*.

### 2.2 Marker-to-link assignment is taken on trust — check it physically, once

G1 catches a label swap **within** a cluster, because two markers trading places changes their
inter-marker distances. G1 **cannot** catch a marker that is physically on the thigh and labelled
as a shank marker: the cluster is still rigid, the distances are still constant, the calibration is
still clean — of the wrong thing. `FRAMEWORK.md` §21.5 lists this as an open assumption.

**This is a five-minute job with two people and the labelling file, at the moment the markers go
on, and it can never be done as cheaply again.** Read the list aloud, point at the robot.

While you are there: `NatNetMocapSource.getUnfedMarkers()` must be empty at startup. A marker no
Motive id feeds can never become visible, so its cluster can never reach three.

### 2.3 Mount rigidity is what G1 measures, and it has a floor

At sigma = 0.3 mm, G1's detection threshold is about **1.7 mm for a slip** (a step of amplitude
`a` has std `a/2`) and about **3.1 mm for a creep** (the same total travel spread linearly has std
`a/sqrt(12)`). A green G1 does *not* rule out a slowly loosening mount below ~3 mm of travel. Bolt
things down; do not tape the gauge.

---

## 3. Before the first capture

### 3.1 Measure `sigma`. There is no default anywhere, on purpose.

`--sigma <metres>` is required by `CalibrationRunner` in both modes and has no default. The wand
residual (~0.93 mm, averaged over the whole lab) **is not a substitute** — every threshold
downstream is expressed in terms of the per-axis noise at the gantry.

Get it the way `FRAMEWORK.md` §20.1 asks: log 60 s of static markers at the gantry, take the
per-axis standard deviation of each reconstructed position, and check the anisotropy — 2–3x is
normal and the weak axis will point at the missing camera side. That same 60 s doubles as the
NatNet smoke test in `RUNNING.md`, *Watch out for — `mocap`*.

Do §20.2 first if you can: move the mid-rectangle cameras to converge on the gantry, recalibrate a
tight ~2x2x2.5 m volume, target < 0.3 mm, then repeat the static test. Every 0.1 mm off `sigma`
is a proportional 0.95 mm off the held-out RMS at a 140 mm bracket.

### 3.2 Measure the world tilt. `TiltMeasurement` has no silent default by design.

Unmeasured floor tilt is **systematic and ungated** — no gate in §15 looks at it, it does not
average out, and it lands directly in CoM height as `||c|| * sin(theta)`:

```
~7 mm of CoM height at theta = 0.5 deg, with ||c|| = 0.8 m,  against a 0.1 deg target
```

Admissible methods are exactly `TiltMeasurement.Method`:

| method | notes |
|---|---|
| `PLUMB_LINE` | a plumb line sighted against the mocap volume |
| `PRECISION_LEVEL` | a level on a surface with a known relation to the mocap frame |
| `STATIC_IMU_AVERAGE` | long accelerometer average, resolved into the mocap frame |
| `ASSUMED_LEVEL` | **forbidden for real work.** Exists so "nobody measured it" is representable and visible rather than being spelled `0.0`. Requires a written justification. |

Entered as `--world-tilt <degrees>` on both `CalibrationRunner` and `ReplayRunner`. Omit it and
`CalibrationRunner` records NaN and prints a warning; `ReplayRunner` constructs
`assumedLevel("no --world-tilt given to ReplayRunner")` and prints `ASSUMED_LEVEL` in its header.

> **Two limits of the CLI worth knowing before you rely on it.**
> `CalibrationRunner --world-tilt` stores the angle in provenance and nothing else — it never
> builds a `TiltMeasurement`, so the *method* is not recorded. `ReplayRunner --world-tilt <deg>`
> calls `fromTiltAngles(deg, 0.0, PRECISION_LEVEL, ...)`: it assumes the tilt is **about x only**
> and hard-codes the method to `PRECISION_LEVEL` regardless of how you actually measured it.
> A 0.5 deg tilt toward +x and one toward +y give the same height error and completely different
> horizontal CoM, and the scalar CLI cannot express the difference. **Record the method and the
> direction in `--note` and in the lab notebook**; the type carries a full unit vector even though
> the CLI does not.

### 3.3 Configure Motive: Z-up, metres, honest provenance

Motive defaults to **Y-up**. `alex-mocap` requires **Z-up**. A Y-up stream reconstructs a robot
lying on its side and *registers cleanly*, because the constellation is internally consistent
either way — nothing downstream can detect it. Fix it at `Streaming -> Up Axis -> Z-Up`, then hold
a marker at a known height and echo it: `position.z` should be that height and `position.y` should
not. Ten seconds, catches the whole class.

Units are metres. Motive can be configured in millimetres. Check.

Marker provenance is a byte, not a boolean: `NOT_SEEN` / `OBSERVED` / `POINT_CLOUD_SOLVED` /
`MODEL_SOLVED` / `UNKNOWN`. **Do not launder a model-solved marker as observed** — it smuggles
Motive's own pose estimate back in through the marker channel, expressed in Motive's stored layout,
which is the exact thing calibration exists to replace. It also makes the residual *smaller* and
the conditioning *better* while the answer gets worse.

Full contract, bit definitions and the worked publisher:
[MOCAP_STREAMING.md](../ihmc-alex-sdk/alex-ros2/alex_msgs/MOCAP_STREAMING.md), traps 1 and 2. Do
not re-derive it here.

### 3.4 Share the timestamp epoch with the encoder stream

`timestamp_nanoseconds` must share an epoch with the robot's encoder stream. If it does not, the
reconstructed pose **silently lags** whatever it is compared against, and it reads as a phase error
in the estimator — an estimator bug, not a bookkeeping one. `FRAMEWORK.md` §18.3 lists this as one
of the three silent failures.

`CalibrationRunner` pairs mocap rows with encoder rows **by index** — the only thing two
independently written logs agree on — and prints the worst `|mocap - encoder|` skew. If no capture
carries both timestamps it prints `UNKNOWN` and says the pairing cannot be checked at all. **Read
that line.**

If Motive's timestamp cannot be resolved into the robot's epoch, publish the transmit time and say
so in the handoff, so the printed skew is known to be meaningless rather than assumed meaningful.

### 3.5 The network will fail silently until someone widens the whitelist

The consumer machine is on **ROS domain 42** with a **loopback-only** DDS interface whitelist. As
shipped, nothing published from another machine will ever arrive, and there is no error on either
side — DDS discovery simply never matches.

`~/.ihmc/jros2.properties` on the consumer, verified as of this writing:

```properties
jros2.fastdds.interface.whitelist= 127.0.0.1/8
jros2.ros.domain.id=42
jros2.fastdds.intraprocess.delivery=false
```

Note the properties file **wins over** `ROS_DOMAIN_ID`, so exporting it will not do what you
expect. Widening the whitelist is the consumer side's job and it is the first thing to check when
nothing arrives. Details, precedence order, Tailscale caveat and the `ros2 topic` checks:
[MOCAP_STREAMING.md](../ihmc-alex-sdk/alex-ros2/alex_msgs/MOCAP_STREAMING.md), *Network* and
*Verifying it works*.

**Prove the publisher and a subscriber talk on the publisher's own box first.** Conflating "the
message is wrong" with "the network is wrong" is how a day disappears.

---

## 4. Before trusting a number

### 4.1 Run G1 first, always

G1 consumes raw mocap: **no FK, no URDF, no encoders, no calibration.** It is therefore the only
gate whose failure unambiguously indicts the mocap and the mounting rather than the model. Every
other gate mixes in the URDF, so a failure there has several possible authors.

```bash
./gradlew installDist
./build/install/alex-mocap/bin/alex-mocap --gate g1 --input capture.csv --sigma 0.0003
```

Three things about reading it:

- **The real margin is 2.12x, not 3x.** A perfectly rigid pair still shows `sqrt(2)*sigma` of
  spread, so the default 3-sigma threshold sits `3/sqrt(2) = 2.12x` above the noise floor. Both are
  printed. Know this before tightening `--sigma-multiplier`.
- **`NOT_EVALUATED` is not `PASS`, and it exits 1.** A pair needs `--min-samples` (default 100)
  co-visible frames before it is judged. A cluster whose markers never appear together would
  otherwise produce the most confident possible green.
- Passing checks are summarised by count. Forty green rows is how a red one gets missed.

### 4.2 Record the capture — and check the drop count

Record with `MocapFrameRecorder` (CSV: one header comment, one schema row, one line per frame;
invisible markers are `NaN`, never `0.0`).

**`NatNetMocapSource` keeps only the latest frame and drops the rest on purpose.** That is correct
for a control loop and **wrong for logging**. After any capture you intend to calibrate from:

| accessor | what it must show |
|---|---|
| `getFramesReceived()` | ~ duration x Motive rate. Materially short = packets dropped on the wire. |
| **`getDroppedFrameCount()`** | **0.** Non-zero while recording means the recorder is not keeping up. |
| `getUnlabelledMarkerCount()` | steady and small. Comparable to your labelled count = the ids are wrong. |
| `getUnfedMarkers()` | empty. |

A log with holes still produces a confident calibration — just from fewer captures than you think.

Write the read loop as `while (!source.isFinished()) { if (source.read(frame)) ... }`. The shorter
`while (source.read(frame))` works perfectly on a replay and exits on the first dropped packet
against live capture.

### 4.3 Calibrate

```bash
./build/install/alex-mocap/bin/alex-mocap \
    --calibrate \
    --input      capture.csv \
    --encoders   encoders.csv \
    --urdf       robot.urdf \
    --sigma      0.0003 \
    --world-tilt 0.08 \
    --gauge      PELVIS_LINK \
    --note       "plumb line, 2 people, 0.08 deg toward +x" \
    --output     calibration.json
```

`--gauge` defaults to the URDF root link, which is what `Delta = ^c T_b` is defined against. On
Alex that is `PELVIS_LINK`, which is what you want — `FRAMEWORK.md` §1 forbids the torso as gauge.

Exit codes: `0` every gate passed, `1` a gate failed **or could not be fully evaluated**, `2` usage
or I/O error.

### 4.4 Read the `CalibrationReport` in this order

1. **`skew`** — worst `|mocap - encoder|`, or `UNKNOWN`. See 3.4.
2. **`iterations ... (converged)`** — read `isConverged()`, **not `sigma_3`**. At a 5 % excursion
   fraction the base step's `sigma_3` falls only 7.6x while the layout error goes from 0.31 mm to
   **34.96 mm**. The conditioning number moves by less than an order of magnitude for a
   hundredfold loss of accuracy.
3. **The per-marker residual column, not the summary RMS.** On the demo it climbs 0.41 mm at the
   pelvis to 2.79 mm at the foot on data whose marker noise is 0.3 mm everywhere. That gradient is
   the gauge cluster's angular error times the lever arm — it is section 1.1, showing up as a
   number. On Alex it is worse than the toy for a purely geometric reason: pelvis to foot is 0.89 m
   here against ~0.6 m there.
4. **In-sample RMS is not an accuracy claim, and the gap is large.** Measured on the scattered
   Alex set: in-sample **4.47 mm** for a case whose actual layout error is **0.76 mm** — a factor
   of six. It carries the base-pose fit residual too. `CalibrationReport` says so in the label.
5. **Base position and rotation separately.** At sigma = 0.3 mm, K = 30: base position error
   0.84 mm, base **rotation** error 7.63 mrad (0.437 deg). The rotation is the one that matters —
   0.44 deg at the pelvis is **6.8 mm at a foot 0.89 m away** — and it is also, unchanged, the
   frame-to-frame noise on the runtime pelvis orientation F10 hands to the EKF comparison.

### 4.5 Replay

```bash
java -cp "build/install/alex-mocap/lib/*" us.ihmc.alexMocap.ReplayRunner \
    --input capture.csv --encoders encoders.csv --urdf robot.urdf \
    --calibration calibration.json --output-directory out/ \
    --world-tilt 0.08 --sigma 0.0003 --rate 200 \
    --velocity --error-budget
```

`ReplayRunner` has **no start script** — `applicationName = "alex-mocap"` is bound to
`CalibrationRunner` alone, so it is launched by class name off the installed lib directory.
`--visualize` needs a display; `--mesh-dir <ihmc-alex-sdk/alex-models>` is needed for `package://`
meshes, which do **not** live beside the URDF. `--velocity` is an offline second pass over the
completed log — a centred Savitzky-Golay window needs samples from the future — not a runtime
quantity.

Exit `1` means at least one frame was refused, and a refused frame has **no CoM at all** — not a
degraded one. A refused marked link also orphans its unmarked descendants: `KinematicChainCoupler`
binds each unmarked link to its nearest marked ancestor **once, at construction**, so refusing
`LEFT_SHIN` also refuses `LEFT_ANKLE_Y_LINK` — 6.39 kg leaves the sum, not 6.34. The CoM goes NaN
rather than becoming the CoM of a robot missing a shin. That is the conservative branch and the
right one.

Read the conditioning table's **visible-count histogram** per cluster. If clusters drop below 3
visible markers during leg sweeps, fix the mounting before anything else (`FRAMEWORK.md` §20.4).

### 4.6 Known-open items that will bite you, stated honestly

| item | status | what it means at 8am |
|---|---|---|
| **G2 on scattered markers** | **open.** 21 of 28 markers fail, every one with the verdict *"indicts nothing — isotropic, consistent with mocap noise"*. Observed back-projection spread 7.5 mm against an expected 2.1 mm, while the layout is recovered to ~1 mm. The **expected** spread is what is wrong: it has no cluster radius and no gauge lever arm in it. | A scattered run **exits 1** and is not necessarily bad. Read the layout-recovery table, which `AlexLegDemo` prints *before* the exit check for exactly this reason. Decision needed: fix the expected-spread model, or declare a scattered set outside §15's scope. |
| **G2 sensitivity** | A green G2 on Alex is not "no joint offset". At sigma = 0.3 mm a 0.5 deg `LEFT_HIP_Y` offset does not fire it — worst marker spreads 3.59 mm against 2.08 mm expected, ratio 1.7 against a 3-sigma bar. At sigma = 0.05 mm it fires hard. | G2's sensitivity scales with the **gauge cluster's angular accuracy**, not with capture count. Same lever as 1.1. |
| **G3 `VolumeDistortionGate`** | **empty placeholder — the file is 0 bytes.** | Needs a rigid two-marker artifact of known length, carried through the working volume. Nothing checks lens distortion or calibration extrapolation today. **Fabricate the artifact alongside the bracket** — it is the other hardware item with a lead time. |
| **G4 held-out** | The gate class exists and is tested, but **no CLI wires it in.** `CalibrationRunner`'s class javadoc claims it "reports G2 and G4"; the code adds only `BootstrapSpreadGate`, and the usage text correctly says G2 only. | To get a held-out number you currently need `AlexLegDemoTest`'s harness or your own. Do not read "gates passed" as "held-out RMS is under 2.2 mm". |
| **G4 interpretation** | It does **not** show the in-sample/held-out asymmetry `PR_PLAN` predicted. An i.i.d. split carries a systematic bias equally into both halves; held-out detects *overfitting*, and 90 parameters against 1680 observations have none. | **The absolute level is the signal, not the ratio.** A held-out RMS equal to in-sample is normal and says nothing reassuring. |
| **sigma_2 vs sigma_3** | **Unresolved.** The code refuses on `sigma_2`; `FRAMEWORK.md` §9/§18.1 say `sigma_3`. Coplanar clusters — "the realistic case, markers on a flat link face", per §2 itself — have `sigma_3 ~ 0` and are perfectly well posed: three non-collinear points fix 6 DOF. Refusing on `sigma_3` rejected **one frame in six** of a healthy replay (toy `l_shank`, nominal `sigma_3 = 3.1e-08 m^2`, 0.17 mm out of plane, *below* the 0.3 mm noise). | Needs an explicit decision: amend §9/§18.1, or revert the code. Until then, know which one your build refuses on before you interpret a refusal. |
| **URDF inertials** | CAD, assumed, and **the dominant error term.** F11 on real inertials: mass **4.90 mm** / link-CoM **2.73 mm** / mocap **0.164 mm**. CAD dominates 33x; perfect mocap would buy **1.00x**. | **Weigh the robot.** It is one number and it catches the fault the CoM cannot see — scaling every link mass by a constant leaves the CoM exactly unchanged. G5's total-mass check fires at 19 % against a 5 % bar on a 17.5 kg discrepancy. |

**The failure mode this project has:** every bug found here was a small residual with a wrong
answer, never a loud one. The gauge freedom at `J = 7.6e-29`. A 2 m local minimum reached
monotonically. A forgotten F8 correction biasing every CoM 7 mm low. **Be wary of any green number
not paired with a conditioning number.**

---

## 5. One capture nobody has taken: `delta`

`delta` is the disagreement between **Motive's asset marker layout** and **our calibrated layout**.
It is currently *estimated at ~2 mm and has never been measured*, and it sets how badly a
model-solved marker biases the pose. Registration stiffness goes as `N * r^2`, so one
correspondence displaced by `delta` biases the pose by

```
dtheta ~ delta / (N * r)          dt ~ delta / N
```

For a pelvis bracket with `N = 4`, `r ~ 70 mm`, `delta ~ 2 mm`: **~7 mrad, about 6 mm of CoM
error** through the body's lever arm. Larger than every other term in the error budget combined,
and **systematic**, so it does not average out.

**One capture settles it**: with the robot static and every marker `OBSERVED`, compare Motive's
asset layout for each cluster against the calibrated `^i p_ij` from `calibration.json`. Do it while
you are already at the gantry with a fresh calibration in hand. If `delta` turns out small, the
provenance discipline in section 3.3 relaxes; if it turns out large, you have quantified the single
biggest reason not to accept `MODEL_SOLVED` markers.

---

## 6. Bringing up the live stream

Only after sections 1–5. The offline path is the one that proves things; the live path adds
network, threading and Motive's own solver, and each of those hides the others.

1. **Publisher and subscriber on the publisher's own machine.** No network yet.
2. **`ros2 topic list | grep mocap_markers`, then `topic hz`, then `topic echo --once`,** all with
   `ROS_DOMAIN_ID=42`. Nothing on the wire = domain or whitelist, before anything else.
3. **Z-up check** — marker at a known height, `position.z` is it.
4. **Provenance check** — occlude one marker by hand. Its provenance must leave `OBSERVED`. If it
   stays `OBSERVED` with a plausible position, the params bitfield is being read wrong and the
   consumer cannot detect it downstream.
5. **Parallel sequences** — `motive_id`, `position`, `provenance` all the same length. Three
   different lengths means every frame is dropped, and counted.
6. **`frame_number` monotonic and gap-free** over a quiet minute. Gaps are the network; a frozen
   counter is Motive.
7. **Then wire `NatNetMocapSource.onFrameReceived(...)`.** Note the class does **not** open a
   socket and there is deliberately no `connect()` that throws — the seam is visible at the call
   site by design. The NatNet client itself is `us.ihmc.mocap`'s in ihmc-open-robotics-software and
   is not on this build's classpath.

Everything about the publisher side —QoS, dependency coordinate, allocation-free `add()`, the
`clear()` that is not optional, the 256-marker cap — is in
[MOCAP_STREAMING.md](../ihmc-alex-sdk/alex-ros2/alex_msgs/MOCAP_STREAMING.md). Hand that file over
whole.

---

## Checklist

**Before ordering (weeks of lead time)**

- [ ] Pelvis gauge bracket designed with **spread >= 182 mm** (or a written commitment to
      `sigma <= 0.23 mm` at 140 mm). 4 markers, non-collinear, asymmetric, rigid mount.
- [ ] Marked-link set chosen, with the resulting FK-posed mass fraction **written down**
      (58.45 % legs-only, 34.18 % with a torso cluster).
- [ ] Torso cluster ordered, or its absence justified in writing.
- [ ] Marked set checked for exact symmetries by hand. **No pair of marked links rotating about
      parallel axes with nothing else to break the freedom.** (Pelvis + both `HIP_X_LINK` = 57.3 mm
      wrong with every indicator green.)
- [ ] G3's rigid two-marker artifact of known length fabricated.
- [ ] 4 markers minimum per cluster ordered; 5 on the limbs if occlusion is expected.

**Before markers go on**

- [ ] Every marker name is `<EXACT_URDF_LINK_NAME>_<n>`. Checked against the URDF, not from memory.
- [ ] Labelling file (motive id -> name) written, ids unique, names unique.
- [ ] **Marker-to-link assignment verified physically, two people, list read aloud.**
- [ ] `getUnfedMarkers()` empty at startup.

**Before the first capture**

- [ ] Cameras converged on a tight ~2x2x2.5 m volume and recalibrated.
- [ ] **Per-axis `sigma` measured** at the gantry from 60 s static; anisotropy noted.
- [ ] **World tilt measured**, method one of `PLUMB_LINE` / `PRECISION_LEVEL` /
      `STATIC_IMU_AVERAGE`; magnitude **and direction** recorded in `--note` and the notebook.
      (0.5 deg = ~7 mm of CoM height at `||c|| = 0.8 m`.)
- [ ] Motive set to **Z-up** and **metres**; both verified by echoing a marker at a known height.
- [ ] Provenance byte verified by occluding a marker by hand.
- [ ] Timestamp epoch shared with the encoder stream, or the fallback documented in the handoff.
- [ ] Robot **weighed**. (CAD inertials dominate the error budget 33x.)
- [ ] Robot suspended from the gantry, harnessed at pelvis and torso, **not standing.**

**Before trusting a number**

- [ ] **G1 run first**, and passed — with no `NOT_EVALUATED` rows.
- [ ] `getDroppedFrameCount() == 0` on the capture being calibrated from.
- [ ] `skew` line read and not `UNKNOWN`.
- [ ] `isConverged()` true.
- [ ] Per-marker residual gradient read; base **rotation** error read separately from position.
- [ ] `--world-tilt` passed to both `CalibrationRunner` and `ReplayRunner`; neither report says
      `ASSUMED_LEVEL`.
- [ ] Visible-count histogram per cluster shows 4 visible through the whole leg sweep.
- [ ] Any G2 failure diagnosed against the scattered-marker open item before being believed.
- [ ] `delta` measured, or explicitly deferred in writing.

**Live stream**

- [ ] Publisher and subscriber proven on the publisher's own machine first.
- [ ] Domain 42 on both ends; consumer whitelist widened past `127.0.0.1/8`.
- [ ] `frame_number` monotonic and gap-free over a quiet minute.
