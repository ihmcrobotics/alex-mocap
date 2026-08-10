# PR4 — the pipeline on the real Alex model, leg-only markers

Branch `pr4-alex-demo`, worktree `/home/llibshutz/alex/alex-mocap-demo`. Local only, not
pushed, no PR opened. `./gradlew clean build --rerun-tasks` is green: **168 tests, 0
failures, 0 skipped, nothing disabled.** Three commits.

---

## What happened

It worked, and the Step 0 gate passed on the first path: `RobotModelHandle.fromURDF` loads
`alex_with_imus.urdf` **unmodified**, so no sanitizer was written and the URDF is vendored
byte-identically. On the real 91.5 kg model the solver behaves exactly as the toy said it
would — noiseless recovery is exact to 6e-16 m, `J` is monotone across every half-step, and
the CoM agrees with Mecano's own `CenterOfMassCalculator` to 1.1e-15 m. What changed is
everything downstream of *geometry*: Alex's pelvis-to-foot lever arm is 0.89 m against the
toy's ~0.6 m, and that single factor is what makes the headline result uncomfortable. **At
FRAMEWORK §1's recommended 120–150 mm gauge bracket and §17's 0.3 mm target noise, Alex's
held-out marker RMS is 2.86 mm — above the TALOS 2.2 mm bar §15 names as the target — on
synthetic data with a perfect URDF and nothing wrong but mocap noise.** The predicted hip-X
degeneracy reproduced exactly, including the part that matters (nothing detects it). And I
found a real spec problem in `RobotModelHandle`'s javadoc: SCS2 silently rewrites the URDF's
link frames, so the promise that a calibrated `^i p̂_ij` is CAD-comparable is false for
Alex's arms.

---

## The numbers

All measured, all reproducible from the tests, all stated in a javadoc next to the threshold
they set. Default seed unless noted.

| Quantity | Alex | Toy (PR2/PR3) | Note |
|---|---|---|---|
| Joints / links after merge | 29 / 30 | 6 / 7 | from 141 links, 140 joints, 111 fixed |
| Total mass | 91.512588 kg | 28.0 kg | preserved exactly by `simplifyKinematics` |
| Noiseless K=5: layout / base | 6.34e-16 m / 6.28e-16 m | 4.6e-15 m | exact |
| **Layout error, σ=0.3 mm, K=30** | **0.3140 mm** | 0.383 mm (RMS) | worst marker |
| Layout error, 10-seed range | 0.45 – 2.47 mm, median 0.76 | — | fat tail, see *wary of* |
| **Base pose error, position** | **0.8432 mm** | not asserted | worst over 30 captures |
| **Base pose error, rotation** | **7.6331 mrad = 0.437°** | not asserted | the one that matters |
| In-sample marker RMS, σ=0.3 mm | 1.97 mm (K=40) | 1.87 mm | per-marker 0.35 → 2.86 mm |
| Layout / base rot at σ=0.93 mm | 0.986 mm / 23.79 mrad | — | in-sample RMS 6.25 mm |
| **Held-out RMS, 140 mm bracket** | **2.856 mm** | — | **TALOS bar is 2.2 mm** |
| Held-out RMS, 300 mm bracket | 1.430 mm | — | passes |
| **Base-step σ₃, full sweep** | **4.864e-02 m²** | **9.892e-03 m²** | 4.9× — but see below |
| Base-step σ₃, 5% excursion | 6.421e-03 m² | — | only 7.6× down; layout 34.96 mm |
| **CoM vs Mecano** | **1.110e-15 m** over 8 captures | 1e-9 asserted | M agrees to 1e-9 |
| **Chained mass, 7 clusters** | **53.4925 kg = 58.45%** | 0 kg (all marked) | 23 of 30 links |
| Chained mass, + torso cluster | 31.2825 kg = 34.18% | — | one cluster |
| Chained mass, all 13 leg links | 47.2057 kg = 51.58% | — | stubs buy 6.3 kg |
| **F11: mass × lever arm** | **4.690 mm** | 4.098 mm | at 5% mass uncertainty |
| **F11: link-CoM error** | **2.727 mm** | 4.033 mm | at 5 mm/axis |
| **F11: pose error (mocap)** | **0.164 mm** | 0.242 mm | the only controlled term |
| F11: CAD/mocap ratio | **33×** | 17× | perfect mocap buys 1.00× |

**On the σ₃ comparison the plan asked for.** Alex's base-step σ₃ is 4.9× the toy's, but that
is not a like-for-like statement: σ₃ has units of m² and is a mean-squared spread of the
predicted-point cloud, so it scales with the *square of the robot*. As a length,
`√σ₃` = 0.220 m for Alex against 0.0995 m for the toy — a factor of 2.2 against a
robot-size ratio of roughly 1.5. So Alex is genuinely somewhat *better* conditioned than the
toy even after normalising for size, despite the narrower hip-X range (70° against 183°),
because hip-Y is 195° and the knee is 140°. **I would not read σ₃ across robots without that
normalisation, and I would not read it as a safety margin at all — see finding 3.**

---

## Step 0 — the gate

**Passed, first path.** `RobotModelHandle.fromURDF(alex_with_imus.urdf)` returned
`RobotModelHandle[rootBody, base=PELVIS_LINK, 29 joints, 30 links]`, total mass 91.512588,
and no link with a non-finite mass or non-finite `^i c_i`. No `URDFSanitizer` was written and
no equivalence test was needed. The URDF is vendored byte-identically to
`src/test/resources/us/ihmc/alexMocap/model/alex.urdf`; sha256
`e453edccdfbc6a86bdf123e43eec87e461484003fe37e8eb0ea1841f2c821fb3` matches the source.

None of the listed hazards blocked the load, and I checked each rather than inferring:

- **11 `<capsule>` collisions** — SCS2 has no `URDFCapsule`, but collision geometry is off the
  path from URDF to a Mecano tree.
- **70 links with no `<inertial>`, 19 with mass 0 and zero inertia** — every one of them sits
  below a `fixed` joint and is merged away before a `RigidBody` exists. The feared 0/0 → NaN
  in the merge does not occur. `testTheRealModelLoads` asserts finite, strictly positive mass
  and finite `^i c_i` for all 30 survivors as a tripwire against a future URDF edit.
- **8 `<mimic>` on `type="fixed"` joints, 17 `<gazebo>` blocks** — ignored by `URDFTools`.
- **17 missing hand meshes** — you get ~100 lines of
  `SDFTools: Unable to resolve the path: package://abilityHand/…` on stderr and nothing else.
  Cosmetic, no flag to silence it, noted in RUNNING.md so nobody chases it.

---

## Findings

### 1. SCS2 rewrites the URDF's link frames, and `RobotModelHandle`'s javadoc is wrong about it

Test 2 was supposed to pin the `transformToZUp` no-op *for the leg chain*, on the expectation
that it is not a no-op for the arms. Measurement said every one of the 30 links has an
**exactly identity** `^b T_i` rotation at `q = 0` — including the shoulders, which declare
`rpy="0.698132 0 0"`. That is not what a no-op looks like, so I chased it into SCS2's source.

`URDFTools.toRobotDefinition` calls `RobotDefinition.transformAllFramesToZUp()` — **default
`true`**, alongside the already-known `simplifyKinematics` — which recursively zeroes the
rotation of every joint's `transformToParent` and compensates by rotating, in place: the
joint axis, the inertia pose, the moment of inertia, every child joint's transform, and every
visual, collision, sensor and kinematic-point pose.

**The kinematics and the physics are exactly preserved.** I verified the composition by hand:
`^b T_LEFT_SHOULDER_X_LINK` comes out at `(0.002, 0.232, 0.469)`, which is the URDF's
`(0, 0.15676, -0.013)` offset rotated by `R_x(0.698132)` and added — correct. Nothing in
F1–F11 is numerically wrong, because `^i p_ij` is solved for and `^i c_i`, `^b T_i` and F9's
sum are all expressed in the same rewritten frames.

**What is not preserved is the identity of the link frame.** `RobotModelHandle`'s javadoc
says, as the stated practical payoff of choosing `getFrameAfterJoint()` over
`getBodyFixedFrame()`:

> The practical consequence: a calibrated `^i p̂_ij` printed by this pipeline is directly
> comparable to a CAD marker position, which is what makes a layout auditable by a human.

That is **false for any link below a joint with a non-zero `<origin rpy>`** — on Alex, every
link from the shoulders outward. `LEFT_SHOULDER_Y_LINK`'s `^i c_i` comes back as
`(-0.00264, +0.09735, +0.07277)` where the URDF's `<inertial><origin>` says
`(-0.00264, 0.12135, -0.006824)`. The difference is exactly `R_x(0.698132)`. The legs are
unaffected — all 12 leg joints declare `rpy="0 0 0"` — which is why this demonstration's own
numbers are safe.

`testLinkFramesAreBaseAlignedAtZeroBecauseScs2RewritesThem` pins both halves: the shoulder's
`^i c_i` must equal `R_x(0.698132)` applied to the URDF vector, and `LEFT_THIGH`'s must equal
the URDF vector unrotated. The toy 6-DOF URDF declares `rpy="0 0 0"` on all six joints, so no
existing test could have caught this.

I did **not** change the behaviour. `setTransformToZUp(false)` on `URDFParserProperties` would
turn it off and make `^i p̂_ij` CAD-comparable everywhere, but that is a change to what
FRAMEWORK §3 means by "the link frame", i.e. a specification decision. It is ranked first
under *what I would do next*.

### 2. A 140 mm gauge bracket does not clear the TALOS bar on Alex

This is the most actionable thing in the PR. Held-out marker RMS, 3-seed means, K = 40:

| gauge spread | σ = 0.93 mm | σ = 0.30 mm | σ = 0.10 mm |
|---|---|---|---|
| 60 mm | 20.44 | 6.56 | 2.18 |
| **140 mm** | 8.87 | **2.86** | 0.95 |
| 200 mm | 6.34 | 2.04 | 0.68 |
| 300 mm | 4.44 | 1.43 | 0.48 |

That is FRAMEWORK §1's `σ/(√N·r_perp)` to within a couple of percent in both variables:
linear in σ (0.93/0.30 = 3.10 against a measured 8.87/2.86 = 3.10) and inverse in the spread
(140/60 = 2.33 against 6.56/2.86 = 2.29). Collected:

```
held-out RMS  ≈  2.86 mm · (σ / 0.3 mm) · (140 mm / gauge spread)
```

**To clear 2.2 mm on Alex you need a bracket of at least ~182 mm at σ = 0.3 mm, or σ ≤ 0.23 mm
at 140 mm.** FRAMEWORK §1's "120–150 mm" recommendation has no lever arm in it; the toy sits
comfortably under the bar at 140 mm and Alex does not, purely because 0.89 m > 0.6 m. This is
a bracket, not a camera, and it is by a wide margin the cheapest accuracy in the project.

`testHeldOutResidualMissesTheTalosBarAtTheRecommendedBracketWidth` asserts the failure at
140 mm, the pass at 300 mm, and the `1/r_perp` exponent across a 5× range of widths.

### 3. The hip-X-only marked set is degenerate exactly as predicted — and nothing catches it

Prediction: `PELVIS_LINK` never rotates relative to the base, `*_HIP_X_LINK`'s orientation is
`R_x(q)` about an axis that is the same `(1 0 0)` on both sides, so `Δ → Δ·G` for a
translation `g` along `x` is an exact symmetry of `J`. **Confirmed, and quantitatively.**

At a 200-iteration cap, on **noiseless** data:

- `J = 3.02e-6 m²`, in-sample RMS **0.112 mm on data with no noise in it**
- layout error **57.345 mm**
- common displacement `(+0.055940, +0.000149, +0.000188)`, `|mean| = 55.941 mm`, worst
  per-marker deviation from it 1.584 mm (2.8%)
- the `(y, z)` components are **0.43% of the shift** — it is along `x` to within a fraction of
  a percent
- `isFullySolved()` is true, the CLI exits 0, and **G2 passes**

Uncapped at 500 iterations `J` falls to 2.7e-8 and the answer moves to 56.1 mm: `J` descends
geometrically forever while the answer does not move, because A′ is sliding down a flat
valley. The cap is explicit so the test is fast and deterministic.

**`σ₃` does not detect it, and I now think no threshold on `σ₃` could.** Base-step σ₃ is
8.79e-4 m² for the degenerate set against 2.21e-3 m² for the identifiable pelvis+thighs set
at the same K — a factor of 2.5. The factor separating two sets that are *both* identifiable
(pelvis+thighs vs the full 7 clusters) is **26**. So the degenerate/identifiable gap is an
order of magnitude *smaller* than the awkward/comfortable gap. This is PR2's toy finding
reproduced at Alex scale with the numbers to back the "no threshold exists" claim.

`AlexLegDemoCliTest.testTheDegenerateMarkedSetLooksPerfectlyHealthyFromTheCli` pins the
operational half: nothing an operator sees at the console distinguishes this from a good run.

### 4. `PELVIS_LINK` + both thighs alone is still identifiable on Alex — unlike the toy

The toy needed its feet marked, because with only the pelvis, thighs and shanks marked every
marked link's orientation was a rotation about `y` alone. Alex is not exposed to that at the
thigh: the chain runs `HIP_X`(x) → `HIP_Z`(z) → `HIP_Y`(y), so a thigh's orientation already
spans three independent axes. Measured noiseless at K = 5: layout 8.07e-16 m, base 8.67e-16 m,
267 iterations. The cost is in conditioning, not in the answer: σ₃ is 1.74e-3 m² against
4.59e-2 for the 7-cluster set — 26× less comfortable, and still exact.

### 5. G2 is floored on Alex at the target noise, and it fires on both branches

Two differences from the toy, both asserted as tests so they are documented by something that
runs:

- **At σ = 0.3 mm, a 0.5° `LEFT_HIP_Y` offset does not fire G2.** The worst affected marker
  (`LEFT_FOOT`) spreads 3.59 mm against 2.08 mm expected — ratio 1.7, under the 3σ threshold.
  Alex's gauge-driven floor at the target noise is larger than the fault. The *indictment
  column is still correct* (`LEFT_THIGH` names `LEFT_HIP_Y` at r = 0.60); only the verdict is
  green. **A green G2 at the gantry is not "no joint offset".** Same lever as finding 2: G2's
  sensitivity scales with the gauge cluster's angular accuracy, not with K.
- **At σ = 0.05 mm it fires hard, but on both branches.** `LEFT_SHIN` 1.47 mm /
  `RIGHT_SHIN` 0.76 mm, `LEFT_FOOT` 2.29 mm / `RIGHT_FOOT` 1.08 mm. G2 is handed the
  *solved* `Δ`, and A′ absorbs part of a one-branch fault into `Δ`, which is global. So the
  localisation claim that survives on a real robot is the **mirror comparison** (~2×), not
  "the other branch is clean", which is what the toy showed. RUNNING.md's advice to pass the
  solved `Δ` for diagnosis is still right; this is the price of it.

### 6. Base pose position and rotation must be reported separately

The toy's helper took the max of the two, which hid the interesting one. At σ = 0.3 mm,
K = 30: position 0.84 mm, **rotation 7.63 mrad = 0.437°**. The rotation is the number that
matters — 0.44° at the pelvis is 6.8 mm at a foot 0.89 m away — and it is *also*, unchanged,
the frame-to-frame noise on the runtime pelvis **orientation** that F10 hands to the EKF
comparison, because F6 is single-frame with no averaging (§9). If the EKF is being validated
for pelvis attitude, **0.44° is the ground truth's own noise floor at a 140 mm bracket.**
I have not seen that stated anywhere in FRAMEWORK.md, and it follows the same `1/r_perp` law
as everything else in finding 2.

### 7. Marking all 13 leg links is worse than marking 7

Layout error goes *up*, 0.3140 → 0.7332 mm, and chained mass only falls 53.49 → 47.21 kg. The
extra clusters sit on the hip and ankle stubs, at essentially zero lever arm from their
parent: they add 24 markers whose own layouts must be recovered while contributing almost no
information about `Δ`. FRAMEWORK §1's "descending order of mass × lever arm" is doing real
work here — the stubs are 2.33, 0.76 and **0.050** kg.

---

## What to be wary of

- **The 3σ-style thresholds are all ~3× a measured value, and every measured value is in a
  comment next to it.** None was chosen to make a test pass. But three deserve flagging:
  - `testRecoveryAtTheTargetNoise`'s 1.0 mm layout bound is 3.2× the default seed's 0.3140 mm,
    and the **10-seed range of that statistic is 0.45–2.47 mm**. It is a max over 28 markers
    and its tail is RUNNING.md's "reference-shape noise does not average at all" —
    `BaseInitializer` bakes capture 0's marker noise into the gauge shape for the whole
    session, and seeds 7 and 9 draw a bad capture 0 (24 and 20 mrad of base rotation against a
    typical 8). **Changing the seed there without re-measuring will look like a regression.**
    I chose a fixed seed + tight bound over a loose bound, per PR_PLAN's rule, but it is the
    most seed-sensitive assertion in the file.
  - `testHeldOutResidualMissesTheTalosBarAtTheRecommendedBracketWidth` averages three seeds
    rather than using one. A single seed put the 140 mm case at 2.2004 mm against a 2.2000 mm
    bar — a 0.02% margin, i.e. a coin flip dressed as an assertion. The 3-seed mean is 2.856,
    a 30% margin. **Still not a large margin**, and it is the assertion I would expect to be
    the first to move if anything upstream changes.
  - `testHipXOnlyMarkedSetIsDegenerateAlongX` asserts the per-marker deviation from the common
    shift is under 10% of it; measured 2.8% at the 200-iteration cap. That number is a
    function of the cap (0.29% at 500), so it is a property of how far A′ got, not a physical
    constant.
- **`AlexLegDemoTest` reads its inertial ground truth from hardcoded URDF values** in
  `testLinkFramesAreBaseAlignedAtZeroBecauseScs2RewritesThem` (`-0.00264, 0.12135, -0.006824`
  and the thigh's). They are pinned by the vendored file, but if the URDF is re-vendored those
  three lines must be re-read from it.
- **The F11 budget uses `--mass-uncertainty 5%` and 5 mm/axis link-CoM uncertainty**, which are
  `ReplayRunner`'s defaults and are **assumed, not measured**. Nobody has weighed the robot.
  The *ratio* (CAD 33× mocap) is robust to those choices — at 2% and 2 mm it is still 15× —
  but the absolute 5.4 mm is not a measurement of anything.
- **Nothing here measures the real robot.** The URDF is assumed correct exactly as §3 assumes
  it. What is now real is the geometry, the lever arms, the joint ranges and the inertials —
  and those are what set every number above. G2 and G4 on real captures remain the only things
  that can test the URDF itself.
- **Nothing was disabled, skipped, or deleted.** All 168 tests run and pass.
- I did not modify `SyntheticCaptures`, `CalibrationRunner`, `ReplayRunner`, or any existing
  test. Two files outside the new test code changed: `RobotModelHandle` gained four
  joint-limit accessors (additive), and `build.gradle.kts` gained the `-Dalex.demo.verbose`
  forwarding — Gradle's `Test` task does not inherit system properties, so without it the
  documented flag was accepted and silently did nothing.

---

## What I would do next, ranked

1. **Decide on `setTransformToZUp(false)`.** One line in `URDFLoader`. It would make
   `^i p̂_ij` CAD-comparable on every link, restoring the promise `RobotModelHandle`'s javadoc
   already makes, and it changes nothing numerically for the legs (all `rpy="0 0 0"`). The
   risk is that it changes what `^b T_i` means for the arms in any *existing* saved
   calibration, and it is a §3 specification decision. **This is a 10-minute change and a
   30-minute conversation, and it is the only correctness-adjacent item on this list.**
2. **Build a ≥ 180 mm pelvis outrigger bracket** before any gantry session. Finding 2 makes
   this a number rather than a preference: at 140 mm and σ = 0.3 mm the pipeline's own noise
   floor eats the entire TALOS budget with a perfect model, so a real-robot residual would be
   uninterpretable — you could not tell model error from bracket geometry. 200 mm gets to
   2.04 mm, 300 mm to 1.43 mm. Nothing else on this list buys as much for as little.
3. **Add a `TORSO_LINK` cluster to the marker plan.** It takes chained mass from 58.45% to
   34.18% and, more importantly, shortens the error path for the 24.996 kg above it from
   "pelvis, through the unmodelled `SPINE_Z`" to "measured torso". FRAMEWORK §1 already lists
   TORSO as marked-but-not-gauge; this quantifies why.
4. **Weigh the robot.** §14 and §21.1 both say so, it takes minutes, and finding 6's table
   says the CAD terms are 33× the mocap term. Every hour spent on mocap accuracy is spent on
   the 3% term until this is done.
5. **Write a G5 that tests for gauge degeneracy directly.** Finding 3 shows `σ₃` cannot, and
   the failure is silent all the way through the CLI. The test is cheap and structural, not
   statistical: for the marked set, form the per-capture rotations `R_i(q^(k))` of every
   marked link, stack `(R_i(q^(k)) - R_i(q^(0)))` over all `(i, k)`, and check its null space
   is empty. A non-empty null space is exactly the set of free translation directions `g`, and
   it is computable **before any data is captured** — from the URDF and the intended sweep
   alone. That makes it a marker-plan gate, which is where it belongs.
6. **Measure the per-axis σ at the gantry** (§20.1). Every number in this report is
   parameterised by it and none of them is a prediction until it is measured. The anisotropy
   matters too: finding 2's law is written in a scalar σ, and a 2–3× weak axis pointing at the
   missing camera side will not behave like its average.
7. **Implement the two-stage A′** that RUNNING.md describes and PR2 deliberately left out: run
   A′ to convergence, rebuild the gauge cluster's shape from the *converged* pelvis layout
   (averaged over all K, so `√K` quieter than any single capture), then run A′ again with that
   new fixed `^W T_c^(k)`. Monotonicity survives because each run minimises its own fixed
   objective. On the toy this was worth doing; on Alex it is worth more, because the
   reference-shape term is what produces the 0.45–2.47 mm fat tail in the layout error and it
   enters multiplied by a 0.89 m lever arm. **I have not measured what it would buy on Alex**
   — the honest expectation is that it mostly removes the tail rather than the median, since
   the per-capture gauge noise is a separate term and does average. It is still a
   specification change to §8, which is why it is below the hardware items.
