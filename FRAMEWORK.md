# FRAMEWORK.md

Whole-body CoM ground truth from OptiTrack, for validating a Kalman-filter pelvis
pose and velocity estimator. Rig-free: no jig, no zeroing fixture, no manual
landmark identification. Markers are placed by hand and their positions are
never measured by hand.

This document is the authoritative specification. Where it disagrees with any
diagram, prose summary, or comment in the code, this document wins.

---

## 0. Notation

| Symbol | Meaning |
|---|---|
| `q^(k)` | encoder vector at capture `k` |
| `W` | mocap world frame (as reported by Motive) |
| `Wg` | gravity-aligned world frame (see F8) |
| `b` | URDF base link frame — the pelvis link |
| `c` | Motive marker-cluster frame for the pelvis cluster |
| `i` | link index |
| `iT_j` | homogeneous transform taking a point in frame `j` to frame `i` |
| `^i p_ij` | position of marker `j` in the frame of link `i` — **unknown, constant** |
| `^W m_ijk` | measured world position of marker `j` on link `i` at capture `k` |
| `m_i`, `^i c_i` | mass and link-frame CoM of link `i`, from URDF inertial blocks |
| `M` | total mass, `sum_i m_i` |
| `K` | number of calibration captures |
| `Δ` | the single global unknown `^c T_b` (cluster frame to URDF pelvis link frame) |

Every quantity below is tagged **measured**, **chosen**, **derived**, or
**assumed**. Do not silently promote an assumed quantity to measured.

### Observation model

Everything in F3–F5 inverts this equation; F6–F9 consume its solution.

```
^W m_ijk  =  ^W T_b^(k) · ^b T_i(q^(k)) · ^i p_ij  +  ε_ijk
```

with `ε_ijk` zero-mean mocap noise.

### The apparent circularity, and why it is not one

Two distinct objects are both called "the pose of link i". Conflating them is
the source of the objection *"I need the calibration to get link poses, but I
need link poses to calibrate."*

| Object | Source | Answers | Role |
|---|---|---|---|
| `^W T_i(q)` | URDF + encoders (FK) | where the *model* says the link is | calibration-time reference |
| `^W T̂_i` | marker cluster + Umeyama | where the link *actually* is | runtime ground truth |

`^b T_i(q^(k))` is a function of joint angles alone. It contains no mocap and
does not depend on where the robot is in the room. The calibration consumes the
first object and produces the second. They are compared, never substituted.

---

## 1. Capture setup — hanging, not standing

**The robot is suspended from the gantry, harnessed at pelvis and torso. It does
not stand on its feet.** This is a locked decision, not a convenience.

Reasons, in order of force:

1. **Standing closes a kinematic loop through the ground.** That blocks the wide
   joint excursion F5 needs for identifiability and G2 needs for diagnosis.
2. **Standing gravity-loads the legs**, injecting elastic deflection the URDF
   cannot represent, directly into the calibration.
3. **Standing does not fix pelvis z/roll/pitch.** An earlier draft claimed the
   ground plane constrains three base DOF for free. For a humanoid this is
   false: those DOF become FK-derived *through the very leg joints being
   calibrated*. Do not reintroduce this argument.

**The gantry is not a zeroing rig.** No gantry dimension enters any formula. The
only requirement on it is rigidity, not knowledge of its geometry.

### Marker clusters

| Cluster | Role |
|---|---|
| `PELVIS` | **gauge cluster.** FK root, IMU location. Reading it costs zero joints. |
| `TORSO` | marked but not gauge. Makes the spine observable (6 measured DOF vs 1 URDF DOF = 5 redundant). Runtime fallback if the pelvis cluster occludes. |
| thighs, shanks | mark in descending order of `mass × lever arm` |

Torso must **not** be the gauge. Under suspension the spine joint carries the
full load in tension with off-axis deflection the URDF does not model; making
torso the gauge chains every link pose through that error.

Cluster geometry: ≥4 markers (3 is the pose minimum, the 4th feeds G1),
non-collinear, asymmetric to prevent label aliasing, maximum practical spread.
Prefer an outrigger bracket taking pelvis marker spread to 120–150 mm — cluster
angular accuracy scales as `σ / (√N · r_perp)`.

---

## 2. The registration primitive

Given `L` correspondences between source points `{a_l}` and target points
`{b_l}`, the least-squares pose minimising `sum_l ||b_l - (R a_l + t)||²` over
`R ∈ SO(3), t ∈ R³` is closed form. With centroids `ā`, `b̄`:

```
H = (1/L) · sum_l (b_l - b̄)(a_l - ā)^T  =  U Σ V^T

R = U · diag(1, 1, det(U V^T)) · V^T

t = b̄ - R ā
```

**Umeyama, not Arun.** Decisive reason: the determinant factor. Without it a
noisy or near-planar cluster — the realistic case, markers on a flat link face —
can drive the raw `U V^T` to `det = -1`, i.e. a reflection. That is not a
rotation, and downstream code will propagate it without complaint. Cost of the
fix is one determinant.

**Normalise `H` by `L` before decomposing.** Scaling by a positive constant
leaves `U` and `V` untouched, so `R` and `t` are unchanged, but it makes the
singular values mean-squared spreads with units of length², comparable across
frames with different numbers of visible markers. Without this, `σ₃` drops when
a marker occludes for reasons that have nothing to do with geometry.

**Sort the singular values explicitly.** EJML does not order them. Without an
explicit descending sort, `σ₃` is whichever singular value landed third and the
conditioning monitor is noise.

This one primitive is consumed by F5, F6, G1, and G4. There must be exactly one
implementation.

---

## 3. F1 — FK reference

`^b T_i(q)` from the URDF inertial and kinematic blocks, evaluated by Mecano.
Set joint angles, update frames, read `RigidBodyBasics.getBodyFixedFrame()`.

There is no algorithm to write here. Status: **assumed** — the URDF is trusted
to a reasonable degree for joint offsets and link geometry. This assumption is
what G2 tests.

---

## 4. F2 — base initialisation

In the hanging configuration the base pose is unknown per capture. Initialise
`Δ = I`, so `^W T_b^(0) = ^W T_c^(0)`, the raw pelvis cluster pose from F6 with
an arbitrary cluster-frame convention.

Status: **chosen.** F5 absorbs the error; this only has to be close enough to
start the loop.

---

## 5. F3 — single-capture bootstrap ("software T-pose")

Pick one capture, `k = 0`. With `Δ = I` from F2, back-project every visible
marker into its link frame:

```
^i p_ij^(0)  =  ( ^W T_c^(0) · ^b T_i(q^(0)) )^-1 · ^W m_ij0
```

This is the initial layout. It has no averaging and inherits the full arbitrary
offset of the cluster frame, which is exactly what F5 then solves for.

Status: **derived**, exact given the model. Requires only one capture.

---

## 6. F4 — marker step

Given per-capture base poses, the layouts are the unweighted mean of
back-projections:

```
^i p̂_ij  =  (1 / K_ij) · sum_k  ( ^W T_i^(k) )^-1 · ^W m_ijk
```

where `K_ij` counts the captures in which marker `j` on link `i` was visible,
and `^W T_i^(k) = ^W T_c^(k) · Δ · ^b T_i(q^(k))`.

**Derivation.** The normal equations of `min_p sum_k ||R^(k) p + t^(k) - m||²`
are `(sum_k R^(k)T R^(k)) p̂ = sum_k R^(k)T (m - t^(k))`. Each `R^(k)` is a
rotation, so `R^T R = I`, the Gram matrix collapses to `K·I`, and the estimator
is a plain mean. No iteration, no initial guess, no local minimum.

**The property that matters.** Averaging annihilates zero-mean mocap noise at
`σ/√K` — 0.93 mm at K=30 gives 0.17 mm. It does **not** touch a systematic error
in `^W T_i^(k)`: a joint offset, wrong link length, or gravity sag is correlated
with configuration and survives the mean as a bias. More captures do not help.
Pose diversity lets you *detect* the bias (G2) but never *correct* it.

Status: **derived**, exact given the base poses.

---

## 7. F5 — base step

Given layouts, recover the base pose. In the hanging configuration the base pose
is not one unknown per capture plus a fixture — it is the measured pelvis cluster
pose per capture, times **one global constant** `Δ`.

```
^W T_b^(k)  =  ^W T_c^(k) · Δ
```

`^W T_c^(k)` comes from F6 applied to the pelvis cluster. `Δ = ^c T_b` is the
fixed offset from the Motive cluster frame to the URDF pelvis link frame.

**Solving for Δ is a single Procrustes.** For every visible `(i, j, k)` define

```
a_ijk = ^b T_i(q^(k)) · ^i p_ij          (predicted point, base frame)
b_ijk = ( ^W T_c^(k) )^-1 · ^W m_ijk     (measurement pulled into cluster frame)
```

The model says `b_ijk = Δ · a_ijk`. Stack all `(i,j,k)` and apply §2. One
registration, all captures and all links at once.

Status: **derived**. Note that pelvis-cluster markers contribute nothing to `Δ`
beyond a constant — the information comes from the marked links *below* the
pelvis, which is why leg marking and wide joint excursion matter.

---

## 8. Approach A′ — the alternating loop

```
F3  →  initial layouts
repeat:
    F5  →  Δ  given layouts
    F4  →  layouts given Δ
until converged
```

Objective, shared by both steps:

```
J = sum_{i,j,k}  || ^W m_ijk  -  ^W T_c^(k) · Δ · ^b T_i(q^(k)) · ^i p_ij ||²
```

**Why this and not bundle adjustment.** Each step is the exact global minimum of
its subproblem, so `J` is monotonically non-increasing by construction. About
twenty lines, no Jacobians, no line search, no manifold parameterisation, no
gauge freedom to fix. The base pose is the only quantity not trusted; joint
offsets, link geometry, and inertials are trusted to a reasonable degree, and
promoting them to unknowns buys nothing while costing all of the above.

**Convergence.** Stop when the relative decrease in `J` falls below `1e-9`, or
after 50 iterations. Assert monotonicity every iteration in a test — it is the
property that justifies the whole choice.

**Escalation to Approach B (full nonlinear bundle adjustment over base poses,
joint offsets, and geometry) is not speculative.** Escalate only if G2 fires
*with structure*:

| G2 spread correlates with | Indicts | Response |
|---|---|---|
| a particular joint's excursion | that joint's offset, or link geometry below it | promote `δ` for that joint |
| limb load | joint elasticity under gravity | promote elastic parameters |
| nothing (spread is isotropic, matches σ) | nothing — this is mocap noise | A′ is sufficient |

---

## 9. F6 — runtime cluster-to-link pose

Per marked link, per frame: apply §2 between the calibrated layout `{^i p̂_ij}`
and the live measurements `{^W m_ij}`. Yields `^W T̂_i`.

Encoders are **not** used here. This is the whole point.

**F6 is single-frame with no averaging.** Offline calibration divides mocap noise
by `√K`; F6 does not. `σ` lands undiluted on every runtime pose.

**Log `σ₃` and the visible marker count on every frame, for every cluster.**
Rank deficiency is silent: with fewer than 3 non-collinear visible markers the
SVD still returns a well-formed rotation. Nothing downstream can detect this from
the transform alone.

---

## 10. F7 — FK chaining for unmarked links

For a link with no cluster, walk to the nearest marked ancestor `p`:

```
^W T̂_i  =  ^W T̂_p · ^p T_i(q)
```

This reintroduces encoder dependence for that link. It is a deliberate trade:
mark links in descending order of `mass × lever arm`, and accept FK chaining for
the light, close-in ones where the CoM contribution is small.

---

## 11. F8 — world-frame gravity alignment

Motive's world frame is level only to the accuracy of the ground-plane
calibration, and its residual tilt is **systematic and ungated**. A tilt `θ` puts

```
error  =  ||c|| · sin(θ)
```

into the CoM height — roughly **7 mm at θ = 0.5° with ||c|| = 0.8 m**. Target:
`θ < 0.1°`.

`θ` must be **measured** (plumb line, precision level, or a long static IMU
average), never assumed. It is a capture-session constant.

**Implement F8 as a `ReferenceFrame` node, not as a correction function.** Define
`Wg` as the parent of Motive's `W` with a fixed transform. Anything expressed in
`Wg` is then correct by construction, and a frame mismatch is a thrown exception
rather than a silent bias. A correction applied at call sites can be forgotten at
call sites.

---

## 12. F9 — whole-body CoM

```
^Wg c  =  (1/M) · sum_i  m_i · ^Wg T̂_i · ^i c_i          M = sum_i m_i
```

with `m_i` and `^i c_i` from the URDF inertial blocks (**assumed** — see F11).

Mecano's `CenterOfMassCalculator` implements this. Do not reimplement it; feed it
the measured poses.

---

## 13. F10 — pelvis state for EKF comparison

**Pose** is `^Wg T̂_b` directly from F6 on the pelvis cluster. Available per frame.

**Velocity is not a runtime quantity.** Raw single differencing at 200 Hz with
`σ = 0.93 mm` gives roughly

```
√2 · σ / Δt  ≈  0.13 m/s
```

which is unusable — it exceeds the ContactNet baselines it is meant to validate
(0.0844 and 0.0254 m/s) by a wide margin.

**Use a centred Savitzky–Golay differentiator, offline, over the logged pose
trajectory.** A 0.1 s centred window gives roughly 0.0037 m/s. Centred is
**zero-lag by construction** — the coefficients are antisymmetric about the
centre sample. This matters more than the noise reduction: lag against an
estimator being validated for velocity is a *systematic comparison error* that
reads exactly like estimator bias.

A centred window cannot execute causally. Therefore `PelvisStateExtractor` (the
runtime class) **must expose no velocity accessor at all.** If it does, someone
will call it and compare 0.13 m/s noise against a 0.025 m/s estimator.

### The three-pelvis-frames hazard

Three distinct frames are all called "pelvis":

1. the Motive marker-cluster frame (`c`)
2. the URDF pelvis link frame (`b`)
3. the IMU mounting frame

Calibration gives `c → b` for free — that is `Δ`. If the EKF reports velocity in
the IMU frame, F10 additionally needs the fixed `b → imu_link` transform, which
is an **unverified CAD number**. Confirm which frame the EKF publishes, and
verify `r` against the physical mounting.

The cost of getting this wrong: the `ω × r` term at `ω = 1 rad/s`, `r = 0.1 m` is
`0.1 m/s` — enough to swamp the entire comparison. It reads as an estimator
regression, not as a bookkeeping error.

Make all three frames named nodes in the Euclid frame tree so a mismatch throws.

---

## 14. F11 — error budget

Differentiate F9, with `sum_i δm_i = 0` when total mass is pinned to a scale
reading:

```
δ(^W c)  ≈  (1/M) · sum_i [
      δm_i · ( ^W T_i ^i c_i  -  ^W c )      ← mass error × lever arm
    + m_i  · ^W R_i · δ(^i c_i)              ← link-CoM error
    + m_i  · ( δ ^W T_i ) · ^i c_i           ← pose error (mocap)
]
```

**The third term is the only one this pipeline controls, and mocap drives it
sub-millimetre.** The first two are CAD-sourced and enter weighted by full link
mass and lever arm; for a heavy link with a long moment arm they dominate the
third by orders of magnitude.

Conclusion: **the calibration validates pose tightly and leaves CoM exactly as
good as the URDF.** Marking a link gives its pose, never its CoM.

Cheapest available check: **weigh the robot.** It constrains `sum_i δm_i` for
free, in minutes.

The only stage that converts the first two terms from assumed to measured is the
SESC / force-plate stage (§16). It reuses F6 poses directly.

---

## 15. Gates

Each gate runs *before* the thing it protects, costs minutes, and returns
pass/fail — not a number to be interpreted later.

### G1 — rigidity

```
Var_k( || ^W m_ijk - ^W m_ij'k || )  ≈  0     for all pairs j, j' in a cluster
```

Runs on raw mocap. No FK, no URDF, no encoders, no calibration. It therefore
isolates mount slop and label swaps from every modelling question. **Run it
first, always.** It is the only gate that is purely a mocap-and-mounting
question.

Threshold: **chosen** — fail if any pair's standard deviation exceeds `3σ` where
`σ` is the measured per-axis position noise.

### G2 — bootstrap spread

Evaluate F4 *per capture* instead of averaging, giving `K` independent estimates
of the same `^i p_ij`. If FK were exact and mocap noise zero-mean, their spread
would be pure mocap noise. Anything larger is systematic, and its pattern against
`q^(k)` says which — see the escalation table in §8.

Needs no optimiser, no initial guess, and no ground truth. It is a property of
the setup, not of the estimator, so it runs at time zero and tells you whether
Approach B is even necessary.

### G3 — volume distortion

Carry a rigid two-marker artifact of fixed known length through the working
volume; assert the measured length is stable across the volume, not merely at the
centre. Catches lens-distortion and calibration-extrapolation error that a wand
residual averaged over the whole lab conceals.

### G4 — held-out reprojection

Withhold whole captures from the fit, predict their markers, report held-out RMS.
**In-sample residuals are not an accuracy claim.** Literature number to beat:
TALOS cross-validated ≈ 2.2 mm; with rigid mounts on a good volume you should be
under it.

### The signal is residual structure, not residual norm

A low in-sample RMS with residuals correlated against `q^(k)` means the fit has
absorbed a systematic error into the marker positions, where it sits undetected
and then reappears as CoM bias at runtime.

---

## 16. SESC / force-plate stage (optional, ranked first among optionals)

Under static equilibrium the ground projection of the CoM equals the measured
centre of pressure. The statically equivalent serial chain rewrites F9 as a
linear function of a reduced parameter vector `φ`, so each static pose gives one
linear equation:

```
Φ(q^(k)) · φ  =  CoP^(k)          φ̂ = (Φ^T Φ)^-1 Φ^T CoP
```

`φ̂` is obtained without trusting the CAD inertials at all — only measured link
poses from F6 and force-plate CoP. Where it disagrees with F9, `φ̂` is the better
ground truth.

Note this requires the robot on the ground, i.e. a *separate* capture session
from the hanging calibration.

---

## 17. Operating numbers

| Quantity | Value | Status |
|---|---|---|
| OptiTrack cameras | 12 | measured |
| Current calibration residual | ~0.93 mm | measured — but averaged over the whole lab |
| Camera coverage | ~3 of 4 sides | measured — weak axis points at the missing side |
| Target residual, tight ~2×2×2.5 m volume | < 0.2–0.3 mm | chosen |
| Per-axis σ at the gantry | — | **must be measured**; anisotropy of 2–3× is normal |
| `K` (calibration captures) | ≥ 30 | chosen; a starting guess, set by G2 in practice |
| Layout noise after F4 at K=30 | σ/√K ≈ 0.17 mm | derived |
| World tilt `θ` | target < 0.1° | must be measured, never assumed |
| `||c||` for tilt budgeting | ~0.8 m | assumed |
| Runtime rate | 200 Hz | chosen |
| Raw single-difference velocity noise | ~0.13 m/s | derived — unusable |
| Centred SG, 0.1 s window | ~0.0037 m/s | derived |
| ContactNet baseline / run 2 | 0.0844 / 0.0254 m/s | measured — the bar to clear |
| Held-out marker RMS | ≈ 2 mm | literature (TALOS); unmeasured here |
| URDF inertials `m_i`, `^i c_i` | CAD | **assumed — the dominant error** |

**`σ/√N` averaging holds for random noise but NOT for calibration bias**, which
is common-mode across a small cluster. Do not apply it to systematic terms.

---

## 18. Three silent failures

These are the highest-priority instrumentation targets. Each fails without an
exception, an error message, or an obviously wrong number.

1. **Rank-deficient `H` in F6.** Fewer than 3 non-collinear visible markers still
   yields a well-formed rotation from the SVD. *Mitigation:* log `σ₃` every frame
   per cluster; refuse rather than return a pose below threshold.
2. **World-frame tilt in F8.** Systematic, ungated, ~7 mm at 0.5°. *Mitigation:*
   measure `θ` physically; model `Wg` as a frame node.
3. **Frame or timestamp mismatch in F10.** Reads as an estimator regression, not
   a bookkeeping error. *Mitigation:* named frames in the Euclid tree; confirm
   which frame the EKF publishes.

---

## 19. Package map

Root: `us.ihmc.comgt` — nested to avoid a split package with the existing
`us.ihmc.mocap` NatNet client in ihmc-open-robotics-software.

| Stage | Package | Type | What the libraries give free |
|---|---|---|---|
| F1 | `model` | `RobotModelHandle` | Mecano: `setQ`, `updateFrames`, `getBodyFixedFrame` |
| F2 | `calibration` | `BaseInitializer` | — |
| F3 | `calibration` | `BootstrapSolver` | — |
| F4 | `calibration` | `MarkerLayoutSolver` | Euclid: `FramePoint3D.changeFrame` *is* the back-projection |
| F5 | `calibration` | `BaseAlignmentSolver` | shares `registration` with F6 |
| F6 | `registration` | `RigidBodyRegistration` | nothing — EJML SVD, write it |
| F7 | — | *none* | Mecano frame chaining |
| F8 | `frames` | `GravityAlignedWorldFrame` | Euclid `ReferenceFrame` |
| F9 | — | *none* | Mecano `CenterOfMassCalculator` |
| F10 | `runtime` + `postprocess` | `PelvisStateExtractor`, `PelvisTwistEstimator` | Euclid `FramePose3D` |
| F11 | `postprocess` | `ComErrorBudget` | — |
| G1–G4 | `gates` | one class each | `registration` for G1, G4 |

### Dependency rules — enforce with a test

```
core   ←  model, mocap, frames, registration
core, model, frames, registration  ←  calibration
core, model, frames, registration  ←  runtime
core, model, registration          ←  gates
core                               ←  postprocess
everything                         ←  scs2
```

- `calibration` and `runtime` **never** import each other. `CalibrationResult`
  lives in `core` for exactly this reason.
- `gates` does not import `calibration`. G2 evaluates F4 per-capture directly,
  which is what lets it run before the calibrator exists.
- Nothing outside `scs2` imports `scs2`. Core packages see only Euclid, Mecano,
  and EJML, so the whole calibration is headless-testable.

### Libraries

Euclid (geometry, frames), Mecano (rigid-body dynamics, FK, CoM), EJML (matrix
and SVD), SCS2 (URDF loading via `RobotDefinition`, simulation, visualisation),
YoVariables (live telemetry).

Language is Java, not Python: the pipeline runs alongside the Java estimator in
SCS2 with YoVariables, and the harness must exercise the shipping classes.

---

## 20. Hardware sequencing

Software work is blocked on these. They are hard blockers, not checkpoints.

1. **Static-marker test at the gantry, current setup.** Log 60 s, take per-axis
   standard deviation of the reconstructed position. The per-axis σ is what every
   formula above actually uses; the wand residual is an average over the whole
   lab and is not a substitute. Check anisotropy — the weak axis will point
   toward the missing camera side, and it propagates into `σ₃` of F6.
2. **Move the mid-rectangle cameras to converge on the gantry volume.**
   Recalibrate a tight ~2×2×2.5 m volume. Target < 0.3 mm.
3. **Repeat the static test** to confirm the improvement.
4. **Mount clusters, hang the robot, sweep the legs.** Log visible-count and `σ₃`
   histograms per cluster. If clusters drop below 3 visible markers during leg
   sweeps, fix the mounting before writing a line of F3–F5.
5. **Run G1.**

A walking volume is not needed yet — F1–F5 are static gantry captures.

Motive's `.cal` file is proprietary binary. What is usable is the exported camera
poses or the calibration report, from which a per-axis sigma map and coverage
count can be computed.

---

## 21. Open questions, ranked

1. **Are the URDF inertial blocks trustworthy?** The single biggest gap, and
   everything else is downstream of it. F11 says the pipeline can be perfect and
   the CoM still wrong. Until §16 runs, the CoM ground truth is a CAD claim
   wearing a mocap costume. Weigh the robot first — minutes. Force plate after.
2. **Joint elasticity under gravity.** Harmonic drives and structural sag mean
   the encoder `q` is not the joint angle under load. Static captures suppress
   inertial loading but not gravity. G2's load-correlated spread is the
   diagnostic.
3. **Runtime occlusion.** F6 needs ≥3 non-collinear markers visible every frame,
   per link. With 12 cameras and a self-occluding humanoid this is not
   guaranteed. Measure visible-count histograms over a real motion before
   trusting any runtime number.
4. **Which frame does the EKF report velocity in?** See §13.
5. **Marker-to-link assignment under partial visibility.** G1 catches label swaps
   *within* a cluster; it does not catch a marker assigned to the wrong link.
   Currently assumed correct from Motive rigid-body definitions.

---

## 22. Reference map

Minimal reading to implement the core is three: Umeyama → TALOS → Ehrig 2006.

| Reference | Sections |
|---|---|
| TALOS whole-body elasto-geometric calibration, RAS 2023 | the spine: observation model, both estimators, acceptance threshold |
| Umeyama 1991, TPAMI 13(4):376–380 | §2, F6, gates |
| Ehrig et al. 2006 (SCoRE), J. Biomech. 39(15) | observability and QC |
| Cotton, Murray & Fraisse 2008/2009 (SESC) | §16 |
| Arun, Huang & Blostein 1987, TPAMI 9(5) | §2 |
| Horn 1987, JOSA A 4(4) | §2, best for intuition |
| Veldpaus et al. 1988, J. Biomech. 21(1) | §2, noise propagation for markers |
| Cappozzo et al. 1995 (CAST), Clin. Biomech. 10(4) | cluster rationale |
| Espiau & Boulic 1998 (mass centre of chains), INRIA | §16 |

Caveat carried from the source document: the §16 entries and the TALOS author
list are cited to venue and method family rather than a verified author string.
Confirm before quoting any of them formally.
