# PR_PLAN.md

Build plan for the whole-body CoM ground truth pipeline. Read `FRAMEWORK.md`
first; it is the authoritative specification and this document does not repeat
its derivations.

Three checkpoints. Each is a PR whose test suite runs in CI with **no hardware,
no live mocap, and no URDF it does not ship itself.** The split is not even in
file count — PR2 is the largest — it is even in falsifiability.

Root package: `us.ihmc.comgt`, under `src/main/java/us/ihmc/comgt/`.
Tests under `src/test/java/us/ihmc/comgt/`, mirroring the package structure.

---

## Rules that apply to every PR

**Dependency direction is enforced by a test, not by discipline.** Add it in PR1
and extend it in each later PR. Scan compiled class imports; fail if `runtime`
references `calibration`, if `calibration` references `runtime`, or if anything
outside `scs2` references `scs2`. Roughly ten lines, and it is the only thing
that keeps the architecture real once someone is in a hurry.

**Every randomised test uses a fixed seed and asserts a loose threshold.** A test
that flakes gets disabled, and a disabled test is worse than no test. Where a
statistical property is asserted, set the threshold at 3–5× the theoretical value
and state the theoretical value in a comment.

**No mocking of core logic.** If a test needs a robot, generate the data by
forward kinematics from a real (toy) URDF. If it needs mocap, generate points and
add seeded Gaussian noise. Mocking `RigidBodyRegistration` to test something that
consumes it tests the mock.

**Garbage-free means asserted, not intended.** Where a class is documented as
allocation-free, there is a test that runs it 10,000 times after warmup and
asserts zero allocation.

**Do not add a σ₃ threshold, a residual threshold, or any pass/fail policy to a
primitive.** Primitives report numbers. Gates and estimators decide. See §2 and
§9 of FRAMEWORK.md.

---

## PR 1 — everything that needs no robot

**Scope:** `core`, `registration`, `mocap`, and
`gates/{Gate, GateResult, RigidityGate, GateRunner}`.

**Decisive reason for this cut:** G1 is the only gate with no URDF, no FK, and no
encoders in it, and it is the gate needed *now* for the gantry captures. This PR
ships a CLI that eats a CSV of marker frames and prints per-cluster inter-marker
distance variance with a pass/fail. That is real work product on day one, not
scaffolding.

### File order

Write in this order. Each file compiles against only what precedes it.

```
registration/RegistrationResult.java        ← value type, no dependencies
registration/RigidBodyRegistration.java     ← the true leaf: Euclid + EJML only
core/MarkerId.java
core/MarkerObservation.java                 id + Point3D + visible
core/MocapFrame.java                        timestamp + observations
core/MarkerCluster.java                     link name + member ids
core/EncoderSample.java                     timestamp + q
core/Capture.java                           paired MocapFrame + EncoderSample
core/ClusterLayout.java                     calibrated positions in link frame
core/CalibrationResult.java                 layouts + Δ + provenance
core/CalibrationResultIO.java               JSON read/write
core/GroundTruthSample.java                 runtime output record
mocap/MocapSource.java                      interface
mocap/CsvReplayMocapSource.java
mocap/MocapFrameRecorder.java
mocap/MarkerLabeling.java                   Motive ids → MarkerId
mocap/NatNetMocapSource.java                ← last; hardware smoke test only
gates/GateResult.java
gates/Gate.java
gates/RigidityGate.java                     G1
gates/GateRunner.java
CalibrationRunner.java                      CLI entry: run G1 on a CSV
```

`RigidBodyRegistration` is stateful (it owns preallocated EJML scratch) and
explicitly **not thread safe**. One instance per caller. Do not extract a
separate `RegistrationWorkspace` class — a shared workspace object invites
sharing, and the ownership contract is clearer when the scratch space is
obviously owned.

`CsvReplayMocapSource` must read exactly what `MocapFrameRecorder` writes. That
pair is the entire test harness for G1 and G2, so it comes before the NatNet
client, not after.

### Tests

| Test | Assertion |
|---|---|
| Exact recovery | Plant a random `RigidBodyTransform`, transform 4 points, register → recover to `1e-12` |
| Reflection guard | Coplanar cluster + noise, seeded so raw `U Vᵀ` has `det = −1`; assert `det(R) = +1` and `RᵀR = I` over ≥1000 seeds |
| Rank deficiency | Collinear markers → `σ₃ ≈ 0` while the returned rotation is still well formed; assert the caller can detect it from `RegistrationResult` alone |
| Singular value ordering | Construct `H` with known singular values in scrambled order; assert `getSigma1() ≥ getSigma3()` always |
| Count normalisation | Same cluster geometry with 4 vs 5 visible markers; assert `σ₃` differs by less than 10% |
| Noise scaling | Seeded RNG, 1000 trials at `σ = 0.3 mm`; assert recovered translation error std matches `σ/√N` within 10% |
| Below minimum | 2 valid correspondences → returns false, transform is NaN |
| Garbage free | 10,000 registrations after warmup, assert zero allocation |
| G1 true positive | Synthetic cluster with one marker drifting 2 mm over the capture → gate **fails** |
| G1 true negative | Rigid cluster at `σ = 0.3 mm` → gate **passes** |
| CSV round trip | Write a `MocapFrame` list, read back, assert equality including visibility flags |
| Dependency graph | As described above |

The G1 pair matters more than it looks. **A gate that only has a passing test is
a gate you have never seen fire.**

### Definition of done

- All tests green in CI with no external resources.
- `CalibrationRunner --gate g1 --input <csv>` prints a per-cluster pass/fail
  table and exits non-zero on failure.
- `NatNetMocapSource` compiles and has a documented manual smoke-test procedure.
  It has no unit test — a mocked NatNet client tests the mock.

---

## PR 2 — plant and recover

**Scope:** `model`, `frames`, `calibration`, and
`gates/{BootstrapSpreadGate, HeldOutResidualGate}`.

This is the largest PR and the one where a design error is most expensive. Its
acceptance test is a single closed loop that either works or the whole approach
is wrong.

### File order

```
model/UrdfLoader.java                       wraps SCS2 URDFTools
model/RobotModelHandle.java                 setQ, updateFrames, link → ReferenceFrame
frames/FrameNames.java
frames/TiltMeasurement.java                 plumb / level / static IMU average
frames/GravityAlignedWorldFrame.java        F8 as a ReferenceFrame node
frames/PelvisFrameTriad.java                motive cluster | urdf pelvis | imu
calibration/CaptureSet.java
calibration/BaseInitializer.java            F2
calibration/BootstrapSolver.java            F3
calibration/MarkerLayoutSolver.java         F4
calibration/BaseAlignmentSolver.java        F5
calibration/AlternatingCalibrator.java      A′ loop + convergence
calibration/CalibrationReport.java          per-iteration and per-marker residuals
gates/BootstrapSpreadGate.java              G2
gates/HeldOutResidualGate.java              G4
```

Check a toy 6-DOF URDF into `src/test/resources/`. It must have at least two
serial branches below the base so that F5 has something to identify `Δ` from.

### The acceptance test — plant and recover

Plant a known marker layout and a known `Δ`. Generate `K` captures by forward
kinematics at scattered joint configurations. Add seeded noise. Run A′. Assert
recovery of both.

| Case | Assertion |
|---|---|
| Noiseless, K = 5 | Layout and `Δ` recovered to `1e-9`. Any failure here is an algebra bug, not a tuning problem |
| `σ = 0.3 mm`, K = 30, seeded | Layout recovered under 0.2 mm. Theoretical is `σ/√K ≈ 0.055 mm`; the threshold is deliberately loose |
| Monotone descent | `J` is non-increasing across **every** A′ iteration. This is the property that justifies A′ over bundle adjustment — assert it, do not assume it |
| Convergence | Terminates in under ~20 iterations; a second run from a perturbed seed lands in the same place |
| Partial visibility | Randomly occlude 20% of observations; assert recovery still succeeds and `K_ij` bookkeeping is correct |

### The diagnostic tests — the reason G2 exists

Inject a fault into the **generator** but not into the **model**, then assert the
gate both fires and points at the right thing.

| Injection | Assertion |
|---|---|
| 0.5° offset on one joint | G2 **fails**, and the reported spread correlates with *that specific joint's* excursion above threshold. This is the test that proves G2 says *which*, not merely *that* |
| Load-proportional deflection | G2 **fails**, and the correlation lands on load, not on any single joint's excursion |
| Clean data | G2 **passes**, spread consistent with mocap noise alone |
| G4 clean | Held-out RMS within a small factor of in-sample RMS |
| G4 with injected offset | Held-out RMS blows up while in-sample stays low. That asymmetry is the entire point of held-out validation |

### Be honest about what this proves

The toy URDF tests the solver, not TALOS's 2.2 mm number. It cannot tell you
whether the real robot's geometry is good enough. It tells you that when the
geometry *is* good, the code recovers the answer — which is exactly what must be
true before pointing it at real data.

### Definition of done

- Plant-and-recover green at both noise levels.
- All three G2 injection cases green.
- `CalibrationRunner --calibrate` produces a `CalibrationResult` JSON from a CSV
  capture set plus a URDF, and a `CalibrationReport` with per-iteration `J` and
  per-marker residuals.
- Dependency test extended: `calibration` does not import `runtime`; `gates` does
  not import `calibration`.

---

## PR 3 — runtime and the second pass

**Scope:** `runtime`, `postprocess`, `scs2`.

### File order

```
runtime/MeasuredLinkPoses.java              link → pose + σ₃ + visibleCount
runtime/LinkPoseEstimator.java              F6, owns the refusal policy
runtime/KinematicChainCompleter.java        F7
runtime/CenterOfMassGroundTruth.java        F9, wraps Mecano
runtime/PelvisStateExtractor.java           F10 pose only — no getTwist()
postprocess/SavitzkyGolayDifferentiator.java
postprocess/PelvisTwistEstimator.java       centred window over logged poses
postprocess/ComErrorBudget.java             F11
postprocess/ErrorBudgetReport.java
scs2/GroundTruthYoVariables.java
scs2/ConditioningMonitor.java               σ₃ + visible count per frame
scs2/GroundTruthYoGraphics.java
scs2/GroundTruthSessionVisualizer.java
ReplayRunner.java                           runtime entry: logged mocap → CoM + pelvis
```

### Tests

| Test | Assertion |
|---|---|
| CoM self-consistency | Feed F6 exact poses generated from FK; assert `CenterOfMassGroundTruth` matches Mecano's `CenterOfMassCalculator` on the same configuration to `1e-9` |
| F7 chaining | Mark only the pelvis; assert unmarked link poses match FK exactly |
| F8 tilt injection | Inject `θ = 0.5°`, `‖c‖ = 0.8 m`; assert height error equals `0.8·sin(0.5°) = 6.98 mm` to within 0.01 mm |
| F8 correction | Apply the measured tilt; assert error returns to zero |
| No twist at runtime | By reflection: `PelvisStateExtractor` exposes no method returning a velocity or twist type |
| SG accuracy | Analytic sinusoidal trajectory + `σ = 0.93 mm` at 200 Hz; assert recovered velocity RMS under 0.005 m/s for a 0.1 s centred window |
| **SG zero lag** | Same trajectory: assert the cross-correlation peak between truth and estimate sits at zero lag, **and** that a causal filter of equal width does not |
| Occlusion refusal | Drop a cluster to 2 visible markers; assert `LinkPoseEstimator` refuses rather than returning a pose |
| Low conditioning refusal | Near-collinear cluster; assert refusal driven by `σ₃`, and that the refusal is logged |
| F11 | Perturb one link mass by 1%; assert the CoM shift matches the closed form in FRAMEWORK.md §14 |

**The SG lag test is not optional.** Lag against an EKF being validated for
velocity is a systematic comparison error that looks exactly like estimator bias.
A filter that is accurate but delayed passes every accuracy test you would
otherwise write.

### Definition of done

- All tests green.
- `ReplayRunner` consumes a logged CSV plus a `CalibrationResult` and emits a
  CoM trajectory, a pelvis pose trajectory, and per-frame `σ₃` / visible-count
  logs.
- `PelvisTwistEstimator` runs as a documented second pass over the pose log.
- Visualiser opens. It gets no test — if a JavaFX window does not appear you will
  know within seconds.

---

## What is deliberately not gated

- `NatNetMocapSource` — hardware smoke test only.
- `GroundTruthSessionVisualizer` — no test.
- Real-robot accuracy — no synthetic test can establish it. That is what G1–G4
  on real captures are for, and they are run by a human at the gantry.

## If you want two PRs instead of three

Merge PR1 into PR2. Do **not** merge PR2 into PR3. PR3's tests all assume a
working calibration; folding it upward produces one enormous PR where a failure
could be in any of nine components.
