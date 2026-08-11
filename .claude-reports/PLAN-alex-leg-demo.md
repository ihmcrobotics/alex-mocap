# Plan — demonstrate the pipeline on the real Alex model, leg-only markers

## Context

`alex-mocap` (PRs #1–#3) is verified only against a **toy 6-DOF URDF**. Every accuracy
claim in `RUNNING.md` carries the caveat "tests the solver, not the robot". Lucas wants a
demonstration on the **real Alex model** — the one his Python InEKF uses — with a
randomized marker set on the legs, because the legs carry the joints the filter estimates.

Source URDF (read-only, do not modify):
`/home/llibshutz/alex/invariant-estimation/assets/alex_with_imus.urdf`

Branch `pr4-alex-demo`, worktree `/home/llibshutz/alex/alex-mocap-demo`, off `pr3`.
**The main tree at `/home/llibshutz/alex/alex-mocap` must not be touched.**

## Established facts (trust these; re-verify only if something contradicts them)

- 141 links / 140 joints (29 revolute, 111 fixed), root `PELVIS_LINK`, **91.5126 kg**, no
  floating joint, strict tree.
- SCS2's `simplifyKinematics` (default **on**) merges all fixed joints, leaving an expected
  **29 joints / 30 links**, mass preserved.
- Leg chain per side, from `PELVIS_LINK`:
  `HIP_X`(x)→`*_HIP_X_LINK`, `HIP_Z`(z)→`*_HIP_Z_LINK`, `HIP_Y`(y)→`*_THIGH`,
  `KNEE_Y`(y)→`*_SHIN`, `ANKLE_Y`(y)→`*_ANKLE_Y_LINK`, `ANKLE_X`(x)→`*_FOOT`.
  Prefixes are `LEFT_`/`RIGHT_`. Note `THIGH`/`SHIN`/`FOOT` have **no** `_LINK` suffix.
- Masses: PELVIS 7.161; per side HIP_X_LINK 2.332, HIP_Z_LINK 0.761, THIGH 8.281,
  SHIN 6.338, ANKLE_Y_LINK 0.050, FOOT 0.810; TORSO 22.21.
- The filter's actual states are **9 joints**: `SPINE_Z` + both `HIP_X/Z/Y` + `KNEE_Y`.
  Ankles are measured inputs, not states. Foot chains use all 12 leg joints.
- Hazards, all believed non-blocking but unproven: 11 `<capsule>` collisions (SCS2 has no
  `URDFCapsule`), 17 missing hand meshes, 70 links with no `<inertial>`, 19 with mass 0
  and zero inertia, 8 `<mimic>` on `type="fixed"` joints, 17 `<gazebo>` blocks.

## Step 0 — blocking gate, do this first

Probe `RobotModelHandle.fromURDF(sourceUrdf)`. Print joint count, link count, base link
name, total mass, and whether any link has non-finite mass or `^i c_i`.

- **Loads** → vendor the URDF byte-identically to
  `src/test/resources/us/ihmc/alexMocap/model/alex.urdf` and proceed.
- **Throws** → write a test-scope `URDFSanitizer` that DOM-strips `<visual>`,
  `<collision>` and `<gazebo>` only, vendor the stripped file, and add an equivalence test
  asserting identical joint names in order, link names, per-link mass, per-link `^i c_i`,
  and `^b T_i` for **every link at 20 random configurations**. The last one is what
  catches a frame change; the others alone would not.

Record which path was taken in the report either way.

## Step 1 — joint limits on `RobotModelHandle`

Add `getJointLimitLower/Upper(int|String)` reading `OneDoFJointBasics.getJointLimitLower/Upper()`.
Verified reachable: `RevoluteJointDefinition.toJoint` calls `setJointLimits`. One parse,
one source of truth — do **not** re-parse the URDF for limits.

## Step 2 — `RobotCaptures`, a generator for a real robot

New file `src/test/java/us/ihmc/alexMocap/calibration/RobotCaptures.java`. **Do not modify
`SyntheticCaptures`** — four seeded test classes depend on its exact random draw order, and
its hardcoded tables are a documented deliberate choice.

Must:
- randomize only a **named subset** of joints, holding the rest at a rest angle **clamped
  into limits**, printing any joint it had to clamp (`LEFT_KNEE_Y` has `lower="0"`, exactly
  on the boundary);
- **throw** on a non-finite joint limit rather than sampling NaN;
- support a `jointExcursionFraction` so a narrow-sweep case is expressible;
- plant markers centred on each link's **CoM** (`packCenterOfMassInLinkFrame`), not on a
  fixed offset from the link origin — Alex's link origins sit at joint axes, so the toy's
  offset would put thigh markers inside the pelvis. Keep a small deterministic offset so the
  cluster is not centred on anything.
- **resample near-collinear constellations** (covariance `λ₂ < (spread/10)²`), with a knob to
  disable. Guard collinearity (σ₂), *not* coplanarity (σ₃) — coplanar is the normal case.
- keep the `q_true` / `q_reported` split so fault injection is a two-line addition;
- gauge spread 0.14 m (§1's outrigger bracket), limb spread 0.06 m.

## Step 3 — the demonstration

`src/test/java/us/ihmc/alexMocap/AlexLegDemoTest.java`. Verbose tables behind
`-Dalex.demo.verbose=true` so CI stays quiet.

Primary marked set — **7 clusters**: `PELVIS_LINK` (gauge, structurally required),
`LEFT/RIGHT_THIGH`, `LEFT/RIGHT_SHIN`, `LEFT/RIGHT_FOOT`. That is what §1's "descending
order of mass × lever arm" actually selects (8.28 / 6.34 / 0.81 kg against 2.33 / 0.76 /
**0.05** kg for the stubs). Add one case marking all 13 leg links.

Tests:
1. URDF loads: 29 joints, 30 links, root `PELVIS_LINK`, mass 91.5126 ± 1e-4; **every link's
   mass and `^i c_i` finite** (tripwire for the `merge` 0/0 → NaN hazard); leg limits finite.
2. At `q = 0` every leg link's `^b T_i` rotation is identity to 1e-12 (pins the
   `transformToZUp` no-op claim for the leg chain; it is **not** a no-op for the arms).
3. Noiseless K=5 → layout and base pose to 1e-9.
4. σ = 0.3 mm, K = 30, seeded. **Measure first, then set the threshold at ~3× measured with
   the measured value in a comment.** Do not guess it — the toy's 0.2 mm does not transfer.
5. `J` monotone across every A′ half-step.
6. `PELVIS_LINK` + both `THIGH` only → still exact. Identifiability with margin.
7. `PELVIS_LINK` + both `HIP_X_LINK` only → **predicted degenerate**: both have `R = R_x(q)`
   with parallel `(1 0 0)` axes, so `g ∥ x` is an exact symmetry. Assert `J` → machine zero
   **and** a common displacement along x. This reproduces PR2's finding at Alex scale.
   Cap iterations explicitly.
8. Chained mass: expect ~53.5 kg of 91.5 (58.5 %) for the 7-cluster set. Assert `TORSO_LINK`
   chains from `PELVIS_LINK`.
9. CoM vs Mecano's `CenterOfMassCalculator` to 1e-9 on the real 91.5 kg model.
10. G2 and G4 pass on clean data.
11. G2 fires on a 0.5° `LEFT_HIP_Y` offset and localises to the **left** branch.
12. Narrow sweep (`jointExcursionFraction = 0.05`) collapses the base step's σ₃ relative to
    the full sweep. Alex's hip-X range is 70° against the toy's 183°; rank is fine,
    conditioning may not be. Measure it.

`src/test/java/us/ihmc/alexMocap/AlexLegDemoCliTest.java`: drive the **shipping** CLIs
unchanged — `CalibrationRunner --calibrate` then `ReplayRunner` — over `@TempDir` CSVs
written with `MocapFrameRecorder` / `CsvEncoderLog`. Naming markers `<LINK>_M<j>` makes
`inferClusters` produce exact link names with no `--cluster` flag; assert that rather than
relying on it.

## Step 4 — write up

- `RUNNING.md`: a "Demonstration: the real Alex model" section with real pasted output.
- `.claude-reports/2026-08-10-alex-leg-demo.md`: the morning report (see below).

## The headline finding to surface

With only pelvis + legs marked, **~58 % of Alex's mass has its pose from FK, not markers**.
`TORSO_LINK` alone is 22.21 kg — 24.3 % of the robot — chained off the pelvis through one
`SPINE_Z` joint. `ReplayRunner` already prints this; make sure the demo shows it and the
report leads with it. One torso cluster is the highest-leverage addition after the gauge.

## Rules

- Fixed seeds, loose thresholds, measured values stated in comments (PR_PLAN).
- Never loosen a threshold to make a test pass without understanding why it moved.
- Do not modify `SyntheticCaptures`, `CalibrationRunner`, `ReplayRunner`, or any existing
  test. If one of them genuinely must change, stop and record why in the report instead.
- Everything must stay headless. No JavaFX in the test path.
- Commit in logical steps with real messages. Leave the branch green.
