# Wiring the SCS2 mocap ground-truth track

Everything in `src/` is finished and tested. This directory holds the part that lives in the
**`alex` repository** and therefore has not been applied — it needs three edits outside this repo.

**Status: written against the real APIs, not yet compiled.** Method signatures were checked against
`scs2-simulation-construction-set-17-0.33.2` and `scs2-simulation-17-0.33.2` with `javap`
(`SimulationConstructionSet2.addYoGraphic`, `.addRegistry`, `Robot.getControllerManager().addController`,
`Robot.getRigidBody`, `AlexRobotModel.getRobotDefinition`), but nothing here has been through a
compiler, because `alex` cannot see `alex-mocap` until step 1 below.

---

## 1. Put `alex-mocap` in the composite build

`~/workspaces/mocap/settings.gradle.kts`, after the existing configurator lines:

```kotlin
// alex-mocap is a plain Gradle build, not an ihmc-build project, so findAndIncludeCompositeBuilds()
// does not pick it up. Substitution matches on group:name, which is why alex-mocap's build.gradle.kts
// sets `group = "us.ihmc"`.
includeBuild("alex-mocap")
```

## 2. Declare the dependency

`~/workspaces/mocap/alex/build.gradle.kts`, in `mainDependencies`:

```kotlin
api("us.ihmc:alex-mocap:0.1.0")
```

The version is ignored once `includeBuild` substitutes the local project, but Gradle wants one.

## 3. Drop in the track

```
cp integration/alex/AlexMocapGroundTruthTrack.java \
   ~/workspaces/mocap/alex/src/main/java/us/ihmc/alex/simulation/
```

Then:

```bash
export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
cd ~/workspaces/mocap/alex && ../gradlew compileJava
```

Run it from IntelliJ like `AlexFlatGroundWalkingTrack`, or:

```bash
cd ~/workspaces/mocap/alex && ../gradlew run -PmainClass=us.ihmc.alex.simulation.AlexMocapGroundTruthTrack
```

`-Dmocap.occlusion=0.12` turns on occlusion. Leave it off unless you specifically want to see
refusals — it refuses 63 % of frames, and a mostly-NaN trace is hard to read.

---

## Version alignment — already done, worth knowing why

`alex-mocap` was on scs2 `17-0.30.0` / mecano `17-0.19.2`; `ihmc-avatar-interfaces` declares
scs2 `17-0.33.2` / mecano `17-0.19.3`. Sharing a classpath with two SCS2 minor versions is not a
thing anyone should debug, so `alex-mocap` moved to match the stack (commit `68b597e`). All 195
tests pass across the bump. `euclid` was already `0.22.5` on both sides.

## What you will see

- **28 marker spheres**, four per link, coloured per cluster. Yellow is the pelvis gauge.
- **A gold sphere** — the mocap chain's centre of mass.
- **A green sphere** — the simulation's actual centre of mass.

They sit on top of each other when things are right. The variables to plot:

| variable | meaning |
|---|---|
| `mocapMocapMinusActualComMagnitude` | per-frame &#124;mocap − actual&#124; |
| `mocapMocapMinusActualComMean` | a floor here is a **bias**, not noise |
| `mocapMocapMinusActualComStandardDeviation` | jitter |
| `mocapVisibleMarkerCount` | 28 unless occluded |
| `mocapRefusedClusterCount` | clusters with no usable pose |
| `mocap<LINK>Sigma3SquaredMetres` | per-cluster conditioning, **m²** not m |

## Two honesty caveats, both encoded in the code

**The layout is planted, not calibrated.** `SimulatedMocapGroundTruth.demonstration(...)` hands the
runtime the constellation's own truth, so what is on screen is F6–F9 with calibration error set to
zero. `isUsingPlantedLayout()` is true and `summary()` says so. A calibrated layout on Alex carries
a further **2.86 mm** held-out RMS at FRAMEWORK.md §1's 140 mm gauge bracket and §17's 0.3 mm
noise — against a 2.2 mm TALOS bar.

**The mocap chain and the simulation share a URDF**, so link masses and link-CoM offsets are
identical by construction. F11 measured those as the dominant real-world terms: mass 4.90 mm /
link-CoM 2.73 mm / mocap 0.164 mm, CAD dominating by 33×. The error on screen is the mocap chain's
own, with the largest real contributor removed. Weigh the robot.

Encoders are perfect too, which matters because a legs-only marker set leaves **58.45 %** of Alex's
mass posed by FK rather than by markers — `TORSO_LINK` alone is 24 % of it, through a single
`SPINE_Z` joint that is itself an estimator state. One torso cluster takes the FK-posed fraction to
34.18 %, and is the highest-leverage addition after the gauge bracket.
