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

Still none. Everything under `src/main/java` except the `registration` package is an empty
placeholder (see `git log`: "add all files as empty for now"). The two intended entry
points are:

- `us.ihmc.alexMocap.CalibrationRunner` — PR1 definition of done is
  `--gate g1 --input <csv>` printing a per-cluster pass/fail table, exit non-zero on fail
- `us.ihmc.alexMocap.ReplayRunner` — PR3

Once either has a `main`, add an `application` plugin block or a `JavaExec` task and
document the invocation here.

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
| `org.ejml:ejml-core` | 0.39 | `implementation` |
| `org.ejml:ejml-ddense` | 0.39 | `implementation` |
| `org.junit.jupiter:junit-jupiter` | 5.10.2 | `testImplementation` |

Euclid and mecano are `api` because their types (`RigidBodyTransform`, `ReferenceFrame`,
`RigidBodyBasics`) show up in this package's own public signatures.

EJML is `implementation` on purpose. No EJML type appears in a public signature —
`RegistrationResult` reports doubles and a `RigidBodyTransform` — and keeping it off the
api surface is what stops a caller reaching past `RigidBodyRegistration` and writing a
second SVD. FRAMEWORK.md §2: *"There must be exactly one implementation."*

`0.39` is not a new version in the graph: euclid already pulls `ejml-core:0.39` and mecano
pulls `ejml-ddense:0.39`. It is declared explicitly so that `registration` does not depend
on mecano in order to get an SVD.

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

Currently 33 tests, no external resources, no display:

| Class | Covers |
|---|---|
| `registration.RigidBodyRegistrationTest` | exact recovery, reflection guard, rank deficiency, singular-value ordering, count normalisation, `σ/√N` noise scaling, below-minimum NaN, allocation-free, capacity growth |
| `core.CoreDataTypesTest` | dense marker sets, cross-marker-set rejection, NaN-when-unset, visible counts, capture skew, joint-order checking, `K_ij` bookkeeping, no-velocity-by-reflection, allocation-free frame loop |
| `core.CalibrationResultIOTest` | exact JSON round trip incl. byte-identical rewrite, `Δ` over 1000 random transforms, NaN survival, unknown-marker rejection, format version, parse errors |
| `PackageDependencyTest` | the FRAMEWORK.md §19 dependency table, by scanning compiled class files |

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
