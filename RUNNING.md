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

`us.ihmc.alexMocap.registration` — the registration primitive of FRAMEWORK.md §2, consumed
by F5, F6, G1 and G4. Two classes:

- `RigidBodyRegistration` — Umeyama closed-form pose from point correspondences. Stateful,
  **not thread safe** (owns preallocated EJML scratch), allocation-free in steady state.
  One instance per caller.
- `RegistrationResult` — pose plus the conditioning numbers (`σ₁ ≥ σ₂ ≥ σ₃`, correspondence
  count, reflection-corrected flag). Reports numbers, decides nothing.

Occlusion needs no special handling: an unseen marker is a correspondence you never add.

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

Currently 11 tests, no external resources, no display:

| Class | Covers |
|---|---|
| `registration.RigidBodyRegistrationTest` | exact recovery, reflection guard, rank deficiency, singular-value ordering, count normalisation, `σ/√N` noise scaling, below-minimum NaN, allocation-free, capacity growth |
| `PackageDependencyTest` | the FRAMEWORK.md §19 dependency table, by scanning compiled class files |

### Reading the tests

Two conventions carried from `PR_PLAN.md`, worth knowing before you change one:

- **Fixed seeds, loose thresholds.** Every randomised test seeds its `Random` explicitly and
  asserts a threshold several times the theoretical value, with that value stated in a
  comment. A test that flakes gets disabled and a disabled test is worse than none.
- **The thresholds are load-bearing.** Each of the guards in `RigidBodyRegistration` was
  mutation-checked: remove the determinant repair and the reflection test throws
  `NotARotationMatrixException`; remove `sortDescending()` and `σ₁` comes back as the
  *smallest* singular value (EJML genuinely returns them unordered — this is not a
  theoretical concern); drop the `1/L` on `H` and the count-normalisation test reports a
  25% shift; allocate one `double[4]` inside `compute` and the allocation test reports
  480,000 bytes. If you loosen a threshold, re-run that check.

### The allocation test

`testRegistrationIsAllocationFree` measures with `com.sun.management.ThreadMXBean`
`getThreadAllocatedBytes` and asserts exactly zero over 10,000 registrations after a 20,000
iteration warmup. It validates its own meter first — a loop known to allocate 1024
`Point3D`s must read as allocating — so a zero from the real loop is not vacuous. No JVM
flags are needed: HotSpot's counter includes the in-progress TLAB, so TLAB batching does
not hide allocation.

If this fails after a dependency bump, suspect EJML: the zero depends on
`SvdImplicitQrDecompose_DDRM` reusing its internals across same-size `decompose` calls, and
on `getU`/`getV` reshaping rather than reallocating a preallocated 3×3.
