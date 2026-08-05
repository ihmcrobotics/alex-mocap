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

There are none yet. As of this writing all 53 files under `src/main/java` are empty
placeholders (see `git log`: "add all files as empty for now"), so `./gradlew build`
succeeds but produces an effectively empty jar. The two intended entry points are:

- `us.ihmc.alexMocap.CalibrationRunner`
- `us.ihmc.alexMocap.ReplayRunner`

Once either has a `main`, add an `application` plugin block or a `JavaExec` task and
document the invocation here.

## Dependencies

Declared in `gradle/libs.versions.toml`, consumed in `build.gradle.kts`:

| Module | Version |
|---|---|
| `us.ihmc:euclid` | 0.22.5 |
| `us.ihmc:euclid-frame` | 0.22.5 |
| `us.ihmc:euclid-geometry` | 0.22.5 |
| `us.ihmc:mecano` | 17-0.19.2 |

They are `api` rather than `implementation` because euclid and mecano types
(`RigidBodyTransform`, `ReferenceFrame`, `RigidBodyBasics`) show up in this package's
own public signatures.

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

No test dependency is declared yet — `src/test` does not exist, so `./gradlew build`
reports `test NO-SOURCE`. The test plan in `README.md` (exact recovery, reflection guard,
SVD rank deficiency, no-garbage-allocation) will need JUnit 5 added to the catalog before
any of it can be written.
