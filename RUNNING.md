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

Note: `org.gradle.configuration-cache=true` is set in `gradle.properties`. Editing
`build.gradle.kts` or `libs.versions.toml` invalidates the cache and the next build
recalculates the task graph — that is expected, not a failure.

## Entry points

There are none yet. As of this writing all 53 files under `src/main/java` are empty
placeholders (see `git log`: "add all files as empty for now"), so `./gradlew build`
succeeds but produces an effectively empty jar. The two intended entry points are:

- `us.ihmc.CalibrationRunner`
- `us.ihmc.ReplayRunner`

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
