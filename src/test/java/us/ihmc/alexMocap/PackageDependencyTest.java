package us.ihmc.alexMocap;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.registration.RigidBodyRegistration;

/**
 * Enforces the dependency direction of FRAMEWORK.md §19 by reading the compiled classes, because
 * discipline does not survive someone being in a hurry.
 * <p>
 * The two rules that matter most, and the reason this file exists at all:
 * </p>
 * <ul>
 * <li><b>{@code calibration} and {@code runtime} never import each other.</b> {@code
 * CalibrationResult} lives in {@code core} for exactly this reason. The moment one imports the
 * other, the offline calibration and the 200 Hz runtime loop are one component.</li>
 * <li><b>Nothing outside {@code scs2} imports the SCS2 <i>visualizer</i>.</b> That is what keeps
 * the whole calibration headless-testable in CI with no display. See
 * {@link #testOnlyModelReachesScs2AndOnlyItsHeadlessHalf()} for why this is stated in terms of
 * the visualizer rather than of SCS2 as a whole.</li>
 * </ul>
 * <p>
 * The scan looks for internal class names in each class file's constant pool. Any type a class
 * references -- imported, extended, thrown, or named only in a signature -- appears there
 * literally, so a forbidden reference cannot hide from this. It is deliberately cruder than a
 * bytecode library: no dependency, and nothing to keep up to date.
 * </p>
 */
public class PackageDependencyTest
{
   private static final String ROOT = "us/ihmc/alexMocap/";

   /**
    * Allowed dependencies, per package. FRAMEWORK.md §19.
    * <p>
    * Note that {@code registration} is pinned tighter than §19 permits. The table there allows
    * {@code registration -> core}; PR_PLAN.md calls {@code RigidBodyRegistration} "the true leaf:
    * Euclid + EJML only", and that is the stronger and more useful claim -- it is what lets F5,
    * F6, G1 and G4 all share one implementation without any of them dragging a data model along.
    * Relaxing this is a deliberate one-line decision, which is the point.
    * </p>
    */
   private static final Map<String, Set<String>> ALLOWED_DEPENDENCIES = new LinkedHashMap<>();

   static
   {
      ALLOWED_DEPENDENCIES.put("registration", Set.of());
      ALLOWED_DEPENDENCIES.put("core", Set.of());
      ALLOWED_DEPENDENCIES.put("model", Set.of("core"));
      ALLOWED_DEPENDENCIES.put("mocap", Set.of("core"));
      ALLOWED_DEPENDENCIES.put("frames", Set.of("core"));
      ALLOWED_DEPENDENCIES.put("postprocess", Set.of("core"));
      ALLOWED_DEPENDENCIES.put("gates", Set.of("core", "model", "registration"));
      ALLOWED_DEPENDENCIES.put("calibration", Set.of("core", "model", "frames", "registration"));
      ALLOWED_DEPENDENCIES.put("runtime", Set.of("core", "model", "frames", "registration"));
      ALLOWED_DEPENDENCIES.put("scs2", Set.of("core", "model", "mocap", "frames", "registration", "calibration", "runtime", "postprocess", "gates"));
      // The root package holds the CLI entry points, which wire everything together.
      ALLOWED_DEPENDENCIES.put("", Set.of("core", "model", "mocap", "frames", "registration", "calibration", "runtime", "postprocess", "gates", "scs2"));
   }

   @Test
   public void testDependencyDirection() throws IOException, URISyntaxException
   {
      Path classesRoot = mainClassesDirectory();
      List<String> violations = new ArrayList<>();
      int classesScanned = 0;

      try (Stream<Path> files = Files.walk(classesRoot))
      {
         for (Path classFile : files.filter(p -> p.toString().endsWith(".class")).toList())
         {
            String owner = packageOf(classesRoot.relativize(classFile).toString().replace('\\', '/'));
            Set<String> allowed = ALLOWED_DEPENDENCIES.get(owner);

            if (allowed == null)
            {
               violations.add("Package '" + owner + "' is not in the dependency table. Add it deliberately, with its allowed set.");
               continue;
            }

            classesScanned++;

            for (String referenced : referencedPackages(Files.readAllBytes(classFile)))
            {
               if (referenced.equals(owner) || allowed.contains(referenced))
                  continue;

               violations.add(classesRoot.relativize(classFile) + " references package '" + referenced + "', which '" + owner + "' may not depend on.");
            }
         }
      }

      assertTrue(classesScanned > 0, "No compiled classes found under " + classesRoot + "; the scan would pass vacuously.");

      if (!violations.isEmpty())
         fail("Dependency direction violated (FRAMEWORK.md §19):\n  " + String.join("\n  ", new TreeSet<>(violations)));
   }

   /**
    * FRAMEWORK.md §19 as it is actually meant, enforced against the <b>external</b> SCS2 library
    * rather than against this project's own {@code scs2} package.
    *
    * <h2>Why this test has to exist separately</h2>
    * <p>
    * {@link #testDependencyDirection()} scans for {@code us/ihmc/alexMocap/} names only. It is
    * therefore completely blind to {@code import us.ihmc.scs2.…}, which means the rule everyone
    * believes is being enforced -- "core packages see only Euclid, Mecano, and EJML" -- was in fact
    * enforced nowhere. Any package could have taken a dependency on SCS2 and no test would have
    * moved. That is the precise failure mode the dependency test was written to prevent, so leaving
    * it would have been worse than not having the rule at all.
    * </p>
    *
    * <h2>What the rule became, and why it was narrowed</h2>
    * <p>
    * SCS2 ships as separate artifacts. {@code scs2-definition} is headless: it is
    * {@code URDFTools}, {@code RobotDefinition}, and {@code RobotDefinition.newInstance()} which
    * hands back a Mecano tree. It pulls no JavaFX and pins the same euclid and mecano versions this
    * project already declares. {@code scs2-session-visualizer-jfx} is the one that needs a display.
    * </p>
    * <p>
    * §19's stated motive is headless-testability, and that motive is satisfied by banning the
    * visualizer. Banning {@code scs2-definition} as well would buy nothing and cost a hand-written
    * URDF parser. So the rule enforced here is the narrower one:
    * </p>
    * <ul>
    * <li>Only {@code model} may reference SCS2 at all, and only {@code us/ihmc/scs2/definition/}.
    * {@code URDFLoader} keeps every SCS2 type inside method bodies, which is why the Gradle
    * dependency is {@code implementation} and not {@code api}.</li>
    * <li>Nothing outside the {@code scs2} package may reference any other SCS2 subpackage --
    * {@code session}, {@code simulation}, {@code sharedMemory} and friends. When PR3 adds the
    * visualizer, this is the line it must not cross.</li>
    * </ul>
    */
   @Test
   public void testOnlyModelReachesScs2AndOnlyItsHeadlessHalf() throws IOException, URISyntaxException
   {
      Path classesRoot = mainClassesDirectory();
      List<String> violations = new ArrayList<>();

      try (Stream<Path> files = Files.walk(classesRoot))
      {
         for (Path classFile : files.filter(p -> p.toString().endsWith(".class")).toList())
         {
            String owner = packageOf(classesRoot.relativize(classFile).toString().replace('\\', '/'));
            String contents = new String(Files.readAllBytes(classFile), java.nio.charset.StandardCharsets.ISO_8859_1);

            for (String referenced : new TreeSet<>(referencedExternalScs2Packages(contents)))
            {
               boolean headless = referenced.startsWith("definition/");

               if (owner.equals("scs2"))
                  continue;

               if (owner.equals("model") && headless)
                  continue;

               violations.add(classesRoot.relativize(classFile) + " references SCS2 '" + referenced + "'. "
                     + (headless ? "Only 'model' may use scs2-definition." : "Only the 'scs2' package may use SCS2 beyond scs2-definition."));
            }
         }
      }

      if (!violations.isEmpty())
         fail("SCS2 containment violated (FRAMEWORK.md §19):\n  " + String.join("\n  ", new TreeSet<>(violations)));
   }

   /** Every {@code us/ihmc/scs2/<pkg>/} occurrence in a class file's constant pool. */
   private static Set<String> referencedExternalScs2Packages(String classFileContents)
   {
      String prefix = "us/ihmc/scs2/";
      Set<String> packages = new TreeSet<>();
      int index = classFileContents.indexOf(prefix);

      while (index >= 0)
      {
         int start = index + prefix.length();
         int end = start;

         while (end < classFileContents.length() && isInternalNameCharacter(classFileContents.charAt(end)))
            end++;

         String remainder = classFileContents.substring(start, end);
         int lastSlash = remainder.lastIndexOf('/');

         if (lastSlash > 0)
            packages.add(remainder.substring(0, lastSlash + 1));

         index = classFileContents.indexOf(prefix, end);
      }

      return packages;
   }

   /**
    * The claim PR_PLAN.md makes about the leaf, stated as its own test so that a failure names the
    * thing that broke rather than showing up as one line in a table.
    */
   @Test
   public void testRegistrationDependsOnNothingInThisProject() throws IOException, URISyntaxException
   {
      Path classesRoot = mainClassesDirectory();
      Path registrationDirectory = classesRoot.resolve("us/ihmc/alexMocap/registration");
      assertTrue(Files.isDirectory(registrationDirectory), "Expected compiled registration classes at " + registrationDirectory);

      try (Stream<Path> files = Files.walk(registrationDirectory))
      {
         for (Path classFile : files.filter(p -> p.toString().endsWith(".class")).toList())
         {
            for (String referenced : referencedPackages(Files.readAllBytes(classFile)))
            {
               if (!referenced.equals("registration"))
                  fail(classFile.getFileName() + " references '" + referenced + "'. The registration primitive is Euclid + EJML only.");
            }
         }
      }
   }

   /** Sub-package of {@link #ROOT} owning a class file, or {@code ""} for the root package. */
   private static String packageOf(String relativeClassPath)
   {
      String withoutRoot = relativeClassPath.substring(ROOT.length());
      int lastSlash = withoutRoot.lastIndexOf('/');
      return lastSlash < 0 ? "" : withoutRoot.substring(0, lastSlash);
   }

   /**
    * Every {@code us/ihmc/alexMocap/<pkg>/} occurrence in the raw class bytes, reduced to the set
    * of sub-package names. Referenced type names live in the constant pool as plain UTF-8, so a
    * literal scan finds them without a class-file parser.
    */
   private static Set<String> referencedPackages(byte[] classFileBytes)
   {
      Set<String> packages = new TreeSet<>();
      String contents = new String(classFileBytes, java.nio.charset.StandardCharsets.ISO_8859_1);
      int index = contents.indexOf(ROOT);

      while (index >= 0)
      {
         int start = index + ROOT.length();
         int end = start;

         while (end < contents.length() && isInternalNameCharacter(contents.charAt(end)))
            end++;

         String remainder = contents.substring(start, end);
         int lastSlash = remainder.lastIndexOf('/');
         packages.add(lastSlash < 0 ? "" : remainder.substring(0, lastSlash));

         index = contents.indexOf(ROOT, end);
      }

      return packages;
   }

   /**
    * Characters a JVM internal class name can contain, enumerated rather than delegated to
    * {@link Character#isJavaIdentifierPart}.
    * <p>
    * {@code isJavaIdentifierPart} returns {@code true} for ignorable control characters, including
    * {@code } -- which is exactly the separator the compiler uses inside a record's
    * {@code ObjectMethods} bootstrap string. Using it here glued that string onto the class name
    * before it and invented package names like
    * {@code core/CalibrationResult$ProvenanceurdfLjava/lang}.
    * </p>
    */
   private static boolean isInternalNameCharacter(char c)
   {
      return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '$' || c == '/';
   }

   private static Path mainClassesDirectory() throws URISyntaxException
   {
      return Path.of(RigidBodyRegistration.class.getProtectionDomain().getCodeSource().getLocation().toURI());
   }
}
