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
 * <li><b>Nothing outside {@code scs2} imports {@code scs2}.</b> That is what keeps the whole
 * calibration headless-testable in CI with no display.</li>
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

         while (end < contents.length() && (Character.isJavaIdentifierPart(contents.charAt(end)) || contents.charAt(end) == '/'))
            end++;

         String remainder = contents.substring(start, end);
         int lastSlash = remainder.lastIndexOf('/');
         packages.add(lastSlash < 0 ? "" : remainder.substring(0, lastSlash));

         index = contents.indexOf(ROOT, end);
      }

      return packages;
   }

   private static Path mainClassesDirectory() throws URISyntaxException
   {
      return Path.of(RigidBodyRegistration.class.getProtectionDomain().getCodeSource().getLocation().toURI());
   }
}
