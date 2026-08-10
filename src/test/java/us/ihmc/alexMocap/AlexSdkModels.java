package us.ihmc.alexMocap;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Finds {@code ihmc-alex-sdk}'s mesh roots, without anybody typing an absolute path.
 *
 * <h2>Why this exists</h2>
 * <p>
 * The URDF this project calibrates against is vendored into test resources so CI needs nothing
 * outside the repo, and it has no meshes beside it. The meshes are in the SDK, which is a sibling
 * checkout in an IHMC repository group. Nothing in a Gradle build tells us where that is.
 * </p>
 * <p>
 * The previous answer was a hard-coded path into a colleague's Python estimator checkout
 * ({@code ~/alex/invariant-estimation/assets/}), which is wrong twice over: it points at a copy
 * rather than at the source of truth, and it is an absolute path in a repository that has already
 * been moved once. This walks up from wherever the build is running and looks for the SDK instead.
 * </p>
 *
 * <h2>What resolves where</h2>
 * <p>
 * Measured against the vendored Alex URDF's 37 {@code package://} references:
 * </p>
 * <ul>
 * <li>{@code <sdk>/alex-models/} resolves 29 -- every leg, pelvis and torso mesh, which is all the
 * legs-and-pelvis demonstration needs;</li>
 * <li>{@code <group>/ihmc_hands_ros2/meshes/} resolves the remaining 8, which are ability-hand
 * hulls. Optional: without it the hands draw as nothing and the rest of the robot is unaffected.</li>
 * </ul>
 */
public final class AlexSdkModels
{
   /** Checked first, so a non-standard layout can be pointed at explicitly. */
   public static final String SYSTEM_PROPERTY = "alex.sdk.dir";

   /** Checked second. */
   public static final String ENVIRONMENT_VARIABLE = "ALEX_SDK_DIR";

   /** How far up from the working directory to look for a sibling SDK checkout. */
   private static final int SEARCH_HEIGHT = 6;

   private AlexSdkModels()
   {
   }

   /**
    * The SDK's {@code alex-models} directory, if it can be found.
    * <p>
    * Order: {@code -Dalex.sdk.dir}, {@code $ALEX_SDK_DIR}, then a walk up from the working directory
    * looking for {@code ihmc-alex-sdk/alex-models}. Each candidate is checked for a file that must
    * be there, so a stale or half-cloned directory is rejected rather than accepted and then failing
    * later as "no meshes".
    * </p>
    */
   public static Optional<Path> findModelsDirectory()
   {
      for (String override : new String[] {System.getProperty(SYSTEM_PROPERTY), System.getenv(ENVIRONMENT_VARIABLE)})
      {
         if (override == null || override.isBlank())
            continue;

         Path candidate = Path.of(override);

         // Accept either the SDK checkout or the alex-models directory inside it; both are things a
         // person would reasonably set this to.
         for (Path probe : new Path[] {candidate, candidate.resolve("alex-models")})
         {
            if (isModelsDirectory(probe))
               return Optional.of(probe.toAbsolutePath().normalize());
         }
      }

      Path directory = Path.of("").toAbsolutePath();

      for (int height = 0; height <= SEARCH_HEIGHT && directory != null; height++, directory = directory.getParent())
      {
         Path candidate = directory.resolve("ihmc-alex-sdk").resolve("alex-models");

         if (isModelsDirectory(candidate))
            return Optional.of(candidate.normalize());
      }

      return Optional.empty();
   }

   /** {@code ihmc_hands_ros2/meshes}, the root the ability-hand hulls resolve under. */
   public static Optional<Path> findHandMeshesDirectory()
   {
      Path directory = Path.of("").toAbsolutePath();

      for (int height = 0; height <= SEARCH_HEIGHT && directory != null; height++, directory = directory.getParent())
      {
         Path candidate = directory.resolve("ihmc_hands_ros2").resolve("meshes");

         if (Files.isDirectory(candidate.resolve("abilityHand")))
            return Optional.of(candidate.normalize());
      }

      return Optional.empty();
   }

   /**
    * Every mesh root that could be found, in the order they should be searched.
    * <p>
    * Possibly empty: nothing here throws, because a missing SDK means "the robot draws bare", not
    * "the demonstration is invalid". The caller decides whether to say so.
    * </p>
    */
   public static List<String> meshResourceDirectories()
   {
      List<String> directories = new ArrayList<>(2);
      findModelsDirectory().ifPresent(path -> directories.add(path.toString()));
      findHandMeshesDirectory().ifPresent(path -> directories.add(path.toString()));
      return directories;
   }

   /**
    * A sentinel that must exist: the pelvis mesh the demonstration actually draws.
    * <p>
    * Checking for a specific file rather than for the directory means a partially-cloned or renamed
    * SDK is rejected here, where the message can say so, instead of surviving to become 264 silent
    * JavaFX import failures.
    * </p>
    */
   private static boolean isModelsDirectory(Path candidate)
   {
      return Files.isRegularFile(candidate.resolve("alex_virtual_description")
                                          .resolve("alex_v1_description")
                                          .resolve("meshes")
                                          .resolve("legs")
                                          .resolve("Pelvis.obj"));
   }

   /** One line for a console banner, saying what was found or what to set. */
   public static String describe()
   {
      Optional<Path> models = findModelsDirectory();

      if (models.isEmpty())
         return "ihmc-alex-sdk not found; the robot will draw as bare frames. Set -D" + SYSTEM_PROPERTY + " or $" + ENVIRONMENT_VARIABLE
               + " to the SDK checkout.";

      return "meshes from " + models.get() + (findHandMeshesDirectory().isEmpty() ? " (ability-hand meshes not found; hands will not draw)" : "");
   }
}
