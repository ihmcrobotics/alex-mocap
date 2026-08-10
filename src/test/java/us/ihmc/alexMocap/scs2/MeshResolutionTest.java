package us.ihmc.alexMocap.scs2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.AlexSdkModels;
import us.ihmc.alexMocap.calibration.RobotCaptures;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.visual.VisualDefinition;

/**
 * Whether Alex's meshes actually resolve, headlessly.
 *
 * <h2>Why this is a test and not something you notice by looking</h2>
 * <p>
 * An unresolvable {@code package://} reference does not throw. It leaves
 * {@code ModelFileGeometryDefinition.getFileName()} null, which survives
 * {@code RobotDefinition.newInstance()} untouched -- that method never looks at geometry -- and only
 * becomes a {@code NullPointerException} much later, inside {@code SimulationSession.addRobot}. So
 * the entire headless test suite passes with every mesh broken, and the failure surfaces as a
 * visualizer crash or a robot drawn as bare coordinate frames.
 * </p>
 * <p>
 * That is exactly what happened here: the demo pointed at a copy of the URDF in a Python checkout
 * because the vendored copy had no meshes beside it, and when that copy moved, 264 meshes silently
 * stopped loading. Counting resolved references is the only way this gets caught by CI.
 * </p>
 *
 * <h2>Skipped, not failed, without the SDK</h2>
 * <p>
 * {@code ihmc-alex-sdk} is a sibling checkout, not a Gradle dependency, so a machine that has this
 * repository alone cannot run these. They are assumptions rather than failures because the project's
 * standing promise is that {@code ./gradlew build} needs nothing outside the repository -- and
 * breaking that to check a mesh path would be a bad trade.
 * </p>
 */
public class MeshResolutionTest
{
   /** Every mesh the legs-and-pelvis demonstration actually draws. */
   private static final List<String> DEMONSTRATION_MESHES = List.of("Pelvis.obj",
                                                                     "LeftThigh.obj",
                                                                     "RightThigh.obj",
                                                                     "LeftShin.obj",
                                                                     "RightShin.obj",
                                                                     "Foot.obj",
                                                                     "AnklePitchLink.obj");

   private static RobotDefinition load(List<String> resourceDirectories) throws Exception
   {
      Path urdf = RobotCaptures.alexUrdfPath();

      try (InputStream stream = Files.newInputStream(urdf))
      {
         return URDFTools.toRobotDefinition(URDFTools.loadURDFModel(stream, resourceDirectories, MeshResolutionTest.class.getClassLoader()));
      }
   }

   private static List<ModelFileGeometryDefinition> meshes(RobotDefinition robotDefinition)
   {
      List<ModelFileGeometryDefinition> found = new ArrayList<>();

      for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
      {
         for (VisualDefinition visual : body.getVisualDefinitions())
         {
            GeometryDefinition geometry = visual.getGeometryDefinition();

            if (geometry instanceof ModelFileGeometryDefinition mesh)
               found.add(mesh);
         }
      }

      return found;
   }

   /**
    * With the SDK as the mesh root, every mesh the demonstration draws resolves.
    * <p>
    * The ability-hand hulls are deliberately not required: they live in a different repository and
    * the legs demonstration does not draw them. See {@link #testAbilityHandMeshesNeedTheHandsRepository()}.
    * </p>
    */
   @Test
   public void testSdkResolvesEveryMeshTheDemonstrationDraws() throws Exception
   {
      Optional<Path> models = AlexSdkModels.findModelsDirectory();
      Assumptions.assumeTrue(models.isPresent(), "ihmc-alex-sdk not beside this checkout; " + AlexSdkModels.describe());

      List<ModelFileGeometryDefinition> meshes = meshes(load(List.of(models.get().toString())));
      assertTrue(meshes.size() > 20, "Expected a full-body model, found " + meshes.size() + " meshes.");

      List<String> unresolvedDemonstrationMeshes = new ArrayList<>();

      for (ModelFileGeometryDefinition mesh : meshes)
      {
         if (mesh.getFileName() != null)
            continue;

         // A null file name is the unresolved case. Only complain about the ones this project draws.
         for (String required : DEMONSTRATION_MESHES)
         {
            if (mesh.getName() != null && mesh.getName().contains(required))
               unresolvedDemonstrationMeshes.add(mesh.getName());
         }
      }

      assertEquals(List.of(), unresolvedDemonstrationMeshes, "These leg and pelvis meshes did not resolve under " + models.get() + ".");

      long resolved = meshes.stream().filter(mesh -> mesh.getFileName() != null).count();

      // Measured: 29 of 37 package:// references resolve under alex-models/. The 8 that do not are
      // ability-hand hulls. Asserting a floor rather than the exact count so that adding a mesh to
      // the SDK does not fail this, while losing the whole mesh root still does.
      assertTrue(resolved >= 25, "Only " + resolved + " of " + meshes.size() + " meshes resolved under " + models.get() + ".");
   }

   /**
    * Without any mesh root, nothing resolves -- which is the bug this guards against.
    * <p>
    * Worth asserting the failure explicitly. If SCS2 ever started resolving {@code package://}
    * against something implicit, the test above would keep passing for the wrong reason and this one
    * would tell us.
    * </p>
    */
   @Test
   public void testNothingResolvesWithoutAMeshRoot() throws Exception
   {
      List<ModelFileGeometryDefinition> meshes = meshes(load(List.of()));

      long resolved = meshes.stream().filter(mesh -> mesh.getFileName() != null).count();

      assertEquals(0, resolved, "No package:// reference should resolve with no resource directory supplied.");
   }

   /** The ability-hand hulls come from ihmc_hands_ros2, not from the SDK. */
   @Test
   public void testAbilityHandMeshesNeedTheHandsRepository() throws Exception
   {
      Optional<Path> models = AlexSdkModels.findModelsDirectory();
      Optional<Path> hands = AlexSdkModels.findHandMeshesDirectory();
      Assumptions.assumeTrue(models.isPresent() && hands.isPresent(), "Needs both ihmc-alex-sdk and ihmc_hands_ros2; " + AlexSdkModels.describe());

      long withSdkOnly = meshes(load(List.of(models.get().toString()))).stream().filter(mesh -> mesh.getFileName() != null).count();
      long withBoth = meshes(load(List.of(models.get().toString(), hands.get().toString()))).stream().filter(mesh -> mesh.getFileName() != null).count();

      assertTrue(withBoth > withSdkOnly,
                 "Adding " + hands.get() + " should resolve the ability-hand hulls; got " + withSdkOnly + " then " + withBoth + ".");
   }

   /** The locator must reject a directory that is not an SDK, rather than accept it and fail later. */
   @Test
   public void testLocatorRejectsAWrongDirectory()
   {
      String previous = System.getProperty(AlexSdkModels.SYSTEM_PROPERTY);

      try
      {
         System.setProperty(AlexSdkModels.SYSTEM_PROPERTY, System.getProperty("java.io.tmpdir"));
         // Falls through to the upward walk, which may still find the real SDK -- so the assertion
         // is only that the bogus override was not itself accepted.
         Optional<Path> found = AlexSdkModels.findModelsDirectory();
         assertTrue(found.isEmpty() || !found.get().startsWith(System.getProperty("java.io.tmpdir")),
                    "A directory with no Pelvis.obj must not be accepted as the SDK.");
      }
      finally
      {
         if (previous == null)
            System.clearProperty(AlexSdkModels.SYSTEM_PROPERTY);
         else
            System.setProperty(AlexSdkModels.SYSTEM_PROPERTY, previous);
      }
   }
}
