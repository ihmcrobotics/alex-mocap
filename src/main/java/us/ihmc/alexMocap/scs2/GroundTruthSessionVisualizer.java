package us.ihmc.alexMocap.scs2;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.GroundTruthSample;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Opens SCS2 on a computed ground-truth trajectory: the robot at each logged configuration, with
 * the CoM and pelvis pose drawn over it and every conditioning variable available to plot.
 *
 * <h2>It replays; it does not simulate</h2>
 * <p>
 * The session is built with {@code newDoNothingPhysicsEngineFactory()}. That is the whole trick:
 * SCS2's session machinery gives the timeline, the buffer, the plotting and the 3-D view for free,
 * while the physics engine that would otherwise integrate the robot forward is replaced by nothing
 * at all. Each tick simply pushes one logged sample into the YoVariables and sets the robot's
 * joints to that capture's encoder reading. A simulating session would drop the robot on the floor
 * and overwrite the very poses being inspected.
 * </p>
 *
 * <h2>No test, by design</h2>
 * <p>
 * PR_PLAN.md: "Visualiser opens. It gets no test -- if a JavaFX window does not appear you will
 * know within seconds." Everything worth asserting about the data has already been asserted
 * headlessly by the time it reaches this class, and a test that starts a toolkit would make the
 * suite need a display, which is the one thing FRAMEWORK.md §19 spends a dependency rule to
 * prevent.
 * </p>
 * <p>
 * This is also the only class in the project that touches JavaFX. {@code PackageDependencyTest}
 * enforces that, so the rest of the pipeline stays runnable over SSH.
 * </p>
 */
public class GroundTruthSessionVisualizer
{
   private GroundTruthSessionVisualizer()
   {
   }

   /**
    * Opens the visualizer, plays the trajectory through once, and <b>blocks until the window is
    * closed</b>.
    *
    * @param urdfFile            the robot to draw.
    * @param samples             the computed ground truth, in order.
    * @param encoderSamples      the matching encoder readings, used to pose the robot.
    * @param gravityAlignedWorld {@code Wg}: the frame the CoM and pelvis pose are expressed in.
    * @param sampleRateHz        playback rate.
    */
   public static void show(Path urdfFile,
                           List<GroundTruthSample> samples,
                           List<EncoderSample> encoderSamples,
                           ReferenceFrame gravityAlignedWorld,
                           double sampleRateHz)
         throws IOException
   {
      if (samples.isEmpty())
         throw new IllegalArgumentException("Nothing to show: the trajectory is empty.");
      if (samples.size() != encoderSamples.size())
         throw new IllegalArgumentException("Got " + samples.size() + " ground truth samples and " + encoderSamples.size() + " encoder samples.");

      RobotDefinition robotDefinition = loadRobotDefinition(urdfFile);
      dropUnrepresentableGeometry(robotDefinition, urdfFile);

      SimulationSession session = new SimulationSession(PhysicsEngineFactory.newDoNothingPhysicsEngineFactory());
      session.setSessionDTSeconds(1.0 / sampleRateHz);

      Robot robot = session.addRobot(robotDefinition);

      GroundTruthYoVariables variables = new GroundTruthYoVariables("gt", samples.get(0).getLinkNames(), gravityAlignedWorld);
      session.getRootRegistry().addChild(variables.getRegistry());
      session.addYoGraphicDefinition(GroundTruthYoGraphics.create("groundTruth", variables));

      int[] sampleIndex = {0};

      session.addBeforePhysicsCallback(time ->
      {
         int index = Math.min(sampleIndex[0], samples.size() - 1);

         variables.update(samples.get(index));
         setRobotConfiguration(robot, encoderSamples.get(index));

         // Stop on the last sample instead of spinning on it forever. Without this the session
         // keeps ticking, re-publishing the final capture, and the buffer fills with copies of one
         // frame -- which looks like a frozen robot rather than a finished replay.
         if (index >= samples.size() - 1)
            session.setSessionMode(SessionMode.PAUSE);
         else
            sampleIndex[0] = index + 1;
      });

      SessionVisualizerControls controls = SessionVisualizer.startSessionVisualizer(session);
      controls.waitUntilVisualizerFullyUp();

      // Room for the whole trajectory, so every capture stays scrubbable rather than scrolling out
      // of a ring buffer sized for something else.
      session.submitBufferSizeRequestAndWait(samples.size() + 1);

      // Play it through once on open. The alternative is a window showing capture 0 and a play
      // button, which makes a working replay look like a static screenshot.
      session.setSessionMode(SessionMode.RUNNING);

      // THE BLOCKING CALL, and the reason this method exists rather than being three lines inline.
      //
      // startSessionVisualizer returns as soon as the toolkit is up -- it does NOT wait for the
      // window to close. Returning here would hand control back to a caller that then reaches the
      // end of main and calls System.exit, killing the JVM and the window with it. The symptom is
      // a console that prints its whole report and exits while the user is still waiting for a
      // window to appear, which reads as "the visualizer silently does nothing".
      controls.waitUntilVisualizerDown();
   }

   /**
    * Removes every visual and collision shape whose geometry SCS2 could not build, and says how many.
    *
    * <h2>Why this is needed, and why only here</h2>
    * <p>
    * SCS2's URDF parser has no representation for some geometry a real URDF contains. When it meets
    * one it returns a {@code null} {@code GeometryDefinition} and stores it in the
    * {@code CollisionShapeDefinition} without dereferencing it — no warning, no exception. The
    * kinematics path never notices, because {@code RobotDefinition.newInstance} does not touch
    * geometry at all. {@code SimulationSession.addRobot} does: it builds collidables for the physics
    * engine, and {@code CollisionTools.toFrameShape3D} dereferences the null.
    * </p>
    * <p>
    * On Alex that is 11 {@code <capsule>} collision shapes — capsule is an SDF/IHMC extension, not
    * standard URDF geometry, and SCS2's item set has {@code URDFBox}, {@code URDFCylinder},
    * {@code URDFSphere} and {@code URDFMesh} only. The 17 ability-hand meshes missing from disk fail
    * the same way, by a different route.
    * </p>
    * <p>
    * The fix is deliberately surgical: drop <b>only</b> the shapes with no geometry, not all visuals.
    * Alex's other 29 meshes resolve fine, so the robot still renders as a robot rather than as a
    * collection of coordinate frames. Nothing here affects the pipeline — visuals and collisions are
    * for looking at, and F1 through F11 never read either.
    * </p>
    * <p>
    * It prints what it dropped. Silently discarding part of a robot model is how someone later spends
    * an afternoon wondering why a link has no shape.
    * </p>
    */
   private static void dropUnrepresentableGeometry(RobotDefinition robotDefinition, Path urdfFile)
   {
      int[] droppedCollisions = {0};
      int[] droppedVisuals = {0};
      List<String> affectedLinks = new ArrayList<>();

      robotDefinition.forEachRigidBodyDefinition(body ->
      {
         int before = body.getCollisionShapeDefinitions().size() + body.getVisualDefinitions().size();

         int collisionsBefore = body.getCollisionShapeDefinitions().size();
         body.getCollisionShapeDefinitions().removeIf(shape -> shape == null || isUnbuildable(shape.getGeometryDefinition()));
         droppedCollisions[0] += collisionsBefore - body.getCollisionShapeDefinitions().size();

         int visualsBefore = body.getVisualDefinitions().size();
         body.getVisualDefinitions().removeIf(visual -> visual == null || isUnbuildable(visual.getGeometryDefinition()));
         droppedVisuals[0] += visualsBefore - body.getVisualDefinitions().size();

         if (before != body.getCollisionShapeDefinitions().size() + body.getVisualDefinitions().size())
            affectedLinks.add(body.getName());
      });

      if (droppedCollisions[0] + droppedVisuals[0] == 0)
         return;

      System.out.println("visualizer: dropped " + droppedCollisions[0] + " collision shape(s) and " + droppedVisuals[0]
            + " visual(s) from " + urdfFile.getFileName() + " that SCS2 could not build geometry for.");
      System.out.println("            affected links: " + String.join(", ", affectedLinks));
      System.out.println("            Usually <capsule> (an SDF extension SCS2 does not parse) or a mesh missing from disk.");
      System.out.println("            Appearance only — the calibration and runtime never read visual or collision geometry.");
   }

   /**
    * Whether SCS2 will fail to turn this geometry into a shape. Two distinct causes, both of which
    * Alex exhibits and neither of which announces itself at parse time.
    *
    * <ol>
    * <li><b>The geometry is null.</b> The URDF used an element SCS2 has no item class for --
    * {@code <capsule>} is an SDF/IHMC extension, and the parser's set is box, cylinder, sphere and
    * mesh. JAXB skips the unknown element non-fatally, {@code toGeometryDefinition} returns null,
    * and {@code CollisionShapeDefinition} stores it. {@code toFrameShape3D} then falls through its
    * whole {@code instanceof} chain and dereferences the null in its own "unhandled geometry type"
    * warning. 11 shapes on Alex.</li>
    * <li><b>A mesh whose file could not be resolved.</b> {@code SDFTools.tryToConvertToPath} prints
    * to stderr and returns null, leaving a {@code ModelFileGeometryDefinition} with a null file
    * name. It survives the {@code instanceof} chain and dies one line earlier, on
    * {@code getFileName().toLowerCase()}. On Alex these are the ability-hand meshes, referenced as
    * {@code package://abilityHand/*.obj} and as bare relative paths, and deliberately not
    * vendored.</li>
    * </ol>
    *
    * <p>
    * The first was found by running it; the second was hiding directly underneath and only appeared
    * once the first was fixed. That ordering is worth remembering — the fix for a null-geometry bug
    * is rarely a null check on the geometry alone.
    * </p>
    */
   private static boolean isUnbuildable(GeometryDefinition geometry)
   {
      if (geometry == null)
         return true;

      if (geometry instanceof ModelFileGeometryDefinition model)
         return model.getFileName() == null || model.getFileName().isBlank();

      return false;
   }

   /** Poses the drawn robot at one capture's encoder reading. */
   private static void setRobotConfiguration(Robot robot, EncoderSample encoderSample)
   {
      for (int i = 0; i < encoderSample.getJointCount(); i++)
      {
         String jointName = encoderSample.getJointNames().get(i);

         if (robot.getJoint(jointName) instanceof OneDoFJointBasics joint)
            joint.setQ(encoderSample.getQ(i));
      }

      robot.getRootBody().updateFramesRecursively();
   }

   private static RobotDefinition loadRobotDefinition(Path urdfFile) throws IOException
   {
      // Loaded here rather than through model.URDFLoader on purpose: that class returns a Mecano
      // tree and deliberately lets no SCS2 type escape into a signature, which is what keeps the
      // rest of the project free of this dependency. The visualizer is the one place a
      // RobotDefinition is the thing actually wanted.
      // Two things the convenient File overload does not do, both needed for meshes to load.
      //
      // It passes NO resource directories, so `package://` has nothing to resolve against. Alex's
      // meshes sit in <urdf folder>/alex_virtual_description/, and the authority in
      // `package://alex_virtual_description/...` is that directory's own name -- so the URDF's
      // parent folder is exactly the right root.
      //
      // And it passes a NULL ClassLoader, which is the one that actually bites: the resolver needs
      // a loader before it will consult anything, so with the File overload every mesh fails no
      // matter what directories are supplied. Verified directly against SDFTools.tryToConvertToPath
      // -- the same URI and directory resolve with a loader and fail without one.
      //
      // The cost of getting this wrong is not an error. It is 56 silently dropped visuals and a
      // robot drawn as bare coordinate frames, which reads as "the model is broken".
      List<String> resourceDirectories = urdfFile.getParent() == null ? List.of() : List.of(urdfFile.getParent().toString());

      try (InputStream urdfStream = Files.newInputStream(urdfFile))
      {
         return URDFTools.toRobotDefinition(URDFTools.loadURDFModel(urdfStream, resourceDirectories, GroundTruthSessionVisualizer.class.getClassLoader()));
      }
      catch (Exception e)
      {
         throw new IOException("Failed to load the URDF at " + urdfFile.toAbsolutePath() + " for visualisation: " + e, e);
      }
   }
}
