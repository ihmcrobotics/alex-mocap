package us.ihmc.alexMocap.scs2;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.GroundTruthSample;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
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
   /**
    * The ghost's colour: pale cyan at 35 % opacity.
    * <p>
    * Cyan rather than a grey because the solid robot is already grey, and a translucent grey ghost
    * reads as a rendering artefact rather than as a second robot. 35 % is low enough to see the
    * solid robot through it and high enough to see the ghost against the sky.
    * </p>
    */
   private static final ColorDefinition GHOST_COLOR = new ColorDefinition(0.35, 0.85, 0.95, 0.35);

   private GroundTruthSessionVisualizer()
   {
   }

   /**
    * Repaints every visual on a robot in one translucent colour.
    * <p>
    * Materials are replaced rather than modified in place: a URDF visual may share a
    * {@code MaterialDefinition} instance between links, and mutating one would silently repaint the
    * solid robot too -- the two definitions are loaded separately but from the same file.
    * </p>
    */
   private static void makeTranslucent(RobotDefinition robotDefinition, ColorDefinition color)
   {
      for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
      {
         for (VisualDefinition visual : body.getVisualDefinitions())
         {
            MaterialDefinition material = new MaterialDefinition(color);
            material.setDiffuseColor(color);
            visual.setMaterialDefinition(material);
         }
      }
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
      show(urdfFile, List.of(), samples, encoderSamples, gravityAlignedWorld, sampleRateHz);
   }

   /**
    * As {@link #show(Path, List, List, ReferenceFrame, double)}, with the mesh roots given
    * explicitly.
    *
    * <h2>Why the mesh root is separable from the URDF</h2>
    * <p>
    * {@code package://} resolves against a directory whose <i>name</i> is the authority in the URI,
    * and nothing requires that directory to sit beside the URDF. On Alex it does not: the model this
    * project calibrates against is a vendored copy in test resources -- deliberately, so CI needs
    * nothing outside the repo -- while the meshes live in {@code ihmc-alex-sdk/alex-models/}. Tying
    * the two together forces a choice between a model whose bytes are pinned and a robot that draws,
    * and the single-root behaviour quietly took the second.
    * </p>
    * <p>
    * Splitting them means the same bytes, and the same {@code sha256} in the provenance, can be
    * drawn with the SDK's meshes. Measured against the vendored Alex URDF: 29 of its 37
    * {@code package://} references resolve under {@code alex-models/}, which is every leg, pelvis
    * and torso mesh. The 8 that do not are ability-hand hulls, which live under
    * {@code ihmc_hands_ros2/meshes/} -- pass that as a second root if the hands matter, or leave it
    * out and let them drop.
    * </p>
    *
    * @param resourceDirectories roots for {@code package://} lookups. Empty falls back to the URDF's
    *                            own parent directory, which is the right answer when the meshes do
    *                            sit beside it.
    */
   public static void show(Path urdfFile,
                           List<String> resourceDirectories,
                           List<GroundTruthSample> samples,
                           List<EncoderSample> encoderSamples,
                           ReferenceFrame gravityAlignedWorld,
                           double sampleRateHz)
         throws IOException
   {
      show(urdfFile, resourceDirectories, samples, encoderSamples, null, null, null, gravityAlignedWorld, sampleRateHz);
   }

   /**
    * The full view: the robot, the marker cloud it is measured by, and a ghost at the reconstructed
    * pose.
    *
    * <h2>Solid is truth, ghost is what mocap recovered</h2>
    * <p>
    * When {@code truthBasePoses} is supplied the solid robot is drawn at the <b>planted</b> base
    * pose and the translucent ghost at the <b>measured</b> one, so the separation between them is
    * the pipeline's pelvis error, to scale, in the same picture as the markers that produced it.
    * On a healthy leg marker set they sit on top of each other -- which is the result. Run
    * {@code AlexLegDemo --degenerate} and the ghost walks 56 mm off along x, which is the same
    * number the report prints, except that you can see it.
    * </p>
    * <p>
    * Without {@code truthBasePoses} there is no ghost and the single robot is drawn at the measured
    * pose, as before. A replay of real captures has no truth to draw.
    * </p>
    *
    * @param frames         one per sample, for the marker cloud. Null draws no markers.
    * @param clusters       the marker clusters, for colouring. Required when {@code frames} is given.
    * @param truthBasePoses one per sample, {@code ^Wg T_b} as planted. Null draws no ghost.
    */
   public static void show(Path urdfFile,
                           List<String> resourceDirectories,
                           List<GroundTruthSample> samples,
                           List<EncoderSample> encoderSamples,
                           List<MocapFrame> frames,
                           List<MarkerCluster> clusters,
                           List<RigidBodyTransformReadOnly> truthBasePoses,
                           ReferenceFrame gravityAlignedWorld,
                           double sampleRateHz)
         throws IOException
   {
      if (samples.isEmpty())
         throw new IllegalArgumentException("Nothing to show: the trajectory is empty.");
      if (samples.size() != encoderSamples.size())
         throw new IllegalArgumentException("Got " + samples.size() + " ground truth samples and " + encoderSamples.size() + " encoder samples.");
      if (frames != null && frames.size() != samples.size())
         throw new IllegalArgumentException("Got " + frames.size() + " mocap frames for " + samples.size() + " samples.");
      if (frames != null && clusters == null)
         throw new IllegalArgumentException("Marker frames were supplied without their clusters, so the markers cannot be grouped or coloured.");
      if (truthBasePoses != null && truthBasePoses.size() != samples.size())
         throw new IllegalArgumentException("Got " + truthBasePoses.size() + " truth base poses for " + samples.size() + " samples.");

      RobotDefinition robotDefinition = loadRobotDefinition(urdfFile, resourceDirectories);
      dropUnrepresentableGeometry(robotDefinition, urdfFile);

      SimulationSession session = new SimulationSession(PhysicsEngineFactory.newDoNothingPhysicsEngineFactory());
      session.setSessionDTSeconds(1.0 / sampleRateHz);

      Robot robot = session.addRobot(robotDefinition);
      JointBasics floatingJoint = findFloatingJoint(robot);

      // The ghost needs its own definition: addRobot consumes the one it is given, and the two
      // robots must differ in material anyway.
      Robot ghost = null;
      JointBasics ghostFloatingJoint = null;

      if (truthBasePoses != null)
      {
         RobotDefinition ghostDefinition = loadRobotDefinition(urdfFile, resourceDirectories);
         dropUnrepresentableGeometry(ghostDefinition, urdfFile);
         ghostDefinition.setName(ghostDefinition.getName() + "Ghost");
         makeTranslucent(ghostDefinition, GHOST_COLOR);
         ghost = session.addRobot(ghostDefinition);
         ghostFloatingJoint = findFloatingJoint(ghost);
      }

      GroundTruthYoVariables variables = new GroundTruthYoVariables("gt", samples.get(0).getLinkNames(), gravityAlignedWorld);
      session.getRootRegistry().addChild(variables.getRegistry());
      session.addYoGraphicDefinition(GroundTruthYoGraphics.create("groundTruth", variables));

      MocapMarkerYoVariables markerVariables = null;

      if (frames != null)
      {
         // The cloud is in the motive world: markers are raw observations and F8's tilt correction
         // is applied downstream of them. Drawing them in Wg would float the cloud off the robot.
         markerVariables = new MocapMarkerYoVariables("gt", clusters, frames.get(0).getMarkers(), ReferenceFrame.getWorldFrame());
         session.getRootRegistry().addChild(markerVariables.getRegistry());
         session.addYoGraphicDefinition(markerVariables.createYoGraphics("mocapMarkers"));
      }

      int[] sampleIndex = {0};
      Robot finalGhost = ghost;
      JointBasics finalGhostJoint = ghostFloatingJoint;
      MocapMarkerYoVariables finalMarkers = markerVariables;

      session.addBeforePhysicsCallback(time ->
      {
         int index = Math.min(sampleIndex[0], samples.size() - 1);

         variables.update(samples.get(index));

         if (finalMarkers != null)
            finalMarkers.update(frames.get(index));

         if (finalGhost != null)
         {
            // Solid robot at truth, ghost at the reconstruction.
            setRobotConfiguration(robot, floatingJoint, encoderSamples.get(index), truthBasePoses.get(index));
            setRobotConfiguration(finalGhost, finalGhostJoint, encoderSamples.get(index), samples.get(index).getPelvisPose());
         }
         else
         {
            setRobotConfiguration(robot, floatingJoint, encoderSamples.get(index), samples.get(index).getPelvisPose());
         }

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
   static void dropUnrepresentableGeometry(RobotDefinition robotDefinition, Path urdfFile)
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
   /**
    * The joint SCS2 inserts above the URDF root link. There is exactly one.
    * <p>
    * Found by type rather than by name or position. The name is SCS2's to choose, and
    * {@code getChildrenJoints().get(0)} would silently pick a real URDF joint if the tree were ever
    * built without the synthetic root -- placing the robot by moving its hip.
    * </p>
    */
   static JointBasics findFloatingJoint(Robot robot)
   {
      for (JointBasics joint : robot.getRootBody().childrenSubtreeIterable())
      {
         if (joint instanceof FloatingJointBasics)
            return joint;
      }

      throw new IllegalStateException("No floating joint in the robot SCS2 built from this URDF, so there is nothing to place the robot with. "
            + "Every URDF gets one -- see URDFLoader's note on the synthetic rootBody.");
   }

   /**
    * Poses the drawn robot: joint angles from the encoders, and <b>where it is</b> from the measured
    * pelvis.
    *
    * <h2>Why the base pose has to be set here</h2>
    * <p>
    * Setting only the joint angles leaves the floating joint at identity, which draws the robot at
    * the world origin. That is wrong in a way that is easy to miss and hard to interpret: the gold
    * CoM sphere and the pelvis triad <i>are</i> in measured world coordinates, so they appear
    * wherever the robot actually was -- on this capture set about 2.4 m away, at
    * {@code (1.00, 1.99, 1.41)} -- while the robot itself sits at the origin with its legs below the
    * grid. It reads as "the robot is floating in the air in a strange pose", which is a description
    * of the symptom and not of the fault.
    * </p>
    * <p>
    * {@code GroundTruthSample.getPelvisPose()} is {@code ^Wg T̂_b}, the URDF pelvis <b>link</b> frame
    * in the gravity-aligned world -- see {@code runtime.PelvisStateExtractor} on the
    * three-pelvis-frames hazard. That is exactly the transform the floating joint holds, so this is
    * a direct assignment and not a conversion.
    * </p>
    *
    * <h2>A refused pelvis holds the last pose</h2>
    * <p>
    * When F6 refuses the pelvis cluster the pose is NaN. Assigning that would put NaN into the
    * frame tree, where it stays -- every subsequent frame would draw nothing, so one bad capture
    * would end the replay. Holding the last good pose keeps the robot on screen, and the dropout is
    * still visible: the CoM sphere vanishes and {@code gtRefusedLinkCount} steps.
    * </p>
    */
   static void setRobotConfiguration(Robot robot,
                                             JointBasics floatingJoint,
                                             EncoderSample encoderSample,
                                             RigidBodyTransformReadOnly pelvisPose)
   {
      for (int i = 0; i < encoderSample.getJointCount(); i++)
      {
         String jointName = encoderSample.getJointNames().get(i);

         if (robot.getJoint(jointName) instanceof OneDoFJointBasics joint)
            joint.setQ(encoderSample.getQ(i));
      }

      if (!pelvisPose.containsNaN())
         floatingJoint.setJointConfiguration(pelvisPose);

      robot.getRootBody().updateFramesRecursively();
   }

   private static RobotDefinition loadRobotDefinition(Path urdfFile, List<String> resourceDirectories) throws IOException
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
      //
      // The caller's roots win when given; falling back to the URDF's parent keeps the old
      // behaviour for a model that does ship its meshes alongside.
      List<String> roots = resourceDirectories.isEmpty()
            ? (urdfFile.getParent() == null ? List.of() : List.of(urdfFile.getParent().toString()))
            : List.copyOf(resourceDirectories);

      try (InputStream urdfStream = Files.newInputStream(urdfFile))
      {
         return URDFTools.toRobotDefinition(URDFTools.loadURDFModel(urdfStream, roots, GroundTruthSessionVisualizer.class.getClassLoader()));
      }
      catch (Exception e)
      {
         throw new IOException("Failed to load the URDF at " + urdfFile.toAbsolutePath() + " for visualisation: " + e, e);
      }
   }
}
