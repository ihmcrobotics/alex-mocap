package us.ihmc.alexMocap.scs2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.AlexSdkModels;
import us.ihmc.alexMocap.calibration.RobotCaptures;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Where the visualizer actually draws the robot.
 *
 * <h2>The bug this exists for</h2>
 * <p>
 * {@code setRobotConfiguration} used to set joint angles only, leaving the floating joint at
 * identity. The robot was therefore drawn at the world origin while the gold CoM sphere and the
 * pelvis triad -- which are in measured world coordinates -- appeared wherever the robot really was.
 * On the demonstration's capture set that is {@code (1.00, 1.99, 1.41)}: a suspended gantry pose
 * about 2.4 m from the origin. What you see is a robot hanging in space in a strange posture with
 * its ground truth nowhere near it.
 * </p>
 * <p>
 * It is a good example of this project's characteristic failure: nothing threw, no number was wrong,
 * every headless test passed, and the only symptom was a picture that looked odd. So this is
 * asserted on the real {@link Robot} SCS2 builds, not on a Mecano stand-in.
 * </p>
 *
 * <h2>What it does not cover</h2>
 * <p>
 * The robot is constructed directly rather than through {@code SimulationSession.addRobot}, so this
 * says nothing about the session wiring or about JavaFX rendering. It pins the placement arithmetic,
 * which is what was wrong.
 * </p>
 */
public class VisualizerPlacementTest
{
   private static final double EPSILON = 1.0e-10;

   private static Robot alexRobot() throws Exception
   {
      Optional<Path> models = AlexSdkModels.findModelsDirectory();
      Path urdf = RobotCaptures.alexUrdfPath();

      RobotDefinition definition;

      try (InputStream stream = Files.newInputStream(urdf))
      {
         List<String> roots = models.map(path -> List.of(path.toString())).orElse(List.of());
         definition = URDFTools.toRobotDefinition(URDFTools.loadURDFModel(stream, roots, VisualizerPlacementTest.class.getClassLoader()));
      }

      // Same guard the visualizer applies before handing the definition to SCS2: a geometry SCS2
      // has no class for is stored as null and dereferenced later.
      GroundTruthSessionVisualizer.dropUnrepresentableGeometry(definition, urdf);

      return new Robot(definition, ReferenceFrame.getWorldFrame());
   }

   private static EncoderSample zeroedEncoders(RobotModelHandle model)
   {
      EncoderSample encoders = new EncoderSample(model.getJointNames());
      encoders.setQ(new double[model.getJointCount()]);
      return encoders;
   }

   /** A pose nothing could produce by accident: off-origin on every axis, and rotated. */
   private static RigidBodyTransform gantryPose()
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.getTranslation().set(1.0007193215813908, 1.9894222357700524, 1.4122244606001086);
      pose.getRotation().setYawPitchRoll(0.31, -0.12, 0.07);
      return pose;
    }

   @Test
   public void testFloatingJointIsFoundAndIsNotAUrdfJoint() throws Exception
   {
      Robot robot = alexRobot();
      JointBasics floatingJoint = GroundTruthSessionVisualizer.findFloatingJoint(robot);

      assertNotNull(floatingJoint);

      // The joint SCS2 injects appears in no URDF, so its name must not be one of Alex's.
      RobotModelHandle model = RobotCaptures.alexModel();
      assertFalse(model.getJointNames().contains(floatingJoint.getName()),
                  "Found '" + floatingJoint.getName() + "', which is a real URDF joint -- placing the robot would bend it.");

      assertEquals("PELVIS_LINK", floatingJoint.getSuccessor().getName(), "The floating joint's successor must be the URDF root link.");
   }

   /**
    * <b>The regression.</b> After posing, the pelvis link frame must be exactly where the measured
    * pelvis pose said it was.
    */
   @Test
   public void testRobotIsDrawnAtTheMeasuredPelvisPose() throws Exception
   {
      Robot robot = alexRobot();
      JointBasics floatingJoint = GroundTruthSessionVisualizer.findFloatingJoint(robot);
      RobotModelHandle model = RobotCaptures.alexModel();

      // EncoderSample starts at NaN by project convention ("unset is NaN, never zero"), and SCS2's
      // SimRevoluteJoint rejects a NaN q outright. A real capture always has every column filled.
      EncoderSample encoders = zeroedEncoders(model);
      encoders.setQ(model.indexOfJoint("LEFT_KNEE_Y"), 0.7);

      RigidBodyTransform pelvisPose = gantryPose();

      GroundTruthSessionVisualizer.setRobotConfiguration(robot, floatingJoint, encoders, pelvisPose);

      RigidBodyTransform drawn = new RigidBodyTransform(robot.getRigidBody("PELVIS_LINK").getParentJoint().getFrameAfterJoint().getTransformToRoot());

      assertTrue(drawn.epsilonEquals(pelvisPose, EPSILON),
                 "The drawn pelvis is at " + drawn.getTranslation() + " but was measured at " + pelvisPose.getTranslation()
                       + ". Before the fix this was the origin.");

      // And the joint angle took effect, so the two halves of posing do not fight.
      assertEquals(0.7, ((OneDoFJointBasics) robot.getJoint("LEFT_KNEE_Y")).getQ(), EPSILON);

      // A leg link must have moved with the base rather than staying near the origin: the whole
      // point is that the robot travels, not just its root.
      double shinHeight = robot.getRigidBody("LEFT_SHIN").getParentJoint().getFrameAfterJoint().getTransformToRoot().getTranslationZ();
      assertTrue(shinHeight > 0.5, "The shin is at z = " + shinHeight + " m; the robot did not travel with its base.");
   }

   /**
    * A refused pelvis holds the last good pose instead of poisoning the frame tree.
    * <p>
    * NaN in a floating joint does not recover -- every later frame would draw nothing, so a single
    * bad capture would end the replay. The dropout stays visible through the CoM sphere vanishing.
    * </p>
    */
   @Test
   public void testRefusedPelvisHoldsTheLastPoseRatherThanPoisoningTheTree() throws Exception
   {
      Robot robot = alexRobot();
      JointBasics floatingJoint = GroundTruthSessionVisualizer.findFloatingJoint(robot);
      RobotModelHandle model = RobotCaptures.alexModel();
      EncoderSample encoders = zeroedEncoders(model);

      RigidBodyTransform good = gantryPose();
      GroundTruthSessionVisualizer.setRobotConfiguration(robot, floatingJoint, encoders, good);

      RigidBodyTransform refused = new RigidBodyTransform();
      refused.setToNaN();
      GroundTruthSessionVisualizer.setRobotConfiguration(robot, floatingJoint, encoders, refused);

      RigidBodyTransform drawn = new RigidBodyTransform(robot.getRigidBody("PELVIS_LINK").getParentJoint().getFrameAfterJoint().getTransformToRoot());

      assertFalse(drawn.containsNaN(), "A refused pelvis put NaN into the frame tree; the replay would draw nothing from here on.");
      assertTrue(drawn.epsilonEquals(good, EPSILON), "A refused pelvis should hold the last good pose.");
   }

   /** Sanity: the capture set really is off-origin, so the bug was worth fixing. */
   @Test
   public void testDemonstrationCapturesAreNowhereNearTheOrigin() throws Exception
   {
      Assumptions.assumeTrue(AlexSdkModels.findModelsDirectory().isPresent(), "needs the SDK only for a consistent model load");

      RobotCaptures.Planted planted = RobotCaptures.generate(new RobotCaptures.Options().captures(4).noise(0.0));

      for (RigidBodyTransform basePose : planted.basePoses)
      {
         assertTrue(basePose.getTranslation().norm() > 1.0,
                    "A capture base pose sits at " + basePose.getTranslation() + ", close enough to the origin that drawing the robot "
                          + "at identity would not have looked wrong.");
      }
   }
}
