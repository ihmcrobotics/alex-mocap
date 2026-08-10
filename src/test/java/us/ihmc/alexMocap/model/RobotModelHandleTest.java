package us.ihmc.alexMocap.model;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * F1 (FRAMEWORK.md §3). The URDF is trusted, so there is no algorithm here to get wrong -- what
 * these tests pin down is the <b>frame conventions</b>, which are where this class can be wrong in
 * a way nothing downstream would notice.
 */
public class RobotModelHandleTest
{
   private static final double EPSILON = 1.0e-12;

   private static Path toyURDF() throws Exception
   {
      return Path.of(RobotModelHandleTest.class.getResource("toy6dof.urdf").toURI());
    }

   private static RobotModelHandle toyModel() throws Exception
   {
      return RobotModelHandle.fromURDF(toyURDF());
   }

   @Test
   public void testLoadsToyRobot() throws Exception
   {
      RobotModelHandle model = toyModel();

      assertEquals(List.of("l_hip", "l_knee", "l_ankle", "r_hip", "r_knee", "r_ankle"), model.getJointNames());
      assertEquals(6, model.getJointCount(), "PR_PLAN.md asks for a toy 6-DOF URDF.");
      assertEquals("pelvis", model.getBaseLinkName());
      assertEquals(28.0, model.getTotalMass(), EPSILON);
   }

   /**
    * The synthetic {@code rootBody} SCS2 inserts is not a URDF link and has no link frame. If it
    * leaked into the link map, a {@code MarkerCluster} naming it would resolve to scaffolding.
    */
   @Test
   public void testSyntheticRootBodyIsNotAUrdfLink() throws Exception
   {
      RobotModelHandle model = toyModel();

      assertEquals("rootBody", model.getRootBody().getName(), "If SCS2 stops naming it this, the assertions below stop meaning anything.");
      assertFalse(model.hasLink("rootBody"));
      assertEquals(List.of("pelvis", "l_thigh", "l_shank", "l_foot", "r_thigh", "r_shank", "r_foot"), model.getLinkNames());
   }

   /**
    * <b>The property that makes F1 a valid FK reference at all.</b>
    * <p>
    * FRAMEWORK.md §0 answers the "apparent circularity" objection by asserting that
    * {@code ^b T_i(q)} is a function of joint angles alone -- it contains no mocap and does not
    * depend on where the robot is in the room. SCS2 quietly attaches the URDF root beneath a
    * {@code SixDoFJoint}, so that claim is not automatic: it holds only because {@code b} is
    * defined as the frame after that joint.
    * </p>
    * <p>
    * Moving the floating joint to a random pose is exactly "put the robot somewhere else in the
    * room". Every {@code ^b T_i} must be bit-for-bit unchanged. If this test ever fails, the FK
    * reference has become a function of the base pose and the whole calibration is circular in
    * precisely the way §0 denies.
    * </p>
    */
   @Test
   public void testLinkToBaseIsIndependentOfTheFloatingJoint() throws Exception
   {
      Random random = new Random(20260809L);
      RobotModelHandle model = toyModel();

      double[] q = {0.31, 0.72, -0.15, -0.44, 0.93, 0.22};
      model.setQ(q);
      model.updateFrames();

      RigidBodyTransform[] before = new RigidBodyTransform[model.getLinkNames().size()];

      for (int i = 0; i < model.getLinkNames().size(); i++)
      {
         before[i] = new RigidBodyTransform();
         model.packLinkToBase(model.getLinkNames().get(i), before[i]);
      }

      // The floating joint: the single joint between the synthetic root and the URDF root link.
      JointBasics floatingJoint = model.getRootBody().getChildrenJoints().get(0);

      for (int trial = 0; trial < 20; trial++)
      {
         floatingJoint.setJointConfiguration(EuclidCoreRandomTools.nextRigidBodyTransform(random));
         model.updateFrames();

         RigidBodyTransform after = new RigidBodyTransform();

         for (int i = 0; i < model.getLinkNames().size(); i++)
         {
            model.packLinkToBase(model.getLinkNames().get(i), after);
            assertTrue(before[i].epsilonEquals(after, EPSILON),
                       "^b T_" + model.getLinkNames().get(i) + " moved when the base pose changed. FRAMEWORK.md §0 requires it to depend on q alone."
                             + "\n  before: " + before[i] + "\n  after:  " + after);
         }
      }
   }

   /**
    * The link frame is the URDF link frame, not Mecano's centre-of-mass frame.
    * <p>
    * At {@code q = 0} the {@code l_thigh} link frame sits at its parent joint's origin,
    * {@code (0, 0.09, -0.02)} straight out of the URDF. Its CoM frame sits 0.15 m further down.
    * Reading {@code getBodyFixedFrame()} -- which FRAMEWORK.md §3 literally instructs -- would
    * return the latter and make {@code ^i c_i} identically zero, deleting the {@code δ(^i c_i)}
    * term from §14's error budget.
    * </p>
    */
   @Test
   public void testLinkFrameIsTheUrdfLinkFrameNotTheCenterOfMassFrame() throws Exception
   {
      RobotModelHandle model = toyModel();
      model.setQ(new double[6]);
      model.updateFrames();

      RigidBodyTransform linkToBase = new RigidBodyTransform();
      model.packLinkToBase("l_thigh", linkToBase);

      assertEquals(0.00, linkToBase.getTranslationX(), EPSILON);
      assertEquals(0.09, linkToBase.getTranslationY(), EPSILON, "The l_hip joint origin, straight from the URDF.");
      assertEquals(-0.02, linkToBase.getTranslationZ(), EPSILON);

      // And the CoM is a real, non-zero vector in that frame -- which is what §12 and §14 need.
      Point3D centerOfMass = new Point3D();
      model.packCenterOfMassInLinkFrame("l_thigh", centerOfMass);
      assertEquals(0.0, centerOfMass.getX(), EPSILON);
      assertEquals(0.0, centerOfMass.getY(), EPSILON);
      assertEquals(-0.15, centerOfMass.getZ(), EPSILON, "The l_thigh inertial origin, straight from the URDF.");

      RigidBodyBasics thigh = model.getLink("l_thigh");
      assertNotNull(thigh.getInertia());
      assertEquals(5.0, model.getMass("l_thigh"), EPSILON);
   }

   /** FK against hand-computed URDF arithmetic, so a Mecano upgrade that changes conventions fails here. */
   @Test
   public void testForwardKinematicsAtZeroMatchesTheUrdfByHand() throws Exception
   {
      RobotModelHandle model = toyModel();
      model.setQ(new double[6]);
      model.updateFrames();

      RigidBodyTransform t = new RigidBodyTransform();

      // l_foot = l_hip(0, 0.09, -0.02) + l_knee(0,0,-0.30) + l_ankle(0,0,-0.28)
      model.packLinkToBase("l_foot", t);
      assertEquals(0.00, t.getTranslationX(), EPSILON);
      assertEquals(0.09, t.getTranslationY(), EPSILON);
      assertEquals(-0.60, t.getTranslationZ(), EPSILON);

      // r_foot = r_hip(0, -0.09, -0.02) + r_knee(0,0,-0.29) + r_ankle(0,0,-0.27)
      model.packLinkToBase("r_foot", t);
      assertEquals(0.00, t.getTranslationX(), EPSILON);
      assertEquals(-0.09, t.getTranslationY(), EPSILON);
      assertEquals(-0.58, t.getTranslationZ(), EPSILON, "Left and right lengths differ on purpose; see the URDF header.");
   }

   /** A rotation about the hip must move the foot, or the toy robot cannot identify anything. */
   @Test
   public void testHipRotationMovesTheFoot() throws Exception
   {
      RobotModelHandle model = toyModel();
      RigidBodyTransform t = new RigidBodyTransform();

      model.setQ(new double[] {0.5, 0.0, 0.0, 0.0, 0.0, 0.0});
      model.updateFrames();
      model.packLinkToBase("l_foot", t);

      // l_hip is a ROLL joint: R_x(θ) takes the hip-relative arm (0, 0, armZ) to
      // (0, -armZ·sinθ, armZ·cosθ). See the URDF header for why the hip rolls rather than pitches.
      double armZ = -0.58;
      assertEquals(0.0, t.getTranslationX(), EPSILON, "l_hip rotates about x; the foot must not leave its x plane.");
      assertEquals(0.09 - armZ * Math.sin(0.5), t.getTranslationY(), 1.0e-9);
      assertEquals(-0.02 + armZ * Math.cos(0.5), t.getTranslationZ(), 1.0e-9);

      // The other branch must not have moved.
      model.packLinkToBase("r_foot", t);
      assertEquals(-0.58, t.getTranslationZ(), EPSILON);
   }

   /**
    * A permuted joint vector produces a completely plausible FK result at the wrong configuration.
    * {@link RobotModelHandle#setConfiguration} is the only entry point that can catch it.
    */
   @Test
   public void testSetConfigurationRejectsAPermutedJointOrder() throws Exception
   {
      RobotModelHandle model = toyModel();

      EncoderSample correct = new EncoderSample(model.getJointNames());
      correct.setQ(new double[] {0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
      model.setConfiguration(correct);
      assertEquals(0.3, model.getQ(model.indexOfJoint("l_ankle")), EPSILON);

      EncoderSample permuted = new EncoderSample(List.of("r_hip", "r_knee", "r_ankle", "l_hip", "l_knee", "l_ankle"));
      permuted.setQ(new double[] {0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
      assertThrows(IllegalArgumentException.class, () -> model.setConfiguration(permuted));
   }

   @Test
   public void testUnknownLinkAndJointNamesFailLoudly() throws Exception
   {
      RobotModelHandle model = toyModel();

      assertThrows(IllegalArgumentException.class, () -> model.getLink("torso"));
      assertThrows(IllegalArgumentException.class, () -> model.getLinkFrame("torso"));
      assertThrows(IllegalArgumentException.class, () -> model.indexOfJoint("neck"));
   }

   @Test
   public void testSha256IsStableAndContentAddressed(@TempDir Path directory) throws Exception
   {
      String hash = URDFLoader.sha256(toyURDF());
      assertEquals(64, hash.length(), "SHA-256 as lowercase hex.");
      assertEquals(hash, URDFLoader.sha256(toyURDF()), "Provenance is worthless if the hash is not reproducible.");

      Path edited = directory.resolve("edited.urdf");
      Files.writeString(edited, Files.readString(toyURDF()).replace("value=\"10.0\"", "value=\"10.5\""));
      assertFalse(hash.equals(URDFLoader.sha256(edited)), "An edited-in-place URDF must not keep its hash; that is the whole point.");
   }

   @Test
   public void testMissingAndMalformedUrdfsReportSomethingActionable(@TempDir Path directory) throws Exception
   {
      Path missing = directory.resolve("nope.urdf");
      IOException notFound = assertThrows(IOException.class, () -> URDFLoader.load(missing));
      assertTrue(notFound.getMessage().contains("nope.urdf"), "The path belongs in the message: " + notFound.getMessage());

      // JAXB's UnmarshalException has a null message and hides the useful text in its cause. A
      // loader that reports getMessage() directly says "...: null" and tells you nothing.
      Path malformed = directory.resolve("malformed.urdf");
      Files.writeString(malformed, "<?xml version=\"1.0\"?>\n<robot name=\"broken\">\n  <link name=\"a\">\n</robot>\n");

      IOException parseFailure = assertThrows(IOException.class, () -> URDFLoader.load(malformed));
      assertTrue(parseFailure.getMessage().contains("malformed.urdf"), parseFailure.getMessage());
      assertFalse(parseFailure.getMessage().endsWith("null"), "The parse failure must carry the underlying reason: " + parseFailure.getMessage());
   }

   /**
    * A root link with nothing <i>articulated</i> below it gives F5 nothing to identify {@code Δ}
    * from (FRAMEWORK.md §7). A welded-on link is not a degree of freedom.
    * <p>
    * The URDF here has a joint, so it parses; it just has no one-DoF joint. That is the case the
    * loader's own guard exists for. A URDF with no {@code <joint>} element at all never reaches
    * the guard -- SCS2 dereferences a null joint list and throws first -- which is covered by
    * {@link #testUrdfWithNoJointElementAtAllStillFailsLoudly}.
    * </p>
    */
   @Test
   public void testUrdfWithOnlyFixedJointsIsRejected(@TempDir Path directory) throws Exception
   {
      Path welded = directory.resolve("welded.urdf");
      Files.writeString(welded,
                        """
                        <?xml version="1.0"?>
                        <robot name="welded">
                          <link name="pelvis">
                            <inertial>
                              <origin xyz="0 0 0" rpy="0 0 0"/>
                              <mass value="1.0"/>
                              <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
                            </inertial>
                          </link>
                          <joint name="welded_on" type="fixed">
                            <parent link="pelvis"/>
                            <child link="lump"/>
                            <origin xyz="0 0 -0.1" rpy="0 0 0"/>
                          </joint>
                          <link name="lump">
                            <inertial>
                              <origin xyz="0 0 0" rpy="0 0 0"/>
                              <mass value="0.5"/>
                              <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
                            </inertial>
                          </link>
                        </robot>
                        """);

      assertThrows(IllegalArgumentException.class, () -> URDFLoader.load(welded));
   }

   /** Degenerate, but it must not escape as a bare NullPointerException from inside SCS2. */
   @Test
   public void testUrdfWithNoJointElementAtAllStillFailsLoudly(@TempDir Path directory) throws Exception
   {
      Path lonely = directory.resolve("lonely.urdf");
      Files.writeString(lonely,
                        """
                        <?xml version="1.0"?>
                        <robot name="lonely">
                          <link name="pelvis">
                            <inertial>
                              <origin xyz="0 0 0" rpy="0 0 0"/>
                              <mass value="1.0"/>
                              <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
                            </inertial>
                          </link>
                        </robot>
                        """);

      IOException failure = assertThrows(IOException.class, () -> URDFLoader.load(lonely));
      assertTrue(failure.getMessage().contains("lonely.urdf"), failure.getMessage());
   }
}
