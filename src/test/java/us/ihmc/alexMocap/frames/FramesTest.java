package us.ihmc.alexMocap.frames;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * F8 (FRAMEWORK.md §11) and the three-pelvis-frames hazard (§13).
 * <p>
 * Each test creates its frames with a unique name suffix. Euclid rejects duplicate frame names
 * under one parent, and sharing them between tests would make one test's tilt leak into another's.
 * </p>
 */
public class FramesTest
{
   private static final double EPSILON = 1.0e-12;

   /** FRAMEWORK.md §11's headline number, which is the reason F8 exists at all. */
   @Test
   public void testSevenMillimetresAtHalfADegree()
   {
      TiltMeasurement tilt = TiltMeasurement.fromTiltAngles(Math.toRadians(0.5), 0.0, TiltMeasurement.Method.PRECISION_LEVEL, "§11 worked example");

      assertEquals(0.5, Math.toDegrees(tilt.getTiltMagnitude()), 1.0e-9);
      assertEquals(0.8 * Math.sin(Math.toRadians(0.5)), tilt.getComHeightError(0.8), 1.0e-12);
      assertEquals(6.98e-3, tilt.getComHeightError(0.8), 1.0e-5, "§11: roughly 7 mm at θ = 0.5° with ||c|| = 0.8 m.");
   }

   /** The 0.1° target of §11, for contrast: the same lever arm, an order of magnitude less error. */
   @Test
   public void testTargetTiltKeepsTheErrorUnderTwoMillimetres()
   {
      TiltMeasurement tilt = TiltMeasurement.fromTiltAngles(Math.toRadians(0.1), 0.0, TiltMeasurement.Method.PLUMB_LINE, "§11 target");
      assertTrue(tilt.getComHeightError(0.8) < 1.5e-3, "Expected under 1.5 mm, got " + tilt.getComHeightError(0.8));
   }

   /** The correction must carry measured up onto +z, whichever way the tilt leans. */
   @Test
   public void testCorrectionCarriesMeasuredUpOntoZ()
   {
      double[][] tilts = {{0.5, 0.0}, {0.0, 0.5}, {-0.3, 0.4}, {0.2, -0.7}};

      for (double[] angles : tilts)
      {
         TiltMeasurement tilt = TiltMeasurement.fromTiltAngles(Math.toRadians(angles[0]),
                                                               Math.toRadians(angles[1]),
                                                               TiltMeasurement.Method.STATIC_IMU_AVERAGE,
                                                               "sweep");

         RigidBodyTransform correction = new RigidBodyTransform();
         tilt.packMotiveWorldToGravityAligned(correction);

         Vector3D up = new Vector3D(tilt.getUpInMotiveWorld());
         correction.transform(up);

         assertEquals(0.0, up.getX(), 1.0e-12, "tilt " + angles[0] + "," + angles[1]);
         assertEquals(0.0, up.getY(), 1.0e-12, "tilt " + angles[0] + "," + angles[1]);
         assertEquals(1.0, up.getZ(), 1.0e-12, "tilt " + angles[0] + "," + angles[1]);

         // A tilt constrains two DOF, not three. The correction must not invent heading.
         assertEquals(0.0, correction.getTranslation().norm(), EPSILON, "A tilt is an orientation error; it must not translate the world.");
      }
   }

   /**
    * The F8 injection/correction pair, expressed through the frame tree rather than through the
    * transform: a point that is level in truth reads as tilted in Motive's frame, and
    * {@code changeFrame} is the entire fix.
    */
   @Test
   public void testFrameTreeAppliesTheCorrectionOnChangeFrame()
   {
      TiltMeasurement tilt = TiltMeasurement.fromTiltAngles(Math.toRadians(0.5), 0.0, TiltMeasurement.Method.PRECISION_LEVEL, "injection");
      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(tilt, ReferenceFrame.getWorldFrame(), "_changeFrame");

      // A CoM 0.8 m along Motive's +z. In Wg it is not straight up any more.
      FramePoint3D com = new FramePoint3D(world.getMotiveWorld(), 0.0, 0.0, 0.8);
      com.changeFrame(world.getGravityAlignedWorld());

      assertEquals(0.8, com.norm(), 1.0e-12, "The correction is a rotation; it cannot change the distance.");
      assertEquals(0.8 * Math.cos(Math.toRadians(0.5)), com.getZ(), 1.0e-12);

      double heightError = 0.8 - com.getZ();
      assertTrue(heightError > 0.0);
      assertEquals(3.05e-5, heightError, 1.0e-6, "Height error for a CoM on the tilt axis' perpendicular is the second-order term.");
   }

   /** With no tilt the two frames coincide, so a correct pipeline is a no-op rather than a shift. */
   @Test
   public void testLevelWorldIsANoOp()
   {
      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("unit test"), ReferenceFrame.getWorldFrame(), "_level");

      FramePoint3D point = new FramePoint3D(world.getMotiveWorld(), 0.3, -0.4, 0.8);
      point.changeFrame(world.getGravityAlignedWorld());

      assertEquals(0.3, point.getX(), EPSILON);
      assertEquals(-0.4, point.getY(), EPSILON);
      assertEquals(0.8, point.getZ(), EPSILON);
      assertFalse(world.isTiltMeasured(), "An assumed-level world must not claim to be measured.");
   }

   /** §11 forbids assuming the tilt. The type cannot prevent it, but it can refuse to hide it. */
   @Test
   public void testAssumedLevelMustJustifyItselfAndAdmitsItIsNotMeasured()
   {
      assertThrows(IllegalArgumentException.class, () -> TiltMeasurement.assumedLevel(""));
      assertThrows(IllegalArgumentException.class, () -> TiltMeasurement.assumedLevel(null));

      TiltMeasurement assumed = TiltMeasurement.assumedLevel("gantry not yet instrumented");
      assertFalse(assumed.isMeasured());
      assertEquals(0.0, assumed.getTiltMagnitude(), EPSILON);
      assertTrue(assumed.toString().contains("ASSUMED_LEVEL"), "It has to be visible in a report: " + assumed);

      // And a measured tilt must not be constructible under the ASSUMED_LEVEL method.
      assertThrows(IllegalArgumentException.class,
                   () -> TiltMeasurement.fromMeasuredUp(new Vector3D(0, 0, 1), TiltMeasurement.Method.ASSUMED_LEVEL, "contradiction"));
   }

   /**
    * A tilt of 90° or more is an axis-order mismatch, not a ground-plane residual. Accepting it
    * would silently rotate the whole capture session.
    */
   @Test
   public void testGrosslyWrongUpDirectionIsRejected()
   {
      assertThrows(IllegalArgumentException.class,
                   () -> TiltMeasurement.fromMeasuredUp(new Vector3D(0, 0, -1), TiltMeasurement.Method.PLUMB_LINE, "upside down"));
      assertThrows(IllegalArgumentException.class,
                   () -> TiltMeasurement.fromMeasuredUp(new Vector3D(1, 0, 0), TiltMeasurement.Method.PLUMB_LINE, "z is not up"));
      assertThrows(IllegalArgumentException.class,
                   () -> TiltMeasurement.fromMeasuredUp(new Vector3D(0, 0, 0), TiltMeasurement.Method.PLUMB_LINE, "degenerate"));
   }

   /**
    * §13: the three pelvis frames chain in one order and only one. A point at the IMU origin, taken
    * out to the world, must equal cluster pose composed with Δ composed with the CAD offset.
    */
   @Test
   public void testPelvisTriadComposesInTheOneCorrectOrder()
   {
      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("unit test"), ReferenceFrame.getWorldFrame(), "_triad");
      PelvisFrameTriad triad = new PelvisFrameTriad(world, "_triad");

      RigidBodyTransform clusterPose = new RigidBodyTransform();
      clusterPose.getTranslation().set(1.0, 2.0, 3.0);
      clusterPose.getRotation().setToYawOrientation(Math.PI / 2.0);

      RigidBodyTransform delta = new RigidBodyTransform();
      delta.getTranslation().set(0.01, 0.02, -0.03);

      RigidBodyTransform baseToImu = new RigidBodyTransform();
      baseToImu.getTranslation().set(0.1, 0.0, 0.0);

      triad.setClusterPoseInMotiveWorld(clusterPose);
      triad.setClusterToBase(delta);
      triad.setBaseToImu(baseToImu, false);

      FramePoint3D imuOrigin = new FramePoint3D(triad.getImuFrame());
      imuOrigin.changeFrame(world.getMotiveWorld());

      RigidBodyTransform expected = new RigidBodyTransform(clusterPose);
      expected.multiply(delta);
      expected.multiply(baseToImu);

      assertEquals(expected.getTranslationX(), imuOrigin.getX(), 1.0e-12);
      assertEquals(expected.getTranslationY(), imuOrigin.getY(), 1.0e-12);
      assertEquals(expected.getTranslationZ(), imuOrigin.getZ(), 1.0e-12);
   }

   /** §13's cost-of-being-wrong number, as an assertion rather than as a worked example. */
   @Test
   public void testLeverArmVelocityErrorMatchesTheFrameworkExample()
   {
      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("unit test"), ReferenceFrame.getWorldFrame(), "_lever");
      PelvisFrameTriad triad = new PelvisFrameTriad(world, "_lever");

      RigidBodyTransform baseToImu = new RigidBodyTransform();
      baseToImu.getTranslation().set(0.1, 0.0, 0.0);
      triad.setBaseToImu(baseToImu, false);

      assertEquals(0.1, triad.getVelocityErrorFromLeverArm(1.0), 1.0e-12, "§13: ω × r at ω = 1 rad/s, r = 0.1 m is 0.1 m/s.");
      assertFalse(triad.isImuTransformVerified(), "The CAD number is unverified until someone says otherwise.");
      assertTrue(triad.toString().contains("UNVERIFIED"));
   }
}
