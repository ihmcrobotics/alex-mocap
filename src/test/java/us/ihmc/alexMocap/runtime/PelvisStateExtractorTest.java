package us.ihmc.alexMocap.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.frames.GravityAlignedWorldFrame;
import us.ihmc.alexMocap.frames.TiltMeasurement;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * F10 (FRAMEWORK.md §13), and above all the thing it must <b>not</b> have.
 */
public class PelvisStateExtractorTest
{
   /**
    * <b>By reflection: no method here returns a velocity or a twist.</b>
    * <p>
    * §13's reasoning: raw single differencing at 200 Hz with {@code σ = 0.93 mm} gives ~0.13 m/s
    * against ContactNet baselines of 0.0844 and 0.0254 m/s. A centred window cannot execute
    * causally, so there is no correct runtime velocity to expose -- and if the class exposed one
    * anyway, somebody would call it and read a fivefold estimator regression that was not there.
    * </p>
    * <p>
    * A comment saying so is not enforcement. This test is, and it is written against names and
    * return types rather than against a fixed method list so that it also catches a
    * plausibly-named addition nobody thought of.
    * </p>
    */
   @Test
   public void testExposesNoVelocityOrTwist()
   {
      List<String> offenders = new ArrayList<>();

      String[] forbiddenNameFragments = {"velocity", "twist", "speed", "rate", "derivative", "vel"};
      String[] forbiddenTypeFragments = {"twist", "spatialvector", "velocity"};

      for (Method method : PelvisStateExtractor.class.getMethods())
      {
         if (method.getDeclaringClass() != PelvisStateExtractor.class)
            continue;

         String name = method.getName().toLowerCase(Locale.ROOT);
         String returnType = method.getReturnType().getSimpleName().toLowerCase(Locale.ROOT);

         for (String fragment : forbiddenNameFragments)
         {
            if (name.contains(fragment))
               offenders.add(method.getName() + " (name suggests a velocity)");
         }

         for (String fragment : forbiddenTypeFragments)
         {
            if (returnType.contains(fragment))
               offenders.add(method.getName() + " returns " + method.getReturnType().getSimpleName());
         }
      }

      assertTrue(offenders.isEmpty(),
                 "PelvisStateExtractor must expose no velocity at all (FRAMEWORK.md §13). Found: " + offenders
                       + "\nVelocity belongs in postprocess.PelvisTwistEstimator, offline, over the logged poses.");
   }

   /** The pose is reported in Wg, so the F8 correction cannot be forgotten by a caller. */
   @Test
   public void testPoseIsReportedInTheGravityAlignedFrame()
   {
      TiltMeasurement tilt = TiltMeasurement.fromTiltAngles(Math.toRadians(0.5), 0.0, TiltMeasurement.Method.PRECISION_LEVEL, "test");
      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(tilt, ReferenceFrame.getWorldFrame(), "_pelvisState");

      List<String> linkNames = List.of("pelvis", "l_thigh");
      PelvisStateExtractor extractor = new PelvisStateExtractor("pelvis", linkNames, world.getGravityAlignedWorld(), world.getMotiveWorld());

      assertEquals(world.getGravityAlignedWorld(), extractor.getPelvisFrame());

      MeasuredLinkPoses poses = new MeasuredLinkPoses(linkNames);
      RigidBodyTransform inMotiveWorld = new RigidBodyTransform();
      inMotiveWorld.getTranslation().set(0.0, 0.0, 1.0);
      poses.setMeasured(0, inMotiveWorld, 1.0e-3, 4);

      assertTrue(extractor.extract(poses));

      // A point 1 m up Motive's z is not 1 m up in Wg when the world is tilted by 0.5°.
      //
      // The displacement is in y, not x: a tilt *about x* rotates the z axis toward y. Getting this
      // backwards is easy and is exactly the kind of axis slip §13's named frames exist to catch.
      assertEquals(Math.cos(Math.toRadians(0.5)), extractor.getPelvisPose().getTranslationZ(), 1.0e-12);
      assertEquals(-Math.sin(Math.toRadians(0.5)), extractor.getPelvisPose().getTranslationY(), 1.0e-12);
      assertEquals(0.0, extractor.getPelvisPose().getTranslationX(), 1.0e-12, "A tilt about x cannot move anything along x.");
   }

   /** A refused pelvis cluster gives no pose, and says so rather than returning the last one. */
   @Test
   public void testRefusedPelvisGivesNoPose()
   {
      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("test"), ReferenceFrame.getWorldFrame(), "_pelvisRefusal");

      List<String> linkNames = List.of("pelvis", "l_thigh");
      PelvisStateExtractor extractor = new PelvisStateExtractor("pelvis", linkNames, world.getGravityAlignedWorld(), world.getMotiveWorld());

      MeasuredLinkPoses poses = new MeasuredLinkPoses(linkNames);
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.getTranslation().set(1.0, 2.0, 3.0);
      poses.setMeasured(0, pose, 1.0e-3, 4);

      assertTrue(extractor.extract(poses));
      assertEquals(1.0, extractor.getPelvisPose().getTranslationX(), 1.0e-12);

      // Now refuse it. The previous pose must not survive.
      poses.setRefused(0, Double.NaN, 2, "only 2 markers visible");

      assertFalse(extractor.extract(poses));
      assertFalse(extractor.isPoseAvailable());
      assertTrue(Double.isNaN(extractor.getPelvisPose().getTranslationX()), "A refused pose must be NaN, not the last good one.");
   }
}
