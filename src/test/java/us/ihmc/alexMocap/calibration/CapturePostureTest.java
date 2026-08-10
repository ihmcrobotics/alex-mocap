package us.ihmc.alexMocap.calibration;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * What the generated capture postures look like geometrically.
 *
 * <h2>Why a test about appearance</h2>
 * <p>
 * Uniform independent draws across each joint's full URDF range produce configurations that are
 * legal, well conditioned, and absurd: Alex's {@code HIP_Y} range is {@code [-150°, +45°]} and its
 * {@code KNEE_Y} is {@code [0°, 140°]}, so a draw routinely folds one thigh against the chest while
 * the other kicks forward. Rendered, it does not read as a robot on a gantry; it reads as a fault.
 * </p>
 * <p>
 * That is a real cost even though no number is wrong. The demonstration exists to be looked at, and
 * a picture that reads as broken sends the reader looking for a bug in the pipeline instead of at
 * the CoM. So "the feet hang below the pelvis" is pinned here as a property, with the numbers that
 * chose the default.
 * </p>
 *
 * <h2>Sweep about rest, not about the midpoint</h2>
 * <p>
 * The distinction is load-bearing on this robot. The midpoint of Alex's leg ranges is
 * {@code HIP_Y = -52.5°, KNEE_Y = +70°} -- a deep tuck -- so narrowing
 * {@code jointExcursionFraction} converges on a squat. {@code sweepAboutRest} centres on the rest
 * pose instead, where all-zeros puts the feet 0.890 m below the pelvis at a 0.240 m stance.
 * </p>
 */
public class CapturePostureTest
{
   /** The demonstration's default half-range, radians. */
   private static final double SWEEP = 0.45;

   /** Feet must stay at least this far below the pelvis. Measured worst case at SWEEP: 0.733 m. */
   private static final double MINIMUM_FOOT_DROP = 0.70;

   /** Widest tolerable stance. Measured worst case at SWEEP: 0.932 m. */
   private static final double MAXIMUM_STANCE_WIDTH = 1.00;

   private record Posture(double footDrop, double stanceWidth)
   {
   }

   private static Posture posture(RobotModelHandle model)
   {
      RigidBodyTransform toPelvis = new RigidBodyTransform(model.getLinkFrame("PELVIS_LINK").getTransformToRoot());
      toPelvis.invert();

      RigidBodyTransform left = new RigidBodyTransform(toPelvis);
      left.multiply(model.getLinkFrame("LEFT_FOOT").getTransformToRoot());

      RigidBodyTransform right = new RigidBodyTransform(toPelvis);
      right.multiply(model.getLinkFrame("RIGHT_FOOT").getTransformToRoot());

      return new Posture(-0.5 * (left.getTranslationZ() + right.getTranslationZ()),
                         Math.abs(left.getTranslationY() - right.getTranslationY()));
   }

   private static Posture worstPosture(RobotCaptures.Planted planted) throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      double worstDrop = Double.POSITIVE_INFINITY;
      double worstStance = 0.0;

      for (double[] q : planted.reportedJointAngles)
      {
         model.setQ(q);
         model.updateFrames();
         Posture posture = posture(model);
         worstDrop = Math.min(worstDrop, posture.footDrop());
         worstStance = Math.max(worstStance, posture.stanceWidth());
      }

      return new Posture(worstDrop, worstStance);
   }

   /** The rest pose is a robot hanging straight: this is what the sweep is centred on. */
   @Test
   public void testRestPoseHangsStraight() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      model.setQ(new double[model.getJointCount()]);
      model.updateFrames();

      Posture posture = posture(model);

      // Measured: 0.890 m below the pelvis at a 0.240 m stance.
      assertTrue(posture.footDrop() > 0.85, "Rest-pose foot drop was " + posture.footDrop() + " m; q = 0 should be a straight-legged robot.");
      assertTrue(posture.stanceWidth() < 0.35, "Rest-pose stance was " + posture.stanceWidth() + " m.");
   }

   /** <b>The property.</b> At the demonstration's default sweep, the robot always reads as hanging. */
   @Test
   public void testDefaultSweepKeepsTheFeetUnderTheRobot() throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(new RobotCaptures.Options().captures(60).noise(0.0).sweepAboutRest(SWEEP));

      Posture worst = worstPosture(planted);

      assertTrue(worst.footDrop() > MINIMUM_FOOT_DROP,
                 "Worst foot drop was " + worst.footDrop() + " m, below " + MINIMUM_FOOT_DROP + " m -- the robot stops looking like it is hanging.");
      assertTrue(worst.stanceWidth() < MAXIMUM_STANCE_WIDTH, "Worst stance width was " + worst.stanceWidth() + " m.");
   }

   /**
    * And the full-range sweep does not, which is what makes the test above worth having.
    * <p>
    * Without this, a change that quietly made every sweep narrow would leave the property test
    * passing and prove nothing.
    * </p>
    */
   @Test
   public void testFullRangeSweepProducesPosturesNoOperatorWouldCommand() throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(new RobotCaptures.Options().captures(60).noise(0.0));

      Posture worst = worstPosture(planted);

      // Measured over the demonstration's 60 captures: a foot ends up 0.18 m below the pelvis.
      assertTrue(worst.footDrop() < MINIMUM_FOOT_DROP,
                 "Full-range draws gave a worst foot drop of " + worst.footDrop() + " m. If this is now above " + MINIMUM_FOOT_DROP
                       + " m the sampler changed, and the default sweep may no longer be buying anything.");
   }

   /** A one-sided joint gets a one-sided sweep rather than a window shifted past its stop. */
   @Test
   public void testKneeSweepStaysOnTheBendingSide() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      int knee = model.indexOfJoint("LEFT_KNEE_Y");

      // The URDF declares lower="0": the knee does not hyperextend, and rest sits exactly on it.
      assertTrue(model.getJointLimitLower(knee) == 0.0, "This test is about a joint whose rest angle is on a limit.");

      RobotCaptures.Planted planted = RobotCaptures.generate(new RobotCaptures.Options().captures(60).noise(0.0).sweepAboutRest(SWEEP));

      for (double[] q : planted.reportedJointAngles)
      {
         assertTrue(q[knee] >= 0.0 && q[knee] <= SWEEP + 1.0e-12,
                    "Knee angle " + q[knee] + " rad is outside [0, " + SWEEP + "]; the sweep window was shifted rather than clipped.");
      }
   }
}
