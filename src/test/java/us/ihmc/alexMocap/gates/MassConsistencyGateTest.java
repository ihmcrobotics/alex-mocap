package us.ihmc.alexMocap.gates;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.AllocationMeasurement;
import us.ihmc.alexMocap.calibration.RobotCaptures;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * G5.
 * <p>
 * Two halves. The first tests the gate's own arithmetic against synthetic samples. The second tests
 * the <b>claims the gate rests on</b> against the real Alex URDF -- that a uniform mass rescale
 * cannot move the centre of mass, and that a link's detectability is set by its horizontal lever
 * arm. Those are the reasons G5 has the shape it does, and if either stopped holding the gate would
 * still pass every arithmetic test while measuring nothing.
 * </p>
 */
public class MassConsistencyGateTest
{
   /** The vendored Alex URDF's total, kg. Stated so a model swap fails here rather than silently. */
   private static final double ALEX_MASS = 91.512588;

   private static final double GRAVITY = 9.81;

   /** F11's mass term on Alex at the budget's assumed 5 %. The bar an offset has to clear. */
   private static final double EXPECTED_COM_UNCERTAINTY = 0.00490;

   private static final double MASS_UNCERTAINTY_FRACTION = 0.05;

   /** 3 x 4.90 mm. Written out so a change to the multiplier is visible as a test edit. */
   private static final double DISTRIBUTION_THRESHOLD = 3.0 * EXPECTED_COM_UNCERTAINTY;

   private static final int SAMPLES = 2000;

   // ------------------------------------------------------------------------------------------
   // The gate's arithmetic
   // ------------------------------------------------------------------------------------------

   /**
    * True negative: a model that agrees with the floor passes.
    * <p>
    * Centre-of-pressure noise is 1 mm per axis, well above a real plate, and over 2000 samples its
    * mean lands at 1 mm / sqrt(2000) = 0.022 mm against a 14.7 mm bar. This test should never be
    * marginal.
    * </p>
    */
   @Test
   public void testAgreeingModelPasses()
   {
      MassConsistencyGate gate = gate();
      accumulate(gate, new Random(1234L), SAMPLES, 0.0, 0.0, ALEX_MASS);

      GateResult result = gate.run();

      assertTrue(result.isPassed(), () -> "A model that agrees with the floor must pass:\n" + result);
      assertEquals(3, result.getFindings().size(), "Total mass, plus one finding per horizontal axis.");
      assertEquals(3, result.countWithStatus(GateResult.Status.PASS));
   }

   /**
    * True positive on weight: a robot lighter than its model fires the total-mass finding.
    * <p>
    * 74 kg against the URDF's 91.5 is the discrepancy that prompted this gate: 19 % against a 5 %
    * bar.
    * </p>
    */
   @Test
   public void testTotalMassErrorFires()
   {
      MassConsistencyGate gate = gate();
      accumulate(gate, new Random(1234L), SAMPLES, 0.0, 0.0, 74.0);

      GateResult result = gate.run();

      assertFalse(result.isPassed(), () -> "A 19 % mass error must fail:\n" + result);
      // 1 N of per-sample force noise over 2000 samples leaves about 0.022 N of mean, which is
      // 0.002 kg. 0.01 kg is a few times that and still four hundred times smaller than the fault.
      assertEquals(74.0, gate.getImpliedTotalMass(), 0.01, "The implied mass is sum(F_z)/g and nothing else.");
      assertEquals(1, result.getFailures().size(), "Only the weight is wrong; the balance is not.");
      assertEquals("total mass", result.getFailures().get(0).subject());
   }

   /**
    * True positive on balance, and it names the axis.
    * <p>
    * A magnitude would be positive under any fault. Reporting axes separately is what makes the
    * verdict say <i>which way</i> the mass is misplaced, which is the half that localises it.
    * </p>
    */
   @Test
   public void testDisplacedMassFiresOnlyTheAxisItMovedIn()
   {
      MassConsistencyGate gate = gate();
      accumulate(gate, new Random(1234L), SAMPLES, 0.0, 0.020, ALEX_MASS);

      GateResult result = gate.run();

      assertFalse(result.isPassed(), () -> "A 20 mm offset against a 14.7 mm bar must fail:\n" + result);
      assertEquals(1, result.getFailures().size(), "Only y moved.");
      assertEquals("mass distribution y", result.getFailures().get(0).subject());
      assertEquals(0.020, gate.getMeanOffsetY(), 1.0e-4);
   }

   /**
    * The statistical claim the threshold rests on: noise averages down, a bias does not.
    * <p>
    * Both runs carry the same 5 mm per-axis noise -- five times a real plate's. The one with no
    * bias converges to 0.11 mm and passes; the one with a 20 mm bias stays at 20 mm and fails. If
    * this ever inverted, the gate would be reporting the force plate rather than the robot.
    * </p>
    */
   @Test
   public void testNoiseAveragesDownButBiasDoesNot()
   {
      MassConsistencyGate noiseOnly = gate();
      accumulate(noiseOnly, new Random(99L), 5000, 0.0, 0.0, ALEX_MASS, 0.005);

      MassConsistencyGate biased = gate();
      accumulate(biased, new Random(99L), 5000, 0.0, 0.020, ALEX_MASS, 0.005);

      assertTrue(noiseOnly.run().isPassed(), () -> "Zero-mean noise must average down:\n" + noiseOnly.run());
      assertFalse(biased.run().isPassed(), () -> "A bias must survive averaging:\n" + biased.run());

      assertTrue(Math.abs(noiseOnly.getMeanOffsetY()) < 0.5e-3,
                 "5 mm of noise over 5000 samples should leave well under 0.5 mm of mean, got " + noiseOnly.getMeanOffsetY());
      assertEquals(0.020, biased.getMeanOffsetY(), 0.5e-3, "The bias must come back at its planted value.");
   }

   /**
    * INCOMPLETE is not PASS.
    * <p>
    * With two states, a robot that was never still would produce the most confident possible green,
    * because nothing contradicted it.
    * </p>
    */
   @Test
   public void testTooFewSamplesIsNotEvaluatedRatherThanPassed()
   {
      MassConsistencyGate gate = gate();
      accumulate(gate, new Random(1234L), 10, 0.0, 0.020, ALEX_MASS);

      GateResult result = gate.run();

      assertFalse(result.isPassed(), "Ten samples is not a measurement.");
      assertTrue(result.isIncomplete(), () -> "It should be incomplete, not failed:\n" + result);
      assertEquals(3, result.countWithStatus(GateResult.Status.NOT_EVALUATED));
   }

   /** A walking robot offers samples the gate must refuse, and must say how many it refused. */
   @Test
   public void testDynamicSamplesAreRejectedAndCounted()
   {
      MassConsistencyGate gate = gate();
      Point3D com = new Point3D(0.0, 0.500, 0.9);
      Point3D cop = new Point3D();

      for (int i = 0; i < SAMPLES; i++)
         gate.accumulate(com, cop, ALEX_MASS * GRAVITY, false);

      assertEquals(0, gate.getQuasiStaticSampleCount(), "Nothing offered was quasi-static.");
      assertEquals(SAMPLES, gate.getSamplesRejectedAsDynamic());
      assertTrue(gate.run().isIncomplete(), "A half-metre offset that was never static must not become a verdict.");
   }

   /**
    * A refused cluster leaves NaN, and one NaN in a mean poisons every number downstream.
    * <p>
    * The clean samples are identical in both runs, so the means must be too: the NaN samples have to
    * be dropped, not defaulted to zero, which would drag the mean toward the origin.
    * </p>
    */
   @Test
   public void testNaNSamplesAreDroppedRatherThanPoisoningTheMean()
   {
      MassConsistencyGate clean = gate();
      MassConsistencyGate interleaved = gate();

      Point3D com = new Point3D(0.0, 0.020, 0.9);
      Point3D cop = new Point3D();
      Point3D nan = new Point3D(Double.NaN, Double.NaN, Double.NaN);

      for (int i = 0; i < SAMPLES; i++)
      {
         clean.accumulate(com, cop, ALEX_MASS * GRAVITY, true);

         interleaved.accumulate(com, cop, ALEX_MASS * GRAVITY, true);
         interleaved.accumulate(nan, cop, ALEX_MASS * GRAVITY, true);
      }

      assertEquals(clean.getMeanOffsetY(), interleaved.getMeanOffsetY(), 1.0e-12);
      assertEquals(clean.getImpliedTotalMass(), interleaved.getImpliedTotalMass(), 1.0e-12);
      assertEquals(SAMPLES, interleaved.getQuasiStaticSampleCount(), "The NaN samples must not count as measurements.");
   }

   /** It runs inside the control loop, so accumulation must not allocate. */
   @Test
   public void testAccumulationIsAllocationFree()
   {
      MassConsistencyGate gate = gate();
      Point3D com = new Point3D(0.0, 0.001, 0.9);
      Point3D cop = new Point3D();

      // Warm up outside the measurement so class loading is not counted as allocation.
      accumulate(gate, new Random(1L), 100, 0.0, 0.0, ALEX_MASS);

      AllocationMeasurement.assertAllocationFree("10,000 G5 accumulations", () ->
      {
         for (int i = 0; i < 10_000; i++)
            gate.accumulate(com, cop, ALEX_MASS * GRAVITY, true);
      });
   }

   @Test
   public void testReset()
   {
      MassConsistencyGate gate = gate();
      accumulate(gate, new Random(1234L), SAMPLES, 0.0, 0.020, ALEX_MASS);
      assertFalse(gate.run().isPassed());

      gate.reset();

      assertEquals(0, gate.getQuasiStaticSampleCount());
      assertTrue(gate.run().isIncomplete(), "After a reset there is nothing to judge.");
   }

   @Test
   public void testConstructorRejectsUnusableParameters()
   {
      assertThrows(IllegalArgumentException.class,
                   () -> new MassConsistencyGate(0.0, GRAVITY, EXPECTED_COM_UNCERTAINTY, MASS_UNCERTAINTY_FRACTION),
                   "A massless model is not a model.");
      assertThrows(IllegalArgumentException.class,
                   () -> new MassConsistencyGate(ALEX_MASS, 0.0, EXPECTED_COM_UNCERTAINTY, MASS_UNCERTAINTY_FRACTION),
                   "Gravity scales the implied mass directly.");
      assertThrows(IllegalArgumentException.class,
                   () -> new MassConsistencyGate(ALEX_MASS, GRAVITY, Double.NaN, MASS_UNCERTAINTY_FRACTION),
                   "An unmeasured uncertainty has no default; NaN must not become a threshold.");
   }

   // ------------------------------------------------------------------------------------------
   // The claims the gate rests on, against the real Alex URDF
   // ------------------------------------------------------------------------------------------

   /**
    * <b>The reason G5 checks weight and balance separately.</b>
    * <p>
    * {@code c = sum(m_i r_i) / sum(m_i)} is a weighted average, so scaling every link by a common
    * factor cancels exactly. Rescaling Alex to the 74 kg figure -- a 19 % change in total mass --
    * moves the centre of mass by <b>zero</b>, to machine precision. Total mass is the one degree of
    * freedom the CoM cannot see, which is why a total-mass measurement cannot stand in for the
    * distribution check.
    * </p>
    */
   @Test
   public void testUniformMassRescaleDoesNotMoveTheCenterOfMass() throws Exception
   {
      RobotModelHandle model = alex();
      Point3D before = centerOfMassInBase(model);

      double factor = 74.0 / model.getTotalMass();
      scaleEveryLinkMass(model, factor);

      assertEquals(74.0, model.getTotalMass(), 1.0e-9, "The rescale should hit the target mass exactly.");

      Point3D after = centerOfMassInBase(model);

      assertEquals(0.0, before.distance(after), 1.0e-12, "A uniform rescale must not move the CoM at all. before=" + before + " after=" + after);
   }

   /**
    * A concentrated mass error moves the CoM by the mass ratio times the lever arm.
    * <p>
    * This is the relation the whole gate is calibrated against:
    * {@code delta = (m_error / M_new) * (r_link - c_old)}. If it stopped holding, every threshold
    * in G5 would be meaningless while every arithmetic test above still passed.
    * </p>
    */
   @Test
   public void testConcentratedMassErrorMovesTheCenterOfMassByMassRatioTimesLeverArm() throws Exception
   {
      RobotModelHandle model = alex();
      Point3D before = centerOfMassInBase(model);
      Point3D linkCenterOfMass = linkCenterOfMassInBase(model, "LEFT_THIGH");

      double added = model.getMass("LEFT_THIGH");
      scaleLinkMass(model, "LEFT_THIGH", 2.0);

      Point3D after = centerOfMassInBase(model);

      Point3D predicted = new Point3D(linkCenterOfMass);
      predicted.sub(before);
      predicted.scale(added / model.getTotalMass());
      predicted.add(before);

      assertEquals(0.0, predicted.distance(after), 1.0e-9, "predicted=" + predicted + " actual=" + after);
   }

   /**
    * <b>G5's blind spot, pinned so nobody can quietly claim it away.</b>
    * <p>
    * A static centre of pressure compares <i>horizontal</i> positions, so a link's detectability is
    * set by its horizontal distance from the whole-body CoM. Measured on the vendored URDF at the
    * rest pose: {@code TORSO_LINK} 0.0277 m against {@code LEFT_SHOULDER_Z_LINK} 0.2700 m -- a
    * factor of 9.7. ({@code LEFT_THIGH} is 0.1375 m, {@code PELVIS_LINK} 0.0713 m.) Since the CoP
    * offset is {@code (m_error / M) * lever}, clearing the 14.7 mm bar takes about 5.0 kg on the
    * shoulder's lever and about 49 kg on the torso's.
    * </p>
    * <p>
    * That matters because {@code TORSO_LINK} is simultaneously Alex's heaviest link at 22.21 kg and
    * the only one the SDK's V1 and V2 models disagree about, by 10.7 kg. <b>The check that would
    * catch that disagreement is the one this geometry defeats.</b> Vary the posture to give the
    * torso a lever arm; a single neutral stance will not do it.
    * </p>
    */
   @Test
   public void testTorsoHasAlmostNoHorizontalLeverArmAtRest() throws Exception
   {
      RobotModelHandle model = alex();
      Point3D bodyCenterOfMass = centerOfMassInBase(model);

      double torso = horizontalLeverArm(model, "TORSO_LINK", bodyCenterOfMass);
      double shoulder = horizontalLeverArm(model, "LEFT_SHOULDER_Z_LINK", bodyCenterOfMass);

      // Measured 0.0277 m and 0.2700 m. Loose bounds around those, so a real geometry change trips
      // this and floating-point drift does not.
      assertTrue(torso < 0.05, "TORSO_LINK's horizontal lever arm should be tiny, measured 0.0277 m, got " + torso);
      assertTrue(shoulder > 0.20, "LEFT_SHOULDER_Z_LINK's should be large, measured 0.2700 m, got " + shoulder);

      // The consequence, stated as the assertion rather than left to the reader: the same mass error
      // on the torso is an order of magnitude less visible than on the shoulder.
      assertTrue(shoulder / torso > 5.0, "The torso should be at least 5x harder to see, got " + (shoulder / torso));
   }

   // ------------------------------------------------------------------------------------------

   private static MassConsistencyGate gate()
   {
      return new MassConsistencyGate(ALEX_MASS, GRAVITY, EXPECTED_COM_UNCERTAINTY, MASS_UNCERTAINTY_FRACTION);
   }

   private static void accumulate(MassConsistencyGate gate, Random random, int samples, double offsetX, double offsetY, double trueMass)
   {
      accumulate(gate, random, samples, offsetX, offsetY, trueMass, 1.0e-3);
   }

   /**
    * Feeds the gate a robot whose measured CoP sits {@code (offsetX, offsetY)} away from where the
    * model puts the CoM, weighing {@code trueMass}, with zero-mean noise on both.
    */
   private static void accumulate(MassConsistencyGate gate,
                                  Random random,
                                  int samples,
                                  double offsetX,
                                  double offsetY,
                                  double trueMass,
                                  double copNoise)
   {
      Point3D modelCenterOfMass = new Point3D();
      Point3D centerOfPressure = new Point3D();

      for (int i = 0; i < samples; i++)
      {
         // The model's CoM is offset from the truth; the plate measures the truth, plus noise.
         modelCenterOfMass.set(offsetX, offsetY, 0.9);
         centerOfPressure.set(copNoise * random.nextGaussian(), copNoise * random.nextGaussian(), 0.0);

         gate.accumulate(modelCenterOfMass, centerOfPressure, trueMass * GRAVITY + random.nextGaussian(), true);
      }
   }

   private static RobotModelHandle alex() throws Exception
   {
      RobotModelHandle model = RobotModelHandle.fromURDF(RobotCaptures.alexUrdfPath());
      model.updateFrames();
      return model;
   }

   /** Whole-body CoM in the base frame, summed the same way F9 does it. */
   private static Point3D centerOfMassInBase(RobotModelHandle model)
   {
      Point3D weightedSum = new Point3D();
      Point3D linkCenterOfMass = new Point3D();
      double total = 0.0;

      for (String link : model.getLinkNames())
      {
         double mass = model.getMass(link);

         if (mass == 0.0)
            continue;

         packLinkCenterOfMassInBase(model, link, linkCenterOfMass);
         weightedSum.scaleAdd(mass, linkCenterOfMass, weightedSum);
         total += mass;
      }

      weightedSum.scale(1.0 / total);
      return weightedSum;
   }

   private static Point3D linkCenterOfMassInBase(RobotModelHandle model, String linkName)
   {
      Point3D centerOfMass = new Point3D();
      packLinkCenterOfMassInBase(model, linkName, centerOfMass);
      return centerOfMass;
   }

   private static void packLinkCenterOfMassInBase(RobotModelHandle model, String linkName, Point3D toPack)
   {
      RigidBodyTransform linkToBase = new RigidBodyTransform();
      model.packCenterOfMassInLinkFrame(linkName, toPack);
      model.packLinkToBase(linkName, linkToBase);
      linkToBase.transform(toPack);
   }

   private static double horizontalLeverArm(RobotModelHandle model, String linkName, Point3D bodyCenterOfMass)
   {
      Point3D linkCenterOfMass = linkCenterOfMassInBase(model, linkName);
      double dx = linkCenterOfMass.getX() - bodyCenterOfMass.getX();
      double dy = linkCenterOfMass.getY() - bodyCenterOfMass.getY();
      return Math.hypot(dx, dy);
   }

   private static void scaleEveryLinkMass(RobotModelHandle model, double factor)
   {
      for (RigidBodyBasics body : model.getRootBody().subtreeIterable())
      {
         if (body.getInertia() != null)
            body.getInertia().setMass(body.getInertia().getMass() * factor);
      }
   }

   private static void scaleLinkMass(RobotModelHandle model, String linkName, double factor)
   {
      for (RigidBodyBasics body : model.getRootBody().subtreeIterable())
      {
         if (body.getName().equals(linkName))
         {
            body.getInertia().setMass(body.getInertia().getMass() * factor);
            return;
         }
      }

      throw new IllegalArgumentException("No link named '" + linkName + "'.");
   }
}
