package us.ihmc.alexMocap.gates;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.AllocationMeasurement;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * G1.
 * <p>
 * The pair of tests that matters is true-positive and true-negative together. A gate with only a
 * passing test is a gate you have never seen fire, and a gate with only a failing test is one that
 * might reject everything.
 * </p>
 */
public class RigidityGateTest
{
   /** Measured per-axis noise at the gantry, FRAMEWORK.md §17's target for the tight volume. */
   private static final double SIGMA = 0.3e-3;

   private static final List<MarkerId> MARKER_SET = MarkerId.createDenseSet("PELVIS_1", "PELVIS_2", "PELVIS_3", "PELVIS_4");

   /**
    * A cluster in a 120 mm outrigger arrangement: non-collinear, asymmetric, in the spirit of
    * FRAMEWORK.md §1.
    */
   private static final double[][] CLUSTER_GEOMETRY = {{0.060, 0.000, 0.010}, {-0.045, 0.055, -0.008}, {-0.050, -0.048, 0.012}, {0.030, -0.062, -0.015}};

   /**
    * True negative: a rigid cluster at the measured noise level passes.
    * <p>
    * The pairs sit at the noise floor of {@code √2 σ = 0.42 mm} against a threshold of
    * {@code 3σ = 0.90 mm}, so this is a 2.1× margin and the test should never be marginal.
    * </p>
    */
   @Test
   public void testRigidClusterPasses()
   {
      RigidityGate gate = gate();
      accumulateRigidCapture(gate, new Random(4242L), 600, 0.0);

      GateResult result = gate.run();

      assertTrue(result.isPassed(), () -> "A rigid cluster at sigma should pass:\n" + report(gate, result));
      assertEquals(6, result.getFindings().size(), "Four markers give six pairs.");
      assertEquals(6, result.countWithStatus(GateResult.Status.PASS));

      // Every pair should land near the sqrt(2)*sigma floor rather than merely under threshold.
      for (GateResult.Finding finding : result.getFindings())
      {
         double ratioToFloor = finding.measured() / gate.getNoiseFloor();
         assertTrue(ratioToFloor > 0.8 && ratioToFloor < 1.25,
                    "Pair " + finding.subject() + " measured " + finding.measured() + " m, floor is " + gate.getNoiseFloor() + " m");
      }
   }

   /**
    * True positive, and PR1's stated acceptance criterion: a marker moving 2 mm during the capture
    * fails, and the failures name the marker responsible.
    * <p>
    * The fault is modelled as a <b>step</b> -- the mount slips once, partway through -- which is
    * what "2 mm of slop" physically is. That distinction is load-bearing; see
    * {@link #testCreepNeedsMoreTravelThanASlipToDetect()}.
    * </p>
    */
   @Test
   public void testSlippingMountFailsAtTwoMillimetres()
   {
      RigidityGate gate = gate();
      accumulateCapture(gate, new Random(4242L), 600, 2.0e-3, Fault.STEP);

      GateResult result = gate.run();

      assertFalse(result.isPassed(), () -> "A 2 mm slip must fail G1:\n" + report(gate, result));
      assertFalse(result.isIncomplete(), "This is a failure, not an inability to evaluate.");
      assertEquals(GateResult.Status.FAIL, result.getOverallStatus());

      List<GateResult.Finding> failures = result.getFailures();
      assertTrue(failures.size() >= 2, () -> "The slip should disturb more than one pair:\n" + report(gate, result));

      // The point of a per-pair table: the failures indict the marker that moved, not the cluster.
      for (GateResult.Finding failure : failures)
      {
         assertTrue(failure.subject().contains("PELVIS_1"), "Failure should involve the marker that moved: " + failure.subject());
         assertTrue(failure.measured() > failure.threshold());
      }
   }

   /**
    * A slip and a creep of the same total travel are not equally detectable, and the difference is
    * a factor of 1.7 rather than a rounding error.
    * <p>
    * A step of amplitude {@code a} puts half the samples at each end: {@code std = a/2}. A linear
    * creep spreads them uniformly over {@code [0, a]}: {@code std = a/√12 ≈ a/3.46}. Solving
    * {@code √(2σ² + (k·a)²) = 3σ} for a baseline-aligned pair at {@code σ = 0.3 mm} puts the
    * detection threshold near <b>1.7 mm for a step</b> and <b>3.1 mm for a creep</b>.
    * </p>
    * <p>
    * So <b>2 mm of slow creep passes G1</b>. That is not a bug in the gate -- it is what a 3σ
    * threshold on a standard deviation means -- but it is worth knowing before trusting a green
    * G1 to rule out a slowly loosening mount.
    * </p>
    */
   @Test
   public void testCreepNeedsMoreTravelThanASlipToDetect()
   {
      assertTrue(fails(2.0e-3, Fault.STEP), "A 2 mm slip is caught.");
      assertFalse(fails(2.0e-3, Fault.CREEP), "The same 2 mm as a slow creep is not: std is 1.7x smaller.");

      assertFalse(fails(1.0e-3, Fault.STEP), "1 mm of slip is under the threshold; G1 should stay quiet.");
      assertTrue(fails(5.0e-3, Fault.CREEP), "5 mm of creep is past it.");
   }

   /**
    * Only the component of a movement along a pair's baseline changes that pair's distance. A
    * marker shifting perpendicular to a baseline is invisible to that pair however far it goes.
    * <p>
    * PELVIS_1 and PELVIS_4 are separated by (30, 62, 25) mm, so a shift along x projects onto only
    * 41% of their baseline; the other two pairs see 88% and 92%. At 5 mm of creep that pair still
    * reads under threshold while the other two are well over.
    * </p>
    * <p>
    * This is why a cluster is checked pairwise instead of by one aggregate number: to hide from G1
    * a shift would have to be perpendicular to <i>every</i> baseline at once, which a non-collinear
    * cluster does not permit.
    * </p>
    */
   @Test
   public void testAShiftPerpendicularToABaselineIsInvisibleToThatPair()
   {
      RigidityGate gate = gate();
      accumulateCapture(gate, new Random(4242L), 600, 5.0e-3, Fault.CREEP);

      GateResult result = gate.run();
      assertFalse(result.isPassed());

      GateResult.Finding nearlyPerpendicular = findingFor(result, "PELVIS_1-PELVIS_4");
      GateResult.Finding wellAligned = findingFor(result, "PELVIS_1-PELVIS_3");

      assertEquals(GateResult.Status.PASS,
                   nearlyPerpendicular.status(),
                   () -> "The nearly perpendicular pair should not see a 5 mm shift along x:\n" + report(gate, result));
      assertEquals(GateResult.Status.FAIL, wellAligned.status());
      assertTrue(wellAligned.measured() > 1.7 * nearlyPerpendicular.measured(), "The aligned pair should see far more of the same movement.");
   }

   private static GateResult.Finding findingFor(GateResult result, String pair)
   {
      return result.getFindings().stream().filter(finding -> finding.subject().endsWith(pair)).findFirst().orElseThrow();
   }

   private boolean fails(double amplitude, Fault fault)
   {
      RigidityGate gate = gate();
      accumulateCapture(gate, new Random(4242L), 600, amplitude, fault);
      return !gate.run().isPassed();
   }

   /**
    * A pair that was rarely co-visible is reported NOT EVALUATED and the gate does not pass.
    * <p>
    * This is the failure mode a two-state gate has: with only PASS and FAIL, a cluster whose
    * markers never appear together produces the most confident possible green, because nothing
    * contradicted it.
    * </p>
    */
   @Test
   public void testRarelyCoVisiblePairIsNotEvaluatedRatherThanPassed()
   {
      RigidityGate gate = gate();
      Random random = new Random(7L);
      MocapFrame frame = new MocapFrame(MARKER_SET);

      for (int f = 0; f < 600; f++)
      {
         frame.clear();
         frame.setTimestampNanoseconds(f * 5_000_000L);

         for (int m = 0; m < MARKER_SET.size(); m++)
         {
            // PELVIS_4 is visible in only the first 20 frames, so its three pairs never reach the
            // 100-sample minimum. The other three markers are visible throughout.
            if (m == 3 && f >= 20)
               continue;

            frame.get(m).setVisible(CLUSTER_GEOMETRY[m][0] + SIGMA * random.nextGaussian(),
                                    CLUSTER_GEOMETRY[m][1] + SIGMA * random.nextGaussian(),
                                    CLUSTER_GEOMETRY[m][2] + SIGMA * random.nextGaussian());
         }

         gate.accumulate(frame);
      }

      GateResult result = gate.run();

      assertFalse(result.isPassed(), "Three unevaluated pairs must not read as a pass.");
      assertTrue(result.isIncomplete(), "Nothing failed; three checks could not run.");
      assertEquals(GateResult.Status.NOT_EVALUATED, result.getOverallStatus());
      assertEquals(3, result.countWithStatus(GateResult.Status.NOT_EVALUATED));
      assertEquals(3, result.countWithStatus(GateResult.Status.PASS));

      for (GateResult.Finding finding : result.getFindings())
      {
         if (finding.status() == GateResult.Status.NOT_EVALUATED)
         {
            assertTrue(finding.subject().contains("PELVIS_4"));
            assertEquals(20, finding.sampleCount());
            assertTrue(Double.isNaN(finding.measured()), "An unevaluated check has no measurement to report.");
         }
      }
   }

   /**
    * A label swap within a cluster is the other thing G1 exists to catch: two markers trading
    * identities changes every distance that involves either of them.
    */
   @Test
   public void testLabelSwapWithinAClusterFails()
   {
      RigidityGate gate = gate();
      Random random = new Random(31L);
      MocapFrame frame = new MocapFrame(MARKER_SET);

      for (int f = 0; f < 600; f++)
      {
         frame.clear();
         frame.setTimestampNanoseconds(f * 5_000_000L);

         // Halfway through the capture, markers 1 and 2 swap labels.
         boolean swapped = f >= 300;

         for (int m = 0; m < MARKER_SET.size(); m++)
         {
            int source = m;

            if (swapped && m == 1)
               source = 2;
            else if (swapped && m == 2)
               source = 1;

            frame.get(m).setVisible(CLUSTER_GEOMETRY[source][0] + SIGMA * random.nextGaussian(),
                                    CLUSTER_GEOMETRY[source][1] + SIGMA * random.nextGaussian(),
                                    CLUSTER_GEOMETRY[source][2] + SIGMA * random.nextGaussian());
         }

         gate.accumulate(frame);
      }

      GateResult result = gate.run();

      assertFalse(result.isPassed(), "A mid-capture label swap must fail G1.");
      assertTrue(result.getFailures().size() >= 2, "The swap should disturb several pairs, not one.");
   }

   /** The cluster moving as a rigid body must not disturb G1 -- only relative motion counts. */
   @Test
   public void testWholeBodyMotionDoesNotTripTheGate()
   {
      RigidityGate gate = gate();
      Random random = new Random(1234L);
      MocapFrame frame = new MocapFrame(MARKER_SET);
      Point3D point = new Point3D();

      for (int f = 0; f < 600; f++)
      {
         frame.clear();
         frame.setTimestampNanoseconds(f * 5_000_000L);

         // A different rigid pose every frame: the robot swinging on the gantry.
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         for (int m = 0; m < MARKER_SET.size(); m++)
         {
            point.set(CLUSTER_GEOMETRY[m][0], CLUSTER_GEOMETRY[m][1], CLUSTER_GEOMETRY[m][2]);
            pose.transform(point);
            frame.get(m).setVisible(point.getX() + SIGMA * random.nextGaussian(),
                                    point.getY() + SIGMA * random.nextGaussian(),
                                    point.getZ() + SIGMA * random.nextGaussian());
         }

         gate.accumulate(frame);
      }

      assertTrue(gate.run().isPassed(), "G1 measures inter-marker distances, which are invariant to the cluster's pose.");
   }

   /** Sigma must be measured; the gate refuses to invent one. */
   @Test
   public void testSigmaIsRequiredAndMustBePositive()
   {
      List<MarkerCluster> clusters = List.of(new MarkerCluster("pelvis", MARKER_SET));

      assertThrows(IllegalArgumentException.class, () -> new RigidityGate(clusters, 0.0));
      assertThrows(IllegalArgumentException.class, () -> new RigidityGate(clusters, -1.0e-3));
      assertThrows(IllegalArgumentException.class, () -> new RigidityGate(clusters, Double.NaN));
      assertThrows(IllegalArgumentException.class, () -> new RigidityGate(List.of(), SIGMA));
   }

   /** The stated relationship between threshold and noise floor, asserted rather than commented. */
   @Test
   public void testThresholdIsAMultipleOfSigmaAndTheFloorIsSqrt2Sigma()
   {
      RigidityGate gate = gate();

      assertEquals(3.0 * SIGMA, gate.getThreshold(), 1.0e-15);
      assertEquals(Math.sqrt(2.0) * SIGMA, gate.getNoiseFloor(), 1.0e-15);
      assertEquals(2.121, gate.getThreshold() / gate.getNoiseFloor(), 1.0e-3, "The real margin is 3/sqrt(2), not 3.");
   }

   /** Accumulation runs at capture rate over a long log; it must not allocate. */
   @Test
   public void testAccumulationIsAllocationFree()
   {
      RigidityGate gate = gate();
      Random random = new Random(5L);
      MocapFrame frame = new MocapFrame(MARKER_SET);

      for (int m = 0; m < MARKER_SET.size(); m++)
         frame.get(m).setVisible(CLUSTER_GEOMETRY[m][0], CLUSTER_GEOMETRY[m][1], CLUSTER_GEOMETRY[m][2]);

      frame.setTimestampNanoseconds(1L);

      AllocationMeasurement.assertAllocationFree("10,000 G1 frame accumulations", () ->
      {
         for (int i = 0; i < 10_000; i++)
            gate.accumulate(frame);
      });

      assertTrue(gate.getFramesAccumulated() > 0);
      assertTrue(random.nextDouble() >= 0.0);
   }

   private static RigidityGate gate()
   {
      return new RigidityGate(List.of(new MarkerCluster("pelvis", MARKER_SET)), SIGMA);
   }

   /** How a loose mount moves: all at once, or gradually. */
   private enum Fault
   {
      /** The mount slips once, halfway through the capture. Sample std is {@code a/2}. */
      STEP,
      /** The mount creeps steadily over the capture. Sample std is {@code a/√12}. */
      CREEP,
      NONE
   }

   private static void accumulateRigidCapture(RigidityGate gate, Random random, int frameCount, double amplitude)
   {
      accumulateCapture(gate, random, frameCount, amplitude, amplitude == 0.0 ? Fault.NONE : Fault.CREEP);
   }

   /**
    * Fills the gate with a static cluster plus seeded Gaussian noise, moving marker 0 along x by
    * {@code amplitude} according to {@code fault}.
    */
   private static void accumulateCapture(RigidityGate gate, Random random, int frameCount, double amplitude, Fault fault)
   {
      MocapFrame frame = new MocapFrame(MARKER_SET);

      for (int f = 0; f < frameCount; f++)
      {
         frame.clear();
         frame.setTimestampNanoseconds(f * 5_000_000L);

         double offset = switch (fault)
         {
            case STEP -> f >= frameCount / 2 ? amplitude : 0.0;
            case CREEP -> amplitude * f / (frameCount - 1.0);
            case NONE -> 0.0;
         };

         for (int m = 0; m < MARKER_SET.size(); m++)
         {
            frame.get(m).setVisible(CLUSTER_GEOMETRY[m][0] + (m == 0 ? offset : 0.0) + SIGMA * random.nextGaussian(),
                                    CLUSTER_GEOMETRY[m][1] + SIGMA * random.nextGaussian(),
                                    CLUSTER_GEOMETRY[m][2] + SIGMA * random.nextGaussian());
         }

         gate.accumulate(frame);
      }
   }

   private static String report(RigidityGate gate, GateResult result)
   {
      return new GateRunner.Report(List.of(gate), List.of(result)).format();
   }
}
