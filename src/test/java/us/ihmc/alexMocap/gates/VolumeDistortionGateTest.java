package us.ihmc.alexMocap.gates;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * G3.
 * <p>
 * As with G1's test class, the pair that matters is true-positive and true-negative together.
 * </p>
 */
public class VolumeDistortionGateTest
{
   /** Measured per-axis noise at the gantry, FRAMEWORK.md §17's target for the tight volume. */
   private static final double SIGMA = 0.3e-3;

   private static final List<MarkerId> MARKER_SET = MarkerId.createDenseSet("WAND_1", "WAND_2");
   private static final MarkerId MARKER_A = MARKER_SET.get(0);
   private static final MarkerId MARKER_B = MARKER_SET.get(1);

   /** A plausible calibration-wand length: long enough to be worth carrying, unrelated to G1's mm-scale clusters. */
   private static final double KNOWN_LENGTH = 0.500;

   private static final Point3D CENTRE = new Point3D(0.0, 0.0, 0.0);

   private static VolumeDistortionGate gate()
   {
      return new VolumeDistortionGate(MARKER_A, MARKER_B, KNOWN_LENGTH, SIGMA, CENTRE);
   }

   /**
    * True negative: a wand measured at its known length, swept to all eight corners of the volume,
    * passes on all six sides.
    * <p>
    * 25 samples per corner puts 100 on every side (each corner touches three sides at once), well
    * past {@link VolumeDistortionGate#DEFAULT_MINIMUM_SAMPLES}, and the deterministic corner
    * enumeration means every side's sample count is exact rather than approximate.
    * </p>
    */
   @Test
   public void testCleanSweepToAllCornersPassesEverySide()
   {
      VolumeDistortionGate gate = gate();
      accumulateCornerSweep(gate, new Random(31_001L), 25, 0.0);

      GateResult result = gate.run();

      assertTrue(result.isPassed(), () -> "A clean sweep should pass every side:\n" + report(gate, result));
      assertEquals(6, result.getFindings().size(), "Six sides: X-, X+, Y-, Y+, Z-, Z+.");
      assertEquals(6, result.countWithStatus(GateResult.Status.PASS));

      for (GateResult.Finding finding : result.getFindings())
         assertTrue(finding.measured() < finding.threshold(), finding.subject() + ": bias " + finding.measured() + " m, threshold " + finding.threshold() + " m");
   }

   /**
    * True positive: a wand that reads 2 mm long specifically on the +X side of the volume fails
    * there, and the -X side -- which never sees the fault at all -- passes.
    * <p>
    * Not asserted here: the Y/Z sides. Every sample this fault touches sits on the +X side AND on
    * whichever Y/Z sides its corner belongs to, so half of each Y/Z side's samples are biased and
    * half are not (FRAMEWORK.md-consistent with a fault localised to one axis showing up diluted,
    * not absent, on the orthogonal checks) -- a real and correctly-attributed effect, but not the
    * one this test is pinning down. {@code X-} versus {@code X+} is the clean, undiluted contrast:
    * X- gets none of the biased corners, X+ gets all of them.
    * </p>
    */
   @Test
   public void testLocalizedDistortionFailsThatSideAndSparesTheOppositeSide()
   {
      VolumeDistortionGate gate = gate();
      accumulateCornerSweep(gate, new Random(31_002L), 25, 2.0e-3);

      GateResult result = gate.run();

      assertFalse(result.isPassed(), () -> "A 2 mm bias on one side must fail G3:\n" + report(gate, result));
      assertFalse(result.isIncomplete(), "This is a failure, not an inability to evaluate.");

      GateResult.Finding xPlus = findingFor(result, "X+");
      GateResult.Finding xMinus = findingFor(result, "X-");

      assertEquals(GateResult.Status.FAIL, xPlus.status(), "X+: " + xPlus);
      assertEquals(GateResult.Status.PASS, xMinus.status(), "X-: " + xMinus);
      assertTrue(xPlus.measured() > xPlus.threshold());
      assertTrue(xMinus.measured() < xMinus.threshold());
   }

   /**
    * A side the sweep never reached reports {@link GateResult.Status#NOT_EVALUATED}, never a silent
    * pass -- the same rule G1 applies to a pair that is never co-visible.
    */
   @Test
   public void testUnreachedSideIsNotEvaluatedNotPassed()
   {
      VolumeDistortionGate gate = gate();
      // Every sample here sits at (+1, 0, 0): contributes to X+, and defaults to Y+/Z+ on the tie
      // at the centre. X-, Y-, Z- never get a single sample.
      accumulateAt(gate, new Random(31_003L), 10, 1.0, 0.0, 0.0, 0.0);

      GateResult result = gate.run();

      assertFalse(result.isPassed());
      assertTrue(result.isIncomplete(), "Nothing failed; some sides simply were never visited.");
      assertEquals(GateResult.Status.NOT_EVALUATED, findingFor(result, "X-").status());
      assertEquals(GateResult.Status.NOT_EVALUATED, findingFor(result, "Y-").status());
      assertEquals(GateResult.Status.NOT_EVALUATED, findingFor(result, "Z-").status());
      // X+/Y+/Z+ got the 10 samples, but 10 < DEFAULT_MINIMUM_SAMPLES too.
      assertEquals(GateResult.Status.NOT_EVALUATED, findingFor(result, "X+").status());
   }

   @Test
   public void testConstructorRejectsBadInputs()
   {
      assertThrows(IllegalArgumentException.class, () -> new VolumeDistortionGate(null, MARKER_B, KNOWN_LENGTH, SIGMA, CENTRE));
      assertThrows(IllegalArgumentException.class, () -> new VolumeDistortionGate(MARKER_A, MARKER_A, KNOWN_LENGTH, SIGMA, CENTRE),
                   "The two markers must be distinct.");
      assertThrows(IllegalArgumentException.class, () -> new VolumeDistortionGate(MARKER_A, MARKER_B, 0.0, SIGMA, CENTRE),
                   "Known length must be positive.");
      assertThrows(IllegalArgumentException.class, () -> new VolumeDistortionGate(MARKER_A, MARKER_B, -0.5, SIGMA, CENTRE));
      assertThrows(IllegalArgumentException.class, () -> new VolumeDistortionGate(MARKER_A, MARKER_B, KNOWN_LENGTH, 0.0, CENTRE),
                   "Sigma must be positive and measured, never assumed.");
      assertThrows(IllegalArgumentException.class,
                   () -> new VolumeDistortionGate(MARKER_A, MARKER_B, KNOWN_LENGTH, SIGMA, 3.0, 0, CENTRE),
                   "Need at least one sample per side.");
      assertThrows(IllegalArgumentException.class, () -> new VolumeDistortionGate(MARKER_A, MARKER_B, KNOWN_LENGTH, SIGMA, null),
                   "A volume centre is required to bin samples against.");
   }

   @Test
   public void testThresholdTightensWithMoreSamplesUnlikeG1sFixedMargin()
   {
      VolumeDistortionGate gate = gate();
      // Unlike RigidityGate.getThreshold(), which is one number independent of sample count, G3's
      // threshold is a standard-error-of-the-mean bound: it must shrink as 1/sqrt(N), because the
      // question is "is this fixed known length measured correctly", and more data resolves a
      // smaller true bias.
      assertTrue(gate.getThreshold(400) < gate.getThreshold(100));
      assertEquals(gate.getThreshold(100) / 2.0, gate.getThreshold(400), 1.0e-12, "4x the samples halves the threshold (1/sqrt(4)).");
   }

   private static GateResult.Finding findingFor(GateResult result, String side)
   {
      for (GateResult.Finding finding : result.getFindings())
         if (finding.subject().equals(side))
            return finding;

      throw new AssertionError("No finding for side '" + side + "' in " + result.getFindings());
   }

   /**
    * Accumulates {@code samplesPerCorner} frames at each of the eight corners of a 2x2x2 m cube
    * about {@link #CENTRE}, adding {@code biasOnPositiveXMeters} to the wand's apparent length only
    * at the four corners with {@code x > 0}.
    */
   private static void accumulateCornerSweep(VolumeDistortionGate gate, Random random, int samplesPerCorner, double biasOnPositiveXMeters)
   {
      double extent = 1.0;

      for (int sx : new int[] {-1, 1})
         for (int sy : new int[] {-1, 1})
            for (int sz : new int[] {-1, 1})
            {
               double bias = sx > 0 ? biasOnPositiveXMeters : 0.0;
               accumulateAt(gate, random, samplesPerCorner, sx * extent, sy * extent, sz * extent, bias);
            }
   }

   /** Accumulates {@code count} noisy samples of the wand centred at {@code (midX, midY, midZ)}, at {@code KNOWN_LENGTH + biasMeters} apparent length. */
   private static void accumulateAt(VolumeDistortionGate gate, Random random, int count, double midX, double midY, double midZ, double biasMeters)
   {
      MocapFrame frame = new MocapFrame(MARKER_SET);
      double half = (KNOWN_LENGTH + biasMeters) / 2.0;

      for (int i = 0; i < count; i++)
      {
         frame.get(MARKER_A).setVisible(midX - half + SIGMA * random.nextGaussian(), midY + SIGMA * random.nextGaussian(), midZ + SIGMA * random.nextGaussian());
         frame.get(MARKER_B).setVisible(midX + half + SIGMA * random.nextGaussian(), midY + SIGMA * random.nextGaussian(), midZ + SIGMA * random.nextGaussian());
         gate.accumulate(frame);
      }
   }

   private static String report(VolumeDistortionGate gate, GateResult result)
   {
      StringBuilder builder = new StringBuilder(result.getSummary()).append('\n');

      for (GateResult.Finding finding : result.getFindings())
         builder.append("  ").append(finding).append('\n');

      return builder.toString();
   }
}
