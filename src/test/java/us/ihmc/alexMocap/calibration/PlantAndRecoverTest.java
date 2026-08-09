package us.ihmc.alexMocap.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.calibration.BaseInitializer.GaugeTracking;
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * PR_PLAN.md's acceptance test: "a single closed loop that either works or the whole approach is
 * wrong".
 * <p>
 * Plant a known marker layout and known base poses, generate {@code K} captures by forward
 * kinematics at scattered joint configurations, add seeded noise, run A′, and assert recovery.
 * </p>
 *
 * <h2>Be honest about what this proves</h2>
 * <p>
 * The toy URDF tests the solver, not TALOS's 2.2 mm. It cannot say whether the real robot's
 * geometry is good enough. It says that when the geometry <i>is</i> good, the code recovers the
 * answer -- which is exactly what must be true before pointing it at real data.
 * </p>
 */
public class PlantAndRecoverTest
{
   /** FRAMEWORK.md §17: the current whole-lab calibration residual. */
   private static final double SIGMA_CURRENT = 0.93e-3;

   /** §17: the target for a tight ~2×2×2.5 m volume at the gantry. */
   private static final double SIGMA_TARGET = 0.3e-3;

   private static CalibrationResult calibrate(SyntheticCaptures.Planted planted, GaugeTracking tracking, CalibrationReport report)
   {
      return new AlternatingCalibrator().calibrate(planted.captureSet, planted.model, tracking, report);
   }

   /** Largest distance between a recovered marker position and its planted truth, in metres. */
   private static double worstLayoutError(SyntheticCaptures.Planted planted, CalibrationResult recovered)
   {
      double worst = 0.0;

      for (MarkerCluster cluster : planted.clusters)
      {
         ClusterLayout truth = planted.plantedLayout(cluster.getLinkName());
         ClusterLayout estimate = recovered.getLayout(cluster.getLinkName());

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            if (estimate.getObservationCount(j) == 0)
               continue;

            worst = Math.max(worst, new Point3D(truth.getPositionInLinkFrame(j)).distance(new Point3D(estimate.getPositionInLinkFrame(j))));
         }
      }

      return worst;
   }

   /**
    * Largest base-pose error over the captures, in metres.
    * <p>
    * This is the convention-free form of "did it recover {@code Δ}". See
    * {@link SyntheticCaptures} for why {@code Δ̂ == Δ*} is not the right assertion: {@code c}'s
    * convention is the solver's to choose, but {@code ^W T_c^(k) · Δ̂} must reproduce the planted
    * base pose at every capture regardless of that choice.
    * </p>
    */
   private static double worstBasePoseError(SyntheticCaptures.Planted planted, GaugeTracking tracking, CalibrationResult recovered)
   {
      RigidBodyTransform reconstructed = new RigidBodyTransform();
      double worst = 0.0;

      for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
      {
         if (!tracking.isUsable(k))
            continue;

         reconstructed.set(tracking.getClusterToWorld(k));
         reconstructed.multiply(recovered.getClusterToBase());

         Point3D origin = new Point3D();
         reconstructed.transform(origin);

         Point3D plantedOrigin = new Point3D();
         planted.basePoses[k].transform(plantedOrigin);

         worst = Math.max(worst, origin.distance(plantedOrigin));

         // Orientation too: a base pose recovered to the right position with the wrong rotation
         // would pass a position-only check and be badly wrong everywhere else.
         RigidBodyTransform error = new RigidBodyTransform(planted.basePoses[k]);
         error.invert();
         error.multiply(reconstructed);
         worst = Math.max(worst, Math.abs(new us.ihmc.euclid.axisAngle.AxisAngle(error.getRotation()).getAngle()));
      }

      return worst;
   }

   /**
    * Noiseless, {@code K = 5}. Any failure here is an algebra bug, not a tuning problem.
    */
   @Test
   public void testNoiselessRecoveryIsExact() throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(5).noise(0.0));
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);

      CalibrationReport report = new CalibrationReport();
      CalibrationResult recovered = calibrate(planted, tracking, report);

      assertTrue(recovered.isFullySolved(), "Every marker should have a finite calibrated position.\n" + report.toTable());
      assertTrue(worstLayoutError(planted, recovered) < 1.0e-9,
                 "Layout error " + worstLayoutError(planted, recovered) + " m exceeds 1e-9.\n" + report.toTable());
      assertTrue(worstBasePoseError(planted, tracking, recovered) < 1.0e-9,
                 "Base pose error " + worstBasePoseError(planted, tracking, recovered) + " exceeds 1e-9.\n" + report.toTable());
   }

   /**
    * {@code σ = 0.3 mm}, {@code K = 30}, gauge cluster at FRAMEWORK.md §1's recommended 140 mm.
    *
    * <h2>The threshold is 2.5 mm, and PR_PLAN.md's 0.2 mm is not reachable</h2>
    * <p>
    * PR_PLAN.md sets this bound from "theoretical is {@code σ/√K ≈ 0.055 mm}". That is F4's noise
    * floor (§6) and it is correct as far as it goes, but it is <b>not</b> the dominant term, so no
    * implementation can meet a threshold derived from it.
    * </p>
    * <p>
    * What dominates is the gauge cluster's <i>angular</i> uncertainty. The base pose of every
    * capture comes from registering four pelvis markers, so it carries an angular error of order
    * {@code σ / (√N · r_perp)} -- FRAMEWORK.md §1 states exactly this scaling, in the sentence that
    * asks for an outrigger bracket. That angle is then multiplied by the lever arm out to each
    * link. For the toy's foot at ~0.6 m from the pelvis, with {@code N = 4} and a 140 mm cluster,
    * it is roughly twenty times {@code σ/√K}.
    * </p>
    * <p>
    * Measured behaviour, which {@link #testLayoutErrorObeysTheGaugeClusterScalingLaw()} pins down:
    * the error falls as {@code 1/√K} and as {@code 1/r_perp}, and grows with distance from the
    * pelvis (pelvis ~0.37 mm, foot ~1.29 mm at these settings). The 2.5 mm bound here is about
    * twice the measured worst case, per PR_PLAN.md's own rule that a flaky test is worse than no
    * test.
    * </p>
    * <p>
    * <b>The practical consequence is a hardware one.</b> Marker-layout accuracy on distal links is
    * set by the pelvis cluster's spread and by {@code σ}, not by how many captures are taken. That
    * is the quantitative argument for both the outrigger bracket (§1) and the camera-repositioning
    * work in §20.
    * </p>
    */
   @Test
   public void testRecoveryUnderRealisticNoise() throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(30).noise(SIGMA_TARGET));
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);

      CalibrationReport report = new CalibrationReport();
      CalibrationResult recovered = calibrate(planted, tracking, report);

      double layoutError = worstLayoutError(planted, recovered);
      assertTrue(layoutError < 2.5e-3, "Layout error " + 1000.0 * layoutError + " mm exceeds 2.5 mm.\n" + report.toTable());

      // And the pelvis itself, which has no lever arm, should be far better than the extremities.
      ClusterLayout pelvisTruth = planted.plantedLayout("pelvis");
      ClusterLayout pelvisEstimate = recovered.getLayout("pelvis");
      double pelvisError = 0.0;

      for (int j = 0; j < pelvisEstimate.getMarkerCount(); j++)
      {
         pelvisError = Math.max(pelvisError,
                                new Point3D(pelvisTruth.getPositionInLinkFrame(j)).distance(new Point3D(pelvisEstimate.getPositionInLinkFrame(j))));
      }

      assertTrue(pelvisError < 1.0e-3,
                 "The gauge cluster's own layout has no lever arm to amplify anything and should be well under 1 mm, was " + 1000.0 * pelvisError + " mm.");
   }

   /**
    * The same at the noise the lab actually has today ({@code σ = 0.93 mm}, FRAMEWORK.md §17), so
    * the cost of not doing §20's camera work is a number rather than an intuition.
    */
   @Test
   public void testRecoveryAtTodaysLabNoise() throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(30).noise(SIGMA_CURRENT));
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);

      CalibrationReport report = new CalibrationReport();
      CalibrationResult recovered = calibrate(planted, tracking, report);

      double layoutError = worstLayoutError(planted, recovered);
      assertTrue(layoutError < 8.0e-3, "Layout error " + 1000.0 * layoutError + " mm exceeds 8 mm at σ = 0.93 mm.\n" + report.toTable());
   }

   /**
    * The scaling law behind the thresholds above, asserted rather than asserted-about.
    * <p>
    * Layout error on a distal link is dominated by
    * {@code σ · L / (√N · r_perp · √K)}: the gauge cluster's angular uncertainty times the lever
    * arm, averaged over captures. Two consequences follow, and both are checked here because both
    * drive decisions -- one buys a bracket, the other buys capture time.
    * </p>
    * <ul>
    * <li>Doubling the gauge cluster's spread halves the error. Cheap, and it is what
    * FRAMEWORK.md §1's outrigger bracket is for.</li>
    * <li>Quadrupling {@code K} halves the error. Expensive, and it is why "just take more
    * captures" is the weaker lever of the two.</li>
    * </ul>
    * <p>
    * Tolerances are wide because each point is a single seeded draw rather than an ensemble; the
    * claim under test is the exponent, not the constant.
    * </p>
    */
   @Test
   public void testLayoutErrorObeysTheGaugeClusterScalingLaw() throws Exception
   {
      // Widening the gauge cluster: error ∝ 1/r_perp, exactly as FRAMEWORK.md §1 says.
      double narrow = averagedRmsLayoutError(() -> new SyntheticCaptures.Options().captures(30).noise(SIGMA_TARGET).gaugeSpread(0.07), false);
      double wide = averagedRmsLayoutError(() -> new SyntheticCaptures.Options().captures(30).noise(SIGMA_TARGET).gaugeSpread(0.14), false);

      assertTrue(wide < narrow, "Widening the gauge cluster must reduce layout error: " + 1000 * narrow + " mm -> " + 1000 * wide + " mm.");
      assertEquals(2.0, narrow / wide, 0.7, "Error should scale as 1/r_perp: halving the spread should roughly double the error.");

      // More captures, gauge noise included: real but sub-√K improvement.
      double fewCaptures = averagedRmsLayoutError(() -> new SyntheticCaptures.Options().captures(30).noise(SIGMA_TARGET), false);
      double manyCaptures = averagedRmsLayoutError(() -> new SyntheticCaptures.Options().captures(480).noise(SIGMA_TARGET), false);

      assertTrue(manyCaptures < fewCaptures, "More captures must reduce layout error.");
      assertEquals(2.0, fewCaptures / manyCaptures, 0.8, "16× the captures buys only about 2×, not 4×. See below for why.");
   }

   /**
    * The decomposition behind the thresholds: <b>which</b> noise source costs what.
    *
    * <h2>The result</h2>
    * <p>
    * Measured at {@code σ = 0.3 mm}, RMS layout error over every marker, averaged over three seeds:
    * </p>
    *
    * <pre>
    *   K        all markers noisy      gauge cluster noiseless
    *   30           0.383 mm                0.0885 mm
    *   480          0.188 mm                0.0226 mm
    *   ratio         2.04×                    3.92×  ( = √16 )
    * </pre>
    *
    * <p>
    * Remove noise from the four pelvis markers and everything downstream behaves exactly as
    * FRAMEWORK.md §6 predicts: 0.0885 mm at {@code K = 30} against a {@code σ/√K} floor of
    * 0.055 mm, improving as a textbook {@code 1/√K}. So F4's averaging is correct and PR_PLAN.md's
    * 0.2 mm target <i>is</i> achievable -- for that term.
    * </p>
    * <p>
    * Put the gauge noise back and the error is 4.3× worse and the exponent breaks. Two distinct
    * things are happening:
    * </p>
    * <ol>
    * <li><b>Per-capture gauge noise</b> perturbs {@code ^W T_c^(k)} independently each capture, so
    * it does average as {@code 1/√K} -- but it enters amplified by the lever arm from the pelvis
    * out to each link, which is why distal links are worst.</li>
    * <li><b>Reference-shape noise does not average at all.</b> {@link BaseInitializer} defines the
    * cluster's shape from the marker positions of one reference capture, so that capture's noise
    * is baked into the frame definition for the entire session. No number of captures removes it,
    * and it is what bends the exponent from 0.5 to about 0.25.</li>
    * </ol>
    *
    * <h2>The fix, not implemented here</h2>
    * <p>
    * Item 2 is removable without giving up §8's monotonicity: run A′ to convergence, rebuild the
    * gauge shape from the <i>converged pelvis layout</i> (which is averaged over all {@code K}
    * captures and therefore {@code √K} quieter than any single capture), then run A′ again with
    * that new, still-fixed, {@code ^W T_c^(k)}. Each run minimises its own fixed objective, so
    * {@code J} is monotone within each run -- the guarantee is per-run, and that is all §8 claims.
    * </p>
    * <p>
    * It is left out of PR2 deliberately: it changes A′ from what FRAMEWORK.md §8 describes into a
    * two-stage method, and that is a specification decision rather than an implementation one.
    * This test exists to make the cost of not doing it a measured number.
    * </p>
    */
   @Test
   public void testTheGaugeClusterIsTheDominantErrorSource() throws Exception
   {
      double noisyGaugeFew = averagedRmsLayoutError(() -> new SyntheticCaptures.Options().captures(30).noise(SIGMA_TARGET), false);
      double cleanGaugeFew = averagedRmsLayoutError(() -> new SyntheticCaptures.Options().captures(30).noise(SIGMA_TARGET), true);
      double cleanGaugeMany = averagedRmsLayoutError(() -> new SyntheticCaptures.Options().captures(480).noise(SIGMA_TARGET), true);

      assertTrue(cleanGaugeFew < noisyGaugeFew / 2.0,
                 "The gauge cluster should dominate: " + 1000 * noisyGaugeFew + " mm with its noise, " + 1000 * cleanGaugeFew + " mm without.");

      // Without the gauge cluster's noise, F4's averaging is textbook.
      double theoreticalFloor = SIGMA_TARGET / Math.sqrt(30.0);
      assertTrue(cleanGaugeFew < 4.0 * theoreticalFloor,
                 "With a clean gauge cluster the error should approach σ/√K = " + 1000 * theoreticalFloor + " mm, was " + 1000 * cleanGaugeFew + " mm.");
      assertEquals(4.0, cleanGaugeFew / cleanGaugeMany, 1.0, "With a clean gauge cluster the scaling should be a clean 1/√K.");
   }

   /**
    * RMS layout error over every marker, averaged across seeds.
    * <p>
    * RMS rather than the worst marker, and averaged over seeds rather than a single draw, because
    * the claim under test is an exponent. A max over 28 markers from one seed is a noisy order
    * statistic -- it happened to read 8× where the law predicts 4× -- and tuning the tolerance
    * until that passes would be fitting the test to one random number.
    * </p>
    */
   private static double averagedRmsLayoutError(java.util.function.Supplier<SyntheticCaptures.Options> options, boolean noiselessGauge) throws Exception
   {
      double total = 0.0;
      int seeds = 3;

      for (int seed = 0; seed < seeds; seed++)
      {
         SyntheticCaptures.Planted planted = SyntheticCaptures.generate(options.get().seed(1000L + seed));

         if (noiselessGauge)
            SyntheticCaptures.denoiseCluster(planted, "pelvis");

         GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);
         CalibrationResult recovered = calibrate(planted, tracking, new CalibrationReport());

         double sumOfSquares = 0.0;
         int count = 0;

         for (MarkerCluster cluster : planted.clusters)
         {
            ClusterLayout truth = planted.plantedLayout(cluster.getLinkName());
            ClusterLayout estimate = recovered.getLayout(cluster.getLinkName());

            for (int j = 0; j < cluster.getMarkerCount(); j++)
            {
               double error = new Point3D(truth.getPositionInLinkFrame(j)).distance(new Point3D(estimate.getPositionInLinkFrame(j)));
               sumOfSquares += error * error;
               count++;
            }
         }

         total += Math.sqrt(sumOfSquares / count);
      }

      return total / seeds;
   }

   /**
    * <b>The property that justifies A′ over bundle adjustment.</b>
    * <p>
    * FRAMEWORK.md §8: each step is the exact global minimum of its subproblem, so {@code J} is
    * monotonically non-increasing by construction. Assert it, do not assume it -- if it ever rises,
    * one of the two solvers is not solving its subproblem and the entire reason for choosing this
    * method over a nonlinear least squares has evaporated.
    * </p>
    * <p>
    * Checked after <i>every half-step</i>, not once per iteration, so a failure names which solver
    * broke.
    * </p>
    */
   @Test
   public void testObjectiveIsMonotoneAcrossEveryHalfStep() throws Exception
   {
      for (long seed : new long[] {1L, 2L, 3L, 20260809L})
      {
         SyntheticCaptures.Planted planted = SyntheticCaptures.generate(new SyntheticCaptures.Options().seed(seed)
                                                                                                       .captures(20)
                                                                                                       .noise(SIGMA_CURRENT)
                                                                                                       .occlusion(0.1));
         GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);

         CalibrationReport report = new CalibrationReport();
         calibrate(planted, tracking, report);

         double[] sequence = report.getObjectiveSequence();

         for (int i = 1; i < sequence.length; i++)
         {
            assertTrue(sequence[i] <= sequence[i - 1] * (1.0 + 1.0e-12) + Double.MIN_NORMAL,
                       "J rose at step " + i + " (seed " + seed + "): " + sequence[i - 1] + " -> " + sequence[i]
                             + (i % 2 == 1 ? " across F5, the base step." : " across F4, the marker step.") + "\n" + report.toTable());
         }

         assertTrue(report.isMonotone(1.0e-12), "seed " + seed + "\n" + report.toTable());
      }
   }

   /** §8 caps A′ at 50 iterations; in practice it should be nowhere near that. */
   @Test
   public void testConvergesQuicklyAndReproducibly() throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(30).noise(SIGMA_TARGET));
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);

      CalibrationReport report = new CalibrationReport();
      CalibrationResult first = calibrate(planted, tracking, report);

      assertTrue(report.isConverged(), "A' hit the iteration cap with J still falling.\n" + report.toTable());

      // PR_PLAN.md says "under ~20"; the real figure here is around 100. A' converges linearly,
      // and the rate is damped by the gauge cluster: FRAMEWORK.md §7 notes that pelvis markers
      // "contribute nothing to Δ beyond a constant", and in the alternation they do something
      // slightly worse than nothing -- F4 re-derives their layout from the current Δ, so at the
      // next F5 they supply correspondences consistent with whatever Δ already is and pull against
      // the correction the leg markers are applying.
      //
      // Excluding them from F5 would converge much faster and is tempting. It is not done, because
      // F5 would then minimise only part of J and the half-step monotonicity of §8 -- the entire
      // stated reason for preferring A' to bundle adjustment -- would no longer hold. An iteration
      // is two closed-form solves over a few hundred correspondences; 100 of them is milliseconds.
      // Paying that to keep the guarantee is the right trade.
      assertTrue(report.getIterationCount() <= 150, "Took " + report.getIterationCount() + " iterations; expected under 150.\n" + report.toTable());

      // Deterministic: the same input must produce the same answer, bit for bit. A′ has no random
      // component, so anything else would mean hidden state between runs.
      CalibrationResult second = calibrate(planted, tracking, new CalibrationReport());
      assertTrue(first.getClusterToBase().epsilonEquals(second.getClusterToBase(), 1.0e-15), "A' is not deterministic.");
   }

   /**
    * Partial visibility is the normal case, not an edge case. {@code K_ij} bookkeeping has to be
    * right, because a marker seen in 3 of 30 captures is roughly three times noisier than one seen
    * in all 30 and nothing in the position itself says so.
    */
   @Test
   public void testPartialVisibility() throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(30)
                                                                                                    .noise(SIGMA_TARGET)
                                                                                                    .occlusion(0.2));
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);

      CalibrationReport report = new CalibrationReport();
      CalibrationResult recovered = calibrate(planted, tracking, report);

      assertTrue(recovered.isFullySolved(), "20% occlusion should not leave a marker unsolved.\n" + report.toTable());

      double layoutError = worstLayoutError(planted, recovered);
      assertTrue(layoutError < 3.5e-3, "Layout error " + 1000.0 * layoutError + " mm exceeds 3.5 mm under 20% occlusion.\n" + report.toTable());

      // K_ij must count the captures that actually contributed, and must be well under K.
      int totalObserved = 0;
      int totalPossible = 0;

      for (MarkerCluster cluster : planted.clusters)
      {
         ClusterLayout layout = recovered.getLayout(cluster.getLinkName());

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            assertTrue(layout.getObservationCount(j) > 0);
            assertTrue(layout.getObservationCount(j) <= tracking.getUsableCaptureCount(),
                       "K_ij = " + layout.getObservationCount(j) + " exceeds the usable capture count " + tracking.getUsableCaptureCount() + ".");
            totalObserved += layout.getObservationCount(j);
            totalPossible += tracking.getUsableCaptureCount();
         }
      }

      double observedFraction = (double) totalObserved / totalPossible;
      assertEquals(0.8, observedFraction, 0.05, "With 20% occlusion, about 80% of observations should have been counted.");
   }

   /**
    * A marker never seen at all must come out NaN with {@code K_ij = 0}, not at the link-frame
    * origin. A layout silently reading (0,0,0) is a marker position that looks calibrated.
    */
   @Test
   public void testNeverSeenMarkerStaysUnsolved() throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(10).noise(SIGMA_TARGET));

      // Blind one marker in every capture.
      var blinded = planted.clusters.get(1).getMarker(0);

      for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
         planted.captureSet.getCapture(k).getMocapFrame().get(blinded).setNotVisible();

      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);
      CalibrationResult recovered = calibrate(planted, tracking, new CalibrationReport());

      ClusterLayout layout = recovered.getLayout(planted.clusters.get(1).getLinkName());
      assertEquals(0, layout.getObservationCount(0));
      assertTrue(Double.isNaN(layout.getPositionInLinkFrame(0).getX()), "An unobserved marker must be NaN, not the origin.");
      assertTrue(!recovered.isFullySolved(), "A result with an unobserved marker is not fully solved, and must not claim to be.");

      // And the rest of the calibration must still be good.
      assertTrue(worstLayoutError(planted, recovered) < 3.5e-3);
   }

   /**
    * FRAMEWORK.md §7: pelvis-cluster markers contribute nothing to {@code Δ} beyond a constant --
    * the information comes from the marked links <b>below</b> the pelvis. So a capture set whose
    * legs never move should fail to identify {@code Δ}, and the base step's {@code σ₃} is where
    * that shows up.
    * <p>
    * This is the "looks converged and means nothing" failure FRAMEWORK.md warns about, and it is
    * worth having seen once.
    * </p>
    */
   @Test
   public void testFrozenLegsLeaveTheBaseStepPoorlyConditioned() throws Exception
   {
      SyntheticCaptures.Planted swept = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(20).noise(SIGMA_TARGET));
      CalibrationReport sweptReport = new CalibrationReport();
      calibrate(swept, BaseInitializer.trackGaugeCluster(swept.captureSet), sweptReport);

      SyntheticCaptures.Planted frozen = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(20).noise(SIGMA_TARGET).frozenJoints());
      CalibrationReport frozenReport = new CalibrationReport();
      CalibrationResult frozenResult = calibrate(frozen, BaseInitializer.trackGaugeCluster(frozen.captureSet), frozenReport);

      // The trap: the frozen session still converges to a small J. Nothing in the objective says
      // the answer is meaningless.
      assertTrue(frozenReport.isConverged(), "The frozen session converges -- that is precisely what makes it dangerous.");
      assertTrue(frozenResult.isFullySolved(), "And it produces a complete-looking result.");

      // What does say so is the conditioning of the base step.
      assertTrue(frozenReport.getBaseStepSigma3() < sweptReport.getBaseStepSigma3(),
                 "Frozen legs must condition the base step worse than a swept session. frozen σ₃=" + frozenReport.getBaseStepSigma3() + " swept σ₃="
                       + sweptReport.getBaseStepSigma3());
   }
}
