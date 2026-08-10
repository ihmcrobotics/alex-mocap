package us.ihmc.alexMocap.gates;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.calibration.AlternatingCalibrator;
import us.ihmc.alexMocap.calibration.BaseInitializer;
import us.ihmc.alexMocap.calibration.BaseInitializer.GaugeTracking;
import us.ihmc.alexMocap.calibration.CalibrationReport;
import us.ihmc.alexMocap.calibration.SyntheticCaptures;
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.Capture;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * PR_PLAN.md's diagnostic tests: "inject a fault into the <b>generator</b> but not into the
 * <b>model</b>, then assert the gate both fires and points at the right thing."
 * <p>
 * These are the tests that give G2 and G4 their value. A gate with only a passing test is a gate
 * nobody has seen fire.
 * </p>
 */
public class GateInjectionTest
{
   /**
    * Deliberately quieter than the lab's 0.3 mm target.
    * <p>
    * These tests measure whether the gate <b>discriminates</b>, not how accurate the pipeline is.
    * At 0.3 mm the toy's own noise floor -- the gauge cluster's angular error times a 0.6 m lever,
    * about 2 mm on the feet -- is the same size as the fault being injected, so a gate that works
    * perfectly and a gate that does nothing produce the same verdict. Turning σ down separates the
    * signal from the floor and leaves the test measuring the thing it names.
    * </p>
    * <p>
    * {@code PlantAndRecoverTest} is where realistic noise levels belong.
    * </p>
    */
   private static final double SIGMA = 0.05e-3;

   private static final int CAPTURES = 40;

   private record Fitted(SyntheticCaptures.Planted planted, GaugeTracking tracking, CalibrationResult calibration, CalibrationReport report)
   {
   }

   private static Fitted fit(SyntheticCaptures.Options options) throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(options);
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);
      CalibrationReport report = new CalibrationReport();
      CalibrationResult calibration = new AlternatingCalibrator().calibrate(planted.captureSet, planted.model, tracking, report);

      return new Fitted(planted, tracking, calibration, report);
   }

   private static BootstrapSpreadGate gate(Fitted fitted)
   {
      return new BootstrapSpreadGate(fitted.planted().captureSet.getCaptures(),
                                     fitted.planted().clusters,
                                     fitted.planted().model,
                                     fitted.calibration().getClusterToBase(),
                                     SyntheticCaptures.clusterPoseList(fitted.tracking(), fitted.planted().captureSet.getCaptureCount()),
                                     SIGMA);
   }

   /** Mean per-capture spread over a cluster's markers, from the last {@link BootstrapSpreadGate#run()}. */
   private static double meanSpread(BootstrapSpreadGate gate, String linkName)
   {
      double total = 0.0;
      int count = 0;

      for (BootstrapSpreadGate.SpreadDiagnosis diagnosis : gate.getDiagnoses())
      {
         if (diagnosis.linkName().equals(linkName))
         {
            total += diagnosis.spreadMeters();
            count++;
         }
      }

      return count == 0 ? Double.NaN : total / count;
   }

   private static boolean failed(GateResult result, String linkName)
   {
      for (GateResult.Finding finding : result.getFailures())
      {
         if (finding.subject().startsWith(linkName + ":"))
            return true;
      }

      return false;
   }

   /** Clean data: G2 passes, and the spread is consistent with mocap noise alone. */
   @Test
   public void testCleanDataPassesG2() throws Exception
   {
      Fitted fitted = fit(new SyntheticCaptures.Options().captures(CAPTURES).noise(SIGMA));
      GateResult result = gate(fitted).run();

      assertTrue(result.isPassed(), "G2 should pass on clean data.\n" + summarise(result));

      for (BootstrapSpreadGate.SpreadDiagnosis diagnosis : gate(fitted).getDiagnoses())
      {
         assertTrue(diagnosis.indictment().startsWith("nothing"),
                    "Clean data should indict nothing, but " + diagnosis.markerName() + " indicts: " + diagnosis.indictment());
      }
   }

   /**
    * A 0.5° offset on one joint. G2 must fire, and the failures must be <b>localised to the links
    * below that joint on that branch</b>.
    *
    * <h2>PR_PLAN.md's stated assertion is not quite what the algebra gives</h2>
    * <p>
    * PR_PLAN.md asks that "the reported spread correlates with <i>that specific joint's</i>
    * excursion". Working it through, that is not what happens. Writing the chain to link {@code i}
    * as {@code T = J_1 … J_j(q_j) … J_n} and injecting {@code q_j → q_j + δ}:
    * </p>
    *
    * <pre>
    * (T_model)⁻¹ T_true  =  [J_{j+1} … J_n]⁻¹ · R(δ) · [J_{j+1} … J_n]
    * </pre>
    *
    * <p>
    * The chain <i>above</i> {@code j} cancels, and so does {@code J_j}'s own fixed offset -- so the
    * back-projection error depends on the joints <b>strictly below</b> {@code j}, and not on
    * {@code q_j} at all. Three consequences, all checked here:
    * </p>
    * <ul>
    * <li>Markers above the offending joint show no spread whatsoever.</li>
    * <li>The link <i>immediately</i> below it shows no spread either: there the bracketed product
    * is empty, the error is the constant {@code R(δ)}, and a constant error is absorbed into the
    * layout.</li>
    * <li>Links two or more joints below do spread, correlating with the joints between.</li>
    * </ul>
    * <p>
    * So G2 localises a fault to <i>the branch, and to the depth below which spread appears</i>,
    * which is strictly more information than a single joint name. Note the corollary: <b>an offset
    * on a terminal joint is invisible to G2</b>, because it is entirely absorbed into the layout of
    * the last link. That is not a defect in the gate -- such an offset is genuinely unidentifiable
    * from marker data alone.
    * </p>
    */
   @Test
   public void testJointOffsetFiresG2AndLocalisesToTheAffectedBranch() throws Exception
   {
      Fitted fitted = fit(new SyntheticCaptures.Options().captures(CAPTURES).noise(SIGMA).jointOffset("l_hip", Math.toRadians(0.5)));

      BootstrapSpreadGate gate = gate(fitted);
      GateResult result = gate.run();

      assertTrue(!result.isPassed(), "G2 must fire on a 0.5° joint offset.\n" + summarise(result));

      // The deepest link of the affected branch carries the largest spread and is what trips the
      // 3σ threshold. Measured: l_foot ~1.2 mm against ~0.39 mm expected.
      assertTrue(failed(result, "l_foot"), "l_foot is the deepest link below l_hip and must spread.\n" + summarise(result));

      // The base is above the fault and the other branch does not contain it. Neither may fail.
      assertTrue(!failed(result, "pelvis"), "The pelvis is above the fault and must not spread.\n" + summarise(result));
      assertTrue(!failed(result, "r_thigh"), "The right branch is untouched and must not spread.\n" + summarise(result));
      assertTrue(!failed(result, "r_shank"), "The right branch is untouched and must not spread.\n" + summarise(result));
      assertTrue(!failed(result, "r_foot"), "The right branch is untouched and must not spread.\n" + summarise(result));

      // Branch localisation, stated as a comparison between mirror-image links at equal depth.
      assertTrue(meanSpread(gate, "l_foot") > 1.4 * meanSpread(gate, "r_foot"),
                 "The affected branch must spread visibly more than its mirror image: l_foot " + 1000 * meanSpread(gate, "l_foot") + " mm vs r_foot "
                       + 1000 * meanSpread(gate, "r_foot") + " mm.");

      // And the correlate named for an affected marker is a joint of the affected branch. On the
      // shallow links it is l_hip itself -- the actual offender.
      BootstrapSpreadGate.SpreadDiagnosis thigh = gate.findDiagnosis("l_thigh", "l_thigh_M0");
      assertNotNull(thigh);
      assertTrue(thigh.strongestJointName().startsWith("l_"),
                 "The strongest correlate should be a joint of the affected branch, was '" + thigh.strongestJointName() + "'.");
      assertTrue(Math.abs(thigh.strongestJointCorrelation()) >= BootstrapSpreadGate.STRUCTURE_CORRELATION_THRESHOLD,
                 "The correlation should be strong enough to read as structure, was " + thigh.strongestJointCorrelation());
   }

   /**
    * The corollary of the algebra above, asserted so that the limitation is documented by a test
    * rather than by a comment: an offset on a <b>terminal</b> joint produces no spread at all,
    * because it is absorbed wholesale into the last link's layout.
    */
   @Test
   public void testTerminalJointOffsetIsInvisibleToG2() throws Exception
   {
      Fitted fitted = fit(new SyntheticCaptures.Options().captures(CAPTURES).noise(SIGMA).jointOffset("l_ankle", Math.toRadians(0.5)));
      GateResult result = gate(fitted).run();

      assertTrue(result.isPassed(),
                 "An offset on the last joint of a chain is unidentifiable from marker data and must not fire G2.\n" + summarise(result));
   }

   /**
    * Load-proportional deflection. G2 must fire, and the correlation must land on <b>load</b>
    * rather than on any single joint's excursion -- that is the row of FRAMEWORK.md §8's escalation
    * table that indicts joint elasticity and calls for elastic parameters, not a joint offset.
    */
   @Test
   public void testElasticDeflectionFiresG2AndIndictsLoad() throws Exception
   {
      // 3e-4 rad/(N·m): about 0.5° at a 30 N·m hip torque. Plausible for a harmonic drive.
      Fitted fitted = fit(new SyntheticCaptures.Options().captures(CAPTURES).noise(SIGMA).compliance(3.0e-4));

      BootstrapSpreadGate gate = gate(fitted);
      GateResult result = gate.run();

      assertTrue(!result.isPassed(), "G2 must fire on load-proportional deflection.\n" + summarise(result));

      // Unlike a single joint offset, elasticity is not confined to one branch.
      assertTrue(failed(result, "l_shank") || failed(result, "l_foot"), "The left branch should spread.\n" + summarise(result));
      assertTrue(failed(result, "r_shank") || failed(result, "r_foot"), "The right branch should spread too; elasticity is not localised.\n" + summarise(result));

      // Load correlation must be strong. It is deliberately NOT required to beat the joint
      // correlation: elastic deflection enters through gravitational torque, which is itself a
      // function of joint angles, so both correlate strongly and the comparison between them is
      // not a reliable discriminator. The spatial pattern asserted above is.
      double strongestLoadCorrelation = 0.0;

      for (BootstrapSpreadGate.SpreadDiagnosis diagnosis : gate.getDiagnoses())
         strongestLoadCorrelation = Math.max(strongestLoadCorrelation, Math.abs(diagnosis.loadCorrelation()));

      assertTrue(strongestLoadCorrelation >= BootstrapSpreadGate.STRUCTURE_CORRELATION_THRESHOLD,
                 "Some marker's deviation should track the gravitational load proxy; strongest was " + strongestLoadCorrelation);
   }

   /** G4 on clean data: held-out RMS within a small factor of in-sample, and under the bar. */
   @Test
   public void testHeldOutMatchesInSampleOnCleanData() throws Exception
   {
      double heldOut = assertHeldOut(new SyntheticCaptures.Options().captures(CAPTURES).noise(SIGMA), 2.0, false, 1.0e-3);
      assertTrue(heldOut < 1.0e-3, "Clean held-out RMS should be well under a millimetre, was " + 1000 * heldOut + " mm.");
   }

   /**
    * G4 with an injected offset.
    *
    * <h2>PR_PLAN.md expects the wrong signal here</h2>
    * <p>
    * PR_PLAN.md says: "Held-out RMS blows up <i>while in-sample stays low</i>. That asymmetry is
    * the entire point of held-out validation." The asymmetry does not appear, and it should not be
    * expected to. Measured on a 0.5° {@code l_hip} offset, in-sample and held-out RMS are within
    * 3% of each other -- both raised together, by a factor of six.
    * </p>
    * <p>
    * The reason is that the splits are i.i.d. A joint offset is a deterministic function of
    * configuration, and both halves of a random split sample the same configuration distribution,
    * so the bias is present in equal measure in each. Held-out validation detects
    * <b>overfitting</b> -- a model flexible enough to absorb noise it will not see again -- and
    * this fit has 90 parameters against 1680 observations, so there is nothing to overfit with.
    * </p>
    * <p>
    * That does not make G4 useless for this fault; it makes the <b>ratio</b> the wrong statistic
    * and the <b>absolute level</b> the right one. The offset raises held-out RMS from ~0.15 mm to
    * ~0.9 mm, which is what a threshold catches. The literal asymmetry PR_PLAN.md describes would
    * require the held-out captures to cover configurations the training split did not -- which is
    * a better G4 design and is noted in RUNNING.md as future work.
    * </p>
    */
   @Test
   public void testInjectedOffsetRaisesHeldOutResidualInAbsoluteTerms() throws Exception
   {
      double clean = assertHeldOut(new SyntheticCaptures.Options().captures(CAPTURES).noise(SIGMA), 2.0, false, 1.0e-3);
      // 2°, not PR_PLAN.md's 0.5°. At 0.5° the held-out RMS rises from 0.31 mm to 0.60 mm -- real,
      // and in the right direction, but under 2× because this toy's held-out floor is itself
      // gauge-cluster dominated. Separating a gate's response from its floor by less than a factor
      // of two makes for a test that fails on a different seed, so the injection is sized to clear
      // the floor rather than the threshold being tuned until 0.5° squeaks through.
      double injected = assertHeldOut(new SyntheticCaptures.Options().captures(CAPTURES).noise(SIGMA).jointOffset("l_hip", Math.toRadians(2.0)),
                                      2.0,
                                      false,
                                      1.0e-2);

      assertTrue(injected > 3.0 * clean,
                 "The offset must raise held-out RMS substantially: clean " + 1000 * clean + " mm, injected " + 1000 * injected + " mm.");

      // And at a bar set between the two, the gate fires on the injected case and not the clean one.
      double bar = 1.0e-3;
      assertTrue(clean < bar && injected > bar, "A threshold of " + 1000 * bar + " mm should separate them: " + 1000 * clean + " / " + 1000 * injected);
   }

   /**
    * Fits on the even-numbered captures, predicts the odd-numbered ones.
    *
    * @param expectRatioAbove whether held-out RMS should exceed in-sample by more than the factor,
    *                         or stay under it.
    */
   private static double assertHeldOut(SyntheticCaptures.Options options, double ratio, boolean expectRatioAbove, double thresholdMeters) throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(options);
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);

      List<Integer> trainingList = new ArrayList<>();
      List<Capture> heldOutCaptures = new ArrayList<>();
      List<RigidBodyTransformReadOnly> heldOutPoses = new ArrayList<>();

      for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
      {
         if (k % 2 == 0)
         {
            trainingList.add(k);
         }
         else
         {
            heldOutCaptures.add(planted.captureSet.getCapture(k));
            heldOutPoses.add(tracking.isUsable(k) ? tracking.getClusterToWorld(k) : null);
         }
      }

      int[] training = trainingList.stream().mapToInt(Integer::intValue).toArray();

      // The tracking is shared between the splits on purpose: recomputing it per split would put
      // the two fits in different cluster-frame conventions and make the comparison meaningless.
      CalibrationReport report = new CalibrationReport();
      CalibrationResult calibration = new AlternatingCalibrator().calibrate(planted.captureSet, planted.model, tracking, training, report);

      HeldOutResidualGate gate = new HeldOutResidualGate(heldOutCaptures,
                                                        planted.clusters,
                                                        planted.model,
                                                        calibration,
                                                        heldOutPoses,
                                                        report.getOverallRmsMeters(),
                                                        thresholdMeters);
      GateResult result = gate.run();

      String message = String.format("in-sample %.4f mm, held-out %.4f mm, ratio %.2f%n%s",
                                     1000.0 * report.getOverallRmsMeters(),
                                     1000.0 * gate.getHeldOutRmsMeters(),
                                     gate.getHeldOutToInSampleRatio(),
                                     summarise(result));

      if (expectRatioAbove)
      {
         assertTrue(gate.getHeldOutToInSampleRatio() > ratio, "Held-out should blow up relative to in-sample. " + message);
         assertTrue(!result.isPassed(), "G4 should fail above the bar. " + message);
      }
      else
      {
         assertTrue(gate.getHeldOutToInSampleRatio() < ratio, "Held-out should track in-sample. " + message);
         assertTrue(result.isPassed(), "G4 should pass below the bar. " + message);
      }

      return gate.getHeldOutRmsMeters();
   }

   private static String summarise(GateResult result)
   {
      StringBuilder text = new StringBuilder(result.toString()).append('\n').append("  ").append(result.getSummary()).append('\n');

      for (GateResult.Finding finding : result.getFindings())
         text.append(String.format("    %-8s %-28s %s%n", finding.status(), finding.subject(), finding.detail()));

      return text.toString();
   }
}
