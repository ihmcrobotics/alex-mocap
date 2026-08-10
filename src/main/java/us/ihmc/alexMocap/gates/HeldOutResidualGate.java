package us.ihmc.alexMocap.gates;

import java.util.List;

import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.Capture;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.ObservationModel;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * G4, held-out reprojection (FRAMEWORK.md §15).
 * <p>
 * Withhold whole captures from the fit, predict their markers with the resulting calibration, and
 * report the RMS. <b>In-sample residuals are not an accuracy claim.</b> This is the number worth
 * quoting.
 * </p>
 *
 * <h2>Why whole captures</h2>
 * <p>
 * Withholding individual observations would leak: the base pose of a capture is determined by its
 * other markers, so a "held-out" marker in a capture that was otherwise fitted is predicted with
 * the benefit of that capture's own data. The resulting RMS is somewhere between in-sample and
 * held-out and means neither. {@code CaptureSet.subset} exists to make the whole-capture split the
 * easy thing to do.
 * </p>
 *
 * <h2>The asymmetry is the point</h2>
 * <p>
 * PR_PLAN.md's two G4 cases are a matched pair. On clean data, held-out RMS sits within a small
 * factor of in-sample. With an injected joint offset, held-out RMS blows up <i>while in-sample
 * stays low</i> -- because the fit absorbed the systematic error into the marker layouts, where it
 * is invisible to any in-sample statistic and then reappears as CoM bias at runtime. That
 * asymmetry is the entire reason held-out validation exists, and it is why this gate reports the
 * ratio alongside the absolute number.
 * </p>
 *
 * <h2>This gate does not fit anything</h2>
 * <p>
 * FRAMEWORK.md §19 forbids {@code gates → calibration}, so the caller fits on the training split
 * and hands the resulting {@link CalibrationResult} here along with the held-out captures. That is
 * also the honest division of labour: a gate that ran its own optimiser would be reporting on a fit
 * nobody else could reproduce.
 * </p>
 */
public class HeldOutResidualGate implements Gate
{
   /**
    * FRAMEWORK.md §15: "TALOS cross-validated ≈ 2.2 mm; with rigid mounts on a good volume you
    * should be under it." Literature, not a measurement of this robot.
    */
   public static final double TALOS_CROSS_VALIDATED_RMS_METERS = 2.2e-3;

   private final List<Capture> heldOutCaptures;
   private final List<MarkerCluster> clusters;
   private final RobotModelHandle model;
   private final CalibrationResult calibration;
   private final List<RigidBodyTransformReadOnly> clusterToWorld;
   private final double inSampleRmsMeters;
   private final double thresholdMeters;

   private double heldOutRmsMeters = Double.NaN;
   private int predictedObservationCount;

   /**
    * @param heldOutCaptures   captures withheld from the fit entirely.
    * @param clusters          every marked cluster.
    * @param model             the FK reference.
    * @param calibration       the result of fitting on the <i>other</i> captures.
    * @param clusterToWorld    {@code ^W T_c^(k)} for the held-out captures, in the same order and
    *                          in the same cluster-frame convention the fit used. {@code null} where
    *                          the gauge could not be tracked.
    * @param inSampleRmsMeters the fit's own in-sample RMS, reported for the ratio.
    * @param thresholdMeters   the bar. {@link #TALOS_CROSS_VALIDATED_RMS_METERS} is a reasonable
    *                          default for real hardware; a synthetic test should use something far
    *                          tighter.
    */
   public HeldOutResidualGate(List<Capture> heldOutCaptures,
                              List<MarkerCluster> clusters,
                              RobotModelHandle model,
                              CalibrationResult calibration,
                              List<RigidBodyTransformReadOnly> clusterToWorld,
                              double inSampleRmsMeters,
                              double thresholdMeters)
   {
      if (heldOutCaptures.size() != clusterToWorld.size())
         throw new IllegalArgumentException("Got " + heldOutCaptures.size() + " held-out captures but " + clusterToWorld.size() + " cluster poses.");

      this.heldOutCaptures = heldOutCaptures;
      this.clusters = clusters;
      this.model = model;
      this.calibration = calibration;
      this.clusterToWorld = clusterToWorld;
      this.inSampleRmsMeters = inSampleRmsMeters;
      this.thresholdMeters = thresholdMeters;
   }

   @Override
   public String getName()
   {
      return "G4";
   }

   @Override
   public String getDescription()
   {
      return "Held-out reprojection: predict markers in captures the fit never saw. "
            + "This is the accuracy claim; in-sample residuals are not (FRAMEWORK.md §15).";
   }

   @Override
   public GateResult run()
   {
      GateResult result = new GateResult(getName());

      RigidBodyTransform linkToBase = new RigidBodyTransform();
      RigidBodyTransform linkToWorld = new RigidBodyTransform();
      Point3D predicted = new Point3D();

      double totalSumOfSquares = 0.0;
      int totalCount = 0;

      for (MarkerCluster cluster : clusters)
      {
         ClusterLayout layout = calibration.findLayout(cluster.getLinkName());
         String subject = cluster.getLinkName();

         if (layout == null)
         {
            result.add(GateResult.Finding.notEvaluated(subject, 0, "The calibration has no layout for this link, so nothing can be predicted for it."));
            continue;
         }

         double clusterSumOfSquares = 0.0;
         int clusterCount = 0;

         for (int k = 0; k < heldOutCaptures.size(); k++)
         {
            if (clusterToWorld.get(k) == null)
               continue;

            model.setConfiguration(heldOutCaptures.get(k).getEncoderSample());
            model.packLinkToBase(cluster.getLinkName(), linkToBase);
            ObservationModel.packLinkToWorld(clusterToWorld.get(k), calibration.getClusterToBase(), linkToBase, linkToWorld);

            for (int j = 0; j < cluster.getMarkerCount(); j++)
            {
               MarkerObservation observation = heldOutCaptures.get(k).getMocapFrame().get(cluster.getMarker(j));

               if (!observation.isVisible() || layout.getObservationCount(j) == 0)
                  continue;

               ObservationModel.packPredictedMarkerPosition(linkToWorld, layout.getPositionInLinkFrame(j), predicted);
               clusterSumOfSquares += ObservationModel.squaredResidual(observation.getPosition(), predicted);
               clusterCount++;
            }
         }

         if (clusterCount == 0)
         {
            result.add(GateResult.Finding.notEvaluated(subject, 0, "No held-out observation of this cluster could be predicted."));
            continue;
         }

         double clusterRms = Math.sqrt(clusterSumOfSquares / clusterCount);
         totalSumOfSquares += clusterSumOfSquares;
         totalCount += clusterCount;

         String detail = String.format("held-out RMS %.4f mm over %d observations", 1000.0 * clusterRms, clusterCount);

         if (clusterRms > thresholdMeters)
            result.add(GateResult.Finding.fail(subject, clusterRms, thresholdMeters, clusterCount, detail));
         else
            result.add(GateResult.Finding.pass(subject, clusterRms, thresholdMeters, clusterCount, detail));
      }

      predictedObservationCount = totalCount;
      heldOutRmsMeters = totalCount == 0 ? Double.NaN : Math.sqrt(totalSumOfSquares / totalCount);

      result.setSummary(String.format("held-out RMS %.4f mm vs in-sample %.4f mm (ratio %.2f) over %d observations. %s",
                                      1000.0 * heldOutRmsMeters,
                                      1000.0 * inSampleRmsMeters,
                                      getHeldOutToInSampleRatio(),
                                      totalCount,
                                      getHeldOutToInSampleRatio() > 3.0
                                            ? "A held-out RMS far above in-sample means the fit absorbed a systematic error into the layouts."
                                            : "Held-out and in-sample agree; no sign of absorbed structure."));

      return result;
   }

   /** Overall held-out RMS in metres, across every cluster. Available after {@link #run()}. */
   public double getHeldOutRmsMeters()
   {
      return heldOutRmsMeters;
   }

   public int getPredictedObservationCount()
   {
      return predictedObservationCount;
   }

   /**
    * Held-out RMS divided by in-sample RMS. Near 1 on clean data. Large means the fit absorbed
    * structure it should not have -- which no in-sample statistic can show.
    */
   public double getHeldOutToInSampleRatio()
   {
      return inSampleRmsMeters > 0.0 ? heldOutRmsMeters / inSampleRmsMeters : Double.NaN;
   }
}
