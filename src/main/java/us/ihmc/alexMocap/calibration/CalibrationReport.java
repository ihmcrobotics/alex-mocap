package us.ihmc.alexMocap.calibration;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.calibration.BaseInitializer.GaugeTracking;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.core.ObservationModel;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * What A′ did, recorded so that a converged calibration and a calibration that merely stopped are
 * distinguishable afterwards.
 * <p>
 * Two things live here. The <b>objective trace</b> -- {@code J} after every half-step -- which is
 * the evidence for FRAMEWORK.md §8's monotonicity claim rather than a restatement of it. And the
 * <b>per-marker residuals</b>, which are where §15's closing warning bites: a low overall RMS with
 * residuals correlated against {@code q^(k)} means the fit has absorbed a systematic error into the
 * marker positions, where it sits undetected and reappears as CoM bias at runtime.
 * </p>
 * <p>
 * <b>In-sample residuals are not an accuracy claim.</b> Everything here is measured on the data
 * that was fitted. G4 is what produces a number worth quoting.
 * </p>
 */
public class CalibrationReport
{
   /**
    * {@code J} after each of A′'s two half-steps in one iteration, in m².
    * <p>
    * Both halves are recorded, not just the iteration boundary, because §8's guarantee is per
    * <i>step</i>: F5 is the exact minimiser over {@code Δ} at fixed layouts and F4 is the exact
    * minimiser over layouts at fixed {@code Δ}. An increase in either half localises the bug to one
    * solver, where an iteration-level trace would only say that something is wrong.
    * </p>
    */
   public record IterationRecord(int iteration, double objectiveAfterBaseStep, double objectiveAfterMarkerStep, double relativeDecrease)
   {
   }

   /** Per-marker in-sample residual statistics, in metres. */
   public record MarkerResidual(String linkName, String markerName, int observationCount, double rmsMeters, double maxMeters)
   {
   }

   private final List<IterationRecord> iterations = new ArrayList<>();
   private final List<MarkerResidual> markerResiduals = new ArrayList<>();

   private double bootstrapObjective = Double.NaN;
   private int totalCaptureCount;
   private int usableCaptureCount;
   private int referenceCaptureIndex = -1;
   private double worstGaugeSigma3 = Double.NaN;
   private double baseStepSigma3 = Double.NaN;
   private boolean converged;
   private int observationCount;

   /** {@code J} after F3, before the loop starts. The first point of the trace. */
   public double getBootstrapObjective()
   {
      return bootstrapObjective;
   }

   void setBootstrapObjective(double bootstrapObjective)
   {
      this.bootstrapObjective = bootstrapObjective;
   }

   public List<IterationRecord> getIterations()
   {
      return iterations;
   }

   void addIteration(IterationRecord record)
   {
      iterations.add(record);
   }

   public List<MarkerResidual> getMarkerResiduals()
   {
      return markerResiduals;
   }

   void setMarkerResiduals(List<MarkerResidual> residuals)
   {
      markerResiduals.clear();
      markerResiduals.addAll(residuals);
   }

   public int getIterationCount()
   {
      return iterations.size();
   }

   /**
    * Whether A′ stopped because {@code J} stopped moving, rather than because it hit the iteration
    * cap with {@code J} still falling. The distinction belongs in the provenance of any result
    * anyone quotes.
    */
   public boolean isConverged()
   {
      return converged;
   }

   void setConverged(boolean converged)
   {
      this.converged = converged;
   }

   public int getTotalCaptureCount()
   {
      return totalCaptureCount;
   }

   /**
    * Captures the gauge cluster could actually be tracked into. Captures where it could not are
    * silently absent from every sum in A′, so the count belongs in the report -- a calibration that
    * used 4 of 30 captures should not look like one that used 30.
    */
   public int getUsableCaptureCount()
   {
      return usableCaptureCount;
   }

   public int getReferenceCaptureIndex()
   {
      return referenceCaptureIndex;
   }

   public double getWorstGaugeSigma3()
   {
      return worstGaugeSigma3;
   }

   /** {@code σ₃} of F5's Procrustes. Near zero means {@code Δ} was not identified by the data. */
   public double getBaseStepSigma3()
   {
      return baseStepSigma3;
   }

   /** Total number of {@code (i, j, k)} observations that entered {@code J}. */
   public int getObservationCount()
   {
      return observationCount;
   }

   void setTracking(GaugeTracking tracking, int totalCaptureCount)
   {
      this.totalCaptureCount = totalCaptureCount;
      this.usableCaptureCount = tracking.getUsableCaptureCount();
      this.referenceCaptureIndex = tracking.getReferenceCaptureIndex();
      this.worstGaugeSigma3 = tracking.getWorstSigma3();
   }

   void setBaseStepSigma3(double baseStepSigma3)
   {
      this.baseStepSigma3 = baseStepSigma3;
   }

   void setObservationCount(int observationCount)
   {
      this.observationCount = observationCount;
   }

   public double getFinalObjective()
   {
      if (iterations.isEmpty())
         return bootstrapObjective;

      return iterations.get(iterations.size() - 1).objectiveAfterMarkerStep();
   }

   /**
    * The whole objective trace: {@code J} after F3, then after each half-step, in order. This is
    * the sequence FRAMEWORK.md §8 requires to be non-increasing.
    */
   public double[] getObjectiveSequence()
   {
      double[] sequence = new double[1 + 2 * iterations.size()];
      sequence[0] = bootstrapObjective;

      for (int i = 0; i < iterations.size(); i++)
      {
         sequence[2 * i + 1] = iterations.get(i).objectiveAfterBaseStep();
         sequence[2 * i + 2] = iterations.get(i).objectiveAfterMarkerStep();
      }

      return sequence;
   }

   /**
    * Whether {@code J} never rose.
    *
    * @param relativeTolerance slack as a fraction of the previous value, to absorb floating-point
    *                          noise. It must be small: the point of the check is that a genuine
    *                          increase indicates a broken solver, and a loose tolerance would hide
    *                          exactly that.
    */
   public boolean isMonotone(double relativeTolerance)
   {
      double[] sequence = getObjectiveSequence();

      for (int i = 1; i < sequence.length; i++)
      {
         if (sequence[i] > sequence[i - 1] * (1.0 + relativeTolerance) + Double.MIN_NORMAL)
            return false;
      }

      return true;
   }

   /** In-sample RMS over every observation, in metres. Not an accuracy claim; see G4. */
   public double getOverallRmsMeters()
   {
      if (observationCount == 0)
         return Double.NaN;

      return Math.sqrt(getFinalObjective() / observationCount);
   }

   /**
    * {@code J} for a candidate {@code (Δ, layouts)}: FRAMEWORK.md §8's objective, summed over every
    * visible observation in the given captures.
    *
    * <pre>
    * J = sum_{i,j,k}  || ^W m_ijk  -  ^W T_c^(k) · Δ · ^b T_i(q^(k)) · ^i p_ij ||²
    * </pre>
    *
    * <p>
    * Observations are skipped where the marker was not visible, where the gauge cluster could not
    * be tracked, or where the layout is still unsolved. All three are "there is no prediction to
    * compare against", which is different from "the prediction was perfect" -- and adding zero for
    * them, which is what a naive loop does, is the second reading.
    * </p>
    */
   public static double computeObjective(CaptureSet captureSet,
                                         RobotModelHandle model,
                                         GaugeTracking tracking,
                                         RigidBodyTransformReadOnly clusterToBase,
                                         List<ClusterLayout> layouts)
   {
      return accumulate(captureSet, model, tracking, clusterToBase, layouts, everyCapture(captureSet), null);
   }

   /** {@code J} over a chosen subset of captures. G4 evaluates this on the held-out split. */
   public static double computeObjective(CaptureSet captureSet,
                                         RobotModelHandle model,
                                         GaugeTracking tracking,
                                         RigidBodyTransformReadOnly clusterToBase,
                                         List<ClusterLayout> layouts,
                                         int[] captureIndices)
   {
      return accumulate(captureSet, model, tracking, clusterToBase, layouts, captureIndices, null);
   }

   /** How many observations {@link #computeObjective} actually summed. */
   public static int countObservations(CaptureSet captureSet,
                                       RobotModelHandle model,
                                       GaugeTracking tracking,
                                       RigidBodyTransformReadOnly clusterToBase,
                                       List<ClusterLayout> layouts,
                                       int[] captureIndices)
   {
      int[] counter = new int[1];
      accumulate(captureSet, model, tracking, clusterToBase, layouts, captureIndices, (i, j, squared) -> counter[0]++);
      return counter[0];
   }

   /** Per-marker residual statistics over the given captures. */
   public static List<MarkerResidual> computeMarkerResiduals(CaptureSet captureSet,
                                                             RobotModelHandle model,
                                                             GaugeTracking tracking,
                                                             RigidBodyTransformReadOnly clusterToBase,
                                                             List<ClusterLayout> layouts,
                                                             int[] captureIndices)
   {
      List<MarkerCluster> clusters = captureSet.getClusters();
      double[][] sumOfSquares = new double[clusters.size()][];
      double[][] maximum = new double[clusters.size()][];
      int[][] counts = new int[clusters.size()][];

      for (int i = 0; i < clusters.size(); i++)
      {
         sumOfSquares[i] = new double[clusters.get(i).getMarkerCount()];
         maximum[i] = new double[clusters.get(i).getMarkerCount()];
         counts[i] = new int[clusters.get(i).getMarkerCount()];
      }

      accumulate(captureSet, model, tracking, clusterToBase, layouts, captureIndices, (i, j, squared) ->
      {
         sumOfSquares[i][j] += squared;
         maximum[i][j] = Math.max(maximum[i][j], Math.sqrt(squared));
         counts[i][j]++;
      });

      List<MarkerResidual> residuals = new ArrayList<>();

      for (int i = 0; i < clusters.size(); i++)
      {
         MarkerCluster cluster = clusters.get(i);

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            int count = counts[i][j];
            double rms = count == 0 ? Double.NaN : Math.sqrt(sumOfSquares[i][j] / count);
            double max = count == 0 ? Double.NaN : maximum[i][j];

            residuals.add(new MarkerResidual(cluster.getLinkName(), cluster.getMarker(j).getName(), count, rms, max));
         }
      }

      return residuals;
   }

   /** Receives one squared residual, tagged by cluster and marker index. */
   private interface ResidualVisitor
   {
      void accept(int clusterIndex, int markerIndex, double squaredResidual);
   }

   private static double accumulate(CaptureSet captureSet,
                                    RobotModelHandle model,
                                    GaugeTracking tracking,
                                    RigidBodyTransformReadOnly clusterToBase,
                                    List<ClusterLayout> layouts,
                                    int[] captureIndices,
                                    ResidualVisitor visitor)
   {
      List<MarkerCluster> clusters = captureSet.getClusters();
      RigidBodyTransform linkToBase = new RigidBodyTransform();
      RigidBodyTransform linkToWorld = new RigidBodyTransform();
      Point3D predicted = new Point3D();
      double objective = 0.0;

      for (int index : captureIndices)
      {
         if (!tracking.isUsable(index))
            continue;

         MocapFrame frame = captureSet.getCapture(index).getMocapFrame();
         model.setConfiguration(captureSet.getCapture(index).getEncoderSample());

         for (int i = 0; i < clusters.size(); i++)
         {
            MarkerCluster cluster = clusters.get(i);
            ClusterLayout layout = layouts.get(i);
            model.packLinkToBase(cluster.getLinkName(), linkToBase);
            ObservationModel.packLinkToWorld(tracking.getClusterToWorld(index), clusterToBase, linkToBase, linkToWorld);

            for (int j = 0; j < cluster.getMarkerCount(); j++)
            {
               MarkerObservation observation = frame.get(cluster.getMarker(j));

               if (!observation.isVisible() || layout.getObservationCount(j) == 0)
                  continue;

               ObservationModel.packPredictedMarkerPosition(linkToWorld, layout.getPositionInLinkFrame(j), predicted);
               double squared = ObservationModel.squaredResidual(observation.getPosition(), predicted);
               objective += squared;

               if (visitor != null)
                  visitor.accept(i, j, squared);
            }
         }
      }

      return objective;
   }

   static int[] everyCapture(CaptureSet captureSet)
   {
      int[] indices = new int[captureSet.getCaptureCount()];

      for (int k = 0; k < indices.length; k++)
         indices[k] = k;

      return indices;
   }

   /** A human-readable summary, for the CLI and for a failing test's message. */
   public String toTable()
   {
      StringBuilder table = new StringBuilder();
      table.append("A' calibration report\n");
      table.append(String.format("  captures            %d usable of %d (reference capture %d)%n", usableCaptureCount, totalCaptureCount,
                                 referenceCaptureIndex));
      table.append(String.format("  observations        %d%n", observationCount));
      table.append(String.format("  iterations          %d (%s)%n", iterations.size(), converged ? "converged" : "HIT ITERATION CAP"));
      table.append(String.format("  J after bootstrap   %.6e m^2%n", bootstrapObjective));
      table.append(String.format("  J final             %.6e m^2%n", getFinalObjective()));
      table.append(String.format("  in-sample RMS       %.4f mm  (NOT an accuracy claim; see G4)%n", 1000.0 * getOverallRmsMeters()));
      table.append(String.format("  monotone            %s%n", isMonotone(1.0e-9) ? "yes" : "NO -- a solver is wrong (FRAMEWORK.md section 8)"));
      table.append(String.format("  gauge worst sigma3  %.6e m^2%n", worstGaugeSigma3));
      table.append(String.format("  base step sigma3    %.6e m^2%n", baseStepSigma3));

      if (!markerResiduals.isEmpty())
      {
         table.append("  per-marker in-sample residuals\n");
         table.append(String.format("    %-12s %-14s %6s %10s %10s%n", "link", "marker", "K_ij", "rms (mm)", "max (mm)"));

         for (MarkerResidual residual : markerResiduals)
         {
            table.append(String.format("    %-12s %-14s %6d %10.4f %10.4f%n", residual.linkName(), residual.markerName(), residual.observationCount(),
                                       1000.0 * residual.rmsMeters(), 1000.0 * residual.maxMeters()));
         }
      }

      return table.toString();
   }

   @Override
   public String toString()
   {
      return "CalibrationReport[iterations=" + iterations.size() + ", J=" + getFinalObjective() + ", converged=" + converged + "]";
   }
}
