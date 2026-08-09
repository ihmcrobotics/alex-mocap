package us.ihmc.alexMocap.calibration;

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
 * F4, the marker step (FRAMEWORK.md §6):
 *
 * <pre>
 * ^i p̂_ij  =  (1 / K_ij) · sum_k  ( ^W T_i^(k) )^-1 · ^W m_ijk
 * </pre>
 *
 * <h2>Why a plain mean is the exact answer</h2>
 * <p>
 * The normal equations of {@code min_p sum_k ||R^(k) p + t^(k) - m||²} are
 * {@code (sum_k R^(k)ᵀ R^(k)) p̂ = sum_k R^(k)ᵀ (m - t^(k))}. Every {@code R^(k)} is a rotation, so
 * {@code RᵀR = I}, the Gram matrix collapses to {@code K·I}, and the estimator is an unweighted
 * mean. No iteration, no initial guess, no local minimum, and -- the part that matters for A′ --
 * it is the exact global minimum of {@code J} over the layouts at fixed {@code Δ}.
 * </p>
 *
 * <h2>What averaging does and does not buy</h2>
 * <p>
 * It annihilates zero-mean mocap noise at {@code σ/√K}: 0.93 mm at {@code K = 30} becomes 0.17 mm.
 * It does <b>nothing</b> to a systematic error in {@code ^W T_i^(k)}. A joint offset, a wrong link
 * length, or gravity sag is correlated with configuration and survives the mean as a bias, so more
 * captures do not help. Pose diversity lets G2 <i>detect</i> such a bias but never <i>correct</i>
 * it.
 * </p>
 *
 * <h2>{@code K_ij} is per marker, not per session</h2>
 * <p>
 * Occlusion is normal, and a marker seen in 3 of 30 captures has a layout roughly three times
 * noisier than one seen in all 30, with nothing in the position itself to say so. The count is
 * written into {@link ClusterLayout} alongside each position for exactly that reason.
 * </p>
 *
 * <h2>Contract</h2>
 * <p>
 * Stateful and not thread safe: it owns accumulators sized to the layouts it was last given. One
 * instance per caller. After the first solve of a given shape it does not allocate, so the A′ loop
 * allocates nothing per iteration.
 * </p>
 */
public class MarkerLayoutSolver
{
   private final RigidBodyTransform linkToBase = new RigidBodyTransform();
   private final RigidBodyTransform linkToWorld = new RigidBodyTransform();
   private final Point3D backProjected = new Point3D();
   private final Point3D mean = new Point3D();

   /** {@code [cluster][3 * marker + axis]}, and the matching {@code K_ij}. */
   private double[][] sums = new double[0][];
   private int[][] counts = new int[0][];

   /**
    * Solves the layouts over every capture in the set.
    *
    * @param layoutsToPack one layout per cluster, in the same order as
    *                      {@link CaptureSet#getClusters()}. Overwritten entirely.
    */
   public void solve(CaptureSet captureSet,
                     RobotModelHandle model,
                     GaugeTracking tracking,
                     RigidBodyTransformReadOnly clusterToBase,
                     List<ClusterLayout> layoutsToPack)
   {
      int[] everyCapture = new int[captureSet.getCaptureCount()];

      for (int k = 0; k < everyCapture.length; k++)
         everyCapture[k] = k;

      solve(captureSet, model, tracking, clusterToBase, everyCapture, layoutsToPack);
   }

   /**
    * Solves the layouts over a chosen subset of captures.
    * <p>
    * The subset form is what makes F3 (§5) a special case of this method rather than a second
    * implementation of the same back-projection: a bootstrap is F4 over a single capture, where the
    * mean of one term is that term. It is also what G4 uses to fit on a training split.
    * </p>
    */
   public void solve(CaptureSet captureSet,
                     RobotModelHandle model,
                     GaugeTracking tracking,
                     RigidBodyTransformReadOnly clusterToBase,
                     int[] captureIndices,
                     List<ClusterLayout> layoutsToPack)
   {
      List<MarkerCluster> clusters = captureSet.getClusters();

      if (layoutsToPack.size() != clusters.size())
         throw new IllegalArgumentException("Expected " + clusters.size() + " layouts, got " + layoutsToPack.size() + ".");

      ensureCapacity(clusters);

      for (int i = 0; i < clusters.size(); i++)
      {
         java.util.Arrays.fill(sums[i], 0.0);
         java.util.Arrays.fill(counts[i], 0);
      }

      // Outer loop over captures, not over markers: setConfiguration runs the whole FK update, and
      // hoisting it out of the marker loops turns K*links*markers FK evaluations into K.
      for (int index : captureIndices)
      {
         if (!tracking.isUsable(index))
            continue;

         MocapFrame frame = captureSet.getCapture(index).getMocapFrame();
         model.setConfiguration(captureSet.getCapture(index).getEncoderSample());

         for (int i = 0; i < clusters.size(); i++)
         {
            MarkerCluster cluster = clusters.get(i);
            model.packLinkToBase(cluster.getLinkName(), linkToBase);
            ObservationModel.packLinkToWorld(tracking.getClusterToWorld(index), clusterToBase, linkToBase, linkToWorld);

            for (int j = 0; j < cluster.getMarkerCount(); j++)
            {
               MarkerObservation observation = frame.get(cluster.getMarker(j));

               if (!observation.isVisible())
                  continue;

               ObservationModel.packMarkerInLinkFrame(linkToWorld, observation.getPosition(), backProjected);

               sums[i][3 * j] += backProjected.getX();
               sums[i][3 * j + 1] += backProjected.getY();
               sums[i][3 * j + 2] += backProjected.getZ();
               counts[i][j]++;
            }
         }
      }

      for (int i = 0; i < clusters.size(); i++)
      {
         ClusterLayout layout = layoutsToPack.get(i);

         for (int j = 0; j < clusters.get(i).getMarkerCount(); j++)
         {
            int count = counts[i][j];

            if (count == 0)
            {
               // Never seen. NaN with K_ij = 0, not the origin: an unobserved marker must not read
               // as a marker calibrated to the link frame's origin.
               layout.setNotObserved(j);
               continue;
            }

            mean.set(sums[i][3 * j] / count, sums[i][3 * j + 1] / count, sums[i][3 * j + 2] / count);
            layout.setPositionInLinkFrame(j, mean, count);
         }
      }
   }

   private void ensureCapacity(List<MarkerCluster> clusters)
   {
      if (sums.length == clusters.size())
      {
         boolean shapeMatches = true;

         for (int i = 0; i < clusters.size(); i++)
         {
            if (counts[i].length != clusters.get(i).getMarkerCount())
               shapeMatches = false;
         }

         if (shapeMatches)
            return;
      }

      sums = new double[clusters.size()][];
      counts = new int[clusters.size()][];

      for (int i = 0; i < clusters.size(); i++)
      {
         sums[i] = new double[3 * clusters.get(i).getMarkerCount()];
         counts[i] = new int[clusters.get(i).getMarkerCount()];
      }
   }
}
