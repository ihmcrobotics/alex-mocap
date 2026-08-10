package us.ihmc.alexMocap.runtime;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.registration.RegistrationResult;
import us.ihmc.alexMocap.registration.RigidBodyRegistration;

/**
 * F6, runtime cluster-to-link pose (FRAMEWORK.md §9).
 * <p>
 * Per marked link, per frame: register the calibrated layout {@code {^i p̂_ij}} against the live
 * measurements {@code {^W m_ij}}, yielding {@code ^W T̂_i}.
 * </p>
 * <p>
 * <b>Encoders are not used here. This is the whole point.</b> Every other way of knowing where a
 * link is routes through the URDF and the joint sensors; this one routes through four markers and
 * a camera system, and that independence is what makes the result usable as ground truth for an
 * estimator that consumes those same encoders.
 * </p>
 *
 * <h2>Single frame, no averaging</h2>
 * <p>
 * Offline calibration divides mocap noise by {@code √K}. F6 does not: {@code σ} lands undiluted on
 * every runtime pose. Do not read the calibration's residuals as a runtime accuracy figure.
 * </p>
 *
 * <h2>This class owns the refusal policy, and it is the only one that does</h2>
 * <p>
 * The registration primitive reports numbers and decides nothing (§2), so somebody has to decide,
 * and §9 puts it here. Rank deficiency is <b>silent</b>: with fewer than three non-collinear
 * visible markers the SVD still returns a perfectly well-formed rotation, and nothing downstream
 * can detect it from the transform alone. Two independent guards:
 * </p>
 * <ul>
 * <li><b>Visible count.</b> Below three there is no pose under any conditions.</li>
 * <li><b>{@code σ₂}.</b> Three <i>collinear</i> markers pass the count check and still carry no
 * information about rotation about their common axis. Only a singular value sees that.</li>
 * </ul>
 *
 * <h2>The guard is {@code σ₂}, not {@code σ₃}, and the difference is not cosmetic</h2>
 * <p>
 * FRAMEWORK.md §9 and §18.1 both say to refuse on {@code σ₃}. <b>Refusing on {@code σ₃} rejects the
 * normal case.</b> Writing the layout's covariance as {@code C} with eigenvalues
 * {@code λ₁ ≥ λ₂ ≥ λ₃}:
 * </p>
 * <ul>
 * <li>{@code λ₃ ≈ 0} means the markers are <b>coplanar</b>. A plane is perfectly sufficient to fix
 * a 6-DOF pose -- three non-collinear points already do -- so this costs nothing. §2 says as much
 * in passing: "a noisy or near-planar cluster -- the realistic case, markers on a flat link
 * face".</li>
 * <li>{@code λ₂ ≈ 0} means the markers are <b>collinear</b>, and rotation about that line is
 * genuinely unobservable. This is the failure §18.1 is describing.</li>
 * </ul>
 * <p>
 * The two are easy to conflate because both are "a small singular value", and the cost of
 * conflating them is a gate that fires constantly on good data. Measured on the toy robot: a
 * four-marker limb cluster came out near-coplanar with a nominal {@code σ₃} of
 * {@code 3.1e-08 m²}, an out-of-plane extent of 0.17 mm -- <i>smaller than the 0.3 mm mocap
 * noise</i>. Its {@code σ₃} then fluctuated with the noise and refused roughly one frame in six,
 * on data where every pose was fine. Its {@code σ₂} was four orders of magnitude larger and
 * perfectly steady.
 * </p>
 * <p>
 * {@code σ₃} is still computed, still logged every frame, and still reported per §9 -- it is a
 * genuine measurement of how planar a cluster is, which is worth watching because a planar cluster
 * is where the Umeyama reflection guard earns its keep. It is simply not the right thing to refuse
 * on.
 * </p>
 *
 * <h2>Setting the threshold without getting the units wrong</h2>
 * <p>
 * These are mean-squared spreads with units of <b>length²</b>: a 120 mm cluster reports
 * {@code σ ≈ 0.003}, not {@code 0.12}. Picking an absolute threshold in those units is the easiest
 * way to be wrong by three orders of magnitude, so this class does not ask for one. It computes
 * each cluster's <i>nominal</i> {@code σ₂} from the calibrated layout at construction, and refuses
 * when the live value falls below a fraction of it.
 * </p>
 * <p>
 * The nominal value is exact rather than estimated. Registering the layout against itself gives
 * {@code H = R·C} with {@code C} the layout's own covariance, and a rotation leaves singular values
 * alone -- so the nominal values are the eigenvalues of the marker cloud's covariance, obtained
 * here by running the same primitive F6 uses. One SVD implementation, as §2 requires.
 * </p>
 *
 * <h2>Contract</h2>
 * <p>
 * Stateful and <b>not thread safe</b>: it owns a {@link RigidBodyRegistration} and its scratch.
 * One instance per caller. Allocation-free per frame once constructed.
 * </p>
 */
public class LinkPoseEstimator
{
   /**
    * Refuse when live {@code σ₂} falls below this fraction of the layout's nominal {@code σ₂}.
    * <p>
    * <b>Chosen</b>, not derived. Losing one marker of four typically takes {@code σ₂} to roughly
    * half its nominal value, so 0.25 refuses collapse toward collinearity while tolerating ordinary
    * occlusion. Tighten it once the visible-count histograms of FRAMEWORK.md §20.4 exist and say
    * what ordinary looks like on this robot.
    * </p>
    */
   public static final double DEFAULT_SIGMA2_FRACTION = 0.25;

   private final List<MarkerCluster> clusters;
   private final List<ClusterLayout> layouts;
   private final double[] nominalSigma2;
   private final double[] sigma2Threshold;
   private final double[] nominalSigma3;
   private final int[] linkIndices;

   private final RigidBodyRegistration registration;
   private final RegistrationResult result = new RegistrationResult();

   private final int minimumVisibleMarkers;

   /**
    * @param calibration the layouts F4 produced.
    * @param clusters    the marked clusters, which must all have a layout in the calibration.
    * @param linkNames   the link order of the {@link MeasuredLinkPoses} this will fill.
    */
   public LinkPoseEstimator(CalibrationResult calibration, List<MarkerCluster> clusters, List<String> linkNames)
   {
      this(calibration, clusters, linkNames, RigidBodyRegistration.MINIMUM_CORRESPONDENCES, DEFAULT_SIGMA2_FRACTION);
   }

   public LinkPoseEstimator(CalibrationResult calibration,
                            List<MarkerCluster> clusters,
                            List<String> linkNames,
                            int minimumVisibleMarkers,
                            double sigma2Fraction)
   {
      if (minimumVisibleMarkers < RigidBodyRegistration.MINIMUM_CORRESPONDENCES)
         throw new IllegalArgumentException("Fewer than " + RigidBodyRegistration.MINIMUM_CORRESPONDENCES
               + " markers cannot produce a pose under any conditions; asking for " + minimumVisibleMarkers + " is a configuration error.");
      if (!(sigma2Fraction >= 0.0) || sigma2Fraction >= 1.0)
         throw new IllegalArgumentException("The σ₂ fraction must be in [0, 1), was " + sigma2Fraction + ".");

      this.clusters = List.copyOf(clusters);
      this.minimumVisibleMarkers = minimumVisibleMarkers;
      this.layouts = new ArrayList<>(clusters.size());
      this.nominalSigma2 = new double[clusters.size()];
      this.sigma2Threshold = new double[clusters.size()];
      this.nominalSigma3 = new double[clusters.size()];
      this.linkIndices = new int[clusters.size()];

      int largestCluster = RigidBodyRegistration.MINIMUM_CORRESPONDENCES;

      for (MarkerCluster cluster : clusters)
         largestCluster = Math.max(largestCluster, cluster.getMarkerCount());

      this.registration = new RigidBodyRegistration(largestCluster);

      for (int i = 0; i < this.clusters.size(); i++)
      {
         MarkerCluster cluster = this.clusters.get(i);
         ClusterLayout layout = calibration.findLayout(cluster.getLinkName());

         if (layout == null)
            throw new IllegalArgumentException("The calibration has no layout for '" + cluster.getLinkName() + "'. Known: " + calibration.getLinkNames() + ".");
         if (!layout.isFullySolved())
            throw new IllegalArgumentException("The layout for '" + cluster.getLinkName()
                  + "' has unsolved markers. A NaN layout position would poison every pose it entered.");

         layouts.add(layout);
         computeNominalConditioning(layout, i);
         sigma2Threshold[i] = sigma2Fraction * nominalSigma2[i];

         int index = linkNames.indexOf(cluster.getLinkName());

         if (index < 0)
            throw new IllegalArgumentException("Link '" + cluster.getLinkName() + "' is not among the reported links " + linkNames + ".");

         linkIndices[i] = index;
      }
   }

   /**
    * The singular values a fully visible, undistorted cluster produces: the eigenvalues of the
    * calibrated layout's own covariance, obtained by registering the layout against itself.
    */
   private void computeNominalConditioning(ClusterLayout layout, int clusterIndex)
   {
      registration.clear();

      for (int j = 0; j < layout.getMarkerCount(); j++)
         registration.addCorrespondence(layout.getPositionInLinkFrame(j), layout.getPositionInLinkFrame(j));

      if (!registration.compute(result))
         throw new IllegalArgumentException("The layout for '" + layout.getLinkName() + "' has fewer than "
               + RigidBodyRegistration.MINIMUM_CORRESPONDENCES + " markers.");

      nominalSigma2[clusterIndex] = result.getSigma2();
      nominalSigma3[clusterIndex] = result.getSigma3();

      if (!(nominalSigma2[clusterIndex] > 0.0))
         throw new IllegalArgumentException("The calibrated layout for '" + layout.getLinkName()
               + "' is collinear (σ₂ = 0): its markers lie on a line, so rotation about that line is unobservable and no pose it "
               + "produces can be trusted. This is a mounting problem, not a software one (FRAMEWORK.md §1).");
   }

   /**
    * Estimates every marked link's pose from one mocap frame, packing poses and refusals into
    * {@code toPack}.
    * <p>
    * Unmarked links are left untouched -- {@link KinematicChainCoupler} fills those.
    * </p>
    */
   public void estimate(MocapFrame frame, MeasuredLinkPoses toPack)
   {
      for (int i = 0; i < clusters.size(); i++)
      {
         MarkerCluster cluster = clusters.get(i);
         ClusterLayout layout = layouts.get(i);
         int linkIndex = linkIndices[i];

         registration.clear();
         int visible = 0;

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            MarkerObservation observation = frame.get(cluster.getMarker(j));

            if (!observation.isVisible())
               continue;

            registration.addCorrespondence(layout.getPositionInLinkFrame(j), observation.getPosition());
            visible++;
         }

         if (visible < minimumVisibleMarkers)
         {
            toPack.setRefused(linkIndex,
                              Double.NaN,
                              visible,
                              "only " + visible + " of " + cluster.getMarkerCount() + " markers visible; " + minimumVisibleMarkers + " required");
            continue;
         }

         if (!registration.compute(result))
         {
            toPack.setRefused(linkIndex, Double.NaN, visible, "registration could not produce a pose from " + visible + " correspondences");
            continue;
         }

         // The guard that count alone cannot provide: three collinear markers pass the check above
         // and still say nothing about rotation about their own axis. σ₂, not σ₃ -- see the class
         // javadoc; σ₃ merely says the cluster is flat, which is normal and harmless.
         if (result.getSigma2() < sigma2Threshold[i])
         {
            toPack.setRefused(linkIndex,
                              result.getSigma3(),
                              visible,
                              String.format("σ₂ %.3e m² below the %.3e m² threshold (%.0f%% of the layout's nominal %.3e m²); "
                                    + "the visible markers are too near collinear to fix a rotation",
                                            result.getSigma2(),
                                            sigma2Threshold[i],
                                            100.0 * sigma2Threshold[i] / nominalSigma2[i],
                                            nominalSigma2[i]));
            continue;
         }

         toPack.setMeasured(linkIndex, result.getTransform(), result.getSigma3(), visible);
      }
   }

   public List<MarkerCluster> getClusters()
   {
      return clusters;
   }

   /** The {@code σ₂} a fully visible cluster produces, m². This is what the refusal compares against. */
   public double getNominalSigma2(int clusterIndex)
   {
      return nominalSigma2[clusterIndex];
   }

   /**
    * The {@code σ₃} a fully visible cluster produces, m².
    * <p>
    * Reported because §9 asks for it and because it measures how planar the cluster is -- a small
    * value is where the Umeyama reflection guard matters. It is <b>not</b> what refusal is based
    * on.
    * </p>
    */
   public double getNominalSigma3(int clusterIndex)
   {
      return nominalSigma3[clusterIndex];
   }

   public double getSigma2Threshold(int clusterIndex)
   {
      return sigma2Threshold[clusterIndex];
   }

   public int getMinimumVisibleMarkers()
   {
      return minimumVisibleMarkers;
   }
}
