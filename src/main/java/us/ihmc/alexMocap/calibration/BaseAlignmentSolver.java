package us.ihmc.alexMocap.calibration;

import java.util.List;

import us.ihmc.alexMocap.calibration.BaseInitializer.GaugeTracking;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.registration.RegistrationResult;
import us.ihmc.alexMocap.registration.RigidBodyRegistration;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * F5, the base step (FRAMEWORK.md §7): recover {@code Δ = ^c T_b}, the single global offset from
 * the Motive cluster frame to the URDF pelvis link frame.
 *
 * <h2>One unknown, not K unknowns</h2>
 * <p>
 * The move that makes A′ cheap is in §7's first paragraph. In the hanging configuration the base
 * pose is not one free unknown per capture plus a fixture; it is the <i>measured</i> pelvis cluster
 * pose per capture, times one global constant:
 * </p>
 *
 * <pre>
 * ^W T_b^(k)  =  ^W T_c^(k) · Δ
 * </pre>
 *
 * <p>
 * {@code ^W T_c^(k)} is raw data (see {@link BaseInitializer}), so the base step has six unknowns
 * total no matter how many captures there are.
 * </p>
 *
 * <h2>And it is a single Procrustes</h2>
 * <p>
 * For every visible {@code (i, j, k)}:
 * </p>
 *
 * <pre>
 * a_ijk = ^b T_i(q^(k)) · ^i p_ij          (predicted point, base frame)
 * b_ijk = ( ^W T_c^(k) )^-1 · ^W m_ijk     (measurement pulled into cluster frame)
 * </pre>
 *
 * <p>
 * The model says {@code b_ijk = Δ · a_ijk}. Stacking every triple and applying the §2 registration
 * primitive gives {@code Δ} in closed form -- one registration over all captures and all links at
 * once, which is the exact global minimiser of {@code J} at fixed layouts.
 * </p>
 * <p>
 * That last claim needs one line of justification, since {@code J} is written in world coordinates
 * and this solve is done in cluster coordinates. Pulling a residual through
 * {@code ^W T_c^(k)} does not change its length, because a rigid transform is an isometry:
 * {@code ||m - ^W T_c Δ a|| = ||^W T_c (b - Δ a)|| = ||b - Δ a||}. The two objectives are
 * term-by-term identical.
 * </p>
 *
 * <h2>Where the information actually comes from</h2>
 * <p>
 * §7 closes with a warning worth repeating at the code: pelvis-cluster markers contribute nothing
 * to {@code Δ} beyond a constant. Their {@code ^b T_i} is the identity and their {@code b_ijk} is
 * the same point every capture, so they pin a constant and say nothing about orientation. All the
 * information comes from the marked links <b>below</b> the pelvis. A calibration run with the legs
 * held still can look perfectly converged and mean nothing, which is why leg marking and wide joint
 * excursion are requirements rather than suggestions.
 * </p>
 *
 * <h2>Contract</h2>
 * <p>
 * Stateful and not thread safe; owns the registration and its scratch. One instance per caller.
 * </p>
 */
public class BaseAlignmentSolver
{
   private final RigidBodyRegistration registration;
   private final RegistrationResult result = new RegistrationResult();

   private final RigidBodyTransform linkToBase = new RigidBodyTransform();
   private final RigidBodyTransform worldToCluster = new RigidBodyTransform();
   private final Point3D predictedInBase = new Point3D();
   private final Point3D measuredInCluster = new Point3D();

   public BaseAlignmentSolver()
   {
      this(RigidBodyRegistration.MINIMUM_CORRESPONDENCES);
   }

   /**
    * @param initialCapacity correspondences to preallocate: {@code links × markers × captures} for
    *                        the worst case. Sized right, the solve never allocates.
    */
   public BaseAlignmentSolver(int initialCapacity)
   {
      registration = new RigidBodyRegistration(Math.max(initialCapacity, RigidBodyRegistration.MINIMUM_CORRESPONDENCES));
   }

   /** Solves over every capture in the set. */
   public boolean solve(CaptureSet captureSet, RobotModelHandle model, GaugeTracking tracking, List<ClusterLayout> layouts, RigidBodyTransform deltaToPack)
   {
      int[] everyCapture = new int[captureSet.getCaptureCount()];

      for (int k = 0; k < everyCapture.length; k++)
         everyCapture[k] = k;

      return solve(captureSet, model, tracking, layouts, everyCapture, deltaToPack);
   }

   /**
    * Solves over a chosen subset of captures.
    *
    * @return whether a {@code Δ} was produced. False means fewer than three usable correspondences
    *         existed in the whole subset, in which case {@code deltaToPack} is NaN. As always with
    *         the registration primitive, {@code true} says the solve ran, not that the answer is
    *         good.
    */
   public boolean solve(CaptureSet captureSet,
                        RobotModelHandle model,
                        GaugeTracking tracking,
                        List<ClusterLayout> layouts,
                        int[] captureIndices,
                        RigidBodyTransform deltaToPack)
   {
      List<MarkerCluster> clusters = captureSet.getClusters();
      registration.clear();

      for (int index : captureIndices)
      {
         if (!tracking.isUsable(index))
            continue;

         MocapFrame frame = captureSet.getCapture(index).getMocapFrame();
         model.setConfiguration(captureSet.getCapture(index).getEncoderSample());

         worldToCluster.setAndInvert(tracking.getClusterToWorld(index));

         for (int i = 0; i < clusters.size(); i++)
         {
            MarkerCluster cluster = clusters.get(i);
            ClusterLayout layout = layouts.get(i);
            model.packLinkToBase(cluster.getLinkName(), linkToBase);

            for (int j = 0; j < cluster.getMarkerCount(); j++)
            {
               MarkerObservation observation = frame.get(cluster.getMarker(j));

               if (!observation.isVisible())
                  continue;

               // A marker with no layout yet -- never seen in the bootstrap capture -- contributes
               // no correspondence. Its NaN position would otherwise poison the entire cross
               // covariance and take Δ with it.
               if (layout.getObservationCount(j) == 0)
                  continue;

               predictedInBase.set(layout.getPositionInLinkFrame(j));
               linkToBase.transform(predictedInBase);

               measuredInCluster.set(observation.getPosition());
               worldToCluster.transform(measuredInCluster);

               registration.addCorrespondence(predictedInBase, measuredInCluster);
            }
         }
      }

      boolean solved = registration.compute(result);
      deltaToPack.set(result.getTransform());

      return solved;
   }

   /** Conditioning of the last solve. {@code σ₃ ≈ 0} means {@code Δ} is not identified. */
   public RegistrationResult getLastResult()
   {
      return result;
   }
}
