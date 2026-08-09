package us.ihmc.alexMocap.calibration;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.calibration.BaseInitializer.GaugeTracking;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * F3, the single-capture bootstrap -- the "software T-pose" (FRAMEWORK.md §5).
 *
 * <pre>
 * ^i p_ij^(0)  =  ( ^W T_c^(0) · ^b T_i(q^(0)) )^-1 · ^W m_ij0
 * </pre>
 *
 * <p>
 * With {@code Δ = I} from F2, this back-projects every visible marker of one capture into its link
 * frame. No averaging, and it inherits the full arbitrary offset of the cluster frame -- which is
 * precisely what F5 then solves for. Status: <b>derived</b>, exact given the model, and it needs
 * exactly one capture.
 * </p>
 *
 * <h2>Why this is a thin wrapper and not an implementation</h2>
 * <p>
 * F3 <i>is</i> F4 restricted to a single capture. The back-projection is the same operation and the
 * mean of one term is that term, so writing the arithmetic out again here would be a second
 * implementation of §6's formula that could drift from the first. What F3 contributes that F4 does
 * not is the <b>choice of capture</b> and the guarantee that {@code Δ = I} is what it runs with.
 * </p>
 *
 * <h2>The name is a promise about hardware, not software</h2>
 * <p>
 * "Software T-pose" means no jig, no zeroing fixture, and no operator posing the robot to a
 * reference configuration. Any capture will do; the bootstrap capture is a pointer into data, not a
 * pose someone has to achieve. That is what makes the whole method rig-free.
 * </p>
 */
public class BootstrapSolver
{
   private final MarkerLayoutSolver layoutSolver = new MarkerLayoutSolver();

   /**
    * Builds a fresh set of empty layouts, one per cluster, in {@link CaptureSet#getClusters()}
    * order.
    * <p>
    * Positions start NaN and {@code K_ij} starts zero, so a layout that never gets solved cannot be
    * mistaken for one solved to the origin.
    * </p>
    */
   public static List<ClusterLayout> createEmptyLayouts(CaptureSet captureSet)
   {
      List<ClusterLayout> layouts = new ArrayList<>(captureSet.getClusters().size());

      for (MarkerCluster cluster : captureSet.getClusters())
         layouts.add(new ClusterLayout(cluster));

      return layouts;
   }

   /**
    * Bootstraps from the gauge tracking's reference capture.
    * <p>
    * Using the reference capture rather than an arbitrary one is deliberate: there
    * {@code ^W T_c = I} by construction, so the bootstrap is the cleanest possible statement of
    * §5's formula, and it is guaranteed to be a capture in which the gauge cluster was well seen.
    * </p>
    */
   public List<ClusterLayout> bootstrap(CaptureSet captureSet, RobotModelHandle model, GaugeTracking tracking)
   {
      return bootstrap(captureSet, model, tracking, tracking.getReferenceCaptureIndex());
   }

   /** Bootstraps from a chosen capture. */
   public List<ClusterLayout> bootstrap(CaptureSet captureSet, RobotModelHandle model, GaugeTracking tracking, int bootstrapCaptureIndex)
   {
      if (!tracking.isUsable(bootstrapCaptureIndex))
         throw new IllegalArgumentException("Capture " + bootstrapCaptureIndex + " has no gauge-cluster pose (only " + tracking.getCoVisibleCount(
               bootstrapCaptureIndex) + " markers co-visible with the reference), so nothing can be back-projected from it.");

      List<ClusterLayout> layouts = createEmptyLayouts(captureSet);

      RigidBodyTransform initialClusterToBase = new RigidBodyTransform();
      BaseInitializer.packInitialClusterToBase(initialClusterToBase);

      layoutSolver.solve(captureSet, model, tracking, initialClusterToBase, new int[] {bootstrapCaptureIndex}, layouts);

      return layouts;
   }
}
