package us.ihmc.alexMocap.calibration;

import us.ihmc.alexMocap.core.Capture;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.registration.RegistrationResult;
import us.ihmc.alexMocap.registration.RigidBodyRegistration;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * F2, base initialisation (FRAMEWORK.md §4), and the gauge-cluster tracking F5 needs (§7).
 *
 * <h2>What §4 actually requires</h2>
 * <p>
 * §4 says: initialise {@code Δ = I}, so {@code ^W T_b^(0) = ^W T_c^(0)}, "the raw pelvis cluster
 * pose from F6 with an arbitrary cluster-frame convention". Two things have to be produced for that
 * sentence to mean anything, and only the first is obvious.
 * </p>
 * <ol>
 * <li>{@code Δ = I}. One line. Status: <b>chosen</b> -- F5 absorbs the error, this only has to be
 * close enough to start the loop.</li>
 * <li><b>The arbitrary cluster-frame convention itself</b>, and with it {@code ^W T_c^(k)} for
 * every capture. §7 says these "come from F6 applied to the pelvis cluster", but F6 needs a
 * calibrated layout, which does not exist yet. That is a real circularity, and this class is where
 * it is cut.</li>
 * </ol>
 *
 * <h2>How the circularity is cut</h2>
 * <p>
 * The cluster's <b>shape</b> is defined as the raw gauge-marker positions at one reference capture,
 * taken relative to their centroid. {@code ^W T_c^(k)} is then the rigid motion carrying that shape
 * onto the constellation observed at capture {@code k} -- computable from raw mocap alone, with no
 * layout, no URDF, no encoders. This is exactly the arbitrary convention §4 licenses, and it is the
 * same thing Motive does when it defines a rigid body from a marker set.
 * </p>
 *
 * <h2>Why the shape is centred on its centroid</h2>
 * <p>
 * This looks cosmetic and is not. The obvious alternative -- use the reference positions as they
 * come, so that {@code ^W T_c^(reference) = I} -- puts the cluster frame's origin at the
 * <b>world</b> origin rather than at the markers. {@code Δ = ^c T_b} then has to carry the entire
 * distance from the room's origin to the robot, several metres of it, and F2's {@code Δ = I} is no
 * longer "close enough to start the loop" in any sense: A′ starts several metres from the solution
 * and settles into a local minimum with layouts metres out. It converges, monotonically, to the
 * wrong answer.
 * </p>
 * <p>
 * Centred, {@code Δ}'s translation is the offset from the marker centroid to the pelvis link
 * origin -- centimetres -- and its rotation is the base orientation, which for a robot hanging
 * upright is near identity. {@code Δ = I} is then a genuinely good starting point, which is what
 * §4 was assuming all along.
 * </p>
 * <p>
 * Nothing downstream depends on the choice, because {@code Δ} is only ever used in the composition
 * {@code ^W T_c^(k) · Δ}, in which the convention cancels. It matters only for conditioning -- but
 * it matters completely.
 * </p>
 *
 * <h2>Computed once, never updated</h2>
 * <p>
 * A tempting refinement is to recompute {@code ^W T_c^(k)} each A′ iteration from the freshly
 * solved pelvis layout, which is what §7's "F6 applied to the pelvis cluster" literally suggests.
 * <b>Do not.</b> A′'s monotonicity argument (§8) is that each step is the exact global minimum of
 * its subproblem <i>of a fixed objective</i>. Moving {@code ^W T_c^(k)} between iterations changes
 * {@code J} itself, and a decreasing sequence of values of different functions says nothing at all.
 * These poses are raw-data constants, fixed before the loop starts.
 * </p>
 */
public class BaseInitializer
{
   /**
    * Gauge-cluster tracking for a whole capture set, plus the initial {@code Δ}.
    * <p>
    * {@code σ₃} and the co-visible count are carried per capture because rank deficiency is silent
    * (FRAMEWORK.md §18.1): a gauge cluster reduced to three near-collinear visible markers still
    * yields a well-formed rotation, and nothing in the transform says so.
    * </p>
    */
   public static class GaugeTracking
   {
      private final RigidBodyTransform[] clusterToWorld;
      private final double[] sigma3;
      private final int[] coVisibleCount;
      private final boolean[] usable;
      private final int referenceCaptureIndex;

      private GaugeTracking(int captureCount, int referenceCaptureIndex)
      {
         this.clusterToWorld = new RigidBodyTransform[captureCount];
         this.sigma3 = new double[captureCount];
         this.coVisibleCount = new int[captureCount];
         this.usable = new boolean[captureCount];
         this.referenceCaptureIndex = referenceCaptureIndex;

         for (int k = 0; k < captureCount; k++)
            clusterToWorld[k] = new RigidBodyTransform();
      }

      /** {@code ^W T_c^(k)}. Meaningful only where {@link #isUsable(int)}. */
      public RigidBodyTransform getClusterToWorld(int k)
      {
         return clusterToWorld[k];
      }

      public double getSigma3(int k)
      {
         return sigma3[k];
      }

      public int getCoVisibleCount(int k)
      {
         return coVisibleCount[k];
      }

      /**
       * Whether the gauge cluster could be tracked into this capture at all. A capture with fewer
       * than three markers co-visible with the reference has no cluster pose, and every marker in
       * it is unusable for F4 and F5 -- not because the markers are bad, but because there is no
       * way to say where the robot was.
       */
      public boolean isUsable(int k)
      {
         return usable[k];
      }

      /** The capture whose raw gauge markers define the cluster frame. */
      public int getReferenceCaptureIndex()
      {
         return referenceCaptureIndex;
      }

      public int getUsableCaptureCount()
      {
         int count = 0;

         for (boolean flag : usable)
         {
            if (flag)
               count++;
         }

         return count;
      }

      /** The worst {@code σ₃} over usable captures: the conditioning of the weakest gauge frame. */
      public double getWorstSigma3()
      {
         double worst = Double.POSITIVE_INFINITY;

         for (int k = 0; k < usable.length; k++)
         {
            if (usable[k])
               worst = Math.min(worst, sigma3[k]);
         }

         return worst;
      }
   }

   /** {@code Δ = I}. FRAMEWORK.md §4, status <b>chosen</b>. */
   public static void packInitialClusterToBase(RigidBodyTransform toPack)
   {
      toPack.setToZero();
   }

   /**
    * Tracks the gauge cluster across every capture, defining the cluster frame from the capture in
    * which the most gauge markers are visible.
    * <p>
    * §4 says "capture 0". Choosing the best-observed capture instead is a strict generalisation: it
    * reduces to capture 0 whenever the gauge is fully visible there, which is the normal case, and
    * it avoids throwing away the whole session because one marker happened to be occluded in the
    * first frame. Ties go to the lowest index, so the choice is deterministic and the resulting
    * {@code Δ} convention is reproducible.
    * </p>
    */
   public static GaugeTracking trackGaugeCluster(CaptureSet captureSet)
   {
      MarkerCluster gauge = captureSet.getGaugeCluster();
      int captureCount = captureSet.getCaptureCount();
      int referenceIndex = findBestObservedCapture(captureSet, gauge);

      MocapFrame referenceFrame = captureSet.getCapture(referenceIndex).getMocapFrame();

      if (referenceFrame.getVisibleCount(gauge) < RigidBodyRegistration.MINIMUM_CORRESPONDENCES)
         throw new IllegalArgumentException("No capture has " + RigidBodyRegistration.MINIMUM_CORRESPONDENCES + " visible markers on the gauge cluster '"
               + gauge.getLinkName() + "'. The base pose is unrecoverable for every capture, so there is nothing to calibrate. "
               + "Fix the mounting or the camera coverage before running this (FRAMEWORK.md §20.4).");

      Point3D[] referenceShape = centredReferenceShape(gauge, referenceFrame);

      GaugeTracking tracking = new GaugeTracking(captureCount, referenceIndex);
      RigidBodyRegistration registration = new RigidBodyRegistration(Math.max(gauge.getMarkerCount(), RigidBodyRegistration.MINIMUM_CORRESPONDENCES));
      RegistrationResult result = new RegistrationResult();

      for (int k = 0; k < captureCount; k++)
      {
         MocapFrame frame = captureSet.getCapture(k).getMocapFrame();
         registration.clear();

         for (int j = 0; j < gauge.getMarkerCount(); j++)
         {
            MarkerId marker = gauge.getMarker(j);
            MarkerObservation current = frame.get(marker);

            // Co-visibility, not visibility. A marker seen now but not in the reference capture has
            // no position in the cluster frame, so it cannot contribute a correspondence.
            if (referenceShape[j] != null && current.isVisible())
               registration.addCorrespondence(referenceShape[j], current.getPosition());
         }

         tracking.coVisibleCount[k] = registration.getCorrespondenceCount();

         if (registration.compute(result))
         {
            tracking.clusterToWorld[k].set(result.getTransform());
            tracking.sigma3[k] = result.getSigma3();
            tracking.usable[k] = true;
         }
         else
         {
            tracking.clusterToWorld[k].setToNaN();
            tracking.sigma3[k] = Double.NaN;
            tracking.usable[k] = false;
         }
      }

      return tracking;
   }

   private static int findBestObservedCapture(CaptureSet captureSet, MarkerCluster gauge)
   {
      int bestIndex = 0;
      int bestVisible = -1;

      for (int k = 0; k < captureSet.getCaptureCount(); k++)
      {
         int visible = captureSet.getCapture(k).getMocapFrame().getVisibleCount(gauge);

         if (visible > bestVisible)
         {
            bestVisible = visible;
            bestIndex = k;
         }
      }

      return bestIndex;
   }

   /**
    * The cluster's shape: the reference capture's gauge-marker positions relative to their
    * centroid, or {@code null} for a marker that was not visible there.
    * <p>
    * The centroid is taken over exactly the markers visible in the reference capture, so the shape
    * is a fixed constellation. Recomputing it per capture over whatever happened to be co-visible
    * would move the cluster origin whenever a marker occluded, and {@code ^W T_c^(k)} would pick up
    * a translation that has nothing to do with the robot moving.
    * </p>
    */
   public static Point3D[] centredReferenceShape(MarkerCluster gauge, MocapFrame referenceFrame)
   {
      Point3D centroid = new Point3D();
      int visibleCount = 0;

      for (int j = 0; j < gauge.getMarkerCount(); j++)
      {
         MarkerObservation observation = referenceFrame.get(gauge.getMarker(j));

         if (observation.isVisible())
         {
            centroid.add(observation.getPosition());
            visibleCount++;
         }
      }

      centroid.scale(1.0 / visibleCount);

      Point3D[] shape = new Point3D[gauge.getMarkerCount()];

      for (int j = 0; j < gauge.getMarkerCount(); j++)
      {
         MarkerObservation observation = referenceFrame.get(gauge.getMarker(j));

         if (!observation.isVisible())
            continue;

         shape[j] = new Point3D(observation.getPosition());
         shape[j].sub(centroid);
      }

      return shape;
   }

   /** @see #centredReferenceShape(MarkerCluster, MocapFrame) */
   public static Point3D[] referenceGaugeShape(CaptureSet captureSet, GaugeTracking tracking)
   {
      Capture reference = captureSet.getCapture(tracking.getReferenceCaptureIndex());
      return centredReferenceShape(captureSet.getGaugeCluster(), reference.getMocapFrame());
   }
}
