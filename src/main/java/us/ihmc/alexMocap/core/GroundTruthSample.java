package us.ihmc.alexMocap.core;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * One frame of runtime output: whole-body CoM, pelvis pose, and the per-cluster conditioning that
 * says whether either should be believed.
 * <p>
 * Mutable and reusable. The runtime loop writes one of these per frame at 200 Hz; the logger reads
 * it and moves on.
 * </p>
 *
 * <h2>There is no velocity here, and that is the point</h2>
 * <p>
 * FRAMEWORK.md §13: raw single differencing at 200 Hz with {@code σ = 0.93 mm} gives about
 * {@code √2·σ/Δt ≈ 0.13 m/s}, which exceeds the ContactNet baselines this pipeline exists to
 * validate (0.0844 and 0.0254 m/s) by a wide margin. Velocity comes from a centred
 * Savitzky-Golay differentiator run <b>offline</b> over the logged pose trajectory, and a centred
 * window cannot execute causally.
 * </p>
 * <p>
 * So this type carries no twist, no velocity, and no field anyone could fill with one. If it had a
 * velocity slot, someone would populate it by differencing and compare 0.13 m/s of noise against a
 * 0.025 m/s estimator.
 * </p>
 *
 * <h2>Conditioning is part of the sample, not a side channel</h2>
 * <p>
 * FRAMEWORK.md §9 requires {@code σ₃} and the visible marker count logged every frame for every
 * cluster, and §18.1 requires the refusal logged when a pose is withheld. Those live here rather
 * than in a parallel log because a CoM value and the conditioning that produced it have to be
 * impossible to separate -- reading one without the other is how a rank-deficient frame becomes a
 * trusted number.
 * </p>
 */
public class GroundTruthSample
{
   private long timestampNanoseconds = MocapFrame.NO_TIMESTAMP;

   /** {@code ^Wg c}, the whole-body CoM in the gravity-aligned world frame (FRAMEWORK.md §12). */
   private final Point3D centerOfMass = new Point3D(Double.NaN, Double.NaN, Double.NaN);

   /** {@code ^Wg T̂_b}, the pelvis link pose in the gravity-aligned world frame (§13). */
   private final RigidBodyTransform pelvisPose = new RigidBodyTransform();

   private final List<String> linkNames;
   private final double[] sigma3;
   private final int[] visibleCount;
   private final boolean[] poseAccepted;

   /**
    * @param linkNames the marked links whose conditioning this sample reports, in a stable order.
    */
   public GroundTruthSample(List<String> linkNames)
   {
      if (linkNames == null || linkNames.isEmpty())
         throw new IllegalArgumentException("A ground truth sample must report on at least one marked link.");

      this.linkNames = List.copyOf(linkNames);
      this.sigma3 = new double[linkNames.size()];
      this.visibleCount = new int[linkNames.size()];
      this.poseAccepted = new boolean[linkNames.size()];
      clear();
   }

   public long getTimestampNanoseconds()
   {
      return timestampNanoseconds;
   }

   public void setTimestampNanoseconds(long timestampNanoseconds)
   {
      this.timestampNanoseconds = timestampNanoseconds;
   }

   public Point3DReadOnly getCenterOfMass()
   {
      return centerOfMass;
   }

   public void setCenterOfMass(Point3DReadOnly centerOfMass)
   {
      this.centerOfMass.set(centerOfMass);
   }

   /** Mutable: the runtime writes the pelvis pose straight into it. */
   public RigidBodyTransform getPelvisPose()
   {
      return pelvisPose;
   }

   public List<String> getLinkNames()
   {
      return linkNames;
   }

   public int getLinkCount()
   {
      return linkNames.size();
   }

   /** Smallest singular value of link {@code i}'s registration, in m². See FRAMEWORK.md §9. */
   public double getSigma3(int linkIndex)
   {
      return sigma3[linkIndex];
   }

   /** Markers of link {@code i}'s cluster that were visible this frame. */
   public int getVisibleCount(int linkIndex)
   {
      return visibleCount[linkIndex];
   }

   /**
    * Whether F6 returned a pose for link {@code i}, or refused it.
    * <p>
    * A refusal is a logged event, not an absence. Without it a dropped frame and a frame where the
    * cluster went rank-deficient look the same in the log.
    * </p>
    */
   public boolean isPoseAccepted(int linkIndex)
   {
      return poseAccepted[linkIndex];
   }

   public void setConditioning(int linkIndex, double sigma3, int visibleCount, boolean poseAccepted)
   {
      this.sigma3[linkIndex] = sigma3;
      this.visibleCount[linkIndex] = visibleCount;
      this.poseAccepted[linkIndex] = poseAccepted;
   }

   public int indexOfLink(String linkName)
   {
      int index = linkNames.indexOf(linkName);

      if (index < 0)
         throw new IllegalArgumentException("Link '" + linkName + "' is not reported by this sample. Known: " + linkNames + ".");

      return index;
   }

   /** Whether every marked link produced an accepted pose this frame. */
   public boolean allPosesAccepted()
   {
      for (boolean accepted : poseAccepted)
      {
         if (!accepted)
            return false;
      }

      return true;
   }

   /** Resets to the unset state: NaN CoM and pose, NaN {@code σ₃}, zero visible, all refused. */
   public void clear()
   {
      timestampNanoseconds = MocapFrame.NO_TIMESTAMP;
      centerOfMass.setToNaN();
      pelvisPose.setToNaN();

      for (int i = 0; i < sigma3.length; i++)
      {
         sigma3[i] = Double.NaN;
         visibleCount[i] = 0;
         poseAccepted[i] = false;
      }
   }

   public void set(GroundTruthSample other)
   {
      if (!linkNames.equals(other.linkNames))
         throw new IllegalArgumentException("Samples report on different links: " + linkNames + " vs " + other.linkNames + ".");

      timestampNanoseconds = other.timestampNanoseconds;
      centerOfMass.set(other.centerOfMass);
      pelvisPose.set(other.pelvisPose);

      for (int i = 0; i < sigma3.length; i++)
      {
         sigma3[i] = other.sigma3[i];
         visibleCount[i] = other.visibleCount[i];
         poseAccepted[i] = other.poseAccepted[i];
      }
   }

   @Override
   public String toString()
   {
      return "GroundTruthSample[t=" + timestampNanoseconds + " ns, com=" + centerOfMass + ", " + (allPosesAccepted() ? "all links accepted" : "REFUSALS")
            + "]";
   }
}
