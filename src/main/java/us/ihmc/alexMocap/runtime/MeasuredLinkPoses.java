package us.ihmc.alexMocap.runtime;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * One frame's worth of link poses {@code ^W T̂_i}, plus where each came from and the conditioning
 * behind it.
 * <p>
 * This is the working state of the runtime loop: {@link LinkPoseEstimator} fills the marked links
 * from marker clusters (F6), {@link KinematicChainCoupler} fills the rest by FK chaining (F7), and
 * {@link CenterOfMassGroundTruth} consumes the lot (F9).
 * </p>
 *
 * <h2>Provenance travels with the pose</h2>
 * <p>
 * A measured pose and a chained one are not interchangeable, and nothing about the transform says
 * which it is. FRAMEWORK.md §10 calls chaining "a deliberate trade" -- it reintroduces encoder
 * dependence for that link, which is exactly the dependence F6 exists to remove -- so a consumer
 * that cares must be able to tell. {@link Source} is how.
 * </p>
 * <p>
 * {@link Source#NONE} is the third state and it is not the same as either. A link whose cluster
 * went rank-deficient has no pose at all, and treating that as "zero" or as "the last one" is the
 * silent failure of FRAMEWORK.md §18.1. Its transform is NaN and its {@link #getRefusalReason} says
 * why in words a human can act on.
 * </p>
 *
 * <h2>Contract</h2>
 * <p>
 * Mutable and reusable; the 200 Hz loop overwrites one instance rather than allocating. Not thread
 * safe.
 * </p>
 */
public class MeasuredLinkPoses
{
   /** Where a link's pose came from. */
   public enum Source
   {
      /**
       * No pose. Either the link was never filled this frame, or F6 refused it. Check
       * {@link #getRefusalReason}.
       */
      NONE,
      /**
       * F6: registered from this link's own marker cluster. No encoders involved -- this is the
       * whole point of the method (FRAMEWORK.md §9).
       */
      MEASURED,
      /**
       * F7: chained from a marked ancestor through forward kinematics. Encoder-dependent for this
       * link (FRAMEWORK.md §10).
       */
      CHAINED
   }

   private final List<String> linkNames;
   private final RigidBodyTransform[] poses;
   private final double[] sigma3;
   private final int[] visibleCount;
   private final Source[] sources;
   private final String[] refusalReasons;

   /**
    * @param linkNames every link this frame reports on -- marked and unmarked -- in a stable order.
    */
   public MeasuredLinkPoses(List<String> linkNames)
   {
      if (linkNames == null || linkNames.isEmpty())
         throw new IllegalArgumentException("Must report on at least one link.");

      this.linkNames = List.copyOf(linkNames);
      this.poses = new RigidBodyTransform[linkNames.size()];
      this.sigma3 = new double[linkNames.size()];
      this.visibleCount = new int[linkNames.size()];
      this.sources = new Source[linkNames.size()];
      this.refusalReasons = new String[linkNames.size()];

      for (int i = 0; i < poses.length; i++)
         poses[i] = new RigidBodyTransform();

      clear();
   }

   public List<String> getLinkNames()
   {
      return linkNames;
   }

   public int getLinkCount()
   {
      return linkNames.size();
   }

   /** @throws IllegalArgumentException if this frame does not report on that link. */
   public int indexOf(String linkName)
   {
      int index = linkNames.indexOf(linkName);

      if (index < 0)
         throw new IllegalArgumentException("Link '" + linkName + "' is not reported here. Known: " + linkNames + ".");

      return index;
   }

   /**
    * {@code ^W T̂_i}. NaN when {@link #getSource} is {@link Source#NONE} -- deliberately, so a
    * caller that ignores the source propagates NaN visibly rather than using a plausible identity.
    */
   public RigidBodyTransform getPose(int linkIndex)
   {
      return poses[linkIndex];
   }

   public RigidBodyTransform getPose(String linkName)
   {
      return poses[indexOf(linkName)];
   }

   public Source getSource(int linkIndex)
   {
      return sources[linkIndex];
   }

   public Source getSource(String linkName)
   {
      return sources[indexOf(linkName)];
   }

   public boolean isAvailable(int linkIndex)
   {
      return sources[linkIndex] != Source.NONE;
   }

   /** {@code σ₃} of this link's registration, m². NaN unless the link was measured. */
   public double getSigma3(int linkIndex)
   {
      return sigma3[linkIndex];
   }

   /** Markers of this link's cluster seen this frame. Zero unless the link is marked. */
   public int getVisibleCount(int linkIndex)
   {
      return visibleCount[linkIndex];
   }

   /**
    * Why this link has no pose, or {@code null} if it has one.
    * <p>
    * FRAMEWORK.md §18.1's mitigation is "refuse rather than return a pose below threshold" <i>and</i>
    * log it. A refusal with no reason recorded turns into "the log has a gap here" three weeks
    * later, which is indistinguishable from a dropped frame.
    * </p>
    */
   public String getRefusalReason(int linkIndex)
   {
      return refusalReasons[linkIndex];
   }

   public String getRefusalReason(String linkName)
   {
      return refusalReasons[indexOf(linkName)];
   }

   /** Records an F6 pose. */
   public void setMeasured(int linkIndex, RigidBodyTransform pose, double sigma3, int visibleCount)
   {
      this.poses[linkIndex].set(pose);
      this.sigma3[linkIndex] = sigma3;
      this.visibleCount[linkIndex] = visibleCount;
      this.sources[linkIndex] = Source.MEASURED;
      this.refusalReasons[linkIndex] = null;
   }

   /** Records an F6 refusal: NaN pose, the conditioning that caused it, and the reason. */
   public void setRefused(int linkIndex, double sigma3, int visibleCount, String reason)
   {
      this.poses[linkIndex].setToNaN();
      this.sigma3[linkIndex] = sigma3;
      this.visibleCount[linkIndex] = visibleCount;
      this.sources[linkIndex] = Source.NONE;
      this.refusalReasons[linkIndex] = reason;
   }

   /** Records an F7 pose chained from a marked ancestor. */
   public void setChained(int linkIndex, RigidBodyTransform pose)
   {
      this.poses[linkIndex].set(pose);
      this.sigma3[linkIndex] = Double.NaN;
      this.visibleCount[linkIndex] = 0;
      this.sources[linkIndex] = Source.CHAINED;
      this.refusalReasons[linkIndex] = null;
   }

   /** How many links have a pose from any source. */
   public int getAvailableCount()
   {
      int count = 0;

      for (Source source : sources)
      {
         if (source != Source.NONE)
            count++;
      }

      return count;
   }

   /** Resets every link to NaN and {@link Source#NONE}. Allocation-free. */
   public void clear()
   {
      for (int i = 0; i < poses.length; i++)
      {
         poses[i].setToNaN();
         sigma3[i] = Double.NaN;
         visibleCount[i] = 0;
         sources[i] = Source.NONE;
         refusalReasons[i] = null;
      }
   }

   @Override
   public String toString()
   {
      return "MeasuredLinkPoses[" + getAvailableCount() + "/" + linkNames.size() + " links available]";
   }
}
