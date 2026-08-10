package us.ihmc.alexMocap.core;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * The calibrated marker positions for one cluster, expressed in its link frame:
 * {@code ^i p̂_ij} for every marker {@code j} on link {@code i}.
 * <p>
 * This is F4's output (FRAMEWORK.md §6) and F6's input (§9). It is the thing the whole calibration
 * exists to produce -- the unknown, constant quantity in the observation model.
 * </p>
 * <p>
 * Mutable, because A′ overwrites it once per iteration. Read-only accessors are returned so that a
 * position cannot be written without also updating its observation count.
 * </p>
 *
 * <h2>{@code K_ij}</h2>
 * <p>
 * Each marker carries the number of captures it was visible in. F4 averages over exactly those
 * captures, and a marker seen in 3 of 30 captures has a layout roughly three times noisier than
 * one seen in all 30 -- with nothing in the position itself to say so. Partial visibility is
 * normal, so this bookkeeping is load-bearing rather than diagnostic.
 * </p>
 */
public class ClusterLayout
{
   private final String linkName;
   private final List<MarkerId> markers;
   private final Point3D[] positionsInLinkFrame;
   private final int[] observationCounts;

   /**
    * @param linkName the URDF link this cluster is mounted on.
    * @param markers  the cluster's members. Positions start NaN and counts start zero: an
    *                 un-solved layout must not read as a solved one at the origin.
    */
   public ClusterLayout(String linkName, List<MarkerId> markers)
   {
      if (linkName == null || linkName.isBlank())
         throw new IllegalArgumentException("Link name must be non-blank.");
      if (markers == null || markers.isEmpty())
         throw new IllegalArgumentException("Layout for '" + linkName + "' has no markers.");

      this.linkName = linkName;
      this.markers = List.copyOf(markers);
      this.positionsInLinkFrame = new Point3D[markers.size()];
      this.observationCounts = new int[markers.size()];

      for (int i = 0; i < markers.size(); i++)
         positionsInLinkFrame[i] = new Point3D(Double.NaN, Double.NaN, Double.NaN);
   }

   public ClusterLayout(MarkerCluster cluster)
   {
      this(cluster.getLinkName(), cluster.getMarkers());
   }

   public String getLinkName()
   {
      return linkName;
   }

   public List<MarkerId> getMarkers()
   {
      return markers;
   }

   public int getMarkerCount()
   {
      return markers.size();
   }

   public MarkerId getMarker(int localIndex)
   {
      return markers.get(localIndex);
   }

   /**
    * Position of a member in the link frame, addressed by position within <b>this layout</b>, not
    * by {@link MarkerId#getIndex()}. Layouts hold a handful of markers each; the session-wide
    * index would leave most slots empty.
    */
   public Point3DReadOnly getPositionInLinkFrame(int localIndex)
   {
      return positionsInLinkFrame[localIndex];
   }

   /** @throws IllegalArgumentException if the marker is not a member of this cluster. */
   public Point3DReadOnly getPositionInLinkFrame(MarkerId marker)
   {
      return positionsInLinkFrame[indexOf(marker)];
   }

   /** How many captures this marker was visible in: {@code K_ij}. */
   public int getObservationCount(int localIndex)
   {
      return observationCounts[localIndex];
   }

   public int getObservationCount(MarkerId marker)
   {
      return observationCounts[indexOf(marker)];
   }

   /**
    * @param observationCount {@code K_ij}, the number of captures the position was averaged over.
    *                         Zero means the marker was never seen, and the position must be NaN.
    */
   public void setPositionInLinkFrame(int localIndex, Point3DReadOnly position, int observationCount)
   {
      if (observationCount < 0)
         throw new IllegalArgumentException("Observation count must be non-negative, was " + observationCount + ".");

      positionsInLinkFrame[localIndex].set(position);
      observationCounts[localIndex] = observationCount;
   }

   public void setPositionInLinkFrame(MarkerId marker, Point3DReadOnly position, int observationCount)
   {
      setPositionInLinkFrame(indexOf(marker), position, observationCount);
   }

   /** Marks a member as never observed: NaN position, {@code K_ij = 0}. */
   public void setNotObserved(int localIndex)
   {
      positionsInLinkFrame[localIndex].setToNaN();
      observationCounts[localIndex] = 0;
   }

   /**
    * Position of a member within this layout.
    *
    * @throws IllegalArgumentException if the marker is not a member. Returning -1 would let a
    *                                  caller index with it and read the last marker instead.
    */
   public int indexOf(MarkerId marker)
   {
      for (int i = 0; i < markers.size(); i++)
      {
         if (markers.get(i).equals(marker))
            return i;
      }

      throw new IllegalArgumentException("Marker " + marker + " is not a member of the '" + linkName + "' cluster.");
   }

   /** Whether every member has a finite calibrated position. */
   public boolean isFullySolved()
   {
      for (Point3D position : positionsInLinkFrame)
      {
         if (position.containsNaN())
            return false;
      }

      return true;
   }

   /** The smallest {@code K_ij} over the members: the layout is only as good as its worst marker. */
   public int getMinimumObservationCount()
   {
      int minimum = Integer.MAX_VALUE;

      for (int count : observationCounts)
         minimum = Math.min(minimum, count);

      return minimum;
   }

   public void set(ClusterLayout other)
   {
      if (!markers.equals(other.markers))
         throw new IllegalArgumentException("Layouts cover different markers: " + markers + " vs " + other.markers + ".");

      for (int i = 0; i < positionsInLinkFrame.length; i++)
      {
         positionsInLinkFrame[i].set(other.positionsInLinkFrame[i]);
         observationCounts[i] = other.observationCounts[i];
      }
   }

   @Override
   public String toString()
   {
      return "ClusterLayout[" + linkName + ", " + markers.size() + " markers, min K_ij=" + getMinimumObservationCount() + "]";
   }
}
