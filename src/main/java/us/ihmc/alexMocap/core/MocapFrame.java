package us.ihmc.alexMocap.core;

import java.util.List;

/**
 * One mocap frame: a timestamp and one {@link MarkerObservation} per marker in the session's
 * marker set.
 * <p>
 * The observation array is preallocated and dense, addressed by {@link MarkerId#getIndex()}, so
 * {@link #get(MarkerId)} is an array read. Nothing here allocates after construction, which is
 * what lets a 200 Hz replay or live loop reuse one frame indefinitely.
 * </p>
 *
 * <h2>Timestamps</h2>
 * <p>
 * Nanoseconds, in whatever epoch the source defines. This type does not care which epoch -- it
 * cares that the mocap and encoder clocks agree on one. FRAMEWORK.md §18.3 lists timestamp
 * mismatch in F10 as a silent failure: it reads as an estimator regression rather than as a
 * bookkeeping error. {@link Capture#getTimestampSkewNanoseconds()} is the instrumentation for it.
 * </p>
 * <p>
 * Nanoseconds as a {@code long} rather than seconds as a {@code double} is deliberate. At a
 * wall-clock epoch, a double has about 200 ns of resolution, and 200 ns of jitter at 1 m/s is
 * 0.2 µm -- harmless. But the same double loses exactness under subtraction, and the quantity that
 * matters here is always a difference.
 * </p>
 */
public class MocapFrame
{
   /** Sentinel for "no timestamp set", distinguishable from a real one. */
   public static final long NO_TIMESTAMP = Long.MIN_VALUE;

   private final List<MarkerId> markers;
   private final MarkerObservation[] observations;
   private long timestampNanoseconds = NO_TIMESTAMP;

   /**
    * @param markers the session marker set, dense and stable. Held by reference, not copied: every
    *                frame in a session should share one list.
    */
   public MocapFrame(List<MarkerId> markers)
   {
      MarkerId.checkDenseSet(markers);

      this.markers = markers;
      this.observations = new MarkerObservation[markers.size()];

      for (int i = 0; i < markers.size(); i++)
         observations[i] = new MarkerObservation(markers.get(i));
   }

   public List<MarkerId> getMarkers()
   {
      return markers;
   }

   public int getMarkerCount()
   {
      return observations.length;
   }

   /**
    * The observation for a marker.
    * <p>
    * The identity of the marker at that index is verified rather than assumed. Indices are only
    * meaningful within one marker set, and a caller holding an id from a different set would
    * otherwise silently read someone else's marker -- which is exactly the class of error that
    * produces a plausible, wrong pose.
    * </p>
    *
    * @throws IllegalArgumentException if this marker does not belong to this frame's marker set.
    */
   public MarkerObservation get(MarkerId marker)
   {
      int index = marker.getIndex();

      if (index >= observations.length || !observations[index].getId().equals(marker))
         throw new IllegalArgumentException("Marker " + marker + " does not belong to this frame's marker set of " + observations.length + " markers.");

      return observations[index];
   }

   /** The observation at a raw index, for loops that already walk the whole set. */
   public MarkerObservation get(int index)
   {
      return observations[index];
   }

   public long getTimestampNanoseconds()
   {
      return timestampNanoseconds;
   }

   public void setTimestampNanoseconds(long timestampNanoseconds)
   {
      this.timestampNanoseconds = timestampNanoseconds;
   }

   /** Number of markers the system saw this frame. Log it per cluster (FRAMEWORK.md §9). */
   public int getVisibleCount()
   {
      int count = 0;

      for (MarkerObservation observation : observations)
      {
         if (observation.isVisible())
            count++;
      }

      return count;
   }

   /** Visible markers belonging to one cluster. This is the count F6 refuses on. */
   public int getVisibleCount(MarkerCluster cluster)
   {
      int count = 0;

      for (int i = 0; i < cluster.getMarkerCount(); i++)
      {
         if (get(cluster.getMarker(i)).isVisible())
            count++;
      }

      return count;
   }

   /** Marks every marker unseen and clears the timestamp. Allocation-free. */
   public void clear()
   {
      timestampNanoseconds = NO_TIMESTAMP;

      for (MarkerObservation observation : observations)
         observation.setNotVisible();
   }

   /**
    * Copies another frame's contents into this one. Both frames must share a marker set --
    * copying across sets would reassign observations by index, which is meaningless.
    */
   public void set(MocapFrame other)
   {
      if (other.observations.length != observations.length)
         throw new IllegalArgumentException("Cannot copy a frame of " + other.observations.length + " markers into one of " + observations.length + ".");

      timestampNanoseconds = other.timestampNanoseconds;

      for (int i = 0; i < observations.length; i++)
      {
         if (!observations[i].getId().equals(other.observations[i].getId()))
            throw new IllegalArgumentException(
                  "Marker sets differ at index " + i + ": " + observations[i].getId() + " vs " + other.observations[i].getId() + ".");

         observations[i].set(other.observations[i]);
      }
   }

   @Override
   public String toString()
   {
      return "MocapFrame[t=" + timestampNanoseconds + " ns, " + getVisibleCount() + "/" + observations.length + " visible]";
   }
}
