package us.ihmc.alexMocap.mocap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.alexMocap.core.MarkerId;

/**
 * Maps Motive's streaming marker ids onto this project's {@link MarkerId}s.
 * <p>
 * Motive identifies markers by an integer that means nothing outside Motive, and it is not
 * necessarily small or contiguous -- for labelled markers it is typically a composite of the rigid
 * body id and the marker's index within it. This class turns that into the dense index everything
 * downstream addresses observations by.
 * </p>
 * <p>
 * Lookup is a binary search over a sorted {@code int[]}, so it handles sparse and large ids without
 * a hash map and without allocating. It runs once per marker per frame at 200 Hz.
 * </p>
 *
 * <h2>What this cannot check</h2>
 * <p>
 * FRAMEWORK.md §21.5: G1 catches a label swap <i>within</i> a cluster, because two markers trading
 * places changes the inter-marker distances. It does not catch a marker assigned to the wrong
 * <i>link</i> -- that assignment is taken on trust from the Motive rigid-body definitions, and
 * nothing in this pipeline verifies it. A thigh marker labelled as a shank marker produces a clean
 * calibration of the wrong thing.
 * </p>
 * <p>
 * So this class is a lookup table, not a validator. The one thing it does enforce is that the
 * mapping is a bijection: no Motive id maps to two markers, and no marker is fed by two Motive ids.
 * </p>
 */
public class MarkerLabeling
{
   private final List<MarkerId> markers;
   private final int[] sortedMotiveIds;
   private final int[] markerIndexByPosition;

   private MarkerLabeling(List<MarkerId> markers, int[] sortedMotiveIds, int[] markerIndexByPosition)
   {
      this.markers = markers;
      this.sortedMotiveIds = sortedMotiveIds;
      this.markerIndexByPosition = markerIndexByPosition;
   }

   /**
    * Builds a labelling from Motive id to marker name.
    *
    * @param motiveIdToName the mapping, in the order the marker set should be built. Iteration
    *                       order determines the dense indices, so a {@code LinkedHashMap} gives a
    *                       stable, readable assignment.
    */
   public static MarkerLabeling fromNames(Map<Integer, String> motiveIdToName)
   {
      List<String> names = new ArrayList<>(motiveIdToName.size());
      List<Integer> motiveIds = new ArrayList<>(motiveIdToName.size());

      for (Map.Entry<Integer, String> entry : motiveIdToName.entrySet())
      {
         names.add(entry.getValue());
         motiveIds.add(entry.getKey());
      }

      return new Builder(MarkerId.createDenseSet(names)).addAll(motiveIds, names).build();
   }

   /** Builds a labelling against a marker set you already hold. */
   public static Builder against(List<MarkerId> markerSet)
   {
      return new Builder(markerSet);
   }

   public List<MarkerId> getMarkers()
   {
      return markers;
   }

   public int getLabelledCount()
   {
      return sortedMotiveIds.length;
   }

   /**
    * The dense marker index for a Motive streaming id, or {@code -1} if that id is not labelled.
    * <p>
    * Unlabelled ids are normal -- Motive streams unlabelled point-cloud markers alongside the
    * labelled ones -- so this returns a sentinel rather than throwing. Allocation-free.
    * </p>
    */
   public int indexOf(int motiveId)
   {
      int position = Arrays.binarySearch(sortedMotiveIds, motiveId);
      return position < 0 ? -1 : markerIndexByPosition[position];
   }

   /** @return the marker for a Motive id, or {@code null} if unlabelled. */
   public MarkerId lookup(int motiveId)
   {
      int index = indexOf(motiveId);
      return index < 0 ? null : markers.get(index);
   }

   public boolean isLabelled(int motiveId)
   {
      return indexOf(motiveId) >= 0;
   }

   /** @return the Motive id feeding a marker, or {@code -1} if nothing does. */
   public int motiveIdOf(MarkerId marker)
   {
      for (int i = 0; i < sortedMotiveIds.length; i++)
      {
         if (markerIndexByPosition[i] == marker.getIndex())
            return sortedMotiveIds[i];
      }

      return -1;
   }

   /**
    * Markers in the set that no Motive id feeds. These can never become visible, so a non-empty
    * result means the labelling and the cluster configuration disagree -- worth checking at
    * startup rather than discovering as a cluster that never reaches three visible markers.
    */
   public List<MarkerId> getUnfedMarkers()
   {
      boolean[] fed = new boolean[markers.size()];

      for (int markerIndex : markerIndexByPosition)
         fed[markerIndex] = true;

      List<MarkerId> unfed = new ArrayList<>();

      for (int i = 0; i < markers.size(); i++)
      {
         if (!fed[i])
            unfed.add(markers.get(i));
      }

      return unfed;
   }

   @Override
   public String toString()
   {
      return "MarkerLabeling[" + sortedMotiveIds.length + " labelled of " + markers.size() + " markers]";
   }

   public static final class Builder
   {
      private final List<MarkerId> markerSet;
      private final Map<Integer, MarkerId> byMotiveId = new LinkedHashMap<>();

      private Builder(List<MarkerId> markerSet)
      {
         MarkerId.checkDenseSet(markerSet);
         this.markerSet = markerSet;
      }

      public Builder add(int motiveId, String markerName)
      {
         return add(motiveId, find(markerName));
      }

      /**
       * Every check runs before anything is written. A rejected {@code add} must leave the builder
       * exactly as it was -- validating after a {@code put} would have the failed entry replace a
       * good one on its way out, so the exception would be followed by silent corruption.
       */
      public Builder add(int motiveId, MarkerId marker)
      {
         if (!markerSet.contains(marker))
            throw new IllegalArgumentException("Marker " + marker + " is not in the marker set being labelled.");

         MarkerId existing = byMotiveId.get(motiveId);

         if (existing != null)
            throw new IllegalArgumentException("Motive id " + motiveId + " is already mapped to " + existing + "; cannot also map it to " + marker + ".");

         for (Map.Entry<Integer, MarkerId> entry : byMotiveId.entrySet())
         {
            if (entry.getValue().equals(marker))
               throw new IllegalArgumentException(
                     "Marker " + marker + " is already fed by Motive id " + entry.getKey() + "; it cannot also be fed by " + motiveId + ".");
         }

         byMotiveId.put(motiveId, marker);
         return this;
      }

      private Builder addAll(List<Integer> motiveIds, List<String> names)
      {
         for (int i = 0; i < motiveIds.size(); i++)
            add(motiveIds.get(i), names.get(i));

         return this;
      }

      private MarkerId find(String markerName)
      {
         for (MarkerId marker : markerSet)
         {
            if (marker.getName().equals(markerName))
               return marker;
         }

         throw new IllegalArgumentException("No marker named '" + markerName + "' in the marker set.");
      }

      public MarkerLabeling build()
      {
         int[] sortedMotiveIds = byMotiveId.keySet().stream().mapToInt(Integer::intValue).sorted().toArray();
         int[] markerIndexByPosition = new int[sortedMotiveIds.length];

         for (int i = 0; i < sortedMotiveIds.length; i++)
            markerIndexByPosition[i] = byMotiveId.get(sortedMotiveIds[i]).getIndex();

         return new MarkerLabeling(markerSet, sortedMotiveIds, markerIndexByPosition);
      }
   }
}
