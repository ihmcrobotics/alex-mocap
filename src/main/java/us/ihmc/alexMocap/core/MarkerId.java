package us.ihmc.alexMocap.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Identity of one motion-capture marker: a human-readable name and a dense index.
 * <p>
 * The name is what appears in CSV headers, calibration files, and reports. The index is what the
 * 200 Hz runtime loop uses -- {@link MocapFrame} stores observations in a flat array addressed by
 * it, so looking up a marker is an array read with no hashing and no allocation.
 * </p>
 * <p>
 * Indices are only meaningful relative to a <b>marker set</b>: one list covering every marker in
 * the session, with indices {@code 0..N-1}. Build it once with {@link #createDenseSet} and pass
 * that same list everywhere. Two marker sets built independently will happily assign the same
 * index to different markers, which is why {@link MocapFrame} verifies identity on lookup rather
 * than trusting the index.
 * </p>
 * <p>
 * Immutable. Safe to share, safe as a map key.
 * </p>
 */
public final class MarkerId
{
   private final String name;
   private final int index;

   /**
    * Prefer {@link #createDenseSet}, which assigns indices for you and cannot produce a sparse or
    * duplicated set. Use this directly only when reconstructing a marker whose index is already
    * fixed by an existing set.
    */
   public MarkerId(String name, int index)
   {
      if (name == null || name.isBlank())
         throw new IllegalArgumentException("Marker name must be non-blank.");
      if (index < 0)
         throw new IllegalArgumentException("Marker index must be non-negative, was " + index + " for '" + name + "'.");

      this.name = name;
      this.index = index;
   }

   /**
    * Builds a marker set: one {@code MarkerId} per name, indexed by position.
    *
    * @param names every marker in the session, in a stable order. Duplicates are rejected -- a
    *              repeated name is a configuration typo that would otherwise alias two markers
    *              onto one array slot and silently discard one of them.
    * @return an unmodifiable list whose element {@code i} has {@code getIndex() == i}.
    */
   public static List<MarkerId> createDenseSet(String... names)
   {
      return createDenseSet(List.of(names));
   }

   /** @see #createDenseSet(String...) */
   public static List<MarkerId> createDenseSet(List<String> names)
   {
      Set<String> seen = new HashSet<>();
      List<MarkerId> markers = new ArrayList<>(names.size());

      for (int i = 0; i < names.size(); i++)
      {
         String name = names.get(i);

         if (!seen.add(name))
            throw new IllegalArgumentException("Duplicate marker name '" + name + "' at index " + i + ".");

         markers.add(new MarkerId(name, i));
      }

      return Collections.unmodifiableList(markers);
   }

   /**
    * Verifies that a list really is a dense marker set. Called by every type that addresses
    * observations by index, so a hand-assembled or deserialised set cannot quietly be sparse.
    */
   public static void checkDenseSet(List<MarkerId> markers)
   {
      if (markers == null || markers.isEmpty())
         throw new IllegalArgumentException("Marker set must be non-empty.");

      for (int i = 0; i < markers.size(); i++)
      {
         MarkerId marker = markers.get(i);

         if (marker.getIndex() != i)
            throw new IllegalArgumentException(
                  "Marker set is not dense: '" + marker.getName() + "' sits at position " + i + " but reports index " + marker.getIndex() + ".");
      }
   }

   public String getName()
   {
      return name;
   }

   public int getIndex()
   {
      return index;
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      if (!(object instanceof MarkerId other))
         return false;

      return index == other.index && name.equals(other.name);
   }

   @Override
   public int hashCode()
   {
      return 31 * index + name.hashCode();
   }

   @Override
   public String toString()
   {
      return name + "[" + index + "]";
   }
}
