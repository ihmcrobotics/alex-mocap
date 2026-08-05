package us.ihmc.alexMocap.core;

import java.util.List;

/**
 * The set of markers rigidly mounted on one link: link name plus member {@link MarkerId}s.
 * <p>
 * A cluster is the unit that F6 registers, G1 checks for rigidity, and the conditioning monitor
 * reports {@code σ₃} and visible count for. It is configuration, not measurement -- immutable, and
 * fixed for a capture session.
 * </p>
 * <p>
 * FRAMEWORK.md §1 asks for at least four markers per cluster: three is the pose minimum, and the
 * fourth is what gives G1 a redundant inter-marker distance to check. It also asks for
 * non-collinear, asymmetric geometry with maximum practical spread. <b>None of that is enforced
 * here.</b> Geometry is a measured property and belongs to the gates; the only thing this
 * constructor rejects is fewer than three members, below which no pose exists at all and the
 * configuration is simply a typo.
 * </p>
 */
public final class MarkerCluster
{
   /** Below this, the cluster cannot produce a pose under any conditions. */
   public static final int MINIMUM_MARKERS = 3;

   /** FRAMEWORK.md §1: three for the pose, a fourth so G1 has something redundant to check. */
   public static final int RECOMMENDED_MINIMUM_MARKERS = 4;

   private final String linkName;
   private final List<MarkerId> markers;

   /**
    * @param linkName the URDF link this cluster is mounted on. Used to look up
    *                 {@code ^b T_i(q)} and the link's inertial block; it must match the URDF
    *                 exactly.
    * @param markers  the cluster's members, drawn from the session marker set.
    */
   public MarkerCluster(String linkName, List<MarkerId> markers)
   {
      if (linkName == null || linkName.isBlank())
         throw new IllegalArgumentException("Link name must be non-blank.");
      if (markers == null || markers.size() < MINIMUM_MARKERS)
         throw new IllegalArgumentException("Cluster '" + linkName + "' has " + (markers == null ? 0 : markers.size()) + " markers; " + MINIMUM_MARKERS
               + " is the algebraic minimum for a pose and FRAMEWORK.md §1 asks for " + RECOMMENDED_MINIMUM_MARKERS + ".");

      for (int i = 0; i < markers.size(); i++)
      {
         for (int j = i + 1; j < markers.size(); j++)
         {
            if (markers.get(i).equals(markers.get(j)))
               throw new IllegalArgumentException("Cluster '" + linkName + "' lists marker " + markers.get(i) + " twice.");
         }
      }

      this.linkName = linkName;
      this.markers = List.copyOf(markers);
   }

   public MarkerCluster(String linkName, MarkerId... markers)
   {
      this(linkName, List.of(markers));
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

   public MarkerId getMarker(int index)
   {
      return markers.get(index);
   }

   public boolean contains(MarkerId marker)
   {
      return markers.contains(marker);
   }

   /**
    * Whether this cluster carries the redundancy FRAMEWORK.md §1 asks for. Reported, not enforced
    * -- a three-marker cluster still produces poses, it just gives G1 nothing to check.
    */
   public boolean hasRecommendedRedundancy()
   {
      return markers.size() >= RECOMMENDED_MINIMUM_MARKERS;
   }

   @Override
   public String toString()
   {
      return "MarkerCluster[" + linkName + ", " + markers.size() + " markers]";
   }
}
