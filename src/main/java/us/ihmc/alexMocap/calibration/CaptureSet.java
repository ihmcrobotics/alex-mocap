package us.ihmc.alexMocap.calibration;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.Capture;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;

/**
 * The {@code K} calibration captures, plus the configuration they are interpreted against: the
 * marker set, the joint order, the clusters, and which cluster is the gauge.
 * <p>
 * This is what A′ consumes. Everything in FRAMEWORK.md §5-§8 is indexed by {@code k}, and this is
 * the collection {@code k} indexes.
 * </p>
 *
 * <h2>The gauge cluster</h2>
 * <p>
 * One cluster is distinguished: the pelvis, the one mounted on the base link. FRAMEWORK.md §1 is
 * emphatic that it must be the pelvis and not the torso -- under suspension the spine joint carries
 * the full load in tension with off-axis deflection the URDF does not model, and making torso the
 * gauge chains every link pose through that error.
 * </p>
 * <p>
 * The gauge is checked here to be mounted on the model's base link, because "gauge cluster" and
 * "cluster on the base link" being the same thing is an assumption F5 makes silently: {@code Δ} is
 * defined as {@code ^c T_b} for exactly this {@code c}.
 * </p>
 *
 * <h2>Immutable, and holds its captures by reference</h2>
 * <p>
 * The captures themselves are mutable ({@link Capture} is designed for reuse in a 200 Hz loop), but
 * the set is fixed once built. {@link #subset} is what G4 uses to withhold captures, and it shares
 * the underlying captures rather than copying them -- a held-out capture is the same capture, seen
 * by a fit that was not shown it.
 * </p>
 */
public class CaptureSet
{
   private final List<MarkerId> markers;
   private final List<String> jointNames;
   private final List<MarkerCluster> clusters;
   private final MarkerCluster gaugeCluster;
   private final List<Capture> captures;

   /**
    * @param markers      the session marker set, dense.
    * @param jointNames   the joint order, matching the URDF.
    * @param clusters     every marked cluster, including the gauge.
    * @param gaugeLinkName the link carrying the gauge cluster: the URDF base link.
    * @param captures     the {@code K} captures.
    */
   public CaptureSet(List<MarkerId> markers, List<String> jointNames, List<MarkerCluster> clusters, String gaugeLinkName, List<Capture> captures)
   {
      MarkerId.checkDenseSet(markers);

      if (clusters == null || clusters.isEmpty())
         throw new IllegalArgumentException("A capture set needs at least the gauge cluster.");
      if (captures == null || captures.isEmpty())
         throw new IllegalArgumentException("A capture set needs at least one capture.");

      this.markers = List.copyOf(markers);
      this.jointNames = List.copyOf(jointNames);
      this.clusters = List.copyOf(clusters);
      this.captures = List.copyOf(captures);

      MarkerCluster gauge = null;

      for (MarkerCluster cluster : this.clusters)
      {
         if (cluster.getLinkName().equals(gaugeLinkName))
            gauge = cluster;
      }

      if (gauge == null)
         throw new IllegalArgumentException("No cluster is mounted on the gauge link '" + gaugeLinkName + "'. Clusters cover: " + linkNames() + ".");

      this.gaugeCluster = gauge;

      for (int k = 0; k < this.captures.size(); k++)
      {
         Capture capture = this.captures.get(k);

         if (capture.getMocapFrame().getMarkerCount() != this.markers.size())
            throw new IllegalArgumentException("Capture " + k + " has " + capture.getMocapFrame().getMarkerCount() + " markers, the set declares "
                  + this.markers.size() + ".");

         capture.getEncoderSample().checkJointOrder(this.jointNames);
      }
   }

   /** {@code K}. */
   public int getCaptureCount()
   {
      return captures.size();
   }

   public Capture getCapture(int k)
   {
      return captures.get(k);
   }

   public List<Capture> getCaptures()
   {
      return captures;
   }

   public List<MarkerId> getMarkers()
   {
      return markers;
   }

   public List<String> getJointNames()
   {
      return jointNames;
   }

   public List<MarkerCluster> getClusters()
   {
      return clusters;
   }

   /** The pelvis cluster: the one {@code Δ = ^c T_b} is defined against. */
   public MarkerCluster getGaugeCluster()
   {
      return gaugeCluster;
   }

   public List<String> linkNames()
   {
      List<String> names = new ArrayList<>(clusters.size());

      for (MarkerCluster cluster : clusters)
         names.add(cluster.getLinkName());

      return names;
   }

   /**
    * A capture set over a chosen subset of these captures, sharing the same configuration and the
    * same underlying {@link Capture} objects.
    * <p>
    * This is G4's withholding step (FRAMEWORK.md §15). Whole captures are withheld rather than
    * individual observations, because withholding an observation from a capture whose other markers
    * are still in the fit leaks the base pose for that capture into the "held-out" prediction, and
    * the resulting RMS is not a held-out number at all.
    * </p>
    */
   public CaptureSet subset(int... captureIndices)
   {
      List<Capture> selected = new ArrayList<>(captureIndices.length);

      for (int index : captureIndices)
         selected.add(captures.get(index));

      return new CaptureSet(markers, jointNames, clusters, gaugeCluster.getLinkName(), selected);
   }

   @Override
   public String toString()
   {
      return "CaptureSet[K=" + captures.size() + ", " + clusters.size() + " clusters, gauge=" + gaugeCluster.getLinkName() + "]";
   }
}
