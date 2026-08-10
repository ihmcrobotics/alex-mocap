package us.ihmc.alexMocap.scs2;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The marker cloud itself, as YoVariables and as spheres in the 3-D view.
 *
 * <h2>Why the markers are drawn at all</h2>
 * <p>
 * The CoM sphere and the pelvis triad are the deliverable; the markers are what makes a wrong one
 * diagnosable. A CoM that has jumped is ambiguous between a refused cluster, a swapped label, and a
 * calibration fault, and those look completely different in the cloud: a refusal is four markers
 * going dark together, a swap is one marker sitting on the wrong limb, and a calibration fault
 * leaves the cloud looking perfect while the CoM drifts. Without the cloud on screen all three
 * present identically, as a number that moved.
 * </p>
 *
 * <h2>A marker that is not seen is NaN, not the origin</h2>
 * <p>
 * An occluded marker is set to NaN, which SCS2 declines to render, so it simply disappears. The
 * alternative -- leaving the last position, or zeroing it -- is the project convention's whole point
 * (CLAUDE.md: "Unset is NaN, never zero"): a marker frozen at its last good position is
 * indistinguishable from a marker that is being seen, and one at the origin draws a sphere on the
 * floor that reads as a real detection. NaN is the only value that renders as "no data".
 * </p>
 * <p>
 * Because a dropout is invisible by construction, {@link #getVisibleMarkerCount()} is published
 * alongside: a count is what turns "I think something vanished" into a plot.
 * </p>
 *
 * <h2>Colour is per cluster</h2>
 * <p>
 * Markers are coloured by the link they belong to, cycling through {@link #CLUSTER_COLORS}. That is
 * what makes a mislabelled marker visible at all -- a marker that has been assigned to the wrong
 * cluster draws in the wrong colour, sitting among a group of another colour, which is obvious at a
 * glance and invisible in any scalar.
 * </p>
 */
public class MocapMarkerYoVariables
{
   /**
    * Radius of a marker sphere, metres.
    * <p>
    * A real passive marker is about 6 mm in radius. This is drawn at 20 mm deliberately: at true
    * scale, against a 1.8 m robot, a marker is a couple of pixels until you are almost inside the
    * model, and the point of drawing them is to see the set at a glance. Anything reading a
    * <i>distance</i> off this view is misreading it -- the numbers are in the YoVariables.
    * </p>
    */
   public static final double MARKER_RADIUS = 0.020;

   /**
    * Cycled over the clusters, in marked-link order.
    * <p>
    * Green first, and no blues. The robot is drawn grey and its reconstruction ghost is translucent
    * cyan, so a sky-blue or cyan marker disappears into the ghost exactly where the markers matter
    * most. Every entry here is chosen to survive being drawn over pale cyan, over dark grey, and
    * over SCS2's default sky.
    * </p>
    */
   public static final List<ColorDefinition> CLUSTER_COLORS = List.of(ColorDefinitions.LimeGreen(),
                                                                      ColorDefinitions.OrangeRed(),
                                                                      ColorDefinitions.Yellow(),
                                                                      ColorDefinitions.Magenta(),
                                                                      ColorDefinitions.Chartreuse(),
                                                                      ColorDefinitions.DarkOrange(),
                                                                      ColorDefinitions.White());

   private final YoRegistry registry;
   private final List<MarkerId> markers;
   private final List<MarkerCluster> clusters;
   private final YoFramePoint3D[] positions;
   private final YoInteger visibleMarkerCount;

   /**
    * @param namePrefix prefix for every variable, so two instances can coexist.
    * @param clusters   the marker set, in marked-link order. Determines colour grouping.
    * @param markers    every marker, as the dense set a {@link MocapFrame} is built from.
    * @param world      the frame the marker positions are expressed in. This must be the frame the
    *                   observations arrive in -- normally the motive world, <b>not</b> the
    *                   gravity-aligned one, since F8's correction is applied downstream of the raw
    *                   marker cloud.
    */
   public MocapMarkerYoVariables(String namePrefix, List<MarkerCluster> clusters, List<MarkerId> markers, ReferenceFrame world)
   {
      this.registry = new YoRegistry(namePrefix + "MocapMarkers");
      this.markers = List.copyOf(markers);
      this.clusters = List.copyOf(clusters);
      this.positions = new YoFramePoint3D[this.markers.size()];
      this.visibleMarkerCount = new YoInteger(namePrefix + "VisibleMarkerCount", registry);

      for (int i = 0; i < this.markers.size(); i++)
         positions[i] = new YoFramePoint3D(namePrefix + this.markers.get(i).getName(), world, registry);

      setAllToNaN();
   }

   /** Publishes one frame. Markers that were not seen are set to NaN and stop rendering. */
   public void update(MocapFrame frame)
   {
      if (frame.getMarkerCount() != markers.size())
         throw new IllegalArgumentException("Frame holds " + frame.getMarkerCount() + " markers but this view was built for " + markers.size() + ".");

      int visible = 0;

      for (int i = 0; i < markers.size(); i++)
      {
         MarkerObservation observation = frame.get(i);

         if (observation.isVisible())
         {
            positions[i].set(observation.getPosition());
            visible++;
         }
         else
         {
            positions[i].setToNaN();
         }
      }

      visibleMarkerCount.set(visible);
   }

   /** Hides every marker. For a session that has not yet received a frame. */
   public void setAllToNaN()
   {
      for (YoFramePoint3D position : positions)
         position.setToNaN();

      visibleMarkerCount.set(0);
   }

   /**
    * The spheres, grouped by cluster so the tree in the visualizer matches the marked links.
    *
    * @return a group to hand to {@code SimulationSession.addYoGraphicDefinition}.
    */
   public YoGraphicGroupDefinition createYoGraphics(String name)
   {
      List<YoGraphicDefinition> clusterGroups = new ArrayList<>();
      int markerIndex = 0;

      for (int c = 0; c < clusters.size(); c++)
      {
         MarkerCluster cluster = clusters.get(c);
         ColorDefinition color = CLUSTER_COLORS.get(c % CLUSTER_COLORS.size());
         List<YoGraphicDefinition> spheres = new ArrayList<>();

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            spheres.add(YoGraphicDefinitionFactory.newYoGraphicPoint3D(markers.get(markerIndex).getName(), positions[markerIndex], MARKER_RADIUS, color));
            markerIndex++;
         }

         YoGraphicGroupDefinition clusterGroup = new YoGraphicGroupDefinition(cluster.getLinkName());
         clusterGroup.setChildren(spheres);
         clusterGroups.add(clusterGroup);
      }

      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.setChildren(clusterGroups);

      return group;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   /**
    * Markers seen in the last frame.
    * <p>
    * Worth plotting rather than glancing at: an occlusion is invisible in the 3-D view by design, so
    * this is the only place a dropout leaves a trace you can go back and find.
    * </p>
    */
   public YoInteger getVisibleMarkerCount()
   {
      return visibleMarkerCount;
   }

   public List<MarkerId> getMarkers()
   {
      return markers;
   }
}
