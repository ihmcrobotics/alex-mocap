package us.ihmc.alexMocap.sim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.registration.RegistrationResult;
import us.ihmc.alexMocap.registration.RigidBodyRegistration;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * A randomly drawn marker set: which links carry clusters, and where {@code ^i p_ij} sits on each.
 *
 * <h2>What this is for, and why it is on the production classpath</h2>
 * <p>
 * {@code calibration.RobotCaptures} (test scope) invents a whole <i>capture session</i> -- poses,
 * encoders, and observations -- so that the calibration can be exercised offline. This class does
 * something narrower and different: it draws a marker <b>layout</b>, once, and hands back both the
 * clusters and the truth that generated them. Paired with {@link SimulatedMocapCamera} it is a
 * <b>sensor simulator</b>, the direct analogue of SCS2's own joint and IMU corruptors in
 * {@code SCS2SensorReader}, and it belongs on the same classpath as the simulation it feeds.
 * </p>
 * <p>
 * The guard that keeps {@code RobotCaptures} out of the shipping jar is still worth having, and it
 * is unchanged: nothing here invents an encoder reading or a base pose, so nothing here can be fed
 * to the calibration as if it were a measurement. What it produces is a layout and a set of marker
 * identities. A caller who wants a calibrated layout still has to run F2-F5 on real captures.
 * </p>
 *
 * <h2>The draw is rejected on {@code σ₂}, using the runtime's own primitive</h2>
 * <p>
 * Four points drawn uniformly in a box can come out near-collinear, and a near-collinear cluster
 * does not determine a pose ({@code runtime.LinkPoseEstimator} refuses it). A generator that can
 * plant a cluster the runtime would refuse produces a demonstration that fails for a reason having
 * nothing to do with what is being demonstrated, so a draw whose {@code σ₂} falls below
 * {@link #COLLINEARITY_FRACTION}² of the cluster spread is thrown away and redrawn.
 * </p>
 * <p>
 * {@code σ₂} is obtained by registering the constellation <b>against itself</b> through
 * {@link RigidBodyRegistration}. That is not a trick for its own sake: FRAMEWORK.md §2 requires
 * exactly one implementation of the SVD in this project, and a self-registration's cross-covariance
 * {@code H = (1/L) Σ (p - p̄)(p - p̄)ᵀ} <i>is</i> the covariance of the centred points, so its
 * singular values are the covariance eigenvalues. The number this class rejects on is therefore the
 * same quantity, in the same units, computed by the same code as the number the runtime refuses on.
 * A second hand-rolled 3x3 eigen-solver here could disagree with the estimator, and the way that
 * failure would present is a cluster that passes generation and is refused every frame at run time.
 * </p>
 * <p>
 * {@code σ₂} is in <b>m²</b>, not m -- a mean-squared spread. See CLAUDE.md.
 * </p>
 *
 * <h2>Why the gauge cluster is drawn wider than the limb clusters</h2>
 * <p>
 * Every capture's base pose comes from registering the gauge (pelvis) cluster, so its angular error
 * {@code σ/(√N·r_perp)} multiplies the lever arm out to every other link. Measured on this project:
 * widening the pelvis bracket from 0.06 m to 0.20 m took held-out RMS from 3.00 mm to 0.91 mm, which
 * is a far bigger lever than either lowering {@code σ} or taking more captures. {@link
 * #DEFAULT_GAUGE_SPREAD} is FRAMEWORK.md §1's recommended 140 mm bracket; note that at §17's 0.3 mm
 * noise that is <b>not</b> enough to clear the 2.2 mm TALOS bar (measured: 2.86 mm held-out), and
 * ≥182 mm is what would be.
 * </p>
 *
 * <h2>This does not touch the model's configuration</h2>
 * <p>
 * {@code ^i c_i} is a constant of a link, so the draw is configuration-independent and the model is
 * read, never written. That matters here in a way it did not in {@code RobotCaptures}: this class is
 * built against a <b>live simulation's</b> model, and a generator that quietly set every joint to
 * zero to place its markers would reset the robot.
 * </p>
 */
public final class MarkerConstellation
{
   /** FRAMEWORK.md §2's recommended minimum. Three determines a pose; four survives one occlusion. */
   public static final int DEFAULT_MARKERS_PER_CLUSTER = 4;

   /** Full spread of the gauge (base-link) cluster, metres. FRAMEWORK.md §1's outrigger bracket. */
   public static final double DEFAULT_GAUGE_SPREAD = 0.14;

   /** Full spread of a limb cluster, metres. Constrained by the width of a real limb. */
   public static final double DEFAULT_LIMB_SPREAD = 0.06;

   /**
    * How far a limb cluster stands off sideways from its link's centre of mass, metres.
    * <p>
    * Markers go on the outside of a segment, not at its centre of mass. Alex's thighs and shins are
    * roughly 0.08 m in radius, so this clears the surface with a small gap -- a bracket bolted on,
    * rather than markers floating inside the link.
    * </p>
    * <p>
    * It is not cosmetic. {@code ^i p_ij} is solved for, so a cluster buried at {@code ^i c_i}
    * calibrates exactly as well; but the lever arm from the link frame to its markers is what turns
    * cluster <i>orientation</i> error into link <i>position</i> error, and a buried cluster has a
    * shorter one than any real bracket. It also cannot be seen, which makes a mocap demonstration
    * with invisible mocap.
    * </p>
    */
   public static final double DEFAULT_LIMB_STANDOFF = 0.12;

   /** As {@link #DEFAULT_LIMB_STANDOFF}, for the gauge. §1's bracket is an outrigger anyway. */
   public static final double DEFAULT_GAUGE_STANDOFF = 0.18;

   /**
    * Where a cluster's markers go on its segment.
    * <p>
    * This mirrors {@code RobotCaptures.MarkerPlacement}, which is the offline generator's copy in
    * test scope. Two copies because the two draw from independently seeded streams and CLAUDE.md's
    * standoff note is explicit that adding or moving a draw in either one silently re-poses every
    * fixed-seed dataset that uses it. Keep them consistent in <i>geometry</i>, not by sharing code.
    * </p>
    */
   public enum MarkerPlacement
   {
      /**
       * Four markers in a patch on one face of the segment, as a bolted-on plate gives you. One
       * azimuth per cluster, so the set shares a face.
       */
      BRACKET,
      /**
       * Markers spread over the whole segment, on an ellipsoidal shell around its centre of mass --
       * lateral radius the standoff, axial radius half the segment. This is what taping markers onto
       * a leg actually produces, and it is what {@code AlexLegDemo} uses.
       */
      SCATTERED
   }

   /**
    * Axial half-length used for a link with no children to measure against, metres.
    * <p>
    * In a leg set that is only the feet. Everything else takes its half-length from the URDF.
    * </p>
    */
   public static final double DEFAULT_TERMINAL_LINK_HALF_LENGTH = 0.10;

   /**
    * A draw is rejected when {@code σ₂ < (spread · COLLINEARITY_FRACTION)²}.
    * <p>
    * One tenth of the spread is loose enough that rejections are rare at these cluster sizes -- so
    * the seed still essentially determines the layout -- and tight enough to exclude the draws that
    * would be refused downstream.
    * </p>
    */
   public static final double COLLINEARITY_FRACTION = 0.1;

   /** Redraw attempts before giving up. Generous: at these spreads a rejection is already rare. */
   private static final int MAXIMUM_ATTEMPTS = 200;

   /**
    * Offset of a cluster centroid from its link's centre of mass, metres, in the link frame.
    * <p>
    * Deliberately non-zero and deliberately not a multiple of anything. A cluster centred on the
    * link CoM would make {@code ^i p_ij} and {@code ^i c_i} coincide on average, and a sign error in
    * either would then cancel in the CoM sum -- which is the one place in this project where a wrong
    * answer would look exactly like a right one.
    * </p>
    */
   private static final Point3D CLUSTER_CENTROID_OFFSET = new Point3D(0.021, -0.013, 0.034);

   private final List<MarkerId> markers;
   private final List<MarkerCluster> clusters;
   private final List<ClusterLayout> trueLayouts;
   private final int resampleCount;

   private MarkerConstellation(List<MarkerId> markers, List<MarkerCluster> clusters, List<ClusterLayout> trueLayouts, int resampleCount)
   {
      this.markers = Collections.unmodifiableList(markers);
      this.clusters = Collections.unmodifiableList(clusters);
      this.trueLayouts = Collections.unmodifiableList(trueLayouts);
      this.resampleCount = resampleCount;
   }

   /**
    * Draws a marker set with the default four markers per cluster and the default spreads.
    *
    * @see #random(RobotModelHandle, List, long, int, double, double)
    */
   public static MarkerConstellation random(RobotModelHandle model, List<String> markedLinks, long seed)
   {
      return random(model, markedLinks, seed, DEFAULT_MARKERS_PER_CLUSTER, DEFAULT_GAUGE_SPREAD, DEFAULT_LIMB_SPREAD, DEFAULT_GAUGE_STANDOFF,
                    DEFAULT_LIMB_STANDOFF);
   }

   /**
    * Draws a marker set.
    *
    * @param model             the robot to place markers on. Read only; its configuration is not
    *                          touched.
    * @param markedLinks       links to carry clusters. The base link, if present, is the gauge and
    *                          gets {@code gaugeSpread}; everything else gets {@code limbSpread}.
    * @param seed              fixed seed; the same seed gives the same layout.
    * @param markersPerCluster markers on each link.
    * @param gaugeSpread       full spread of the gauge cluster, metres.
    * @param limbSpread        full spread of a limb cluster, metres.
    * @throws IllegalArgumentException if a link name is not in the model, if the marked set is
    *                                  empty, or if fewer than {@link MarkerCluster#MINIMUM_MARKERS}
    *                                  markers per cluster are requested.
    */
   public static MarkerConstellation random(RobotModelHandle model,
                                            List<String> markedLinks,
                                            long seed,
                                            int markersPerCluster,
                                            double gaugeSpread,
                                            double limbSpread)
   {
      return random(model, markedLinks, seed, markersPerCluster, gaugeSpread, limbSpread, DEFAULT_GAUGE_STANDOFF, DEFAULT_LIMB_STANDOFF);
   }

   /**
    * Draws a marker set with the cluster standoffs given explicitly.
    *
    * @param gaugeStandoff how far the gauge cluster stands off its link's centre of mass, metres.
    * @param limbStandoff  the same for a limb cluster. Zero puts the cluster at the centre of mass,
    *                      which is inside the link.
    * @see #DEFAULT_LIMB_STANDOFF
    */
   public static MarkerConstellation random(RobotModelHandle model,
                                            List<String> markedLinks,
                                            long seed,
                                            int markersPerCluster,
                                            double gaugeSpread,
                                            double limbSpread,
                                            double gaugeStandoff,
                                            double limbStandoff)
   {
      return random(model, markedLinks, seed, markersPerCluster, gaugeSpread, limbSpread, gaugeStandoff, limbStandoff, MarkerPlacement.BRACKET);
   }

   /**
    * Draws a marker set with the placement given explicitly.
    *
    * @param placement {@link MarkerPlacement#BRACKET} keeps the historical draw byte-for-byte;
    *                  {@link MarkerPlacement#SCATTERED} spreads each cluster over its segment.
    * @see MarkerPlacement
    */
   public static MarkerConstellation random(RobotModelHandle model,
                                            List<String> markedLinks,
                                            long seed,
                                            int markersPerCluster,
                                            double gaugeSpread,
                                            double limbSpread,
                                            double gaugeStandoff,
                                            double limbStandoff,
                                            MarkerPlacement placement)
   {
      if (model == null)
         throw new IllegalArgumentException("Model must not be null.");
      if (markedLinks == null || markedLinks.isEmpty())
         throw new IllegalArgumentException("At least one link must carry a cluster.");
      if (markersPerCluster < MarkerCluster.MINIMUM_MARKERS)
         throw new IllegalArgumentException(
               "A cluster needs at least " + MarkerCluster.MINIMUM_MARKERS + " markers to determine a pose, got " + markersPerCluster + ".");

      Set<String> unique = new LinkedHashSet<>(markedLinks);

      if (unique.size() != markedLinks.size())
         throw new IllegalArgumentException("Marked links contain a duplicate: " + markedLinks + ".");

      for (String link : unique)
      {
         if (!model.hasLink(link))
            throw new IllegalArgumentException("No link named '" + link + "' in the model. Known links: " + model.getLinkNames() + ".");
      }

      Random random = new Random(seed);

      List<String> markerNames = new ArrayList<>();

      for (String link : unique)
      {
         for (int j = 0; j < markersPerCluster; j++)
            markerNames.add(link + "_M" + j);
      }

      List<MarkerId> markers = MarkerId.createDenseSet(markerNames);
      List<MarkerCluster> clusters = new ArrayList<>();

      int markerIndex = 0;

      for (String link : unique)
      {
         List<MarkerId> members = new ArrayList<>(markersPerCluster);

         for (int j = 0; j < markersPerCluster; j++)
            members.add(markers.get(markerIndex++));

         clusters.add(new MarkerCluster(link, members));
      }

      List<ClusterLayout> layouts = new ArrayList<>();
      RigidBodyRegistration registration = new RigidBodyRegistration(markersPerCluster);
      RegistrationResult result = new RegistrationResult();
      Point3D linkCenterOfMass = new Point3D();
      int resampleCount = 0;

      for (MarkerCluster cluster : clusters)
      {
         ClusterLayout layout = new ClusterLayout(cluster);

         boolean isGauge = cluster.getLinkName().equals(model.getBaseLinkName());
         double spread = isGauge ? gaugeSpread : limbSpread;
         double standoff = isGauge ? gaugeStandoff : limbStandoff;
         double minimumSigma2 = squared(spread * COLLINEARITY_FRACTION);

         model.packCenterOfMassInLinkFrame(cluster.getLinkName(), linkCenterOfMass);

         boolean scattered = placement == MarkerPlacement.SCATTERED;

         // One azimuth per cluster, applied perpendicular to the limb's long axis: the markers share
         // one face, the way a bracket does. An offset along the long axis would put the thigh's
         // markers near the knee -- which calibrates perfectly well and is wrong.
         //
         // Drawn ONLY when a bracket is going to use it. An unconditional draw consumes a value from
         // the stream and shifts every subsequent one, which would silently re-pose the robot and
         // re-noise every marker in every fixed-seed dataset built on BRACKET. That is not
         // hypothetical -- CLAUDE.md records it happening once already.
         double azimuth = scattered ? 0.0 : 2.0 * Math.PI * random.nextDouble();
         double offsetX = CLUSTER_CENTROID_OFFSET.getX() + standoff * Math.cos(azimuth);
         double offsetY = CLUSTER_CENTROID_OFFSET.getY() + standoff * Math.sin(azimuth);
         double offsetZ = CLUSTER_CENTROID_OFFSET.getZ();

         // Half the distance to the furthest child link -- the segment's own length as the URDF
         // declares it, so a shin does not get a thigh's worth of markers.
         double axialHalfLength = scattered ? halfSegmentLength(model, cluster.getLinkName(), DEFAULT_TERMINAL_LINK_HALF_LENGTH) : 0.0;

         Point3D[] constellation = new Point3D[cluster.getMarkerCount()];

         for (int attempt = 0;; attempt++)
         {
            for (int j = 0; j < constellation.length; j++)
            {
               if (scattered)
               {
                  // A uniformly random direction on a shell, scaled anisotropically: lateral radius
                  // is the standoff, axial radius is half the segment. cos(polar) is drawn uniform
                  // in [-1, 1] rather than the polar angle itself, or the draws bunch at the ends of
                  // the segment. A little radial jitter because real markers do not sit on an
                  // exact ellipsoid.
                  double cosPolar = 2.0 * random.nextDouble() - 1.0;
                  double markerAzimuth = 2.0 * Math.PI * random.nextDouble();
                  double sinPolar = Math.sqrt(Math.max(0.0, 1.0 - cosPolar * cosPolar));
                  double radialScale = 1.0 + 0.15 * (random.nextDouble() - 0.5);

                  constellation[j] = new Point3D(linkCenterOfMass.getX() + CLUSTER_CENTROID_OFFSET.getX()
                        + radialScale * standoff * sinPolar * Math.cos(markerAzimuth),
                                                 linkCenterOfMass.getY() + CLUSTER_CENTROID_OFFSET.getY()
                                                       + radialScale * standoff * sinPolar * Math.sin(markerAzimuth),
                                                 linkCenterOfMass.getZ() + CLUSTER_CENTROID_OFFSET.getZ() + radialScale * axialHalfLength * cosPolar);
               }
               else
               {
                  constellation[j] = new Point3D(linkCenterOfMass.getX() + offsetX + spread * (random.nextDouble() - 0.5),
                                                 linkCenterOfMass.getY() + offsetY + spread * (random.nextDouble() - 0.5),
                                                 linkCenterOfMass.getZ() + offsetZ + spread * (random.nextDouble() - 0.5));
               }
            }

            if (secondCovarianceEigenvalue(registration, result, constellation) >= minimumSigma2)
               break;

            resampleCount++;

            if (attempt > MAXIMUM_ATTEMPTS)
               throw new IllegalStateException("Could not draw a non-collinear constellation for '" + cluster.getLinkName() + "' in " + MAXIMUM_ATTEMPTS
                     + " attempts. At a spread of " + spread + " m this is not bad luck -- check markersPerCluster and the spread.");
         }

         for (int j = 0; j < constellation.length; j++)
            layout.setPositionInLinkFrame(j, constellation[j], 1);

         layouts.add(layout);
      }

      return new MarkerConstellation(markers, clusters, layouts, resampleCount);
   }

   /**
    * Half the distance from a link's frame to its furthest child link's frame, metres.
    * <p>
    * That distance <b>is</b> the segment length: Alex's link origins sit on their parent joint's
    * axis, so {@code LEFT_THIGH}'s frame is at the hip and its child {@code LEFT_SHIN}'s is at the
    * knee. Taking it from the model rather than from a table means the scatter follows whatever URDF
    * is loaded.
    * </p>
    * <p>
    * The <b>maximum</b> over children, not the mean: IMU and sensor stubs are children too and sit
    * almost on top of their parent, so a mean would be dragged to zero by every massless stub bolted
    * to the segment.
    * </p>
    *
    * @param fallback used for a link with no children at all -- only the feet, in a leg set.
    */
   private static double halfSegmentLength(RobotModelHandle model, String linkName, double fallback)
   {
      RigidBodyTransform childToLink = new RigidBodyTransform();
      double furthest = 0.0;

      for (String candidate : model.getLinkNames())
      {
         if (!linkName.equals(model.getParentLinkName(candidate)))
            continue;

         model.packLinkToLink(candidate, linkName, childToLink);
         furthest = Math.max(furthest, childToLink.getTranslation().norm());
      }

      return furthest > 0.0 ? 0.5 * furthest : fallback;
   }

   /**
    * {@code σ₂} of a point set, m², via a self-registration.
    * <p>
    * {@code RigidBodyRegistration} forms {@code H = (1/L) Σ (b - b̄)(a - ā)ᵀ}. With {@code a = b} that
    * is the covariance of the centred points, so its singular values are the covariance eigenvalues,
    * already sorted descending by {@code compute}. See the class javadoc for why this rather than a
    * second eigen-solver.
    * </p>
    */
   private static double secondCovarianceEigenvalue(RigidBodyRegistration registration, RegistrationResult result, Point3D[] points)
   {
      registration.clear();

      for (Point3D point : points)
         registration.addCorrespondence(point, point);

      if (!registration.compute(result))
         throw new IllegalStateException("Self-registration of " + points.length + " points failed; it cannot, so this is a bug.");

      return result.getSigma2();
   }

   private static double squared(double value)
   {
      return value * value;
   }

   /** Every marker, as a dense set: index {@code i} is position {@code i} in a {@code MocapFrame}. */
   public List<MarkerId> getMarkers()
   {
      return markers;
   }

   /** The clusters, in marked-link order. */
   public List<MarkerCluster> getClusters()
   {
      return clusters;
   }

   /**
    * Truth: {@code ^i p_ij} per cluster, in link frames.
    * <p>
    * This is what a calibration is trying to recover. Handing it straight to
    * {@code runtime.LinkPoseEstimator} demonstrates the runtime with a perfect layout, which is a
    * legitimate thing to demonstrate and is <b>not</b> a demonstration of the calibration -- keep
    * the two claims apart when reporting a number obtained this way.
    * </p>
    */
   public List<ClusterLayout> getTrueLayouts()
   {
      return trueLayouts;
   }

   /** Link names carrying clusters, in order. */
   public List<String> getMarkedLinks()
   {
      List<String> names = new ArrayList<>(clusters.size());

      for (MarkerCluster cluster : clusters)
         names.add(cluster.getLinkName());

      return names;
   }

   /** How many constellations were redrawn for near-collinearity. Normally 0. */
   public int getResampleCount()
   {
      return resampleCount;
   }

   @Override
   public String toString()
   {
      return "MarkerConstellation[" + clusters.size() + " clusters, " + markers.size() + " markers, " + resampleCount + " resampled]";
   }
}
