package us.ihmc.alexMocap.sim;

import java.util.List;
import java.util.Random;

import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Turns known link poses into mocap observations: the sensor half of the simulated system.
 *
 * <h2>What it models, and what it does not</h2>
 * <p>
 * Given {@code ^W T_i} for each marked link and a {@link MarkerConstellation}'s {@code ^i p_ij}, the
 * marker's world position is {@code ^W p_ij = ^W T_i · ^i p_ij}. Each observation then gets
 * independent per-axis Gaussian noise, and is dropped entirely with probability
 * {@code occlusionProbability}.
 * </p>
 * <p>
 * That is a deliberately thin model and it is worth being explicit about what it leaves out, because
 * the omissions are the ones that dominate on a real stage: no marker swaps or mislabelling, no
 * ghost markers, no correlated error across markers seen by the same camera, no latency, and no
 * drop-out correlated with limb occlusion geometry. Noise here is i.i.d. and zero-mean, which is the
 * most forgiving assumption available. A CoM error obtained against this camera is a <b>lower
 * bound</b> on the error against a real one.
 * </p>
 * <p>
 * The one systematic effect that <i>is</i> worth reaching for is a constant per-marker bias, and
 * this class does not have it, for a measured reason: G4's held-out split carries a systematic bias
 * equally in both halves, so it is invisible to the gate that exists to catch it. Adding a bias here
 * would produce a demonstration whose fault no gate reports. See CLAUDE.md, "G4 shows no asymmetry".
 * </p>
 *
 * <h2>Noise, and the number to use</h2>
 * <p>
 * FRAMEWORK.md §17's figure for a tight volume at the gantry is {@code σ = 0.3 mm} per axis, which
 * is {@link #GANTRY_NOISE_STANDARD_DEVIATION}. Note that §17's other figure, 0.0037 m/s, is a
 * pelvis-<i>pose</i> noise and not a marker noise -- F6 registers four markers into one pose, so
 * what reaches a differentiator is nearer {@code σ/√N}. Do not set this field from that number.
 * </p>
 *
 * <h2>Occlusion is per-marker and memoryless</h2>
 * <p>
 * A real dropout lasts as long as the limb is behind something, so it spans many frames and takes
 * the same markers with it each time. This one is redrawn every frame and independently per marker,
 * which makes it a test of the refusal path ({@code MarkerCluster.MINIMUM_MARKERS}) rather than a
 * model of occlusion. It is useful for showing that the CoM goes NaN rather than quietly becoming
 * the CoM of a lighter robot; it is not useful for estimating how often that will happen.
 * </p>
 *
 * <h2>Allocation</h2>
 * <p>
 * {@link #observe} allocates nothing. It runs inside an SCS2 controller tick, where a per-frame
 * allocation is a garbage-collection pause in the middle of a physics loop.
 * </p>
 */
public final class SimulatedMocapCamera
{
   /** FRAMEWORK.md §17's per-axis marker noise for a tight volume at the gantry, metres. */
   public static final double GANTRY_NOISE_STANDARD_DEVIATION = 0.3e-3;

   /**
    * Supplies {@code ^W T_i}: where a link actually is.
    * <p>
    * An interface rather than a map so that a caller inside a simulation can read the live frame
    * directly, without building a map of transforms every tick.
    * </p>
    */
   @FunctionalInterface
   public interface LinkPoseSource
   {
      /**
       * Packs {@code ^W T_i} for the named link.
       *
       * @return false if the link has no pose right now, in which case its whole cluster is reported
       *         not-visible.
       */
      boolean packLinkPose(String linkName, RigidBodyTransform poseToPack);
   }

   private final MarkerConstellation constellation;
   private final double noiseStandardDeviation;
   private final double occlusionProbability;
   private final Random random;

   // Preallocated scratch; see the class javadoc on allocation.
   private final RigidBodyTransform linkPose = new RigidBodyTransform();
   private final Point3D markerPosition = new Point3D();

   private int lastVisibleCount;
   private int lastOccludedCount;
   private int lastMissingLinkCount;

   /**
    * @param constellation          the marker set, and the truth to project.
    * @param noiseStandardDeviation per-axis Gaussian noise on each observation, metres. Zero for a
    *                               noiseless camera.
    * @param occlusionProbability   probability that any one marker is missing in any one frame, in
    *                               {@code [0, 1)}.
    * @param seed                   fixed seed, so a run is reproducible.
    */
   public SimulatedMocapCamera(MarkerConstellation constellation, double noiseStandardDeviation, double occlusionProbability, long seed)
   {
      if (constellation == null)
         throw new IllegalArgumentException("Constellation must not be null.");
      if (!(noiseStandardDeviation >= 0.0))
         throw new IllegalArgumentException("Noise standard deviation must be finite and non-negative, got " + noiseStandardDeviation + ".");
      if (!(occlusionProbability >= 0.0) || occlusionProbability >= 1.0)
         throw new IllegalArgumentException("Occlusion probability must be in [0, 1), got " + occlusionProbability
               + ". At 1.0 nothing is ever seen, which is not a camera fault this project models.");

      this.constellation = constellation;
      this.noiseStandardDeviation = noiseStandardDeviation;
      this.occlusionProbability = occlusionProbability;
      this.random = new Random(seed);
   }

   /**
    * A {@link MocapFrame} sized for this camera's marker set.
    * <p>
    * Build one and reuse it; {@link #observe} overwrites it in place.
    * </p>
    */
   public MocapFrame newFrame()
   {
      return new MocapFrame(constellation.getMarkers());
   }

   /**
    * Observes one frame.
    *
    * @param linkPoses   where the links actually are.
    * @param frameToPack overwritten. Must have been built by {@link #newFrame()} -- a frame with a
    *                    different marker set is rejected rather than partially filled, since a
    *                    silently mismatched frame reads downstream as an occlusion pattern.
    */
   public void observe(LinkPoseSource linkPoses, MocapFrame frameToPack)
   {
      if (linkPoses == null)
         throw new IllegalArgumentException("Link pose source must not be null.");
      if (frameToPack.getMarkerCount() != constellation.getMarkers().size())
         throw new IllegalArgumentException("Frame holds " + frameToPack.getMarkerCount() + " markers but this camera observes "
               + constellation.getMarkers().size() + ". Build the frame with newFrame().");

      frameToPack.clear();

      List<MarkerCluster> clusters = constellation.getClusters();
      List<ClusterLayout> layouts = constellation.getTrueLayouts();

      lastVisibleCount = 0;
      lastOccludedCount = 0;
      lastMissingLinkCount = 0;

      for (int c = 0; c < clusters.size(); c++)
      {
         MarkerCluster cluster = clusters.get(c);
         ClusterLayout layout = layouts.get(c);

         // A link with no pose is not the same thing as an occluded marker, but it reaches the
         // estimator the same way -- as a cluster with too few points. It is counted separately so
         // that a wiring fault (a link name the simulation does not know) is distinguishable from a
         // camera that simply did not see anything.
         if (!linkPoses.packLinkPose(cluster.getLinkName(), linkPose))
         {
            lastMissingLinkCount++;
            continue;
         }

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            MarkerId marker = cluster.getMarker(j);

            if (occlusionProbability > 0.0 && random.nextDouble() < occlusionProbability)
            {
               lastOccludedCount++;
               continue;
            }

            markerPosition.set(layout.getPositionInLinkFrame(j));
            linkPose.transform(markerPosition);

            if (noiseStandardDeviation > 0.0)
            {
               markerPosition.add(noiseStandardDeviation * random.nextGaussian(),
                                  noiseStandardDeviation * random.nextGaussian(),
                                  noiseStandardDeviation * random.nextGaussian());
            }

            frameToPack.get(marker).setVisible(markerPosition);
            lastVisibleCount++;
         }
      }
   }

   /** The marker set this camera observes. */
   public MarkerConstellation getConstellation()
   {
      return constellation;
   }

   /** Markers reported visible by the last {@link #observe}. */
   public int getLastVisibleCount()
   {
      return lastVisibleCount;
   }

   /** Markers dropped by the occlusion draw in the last {@link #observe}. */
   public int getLastOccludedCount()
   {
      return lastOccludedCount;
   }

   /**
    * Marked links whose pose the source could not supply in the last {@link #observe}.
    * <p>
    * Non-zero means a wiring fault, not a sensor event. Worth an assertion in a caller rather than a
    * plot: it is constant once the names are right.
    * </p>
    */
   public int getLastMissingLinkCount()
   {
      return lastMissingLinkCount;
   }

   @Override
   public String toString()
   {
      return "SimulatedMocapCamera[sigma=" + 1000.0 * noiseStandardDeviation + " mm, occlusion=" + occlusionProbability + ", "
            + constellation.getMarkers().size() + " markers]";
   }
}
