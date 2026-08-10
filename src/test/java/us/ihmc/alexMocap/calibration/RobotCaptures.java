package us.ihmc.alexMocap.calibration;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

import us.ihmc.alexMocap.core.Capture;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * {@link SyntheticCaptures}' analogue for a <b>real</b> robot model: the vendored Alex URDF, with a
 * randomised marker set on whichever links the caller names.
 *
 * <h2>Why this is a second class and not a parameter on the first</h2>
 * <p>
 * {@link SyntheticCaptures} is not modified, and must not be. Four seeded test classes depend on
 * its exact sequence of {@code Random} draws -- adding one call anywhere inside it changes every
 * number in {@code PlantAndRecoverTest}, {@code GateInjectionTest},
 * {@code RuntimeGroundTruthTest} and {@code ReplayRunnerTest}. Its hardcoded joint-limit and
 * subtree tables are also a deliberate choice documented in its javadoc, and they are exactly what
 * cannot be carried to a 29-joint robot.
 * </p>
 *
 * <h2>The four things that had to change for a real model</h2>
 * <ol>
 * <li><b>Joint limits come from the URDF</b>, through
 * {@link RobotModelHandle#getJointLimitLower(int)}, not from a table in this file. One parse, one
 * source of truth. A non-finite limit <b>throws</b> rather than being sampled -- URDF permits an
 * unbounded revolute joint and uniform sampling inside {@code [-inf, inf]} yields NaN, which then
 * propagates into every marker position and reads as a solver bug.</li>
 * <li><b>Only a named subset of joints is randomised.</b> Alex has 29; the calibration is about
 * the 12 in the legs. The other 17 are held at a rest angle, clamped into their limits.</li>
 * <li><b>Markers are planted around each link's centre of mass</b>
 * ({@link RobotModelHandle#packCenterOfMassInLinkFrame}), not at a fixed offset from the link
 * origin. Alex's link origins sit at their parent joint's axis, so {@code LEFT_THIGH}'s origin is
 * at the hip -- the toy's {@code (+0.03, -0.02, -0.05)} offset would put "thigh" markers inside the
 * pelvis. A small deterministic offset is kept on top of the CoM so the cluster is not centred on
 * anything, which is what stops a whole class of sign error from cancelling.</li>
 * <li><b>Near-collinear constellations are resampled.</b> A random draw of four points can come out
 * close to a line, and a collinear cluster yields a well-formed rotation carrying no information
 * (FRAMEWORK.md §18.1). The guard is on {@code λ₂} of the constellation's covariance, <b>not</b>
 * {@code λ₃}: coplanar is the normal case for markers on a flat link face and must not be
 * resampled away (RUNNING.md, "Refuse on σ₂, not σ₃").</li>
 * </ol>
 *
 * <h2>What "recovered" means here</h2>
 * <p>
 * Unchanged from {@link SyntheticCaptures}: the planted layouts are recoverable exactly, the
 * planted {@code Δ} is not, and the convention-free statement is the base pose
 * {@code ^W T_c^(k) · Δ̂}. See that class's javadoc.
 * </p>
 */
public class RobotCaptures
{
   /** Where the vendored Alex URDF lives, resolved from the test classpath. */
   public static Path alexUrdfPath() throws Exception
   {
      return Path.of(RobotCaptures.class.getResource("/us/ihmc/alexMocap/model/alex.urdf").toURI());
   }

   public static RobotModelHandle alexModel() throws Exception
   {
      return RobotModelHandle.fromURDF(alexUrdfPath());
   }

   /** The 12 leg joints: everything between the pelvis and either foot. */
   public static final String[] LEG_JOINTS = {"LEFT_HIP_X", "LEFT_HIP_Z", "LEFT_HIP_Y", "LEFT_KNEE_Y", "LEFT_ANKLE_Y", "LEFT_ANKLE_X", "RIGHT_HIP_X",
         "RIGHT_HIP_Z", "RIGHT_HIP_Y", "RIGHT_KNEE_Y", "RIGHT_ANKLE_Y", "RIGHT_ANKLE_X"};

   /** The 13 links of the two leg chains plus the pelvis. */
   public static final String[] LEG_LINKS = {"PELVIS_LINK", "LEFT_HIP_X_LINK", "LEFT_HIP_Z_LINK", "LEFT_THIGH", "LEFT_SHIN", "LEFT_ANKLE_Y_LINK", "LEFT_FOOT",
         "RIGHT_HIP_X_LINK", "RIGHT_HIP_Z_LINK", "RIGHT_THIGH", "RIGHT_SHIN", "RIGHT_ANKLE_Y_LINK", "RIGHT_FOOT"};

   /**
    * The primary marked set: the gauge plus the six heaviest leg links.
    * <p>
    * FRAMEWORK.md §1 marks in descending order of {@code mass × lever arm}. On the real inertials
    * that selects thigh (8.281 kg), shin (6.338 kg) and foot (0.810 kg, but at the longest lever of
    * anything below the pelvis) over the hip stubs (2.332 kg and 0.761 kg, both essentially at the
    * pelvis) and the ankle stub (0.050 kg). The pelvis is the gauge and is structurally required,
    * not chosen.
    * </p>
    */
   public static final String[] PRIMARY_MARKED_LINKS = {"PELVIS_LINK", "LEFT_THIGH", "RIGHT_THIGH", "LEFT_SHIN", "RIGHT_SHIN", "LEFT_FOOT", "RIGHT_FOOT"};

   /**
    * The marked set predicted to be degenerate, and the real-robot analogue of
    * {@link SyntheticCaptures}' {@code DEGENERATE_MARKED_LINKS}.
    * <p>
    * {@code PELVIS_LINK} never rotates relative to the base at all, and {@code *_HIP_X_LINK}'s
    * orientation is {@code R_x(q)} for the hip-X joint alone -- and the two hip-X axes are
    * <b>parallel</b>, both {@code (1 0 0)} in the pelvis frame. So every marked link's rotation is
    * a rotation about the same {@code x}, a pure translation {@code g} along {@code x} satisfies
    * {@code R_i(q)ᵀ g = g} at every capture, and {@code Δ → Δ·G} is an exact symmetry of {@code J}.
    * </p>
    */
   public static final String[] HIP_X_ONLY_MARKED_LINKS = {"PELVIS_LINK", "LEFT_HIP_X_LINK", "RIGHT_HIP_X_LINK"};

   private static final int MARKERS_PER_CLUSTER = 4;

   /**
    * Offset of the cluster centroid from the link's centre of mass, metres, in the link frame.
    * <p>
    * Deliberately non-zero and deliberately not a multiple of anything: a cluster centred on the
    * CoM would make {@code ^i p_ij} and {@code ^i c_i} coincide on average, and a sign error in
    * either would then cancel in the CoM sum.
    * </p>
    */
   private static final Point3D CLUSTER_CENTROID_OFFSET = new Point3D(0.021, -0.013, 0.034);

   /** How the markers of one cluster are arranged on their link. */
   public enum MarkerPlacement
   {
      /**
       * All of a cluster's markers in a small box on one face, like a bolted-on bracket.
       * <p>
       * Compact and easy to mount, and the worst case for conditioning: a cluster whose extent is
       * small compared to its distance from the link frame turns a little orientation error into a
       * lot of position error.
       * </p>
       */
      BRACKET,
      /**
       * Markers scattered over the whole segment, on an ellipsoidal shell around the link's centre
       * of mass.
       * <p>
       * This is what taping markers all over a limb gives you, and it is the case worth testing: the
       * constellation spans the segment rather than a hand-sized patch of it, so {@code σ₂} and
       * {@code σ₃} are far larger and the pose is much better determined. It is also the honest
       * test of the framework -- a marker set that only works when the markers are conveniently
       * grouped is not much of a marker set.
       * </p>
       */
      SCATTERED
   }

   /** Knobs. Defaults are the clean, noiseless case with the primary marked set. */
   public static class Options
   {
      public long seed = 20260810L;
      public int captureCount = 30;

      /** Per-axis Gaussian noise on every marker measurement, metres. */
      public double markerNoiseStandardDeviation = 0.0;

      /** Probability that any given marker is occluded in any given capture. */
      public double occlusionProbability = 0.0;

      /**
       * Constant encoder offsets, radians, keyed by joint name. The robot is at
       * {@code q_reported + offset}; the encoder says {@code q_reported}.
       */
      public final Map<String, Double> jointOffsets = new LinkedHashMap<>();

      /** Which links carry clusters. */
      public String[] markedLinks = PRIMARY_MARKED_LINKS;

      /** Which joints are swept. Everything else is held at {@link #restAngle}. */
      public String[] randomizedJoints = LEG_JOINTS;

      /** Where the un-swept joints sit, before clamping into their URDF limits. */
      public double restAngle = 0.0;

      /**
       * Fraction of each swept joint's URDF range actually used, centred on the range midpoint.
       * <p>
       * 1.0 is the full sweep FRAMEWORK.md §1 asks for. A small value is the failure mode that
       * "looks converged and means nothing": rank is generally still fine, conditioning is not.
       * Centred on the midpoint rather than on {@link #restAngle} so a narrow sweep stays inside
       * the limits by construction, whatever the rest angle is.
       * </p>
       */
      public double jointExcursionFraction = 1.0;

      /**
       * If finite, sweep each randomised joint over {@code [rest ± this] ∩ limits} instead of about
       * the range midpoint. Radians. NaN (the default) keeps the midpoint behaviour.
       *
       * <h2>Why a second knob rather than a smaller {@link #jointExcursionFraction}</h2>
       * <p>
       * The two centre on different places, and on Alex that difference is the whole point. The
       * range midpoint is {@code HIP_Y = -52.5°, KNEE_Y = +70°} -- a deep tuck -- so narrowing
       * {@code jointExcursionFraction} converges on a squat, not on a robot hanging from a gantry.
       * Measured, feet relative to pelvis:
       * </p>
       * <pre>
       * all zeros            0.890 m below, 0.240 m stance   <- hanging
       * range midpoint       0.683 m below, 0.774 m stance   <- tucked and splayed
       * full-range draws     0.18-0.67 m below               <- a foot 0.60 m forward
       * rest ± 0.45 rad      >= 0.733 m below                <- hanging, and moving
       * </pre>
       * <p>
       * This is also the more faithful model of the physical procedure. Nobody calibrating a robot
       * on a gantry commands a 140° knee and a -150° hip; they move it through a reachable,
       * non-self-colliding envelope about its rest posture. FRAMEWORK.md §1 asks for excursion, and
       * it is right that excursion is what makes {@code Δ} identifiable -- but excursion about a
       * sensible pose is what an operator can actually deliver.
       * </p>
       */
      public double sweepHalfRangeRadians = Double.NaN;

      /**
       * If finite, reject any draw whose feet are laterally closer than this, metres. NaN (the
       * default) accepts every draw.
       * <p>
       * A sweep about the rest pose keeps the feet <i>below</i> the robot but says nothing about
       * left versus right. On Alex, ±0.45 rad of hip roll on a 0.89 m leg is ±0.39 m of lateral
       * travel against a rest stance of 0.24 m, so the ankles pass through each other regularly.
       * Legal, well conditioned, and it looks like a fault.
       * </p>
       * <p>
       * Rejection rather than tighter hip limits because crossing is a property of the two legs
       * <b>together</b>: a hip roll that is fine alone crosses once the other hip rolls the other
       * way, and no per-joint bound expresses that without discarding most of the envelope. The
       * cost is a mild bias -- draws near the crossing boundary are removed, so the sampled
       * distribution is no longer exactly uniform over the box. {@code crossedLegResampleCount}
       * reports how much was thrown away.
       * </p>
       */
      public double minimumFootSeparation = Double.NaN;

      /** Feet whose lateral separation {@link #minimumFootSeparation} constrains. */
      public String leftFootLink = "LEFT_FOOT";
      public String rightFootLink = "RIGHT_FOOT";

      /**
       * Where the suspended robot nominally hangs, metres, in the motive world.
       * <p>
       * The default is deliberately <b>not</b> the origin. Every {@code ^W T_i} in the pipeline is a
       * full pose, and code that quietly assumes the base is at identity produces exactly the right
       * answer when it is -- which is how the visualizer came to draw the robot at the origin for a
       * whole PR without anybody noticing. Keeping the generator off-origin means a test that gets
       * the base pose wrong reports a metre of error rather than none.
       * </p>
       * <p>
       * A caller that wants the robot in front of the camera can move it; {@code AlexLegDemo} sets
       * {@code (0, 0, 1.4)} so the robot hangs directly above the origin triad. Note it keeps the
       * <b>height</b>: a robot drawn at identity would sit at {@code z = 0}, so the placement bug
       * would still be visible even in that view.
       * </p>
       */
      public double baseNominalX = 1.0;
      public double baseNominalY = 2.0;
      public double baseNominalZ = 1.4;

      /** Base pose wander between captures: a suspended robot is not perfectly still. */
      public double basePositionWander = 0.02;
      public double baseOrientationWander = Math.toRadians(2.0);

      /** Full spread of the gauge cluster, metres. FRAMEWORK.md §1's outrigger bracket. */
      public double gaugeClusterSpread = 0.14;

      /** Full spread of the limb clusters, metres. Constrained by the width of a real limb. */
      public double limbClusterSpread = 0.06;

      /**
       * How far a cluster stands off sideways from its link's centre of mass, metres. NaN or 0
       * leaves it at {@code ^i c_i}, which is the old behaviour.
       *
       * <h2>Why this exists</h2>
       * <p>
       * Without it every marker lands within a few centimetres of the link's centre of mass -- that
       * is, <b>inside the link</b>. The arithmetic does not care: {@code ^i p_ij} is solved for, so
       * a cluster buried in the thigh calibrates exactly as well as one taped to it. Two things do
       * care. Nobody can see the markers, because the mesh is drawn over them, which makes a mocap
       * demonstration in which the mocap is invisible. And it is not where markers go, so the lever
       * arms in the error budget are shorter than the real ones.
       * </p>
       * <p>
       * The offset is applied perpendicular to the limb's long axis, at one azimuth per cluster: a
       * bracket is bolted to the side of a segment and its markers share a face. Offsetting along
       * the long axis instead would put the thigh's markers near the knee.
       * </p>
       */
      public double limbStandoff = Double.NaN;

      /** @see #limbStandoff */
      public double gaugeStandoff = Double.NaN;

      /**
       * How a cluster's markers are arranged on its link.
       * <p>
       * {@link MarkerPlacement#BRACKET} is the default so that every existing seeded dataset keeps
       * its numbers. {@link MarkerPlacement#SCATTERED} is what the demonstration uses.
       * </p>
       */
      public MarkerPlacement placement = MarkerPlacement.BRACKET;

      /**
       * Fallback half-length for a link with no child joint to measure against, metres.
       * <p>
       * Only the feet, in the leg set. Everything else gets its extent from the offset to its child
       * link, which is the segment length the URDF actually declares.
       * </p>
       */
      public double terminalLinkHalfLength = 0.10;

      /**
       * Reject and redraw a constellation whose second covariance eigenvalue falls below
       * {@code (spread/10)²}. See the class javadoc for why the guard is {@code λ₂} and not
       * {@code λ₃}.
       */
      public boolean rejectCollinearConstellations = true;

      public Options seed(long seed)
      {
         this.seed = seed;
         return this;
      }

      public Options captures(int captureCount)
      {
         this.captureCount = captureCount;
         return this;
      }

      public Options noise(double standardDeviation)
      {
         this.markerNoiseStandardDeviation = standardDeviation;
         return this;
      }

      public Options occlusion(double probability)
      {
         this.occlusionProbability = probability;
         return this;
      }

      public Options jointOffset(String jointName, double radians)
      {
         jointOffsets.put(jointName, radians);
         return this;
      }

      public Options marked(String... linkNames)
      {
         this.markedLinks = linkNames;
         return this;
      }

      public Options randomize(String... jointNames)
      {
         this.randomizedJoints = jointNames;
         return this;
      }

      public Options excursionFraction(double fraction)
      {
         this.jointExcursionFraction = fraction;
         return this;
      }

      /** @see #sweepHalfRangeRadians */
      public Options sweepAboutRest(double halfRangeRadians)
      {
         this.sweepHalfRangeRadians = halfRangeRadians;
         return this;
      }

      /** @see #minimumFootSeparation */
      public Options uncrossedLegs(double minimumFootSeparationMetres)
      {
         this.minimumFootSeparation = minimumFootSeparationMetres;
         return this;
      }

      /** @see #placement */
      public Options placement(MarkerPlacement placement)
      {
         this.placement = placement;
         return this;
      }

      /** @see #limbStandoff */
      public Options standoff(double gaugeMetres, double limbMetres)
      {
         this.gaugeStandoff = gaugeMetres;
         this.limbStandoff = limbMetres;
         return this;
      }

      /** @see #baseNominalX */
      public Options basePosition(double x, double y, double z)
      {
         this.baseNominalX = x;
         this.baseNominalY = y;
         this.baseNominalZ = z;
         return this;
      }

      public Options gaugeSpread(double metres)
      {
         this.gaugeClusterSpread = metres;
         return this;
      }

      public Options allowCollinearConstellations()
      {
         this.rejectCollinearConstellations = false;
         return this;
      }

      public Options rigidBase()
      {
         this.basePositionWander = 0.0;
         this.baseOrientationWander = 0.0;
         return this;
      }
   }

   /** Generated captures plus the truth they were generated from. */
   public static class Planted
   {
      public CaptureSet captureSet;
      public RobotModelHandle model;
      public List<MarkerCluster> clusters;
      public List<MarkerId> markers;

      /** Truth: {@code ^i p_ij} per cluster, in link frames. */
      public List<ClusterLayout> layouts;

      /** Truth: {@code ^W T_b^(k)}. */
      public RigidBodyTransform[] basePoses;

      /** The reported (not true) joint vectors, per capture. */
      public double[][] reportedJointAngles;

      /**
       * One line per joint whose requested rest angle did not sit strictly inside its URDF limits.
       * <p>
       * Worth printing rather than swallowing: {@code LEFT_KNEE_Y} and {@code RIGHT_KNEE_Y} declare
       * {@code lower="0"}, so the natural rest angle of zero is <b>exactly on the boundary</b>. It
       * needs no clamping, and a generator that silently clamped would give the same answer as one
       * that silently did not -- which is the situation in which a real out-of-range rest angle goes
       * unnoticed.
       * </p>
       */
      public List<String> restAngleNotes;

      /** How many constellations were redrawn for near-collinearity. */
      public int collinearResampleCount;

      /**
       * How many joint draws were thrown away for crossing the legs.
       * <p>
       * Worth reporting rather than hiding: a large count means the rejection is doing most of the
       * sampling, and the accepted set is correspondingly less uniform than it looks.
       * </p>
       */
      public int crossedLegResampleCount;

      public ClusterLayout plantedLayout(String linkName)
      {
         for (ClusterLayout layout : layouts)
         {
            if (layout.getLinkName().equals(linkName))
               return layout;
         }

         throw new IllegalArgumentException("No planted layout for '" + linkName + "'.");
      }
   }

   /**
    * Lateral distance between the two feet at a candidate configuration, metres, in the base frame.
    * <p>
    * Measured in the base frame rather than the world so it is independent of where the robot is
    * standing and of its yaw. Mutates the model's configuration, which is safe here: the caller
    * overwrites it with the accepted draw immediately afterwards.
    * </p>
    */
   private static double footSeparation(RobotModelHandle model, Options options, double[] q)
   {
      model.setQ(q);
      model.updateFrames();

      RigidBodyTransform left = new RigidBodyTransform();
      RigidBodyTransform right = new RigidBodyTransform();
      model.packLinkToBase(options.leftFootLink, left);
      model.packLinkToBase(options.rightFootLink, right);

      return left.getTranslationY() - right.getTranslationY();
   }

   /**
    * Half the distance from a link's frame to its furthest child link's frame, metres.
    * <p>
    * That distance <b>is</b> the segment length: Alex's link origins sit on their parent joint's
    * axis, so {@code LEFT_THIGH}'s frame is at the hip and its child {@code LEFT_SHIN}'s is at the
    * knee. Taking it from the model rather than from a table means the scatter follows whatever URDF
    * is loaded, and a shin does not get a thigh's worth of markers.
    * </p>
    * <p>
    * IMU and sensor stubs are children too and sit almost on top of their parent, which is why this
    * takes the <b>maximum</b> over children rather than the mean: a mean would be dragged toward
    * zero by every massless stub bolted to the segment.
    * </p>
    *
    * @param fallback used for a link with no children at all -- only the feet, in the leg set.
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

   public static Planted generate(Options options) throws Exception
   {
      Random random = new Random(options.seed);
      Planted planted = new Planted();

      planted.model = alexModel();
      RobotModelHandle model = planted.model;

      List<String> markerNames = new ArrayList<>();

      for (String link : options.markedLinks)
      {
         if (!model.hasLink(link))
            throw new IllegalArgumentException("No link named '" + link + "' in the Alex URDF. Known links: " + model.getLinkNames() + ".");

         for (int j = 0; j < MARKERS_PER_CLUSTER; j++)
            markerNames.add(link + "_M" + j);
      }

      planted.markers = MarkerId.createDenseSet(markerNames);
      planted.clusters = new ArrayList<>();

      int markerIndex = 0;

      for (String link : options.markedLinks)
      {
         List<MarkerId> members = new ArrayList<>();

         for (int j = 0; j < MARKERS_PER_CLUSTER; j++)
            members.add(planted.markers.get(markerIndex++));

         planted.clusters.add(new MarkerCluster(link, members));
      }

      // The layouts have to be planted at q = 0 with frames updated, because ^i c_i is read through
      // a frame lookup and the frames must be live. It is configuration-independent -- ^i c_i is a
      // constant of the link -- but the frame tree still has to have been updated once.
      model.setQ(new double[model.getJointCount()]);
      model.updateFrames();

      planted.layouts = new ArrayList<>();
      planted.collinearResampleCount = 0;

      Point3D centerOfMass = new Point3D();

      for (MarkerCluster cluster : planted.clusters)
      {
         ClusterLayout layout = new ClusterLayout(cluster);

         boolean isGauge = cluster.getLinkName().equals(model.getBaseLinkName());
         double spread = isGauge ? options.gaugeClusterSpread : options.limbClusterSpread;
         double standoff = isGauge ? options.gaugeStandoff : options.limbStandoff;

         boolean scattered = options.placement == MarkerPlacement.SCATTERED;
         boolean standingOff = Double.isFinite(standoff) && standoff > 0.0;

         // One azimuth per cluster: in BRACKET the four markers sit on one face of the segment, the
         // way a bolted-on plate does, rather than being scattered around it.
         //
         // Drawn ONLY when it is going to be used. An unconditional draw here consumes a value from
         // the stream and shifts every subsequent one, which silently changes every fixed-seed
         // dataset in the project -- it re-poses the robot, re-noises the markers, and turns a clean
         // G2 into an indictment of RIGHT_KNEE_Y. That is not hypothetical; it is what this line did
         // before the guard was added.
         double clusterAzimuth = (standingOff && !scattered) ? 2.0 * Math.PI * random.nextDouble() : 0.0;

         // Half the distance to the furthest child link: the segment's own length, as the URDF
         // declares it. A terminal link -- only the feet, in the leg set -- has no child to measure
         // against and falls back to a stated default.
         double axialHalfLength = scattered ? halfSegmentLength(model, cluster.getLinkName(), options.terminalLinkHalfLength) : 0.0;

         model.packCenterOfMassInLinkFrame(cluster.getLinkName(), centerOfMass);

         Point3D[] constellation = new Point3D[cluster.getMarkerCount()];

         // Redraw until the constellation is not near-collinear. The cap is generous: at these
         // spreads a rejection is rare, and a loop that cannot terminate is worse than a throw.
         for (int attempt = 0;; attempt++)
         {
            // Where the cluster sits relative to the link's centre of mass. Without a standoff the
            // markers land within a few centimetres of ^i c_i, which is INSIDE the link -- fine for
            // the arithmetic, invisible on screen, and not where anyone tapes a marker.
            double standoffX = CLUSTER_CENTROID_OFFSET.getX();
            double standoffY = CLUSTER_CENTROID_OFFSET.getY();
            double standoffZ = CLUSTER_CENTROID_OFFSET.getZ();

            if (standingOff)
            {
               // Push the cluster out sideways, perpendicular to the limb's long axis. A real
               // bracket is bolted to the side of a segment, not to its end -- an offset along the
               // long axis would put the thigh's markers somewhere near the knee.
               //
               // The azimuth is drawn once per cluster, so the four markers stay on one face
               // instead of being scattered around the limb.
               standoffX += standoff * Math.cos(clusterAzimuth);
               standoffY += standoff * Math.sin(clusterAzimuth);
            }

            for (int j = 0; j < constellation.length; j++)
            {
               if (scattered)
               {
                  // A uniformly random direction, then scaled anisotropically: the lateral radius is
                  // the standoff, the axial one is half the segment. That is an ellipsoidal shell
                  // hugging the limb, so the markers land all over the segment rather than in a
                  // patch -- which is what someone taping markers to a leg actually produces.
                  //
                  // cos(polar) uniform in [-1, 1] rather than the polar angle itself, or the draws
                  // bunch at the ends of the segment.
                  double cosPolar = 2.0 * random.nextDouble() - 1.0;
                  double azimuth = 2.0 * Math.PI * random.nextDouble();
                  double sinPolar = Math.sqrt(Math.max(0.0, 1.0 - cosPolar * cosPolar));

                  // A little radial jitter so the markers are not exactly on one shell; real markers
                  // sit on a surface that is not an ellipsoid.
                  double radialScale = 1.0 + 0.15 * (random.nextDouble() - 0.5);

                  constellation[j] = new Point3D(centerOfMass.getX() + CLUSTER_CENTROID_OFFSET.getX() + radialScale * standoff * sinPolar * Math.cos(azimuth),
                                                 centerOfMass.getY() + CLUSTER_CENTROID_OFFSET.getY() + radialScale * standoff * sinPolar * Math.sin(azimuth),
                                                 centerOfMass.getZ() + CLUSTER_CENTROID_OFFSET.getZ() + radialScale * axialHalfLength * cosPolar);
               }
               else
               {
                  constellation[j] = new Point3D(centerOfMass.getX() + standoffX + spread * (random.nextDouble() - 0.5),
                                                 centerOfMass.getY() + standoffY + spread * (random.nextDouble() - 0.5),
                                                 centerOfMass.getZ() + standoffZ + spread * (random.nextDouble() - 0.5));
               }
            }

            if (!options.rejectCollinearConstellations || secondCovarianceEigenvalue(constellation) >= squared(spread / 10.0))
               break;

            planted.collinearResampleCount++;

            if (attempt > 200)
               throw new IllegalStateException("Could not draw a non-collinear constellation for '" + cluster.getLinkName() + "' in 200 attempts.");
         }

         for (int j = 0; j < constellation.length; j++)
            layout.setPositionInLinkFrame(j, constellation[j], 1);

         planted.layouts.add(layout);
      }

      // Joint sampling ranges. Everything not named for randomisation is held at the rest angle,
      // clamped into its limits, and the clamp is reported rather than swallowed.
      Set<String> randomized = new LinkedHashSet<>(Arrays.asList(options.randomizedJoints));

      for (String jointName : randomized)
         model.indexOfJoint(jointName); // throws with the known-joint list if the name is wrong

      double[] sampleLower = new double[model.getJointCount()];
      double[] sampleUpper = new double[model.getJointCount()];
      double[] held = new double[model.getJointCount()];
      List<String> notes = new ArrayList<>();

      for (int j = 0; j < model.getJointCount(); j++)
      {
         String jointName = model.getJointNames().get(j);
         double lower = model.getJointLimitLower(j);
         double upper = model.getJointLimitUpper(j);

         if (!Double.isFinite(lower) || !Double.isFinite(upper))
            throw new IllegalArgumentException("Joint '" + jointName + "' has a non-finite limit [" + lower + ", " + upper
                  + "]. Sampling uniformly inside it would produce NaN joint angles, which look downstream like a solver fault "
                  + "rather than a URDF one. Give it a finite <limit> or exclude it from the sweep.");
         if (!(upper > lower))
            throw new IllegalArgumentException("Joint '" + jointName + "' has an empty range [" + lower + ", " + upper + "].");

         held[j] = Math.max(lower, Math.min(upper, options.restAngle));

         if (Double.isFinite(options.sweepHalfRangeRadians))
         {
            // Sweep about the rest pose, clipped to the limits. Not the same thing as a narrow
            // excursion about the midpoint -- see Options.sweepAboutRest.
            //
            // Clipping rather than shifting is deliberate. A joint whose rest angle sits on a limit
            // -- both knees do, lower="0" -- gets a one-sided sweep, which is correct: a knee bends
            // one way. Shifting the window inward to preserve its width would sample angles on the
            // far side of a hard stop, which the real joint cannot reach.
            sampleLower[j] = Math.max(lower, held[j] - options.sweepHalfRangeRadians);
            sampleUpper[j] = Math.min(upper, held[j] + options.sweepHalfRangeRadians);
         }
         else
         {
            double midpoint = 0.5 * (lower + upper);
            double halfSpan = 0.5 * options.jointExcursionFraction * (upper - lower);
            sampleLower[j] = midpoint - halfSpan;
            sampleUpper[j] = midpoint + halfSpan;
         }

         if (held[j] != options.restAngle)
            notes.add(String.format("%s: rest angle %.4f rad clamped to %.4f rad, limits [%.4f, %.4f]", jointName, options.restAngle, held[j], lower, upper));
         else if (held[j] == lower || held[j] == upper)
            notes.add(String.format("%s: rest angle %.4f rad sits exactly on a limit, [%.4f, %.4f]", jointName, held[j], lower, upper));
      }

      planted.restAngleNotes = Collections.unmodifiableList(notes);

      planted.basePoses = new RigidBodyTransform[options.captureCount];
      planted.reportedJointAngles = new double[options.captureCount][model.getJointCount()];

      List<Capture> captures = new ArrayList<>(options.captureCount);

      RigidBodyTransform linkToBase = new RigidBodyTransform();
      Point3D world = new Point3D();

      for (int k = 0; k < options.captureCount; k++)
      {
         RigidBodyTransform basePose = new RigidBodyTransform();
         basePose.getTranslation().set(options.baseNominalX + options.basePositionWander * (random.nextDouble() - 0.5),
                                       options.baseNominalY + options.basePositionWander * (random.nextDouble() - 0.5),
                                       options.baseNominalZ + options.basePositionWander * (random.nextDouble() - 0.5));
         basePose.getRotation().setYawPitchRoll(options.baseOrientationWander * (random.nextDouble() - 0.5),
                                                options.baseOrientationWander * (random.nextDouble() - 0.5),
                                                options.baseOrientationWander * (random.nextDouble() - 0.5));
         planted.basePoses[k] = basePose;

         double[] reported = new double[model.getJointCount()];

         // Redraw until the legs are not crossed, when asked. A sweep about the rest pose keeps the
         // feet below the robot but says nothing about left-versus-right: ±0.45 rad of hip roll on
         // a 0.89 m leg is ±0.39 m of lateral travel against a rest stance of only 0.24 m, so the
         // ankles pass each other regularly. It is a legal configuration and a well conditioned
         // one; it just does not look like anything an operator would set up.
         //
         // Rejection rather than narrower hip limits, because crossing is a property of the two legs
         // together -- a hip roll that is fine on its own crosses once the other hip rolls the other
         // way, and no per-joint bound expresses that without throwing away most of the envelope.
         for (int attempt = 0;; attempt++)
         {
            for (int j = 0; j < model.getJointCount(); j++)
            {
               if (randomized.contains(model.getJointNames().get(j)))
                  reported[j] = sampleLower[j] + random.nextDouble() * (sampleUpper[j] - sampleLower[j]);
               else
                  reported[j] = held[j];
            }

            if (!Double.isFinite(options.minimumFootSeparation) || footSeparation(model, options, reported) >= options.minimumFootSeparation)
               break;

            planted.crossedLegResampleCount++;

            if (attempt > 500)
               throw new IllegalStateException("Could not draw an uncrossed leg configuration in 500 attempts at a foot separation of "
                     + options.minimumFootSeparation + " m. Either the separation is larger than the rest stance or the sweep is very wide.");
         }

         planted.reportedJointAngles[k] = reported.clone();

         // The fault goes in the generator, never in the model: the robot is at q_true, the encoder
         // reports q_reported, and everything downstream sees only the latter.
         double[] trueAngles = reported.clone();

         for (Map.Entry<String, Double> offset : options.jointOffsets.entrySet())
            trueAngles[model.indexOfJoint(offset.getKey())] += offset.getValue();

         model.setQ(trueAngles);
         model.updateFrames();

         MocapFrame frame = new MocapFrame(planted.markers);
         frame.setTimestampNanoseconds(k * 1_000_000L);

         for (int i = 0; i < planted.clusters.size(); i++)
         {
            MarkerCluster cluster = planted.clusters.get(i);
            ClusterLayout layout = planted.layouts.get(i);
            model.packLinkToBase(cluster.getLinkName(), linkToBase);

            for (int j = 0; j < cluster.getMarkerCount(); j++)
            {
               if (random.nextDouble() < options.occlusionProbability)
                  continue;

               world.set(layout.getPositionInLinkFrame(j));
               linkToBase.transform(world);
               basePose.transform(world);

               if (options.markerNoiseStandardDeviation > 0.0)
               {
                  world.add(options.markerNoiseStandardDeviation * random.nextGaussian(),
                            options.markerNoiseStandardDeviation * random.nextGaussian(),
                            options.markerNoiseStandardDeviation * random.nextGaussian());
               }

               frame.get(cluster.getMarker(j)).setVisible(world);
            }
         }

         EncoderSample encoders = new EncoderSample(model.getJointNames());
         encoders.setQ(reported);
         encoders.setTimestampNanoseconds(k * 1_000_000L);

         captures.add(new Capture(frame, encoders));
      }

      planted.captureSet = new CaptureSet(planted.markers, model.getJointNames(), planted.clusters, model.getBaseLinkName(), captures);

      return planted;
   }

   /**
    * {@code λ₂} of a point cloud's covariance: the mean-squared extent along its second principal
    * axis, in m².
    * <p>
    * Collinear points have {@code λ₂ = λ₃ = 0}; coplanar points have {@code λ₂ > 0} and
    * {@code λ₃ = 0}. Only the first is a problem (RUNNING.md, "Refuse on σ₂, not σ₃"), so this is
    * the quantity the rejection test is written against.
    * </p>
    */
   static double secondCovarianceEigenvalue(Point3D[] points)
   {
      double meanX = 0.0, meanY = 0.0, meanZ = 0.0;

      for (Point3D point : points)
      {
         meanX += point.getX();
         meanY += point.getY();
         meanZ += point.getZ();
      }

      meanX /= points.length;
      meanY /= points.length;
      meanZ /= points.length;

      DMatrixRMaj covariance = new DMatrixRMaj(3, 3);

      for (Point3D point : points)
      {
         double[] d = {point.getX() - meanX, point.getY() - meanY, point.getZ() - meanZ};

         for (int r = 0; r < 3; r++)
         {
            for (int c = 0; c < 3; c++)
               covariance.add(r, c, d[r] * d[c] / points.length);
         }
      }

      SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(3, 3, false, false, true);

      if (!svd.decompose(covariance))
         throw new IllegalStateException("Covariance SVD failed.");

      // EJML does not order singular values. For a symmetric PSD matrix they are the eigenvalues.
      double[] eigenvalues = svd.getSingularValues().clone();
      Arrays.sort(eigenvalues);
      return eigenvalues[1];
   }

   private static double squared(double value)
   {
      return value * value;
   }

   /** {@code ^W T_c^(k)} as a list, in the shape the gates take it. */
   public static List<us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly> clusterPoseList(BaseInitializer.GaugeTracking tracking, int captureCount)
   {
      List<us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly> poses = new ArrayList<>(captureCount);

      for (int k = 0; k < captureCount; k++)
         poses.add(tracking.isUsable(k) ? tracking.getClusterToWorld(k) : null);

      return poses;
   }
}
