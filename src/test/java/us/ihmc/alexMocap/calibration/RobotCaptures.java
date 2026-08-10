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

      /** Base pose wander between captures: a suspended robot is not perfectly still. */
      public double basePositionWander = 0.02;
      public double baseOrientationWander = Math.toRadians(2.0);

      /** Full spread of the gauge cluster, metres. FRAMEWORK.md §1's outrigger bracket. */
      public double gaugeClusterSpread = 0.14;

      /** Full spread of the limb clusters, metres. Constrained by the width of a real limb. */
      public double limbClusterSpread = 0.06;

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

         model.packCenterOfMassInLinkFrame(cluster.getLinkName(), centerOfMass);

         Point3D[] constellation = new Point3D[cluster.getMarkerCount()];

         // Redraw until the constellation is not near-collinear. The cap is generous: at these
         // spreads a rejection is rare, and a loop that cannot terminate is worse than a throw.
         for (int attempt = 0;; attempt++)
         {
            for (int j = 0; j < constellation.length; j++)
            {
               constellation[j] = new Point3D(centerOfMass.getX() + CLUSTER_CENTROID_OFFSET.getX() + spread * (random.nextDouble() - 0.5),
                                              centerOfMass.getY() + CLUSTER_CENTROID_OFFSET.getY() + spread * (random.nextDouble() - 0.5),
                                              centerOfMass.getZ() + CLUSTER_CENTROID_OFFSET.getZ() + spread * (random.nextDouble() - 0.5));
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
         basePose.getTranslation().set(1.0 + options.basePositionWander * (random.nextDouble() - 0.5),
                                       2.0 + options.basePositionWander * (random.nextDouble() - 0.5),
                                       1.4 + options.basePositionWander * (random.nextDouble() - 0.5));
         basePose.getRotation().setYawPitchRoll(options.baseOrientationWander * (random.nextDouble() - 0.5),
                                                options.baseOrientationWander * (random.nextDouble() - 0.5),
                                                options.baseOrientationWander * (random.nextDouble() - 0.5));
         planted.basePoses[k] = basePose;

         double[] reported = new double[model.getJointCount()];

         for (int j = 0; j < model.getJointCount(); j++)
         {
            if (randomized.contains(model.getJointNames().get(j)))
               reported[j] = sampleLower[j] + random.nextDouble() * (sampleUpper[j] - sampleLower[j]);
            else
               reported[j] = held[j];
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
