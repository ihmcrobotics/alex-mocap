package us.ihmc.alexMocap.calibration;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import us.ihmc.alexMocap.core.Capture;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Generates calibration captures from known truth, so that a solver can be asked to recover
 * something the test already knows.
 *
 * <h2>The fault goes in the generator, never in the model</h2>
 * <p>
 * This is what makes the G2 diagnostic tests mean anything (PR_PLAN.md, "the diagnostic tests").
 * The <b>robot</b> is at {@code q_true}; the <b>encoder</b> reports {@code q_reported}; the capture
 * stores {@code q_reported} and so does everything downstream. A joint offset or an elastic
 * deflection is the difference between them. The calibrator is therefore wrong about the world in
 * exactly the way a real robot is wrong, rather than being handed a corrupted input it could in
 * principle detect.
 * </p>
 *
 * <h2>What "recovered" can and cannot mean</h2>
 * <p>
 * The planted layouts are recoverable exactly: {@code ^i p_ij} lives in a link frame and does not
 * depend on any convention the solver chooses.
 * </p>
 * <p>
 * <b>The planted {@code Δ} is not.</b> {@code Δ = ^c T_b} is defined against the marker-cluster
 * frame {@code c}, and {@code c}'s convention is arbitrary -- {@link BaseInitializer} fixes it by
 * declaring the gauge markers' raw positions at one reference capture to be the cluster's shape.
 * A generator that plants its own {@code c} will disagree with that choice by a constant, and
 * asserting {@code Δ̂ == Δ*} would be asserting that the solver guessed the generator's convention.
 * </p>
 * <p>
 * What is convention-free, and what the tests assert instead, is the <b>base pose</b>:
 * {@code ^W T_c^(k) · Δ̂} must equal the planted {@code ^W T_b^(k)} for every capture. That is the
 * physically meaningful statement, and it is strictly stronger than a claim about {@code Δ} alone
 * because it has to hold at every {@code k}.
 * </p>
 */
public class SyntheticCaptures
{
   /** Where the toy URDF lives, resolved from the test classpath. */
   public static Path toyUrdfPath() throws Exception
   {
      return Path.of(SyntheticCaptures.class.getResource("/us/ihmc/alexMocap/model/toy6dof.urdf").toURI());
   }

   public static RobotModelHandle toyModel() throws Exception
   {
      return RobotModelHandle.fromURDF(toyUrdfPath());
   }

   /** Knobs. Defaults are the clean, noiseless case. */
   public static class Options
   {
      public long seed = 20260809L;
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

      /**
       * Elastic compliance, rad/(N·m), applied to every joint in proportion to the static
       * gravitational torque it carries. This is FRAMEWORK.md §21.2's "the encoder q is not the
       * joint angle under load", and unlike a constant offset it varies with configuration.
       */
      public double complianceRadiansPerNewtonMeter = 0.0;

      /** Base pose wander between captures: a suspended robot is not perfectly still. */
      public double basePositionWander = 0.02;
      public double baseOrientationWander = Math.toRadians(2.0);

      /**
       * Hold every capture at the same joint configuration.
       * <p>
       * This is FRAMEWORK.md §7's warning made reproducible: with the legs frozen, the marked links
       * below the pelvis say nothing new from capture to capture, and {@code Δ} stops being
       * identified even though A′ still converges and still reports a small {@code J}.
       * </p>
       */
      public boolean frozenJoints = false;

      /** Which links carry clusters. Defaults to {@link SyntheticCaptures#MARKED_LINKS}. */
      public String[] markedLinks = null;

      /**
       * Full spread of the gauge (pelvis) cluster, metres.
       * <p>
       * FRAMEWORK.md §1 asks for an outrigger bracket taking this to 120-150 mm, because cluster
       * angular accuracy scales as {@code σ / (√N · r_perp)}. That requirement is not cosmetic:
       * the gauge cluster's angular error is multiplied by the lever arm out to every other link,
       * and it is the <b>dominant</b> term in layout accuracy -- far above the {@code σ/√K} that
       * an isolated reading of §6 suggests.
       * </p>
       */
      public double gaugeClusterSpread = 0.14;

      /** Full spread of the limb clusters, metres. Constrained by the width of a real limb. */
      public double limbClusterSpread = 0.06;

      public Options gaugeSpread(double metres)
      {
         this.gaugeClusterSpread = metres;
         return this;
      }

      /**
       * Mark only the links whose orientation is a pure {@code y} rotation, leaving the
       * translational gauge freedom along {@code y} open. See {@link SyntheticCaptures#MARKED_LINKS}.
       */
      public Options degenerateMarkerSet()
      {
         this.markedLinks = DEGENERATE_MARKED_LINKS;
         return this;
      }

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

      public Options compliance(double radiansPerNewtonMeter)
      {
         this.complianceRadiansPerNewtonMeter = radiansPerNewtonMeter;
         return this;
      }

      public Options rigidBase()
      {
         this.basePositionWander = 0.0;
         this.baseOrientationWander = 0.0;
         return this;
      }

      public Options frozenJoints()
      {
         this.frozenJoints = true;
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

   /** Joint limits from the toy URDF, so generated poses are physically reachable. */
   private static final Map<String, double[]> JOINT_LIMITS = new LinkedHashMap<>();

   static
   {
      JOINT_LIMITS.put("l_hip", new double[] {-1.6, 1.6});
      JOINT_LIMITS.put("l_knee", new double[] {-0.05, 2.4});
      JOINT_LIMITS.put("l_ankle", new double[] {-0.7, 0.7});
      JOINT_LIMITS.put("r_hip", new double[] {-1.6, 1.6});
      JOINT_LIMITS.put("r_knee", new double[] {-0.05, 2.4});
      JOINT_LIMITS.put("r_ankle", new double[] {-0.7, 0.7});
   }

   /**
    * Which links carry markers, and how many each gets.
    * <p>
    * FRAMEWORK.md §1: the pelvis is the gauge, and the rest are marked in descending order of
    * {@code mass × lever arm}. Four markers each, which is §1's recommended minimum -- three for
    * the pose and a fourth so G1 has a redundant distance to check.
    * </p>
    *
    * <h2>The feet must be marked, and the reason is identifiability</h2>
    * <p>
    * The obvious choice is to leave the feet out: they are the lightest links, F7 covers them by
    * chaining, and omitting them keeps the toy honest about not every link having a cluster. That
    * choice makes the calibration <b>exactly unidentifiable</b>, and it took a perfect fit with a
    * 13 mm layout error to notice.
    * </p>
    * <p>
    * Replacing {@code Δ} by {@code Δ·G} for a pure translation {@code g} shifts each layout by
    * {@code R_i(q)ᵀ g}. That is a genuine symmetry -- same {@code J}, different answer -- exactly
    * when {@code R_i(q)ᵀ g} is the same at every capture, i.e. when {@code g} lies along an axis
    * every marked link merely rotates about. With only the pelvis, thighs and shanks marked, every
    * marked link's orientation is a rotation about {@code y} alone (the hips and knees are both
    * pitch), so {@code g} along {@code y} is free. A′ converges to {@code J = 7.6e-29} -- machine
    * zero -- and lands 13 mm from the truth, with every cluster displaced by the same vector.
    * </p>
    * <p>
    * The feet are the only links whose orientation involves the ankle's {@code x} axis, so marking
    * them is what breaks the symmetry. This is the toy-scale version of FRAMEWORK.md §1's
    * requirement for wide, <i>varied</i> joint excursion -- and a reminder that the failure mode is
    * a small residual and a wrong answer, not a large residual.
    * </p>
    * <p>
    * {@code CalibrationDegeneracyTest} plants this failure deliberately and pins its behaviour,
    * including the part that matters most: {@code σ₃} does <b>not</b> detect it.
    * </p>
    */
   private static final String[] MARKED_LINKS = {"pelvis", "l_thigh", "l_shank", "l_foot", "r_thigh", "r_shank", "r_foot"};

   /** The marked set that leaves the translational gauge freedom open. See {@link #MARKED_LINKS}. */
   static final String[] DEGENERATE_MARKED_LINKS = {"pelvis", "l_thigh", "l_shank", "r_thigh", "r_shank"};
   private static final int MARKERS_PER_CLUSTER = 4;

   public static Planted generate(Options options) throws Exception
   {
      Random random = new Random(options.seed);
      Planted planted = new Planted();

      planted.model = toyModel();
      RobotModelHandle model = planted.model;

      String[] markedLinks = options.markedLinks == null ? MARKED_LINKS : options.markedLinks;
      List<String> markerNames = new ArrayList<>();

      for (String link : markedLinks)
      {
         for (int j = 0; j < MARKERS_PER_CLUSTER; j++)
            markerNames.add(link + "_M" + j);
      }

      planted.markers = MarkerId.createDenseSet(markerNames);
      planted.clusters = new ArrayList<>();

      int markerIndex = 0;

      for (String link : markedLinks)
      {
         List<MarkerId> members = new ArrayList<>();

         for (int j = 0; j < MARKERS_PER_CLUSTER; j++)
            members.add(planted.markers.get(markerIndex++));

         planted.clusters.add(new MarkerCluster(link, members));
      }

      // Plant the layouts: a well-spread, non-collinear, asymmetric constellation per link.
      planted.layouts = new ArrayList<>();

      for (MarkerCluster cluster : planted.clusters)
      {
         ClusterLayout layout = new ClusterLayout(cluster);

         // The gauge cluster gets the outrigger bracket of FRAMEWORK.md §1; the limb clusters are
         // stuck with the width of a limb.
         boolean isGauge = cluster.getLinkName().equals("pelvis");
         double spread = isGauge ? options.gaugeClusterSpread : options.limbClusterSpread;

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            // Offset from the link origin so the cluster is not centred on it: a centred cluster
            // would hide a whole class of sign error.
            Point3D position = new Point3D(spread * (random.nextDouble() - 0.5) + 0.03,
                                           spread * (random.nextDouble() - 0.5) - 0.02,
                                           spread * (random.nextDouble() - 0.5) - 0.05);
            layout.setPositionInLinkFrame(j, position, 1);
         }

         planted.layouts.add(layout);
      }

      planted.basePoses = new RigidBodyTransform[options.captureCount];
      planted.reportedJointAngles = new double[options.captureCount][model.getJointCount()];

      List<Capture> captures = new ArrayList<>(options.captureCount);

      RigidBodyTransform linkToBase = new RigidBodyTransform();
      Point3D world = new Point3D();

      for (int k = 0; k < options.captureCount; k++)
      {
         // A suspended robot drifts a little between captures; the gauge cluster is what tracks it.
         RigidBodyTransform basePose = new RigidBodyTransform();
         basePose.getTranslation().set(1.0 + options.basePositionWander * (random.nextDouble() - 0.5),
                                       2.0 + options.basePositionWander * (random.nextDouble() - 0.5),
                                       1.4 + options.basePositionWander * (random.nextDouble() - 0.5));
         basePose.getRotation().setYawPitchRoll(options.baseOrientationWander * (random.nextDouble() - 0.5),
                                                options.baseOrientationWander * (random.nextDouble() - 0.5),
                                                options.baseOrientationWander * (random.nextDouble() - 0.5));
         planted.basePoses[k] = basePose;

         // Wide joint excursion: FRAMEWORK.md §1 needs it for identifiability, and a narrow sweep
         // is the failure mode where the calibration looks converged and means nothing.
         double[] reported = new double[model.getJointCount()];

         if (options.frozenJoints && k > 0)
         {
            reported = planted.reportedJointAngles[0].clone();
         }
         else
         {
            for (int j = 0; j < model.getJointCount(); j++)
            {
               double[] limits = JOINT_LIMITS.get(model.getJointNames().get(j));
               reported[j] = limits[0] + random.nextDouble() * (limits[1] - limits[0]);
            }
         }

         planted.reportedJointAngles[k] = reported.clone();

         double[] trueAngles = applyInjectedFaults(model, reported, options);

         // FK at the TRUE angles: this is where the markers physically are.
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

         // The encoder reports the REPORTED angles, faults and all.
         EncoderSample encoders = new EncoderSample(model.getJointNames());
         encoders.setQ(reported);
         encoders.setTimestampNanoseconds(k * 1_000_000L);

         captures.add(new Capture(frame, encoders));
      }

      planted.captureSet = new CaptureSet(planted.markers, model.getJointNames(), planted.clusters, "pelvis", captures);

      return planted;
   }

   /**
    * {@code q_true = q_reported + constant offset + compliance × gravitational torque}.
    * <p>
    * The elastic term uses the torque evaluated at {@code q_reported} rather than solving the
    * implicit equation in {@code q_true}. At the deflections of interest -- a few tenths of a
    * degree -- the difference is second order, and the point of the injection is to produce a
    * load-correlated error, not to model a specific drivetrain.
    * </p>
    */
   private static double[] applyInjectedFaults(RobotModelHandle model, double[] reported, Options options)
   {
      double[] trueAngles = reported.clone();

      for (Map.Entry<String, Double> offset : options.jointOffsets.entrySet())
         trueAngles[model.indexOfJoint(offset.getKey())] += offset.getValue();

      if (options.complianceRadiansPerNewtonMeter != 0.0)
      {
         model.setQ(reported);
         model.updateFrames();

         for (int j = 0; j < model.getJointCount(); j++)
            trueAngles[j] += options.complianceRadiansPerNewtonMeter * gravitationalTorque(model, model.getJointNames().get(j));
      }

      return trueAngles;
   }

   /**
    * Which links hang below each joint of the toy robot. Hardcoded because the toy's structure is
    * fixed and known, and because a generic subtree walk here would be a second implementation of
    * something Mecano already does -- with the test then depending on it being right.
    */
   private static final Map<String, String[]> SUBTREE = new LinkedHashMap<>();

   static
   {
      SUBTREE.put("l_hip", new String[] {"l_thigh", "l_shank", "l_foot"});
      SUBTREE.put("l_knee", new String[] {"l_shank", "l_foot"});
      SUBTREE.put("l_ankle", new String[] {"l_foot"});
      SUBTREE.put("r_hip", new String[] {"r_thigh", "r_shank", "r_foot"});
      SUBTREE.put("r_knee", new String[] {"r_shank", "r_foot"});
      SUBTREE.put("r_ankle", new String[] {"r_foot"});
   }

   /** Joint axes in their own child-link frames, from the toy URDF. */
   private static final Map<String, Vector3D> JOINT_AXES = new LinkedHashMap<>();

   static
   {
      JOINT_AXES.put("l_hip", new Vector3D(1, 0, 0));
      JOINT_AXES.put("l_knee", new Vector3D(0, 1, 0));
      JOINT_AXES.put("l_ankle", new Vector3D(0, 1, 0));
      JOINT_AXES.put("r_hip", new Vector3D(1, 0, 0));
      JOINT_AXES.put("r_knee", new Vector3D(0, 1, 0));
      JOINT_AXES.put("r_ankle", new Vector3D(0, 1, 0));
   }

   private static final String[] CHILD_LINK = {"l_thigh", "l_shank", "l_foot", "r_thigh", "r_shank", "r_foot"};
   private static final String[] JOINT_ORDER = {"l_hip", "l_knee", "l_ankle", "r_hip", "r_knee", "r_ankle"};

   /**
    * Static gravitational torque about one joint's axis, N·m:
    * {@code τ_j = Σ_{i below j} ( (r_i - p_j) × m_i g ) · a_j}.
    * <p>
    * The base is treated as upright, so gravity is {@code (0, 0, -9.81)} in base coordinates. For a
    * robot hanging from a gantry that is true to within the same couple of degrees the base pose
    * wanders, which is far inside what this proxy needs to be right about.
    * </p>
    */
   private static double gravitationalTorque(RobotModelHandle model, String jointName)
   {
      String childLink = CHILD_LINK[indexOf(JOINT_ORDER, jointName)];

      RigidBodyTransform childToBase = new RigidBodyTransform();
      model.packLinkToBase(childLink, childToBase);

      Point3D jointOrigin = new Point3D(childToBase.getTranslation());

      Vector3D axis = new Vector3D(JOINT_AXES.get(jointName));
      childToBase.getRotation().transform(axis);

      Vector3D gravity = new Vector3D(0.0, 0.0, -9.81);
      Vector3D torque = new Vector3D();
      Vector3D lever = new Vector3D();
      Vector3D force = new Vector3D();
      Point3D centerOfMass = new Point3D();
      RigidBodyTransform linkToBase = new RigidBodyTransform();
      double total = 0.0;

      for (String link : SUBTREE.get(jointName))
      {
         model.packCenterOfMassInLinkFrame(link, centerOfMass);
         model.packLinkToBase(link, linkToBase);
         linkToBase.transform(centerOfMass);

         lever.sub(centerOfMass, jointOrigin);
         force.setAndScale(model.getMass(link), gravity);
         torque.cross(lever, force);

         total += torque.dot(axis);
      }

      return total;
   }

   private static int indexOf(String[] array, String value)
   {
      for (int i = 0; i < array.length; i++)
      {
         if (array[i].equals(value))
            return i;
      }

      throw new IllegalArgumentException("Unknown joint '" + value + "'.");
   }

   /** Replaces one cluster's measurements with noiseless ones, for isolating error sources. */
   public static void denoiseCluster(Planted planted, String linkName) throws Exception
   {
      RobotModelHandle model = planted.model;
      RigidBodyTransform linkToBase = new RigidBodyTransform();
      Point3D world = new Point3D();

      for (int i = 0; i < planted.clusters.size(); i++)
      {
         MarkerCluster cluster = planted.clusters.get(i);
         if (!cluster.getLinkName().equals(linkName))
            continue;

         ClusterLayout layout = planted.layouts.get(i);

         for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
         {
            model.setQ(planted.reportedJointAngles[k]);
            model.updateFrames();
            model.packLinkToBase(linkName, linkToBase);

            for (int j = 0; j < cluster.getMarkerCount(); j++)
            {
               if (!planted.captureSet.getCapture(k).getMocapFrame().get(cluster.getMarker(j)).isVisible())
                  continue;
               world.set(layout.getPositionInLinkFrame(j));
               linkToBase.transform(world);
               planted.basePoses[k].transform(world);
               planted.captureSet.getCapture(k).getMocapFrame().get(cluster.getMarker(j)).setVisible(world);
            }
         }
      }
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
