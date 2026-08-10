package us.ihmc.alexMocap.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.calibration.RobotCaptures;
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.frames.GravityAlignedWorldFrame;
import us.ihmc.alexMocap.frames.TiltMeasurement;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.runtime.CenterOfMassGroundTruth;
import us.ihmc.alexMocap.runtime.KinematicChainCoupler;
import us.ihmc.alexMocap.runtime.LinkPoseEstimator;
import us.ihmc.alexMocap.runtime.MeasuredLinkPoses;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

/**
 * The {@code sim} package against the real Alex model.
 *
 * <h2>What is being demonstrated, and what is not</h2>
 * <p>
 * These tests hand {@code runtime.LinkPoseEstimator} the <b>true</b> layout rather than a calibrated
 * one. That is deliberate and it narrows the claim: what is verified here is that
 * {@link MarkerConstellation} and {@link SimulatedMocapCamera} produce observations the runtime can
 * consume, and that the resulting CoM is the CoM of the robot that generated them. It is <b>not</b>
 * evidence that a calibration recovers the layout -- that is
 * {@code calibration.AlternatingCalibratorTest}'s job, and running the two together would produce a
 * number that neither claim could be extracted from.
 * </p>
 * <p>
 * The marked set is {@code RobotCaptures.PRIMARY_MARKED_LINKS}: pelvis, both thighs, both shins,
 * both feet. That leaves 58.45 % of Alex's mass posed by forward kinematics rather than by markers
 * (CLAUDE.md), so the CoM these tests check is only as good as the encoders on a real robot -- which
 * on noiseless synthetic data is exactly as good as the URDF.
 * </p>
 */
public class SimulatedMocapTest
{
   /** Fixed, as everything in this project is. */
   private static final long SEED = 20260810L;

   private static final List<String> MARKED_LINKS = List.of(RobotCaptures.PRIMARY_MARKED_LINKS);

   /**
    * A pose source reading the model's own live frames: {@code ^W T_i} straight out of forward
    * kinematics.
    * <p>
    * This is the stand-in for what the SCS2 track supplies from the simulated robot. Reading the
    * frames rather than caching transforms is the point -- it is what makes the camera see the robot
    * move.
    * </p>
    */
   private static SimulatedMocapCamera.LinkPoseSource poseSourceOf(RobotModelHandle model)
   {
      return (linkName, poseToPack) ->
      {
         if (!model.hasLink(linkName))
            return false;

         poseToPack.set(model.getLinkFrame(linkName).getTransformToRoot());
         return true;
      };
   }

   /** A calibration carrying the planted truth, so the runtime runs on a perfect layout. */
   private static CalibrationResult perfectCalibration(MarkerConstellation constellation)
   {
      CalibrationResult calibration = new CalibrationResult();

      for (ClusterLayout layout : constellation.getTrueLayouts())
         calibration.addLayout(layout);

      return calibration;
   }

   /** Poses the robot: random legs, random floating-base placement. */
   private static void poseRobot(RobotModelHandle model, Random random)
   {
      double[] q = new double[model.getJointCount()];

      for (String jointName : RobotCaptures.LEG_JOINTS)
      {
         int index = model.indexOfJoint(jointName);
         double lower = model.getJointLimitLower(index);
         double upper = model.getJointLimitUpper(index);
         q[index] = lower + random.nextDouble() * (upper - lower);
      }

      model.setQ(q);

      JointBasics floatingJoint = model.getRootBody().getChildrenJoints().get(0);
      RigidBodyTransform basePose = new RigidBodyTransform();
      basePose.getTranslation().set(random.nextDouble(), random.nextDouble(), 0.9 + random.nextDouble());
      basePose.getRotation().setYawPitchRoll(random.nextDouble(), 0.2 * random.nextDouble(), 0.2 * random.nextDouble());
      floatingJoint.setJointConfiguration(basePose);

      model.updateFrames();
   }

   private static EncoderSample encodersOf(RobotModelHandle model)
   {
      EncoderSample encoders = new EncoderSample(model.getJointNames());

      for (int j = 0; j < model.getJointCount(); j++)
         encoders.setQ(j, model.getQ(j));

      return encoders;
   }

   // ---------------------------------------------------------------------------------------------
   // MarkerConstellation
   // ---------------------------------------------------------------------------------------------

   /**
    * The generator may not plant a cluster the runtime would refuse.
    * <p>
    * Both sides read {@code σ₂} through the same self-registration, so this is a real check of the
    * rejection threshold rather than of two implementations agreeing by luck.
    * </p>
    */
   @Test
   public void testEveryPlantedClusterClearsTheRuntimeRefusalThreshold() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      MarkerConstellation constellation = MarkerConstellation.random(model, MARKED_LINKS, SEED);

      LinkPoseEstimator estimator = new LinkPoseEstimator(perfectCalibration(constellation),
                                                          constellation.getClusters(),
                                                          model.getLinkNames());

      for (int i = 0; i < constellation.getClusters().size(); i++)
      {
         String link = constellation.getClusters().get(i).getLinkName();

         // The estimator's own threshold is a fraction of nominal σ₂, so clearing the generator's
         // absolute floor has to imply clearing that too -- otherwise the demonstration refuses
         // frames for a reason unrelated to what it is demonstrating.
         assertTrue(estimator.getNominalSigma2(i) > estimator.getSigma2Threshold(i),
                    "Cluster on '" + link + "' has nominal σ₂ = " + estimator.getNominalSigma2(i) + " m², at or below its own refusal threshold "
                          + estimator.getSigma2Threshold(i) + " m².");
      }
   }

   /** A fixed seed fixes the layout; a different one moves it. */
   @Test
   public void testDrawIsReproducibleAndSeedDependent() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();

      MarkerConstellation a = MarkerConstellation.random(model, MARKED_LINKS, SEED);
      MarkerConstellation b = MarkerConstellation.random(model, MARKED_LINKS, SEED);
      MarkerConstellation c = MarkerConstellation.random(model, MARKED_LINKS, SEED + 1);

      for (int i = 0; i < a.getTrueLayouts().size(); i++)
      {
         ClusterLayout first = a.getTrueLayouts().get(i);
         ClusterLayout same = b.getTrueLayouts().get(i);
         ClusterLayout different = c.getTrueLayouts().get(i);

         for (int j = 0; j < first.getMarkerCount(); j++)
         {
            assertEquals(first.getPositionInLinkFrame(j).getX(), same.getPositionInLinkFrame(j).getX(), 0.0, "Same seed must give the same layout.");
            assertNotEquals(first.getPositionInLinkFrame(j).getX(), different.getPositionInLinkFrame(j).getX(), "A different seed must move the markers.");
         }
      }
   }

   /** The gauge cluster is drawn wider than a limb cluster, which is the whole accuracy lever. */
   @Test
   public void testGaugeClusterIsWiderThanTheLimbClusters() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      MarkerConstellation constellation = MarkerConstellation.random(model, MARKED_LINKS, SEED);

      LinkPoseEstimator estimator = new LinkPoseEstimator(perfectCalibration(constellation),
                                                          constellation.getClusters(),
                                                          model.getLinkNames());

      double gaugeSigma2 = Double.NaN;
      double largestLimbSigma2 = 0.0;

      for (int i = 0; i < constellation.getClusters().size(); i++)
      {
         if (constellation.getClusters().get(i).getLinkName().equals(model.getBaseLinkName()))
            gaugeSigma2 = estimator.getNominalSigma2(i);
         else
            largestLimbSigma2 = Math.max(largestLimbSigma2, estimator.getNominalSigma2(i));
      }

      assertTrue(gaugeSigma2 > largestLimbSigma2,
                 "The pelvis gauge (σ₂ = " + gaugeSigma2 + " m²) must be the widest cluster; the widest limb was " + largestLimbSigma2 + " m².");
   }

   @Test
   public void testUnknownLinkIsRejectedWithTheKnownList() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();

      IllegalArgumentException thrown = assertThrows(IllegalArgumentException.class,
                                                     () -> MarkerConstellation.random(model, List.of("NOT_A_LINK"), SEED));

      assertTrue(thrown.getMessage().contains("PELVIS_LINK"), "The message should list the links that do exist, was: " + thrown.getMessage());
   }

   @Test
   public void testTooFewMarkersPerClusterIsRejected() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      assertThrows(IllegalArgumentException.class, () -> MarkerConstellation.random(model, MARKED_LINKS, SEED, 2, 0.14, 0.06));
   }

   // ---------------------------------------------------------------------------------------------
   // SimulatedMocapCamera
   // ---------------------------------------------------------------------------------------------

   /**
    * <b>The test that closes the loop for the simulated camera.</b>
    * <p>
    * Pose the robot, photograph it noiselessly, recover every link pose from the markers, chain the
    * unmarked links through the encoders, sum the CoM -- then compare against Mecano's
    * {@code CenterOfMassCalculator} on the same configuration. The two paths share no code: one goes
    * link frames → marker positions → Umeyama → weighted sum, the other goes joint angles → Mecano's
    * own recursion. This is the check that the SCS2 track's numbers mean what they say.
    * </p>
    */
   @Test
   public void testNoiselessCameraReproducesMecanosCenterOfMass() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      MarkerConstellation constellation = MarkerConstellation.random(model, MARKED_LINKS, SEED);
      SimulatedMocapCamera camera = new SimulatedMocapCamera(constellation, 0.0, 0.0, SEED);

      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("test"),
                                                                    ReferenceFrame.getWorldFrame(),
                                                                    "_simCom");

      LinkPoseEstimator estimator = new LinkPoseEstimator(perfectCalibration(constellation), constellation.getClusters(), model.getLinkNames());
      KinematicChainCoupler coupler = new KinematicChainCoupler(model, model.getLinkNames(), constellation.getMarkedLinks());
      CenterOfMassGroundTruth groundTruth = CenterOfMassGroundTruth.forWholeRobot(model, world.getMotiveWorld(), world.getGravityAlignedWorld());

      CenterOfMassCalculator oracle = new CenterOfMassCalculator(model.getRootBody(), ReferenceFrame.getWorldFrame());

      MocapFrame frame = camera.newFrame();
      MeasuredLinkPoses poses = new MeasuredLinkPoses(model.getLinkNames());
      Point3D measured = new Point3D();
      Random random = new Random(SEED);

      double worstError = 0.0;

      for (int tick = 0; tick < 40; tick++)
      {
         poseRobot(model, random);

         camera.observe(poseSourceOf(model), frame);
         assertEquals(0, camera.getLastMissingLinkCount(), "Every marked link must have a pose; a miss here is a link-name fault.");
         assertEquals(constellation.getMarkers().size(), camera.getLastVisibleCount(), "A noiseless, unoccluded camera must see every marker.");

         estimator.estimate(frame, poses);
         coupler.complete(encodersOf(model), poses);

         assertTrue(groundTruth.compute(poses, measured), "Every link should be measurable on noiseless data.");

         oracle.reset();

         worstError = Math.max(worstError, measured.distance(oracle.getCenterOfMass()));
      }

      // Measured 2.3e-16 m over these 40 poses. The threshold is loose against that and still far
      // below anything physical -- a millimetre here would be a real disagreement.
      assertTrue(worstError < 1.0e-9, "Worst CoM disagreement with Mecano was " + worstError + " m; the two paths must agree to round-off.");
   }

   /** Noiseless observations must return the exact pose that generated them. */
   @Test
   public void testNoiselessCameraRecoversTheLinkPosesExactly() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      MarkerConstellation constellation = MarkerConstellation.random(model, MARKED_LINKS, SEED);
      SimulatedMocapCamera camera = new SimulatedMocapCamera(constellation, 0.0, 0.0, SEED);

      LinkPoseEstimator estimator = new LinkPoseEstimator(perfectCalibration(constellation), constellation.getClusters(), model.getLinkNames());

      MocapFrame frame = camera.newFrame();
      MeasuredLinkPoses poses = new MeasuredLinkPoses(model.getLinkNames());
      Random random = new Random(SEED);
      RigidBodyTransform truth = new RigidBodyTransform();
      Point3D truthOrigin = new Point3D();
      Point3D estimatedOrigin = new Point3D();

      double worstTranslation = 0.0;

      for (int tick = 0; tick < 20; tick++)
      {
         poseRobot(model, random);
         camera.observe(poseSourceOf(model), frame);
         estimator.estimate(frame, poses);

         for (String link : constellation.getMarkedLinks())
         {
            assertTrue(poses.isAvailable(poses.indexOf(link)), "'" + link + "' should be measured on a clean frame.");
            truth.set(model.getLinkFrame(link).getTransformToRoot());
            truthOrigin.set(truth.getTranslation());
            estimatedOrigin.set(poses.getPose(link).getTranslation());
            worstTranslation = Math.max(worstTranslation, truthOrigin.distance(estimatedOrigin));
         }
      }

      // Measured 8.9e-16 m. Umeyama on exact, non-degenerate correspondences is exact to round-off.
      assertTrue(worstTranslation < 1.0e-9, "Worst link-origin disagreement was " + worstTranslation + " m.");
   }

   /**
    * Noise reaches the CoM, and does so at the scale the error budget predicts.
    * <p>
    * The assertion is a band, not a point. A test that only required the error to be "small" would
    * pass with the noise wired to nothing, which is the failure this project keeps finding: a green
    * number with no conditioning attached.
    * </p>
    */
   @Test
   public void testMarkerNoiseReachesTheCenterOfMass() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      MarkerConstellation constellation = MarkerConstellation.random(model, MARKED_LINKS, SEED);

      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("test"),
                                                                    ReferenceFrame.getWorldFrame(),
                                                                    "_simNoise");

      LinkPoseEstimator estimator = new LinkPoseEstimator(perfectCalibration(constellation), constellation.getClusters(), model.getLinkNames());
      KinematicChainCoupler coupler = new KinematicChainCoupler(model, model.getLinkNames(), constellation.getMarkedLinks());
      CenterOfMassGroundTruth groundTruth = CenterOfMassGroundTruth.forWholeRobot(model, world.getMotiveWorld(), world.getGravityAlignedWorld());
      CenterOfMassCalculator oracle = new CenterOfMassCalculator(model.getRootBody(), ReferenceFrame.getWorldFrame());

      SimulatedMocapCamera camera = new SimulatedMocapCamera(constellation, SimulatedMocapCamera.GANTRY_NOISE_STANDARD_DEVIATION, 0.0, SEED);

      MocapFrame frame = camera.newFrame();
      MeasuredLinkPoses poses = new MeasuredLinkPoses(model.getLinkNames());
      Point3D measured = new Point3D();
      Random random = new Random(SEED);

      double sumSquared = 0.0;
      int ticks = 60;

      for (int tick = 0; tick < ticks; tick++)
      {
         poseRobot(model, random);
         camera.observe(poseSourceOf(model), frame);
         estimator.estimate(frame, poses);
         coupler.complete(encodersOf(model), poses);
         assertTrue(groundTruth.compute(poses, measured));

         oracle.reset();
         sumSquared += measured.distanceSquared(oracle.getCenterOfMass());
      }

      double rms = Math.sqrt(sumSquared / ticks);

      // Measured 0.55 mm RMS at σ = 0.3 mm with this seed and marked set. The band is deliberately
      // wide: what it excludes is noise that never arrives (rms → 0, the wiring fault) and noise
      // amplified out of all proportion (rms > 5 mm, a conditioning fault).
      assertTrue(rms > 0.05e-3, "CoM RMS was " + (1000.0 * rms) + " mm -- too small for σ = 0.3 mm noise; is the noise wired in?");
      assertTrue(rms < 5.0e-3, "CoM RMS was " + (1000.0 * rms) + " mm at σ = 0.3 mm, which is a conditioning fault, not measurement noise.");
   }

   /**
    * An occluded cluster is refused, and the CoM goes NaN rather than becoming the CoM of a lighter
    * robot.
    * <p>
    * This is the failure mode the project cares most about: a plausible number with a wrong answer.
    * Dropping a shin silently would move the CoM by centimetres and report success.
    * </p>
    */
   @Test
   public void testOccludingAClusterRefusesRatherThanReturningALighterRobot() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      MarkerConstellation constellation = MarkerConstellation.random(model, MARKED_LINKS, SEED);

      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("test"),
                                                                    ReferenceFrame.getWorldFrame(),
                                                                    "_simOcclusion");

      LinkPoseEstimator estimator = new LinkPoseEstimator(perfectCalibration(constellation), constellation.getClusters(), model.getLinkNames());
      KinematicChainCoupler coupler = new KinematicChainCoupler(model, model.getLinkNames(), constellation.getMarkedLinks());
      CenterOfMassGroundTruth groundTruth = CenterOfMassGroundTruth.forWholeRobot(model, world.getMotiveWorld(), world.getGravityAlignedWorld());

      SimulatedMocapCamera camera = new SimulatedMocapCamera(constellation, 0.0, 0.0, SEED);
      MocapFrame frame = camera.newFrame();
      MeasuredLinkPoses poses = new MeasuredLinkPoses(model.getLinkNames());
      Point3D measured = new Point3D();

      poseRobot(model, new Random(SEED));

      // Hide a whole cluster by refusing its link's pose: two markers left is below
      // MarkerCluster.MINIMUM_MARKERS, so the cluster cannot produce a pose at all.
      String hidden = "LEFT_SHIN";
      camera.observe((linkName, poseToPack) ->
      {
         if (linkName.equals(hidden))
            return false;

         poseToPack.set(model.getLinkFrame(linkName).getTransformToRoot());
         return true;
      }, frame);

      assertEquals(1, camera.getLastMissingLinkCount(), "Exactly one link should have been withheld.");

      estimator.estimate(frame, poses);
      coupler.complete(encodersOf(model), poses);

      // KinematicChainCoupler fills the links that carry no cluster. LEFT_SHIN carries one, so the
      // coupler deliberately does not substitute FK for it: a link the operator chose to measure is
      // either measured or it is missing. That is the conservative branch and it is the one that
      // matters -- the alternative, quietly chaining a marked link off its parent, would hide a
      // camera that had stopped seeing a limb behind a plausible-looking CoM.
      int shinIndex = poses.indexOf(hidden);
      assertFalse(poses.isAvailable(shinIndex), "A marked link that was not seen must not be silently substituted.");

      // The refusal propagates downward. KinematicChainCoupler binds each unmarked link to its
      // nearest marked ancestor once, at construction, so LEFT_ANKLE_Y_LINK is chained off
      // LEFT_SHIN for the life of the run; with the shin refused it has nothing to hang from and is
      // refused too, naming the ancestor. That is worth pinning down: the operator marked seven
      // links, but losing one of them costs the mass of that link *and everything unmarked beneath
      // it*, which is not obvious from the marked set alone.
      String orphan = "LEFT_ANKLE_Y_LINK";
      int orphanIndex = poses.indexOf(orphan);
      assertFalse(poses.isAvailable(orphanIndex), "An unmarked link chained off a refused link cannot have a pose either.");
      assertTrue(poses.getRefusalReason(orphanIndex).contains(hidden), "The refusal should name the ancestor, was: " + poses.getRefusalReason(orphanIndex));

      // 6.39 kg leaves the sum, so there is no CoM for this frame. NaN, not the CoM of a robot
      // missing a shin -- which would sit centimetres away and report success.
      assertFalse(groundTruth.compute(poses, measured), "A refused marked link must refuse the CoM.");
      assertTrue(measured.containsNaN(), "The refused CoM must be NaN, never a partial sum.");
      assertEquals(2, groundTruth.getMissingLinkCount(), "The shin and the ankle link it carries.");
      assertEquals(model.getMass(hidden) + model.getMass(orphan),
                   groundTruth.getMissingMass(),
                   1.0e-12,
                   "The reported missing mass must be the shin's plus its orphaned descendant's.");

      // And the same frame with the shin visible is fine, so the refusal is about the occlusion and
      // nothing else.
      camera.observe(poseSourceOf(model), frame);
      estimator.estimate(frame, poses);
      coupler.complete(encodersOf(model), poses);
      assertTrue(groundTruth.compute(poses, measured), "With every cluster seen, the same pose must yield a CoM.");
      assertFalse(measured.containsNaN());
   }

   /** Every marker in the frame belongs to the constellation, and the dense-set contract holds. */
   @Test
   public void testFrameCarriesExactlyTheConstellationsMarkers() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      MarkerConstellation constellation = MarkerConstellation.random(model, MARKED_LINKS, SEED);
      SimulatedMocapCamera camera = new SimulatedMocapCamera(constellation, 0.0, 0.0, SEED);

      MarkerId.checkDenseSet(constellation.getMarkers());

      assertEquals(MARKED_LINKS.size() * MarkerConstellation.DEFAULT_MARKERS_PER_CLUSTER, constellation.getMarkers().size());
      assertEquals(MARKED_LINKS.size(), constellation.getClusters().size());
      assertEquals(MARKED_LINKS, constellation.getMarkedLinks());

      MocapFrame frame = camera.newFrame();
      assertEquals(constellation.getMarkers().size(), frame.getMarkerCount());

      List<String> clusterLinks = new ArrayList<>();

      for (MarkerCluster cluster : constellation.getClusters())
         clusterLinks.add(cluster.getLinkName());

      assertEquals(new ArrayList<>(Arrays.asList(RobotCaptures.PRIMARY_MARKED_LINKS)), clusterLinks);
   }

   /** A frame from another camera is rejected rather than partially filled. */
   @Test
   public void testForeignFrameIsRejected() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      MarkerConstellation constellation = MarkerConstellation.random(model, MARKED_LINKS, SEED);
      SimulatedMocapCamera camera = new SimulatedMocapCamera(constellation, 0.0, 0.0, SEED);

      MocapFrame foreign = new MocapFrame(MarkerId.createDenseSet("A", "B", "C"));

      assertThrows(IllegalArgumentException.class, () -> camera.observe(poseSourceOf(model), foreign));
   }
}
