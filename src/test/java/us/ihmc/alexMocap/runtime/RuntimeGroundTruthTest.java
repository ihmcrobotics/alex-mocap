package us.ihmc.alexMocap.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.calibration.AlternatingCalibrator;
import us.ihmc.alexMocap.calibration.BaseInitializer;
import us.ihmc.alexMocap.calibration.BaseInitializer.GaugeTracking;
import us.ihmc.alexMocap.calibration.CalibrationReport;
import us.ihmc.alexMocap.calibration.SyntheticCaptures;
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.frames.GravityAlignedWorldFrame;
import us.ihmc.alexMocap.frames.TiltMeasurement;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

/**
 * F6, F7, F8 and F9 end to end: from a calibration, through per-frame registration, to a CoM.
 * <p>
 * These are the tests that check the pipeline actually closes. Everything upstream was verified
 * against planted truth; here the output is verified against an <b>independent oracle</b> --
 * Mecano's own {@code CenterOfMassCalculator} -- on data where the two must agree exactly.
 * </p>
 */
public class RuntimeGroundTruthTest
{
   private static final double EPSILON = 1.0e-9;

   /** A calibrated pipeline over noiseless synthetic captures. */
   private record Fixture(SyntheticCaptures.Planted planted, GaugeTracking tracking, CalibrationResult calibration, RobotModelHandle model,
         GravityAlignedWorldFrame world, List<String> linkNames, List<String> markedLinks)
   {
   }

   private static Fixture fixture(SyntheticCaptures.Options options, String nameSuffix, TiltMeasurement tilt) throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(options);
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);
      CalibrationResult calibration = new AlternatingCalibrator().calibrate(planted.captureSet, planted.model, tracking, new CalibrationReport());

      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(tilt, ReferenceFrame.getWorldFrame(), nameSuffix);

      List<String> markedLinks = new ArrayList<>();

      for (MarkerCluster cluster : planted.clusters)
         markedLinks.add(cluster.getLinkName());

      return new Fixture(planted, tracking, calibration, planted.model, world, planted.model.getLinkNames(), markedLinks);
   }

   private static Fixture noiselessFixture(String nameSuffix) throws Exception
   {
      return fixture(new SyntheticCaptures.Options().captures(8).noise(0.0), nameSuffix, TiltMeasurement.assumedLevel("test"));
   }

   private static MeasuredLinkPoses estimateFrame(Fixture fixture, int capture)
   {
      LinkPoseEstimator estimator = new LinkPoseEstimator(fixture.calibration(), fixture.planted().clusters, fixture.linkNames());
      MeasuredLinkPoses poses = new MeasuredLinkPoses(fixture.linkNames());
      estimator.estimate(fixture.planted().captureSet.getCapture(capture).getMocapFrame(), poses);
      return poses;
   }

   /**
    * <b>The test that closes the loop.</b>
    * <p>
    * Feed F6 exact poses generated from FK, compute the CoM, and compare against Mecano's
    * {@code CenterOfMassCalculator} on the same configuration. The two computations share no code:
    * one goes marker positions → Umeyama → per-link poses → weighted sum, the other goes joint
    * angles → FK → Mecano's own recursion. Agreement to {@code 1e-9} means the whole chain from
    * F1 through F6 to F9 is consistent.
    * </p>
    */
   @Test
   public void testCenterOfMassMatchesMecanoOnFkConsistentPoses() throws Exception
   {
      Fixture fixture = noiselessFixture("_comOracle");
      CenterOfMassGroundTruth groundTruth = CenterOfMassGroundTruth.forWholeRobot(fixture.model(),
                                                                                 fixture.world().getMotiveWorld(),
                                                                                 fixture.world().getGravityAlignedWorld());

      // Mecano's oracle: the same robot, posed by joint angles, with the floating joint set to the
      // planted base pose so the tree's frames coincide with the measured world poses.
      CenterOfMassCalculator oracle = new CenterOfMassCalculator(fixture.model().getRootBody(), ReferenceFrame.getWorldFrame());
      JointBasics floatingJoint = fixture.model().getRootBody().getChildrenJoints().get(0);

      Point3D measured = new Point3D();

      for (int capture = 0; capture < fixture.planted().captureSet.getCaptureCount(); capture++)
      {
         MeasuredLinkPoses poses = estimateFrame(fixture, capture);
         assertTrue(groundTruth.compute(poses, measured), "Every link should be measurable on noiseless data.");

         fixture.model().setConfiguration(fixture.planted().captureSet.getCapture(capture).getEncoderSample());
         floatingJoint.setJointConfiguration(fixture.planted().basePoses[capture]);
         fixture.model().updateFrames();
         oracle.reset();

         assertEquals(oracle.getCenterOfMass().getX(), measured.getX(), EPSILON, "capture " + capture);
         assertEquals(oracle.getCenterOfMass().getY(), measured.getY(), EPSILON, "capture " + capture);
         assertEquals(oracle.getCenterOfMass().getZ(), measured.getZ(), EPSILON, "capture " + capture);
      }

      assertEquals(28.0, groundTruth.getTotalMass(), EPSILON);
      assertEquals(oracle.getTotalMass(), groundTruth.getTotalMass(), EPSILON, "Both must agree on M, or neither CoM means anything.");
   }

   /**
    * F7: mark only the pelvis and assert every other link's chained pose matches FK exactly.
    * <p>
    * With one marked link the whole robot hangs off it, which is the extreme of §10's trade -- and
    * the case where a chaining bug is unmissable.
    * </p>
    */
   @Test
   public void testChainedPosesMatchForwardKinematicsExactly() throws Exception
   {
      Fixture fixture = fixture(new SyntheticCaptures.Options().captures(8).noise(0.0), "_chain", TiltMeasurement.assumedLevel("test"));

      List<String> pelvisOnly = List.of("pelvis");
      List<MarkerCluster> pelvisCluster = new ArrayList<>();

      for (MarkerCluster cluster : fixture.planted().clusters)
      {
         if (cluster.getLinkName().equals("pelvis"))
            pelvisCluster.add(cluster);
      }

      LinkPoseEstimator estimator = new LinkPoseEstimator(fixture.calibration(), pelvisCluster, fixture.linkNames());
      KinematicChainCoupler coupler = new KinematicChainCoupler(fixture.model(), fixture.linkNames(), pelvisOnly);

      assertEquals(fixture.linkNames().size() - 1, coupler.getChainedLinkCount(), "Everything but the pelvis should chain.");
      assertTrue(coupler.getChainedMass() > 0.6 * fixture.model().getTotalMass(),
                 "With only the pelvis marked, most of the robot's mass rests on encoders: " + coupler.getChainedMass() + " kg.");

      for (int capture = 0; capture < fixture.planted().captureSet.getCaptureCount(); capture++)
      {
         MeasuredLinkPoses poses = new MeasuredLinkPoses(fixture.linkNames());
         estimator.estimate(fixture.planted().captureSet.getCapture(capture).getMocapFrame(), poses);

         assertEquals(1, poses.getAvailableCount(), "Only the pelvis is measured before chaining.");
         assertEquals(fixture.linkNames().size() - 1, coupler.complete(fixture.planted().captureSet.getCapture(capture).getEncoderSample(), poses));

         // Truth: the planted base pose composed with FK at that capture's joint angles.
         fixture.model().setConfiguration(fixture.planted().captureSet.getCapture(capture).getEncoderSample());
         RigidBodyTransform expected = new RigidBodyTransform();
         RigidBodyTransform linkToBase = new RigidBodyTransform();

         for (String linkName : fixture.linkNames())
         {
            fixture.model().packLinkToBase(linkName, linkToBase);
            expected.set(fixture.planted().basePoses[capture]);
            expected.multiply(linkToBase);

            assertTrue(expected.epsilonEquals(poses.getPose(linkName), EPSILON),
                       "capture " + capture + ", link " + linkName + "\n  expected " + expected + "\n  actual   " + poses.getPose(linkName));

            if (!linkName.equals("pelvis"))
               assertEquals(MeasuredLinkPoses.Source.CHAINED, poses.getSource(linkName), linkName + " should be chained, and should say so.");
         }
      }
   }

   /** A chained link whose ancestor was refused has no pose either, and the reason names the ancestor. */
   @Test
   public void testChainingFromARefusedAncestorRefusesToo() throws Exception
   {
      Fixture fixture = noiselessFixture("_chainRefusal");

      List<String> pelvisOnly = List.of("pelvis");
      KinematicChainCoupler coupler = new KinematicChainCoupler(fixture.model(), fixture.linkNames(), pelvisOnly);

      // Nothing measured at all: the pelvis has no pose, so nothing can chain from it.
      MeasuredLinkPoses poses = new MeasuredLinkPoses(fixture.linkNames());
      assertEquals(0, coupler.complete(fixture.planted().captureSet.getCapture(0).getEncoderSample(), poses));

      assertEquals(0, poses.getAvailableCount());
      assertNotNull(poses.getRefusalReason("l_thigh"));
      assertTrue(poses.getRefusalReason("l_thigh").contains("pelvis"), poses.getRefusalReason("l_thigh"));
   }

   /**
    * F8 injection and correction, at the CoM rather than at the transform.
    * <p>
    * FRAMEWORK.md §11's headline: a tilt {@code θ} puts {@code ||c||·sin(θ)} into the CoM height.
    * With the CoM roughly 0.8 m from the world origin and {@code θ = 0.5°}, that is 6.98 mm.
    * </p>
    * <p>
    * The correction half matters as much as the injection half: applying the measured tilt must take
    * the error back to zero, not merely reduce it.
    * </p>
    */
   @Test
   public void testWorldTiltInjectionAndCorrection() throws Exception
   {
      TiltMeasurement tilt = TiltMeasurement.fromTiltAngles(Math.toRadians(0.5), 0.0, TiltMeasurement.Method.PRECISION_LEVEL, "injected");

      Fixture level = noiselessFixture("_tiltLevel");
      Fixture corrected = fixture(new SyntheticCaptures.Options().captures(8).noise(0.0), "_tiltCorrected", tilt);

      CenterOfMassGroundTruth uncorrected = CenterOfMassGroundTruth.forWholeRobot(level.model(),
                                                                                 level.world().getMotiveWorld(),
                                                                                 level.world().getGravityAlignedWorld());
      CenterOfMassGroundTruth withCorrection = CenterOfMassGroundTruth.forWholeRobot(corrected.model(),
                                                                                    corrected.world().getMotiveWorld(),
                                                                                    corrected.world().getGravityAlignedWorld());

      MeasuredLinkPoses poses = estimateFrame(level, 0);

      Point3D uncorrectedCom = new Point3D();
      Point3D correctedCom = new Point3D();
      assertTrue(uncorrected.compute(poses, uncorrectedCom));
      assertTrue(withCorrection.compute(poses, correctedCom));

      // The two differ by exactly the tilt rotation, so the distance from the origin is unchanged
      // and the height differs by the §11 term.
      assertEquals(uncorrectedCom.norm(), correctedCom.norm(), 1.0e-12, "A tilt correction is a rotation; it cannot change the distance.");

      double lever = uncorrectedCom.norm();
      double heightChange = Math.abs(correctedCom.getZ() - uncorrectedCom.getZ());

      assertTrue(heightChange > 0.0, "A 0.5° correction must move the CoM height.");
      assertTrue(heightChange <= lever * Math.sin(Math.toRadians(0.5)) + 1.0e-9,
                 "The height change cannot exceed ||c||·sin(θ) = " + 1000.0 * lever * Math.sin(Math.toRadians(0.5)) + " mm, was " + 1000.0 * heightChange);

      // §11's worked example, stated in its own terms: at ||c|| = 0.8 m the bound is 6.98 mm.
      assertEquals(6.98e-3, 0.8 * Math.sin(Math.toRadians(0.5)), 1.0e-5);

      // And with no tilt, the correction is exactly a no-op.
      Fixture noTilt = noiselessFixture("_tiltNone");
      CenterOfMassGroundTruth plain = CenterOfMassGroundTruth.forWholeRobot(noTilt.model(),
                                                                           noTilt.world().getMotiveWorld(),
                                                                           noTilt.world().getGravityAlignedWorld());
      Point3D plainCom = new Point3D();
      assertTrue(plain.compute(estimateFrame(noTilt, 0), plainCom));
      assertEquals(0.0, plainCom.distance(uncorrectedCom), 1.0e-12);
   }

   /** Occlusion below three markers: F6 must refuse, and say so. */
   @Test
   public void testOcclusionRefusal() throws Exception
   {
      Fixture fixture = noiselessFixture("_occlusion");

      MarkerCluster shank = fixture.planted().clusters.stream().filter(c -> c.getLinkName().equals("l_shank")).findFirst().orElseThrow();

      // Drop l_shank to two visible markers.
      var frame = fixture.planted().captureSet.getCapture(0).getMocapFrame();
      frame.get(shank.getMarker(0)).setNotVisible();
      frame.get(shank.getMarker(1)).setNotVisible();

      MeasuredLinkPoses poses = estimateFrame(fixture, 0);

      assertFalse(poses.isAvailable(poses.indexOf("l_shank")), "Two markers cannot produce a pose.");
      assertEquals(MeasuredLinkPoses.Source.NONE, poses.getSource("l_shank"));
      assertEquals(2, poses.getVisibleCount(poses.indexOf("l_shank")));
      assertTrue(poses.getRefusalReason("l_shank").contains("2 of 4"), poses.getRefusalReason("l_shank"));

      // Every other link is unaffected: a refusal is local.
      assertTrue(poses.isAvailable(poses.indexOf("pelvis")));
      assertTrue(poses.isAvailable(poses.indexOf("r_shank")));

      // And there is no CoM for this frame, rather than the CoM of a lighter robot.
      CenterOfMassGroundTruth groundTruth = CenterOfMassGroundTruth.forWholeRobot(fixture.model(),
                                                                                 fixture.world().getMotiveWorld(),
                                                                                 fixture.world().getGravityAlignedWorld());
      Point3D com = new Point3D();
      assertFalse(groundTruth.compute(poses, com), "A missing link means no CoM.");
      assertTrue(com.containsNaN(), "And the packed value must be NaN, not a plausible number.");
      assertEquals(1, groundTruth.getMissingLinkCount());
      assertEquals(3.0, groundTruth.getMissingMass(), EPSILON, "l_shank masses 3 kg; that is the size of what was unmeasurable.");
   }

   /**
    * The silent failure of FRAMEWORK.md §18.1: three <i>collinear</i> markers pass the visible-count
    * check and still yield a perfectly well-formed rotation carrying no information about rotation
    * about their own axis.
    * <p>
    * The guard is {@code σ₂}. See {@link LinkPoseEstimator} for why refusing on {@code σ₃} -- which
    * is what §18.1 literally says -- rejects the normal case instead.
    * </p>
    */
   @Test
   public void testNearCollinearClusterIsRefusedOnSigma2AndLogged() throws Exception
   {
      Fixture fixture = noiselessFixture("_collinear");

      MarkerCluster shank = fixture.planted().clusters.stream().filter(c -> c.getLinkName().equals("l_shank")).findFirst().orElseThrow();
      var frame = fixture.planted().captureSet.getCapture(0).getMocapFrame();

      // Collapse the cluster onto a line: keep marker 0 where it is and place the rest along the
      // direction to marker 1. Four markers still visible -- the count check cannot help.
      Point3D anchor = new Point3D(frame.get(shank.getMarker(0)).getPosition());
      Point3D along = new Point3D(frame.get(shank.getMarker(1)).getPosition());
      Point3D direction = new Point3D(along);
      direction.sub(anchor);

      for (int j = 1; j < shank.getMarkerCount(); j++)
      {
         Point3D placed = new Point3D(direction);
         placed.scale(j);
         placed.add(anchor);
         frame.get(shank.getMarker(j)).setVisible(placed);
      }

      MeasuredLinkPoses poses = estimateFrame(fixture, 0);
      int shankIndex = poses.indexOf("l_shank");

      assertEquals(4, poses.getVisibleCount(shankIndex), "All four markers are visible; the count check cannot catch this.");
      assertFalse(poses.isAvailable(shankIndex), "A collinear cluster must be refused.");

      String reason = poses.getRefusalReason("l_shank");
      assertNotNull(reason, "§18.1's mitigation is to refuse AND log it.");
      assertTrue(reason.contains("σ₂"), "The reason must name the quantity that drove it: " + reason);
      assertTrue(reason.contains("collinear"), "And say what it means in words: " + reason);
   }

   /** The estimator reports the nominal conditioning it compares against, so a threshold is auditable. */
   @Test
   public void testNominalConditioningIsReportedAndPositive() throws Exception
   {
      Fixture fixture = noiselessFixture("_nominal");
      LinkPoseEstimator estimator = new LinkPoseEstimator(fixture.calibration(), fixture.planted().clusters, fixture.linkNames());

      for (int i = 0; i < fixture.planted().clusters.size(); i++)
      {
         assertTrue(estimator.getNominalSigma2(i) > 0.0, "A non-collinear cluster has a positive nominal σ₂.");
         assertEquals(LinkPoseEstimator.DEFAULT_SIGMA2_FRACTION * estimator.getNominalSigma2(i), estimator.getSigma2Threshold(i), 1.0e-15);
      }

      // A fully visible, noiseless cluster sits exactly at its nominal value.
      MeasuredLinkPoses poses = estimateFrame(fixture, 0);

      for (int i = 0; i < fixture.planted().clusters.size(); i++)
      {
         int linkIndex = poses.indexOf(fixture.planted().clusters.get(i).getLinkName());
         assertEquals(estimator.getNominalSigma3(i), poses.getSigma3(linkIndex), 1.0e-12);
      }
   }

   /**
    * A near-coplanar cluster must NOT be refused: that is the flat-link-face case FRAMEWORK.md §2
    * calls realistic, and a plane fixes a pose perfectly well.
    * <p>
    * This is the counterpart to the collinear test, and the reason the guard moved from {@code σ₃}
    * to {@code σ₂}. Before the move, the toy's own {@code l_shank} cluster -- which happens to be
    * near-coplanar -- was refused in roughly one frame in six of a perfectly good replay.
    * </p>
    */
   @Test
   public void testCoplanarClusterIsAccepted() throws Exception
   {
      Fixture fixture = fixture(new SyntheticCaptures.Options().captures(40).noise(0.3e-3), "_coplanar", TiltMeasurement.assumedLevel("test"));
      LinkPoseEstimator estimator = new LinkPoseEstimator(fixture.calibration(), fixture.planted().clusters, fixture.linkNames());

      // Find the flattest cluster: the smallest ratio of nominal σ₃ to σ₂.
      int flattest = 0;

      for (int i = 1; i < fixture.planted().clusters.size(); i++)
      {
         if (estimator.getNominalSigma3(i) / estimator.getNominalSigma2(i) < estimator.getNominalSigma3(flattest) / estimator.getNominalSigma2(flattest))
            flattest = i;
      }

      String flattestLink = fixture.planted().clusters.get(flattest).getLinkName();
      assertTrue(estimator.getNominalSigma3(flattest) < 0.1 * estimator.getNominalSigma2(flattest),
                 "Expected a markedly flat cluster to test with; '" + flattestLink + "' has σ₃/σ₂ = "
                       + estimator.getNominalSigma3(flattest) / estimator.getNominalSigma2(flattest));

      MeasuredLinkPoses poses = new MeasuredLinkPoses(fixture.linkNames());
      int refusals = 0;

      for (int capture = 0; capture < fixture.planted().captureSet.getCaptureCount(); capture++)
      {
         poses.clear();
         estimator.estimate(fixture.planted().captureSet.getCapture(capture).getMocapFrame(), poses);

         if (!poses.isAvailable(poses.indexOf(flattestLink)))
            refusals++;
      }

      assertEquals(0, refusals, "A flat cluster is not a degenerate cluster; '" + flattestLink + "' was refused in " + refusals + " of 40 frames.");
   }
}
