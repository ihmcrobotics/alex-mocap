package us.ihmc.alexMocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.calibration.AlternatingCalibrator;
import us.ihmc.alexMocap.calibration.BaseInitializer;
import us.ihmc.alexMocap.calibration.BaseInitializer.GaugeTracking;
import us.ihmc.alexMocap.calibration.CalibrationReport;
import us.ihmc.alexMocap.calibration.RobotCaptures;
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.Capture;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.frames.GravityAlignedWorldFrame;
import us.ihmc.alexMocap.frames.TiltMeasurement;
import us.ihmc.alexMocap.gates.BootstrapSpreadGate;
import us.ihmc.alexMocap.gates.GateResult;
import us.ihmc.alexMocap.gates.HeldOutResidualGate;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.runtime.CenterOfMassGroundTruth;
import us.ihmc.alexMocap.runtime.KinematicChainCoupler;
import us.ihmc.alexMocap.runtime.LinkPoseEstimator;
import us.ihmc.alexMocap.runtime.MeasuredLinkPoses;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

/**
 * The demonstration: the whole pipeline on the <b>real Alex model</b>, with markers on the legs.
 *
 * <h2>What this adds over {@code PlantAndRecoverTest}</h2>
 * <p>
 * Everything in PR1-PR3 was verified against a toy 6-DOF URDF, and every accuracy claim in
 * RUNNING.md carries the caveat "this tests the solver, not the robot". This class points the same
 * shipping classes at the URDF the Python InEKF uses -- 29 joints, 30 links after SCS2's fixed-joint
 * merge, 91.5126 kg, 0.89 m from pelvis origin to foot -- and reports what changes.
 * </p>
 * <p>
 * It still does not measure the robot. The URDF is assumed correct here exactly as FRAMEWORK.md §3
 * assumes it; what changes is that the <i>geometry</i>, the <i>lever arms</i> and the <i>joint
 * ranges</i> are Alex's rather than a toy's, and those are what set every number below.
 * </p>
 *
 * <h2>The headline</h2>
 * <p>
 * With only the pelvis and legs marked, <b>58.45% of Alex's mass has its pose from FK, not from
 * markers</b>. {@code TORSO_LINK} alone is 22.21 kg -- 24.3% of the robot -- chained off the pelvis
 * through one {@code SPINE_Z} joint, which FRAMEWORK.md §1 specifically warns carries the full
 * suspension load in tension. One torso cluster is the highest-leverage addition after the gauge.
 * </p>
 *
 * <h2>Verbose output</h2>
 * <p>
 * Run with {@code -Dalex.demo.verbose=true} to print the tables. CI stays quiet by default.
 * </p>
 */
public class AlexLegDemoTest
{
   /** FRAMEWORK.md §17: the target per-axis mocap noise for a tight volume at the gantry. */
   private static final double SIGMA_TARGET = 0.3e-3;

   /** §17: what the whole-lab calibration gives today. */
   private static final double SIGMA_CURRENT = 0.93e-3;

   /**
    * Deliberately quieter than either, and used only by the G2 injection test.
    * <p>
    * Same reasoning as {@code GateInjectionTest}: that test measures whether the gate
    * <b>discriminates</b>, and at 0.3 mm Alex's own gauge-dominated floor (see
    * {@link #testHeldOutResidualMissesTheTalosBarAtTheRecommendedBracketWidth()}) is larger than the
    * fault being injected, so a working gate and a broken one produce the same verdict.
    * </p>
    */
   private static final double SIGMA_QUIET = 0.05e-3;

   private static final boolean VERBOSE = Boolean.getBoolean("alex.demo.verbose");

   private static void verbose(String format, Object... arguments)
   {
      if (VERBOSE)
         System.out.printf(format + "%n", arguments);
   }

   private record Fitted(RobotCaptures.Planted planted, GaugeTracking tracking, CalibrationResult calibration, CalibrationReport report)
   {
   }

   private static Fitted fit(RobotCaptures.Options options) throws Exception
   {
      return fit(options, new AlternatingCalibrator());
   }

   private static Fitted fit(RobotCaptures.Options options, AlternatingCalibrator calibrator) throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(options);
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);
      CalibrationReport report = new CalibrationReport();
      CalibrationResult calibration = calibrator.calibrate(planted.captureSet, planted.model, tracking, report);
      return new Fitted(planted, tracking, calibration, report);
   }

   /** Largest distance between a recovered marker position and its planted truth, in metres. */
   private static double worstLayoutError(Fitted fitted)
   {
      double worst = 0.0;

      for (MarkerCluster cluster : fitted.planted().clusters)
      {
         ClusterLayout truth = fitted.planted().plantedLayout(cluster.getLinkName());
         ClusterLayout estimate = fitted.calibration().getLayout(cluster.getLinkName());

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            if (estimate.getObservationCount(j) == 0)
               continue;

            worst = Math.max(worst, new Point3D(truth.getPositionInLinkFrame(j)).distance(new Point3D(estimate.getPositionInLinkFrame(j))));
         }
      }

      return worst;
   }

   /**
    * {@code {worst position error in m, worst rotation error in rad}} of the reconstructed base pose
    * {@code ^W T_c^(k) · Δ̂} against the planted {@code ^W T_b^(k)}.
    * <p>
    * Reported as two numbers rather than one max, unlike the toy's helper: on Alex they differ by a
    * factor of ten in significance. 0.8 mm of base position is negligible next to a 0.89 m leg;
    * 7.6 mrad of base <i>rotation</i> is 6 mm at the foot, and it is the same angular error the
    * runtime pelvis pose carries frame to frame.
    * </p>
    */
   private static double[] worstBasePoseError(Fitted fitted)
   {
      RigidBodyTransform reconstructed = new RigidBodyTransform();
      double worstPosition = 0.0;
      double worstRotation = 0.0;

      for (int k = 0; k < fitted.planted().captureSet.getCaptureCount(); k++)
      {
         if (!fitted.tracking().isUsable(k))
            continue;

         reconstructed.set(fitted.tracking().getClusterToWorld(k));
         reconstructed.multiply(fitted.calibration().getClusterToBase());

         Point3D origin = new Point3D();
         reconstructed.transform(origin);

         Point3D plantedOrigin = new Point3D();
         fitted.planted().basePoses[k].transform(plantedOrigin);
         worstPosition = Math.max(worstPosition, origin.distance(plantedOrigin));

         RigidBodyTransform error = new RigidBodyTransform(fitted.planted().basePoses[k]);
         error.invert();
         error.multiply(reconstructed);
         worstRotation = Math.max(worstRotation, Math.abs(new AxisAngle(error.getRotation()).getAngle()));
      }

      return new double[] {worstPosition, worstRotation};
   }

   // ------------------------------------------------------------------------------------------
   // 1. The model itself.
   // ------------------------------------------------------------------------------------------

   /**
    * The vendored URDF is the real one and it loads: 29 joints, 30 links, {@code PELVIS_LINK} at the
    * root, 91.5126 kg.
    *
    * <h2>The tripwire that matters here</h2>
    * <p>
    * The source URDF has 141 links and 140 joints, of which 111 are {@code fixed}. SCS2's
    * {@code simplifyKinematics} (on by default) merges every fixed joint, and the merge sums masses
    * and mass-weighted CoMs. <b>70 of those 141 links carry no {@code <inertial>} block at all and
    * 19 declare mass 0 with zero inertia</b>, so a merge that divided by an accumulated mass without
    * guarding zero would produce NaN in exactly the links a marker cluster would be mounted on --
    * and a NaN {@code ^i c_i} is invisible until F9 returns a NaN CoM at runtime.
    * </p>
    * <p>
    * It does not happen: every one of the 30 surviving links has a finite, strictly positive mass
    * and a finite {@code ^i c_i}. Asserted rather than assumed, because a future URDF edit or SCS2
    * bump is exactly the thing that would reintroduce it.
    * </p>
    */
   @Test
   public void testTheRealModelLoads() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();

      assertEquals(29, model.getJointCount(), "29 revolute joints survive the fixed-joint merge.");
      assertEquals(30, model.getLinkNames().size(), "30 links survive it: 29 joints plus the root.");
      assertEquals("PELVIS_LINK", model.getBaseLinkName(), "The base link b is the pelvis (FRAMEWORK.md §0).");
      assertEquals(91.512588, model.getTotalMass(), 1.0e-4, "Mass must be preserved exactly by the merge.");

      model.updateFrames();
      Point3D centerOfMass = new Point3D();
      double summedMass = 0.0;

      for (String link : model.getLinkNames())
      {
         double mass = model.getMass(link);
         assertTrue(Double.isFinite(mass) && mass > 0.0, link + " has mass " + mass + "; the fixed-joint merge produced a non-finite or zero mass.");

         model.packCenterOfMassInLinkFrame(link, centerOfMass);
         assertTrue(!centerOfMass.containsNaN() && Double.isFinite(centerOfMass.norm()), link + " has ^i c_i = " + centerOfMass + ".");
         assertTrue(centerOfMass.norm() < 1.0, link + "'s CoM is " + centerOfMass.norm() + " m from its own link origin, which is not a plausible link.");

         summedMass += mass;
         verbose("  %-24s m = %8.4f kg   ^i c_i = (%+.5f %+.5f %+.5f)", link, mass, centerOfMass.getX(), centerOfMass.getY(), centerOfMass.getZ());
      }

      assertEquals(model.getTotalMass(), summedMass, 1.0e-9, "getTotalMass must be the sum of the links it reports.");

      // The generator samples inside these; a non-finite one would be sampled as NaN.
      for (String joint : RobotCaptures.LEG_JOINTS)
      {
         double lower = model.getJointLimitLower(joint);
         double upper = model.getJointLimitUpper(joint);
         assertTrue(Double.isFinite(lower) && Double.isFinite(upper) && upper > lower, joint + " has limits [" + lower + ", " + upper + "].");
         verbose("  %-14s [%+.4f, %+.4f] rad, range %.1f deg", joint, lower, upper, Math.toDegrees(upper - lower));
      }

      // The two the generator has to report on: LEFT/RIGHT_KNEE_Y declare lower="0" exactly.
      assertEquals(0.0, model.getJointLimitLower("LEFT_KNEE_Y"), 0.0, "The knee's lower limit is exactly zero; the rest angle sits on it.");
      assertEquals(0.0, model.getJointLimitLower("RIGHT_KNEE_Y"), 0.0);
   }

   /**
    * At {@code q = 0} every link frame is <b>base-aligned</b> -- and the reason is not the one it
    * looks like.
    *
    * <h2>SCS2 rewrites the URDF's link frames, and it is on by default</h2>
    * <p>
    * The leg chains are the easy half: every joint from {@code PELVIS_LINK} down to either foot
    * declares {@code rpy="0 0 0"}, so their link frames are trivially base-aligned at {@code q = 0}
    * and SCS2's link frame <b>is</b> the URDF link frame.
    * </p>
    * <p>
    * The arms are not. {@code LEFT_SHOULDER_Y} declares {@code rpy="0.698132 0 0"} (40°) and
    * {@code LEFT_SHOULDER_X} declares {@code rpy="-0.698132 0 0"}. Yet the loaded model reports
    * identity for {@code LEFT_SHOULDER_Y_LINK} too. That is because
    * {@code URDFTools.toRobotDefinition} calls {@code RobotDefinition.transformAllFramesToZUp()} --
    * <b>default {@code true}</b>, alongside {@code simplifyKinematics} -- which walks the tree
    * zeroing the rotation of every joint's {@code transformToParent} and compensating by rotating,
    * in place: the joint axis, the inertia pose, the moment of inertia, every child joint's
    * transform, and every visual, collision, sensor and kinematic-point pose.
    * </p>
    * <p>
    * <b>The kinematics and the physics are exactly preserved.</b> Nothing in F1-F11 is wrong. What
    * is <i>not</i> preserved is the identity of the link frame: for any link below a joint with a
    * non-zero {@code rpy}, SCS2's frame is the URDF's frame rotated by the accumulated joint
    * rotation. The consequence is documented in {@code RobotModelHandle}'s javadoc as a promise --
    * "a calibrated {@code ^i p̂_ij} printed by this pipeline is directly comparable to a CAD marker
    * position" -- and that promise <b>does not hold for Alex's arms</b>. It holds for the legs,
    * which is the set this demonstration marks.
    * </p>
    * <p>
    * The toy 6-DOF URDF declares {@code rpy="0 0 0"} on every joint, so no existing test could have
    * caught this.
    * </p>
    */
   @Test
   public void testLinkFramesAreBaseAlignedAtZeroBecauseScs2RewritesThem() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      model.setQ(new double[model.getJointCount()]);
      model.updateFrames();

      RigidBodyTransform linkToBase = new RigidBodyTransform();

      // The leg chain: genuinely a no-op, because the URDF declares rpy="0 0 0" on all 12 joints.
      for (String link : RobotCaptures.LEG_LINKS)
      {
         model.packLinkToBase(link, linkToBase);
         double angle = Math.abs(new AxisAngle(linkToBase.getRotation()).getAngle());
         assertTrue(angle < 1.0e-12, link + "'s ^b T_i rotation at q = 0 is " + angle + " rad, expected identity.");
      }

      // And so is every other link -- but for the arms that is SCS2's doing, not the URDF's.
      for (String link : model.getLinkNames())
      {
         model.packLinkToBase(link, linkToBase);
         assertTrue(Math.abs(new AxisAngle(linkToBase.getRotation()).getAngle()) < 1.0e-12,
                    link + " should be base-aligned at q = 0 after transformAllFramesToZUp.");
      }

      // The proof that this is a rewrite and not a property of the file. Straight from the vendored
      // URDF, LEFT_SHOULDER_Y_LINK's inertial origin is xyz="-0.00264 0.12135 -0.006824" and its
      // parent joint LEFT_SHOULDER_Y declares rpy="0.698132 0 0". Everything above that joint is
      // rpy-free, so the accumulated rotation is exactly R_x(0.698132). SCS2 must therefore report
      // ^i c_i as R_x(0.698132) applied to the URDF vector -- which is not the URDF vector.
      Point3D urdfInertialOrigin = new Point3D(-0.00264, 0.12135, -0.006824);
      Point3D expected = new Point3D(urdfInertialOrigin);
      new RigidBodyTransform(new AxisAngle(1.0, 0.0, 0.0, 0.698132), new Vector3D()).transform(expected);

      Point3D reported = new Point3D();
      model.packCenterOfMassInLinkFrame("LEFT_SHOULDER_Y_LINK", reported);

      assertEquals(expected.getX(), reported.getX(), 1.0e-6);
      assertEquals(expected.getY(), reported.getY(), 1.0e-6);
      assertEquals(expected.getZ(), reported.getZ(), 1.0e-6);
      assertTrue(reported.distance(urdfInertialOrigin) > 0.02,
                 "If this ever becomes zero, SCS2 stopped rewriting frames and ^i p̂_ij became CAD-comparable everywhere. "
                       + "Delete the caveat in RobotModelHandle's javadoc when it does. Measured separation: " + reported.distance(urdfInertialOrigin) + " m.");

      // A leg link, by contrast, matches its URDF inertial block exactly.
      Point3D thigh = new Point3D();
      model.packCenterOfMassInLinkFrame("LEFT_THIGH", thigh);
      assertEquals(0.01414574, thigh.getX(), 1.0e-7);
      assertEquals(0.032755865, thigh.getY(), 1.0e-7);
      assertEquals(-0.105455545, thigh.getZ(), 1.0e-7);
   }

   // ------------------------------------------------------------------------------------------
   // 2. The calibration.
   // ------------------------------------------------------------------------------------------

   /**
    * Noiseless, {@code K = 5}, the 7-cluster set. Any failure here is an algebra bug on real
    * geometry, not a tuning problem.
    * <p>
    * Measured: layout 6.341e-16 m, base position 6.280e-16 m, base rotation 0.0 rad exactly, in 104
    * iterations. The threshold is PR_PLAN.md's 1e-9.
    * </p>
    */
   @Test
   public void testNoiselessRecoveryIsExactOnTheRealModel() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(5).noise(0.0));

      assertTrue(fitted.calibration().isFullySolved(), "Every marker should have a finite calibrated position.\n" + fitted.report().toTable());
      assertTrue(fitted.report().isConverged(), "A' should converge on noiseless data.\n" + fitted.report().toTable());

      double layout = worstLayoutError(fitted);
      double[] base = worstBasePoseError(fitted);
      verbose("noiseless K=5: layout %.3e m, base position %.3e m, base rotation %.3e rad, %d iterations", layout, base[0], base[1],
              fitted.report().getIterationCount());

      assertTrue(layout < 1.0e-9, "Layout error " + layout + " m exceeds 1e-9.\n" + fitted.report().toTable());
      assertTrue(base[0] < 1.0e-9, "Base position error " + base[0] + " m exceeds 1e-9.\n" + fitted.report().toTable());
      assertTrue(base[1] < 1.0e-9, "Base rotation error " + base[1] + " rad exceeds 1e-9.\n" + fitted.report().toTable());

      // The rest angles the generator had to reason about, printed rather than swallowed.
      assertEquals(2, fitted.planted().restAngleNotes.size(), "Both knees declare lower=\"0\" and must be reported: " + fitted.planted().restAngleNotes);

      for (String note : fitted.planted().restAngleNotes)
         verbose("  rest angle: %s", note);
   }

   /**
    * {@code σ = 0.3 mm}, {@code K = 30}, the 7-cluster set, gauge cluster at FRAMEWORK.md §1's
    * recommended 140 mm.
    *
    * <h2>Measured, and where the thresholds come from</h2>
    * <p>
    * At the default seed: worst marker layout error <b>0.3140 mm</b>, worst base <i>position</i>
    * error <b>0.8432 mm</b>, worst base <i>rotation</i> error <b>7.6331 mrad</b> (0.437°),
    * in-sample RMS 2.01 mm, 42 iterations, converged. Thresholds are set at roughly 3× each.
    * </p>
    * <p>
    * Two things a reader should know before touching these numbers.
    * </p>
    * <p>
    * <b>The layout error is an order statistic with a fat tail.</b> Over ten seeds the worst-marker
    * figure ranges 0.45-2.47 mm with a median near 0.7 mm. The tail is not noise in the estimate;
    * it is RUNNING.md's "reference-shape noise does not average at all" -- {@code BaseInitializer}
    * defines the gauge cluster's shape from capture 0's marker positions, so an unlucky reference
    * capture is baked into the session. Seeds 7 and 9 draw a bad capture 0 and carry 24 and 20 mrad
    * of base rotation error against a typical 8. Changing the seed here without re-measuring will
    * look like a regression.
    * </p>
    * <p>
    * <b>The base rotation error is the number to look at, not the position error.</b> 0.44° at the
    * pelvis is 6.8 mm at a foot 0.89 m away, and it is also, unchanged, the frame-to-frame noise on
    * the runtime pelvis <i>orientation</i> that F10 hands to the EKF comparison -- F6 is
    * single-frame with no averaging (§9). It follows the {@code σ/(√N·r_perp)} law of §1 exactly;
    * see {@link #testHeldOutResidualMissesTheTalosBarAtTheRecommendedBracketWidth()}.
    * </p>
    */
   @Test
   public void testRecoveryAtTheTargetNoise() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(30).noise(SIGMA_TARGET));

      double layout = worstLayoutError(fitted);
      double[] base = worstBasePoseError(fitted);

      verbose("σ=0.3 mm K=30: layout %.4f mm, base position %.4f mm, base rotation %.4f mrad (%.4f°), in-sample RMS %.4f mm, %d iterations",
              1000 * layout, 1000 * base[0], 1000 * base[1], Math.toDegrees(base[1]), 1000 * fitted.report().getOverallRmsMeters(),
              fitted.report().getIterationCount());

      if (VERBOSE)
         System.out.println(fitted.report().toTable());

      assertTrue(fitted.report().isConverged(), "A' should converge.\n" + fitted.report().toTable());

      // Measured 0.3140 mm.
      assertTrue(layout < 1.0e-3, "Layout error " + 1000 * layout + " mm exceeds 1.0 mm.\n" + fitted.report().toTable());

      // Measured 0.8432 mm.
      assertTrue(base[0] < 3.0e-3, "Base position error " + 1000 * base[0] + " mm exceeds 3.0 mm.\n" + fitted.report().toTable());

      // Measured 7.6331 mrad.
      assertTrue(base[1] < 25.0e-3, "Base rotation error " + 1000 * base[1] + " mrad exceeds 25 mrad.\n" + fitted.report().toTable());

      // The gauge cluster's own layout has no lever arm to amplify anything. Measured 0.3016 mm.
      double pelvis = clusterLayoutError(fitted, "PELVIS_LINK");
      assertTrue(pelvis < 1.0e-3, "The gauge cluster's own layout should be well under a millimetre, was " + 1000 * pelvis + " mm.");

      // And the per-cluster gradient RUNNING.md describes: the error grows with distance from the
      // gauge. Measured, mm: PELVIS 0.302, THIGH 0.286/0.220, SHIN 0.249/0.314, FOOT 0.096/0.284 --
      // flat in the *layout*, but the in-sample *residual* climbs 0.34 -> 2.9 mm because that is
      // where the gauge cluster's angular error times the lever arm actually lands.
      List<CalibrationReport.MarkerResidual> residuals = fitted.report().getMarkerResiduals();
      double pelvisResidual = 0.0;
      double footResidual = 0.0;

      for (CalibrationReport.MarkerResidual residual : residuals)
      {
         if (residual.linkName().equals("PELVIS_LINK"))
            pelvisResidual = Math.max(pelvisResidual, residual.rmsMeters());
         if (residual.linkName().equals("LEFT_FOOT"))
            footResidual = Math.max(footResidual, residual.rmsMeters());
      }

      assertTrue(footResidual > 4.0 * pelvisResidual,
                 "The in-sample residual must climb from the gauge outward -- that gradient is the dominant error term, not a bad fit. pelvis "
                       + 1000 * pelvisResidual + " mm, foot " + 1000 * footResidual + " mm.");
   }

   /** The same at today's lab noise, so the cost of not doing §20's camera work is a number. */
   @Test
   public void testRecoveryAtTodaysLabNoise() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(30).noise(SIGMA_CURRENT));
      double layout = worstLayoutError(fitted);
      double[] base = worstBasePoseError(fitted);

      verbose("σ=0.93 mm K=30: layout %.4f mm, base rotation %.4f mrad, in-sample RMS %.4f mm", 1000 * layout, 1000 * base[1],
              1000 * fitted.report().getOverallRmsMeters());

      // Measured: layout 0.9860 mm, base rotation 23.7910 mrad, in-sample RMS 6.2479 mm.
      // Thresholds ~3x. Note the in-sample RMS: at today's whole-lab noise the marker residuals on
      // Alex are 6 mm, which is where FRAMEWORK.md §20's camera work stops being optional.
      assertTrue(layout < 3.5e-3, "Layout error " + 1000 * layout + " mm exceeds 3.5 mm at σ = 0.93 mm.\n" + fitted.report().toTable());
      assertTrue(base[1] < 75.0e-3, "Base rotation error " + 1000 * base[1] + " mrad exceeds 75 mrad at σ = 0.93 mm.");
   }

   /** The 13-leg-link case: every link of both leg chains marked, nothing else. */
   @Test
   public void testAllThirteenLegLinksMarked() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(30).noise(SIGMA_TARGET).marked(RobotCaptures.LEG_LINKS));

      assertEquals(13, fitted.planted().clusters.size());
      assertTrue(fitted.calibration().isFullySolved(), fitted.report().toTable());
      assertTrue(fitted.report().isConverged(), fitted.report().toTable());

      double layout = worstLayoutError(fitted);
      verbose("13 leg clusters, σ=0.3 mm K=30: layout %.4f mm, base-step σ₃ %.4e", 1000 * layout, fitted.report().getBaseStepSigma3());

      // Measured 0.7332 mm. Larger than the 7-cluster set's 0.3140 mm rather than smaller, because
      // the added clusters sit on the hip and ankle stubs -- at essentially zero lever arm from
      // their parent, they add correspondences that carry almost no information about Δ while
      // adding 24 more markers whose own layouts must be recovered.
      assertTrue(layout < 2.5e-3, "Layout error " + 1000 * layout + " mm exceeds 2.5 mm.\n" + fitted.report().toTable());

      // Marking all 13 buys 6.3 kg of chained mass back out of 53.5. That is the point: the stubs
      // are cheap to mark and worth almost nothing, and TORSO_LINK is neither.
      KinematicChainCoupler coupler = new KinematicChainCoupler(fitted.planted().model, fitted.planted().model.getLinkNames(), List.of(RobotCaptures.LEG_LINKS));
      assertEquals(47.2057, coupler.getChainedMass(), 1.0e-3, "13 leg clusters still leave 51.6% of the robot on encoders.");
   }

   /**
    * <b>The property that justifies A′ over bundle adjustment</b>, on real geometry: {@code J} is
    * non-increasing across every half-step. Checked per half-step so a failure names which solver
    * broke.
    */
   @Test
   public void testObjectiveIsMonotoneAcrossEveryHalfStep() throws Exception
   {
      for (long seed : new long[] {1L, 2L, 20260810L})
      {
         Fitted fitted = fit(new RobotCaptures.Options().seed(seed).captures(20).noise(SIGMA_CURRENT).occlusion(0.1));
         double[] sequence = fitted.report().getObjectiveSequence();

         for (int i = 1; i < sequence.length; i++)
         {
            assertTrue(sequence[i] <= sequence[i - 1] * (1.0 + 1.0e-12) + Double.MIN_NORMAL,
                       "J rose at step " + i + " (seed " + seed + "): " + sequence[i - 1] + " -> " + sequence[i]
                             + (i % 2 == 1 ? " across F5, the base step." : " across F4, the marker step.") + "\n" + fitted.report().toTable());
         }

         assertTrue(fitted.report().isMonotone(1.0e-12), "seed " + seed + "\n" + fitted.report().toTable());
      }
   }

   // ------------------------------------------------------------------------------------------
   // 3. Identifiability.
   // ------------------------------------------------------------------------------------------

   /**
    * {@code PELVIS_LINK} plus both thighs and nothing else is still exactly identifiable.
    * <p>
    * The toy needed its feet marked to break a translational gauge freedom, because with only the
    * pelvis, thighs and shanks marked every marked link's orientation was a rotation about
    * {@code y} alone. Alex is not exposed to that at the thigh: the chain to {@code *_THIGH} runs
    * {@code HIP_X}(x) → {@code HIP_Z}(z) → {@code HIP_Y}(y), so a thigh's orientation already spans
    * three independent axes and no direction {@code g} satisfies {@code R_i(q)ᵀ g = g} at every
    * capture.
    * </p>
    * <p>
    * Measured noiseless at {@code K = 5}: layout 8.07e-16 m, base 8.67e-16 m, 267 iterations. The
    * margin is in the conditioning, not the answer -- the base step's {@code σ₃} is 1.74e-3 m²
    * against 4.59e-2 m² for the 7-cluster set, a factor of 26. Identifiable, and 26× less
    * comfortable about it.
    * </p>
    */
   @Test
   public void testPelvisAndThighsAloneAreStillIdentifiable() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(5).noise(0.0).marked("PELVIS_LINK", "LEFT_THIGH", "RIGHT_THIGH"));

      double layout = worstLayoutError(fitted);
      double[] base = worstBasePoseError(fitted);
      verbose("pelvis+thighs noiseless: layout %.3e m, base %.3e m, base-step σ₃ %.4e, %d iterations", layout, base[0], fitted.report().getBaseStepSigma3(),
              fitted.report().getIterationCount());

      assertTrue(fitted.report().isConverged(), fitted.report().toTable());
      assertTrue(layout < 1.0e-9, "Layout error " + layout + " m exceeds 1e-9; this set should be exactly identifiable.\n" + fitted.report().toTable());
      assertTrue(base[0] < 1.0e-9, "Base position error " + base[0] + " m exceeds 1e-9.\n" + fitted.report().toTable());

      Fitted sevenClusters = fit(new RobotCaptures.Options().captures(5).noise(0.0));
      assertTrue(fitted.report().getBaseStepSigma3() < sevenClusters.report().getBaseStepSigma3(),
                 "Dropping the shanks and feet must condition the base step worse: " + fitted.report().getBaseStepSigma3() + " vs "
                       + sevenClusters.report().getBaseStepSigma3());
   }

   /**
    * <b>The predicted degeneracy, confirmed.</b> {@code PELVIS_LINK} plus both {@code *_HIP_X_LINK}
    * and nothing else drives {@code J} to noise-floor on noiseless data and lands <b>56 mm from the
    * truth</b>, with every marker displaced by the same vector along {@code x}.
    *
    * <h2>Why</h2>
    * <p>
    * Replacing {@code Δ} by {@code Δ·G} for a pure translation {@code g} shifts each layout by
    * {@code R_i(q)ᵀ g}, which is a genuine symmetry of {@code J} exactly when {@code R_i(q)ᵀ g} is
    * the same at every capture -- i.e. when {@code g} lies along an axis every marked link merely
    * rotates about. {@code PELVIS_LINK} does not rotate relative to the base at all, and
    * {@code *_HIP_X_LINK}'s orientation is {@code R_x(q)} for the hip-X joint alone. Alex's two
    * hip-X axes are <b>parallel</b> -- both {@code (1 0 0)} in the pelvis frame, and both joint
    * origins are {@code rpy="0 0 0"} -- so {@code g} along {@code x} is free.
    * </p>
    *
    * <h2>What the numbers say, and what they do not</h2>
    * <p>
    * At an explicit 200-iteration cap: {@code J = 3.02e-6 m²}, an in-sample RMS of
    * <b>0.112 mm on data with no noise in it</b>, and a layout 57.3 mm wrong. Uncapped at 500 it
    * reaches {@code J = 2.7e-8} and 56.1 mm -- {@code J} falls geometrically forever while the
    * answer does not move, because A′ is sliding down a flat valley. The cap is set explicitly so
    * the test is fast and the sequence is deterministic.
    * </p>
    * <p>
    * <b>{@code σ₃} does not detect it.</b> The base step's {@code σ₃} is 8.79e-4 m² here against
    * 2.21e-3 m² for the identifiable pelvis+thighs set at the same {@code K} -- a factor of 2.5,
    * which is nothing next to the factor of 26 that separates two sets that are <i>both</i>
    * identifiable. There is no threshold on {@code σ₃} that separates "degenerate" from "merely
    * poorly conditioned". This is PR2's finding reproduced at Alex scale.
    * </p>
    */
   @Test
   public void testHipXOnlyMarkedSetIsDegenerateAlongX() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(20).noise(0.0).marked(RobotCaptures.HIP_X_ONLY_MARKED_LINKS),
                          new AlternatingCalibrator(AlternatingCalibrator.DEFAULT_RELATIVE_TOLERANCE, 200));

      double layout = worstLayoutError(fitted);
      verbose("hip-X only: J %.4e, in-sample RMS %.4f mm on noiseless data, layout error %.4f mm, base-step σ₃ %.4e", fitted.report().getFinalObjective(),
              1000 * fitted.report().getOverallRmsMeters(), 1000 * layout, fitted.report().getBaseStepSigma3());

      // A perfect-looking fit. Measured in-sample RMS 0.112 mm with zero noise in the data.
      assertTrue(fitted.report().getOverallRmsMeters() < 0.5e-3,
                 "The fit must look excellent -- that is what makes this dangerous. In-sample RMS was " + 1000 * fitted.report().getOverallRmsMeters() + " mm.");

      // And a badly wrong answer. Measured 57.3 mm.
      assertTrue(layout > 0.03, "The layout must be badly wrong. Measured error " + 1000 * layout + " mm.\n" + fitted.report().toTable());

      // Every marker displaced by the same vector: that is what makes it a gauge freedom rather
      // than a fit that simply failed.
      List<Vector3D> displacements = new ArrayList<>();

      for (MarkerCluster cluster : fitted.planted().clusters)
      {
         ClusterLayout truth = fitted.planted().plantedLayout(cluster.getLinkName());
         ClusterLayout estimate = fitted.calibration().getLayout(cluster.getLinkName());

         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            Vector3D displacement = new Vector3D();
            displacement.sub(estimate.getPositionInLinkFrame(j), truth.getPositionInLinkFrame(j));
            displacements.add(displacement);
         }
      }

      Vector3D mean = new Vector3D();

      for (Vector3D displacement : displacements)
         mean.add(displacement);

      mean.scale(1.0 / displacements.size());

      double worstDeviation = 0.0;

      for (Vector3D displacement : displacements)
      {
         Vector3D deviation = new Vector3D(displacement);
         deviation.sub(mean);
         worstDeviation = Math.max(worstDeviation, deviation.norm());
      }

      verbose("  common displacement (%+.6f %+.6f %+.6f), |mean| %.6f m, worst deviation from it %.3e m", mean.getX(), mean.getY(), mean.getZ(), mean.norm(),
              worstDeviation);

      // Measured at the 200-iteration cap: mean (+0.055940, +0.000149, +0.000188), |mean| 55.941 mm,
      // worst deviation from it 1.584 mm -- 2.8% of the common shift. (Uncapped at 500 iterations
      // the deviation falls to 0.16 mm: the residual scatter is A' still travelling down the valley,
      // not a second error mode.)
      assertTrue(mean.norm() > 0.03, "The common displacement should be the whole of the error, was " + mean.norm() + " m.");
      assertTrue(worstDeviation < 0.10 * mean.norm(),
                 "Every marker must be displaced by nearly the same vector: worst deviation " + worstDeviation + " m against a mean of " + mean.norm() + " m.");

      // And it lies along x, the shared hip-X axis. Measured |(y,z)| / |mean| = 4.3e-3.
      assertTrue(Math.hypot(mean.getY(), mean.getZ()) < 0.02 * mean.norm(),
                 "The free direction is the shared hip-X axis: displacement was (" + mean.getX() + ", " + mean.getY() + ", " + mean.getZ() + ").");

      // σ₃ is blind to it. Both of these are noiseless K=20 runs of the same solver.
      Fitted identifiable = fit(new RobotCaptures.Options().captures(20).noise(0.0).marked("PELVIS_LINK", "LEFT_THIGH", "RIGHT_THIGH"),
                                new AlternatingCalibrator(AlternatingCalibrator.DEFAULT_RELATIVE_TOLERANCE, 200));

      assertTrue(identifiable.report().getBaseStepSigma3() < 5.0 * fitted.report().getBaseStepSigma3(),
                 "σ₃ must NOT separate the degenerate set from a merely awkward one -- if it ever does, this test's whole point has changed. "
                       + "degenerate σ₃ = " + fitted.report().getBaseStepSigma3() + ", identifiable σ₃ = " + identifiable.report().getBaseStepSigma3());
      assertTrue(worstLayoutError(identifiable) < 1.0e-9, "The control must actually be identifiable.");
   }

   /**
    * A narrow sweep collapses the base step's conditioning and, past a point, the answer with it.
    * <p>
    * Alex's hip-X range is 70°, against the toy's 183°. Rank survives that; conditioning does not.
    * Measured at {@code σ = 0.3 mm}, {@code K = 30}, base-step {@code σ₃} in m² and worst layout
    * error in mm:
    * </p>
    *
    * <pre>
    *   excursion fraction     σ₃          layout error   converged
    *   1.00                4.864e-02        0.31 mm        yes (42 iterations)
    *   0.50                3.721e-02        0.49 mm        yes (159)
    *   0.20                1.210e-02        1.16 mm        no  (hit 500)
    *   0.05                6.421e-03       34.96 mm        no  (hit 500)
    * </pre>
    *
    * <p>
    * The 5% sweep is FRAMEWORK.md §1's warning made a number: the fit does not converge, the answer
    * is 35 mm wrong, and {@code σ₃} has fallen only 7.6× -- <b>less than an order of magnitude for
    * a hundredfold loss of accuracy</b>. Read {@code isConverged()}, not {@code σ₃}.
    * </p>
    */
   @Test
   public void testNarrowSweepCollapsesBaseStepConditioning() throws Exception
   {
      Fitted full = fit(new RobotCaptures.Options().captures(30).noise(SIGMA_TARGET));
      Fitted narrow = fit(new RobotCaptures.Options().captures(30).noise(SIGMA_TARGET).excursionFraction(0.05));

      verbose("full sweep:   σ₃ %.4e, layout %.4f mm, converged %s", full.report().getBaseStepSigma3(), 1000 * worstLayoutError(full),
              full.report().isConverged());
      verbose("narrow sweep: σ₃ %.4e, layout %.4f mm, converged %s", narrow.report().getBaseStepSigma3(), 1000 * worstLayoutError(narrow),
              narrow.report().isConverged());

      // Measured ratio 7.6x. The threshold is 3x, deliberately loose.
      assertTrue(narrow.report().getBaseStepSigma3() < full.report().getBaseStepSigma3() / 3.0,
                 "A 5% sweep must collapse the base step's conditioning: narrow σ₃ = " + narrow.report().getBaseStepSigma3() + ", full σ₃ = "
                       + full.report().getBaseStepSigma3());

      // And the accuracy goes with it. Measured 0.31 mm -> 34.96 mm.
      assertTrue(worstLayoutError(narrow) > 10.0 * worstLayoutError(full),
                 "The narrow sweep's layout error should be far worse: " + 1000 * worstLayoutError(narrow) + " mm vs " + 1000 * worstLayoutError(full) + " mm.");

      // The trap: it does not report failure through J. It reports it through isConverged().
      assertTrue(!narrow.report().isConverged(), "The narrow sweep hits the iteration cap -- that, not σ₃, is the signal.");
      assertTrue(full.report().isConverged());
   }

   // ------------------------------------------------------------------------------------------
   // 4. The runtime side.
   // ------------------------------------------------------------------------------------------

   /**
    * <b>The headline.</b> With the pelvis and six leg links marked, 53.4925 kg of Alex's 91.5126 kg
    * -- <b>58.45%</b> -- has its pose from FK rather than from markers.
    * <p>
    * {@code TORSO_LINK} is 22.21 kg of that, 24.3% of the whole robot, and it chains directly off
    * the pelvis through one {@code SPINE_Z} joint. FRAMEWORK.md §1 says that joint "carries the full
    * load in tension with off-axis deflection the URDF does not model" -- which is its reason for
    * refusing to make the torso the gauge, and is equally a reason not to leave the torso chained.
    * Everything above the torso (head, both arms: 18.6 kg) chains through it as well, and
    * {@code KinematicChainCoupler} attributes all of it to {@code PELVIS_LINK} because that is the
    * nearest <i>marked</i> ancestor.
    * </p>
    * <p>
    * Adding one torso cluster takes the chained fraction from 58.45% to 34.18%. That is the
    * single highest-leverage marker decision available after the gauge itself.
    * </p>
    */
   @Test
   public void testChainedMassIsTheMajorityOfTheRobot() throws Exception
   {
      RobotModelHandle model = RobotCaptures.alexModel();
      List<String> linkNames = model.getLinkNames();

      KinematicChainCoupler primary = new KinematicChainCoupler(model, linkNames, List.of(RobotCaptures.PRIMARY_MARKED_LINKS));

      assertEquals(23, primary.getChainedLinkCount(), "30 links, 7 marked.");
      assertEquals(53.4925, primary.getChainedMass(), 1.0e-3);
      assertEquals(0.5845, primary.getChainedMass() / model.getTotalMass(), 1.0e-3, "58.45% of the robot rests on encoders.");

      // TORSO_LINK chains from the pelvis through SPINE_Z, and it is a quarter of the robot.
      boolean torsoFound = false;

      for (int i = 0; i < primary.getChainedLinkCount(); i++)
      {
         if (primary.getChainedLinkName(i).equals("TORSO_LINK"))
         {
            assertEquals("PELVIS_LINK", primary.getAncestorName(i), "TORSO_LINK's nearest marked ancestor is the gauge.");
            torsoFound = true;
         }

         verbose("  chained %-24s <- %s", primary.getChainedLinkName(i), primary.getAncestorName(i));
      }

      assertTrue(torsoFound, "TORSO_LINK must be chained when only the pelvis and legs are marked.");
      assertEquals(22.21, model.getMass("TORSO_LINK"), 1.0e-6);
      assertEquals(0.2427, model.getMass("TORSO_LINK") / model.getTotalMass(), 1.0e-3, "TORSO_LINK alone is 24.3% of Alex.");

      // What one more cluster buys.
      List<String> withTorso = new ArrayList<>(List.of(RobotCaptures.PRIMARY_MARKED_LINKS));
      withTorso.add("TORSO_LINK");
      KinematicChainCoupler improved = new KinematicChainCoupler(model, linkNames, withTorso);

      assertEquals(31.2825, improved.getChainedMass(), 1.0e-3);
      verbose("chained mass: %.4f kg (%.2f%%) with 7 clusters, %.4f kg (%.2f%%) with a torso cluster added", primary.getChainedMass(),
              100 * primary.getChainedMass() / model.getTotalMass(), improved.getChainedMass(), 100 * improved.getChainedMass() / model.getTotalMass());

      assertTrue(improved.getChainedMass() < 0.6 * primary.getChainedMass(), "One torso cluster should remove more than 40% of the chained mass.");
   }

   /**
    * <b>The test that closes the loop on the real model.</b> F6 → F7 → F9 against Mecano's own
    * {@code CenterOfMassCalculator}, on a 91.5 kg tree, to 1e-9.
    * <p>
    * The two computations share no code: one goes marker positions → Umeyama → per-link poses →
    * FK chaining → mass-weighted sum, the other goes joint angles → Mecano's own recursion.
    * Measured worst disagreement over 8 captures: <b>1.11e-15 m</b>, and both agree on
    * {@code M = 91.512588 kg}.
    * </p>
    * <p>
    * This is also the assertion that would catch a NaN {@code ^i c_i} escaping the fixed-joint
    * merge, since Mecano's calculator reads the same inertias by a different route.
    * </p>
    */
   @Test
   public void testCenterOfMassMatchesMecanoOnTheRealModel() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(8).noise(0.0).marked(RobotCaptures.LEG_LINKS));
      RobotModelHandle model = fitted.planted().model;

      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("demonstration; §11 requires a measured tilt on real data"),
                                                                   ReferenceFrame.getWorldFrame(),
                                                                   "_alexDemo");

      CenterOfMassGroundTruth groundTruth = CenterOfMassGroundTruth.forWholeRobot(model, world.getMotiveWorld(), world.getGravityAlignedWorld());
      LinkPoseEstimator estimator = new LinkPoseEstimator(fitted.calibration(), fitted.planted().clusters, model.getLinkNames());
      KinematicChainCoupler coupler = new KinematicChainCoupler(model, model.getLinkNames(), List.of(RobotCaptures.LEG_LINKS));

      CenterOfMassCalculator oracle = new CenterOfMassCalculator(model.getRootBody(), ReferenceFrame.getWorldFrame());
      JointBasics floatingJoint = model.getRootBody().getChildrenJoints().get(0);

      Point3D measured = new Point3D();
      double worst = 0.0;

      for (int k = 0; k < fitted.planted().captureSet.getCaptureCount(); k++)
      {
         MeasuredLinkPoses poses = new MeasuredLinkPoses(model.getLinkNames());
         estimator.estimate(fitted.planted().captureSet.getCapture(k).getMocapFrame(), poses);
         coupler.complete(fitted.planted().captureSet.getCapture(k).getEncoderSample(), poses);

         assertTrue(groundTruth.compute(poses, measured), "Every link should be measurable or chainable on noiseless data, capture " + k);

         model.setConfiguration(fitted.planted().captureSet.getCapture(k).getEncoderSample());
         floatingJoint.setJointConfiguration(fitted.planted().basePoses[k]);
         model.updateFrames();
         oracle.reset();

         worst = Math.max(worst, new Point3D(oracle.getCenterOfMass()).distance(measured));
      }

      verbose("CoM vs Mecano over 8 captures: worst disagreement %.3e m", worst);

      assertTrue(worst < 1.0e-9, "CoM disagrees with Mecano by " + worst + " m.");
      assertEquals(91.512588, groundTruth.getTotalMass(), 1.0e-4);
      assertEquals(oracle.getTotalMass(), groundTruth.getTotalMass(), 1.0e-9, "Both must agree on M, or neither CoM means anything.");
   }

   // ------------------------------------------------------------------------------------------
   // 5. The gates.
   // ------------------------------------------------------------------------------------------

   /** G2 on clean data at the target noise: passes, and indicts nothing. */
   @Test
   public void testG2PassesOnCleanData() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(40).noise(SIGMA_TARGET));
      BootstrapSpreadGate gate = gate(fitted, SIGMA_TARGET);
      GateResult result = gate.run();

      assertTrue(result.isPassed(), "G2 should pass on clean data.\n" + summarise(result));

      double worstRatio = 0.0;

      for (BootstrapSpreadGate.SpreadDiagnosis diagnosis : gate.getDiagnoses())
      {
         worstRatio = Math.max(worstRatio, diagnosis.spreadMeters() / diagnosis.expectedSpreadMeters());
         assertTrue(diagnosis.indictment().startsWith("nothing"),
                    "Clean data should indict nothing, but " + diagnosis.markerName() + " indicts: " + diagnosis.indictment());
      }

      // Measured 1.470. G2's per-marker expectation includes the gauge term scaled by lever arm,
      // which is why a gate built on σ√3 alone would fire here on perfectly clean Alex data.
      verbose("G2 clean: worst spread / expected = %.3f", worstRatio);
      assertTrue(worstRatio < 3.0, "Worst spread/expected was " + worstRatio + ".\n" + summarise(result));
   }

   /**
    * G2 on a 0.5° {@code LEFT_HIP_Y} offset: fires, and the left branch spreads about twice as much
    * as its mirror image.
    *
    * <h2>Two things that differ from the toy</h2>
    * <p>
    * <b>The right branch fires too.</b> On the toy, the unaffected branch showed nothing. On Alex it
    * does show something -- {@code RIGHT_SHIN} 0.76 mm and {@code RIGHT_FOOT} 1.08 mm against
    * {@code LEFT_SHIN} 1.47 mm and {@code LEFT_FOOT} 2.29 mm. The reason is that G2 is handed the
    * <i>solved</i> {@code Δ}, and A′ absorbs part of a one-branch fault into {@code Δ}, which is
    * global. So the localisation claim that survives on a real robot is the <b>mirror
    * comparison</b>, not "the other branch is clean". RUNNING.md's advice to pass the solved
    * {@code Δ} for diagnosis is still right; this is the cost of it.
    * </p>
    * <p>
    * <b>At {@code σ = 0.3 mm} it does not fire at all.</b> Measured: {@code LEFT_SHIN} spreads
    * 2.57 mm against 1.52 mm expected -- a ratio of 1.7, under the 3σ threshold. Alex's gauge-driven
    * floor at the target noise is simply larger than a 0.5° fault. The indictment column still names
    * {@code LEFT_HIP_Y} correctly on {@code LEFT_THIGH}; the verdict does not. See
    * {@link #testG2DoesNotFireOnAHalfDegreeOffsetAtTheTargetNoise()}.
    * </p>
    */
   @Test
   public void testG2FiresOnAHipOffsetAndLocalisesToTheLeftBranch() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(40).noise(SIGMA_QUIET).jointOffset("LEFT_HIP_Y", Math.toRadians(0.5)));
      BootstrapSpreadGate gate = gate(fitted, SIGMA_QUIET);
      GateResult result = gate.run();

      assertTrue(!result.isPassed(), "G2 must fire on a 0.5° LEFT_HIP_Y offset.\n" + summarise(result));
      assertTrue(failed(result, "LEFT_SHIN"), "LEFT_SHIN sits two joints below the fault and must spread.\n" + summarise(result));
      assertTrue(failed(result, "LEFT_FOOT"), "LEFT_FOOT is the deepest link below the fault and must spread most.\n" + summarise(result));
      assertTrue(!failed(result, "PELVIS_LINK"), "The gauge is above the fault and must not spread.\n" + summarise(result));

      // Mirror comparison. Measured: LEFT_SHIN 1.473 vs RIGHT_SHIN 0.762 (1.93x),
      // LEFT_FOOT 2.293 vs RIGHT_FOOT 1.083 (2.12x). The threshold is a deliberately loose 1.4x.
      assertTrue(meanSpread(gate, "LEFT_SHIN") > 1.4 * meanSpread(gate, "RIGHT_SHIN"),
                 "LEFT_SHIN " + 1000 * meanSpread(gate, "LEFT_SHIN") + " mm vs RIGHT_SHIN " + 1000 * meanSpread(gate, "RIGHT_SHIN") + " mm.");
      assertTrue(meanSpread(gate, "LEFT_FOOT") > 1.4 * meanSpread(gate, "RIGHT_FOOT"),
                 "LEFT_FOOT " + 1000 * meanSpread(gate, "LEFT_FOOT") + " mm vs RIGHT_FOOT " + 1000 * meanSpread(gate, "RIGHT_FOOT") + " mm.");

      // And on the link immediately below the fault, the strongest correlate is the offender itself.
      // Measured r = 0.930.
      BootstrapSpreadGate.SpreadDiagnosis thigh = gate.findDiagnosis("LEFT_THIGH", "LEFT_THIGH_M0");
      assertNotNull(thigh);
      assertEquals("LEFT_HIP_Y", thigh.strongestJointName(), "The strongest correlate on LEFT_THIGH should be the offending joint.");
      assertTrue(Math.abs(thigh.strongestJointCorrelation()) >= BootstrapSpreadGate.STRUCTURE_CORRELATION_THRESHOLD,
                 "The correlation should read as structure, was " + thigh.strongestJointCorrelation());

      for (String link : RobotCaptures.PRIMARY_MARKED_LINKS)
         verbose("  mean spread %-14s %.4f mm", link, 1000 * meanSpread(gate, link));
   }

   /**
    * The same 0.5° offset at the <b>target</b> noise: G2 passes. This is a real limit of the setup,
    * not a bug, and it is asserted so that nobody reads a green G2 at the gantry as "no joint
    * offset".
    * <p>
    * Measured: the worst affected marker ({@code LEFT_FOOT}) spreads 3.59 mm against 2.08 mm
    * expected. Everything is bigger than on the toy -- Alex's foot is 0.89 m from the pelvis --
    * so both the signal and the floor grow, and their ratio does not clear 3σ.
    * </p>
    * <p>
    * The lever that changes this is the same one everywhere else in this file: a wider gauge
    * bracket, or lower {@code σ}. G2's sensitivity to a joint offset is proportional to the gauge
    * cluster's angular accuracy, not to the number of captures.
    * </p>
    */
   @Test
   public void testG2DoesNotFireOnAHalfDegreeOffsetAtTheTargetNoise() throws Exception
   {
      Fitted fitted = fit(new RobotCaptures.Options().captures(40).noise(SIGMA_TARGET).jointOffset("LEFT_HIP_Y", Math.toRadians(0.5)));
      BootstrapSpreadGate gate = gate(fitted, SIGMA_TARGET);
      GateResult result = gate.run();

      assertTrue(result.isPassed(),
                 "If this ever starts failing, G2 got more sensitive and the caveat in RUNNING.md should be revisited.\n" + summarise(result));

      // But the structure is there in the diagnoses even though the verdict is green. Measured:
      // LEFT_THIGH's correlate is LEFT_HIP_Y at r = 0.604, over the 0.5 structure threshold.
      BootstrapSpreadGate.SpreadDiagnosis thigh = gate.findDiagnosis("LEFT_THIGH", "LEFT_THIGH_M0");
      assertEquals("LEFT_HIP_Y", thigh.strongestJointName(), "The diagnosis still names the offender even when the verdict is a pass.");
      assertTrue(Math.abs(thigh.strongestJointCorrelation()) >= BootstrapSpreadGate.STRUCTURE_CORRELATION_THRESHOLD);

      verbose("G2 at σ=0.3 mm on a 0.5° offset: passed=%s, LEFT_FOOT mean spread %.4f mm", result.isPassed(), 1000 * meanSpread(gate, "LEFT_FOOT"));
   }

   /**
    * <b>The hardware finding.</b> At FRAMEWORK.md §1's recommended gauge bracket (120-150 mm) and
    * §17's target noise (0.3 mm), Alex's held-out marker RMS is <b>2.86 mm</b> -- above the TALOS
    * cross-validated 2.2 mm the framework names as the bar to beat -- <b>on synthetic data with a
    * perfect URDF and nothing wrong but mocap noise</b>.
    *
    * <h2>The law</h2>
    * <p>
    * Measured over three seeds at {@code K = 40}, held-out RMS in mm:
    * </p>
    *
    * <pre>
    *   gauge spread     σ = 0.93 mm   σ = 0.30 mm   σ = 0.10 mm
    *    60 mm              20.44          6.56          2.18
    *   140 mm               8.87          2.86          0.95
    *   200 mm               6.34          2.04          0.68
    *   300 mm               4.44          1.43          0.48
    * </pre>
    *
    * <p>
    * That is FRAMEWORK.md §1's {@code σ/(√N·r_perp)} to within a couple of percent in both
    * variables: it is linear in {@code σ} (0.93/0.30 = 3.10 against a measured 8.87/2.86 = 3.10) and
    * inverse in the spread (140/60 = 2.33 against 6.56/2.86 = 2.29). Collected:
    * </p>
    *
    * <pre>
    *   held-out RMS  ≈  2.86 mm · (σ / 0.3 mm) · (140 mm / gauge spread)
    * </pre>
    *
    * <p>
    * <b>So to clear 2.2 mm on Alex you need a gauge bracket of at least ~182 mm at σ = 0.3 mm, or
    * σ ≤ 0.23 mm at 140 mm.</b> §1's 120-150 mm recommendation was written without a lever arm in
    * it; Alex's pelvis-to-foot distance is 0.89 m against the toy's ~0.6 m, and that factor is
    * exactly what eats the margin. This is the cheapest accuracy available in the project and it is
    * a bracket, not a camera.
    * </p>
    */
   @Test
   public void testHeldOutResidualMissesTheTalosBarAtTheRecommendedBracketWidth() throws Exception
   {
      // Averaged over three seeds, not measured at one. A single seed puts the 140 mm case at
      // 2.2004 mm against a 2.2000 mm bar -- a 0.02% margin, which is a coin flip dressed as an
      // assertion. The seed-to-seed spread comes from the reference capture whose marker positions
      // BaseInitializer bakes into the gauge shape, and averaging is the honest way to remove it.
      double atRecommendedWidth = averagedHeldOutRms(0.14);
      double atWideBracket = averagedHeldOutRms(0.30);
      double atNarrowBracket = averagedHeldOutRms(0.06);

      verbose("held-out RMS at σ=0.3 mm, 3-seed mean: 60 mm bracket %.4f mm, 140 mm %.4f mm, 300 mm %.4f mm", 1000 * atNarrowBracket, 1000 * atRecommendedWidth,
              1000 * atWideBracket);

      // The finding. Measured 2.8559 mm against the 2.2 mm bar: 30% over.
      assertTrue(atRecommendedWidth > HeldOutResidualGate.TALOS_CROSS_VALIDATED_RMS_METERS,
                 "If this passes, either the geometry changed or the solver improved -- re-measure the table in this javadoc before relaxing anything. "
                       + "Held-out RMS was " + 1000 * atRecommendedWidth + " mm against the TALOS bar of 2.2 mm.");

      // And the gate is seen in its passing direction too, at a bracket wide enough to earn it.
      // Measured 1.4301 mm.
      assertTrue(atWideBracket < HeldOutResidualGate.TALOS_CROSS_VALIDATED_RMS_METERS,
                 "A 300 mm bracket should clear the TALOS bar comfortably, was " + 1000 * atWideBracket + " mm.");

      // 1/r_perp, over a 5x range of bracket widths. Predicted 140/60 = 2.33 and 300/140 = 2.14;
      // measured 2.296 and 1.997.
      assertEquals(2.33, atNarrowBracket / atRecommendedWidth, 0.6, "Held-out RMS should scale as 1/r_perp.");
      assertEquals(2.14, atRecommendedWidth / atWideBracket, 0.6, "Held-out RMS should scale as 1/r_perp.");
   }

   /** Held-out RMS at one gauge spread, averaged over three seeds. See the caller for why. */
   private static double averagedHeldOutRms(double gaugeSpread) throws Exception
   {
      double total = 0.0;
      int seeds = 3;

      for (int seed = 0; seed < seeds; seed++)
         total += heldOutRms(new RobotCaptures.Options().seed(500L + seed).captures(40).noise(SIGMA_TARGET).gaugeSpread(gaugeSpread));

      return total / seeds;
   }

   // ------------------------------------------------------------------------------------------
   // Helpers.
   // ------------------------------------------------------------------------------------------

   private static double clusterLayoutError(Fitted fitted, String linkName)
   {
      ClusterLayout truth = fitted.planted().plantedLayout(linkName);
      ClusterLayout estimate = fitted.calibration().getLayout(linkName);
      double worst = 0.0;

      for (int j = 0; j < truth.getMarkerCount(); j++)
      {
         if (estimate.getObservationCount(j) == 0)
            continue;

         worst = Math.max(worst, new Point3D(truth.getPositionInLinkFrame(j)).distance(new Point3D(estimate.getPositionInLinkFrame(j))));
      }

      return worst;
   }

   private static BootstrapSpreadGate gate(Fitted fitted, double sigma)
   {
      return new BootstrapSpreadGate(fitted.planted().captureSet.getCaptures(),
                                     fitted.planted().clusters,
                                     fitted.planted().model,
                                     fitted.calibration().getClusterToBase(),
                                     RobotCaptures.clusterPoseList(fitted.tracking(), fitted.planted().captureSet.getCaptureCount()),
                                     sigma);
   }

   private static double meanSpread(BootstrapSpreadGate gate, String linkName)
   {
      double total = 0.0;
      int count = 0;

      for (BootstrapSpreadGate.SpreadDiagnosis diagnosis : gate.getDiagnoses())
      {
         if (diagnosis.linkName().equals(linkName))
         {
            total += diagnosis.spreadMeters();
            count++;
         }
      }

      return count == 0 ? Double.NaN : total / count;
   }

   private static boolean failed(GateResult result, String linkName)
   {
      for (GateResult.Finding finding : result.getFailures())
      {
         if (finding.subject().startsWith(linkName + ":"))
            return true;
      }

      return false;
   }

   /** Fits on the even-numbered captures and reports G4's held-out RMS on the odd-numbered ones. */
   private static double heldOutRms(RobotCaptures.Options options) throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(options);
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);

      List<Integer> trainingList = new ArrayList<>();
      List<Capture> heldOutCaptures = new ArrayList<>();
      List<RigidBodyTransformReadOnly> heldOutPoses = new ArrayList<>();

      for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
      {
         if (k % 2 == 0)
         {
            trainingList.add(k);
         }
         else
         {
            heldOutCaptures.add(planted.captureSet.getCapture(k));
            heldOutPoses.add(tracking.isUsable(k) ? tracking.getClusterToWorld(k) : null);
         }
      }

      // The tracking is shared between the splits on purpose: recomputing it per split would put
      // the two fits in different cluster-frame conventions and make the comparison meaningless.
      CalibrationReport report = new CalibrationReport();
      CalibrationResult calibration = new AlternatingCalibrator().calibrate(planted.captureSet,
                                                                           planted.model,
                                                                           tracking,
                                                                           trainingList.stream().mapToInt(Integer::intValue).toArray(),
                                                                           report);

      HeldOutResidualGate gate = new HeldOutResidualGate(heldOutCaptures,
                                                        planted.clusters,
                                                        planted.model,
                                                        calibration,
                                                        heldOutPoses,
                                                        report.getOverallRmsMeters(),
                                                        HeldOutResidualGate.TALOS_CROSS_VALIDATED_RMS_METERS);
      gate.run();
      return gate.getHeldOutRmsMeters();
   }

   private static String summarise(GateResult result)
   {
      StringBuilder text = new StringBuilder(result.toString()).append('\n').append("  ").append(result.getSummary()).append('\n');

      for (GateResult.Finding finding : result.getFindings())
         text.append(String.format("    %-8s %-32s %s%n", finding.status(), finding.subject(), finding.detail()));

      return text.toString();
   }
}
