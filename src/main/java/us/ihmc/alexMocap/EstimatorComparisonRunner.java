package us.ihmc.alexMocap;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.NearestTimeMatcher;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

/**
 * Offline: a mocap ground-truth pelvis trajectory (from {@code ReplayRunner --velocity}) plus an
 * estimator's own logged pelvis state, in -- position/orientation/velocity error statistics out.
 * <p>
 * <b>This tool has no idea what state estimator produced the log it reads.</b> It was written
 * before the InEKF work (a different branch of a different repo -- see the mocap-vs-estimator
 * branch-combination notes) was available to inspect, on purpose: the comparison itself -- time
 * alignment, frame discipline, error statistics -- does not depend on any estimator's internals,
 * only on both sides sharing an epoch and a documented pelvis-state CSV schema (below). Whatever
 * exports an estimator's own state to that schema is a separate, small adapter to write once that
 * estimator exists in this build.
 * </p>
 *
 * <h2>The estimator log schema this tool expects</h2>
 * <pre>
 * # comment lines start with '#' and are ignored, same as every other CSV in this project
 * timestamp_ns,x,y,z,qx,qy,qz,qs,vx,vy,vz,wx,wy,wz
 * </pre>
 * <p>
 * {@code timestamp_ns} must share an epoch with the mocap capture the ground truth was replayed
 * from -- exactly the same requirement {@code Capture}'s own mocap/encoder pairing has, and for the
 * same reason (FRAMEWORK.md §18.3): a mismatched epoch pairs samples that were never simultaneous
 * and the result reads as estimator error rather than as a bookkeeping mistake.
 * </p>
 * <p>
 * {@code x,y,z,qx,qy,qz,qs} is the estimator's own pelvis pose estimate, in whatever world frame it
 * publishes in. {@code vx,vy,vz,wx,wy,wz} is that frame's twist, <b>assumed expressed in the world
 * frame</b> (matching {@code PelvisTwistEstimator}'s own convention on the ground-truth side) --
 * verify this against the estimator being compared before trusting a velocity error number.
 * </p>
 *
 * <h2>FRAMEWORK.md §13's three-pelvis-frames hazard, made explicit rather than left silent</h2>
 * <p>
 * Ground truth is the twist of the <b>URDF pelvis link frame</b>. If the estimator instead
 * publishes its IMU mounting frame's twist, the two disagree by {@code ω × r} for the lever arm
 * {@code r} between the two frames -- at 1 rad/s and 10 cm that is 0.1 m/s, "enough to swamp the
 * entire comparison. It reads as an estimator regression, not as a bookkeeping error." {@code
 * --estimator-frame IMU --imu-offset x,y,z} applies that correction; the default, {@code
 * --estimator-frame PELVIS_LINK}, applies none. Get {@code --imu-offset} wrong -- it is described in
 * FRAMEWORK.md as "an unverified CAD number" -- and this tool will confidently report the wrong
 * velocity error, which is exactly the failure mode this option exists to let a caller correct for
 * rather than silently commit to a guess.
 * </p>
 *
 * <h2>World-frame velocity is NOT alignment-free: the body-frame columns</h2>
 * <p>
 * Velocity does not depend on where the two world frames put their <em>origin</em>, and consumers of
 * this tool took that to mean it does not depend on their alignment at all. It does: it depends on
 * their relative <b>yaw</b>. If the estimator's world is rotated by ψ about gravity relative to the
 * mocap world, the world-frame error is {@code (Rz(ψ) - I) v}, which grows with speed and with ψ. An
 * invariant EKF's global yaw is unobservable and drifts, and the mocap-to-robot yaw registration has
 * a history of large offsets, so this is not a small-print hazard.
 * </p>
 * <p>
 * The body-frame columns remove it: {@code e_v = R_estᵀ v_est − R_trueᵀ v_true}, and likewise for ω.
 * A yaw offset between the worlds rotates both {@code R} and {@code v} on one side together, so it
 * cancels exactly. This is the quantity the offline training objective uses ({@code body_velocity_l2}),
 * so a number fitted there and a number scored here now mean the same thing. The attitude counterpart
 * is the <b>tilt</b> error -- the angle between the two estimated gravity directions in body frame --
 * which is the observable (roll/pitch) part of the orientation error; the full quaternion angle
 * includes yaw and has the same alignment dependence as position.
 * </p>
 * <p>
 * Body-frame twist needs the ground-truth orientation at each ground-truth twist sample. Both come
 * from the same {@code ReplayRunner} run and share timestamps; a twist sample with no pose sample
 * within the time tolerance is dropped and counts against the twist match fraction.
 * </p>
 */
public class EstimatorComparisonRunner
{
   private static final int EXIT_OK = 0;
   private static final int EXIT_INCOMPLETE = 1;
   private static final int EXIT_USAGE = 2;

   public static void main(String[] args)
   {
      System.exit(run(args, System.out, System.err));
   }

   public static int run(String[] args, PrintStream out, PrintStream err)
   {
      Arguments arguments;

      try
      {
         arguments = Arguments.parse(args);
      }
      catch (IllegalArgumentException e)
      {
         err.println("error: " + e.getMessage());
         err.println();
         printUsage(err);
         return EXIT_USAGE;
      }

      if (arguments.help)
      {
         printUsage(out);
         return EXIT_OK;
      }

      try
      {
         return compare(arguments, out, err);
      }
      catch (IOException e)
      {
         err.println("error: " + e.getMessage());
         return EXIT_USAGE;
      }
      catch (IllegalArgumentException | IllegalStateException e)
      {
         err.println("error: " + e.getMessage());
         return EXIT_USAGE;
      }
   }

   // ---------------------------------------------------------------------------------------------
   // Programmatic API: the same comparison, as numbers rather than as a printed report.
   // ---------------------------------------------------------------------------------------------

   /**
    * Compares one estimator log against mocap ground truth and returns the error statistics.
    *
    * <p>This is the CLI's own comparison with the printing and the per-sample CSV writing left out,
    * for callers that need the numbers -- a multi-arm scoring table, most obviously, where re-deriving
    * these statistics or scraping them back out of the printed report would be a second
    * implementation of them (FRAMEWORK.md §2).</p>
    *
    * <p>Note what this deliberately does <b>not</b> do: it applies no IMU-to-pelvis correction, so the
    * caller is asserting the estimator log is already in the pelvis-link frame. It also does not
    * judge the match fraction -- {@link Comparison#poseMatchFraction()} is returned rather than
    * enforced, because a caller scoring several arms wants to see a bad one in its table next to the
    * good ones, not have the run aborted.</p>
    */
   public static Comparison compare(Path groundTruthPose, Path groundTruthVelocity, Path estimatorLog, long maxTimeDeltaNanoseconds)
         throws IOException
   {
      List<PoseSample> truthPoses = readPoseCsv(groundTruthPose);
      List<TwistSample> truthTwists = readTwistCsv(groundTruthVelocity);
      List<EstimatorSample> estimatorSamples = readEstimatorCsv(estimatorLog);

      if (truthPoses.isEmpty())
         throw new IllegalArgumentException(groundTruthPose + " has no pose rows.");
      if (estimatorSamples.isEmpty())
         throw new IllegalArgumentException(estimatorLog + " has no rows.");

      List<PoseErrorSample> poseErrors = matchPoses(truthPoses, estimatorSamples, maxTimeDeltaNanoseconds);
      List<TwistErrorSample> twistErrors = matchTwists(truthTwists, truthPoses, estimatorSamples, maxTimeDeltaNanoseconds);

      return new Comparison(poseErrors.size(),
                            truthPoses.size(),
                            twistErrors.size(),
                            truthTwists.size(),
                            Stats.of(poseErrors.stream().mapToDouble(e -> e.positionError.norm()).toArray()),
                            Stats.of(poseErrors.stream().mapToDouble(e -> Math.toDegrees(e.orientationErrorRadians)).toArray()),
                            Stats.of(twistErrors.stream().mapToDouble(e -> e.linearVelocityError.norm()).toArray()),
                            Stats.of(twistErrors.stream().mapToDouble(e -> e.angularVelocityError.norm()).toArray()),
                            Stats.of(poseErrors.stream().mapToDouble(e -> Math.toDegrees(e.tiltErrorRadians)).toArray()),
                            Stats.of(twistErrors.stream().mapToDouble(e -> e.bodyLinearVelocityError.norm()).toArray()),
                            Stats.of(twistErrors.stream().mapToDouble(e -> e.bodyAngularVelocityError.norm()).toArray()));
   }

   /**
    * Error statistics from one estimator-log-versus-ground-truth comparison.
    *
    * <p>Only {@link #tiltErrorDegrees}, {@link #bodyLinearVelocityErrorNorm} and
    * {@link #bodyAngularVelocityErrorNorm} are independent of the relative yaw between the estimator's
    * and mocap's world frames; see the class javadoc. Position, full orientation and the world-frame
    * twist statistics all assume the two worlds are aligned, and an invariant EKF's yaw is not.</p>
    *
    * <p>The match counts travel with the statistics on purpose. An error number computed from a
    * handful of coincidental time matches is not a small-sample version of the right answer, it is a
    * different quantity, and the fraction is the only thing that distinguishes them -- so a report
    * built from this record can show them together rather than presenting a plausible number whose
    * provenance has been dropped.</p>
    */
   public record Comparison(int matchedPoseSamples,
                            int groundTruthPoseSamples,
                            int matchedTwistSamples,
                            int groundTruthTwistSamples,
                            Stats positionErrorNorm,
                            Stats orientationErrorDegrees,
                            Stats linearVelocityErrorNorm,
                            Stats angularVelocityErrorNorm,
                            Stats tiltErrorDegrees,
                            Stats bodyLinearVelocityErrorNorm,
                            Stats bodyAngularVelocityErrorNorm)
   {
      public double poseMatchFraction()
      {
         return groundTruthPoseSamples == 0 ? Double.NaN : (double) matchedPoseSamples / groundTruthPoseSamples;
      }

      public double twistMatchFraction()
      {
         return groundTruthTwistSamples == 0 ? Double.NaN : (double) matchedTwistSamples / groundTruthTwistSamples;
      }
   }

   /**
    * Summary statistics of one error series. {@link #rmse()} is the paper number; {@link #max()} is
    * the magnitude of the largest deviation, so it is never negative even when {@link #mean()} is.
    *
    * <p>Empty input gives all-NaN rather than zero: an unmatched arm has no error, and reporting 0.0
    * would rank it best.</p>
    */
   public record Stats(double rmse, double mean, double max, double standardDeviation)
   {
      public static Stats of(double[] values)
      {
         if (values.length == 0)
            return new Stats(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

         double sumOfSquares = 0.0;
         double sum = 0.0;
         double max = 0.0;

         for (double value : values)
         {
            sumOfSquares += value * value;
            sum += value;
            max = Math.max(max, Math.abs(value));
         }

         double mean = sum / values.length;
         double variance = 0.0;

         for (double value : values)
            variance += (value - mean) * (value - mean);

         return new Stats(Math.sqrt(sumOfSquares / values.length), mean, max, Math.sqrt(variance / values.length));
      }
   }

   private static int compare(Arguments arguments, PrintStream out, PrintStream err) throws IOException
   {
      List<PoseSample> truthPoses = readPoseCsv(arguments.groundTruthPelvis);
      List<TwistSample> truthTwists = readTwistCsv(arguments.groundTruthVelocity);
      List<EstimatorSample> estimatorSamples = readEstimatorCsv(arguments.estimatorLog);

      if (truthPoses.isEmpty())
         throw new IllegalArgumentException(arguments.groundTruthPelvis + " has no pose rows.");
      if (estimatorSamples.isEmpty())
         throw new IllegalArgumentException(arguments.estimatorLog + " has no rows.");

      if (arguments.estimatorFrame == EstimatorFrame.IMU)
      {
         out.println("Correcting estimator velocity from the IMU frame to the pelvis link frame using --imu-offset "
                     + arguments.imuOffset + " (a fixed offset FRAMEWORK.md section 13 calls \"an unverified CAD number\" -- verify it).");
         out.println();
         correctImuToPelvis(estimatorSamples, arguments.imuOffset);
      }

      out.println("ground truth pose      " + arguments.groundTruthPelvis + "  (" + truthPoses.size() + " samples)");
      out.println("ground truth velocity  " + arguments.groundTruthVelocity + "  (" + truthTwists.size() + " finite samples)");
      out.println("estimator log          " + arguments.estimatorLog + "  (" + estimatorSamples.size() + " samples)");
      out.println("estimator frame        " + arguments.estimatorFrame);
      out.println("max time delta         " + (arguments.maxTimeDeltaNanoseconds / 1.0e6) + " ms");
      out.println();

      List<PoseErrorSample> poseErrors = matchPoses(truthPoses, estimatorSamples, arguments.maxTimeDeltaNanoseconds);
      List<TwistErrorSample> twistErrors = matchTwists(truthTwists, truthPoses, estimatorSamples, arguments.maxTimeDeltaNanoseconds);

      double poseMatchFraction = truthPoses.isEmpty() ? 0.0 : (double) poseErrors.size() / truthPoses.size();
      double twistMatchFraction = truthTwists.isEmpty() ? Double.NaN : (double) twistErrors.size() / truthTwists.size();

      out.println(String.format("matched %d of %d pose samples (%.1f%%) within the time delta", poseErrors.size(), truthPoses.size(),
                                100.0 * poseMatchFraction));

      if (!truthTwists.isEmpty())
         out.println(String.format("matched %d of %d velocity samples (%.1f%%) within the time delta", twistErrors.size(), truthTwists.size(),
                                   100.0 * twistMatchFraction));

      out.println();
      out.print(formatPoseReport(poseErrors));

      if (!twistErrors.isEmpty())
      {
         out.println();
         out.print(formatTwistReport(twistErrors));
      }

      if (arguments.outputDirectory != null)
      {
         Files.createDirectories(arguments.outputDirectory);
         writeComparisonCsv(arguments.outputDirectory.resolve("poseError.csv"), poseErrors);

         if (!twistErrors.isEmpty())
            writeTwistErrorCsv(arguments.outputDirectory.resolve("velocityError.csv"), twistErrors);

         out.println();
         out.println("wrote poseError.csv" + (twistErrors.isEmpty() ? "" : " and velocityError.csv") + " to " + arguments.outputDirectory);
      }

      // A match rate this low means the two logs barely overlap in time at all -- almost certainly
      // an epoch mismatch (FRAMEWORK.md §18.3's silent failure again) rather than a real result, and
      // reporting error statistics from a handful of coincidental matches would be misleading rather
      // than merely imprecise.
      if (poseMatchFraction < arguments.minimumMatchFraction)
      {
         err.println();
         err.println(String.format("warning: only %.1f%% of pose samples matched -- below --min-match-fraction %.1f%%. "
                                   + "Check that both logs share an epoch (FRAMEWORK.md section 18.3).",
                                   100.0 * poseMatchFraction,
                                   100.0 * arguments.minimumMatchFraction));
         return EXIT_INCOMPLETE;
      }

      return EXIT_OK;
   }

   // ---------------------------------------------------------------------------------------------
   // IMU -> pelvis-link velocity correction
   // ---------------------------------------------------------------------------------------------

   /**
    * {@code v_pelvis = v_imu + ω × r}, where {@code r} is the IMU-to-pelvis offset rotated into the
    * world frame by the sample's own orientation estimate. Both points are fixed on the same rigid
    * body, so no correction is needed for angular velocity -- ω is the same for every point on it.
    */
   private static void correctImuToPelvis(List<EstimatorSample> samples, Vector3DReadOnly imuToPelvisInBodyFrame)
   {
      Vector3D leverArmInWorld = new Vector3D();
      Vector3D correction = new Vector3D();

      for (EstimatorSample sample : samples)
      {
         sample.orientation.transform(imuToPelvisInBodyFrame, leverArmInWorld);
         correction.cross(sample.angularVelocity, leverArmInWorld);
         sample.linearVelocity.add(correction);
      }
   }

   // ---------------------------------------------------------------------------------------------
   // Matching: nearest estimator sample in time, within the tolerance, per ground-truth sample.
   // ---------------------------------------------------------------------------------------------

   /**
    * Nearest-neighbour in time, not interpolation. Interpolating the estimator's own trajectory to
    * match ground-truth timestamps would smooth over exactly the estimator behaviour (e.g. a step
    * change on contact) that this comparison exists to reveal. The actual search is
    * {@link NearestTimeMatcher} -- shared with {@code alex}'s NEES consistency checker, which needs
    * the identical estimator-log samples aligned against ground truth for the same reason (see that
    * class's javadoc) and must not carry a second copy of this bookkeeping (FRAMEWORK.md §2).
    */
   private static List<PoseErrorSample> matchPoses(List<PoseSample> truthPoses, List<EstimatorSample> estimatorSamples, long maxDeltaNanoseconds)
   {
      List<PoseErrorSample> errors = new ArrayList<>();

      for (NearestTimeMatcher.Matched<PoseSample, EstimatorSample> matched : NearestTimeMatcher.match(truthPoses,
                                                                                                        estimatorSamples,
                                                                                                        PoseSample::timestampNanoseconds,
                                                                                                        estimate -> estimate.timestampNanoseconds,
                                                                                                        maxDeltaNanoseconds))
      {
         PoseSample truth = matched.truth();
         EstimatorSample estimate = matched.candidate();

         Vector3D positionError = new Vector3D();
         positionError.sub(estimate.position, truth.position);

         double orientationErrorRadians = truth.orientation.distance(estimate.orientation);
         double tiltErrorRadians = tiltError(truth.orientation, estimate.orientation);

         errors.add(new PoseErrorSample(truth.timestampNanoseconds,
                                        matched.timeDeltaNanoseconds(),
                                        positionError,
                                        orientationErrorRadians,
                                        tiltErrorRadians));
      }

      return errors;
   }

   /**
    * Twist errors in both frames. The world-frame pair is kept for continuity and for debugging a
    * registration; the body-frame pair is the one to report, because a yaw offset between the two
    * world frames cancels out of it exactly (class javadoc).
    */
   private static List<TwistErrorSample> matchTwists(List<TwistSample> truthTwists,
                                                     List<PoseSample> truthPoses,
                                                     List<EstimatorSample> estimatorSamples,
                                                     long maxDeltaNanoseconds)
   {
      // Ground-truth orientation for each ground-truth twist sample. Same ReplayRunner run, so the
      // timestamps coincide; the tolerance only guards against a mis-paired file.
      List<OrientedTwistSample> orientedTruth = new ArrayList<>();

      for (NearestTimeMatcher.Matched<TwistSample, PoseSample> matched : NearestTimeMatcher.match(truthTwists,
                                                                                                    truthPoses,
                                                                                                    TwistSample::timestampNanoseconds,
                                                                                                    PoseSample::timestampNanoseconds,
                                                                                                    maxDeltaNanoseconds))
      {
         TwistSample twist = matched.truth();
         orientedTruth.add(new OrientedTwistSample(twist.timestampNanoseconds, twist.linearVelocity, twist.angularVelocity, matched.candidate().orientation));
      }

      List<TwistErrorSample> errors = new ArrayList<>();

      for (NearestTimeMatcher.Matched<OrientedTwistSample, EstimatorSample> matched : NearestTimeMatcher.match(orientedTruth,
                                                                                                                 estimatorSamples,
                                                                                                                 OrientedTwistSample::timestampNanoseconds,
                                                                                                                 estimate -> estimate.timestampNanoseconds,
                                                                                                                 maxDeltaNanoseconds))
      {
         OrientedTwistSample truth = matched.truth();
         EstimatorSample estimate = matched.candidate();

         Vector3D linearError = new Vector3D();
         linearError.sub(estimate.linearVelocity, truth.linearVelocity);
         Vector3D angularError = new Vector3D();
         angularError.sub(estimate.angularVelocity, truth.angularVelocity);

         Vector3D bodyLinearError = bodyFrameDifference(estimate.orientation, estimate.linearVelocity, truth.orientation, truth.linearVelocity);
         Vector3D bodyAngularError = bodyFrameDifference(estimate.orientation, estimate.angularVelocity, truth.orientation, truth.angularVelocity);

         errors.add(new TwistErrorSample(truth.timestampNanoseconds,
                                         matched.timeDeltaNanoseconds(),
                                         linearError,
                                         angularError,
                                         bodyLinearError,
                                         bodyAngularError));
      }

      return errors;
   }

   /**
    * {@code R_estᵀ a_est − R_trueᵀ a_true}: each world-frame vector expressed in its own estimate of the
    * body frame, then differenced. A common rotation about gravity applied to one side's {@code R} and
    * vector together leaves its body-frame vector unchanged, which is the whole point.
    */
   static Vector3D bodyFrameDifference(Quaternion estimateOrientation, Vector3D estimateWorld, Quaternion truthOrientation, Vector3D truthWorld)
   {
      Vector3D estimateBody = new Vector3D();
      estimateOrientation.inverseTransform(estimateWorld, estimateBody);
      Vector3D truthBody = new Vector3D();
      truthOrientation.inverseTransform(truthWorld, truthBody);

      Vector3D difference = new Vector3D();
      difference.sub(estimateBody, truthBody);
      return difference;
   }

   /**
    * Angle between the two estimates of the gravity direction, each in its own body frame -- the
    * roll/pitch part of the attitude error. {@code Rz(ψ) e_z = e_z}, so a yaw offset between the worlds
    * does not appear here, while it does appear in {@link Quaternion#distance}.
    */
   static double tiltError(Quaternion truthOrientation, Quaternion estimateOrientation)
   {
      Vector3D up = new Vector3D(0.0, 0.0, 1.0);
      Vector3D truthUpInBody = new Vector3D();
      truthOrientation.inverseTransform(up, truthUpInBody);
      Vector3D estimateUpInBody = new Vector3D();
      estimateOrientation.inverseTransform(up, estimateUpInBody);

      Vector3D cross = new Vector3D();
      cross.cross(truthUpInBody, estimateUpInBody);
      // atan2 rather than acos: acos loses all precision near zero, which is where a good estimator lives.
      return Math.atan2(cross.norm(), truthUpInBody.dot(estimateUpInBody));
   }

   // ---------------------------------------------------------------------------------------------
   // Statistics and reporting
   // ---------------------------------------------------------------------------------------------

   private static String formatPoseReport(List<PoseErrorSample> errors)
   {
      if (errors.isEmpty())
         return "no matched pose samples -- nothing to report\n";

      double[] x = errors.stream().mapToDouble(e -> e.positionError.getX()).toArray();
      double[] y = errors.stream().mapToDouble(e -> e.positionError.getY()).toArray();
      double[] z = errors.stream().mapToDouble(e -> e.positionError.getZ()).toArray();
      double[] norm = errors.stream().mapToDouble(e -> e.positionError.norm()).toArray();
      double[] orientationDegrees = errors.stream().mapToDouble(e -> Math.toDegrees(e.orientationErrorRadians)).toArray();

      StringBuilder text = new StringBuilder();
      text.append("position error (mm)\n");
      text.append(String.format("  %-8s %10s %10s %10s %10s%n", "", "rmse", "mean", "max", "std"));
      appendStatsRow(text, "x", x, 1000.0);
      appendStatsRow(text, "y", y, 1000.0);
      appendStatsRow(text, "z", z, 1000.0);
      appendStatsRow(text, "norm", norm, 1000.0);
      text.append('\n');
      text.append("orientation error (deg) -- full angle includes yaw and assumes the world frames are aligned\n");
      appendStatsRow(text, "angle", orientationDegrees, 1.0);
      text.append('\n');
      text.append("tilt error (deg) -- roll/pitch only, independent of world-frame yaw\n");
      appendStatsRow(text, "tilt", errors.stream().mapToDouble(e -> Math.toDegrees(e.tiltErrorRadians)).toArray(), 1.0);

      return text.toString();
   }

   private static String formatTwistReport(List<TwistErrorSample> errors)
   {
      double[] vx = errors.stream().mapToDouble(e -> e.linearVelocityError.getX()).toArray();
      double[] vy = errors.stream().mapToDouble(e -> e.linearVelocityError.getY()).toArray();
      double[] vz = errors.stream().mapToDouble(e -> e.linearVelocityError.getZ()).toArray();
      double[] vnorm = errors.stream().mapToDouble(e -> e.linearVelocityError.norm()).toArray();
      double[] wnorm = errors.stream().mapToDouble(e -> e.angularVelocityError.norm()).toArray();

      StringBuilder text = new StringBuilder();
      text.append("linear velocity error, BODY frame (m/s) -- the accuracy claim; independent of world-frame yaw\n");
      text.append(String.format("  %-8s %10s %10s %10s %10s%n", "", "rmse", "mean", "max", "std"));
      appendStatsRow(text, "vx_b", errors.stream().mapToDouble(e -> e.bodyLinearVelocityError.getX()).toArray(), 1.0);
      appendStatsRow(text, "vy_b", errors.stream().mapToDouble(e -> e.bodyLinearVelocityError.getY()).toArray(), 1.0);
      appendStatsRow(text, "vz_b", errors.stream().mapToDouble(e -> e.bodyLinearVelocityError.getZ()).toArray(), 1.0);
      appendStatsRow(text, "norm_b", errors.stream().mapToDouble(e -> e.bodyLinearVelocityError.norm()).toArray(), 1.0);
      text.append('\n');
      text.append("angular velocity error, BODY frame (rad/s)\n");
      appendStatsRow(text, "w_b", errors.stream().mapToDouble(e -> e.bodyAngularVelocityError.norm()).toArray(), 1.0);
      text.append('\n');
      text.append("linear velocity error, WORLD frame (m/s) -- assumes the estimator and mocap worlds share a yaw\n");
      text.append(String.format("  %-8s %10s %10s %10s %10s%n", "", "rmse", "mean", "max", "std"));
      appendStatsRow(text, "vx", vx, 1.0);
      appendStatsRow(text, "vy", vy, 1.0);
      appendStatsRow(text, "vz", vz, 1.0);
      appendStatsRow(text, "norm", vnorm, 1.0);
      text.append(String.format("%n  ContactNet baselines: 0.0844 / 0.0254 m/s%n"));
      text.append('\n');
      text.append("angular velocity error (rad/s)\n");
      appendStatsRow(text, "norm", wnorm, 1.0);

      return text.toString();
   }

   private static void appendStatsRow(StringBuilder text, String label, double[] values, double scale)
   {
      Stats stats = Stats.of(values);
      text.append(String.format("  %-8s %10.4f %10.4f %10.4f %10.4f%n",
                                label, scale * stats.rmse(), scale * stats.mean(), scale * stats.max(), scale * stats.standardDeviation()));
   }

   private static void writeComparisonCsv(Path file, List<PoseErrorSample> errors) throws IOException
   {
      try (BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         writer.write("# per-matched-sample pose error, ground truth minus... no: estimator minus ground truth. Positive means the estimator is ahead.\n");
         writer.write("timestamp_ns,time_delta_ns,error_x,error_y,error_z,error_norm,orientation_error_deg,tilt_error_deg\n");

         for (PoseErrorSample error : errors)
         {
            writer.write(error.timestampNanoseconds + "," + error.timeDeltaNanoseconds + "," + error.positionError.getX() + "," + error.positionError.getY()
                        + "," + error.positionError.getZ() + "," + error.positionError.norm() + "," + Math.toDegrees(error.orientationErrorRadians)
                        + "," + Math.toDegrees(error.tiltErrorRadians) + "\n");
         }
      }
   }

   private static void writeTwistErrorCsv(Path file, List<TwistErrorSample> errors) throws IOException
   {
      try (BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         writer.write("# per-matched-sample velocity error, estimator minus ground truth.\n");
         writer.write("timestamp_ns,time_delta_ns,verror_x,verror_y,verror_z,verror_norm,werror_x,werror_y,werror_z,werror_norm,"
                      + "body_verror_x,body_verror_y,body_verror_z,body_verror_norm,body_werror_norm\n");

         for (TwistErrorSample error : errors)
         {
            writer.write(error.timestampNanoseconds + "," + error.timeDeltaNanoseconds + "," + error.linearVelocityError.getX() + ","
                        + error.linearVelocityError.getY() + "," + error.linearVelocityError.getZ() + "," + error.linearVelocityError.norm() + ","
                        + error.angularVelocityError.getX() + "," + error.angularVelocityError.getY() + "," + error.angularVelocityError.getZ() + ","
                        + error.angularVelocityError.norm() + "," + error.bodyLinearVelocityError.getX() + "," + error.bodyLinearVelocityError.getY() + ","
                        + error.bodyLinearVelocityError.getZ() + "," + error.bodyLinearVelocityError.norm() + "," + error.bodyAngularVelocityError.norm()
                        + "\n");
         }
      }
   }

   // ---------------------------------------------------------------------------------------------
   // CSV readers
   // ---------------------------------------------------------------------------------------------

   private static List<PoseSample> readPoseCsv(Path file) throws IOException
   {
      List<PoseSample> samples = new ArrayList<>();

      for (String line : dataLines(file))
      {
         String[] columns = line.split(",");

         if (columns.length < 8)
            throw new IOException(file + ": expected at least 8 columns (timestamp_ns,x,y,z,qx,qy,qz,qs), got " + columns.length + ": " + line);

         Point3D position = new Point3D(Double.parseDouble(columns[1]), Double.parseDouble(columns[2]), Double.parseDouble(columns[3]));
         Quaternion orientation = new Quaternion(Double.parseDouble(columns[4]),
                                                 Double.parseDouble(columns[5]),
                                                 Double.parseDouble(columns[6]),
                                                 Double.parseDouble(columns[7]));

         samples.add(new PoseSample(Long.parseLong(columns[0]), position, orientation));
      }

      return samples;
   }

   /** {@code pelvisVelocity.csv}'s edge samples are NaN by construction (the centred window has no value there); skipped, not zeroed. */
   private static List<TwistSample> readTwistCsv(Path file) throws IOException
   {
      List<TwistSample> samples = new ArrayList<>();

      for (String line : dataLines(file))
      {
         String[] columns = line.split(",");

         if (columns.length < 7)
            throw new IOException(file + ": expected at least 7 columns (timestamp_ns,vx,vy,vz,wx,wy,wz), got " + columns.length + ": " + line);

         double[] values = new double[6];
         boolean anyNaN = false;

         for (int i = 0; i < 6; i++)
         {
            values[i] = Double.parseDouble(columns[i + 1]);
            anyNaN |= Double.isNaN(values[i]);
         }

         if (anyNaN)
            continue;

         samples.add(new TwistSample(Long.parseLong(columns[0]), new Vector3D(values[0], values[1], values[2]), new Vector3D(values[3], values[4],
                                                                                                                              values[5])));
      }

      return samples;
   }

   private static List<EstimatorSample> readEstimatorCsv(Path file) throws IOException
   {
      List<EstimatorSample> samples = new ArrayList<>();

      for (String line : dataLines(file))
      {
         String[] columns = line.split(",");

         if (columns.length < 14)
         {
            throw new IOException(file + ": expected at least 14 columns (timestamp_ns,x,y,z,qx,qy,qz,qs,vx,vy,vz,wx,wy,wz), got " + columns.length + ": "
                  + line);
         }

         // Columns past the 14th are ignored here on purpose, so an exporter may append its own
         // without breaking this tool: NEES needs the filter's covariance, which this pose/twist
         // schema has no room for and which a downstream consistency check reads from the same file.

         long timestampNanoseconds = Long.parseLong(columns[0]);
         Point3D position = new Point3D(Double.parseDouble(columns[1]), Double.parseDouble(columns[2]), Double.parseDouble(columns[3]));
         Quaternion orientation = new Quaternion(Double.parseDouble(columns[4]),
                                                 Double.parseDouble(columns[5]),
                                                 Double.parseDouble(columns[6]),
                                                 Double.parseDouble(columns[7]));
         Vector3D linearVelocity = new Vector3D(Double.parseDouble(columns[8]), Double.parseDouble(columns[9]), Double.parseDouble(columns[10]));
         Vector3D angularVelocity = new Vector3D(Double.parseDouble(columns[11]), Double.parseDouble(columns[12]), Double.parseDouble(columns[13]));

         samples.add(new EstimatorSample(timestampNanoseconds, position, orientation, linearVelocity, angularVelocity));
      }

      return samples;
   }

   /** Blank lines, comments and the header row stripped -- the one parsing convention every CSV in this project shares. */
   private static List<String> dataLines(Path file) throws IOException
   {
      List<String> lines = new ArrayList<>();

      for (String line : Files.readAllLines(file, StandardCharsets.UTF_8))
      {
         if (line.isBlank() || line.startsWith("#") || line.startsWith("timestamp"))
            continue;

         lines.add(line);
      }

      return lines;
   }

   // ---------------------------------------------------------------------------------------------
   // Data types
   // ---------------------------------------------------------------------------------------------

   private record PoseSample(long timestampNanoseconds, Point3D position, Quaternion orientation)
   {
   }

   private record TwistSample(long timestampNanoseconds, Vector3D linearVelocity, Vector3D angularVelocity)
   {
   }

   /** Mutable, unlike the others: the IMU correction rewrites {@code linearVelocity} in place before matching. */
   private static final class EstimatorSample
   {
      final long timestampNanoseconds;
      final Point3D position;
      final Quaternion orientation;
      final Vector3D linearVelocity;
      final Vector3D angularVelocity;

      EstimatorSample(long timestampNanoseconds, Point3D position, Quaternion orientation, Vector3D linearVelocity, Vector3D angularVelocity)
      {
         this.timestampNanoseconds = timestampNanoseconds;
         this.position = position;
         this.orientation = orientation;
         this.linearVelocity = linearVelocity;
         this.angularVelocity = angularVelocity;
      }
   }

   private record OrientedTwistSample(long timestampNanoseconds, Vector3D linearVelocity, Vector3D angularVelocity, Quaternion orientation)
   {
   }

   private record PoseErrorSample(long timestampNanoseconds,
                                  long timeDeltaNanoseconds,
                                  Vector3D positionError,
                                  double orientationErrorRadians,
                                  double tiltErrorRadians)
   {
   }

   private record TwistErrorSample(long timestampNanoseconds,
                                   long timeDeltaNanoseconds,
                                   Vector3D linearVelocityError,
                                   Vector3D angularVelocityError,
                                   Vector3D bodyLinearVelocityError,
                                   Vector3D bodyAngularVelocityError)
   {
   }

   enum EstimatorFrame
   {
      PELVIS_LINK, IMU
   }

   private static void printUsage(PrintStream stream)
   {
      stream.println("""
            Usage: EstimatorComparisonRunner --ground-truth-pose <pelvis.csv> --ground-truth-velocity <pelvisVelocity.csv>
                                             --estimator-log <csv> [options]

            Compares a mocap ground-truth pelvis trajectory (from `ReplayRunner --velocity`) against
            an estimator's own logged pelvis state. Time-aligns by nearest-neighbour, never by
            interpolation, and reports position/orientation/velocity error statistics.

            The estimator log is a CSV this tool defines, not one any particular estimator already
            writes: timestamp_ns,x,y,z,qx,qy,qz,qs,vx,vy,vz,wx,wy,wz, sharing an epoch with the mocap
            capture the ground truth came from. See this class's javadoc for the full schema and the
            FRAMEWORK.md section 13 three-pelvis-frames hazard this tool corrects for.

            Required:
              --ground-truth-pose <file>       pelvis.csv from ReplayRunner.
              --ground-truth-velocity <file>   pelvisVelocity.csv from `ReplayRunner --velocity`.
              --estimator-log <file>           the estimator's own pelvis state log, in the schema above.

            Optional:
              --output-directory <dir>   write poseError.csv and velocityError.csv here.
              --estimator-frame <name>   PELVIS_LINK (default) or IMU. IMU applies the omega x r
                                         correction FRAMEWORK.md section 13 warns is otherwise
                                         "enough to swamp the entire comparison".
              --imu-offset <x,y,z>       IMU-to-pelvis-link offset in the body frame, metres.
                                         Required when --estimator-frame is IMU.
              --max-time-delta <ms>      matches beyond this are dropped rather than compared.
                                         Default 5.
              --min-match-fraction <f>   below this fraction of matched pose samples, exit
                                         non-zero rather than report statistics from a handful
                                         of coincidental matches. Default 0.5.
              --help

            Exit codes:
              0  every check ran and the match rate cleared --min-match-fraction
              1  match rate too low -- almost always an epoch mismatch between the two logs
              2  usage or I/O error""");
   }

   static final class Arguments
   {
      Path groundTruthPelvis;
      Path groundTruthVelocity;
      Path estimatorLog;
      Path outputDirectory;
      EstimatorFrame estimatorFrame = EstimatorFrame.PELVIS_LINK;
      Vector3D imuOffset;
      long maxTimeDeltaNanoseconds = 5_000_000L;
      double minimumMatchFraction = 0.5;
      boolean help = false;

      static Arguments parse(String[] args)
      {
         Arguments arguments = new Arguments();

         if (args.length == 0)
         {
            arguments.help = true;
            return arguments;
         }

         for (int i = 0; i < args.length; i++)
         {
            switch (args[i])
            {
               case "--help", "-h" -> arguments.help = true;
               case "--ground-truth-pose" -> arguments.groundTruthPelvis = Path.of(value(args, ++i, "--ground-truth-pose"));
               case "--ground-truth-velocity" -> arguments.groundTruthVelocity = Path.of(value(args, ++i, "--ground-truth-velocity"));
               case "--estimator-log" -> arguments.estimatorLog = Path.of(value(args, ++i, "--estimator-log"));
               case "--output-directory" -> arguments.outputDirectory = Path.of(value(args, ++i, "--output-directory"));
               case "--estimator-frame" -> arguments.estimatorFrame = parseFrame(value(args, ++i, "--estimator-frame"));
               case "--imu-offset" -> arguments.imuOffset = parseOffset(value(args, ++i, "--imu-offset"));
               case "--max-time-delta" -> arguments.maxTimeDeltaNanoseconds = (long) (1.0e6 * Double.parseDouble(value(args, ++i, "--max-time-delta")));
               case "--min-match-fraction" -> arguments.minimumMatchFraction = Double.parseDouble(value(args, ++i, "--min-match-fraction"));
               default -> throw new IllegalArgumentException("unknown option '" + args[i] + "'");
            }
         }

         if (arguments.help)
            return arguments;

         require(arguments.groundTruthPelvis, "--ground-truth-pose");
         require(arguments.groundTruthVelocity, "--ground-truth-velocity");
         require(arguments.estimatorLog, "--estimator-log");

         if (arguments.estimatorFrame == EstimatorFrame.IMU && arguments.imuOffset == null)
            throw new IllegalArgumentException("--estimator-frame IMU needs --imu-offset");

         return arguments;
      }

      private static EstimatorFrame parseFrame(String text)
      {
         try
         {
            return EstimatorFrame.valueOf(text.trim().toUpperCase());
         }
         catch (IllegalArgumentException e)
         {
            throw new IllegalArgumentException("--estimator-frame expects PELVIS_LINK or IMU, got '" + text + "'");
         }
      }

      private static Vector3D parseOffset(String text)
      {
         String[] parts = text.split(",");

         if (parts.length != 3)
            throw new IllegalArgumentException("--imu-offset expects x,y,z, got '" + text + "'");

         return new Vector3D(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]), Double.parseDouble(parts[2]));
      }

      private static void require(Path path, String option)
      {
         if (path == null)
            throw new IllegalArgumentException(option + " is required");
      }

      private static String value(String[] args, int index, String option)
      {
         if (index >= args.length)
            throw new IllegalArgumentException(option + " needs a value");

         return args[index];
      }
   }
}
