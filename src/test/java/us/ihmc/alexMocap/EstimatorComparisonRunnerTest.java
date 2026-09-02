package us.ihmc.alexMocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * No real estimator log exists yet -- the InEKF work lives on a different branch combination (see
 * the mocap-vs-estimator branch-combination notes) that is not checked out alongside this one. So
 * every log here is hand-written to the schema {@link EstimatorComparisonRunner} itself defines,
 * with known errors planted by hand, the same way {@code GateInjectionTest} validates the mocap
 * calibration gates against known-bad synthetic data rather than real captures.
 */
public class EstimatorComparisonRunnerTest
{
   private static final long[] TIMESTAMPS_NS = {0L, 10_000_000L, 20_000_000L, 30_000_000L, 40_000_000L};

   @Test
   public void testPerfectMatchGivesZeroError(@TempDir Path directory) throws IOException
   {
      writePoseCsv(directory.resolve("pelvis.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0);
      writeVelocityCsv(directory.resolve("pelvisVelocity.csv"), TIMESTAMPS_NS, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
      writeEstimatorCsv(directory.resolve("estimator.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0);

      Output output = run("--ground-truth-pose", directory.resolve("pelvis.csv").toString(), "--ground-truth-velocity",
                          directory.resolve("pelvisVelocity.csv").toString(), "--estimator-log", directory.resolve("estimator.csv").toString());

      assertEquals(0, output.exitCode, output::toString);
      assertTrue(output.out.contains("matched 5 of 5 pose samples (100.0%)"), output.out);
      assertTrue(output.out.contains(expectedStatsRow("x", 0.0, 0.0, 0.0, 0.0)), "Identical logs should give zero position error.\n" + output.out);
      assertTrue(output.out.contains(expectedStatsRow("angle", 0.0, 0.0, 0.0, 0.0)), "Identical orientations should give zero angle error.\n" + output.out);
      assertTrue(output.out.contains(expectedStatsRow("vx", 0.0, 0.0, 0.0, 0.0)), "Identical velocities should give zero velocity error.\n" + output.out);
   }

   @Test
   public void testConstantPositionBiasIsRecoveredExactly(@TempDir Path directory) throws IOException
   {
      writePoseCsv(directory.resolve("pelvis.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0);
      writeVelocityCsv(directory.resolve("pelvisVelocity.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      // The estimator is off by exactly +10 mm in x, and nothing else. A constant bias has zero
      // variance, so rmse == mean == max and std == 0 -- an unambiguous check that x, y, z are not
      // being mixed up anywhere in the pipeline.
      writeEstimatorCsv(directory.resolve("estimator.csv"), TIMESTAMPS_NS, 0.010, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      Output output = run("--ground-truth-pose", directory.resolve("pelvis.csv").toString(), "--ground-truth-velocity",
                          directory.resolve("pelvisVelocity.csv").toString(), "--estimator-log", directory.resolve("estimator.csv").toString());

      assertEquals(0, output.exitCode, output::toString);
      assertTrue(output.out.contains(expectedStatsRow("x", 10.0, 10.0, 10.0, 0.0)), "A constant 10 mm x bias must show up as x, not smeared into y/z.\n" + output.out);
      assertTrue(output.out.contains(expectedStatsRow("y", 0.0, 0.0, 0.0, 0.0)), output.out);
      assertTrue(output.out.contains(expectedStatsRow("z", 0.0, 0.0, 0.0, 0.0)), output.out);
      assertTrue(output.out.contains(expectedStatsRow("norm", 10.0, 10.0, 10.0, 0.0)), output.out);
   }

   @Test
   public void testConstantVelocityBiasIsRecoveredExactly(@TempDir Path directory) throws IOException
   {
      writePoseCsv(directory.resolve("pelvis.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0);
      writeVelocityCsv(directory.resolve("pelvisVelocity.csv"), TIMESTAMPS_NS, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
      // Estimator over-reports vx by exactly 0.05 m/s.
      writeEstimatorCsv(directory.resolve("estimator.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0);

      Output output = run("--ground-truth-pose", directory.resolve("pelvis.csv").toString(), "--ground-truth-velocity",
                          directory.resolve("pelvisVelocity.csv").toString(), "--estimator-log", directory.resolve("estimator.csv").toString());

      assertEquals(0, output.exitCode, output::toString);
      assertTrue(output.out.contains(expectedStatsRow("vx", 0.05, 0.05, 0.05, 0.0)), "A constant 0.05 m/s vx bias must be recovered exactly.\n" + output.out);
      assertTrue(output.out.contains("ContactNet baselines"), "The report should carry the comparison baselines from FRAMEWORK.md.\n" + output.out);
   }

   /**
    * FRAMEWORK.md section 13's own numeric example: at ω = 1 rad/s and a 0.1 m lever arm, an
    * uncorrected IMU-vs-pelvis frame mismatch is "enough to swamp the entire comparison". This
    * plants exactly that mismatch and checks both halves of the claim: uncorrected, it is a 0.1 m/s
    * spurious error; with --estimator-frame IMU and the right offset, it is exactly zero.
    */
   @Test
   public void testImuFrameCorrectionRemovesTheOmegaCrossRHazard(@TempDir Path directory) throws IOException
   {
      long[] oneTimestamp = {0L};

      // Ground truth: pelvis is stationary in translation, rotating in place at 1 rad/s about z.
      writePoseCsv(directory.resolve("pelvis.csv"), oneTimestamp, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0);
      writeVelocityCsv(directory.resolve("pelvisVelocity.csv"), oneTimestamp, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

      // The estimator publishes the IMU mount's own velocity: the IMU sits 0.1 m in +x of the
      // pelvis link frame (body == world here, identity orientation), so v_imu = v_pelvis - ω x r
      // = (0,0,0) - (0,0,1) x (0.1,0,0) = (0, -0.1, 0).
      writeEstimatorCsv(directory.resolve("estimator.csv"), oneTimestamp, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0, 0.0, -0.1, 0.0, 0.0, 0.0, 1.0);

      Output uncorrected = run("--ground-truth-pose", directory.resolve("pelvis.csv").toString(), "--ground-truth-velocity",
                              directory.resolve("pelvisVelocity.csv").toString(), "--estimator-log", directory.resolve("estimator.csv").toString());

      assertEquals(0, uncorrected.exitCode, uncorrected::toString);
      assertTrue(uncorrected.out.contains(expectedStatsRow("norm", 0.1, 0.1, 0.1, 0.0)),
                 "Uncorrected, the omega x r mismatch must show up as exactly the 0.1 m/s FRAMEWORK.md warns about.\n" + uncorrected.out);

      Output corrected = run("--ground-truth-pose", directory.resolve("pelvis.csv").toString(), "--ground-truth-velocity",
                             directory.resolve("pelvisVelocity.csv").toString(), "--estimator-log", directory.resolve("estimator.csv").toString(),
                             "--estimator-frame", "IMU", "--imu-offset", "0.1,0,0");

      assertEquals(0, corrected.exitCode, corrected::toString);
      assertTrue(corrected.out.contains(expectedStatsRow("norm", 0.0, 0.0, 0.0, 0.0)),
                 "Corrected with the right lever arm, the same data must show zero velocity error.\n" + corrected.out);
   }

   @Test
   public void testEstimatorFrameImuRequiresImuOffset(@TempDir Path directory) throws IOException
   {
      writePoseCsv(directory.resolve("pelvis.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0);
      writeVelocityCsv(directory.resolve("pelvisVelocity.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      writeEstimatorCsv(directory.resolve("estimator.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      Output output = run("--ground-truth-pose", directory.resolve("pelvis.csv").toString(), "--ground-truth-velocity",
                          directory.resolve("pelvisVelocity.csv").toString(), "--estimator-log", directory.resolve("estimator.csv").toString(),
                          "--estimator-frame", "IMU");

      assertEquals(2, output.exitCode);
      assertTrue(output.err.contains("--imu-offset"), output.err);
   }

   /** A near-zero max-time-delta means nothing matches, which must fail loudly rather than report on a handful of coincidences. */
   @Test
   public void testTooLowAMatchRateExitsNonZero(@TempDir Path directory) throws IOException
   {
      writePoseCsv(directory.resolve("pelvis.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0);
      writeVelocityCsv(directory.resolve("pelvisVelocity.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      // The estimator log is offset by 100 ms from every ground-truth timestamp -- with a 5 ms
      // default tolerance, and an even tighter one here, nothing should match.
      long[] shifted = new long[TIMESTAMPS_NS.length];

      for (int i = 0; i < TIMESTAMPS_NS.length; i++)
         shifted[i] = TIMESTAMPS_NS[i] + 100_000_000L;

      writeEstimatorCsv(directory.resolve("estimator.csv"), shifted, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      Output output = run("--ground-truth-pose", directory.resolve("pelvis.csv").toString(), "--ground-truth-velocity",
                          directory.resolve("pelvisVelocity.csv").toString(), "--estimator-log", directory.resolve("estimator.csv").toString(),
                          "--max-time-delta", "1");

      assertEquals(1, output.exitCode, output::toString);
      assertTrue(output.err.contains("epoch"), "The likeliest real-world cause should be named.\n" + output.err);
   }

   /** pelvisVelocity.csv's edge samples are NaN by construction (ReplayRunner/PelvisTwistEstimator); they must be skipped, not compared as zero. */
   @Test
   public void testNaNVelocityEdgesAreSkippedRatherThanTreatedAsZero(@TempDir Path directory) throws IOException
   {
      writePoseCsv(directory.resolve("pelvis.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0);

      try (java.io.BufferedWriter writer = Files.newBufferedWriter(directory.resolve("pelvisVelocity.csv"), StandardCharsets.UTF_8))
      {
         writer.write("# test\n");
         writer.write("timestamp_ns,vx,vy,vz,wx,wy,wz\n");
         writer.write(TIMESTAMPS_NS[0] + ",NaN,NaN,NaN,NaN,NaN,NaN\n");

         for (int i = 1; i < TIMESTAMPS_NS.length - 1; i++)
            writer.write(TIMESTAMPS_NS[i] + ",0.1,0,0,0,0,0\n");

         writer.write(TIMESTAMPS_NS[TIMESTAMPS_NS.length - 1] + ",NaN,NaN,NaN,NaN,NaN,NaN\n");
      }

      writeEstimatorCsv(directory.resolve("estimator.csv"), TIMESTAMPS_NS, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0);

      Output output = run("--ground-truth-pose", directory.resolve("pelvis.csv").toString(), "--ground-truth-velocity",
                          directory.resolve("pelvisVelocity.csv").toString(), "--estimator-log", directory.resolve("estimator.csv").toString());

      assertEquals(0, output.exitCode, output::toString);
      assertTrue(output.out.contains("(3 finite samples)"), "2 of 5 velocity rows are NaN and must not be read as data.\n" + output.out);
      assertTrue(output.out.contains("matched 3 of 3 velocity samples (100.0%)"), output.out);
   }

   // ---------------------------------------------------------------------------------------------
   // Fixtures
   // ---------------------------------------------------------------------------------------------

   private static void writePoseCsv(Path file, long[] timestamps, double x, double y, double z, double qx, double qy, double qz, double qs)
         throws IOException
   {
      try (java.io.BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         writer.write("# test\n");
         writer.write("timestamp_ns,x,y,z,qx,qy,qz,qs\n");

         for (long timestamp : timestamps)
            writer.write(timestamp + "," + x + "," + y + "," + z + "," + qx + "," + qy + "," + qz + "," + qs + "\n");
      }
   }

   private static void writeVelocityCsv(Path file, long[] timestamps, double vx, double vy, double vz, double wx, double wy, double wz) throws IOException
   {
      try (java.io.BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         writer.write("# test\n");
         writer.write("timestamp_ns,vx,vy,vz,wx,wy,wz\n");

         for (long timestamp : timestamps)
            writer.write(timestamp + "," + vx + "," + vy + "," + vz + "," + wx + "," + wy + "," + wz + "\n");
      }
   }

   private static void writeEstimatorCsv(Path file,
                                         long[] timestamps,
                                         double x,
                                         double y,
                                         double z,
                                         double qx,
                                         double qy,
                                         double qz,
                                         double qs,
                                         double vx,
                                         double vy,
                                         double vz,
                                         double wx,
                                         double wy,
                                         double wz)
         throws IOException
   {
      try (java.io.BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         writer.write("# test estimator log\n");
         writer.write("timestamp_ns,x,y,z,qx,qy,qz,qs,vx,vy,vz,wx,wy,wz\n");

         for (long timestamp : timestamps)
         {
            writer.write(timestamp + "," + x + "," + y + "," + z + "," + qx + "," + qy + "," + qz + "," + qs + "," + vx + "," + vy + "," + vz + "," + wx
                        + "," + wy + "," + wz + "\n");
         }
      }
   }

   /** Matches {@code EstimatorComparisonRunner}'s own row format exactly, so a formatting change breaks this test rather than silently drifting apart. */
   private static String expectedStatsRow(String label, double rmse, double mean, double max, double std)
   {
      return String.format("  %-8s %10.4f %10.4f %10.4f %10.4f", label, rmse, mean, max, std);
   }

   private static Output run(String... args)
   {
      ByteArrayOutputStream out = new ByteArrayOutputStream();
      ByteArrayOutputStream err = new ByteArrayOutputStream();
      int exitCode;

      try (PrintStream outStream = new PrintStream(out, true, StandardCharsets.UTF_8);
           PrintStream errStream = new PrintStream(err, true, StandardCharsets.UTF_8))
      {
         exitCode = EstimatorComparisonRunner.run(args, outStream, errStream);
      }

      return new Output(exitCode, out.toString(StandardCharsets.UTF_8), err.toString(StandardCharsets.UTF_8));
   }

   private record Output(int exitCode, String out, String err)
   {
      @Override
      public String toString()
      {
         return "exit " + exitCode + "\n--- stdout ---\n" + out + "--- stderr ---\n" + err;
      }
   }
}
