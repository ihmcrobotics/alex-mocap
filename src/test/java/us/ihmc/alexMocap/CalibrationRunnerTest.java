package us.ihmc.alexMocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.mocap.MocapFrameRecorder;

/**
 * The PR1 definition of done, end to end: {@code CalibrationRunner --gate g1 --input <csv>} prints
 * a per-cluster table and exits non-zero on failure.
 * <p>
 * These run the real CLI over a real file on disk. Nothing is stubbed -- the log is written by
 * {@link MocapFrameRecorder}, read by the replay source, and judged by G1, which is exactly the
 * path a capture takes at the gantry.
 * </p>
 */
public class CalibrationRunnerTest
{
   private static final double SIGMA = 0.3e-3;

   private static final List<MarkerId> MARKER_SET = MarkerId.createDenseSet("PELVIS_1",
                                                                           "PELVIS_2",
                                                                           "PELVIS_3",
                                                                           "PELVIS_4",
                                                                           "L_THIGH_1",
                                                                           "L_THIGH_2",
                                                                           "L_THIGH_3",
                                                                           "L_THIGH_4");

   private static final double[][] GEOMETRY = {{0.060, 0.000, 0.010}, {-0.045, 0.055, -0.008}, {-0.050, -0.048, 0.012}, {0.030, -0.062, -0.015},
                                               {0.050, 0.010, 0.200}, {-0.040, 0.045, 0.190}, {-0.048, -0.040, 0.205}, {0.025, -0.055, 0.185}};

   @Test
   public void testRigidCapturePassesAndExitsZero(@TempDir Path directory) throws IOException
   {
      Path capture = writeCapture(directory.resolve("rigid.csv"), 0.0);
      Output output = run("--gate", "g1", "--input", capture.toString(), "--sigma", Double.toString(SIGMA));

      assertEquals(0, output.exitCode, () -> "Expected a pass:\n" + output);
      assertTrue(output.out.contains("G1: PASS"), output.out);
      assertTrue(output.out.contains("all 12 checks passed"), "Two 4-marker clusters give 6 pairs each:\n" + output.out);
      assertTrue(output.out.contains("PELVIS(4)") && output.out.contains("L_THIGH(4)"), "Clusters should be inferred from marker names:\n" + output.out);
   }

   @Test
   public void testDriftingMarkerFailsAndExitsNonZero(@TempDir Path directory) throws IOException
   {
      Path capture = writeCapture(directory.resolve("slop.csv"), 2.0e-3);
      Output output = run("--gate", "g1", "--input", capture.toString(), "--sigma", Double.toString(SIGMA));

      assertEquals(1, output.exitCode, () -> "A 2 mm slip must exit non-zero:\n" + output);
      assertTrue(output.out.contains("G1: FAIL"), output.out);
      assertTrue(output.out.contains("PELVIS_1"), "The table should name the pairs that failed:\n" + output.out);
      assertTrue(output.out.contains("FAIL"), output.out);
   }

   /**
    * An incomplete gate exits non-zero. A green exit code on checks that could not run is how a
    * gate quietly stops protecting anything.
    */
   @Test
   public void testIncompleteGateExitsNonZero(@TempDir Path directory) throws IOException
   {
      Path capture = writeCapture(directory.resolve("short.csv"), 0.0, 40);
      Output output = run("--gate", "g1", "--input", capture.toString(), "--sigma", Double.toString(SIGMA));

      assertEquals(1, output.exitCode, () -> "40 frames is under the 100-sample minimum:\n" + output);
      assertTrue(output.out.contains("INCOMPLETE"), output.out);
      assertTrue(output.out.contains("not a pass"), "The report should say plainly that incomplete is not a pass:\n" + output.out);

      // Lowering the minimum makes the same capture judgeable.
      Output relaxed = run("--gate", "g1", "--input", capture.toString(), "--sigma", Double.toString(SIGMA), "--min-samples", "20");
      assertEquals(0, relaxed.exitCode, () -> relaxed.toString());
   }

   /** Sigma has no default. Inventing one would be assuming the number FRAMEWORK.md §17 measures. */
   @Test
   public void testSigmaIsRequired(@TempDir Path directory) throws IOException
   {
      Path capture = writeCapture(directory.resolve("rigid.csv"), 0.0);
      Output output = run("--gate", "g1", "--input", capture.toString());

      assertEquals(2, output.exitCode);
      assertTrue(output.err.contains("--sigma is required"), output.err);
      assertTrue(output.err.contains("measured"), "The error should say why there is no default:\n" + output.err);
   }

   @Test
   public void testExplicitClustersOverrideInference(@TempDir Path directory) throws IOException
   {
      Path capture = writeCapture(directory.resolve("rigid.csv"), 0.0);
      Output output = run("--gate",
                          "g1",
                          "--input",
                          capture.toString(),
                          "--sigma",
                          Double.toString(SIGMA),
                          "--cluster",
                          "pelvis=PELVIS_1,PELVIS_2,PELVIS_3");

      assertEquals(0, output.exitCode, output::toString);
      assertTrue(output.out.contains("all 3 checks passed"), "Three markers give three pairs:\n" + output.out);
      assertTrue(output.out.contains("no redundancy for G1"), "A 3-marker cluster should be flagged as thin:\n" + output.out);
   }

   @Test
   public void testUsageErrors(@TempDir Path directory) throws IOException
   {
      Path capture = writeCapture(directory.resolve("rigid.csv"), 0.0);
      String sigma = Double.toString(SIGMA);

      assertEquals(2, run("--gate", "g9", "--input", capture.toString(), "--sigma", sigma).exitCode);
      assertEquals(2, run("--gate", "g1", "--input", directory.resolve("absent.csv").toString(), "--sigma", sigma).exitCode);
      assertEquals(2, run("--gate", "g1", "--input", capture.toString(), "--sigma", "-1").exitCode);
      assertEquals(2, run("--gate", "g1", "--input", capture.toString(), "--sigma", "banana").exitCode);
      assertEquals(2, run("--nonsense").exitCode);
      assertEquals(2, run("--gate", "g1", "--input", capture.toString(), "--sigma", sigma, "--cluster", "malformed").exitCode);
      assertEquals(2, run("--gate", "g1", "--input", capture.toString(), "--sigma", sigma, "--cluster", "x=NOT_A_MARKER").exitCode);

      Output help = run("--help");
      assertEquals(0, help.exitCode);
      assertTrue(help.out.contains("--sigma"));

      assertEquals(0, run().exitCode, "No arguments prints usage rather than failing.");
   }

   @Test
   public void testEmptyLogIsAnError(@TempDir Path directory) throws IOException
   {
      Path empty = directory.resolve("empty.csv");

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(empty, MARKER_SET))
      {
         recorder.flush();
      }

      Output output = run("--gate", "g1", "--input", empty.toString(), "--sigma", Double.toString(SIGMA));

      assertEquals(2, output.exitCode);
      assertTrue(output.err.contains("no frames"), output.err);
   }

   /** Cluster inference is a convention, so it fails loudly rather than guessing. */
   @Test
   public void testClusterInference()
   {
      List<MarkerCluster> clusters = CalibrationRunner.inferClusters(MARKER_SET);

      assertEquals(2, clusters.size());
      assertEquals("PELVIS", clusters.get(0).getLinkName());
      assertEquals("L_THIGH", clusters.get(1).getLinkName());
      assertEquals(4, clusters.get(0).getMarkerCount());

      assertThrows(IllegalArgumentException.class,
                   () -> CalibrationRunner.inferClusters(MarkerId.createDenseSet("PELVIS", "TORSO")),
                   "Names with no underscore cannot be grouped.");
      assertThrows(IllegalArgumentException.class,
                   () -> CalibrationRunner.inferClusters(MarkerId.createDenseSet("A_1", "A_2", "B_1", "B_2")),
                   "A group of two is not a cluster; say so rather than proceeding.");
   }

   /** {@code totalDrift} is applied as a step at the midpoint: a mount that slips, not one that creeps. */
   private static Path writeCapture(Path file, double totalDrift) throws IOException
   {
      return writeCapture(file, totalDrift, 600);
   }

   private static Path writeCapture(Path file, double totalDrift, int frameCount) throws IOException
   {
      Random random = new Random(20260805L);

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(file, MARKER_SET))
      {
         MocapFrame frame = new MocapFrame(MARKER_SET);

         for (int f = 0; f < frameCount; f++)
         {
            frame.clear();
            frame.setTimestampNanoseconds(1_000_000_000L + f * 5_000_000L);
            double drift = f >= frameCount / 2 ? totalDrift : 0.0;

            for (int m = 0; m < MARKER_SET.size(); m++)
            {
               frame.get(m).setVisible(GEOMETRY[m][0] + (m == 0 ? drift : 0.0) + SIGMA * random.nextGaussian(),
                                       GEOMETRY[m][1] + SIGMA * random.nextGaussian(),
                                       GEOMETRY[m][2] + SIGMA * random.nextGaussian());
            }

            recorder.write(frame);
         }
      }

      return file;
   }

   private static Output run(String... args)
   {
      ByteArrayOutputStream out = new ByteArrayOutputStream();
      ByteArrayOutputStream err = new ByteArrayOutputStream();
      int exitCode;

      try (PrintStream outStream = new PrintStream(out, true, StandardCharsets.UTF_8);
           PrintStream errStream = new PrintStream(err, true, StandardCharsets.UTF_8))
      {
         exitCode = CalibrationRunner.run(args, outStream, errStream);
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

   /** Guards against the whole suite silently passing because every exit code is zero. */
   @Test
   public void testExitCodesActuallyDiffer(@TempDir Path directory) throws IOException
   {
      Path rigid = writeCapture(directory.resolve("rigid.csv"), 0.0);
      Path slop = writeCapture(directory.resolve("slop.csv"), 2.0e-3);
      String sigma = Double.toString(SIGMA);

      int pass = run("--gate", "g1", "--input", rigid.toString(), "--sigma", sigma).exitCode;
      int fail = run("--gate", "g1", "--input", slop.toString(), "--sigma", sigma).exitCode;

      assertNotEquals(pass, fail);
      assertEquals(0, pass);
      assertEquals(1, fail);
   }
}
