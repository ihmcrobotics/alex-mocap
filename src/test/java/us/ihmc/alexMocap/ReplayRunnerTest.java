package us.ihmc.alexMocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import us.ihmc.alexMocap.calibration.SyntheticCaptures;
import us.ihmc.alexMocap.core.CsvEncoderLog;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.mocap.MocapFrameRecorder;

/**
 * The whole pipeline, end to end through both CLIs: synthetic captures → two CSVs →
 * {@code CalibrationRunner --calibrate} → {@code ReplayRunner} → a CoM trajectory.
 * <p>
 * This is the test that says PR1, PR2 and PR3 fit together. Every other test checks one stage
 * against planted truth or against an oracle; this one checks that the artifacts each stage writes
 * are the artifacts the next stage can read.
 * </p>
 */
public class ReplayRunnerTest
{
   /** Writes a capture set and calibrates it, leaving all four inputs ReplayRunner needs. */
   private static SyntheticCaptures.Planted prepare(Path directory, SyntheticCaptures.Options options) throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(options);

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(directory.resolve("capture.csv"), planted.markers))
      {
         for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
            recorder.write(planted.captureSet.getCapture(k).getMocapFrame());
      }

      List<EncoderSample> encoders = new ArrayList<>();

      for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
         encoders.add(planted.captureSet.getCapture(k).getEncoderSample());

      CsvEncoderLog.write(directory.resolve("encoders.csv"), encoders);
      Files.copy(SyntheticCaptures.toyUrdfPath(), directory.resolve("toy.urdf"), java.nio.file.StandardCopyOption.REPLACE_EXISTING);

      int calibrationExit = CalibrationRunner.run(new String[] {"--calibrate", "--input", directory.resolve("capture.csv").toString(), "--encoders",
            directory.resolve("encoders.csv").toString(), "--urdf", directory.resolve("toy.urdf").toString(), "--sigma", "0.0003", "--world-tilt", "0.08",
            "--output", directory.resolve("calibration.json").toString()},
                                                  new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8),
                                                  new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8));

      assertEquals(0, calibrationExit, "The calibration step must succeed before the replay can be tested.");

      return planted;
   }

   private static String[] replayArguments(Path directory, String... extra)
   {
      List<String> arguments = new ArrayList<>(List.of("--input",
                                                       directory.resolve("capture.csv").toString(),
                                                       "--encoders",
                                                       directory.resolve("encoders.csv").toString(),
                                                       "--urdf",
                                                       directory.resolve("toy.urdf").toString(),
                                                       "--calibration",
                                                       directory.resolve("calibration.json").toString(),
                                                       "--output-directory",
                                                       directory.toString(),
                                                       "--world-tilt",
                                                       "0.08"));
      arguments.addAll(List.of(extra));
      return arguments.toArray(new String[0]);
   }

   @Test
   public void testReplayProducesTrajectoriesAndConditioning(@TempDir Path directory) throws Exception
   {
      prepare(directory, new SyntheticCaptures.Options().captures(60).noise(0.3e-3));

      ByteArrayOutputStream outBytes = new ByteArrayOutputStream();
      ByteArrayOutputStream errBytes = new ByteArrayOutputStream();

      int exitCode = ReplayRunner.run(replayArguments(directory, "--velocity", "--error-budget"),
                                      new PrintStream(outBytes, true, StandardCharsets.UTF_8),
                                      new PrintStream(errBytes, true, StandardCharsets.UTF_8));

      String out = outBytes.toString(StandardCharsets.UTF_8);
      assertEquals(0, exitCode, "Every frame should produce a CoM on clean data.\n" + out + "\n" + errBytes);

      // The three per-frame artifacts.
      for (String name : new String[] {"com.csv", "pelvis.csv", "conditioning.csv", "pelvisVelocity.csv"})
         assertTrue(Files.isRegularFile(directory.resolve(name)), name + " should have been written.\n" + out);

      List<String> comLines = Files.readAllLines(directory.resolve("com.csv"));
      assertEquals(62, comLines.size(), "One comment, one header, 60 rows.");
      assertTrue(comLines.get(1).startsWith("timestamp_ns,com_x"), comLines.get(1));

      for (int i = 2; i < comLines.size(); i++)
         assertFalse(comLines.get(i).contains("NaN"), "Every frame should have a CoM: " + comLines.get(i));

      // Conditioning is reported per link, and the header names every marked link.
      String conditioningHeader = Files.readAllLines(directory.resolve("conditioning.csv")).get(1);
      assertTrue(conditioningHeader.contains("pelvis_sigma3"), conditioningHeader);
      assertTrue(conditioningHeader.contains("l_foot_accepted"), conditioningHeader);

      // The console report carries the things a human needs to judge the run.
      assertTrue(out.contains("Conditioning over 60 frames"), out);
      assertTrue(out.contains("visible-count histogram"), out);
      assertTrue(out.contains("velocity second pass"), out);
      assertTrue(out.contains("ContactNet bar"), out);
      assertTrue(out.contains("CoM error budget"), out);
      assertTrue(out.contains("as good as the URDF"), "§14's conclusion should be printed, not implied.\n" + out);
   }

   /**
    * The velocity file's edges are NaN and its interior is not. A centred window has no value near
    * the ends, and filling those with a one-sided estimate would reintroduce the lag §13 forbids.
    */
   @Test
   public void testVelocityFileHasNaNEdgesAndFiniteInterior(@TempDir Path directory) throws Exception
   {
      prepare(directory, new SyntheticCaptures.Options().captures(80).noise(0.3e-3));

      ReplayRunner.run(replayArguments(directory, "--velocity"),
                       new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8),
                       new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8));

      List<String> lines = Files.readAllLines(directory.resolve("pelvisVelocity.csv"));
      List<String> rows = lines.subList(3, lines.size());

      assertEquals(80, rows.size());

      // 0.1 s at 200 Hz is a half-window of 10 samples.
      for (int i = 0; i < 10; i++)
      {
         assertTrue(rows.get(i).contains("NaN"), "Leading edge " + i + " must be NaN: " + rows.get(i));
         assertTrue(rows.get(rows.size() - 1 - i).contains("NaN"), "Trailing edge " + i + " must be NaN.");
      }

      for (int i = 10; i < rows.size() - 10; i++)
         assertFalse(rows.get(i).contains("NaN"), "Interior sample " + i + " must be finite: " + rows.get(i));
   }

   /**
    * A refused frame exits non-zero. A missing link means there is no CoM for that frame, which is
    * a failure rather than a footnote.
    */
   @Test
   public void testRefusedFrameExitsNonZero(@TempDir Path directory) throws Exception
   {
      SyntheticCaptures.Planted planted = SyntheticCaptures.generate(new SyntheticCaptures.Options().captures(30).noise(0.3e-3));

      // Blind three of the four l_shank markers in one frame, before writing the log.
      var cluster = planted.clusters.stream().filter(c -> c.getLinkName().equals("l_shank")).findFirst().orElseThrow();

      for (int j = 0; j < 3; j++)
         planted.captureSet.getCapture(7).getMocapFrame().get(cluster.getMarker(j)).setNotVisible();

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(directory.resolve("capture.csv"), planted.markers))
      {
         for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
            recorder.write(planted.captureSet.getCapture(k).getMocapFrame());
      }

      List<EncoderSample> encoders = new ArrayList<>();

      for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
         encoders.add(planted.captureSet.getCapture(k).getEncoderSample());

      CsvEncoderLog.write(directory.resolve("encoders.csv"), encoders);
      Files.copy(SyntheticCaptures.toyUrdfPath(), directory.resolve("toy.urdf"), java.nio.file.StandardCopyOption.REPLACE_EXISTING);

      assertEquals(0,
                   CalibrationRunner.run(new String[] {"--calibrate", "--input", directory.resolve("capture.csv").toString(), "--encoders",
                         directory.resolve("encoders.csv").toString(), "--urdf", directory.resolve("toy.urdf").toString(), "--sigma", "0.0003", "--output",
                         directory.resolve("calibration.json").toString()},
                                         new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8),
                                         new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8)));

      ByteArrayOutputStream outBytes = new ByteArrayOutputStream();
      ByteArrayOutputStream errBytes = new ByteArrayOutputStream();

      int exitCode = ReplayRunner.run(replayArguments(directory), new PrintStream(outBytes, true, StandardCharsets.UTF_8),
                                      new PrintStream(errBytes, true, StandardCharsets.UTF_8));

      String out = outBytes.toString(StandardCharsets.UTF_8);
      String err = errBytes.toString(StandardCharsets.UTF_8);

      assertEquals(1, exitCode, "A refused frame must exit non-zero.\n" + out);
      assertTrue(err.contains("no CoM"), err);
      assertTrue(err.contains("3.000 kg"), "The report should say how much mass went unmeasured: " + err);

      // And the CoM row for that frame is NaN, not an interpolation.
      List<String> comLines = Files.readAllLines(directory.resolve("com.csv"));
      assertTrue(comLines.get(2 + 7).contains("NaN"), "The refused frame's CoM row: " + comLines.get(2 + 7));
      assertFalse(comLines.get(2 + 6).contains("NaN"), "Its neighbours are unaffected.");
   }

   /** Without --world-tilt the run says so rather than silently assuming level. */
   @Test
   public void testUnmeasuredTiltIsVisible(@TempDir Path directory) throws Exception
   {
      prepare(directory, new SyntheticCaptures.Options().captures(30).noise(0.3e-3));

      ByteArrayOutputStream outBytes = new ByteArrayOutputStream();

      List<String> arguments = new ArrayList<>(List.of(replayArguments(directory)));
      arguments.remove(arguments.indexOf("--world-tilt") + 1);
      arguments.remove("--world-tilt");

      ReplayRunner.run(arguments.toArray(new String[0]), new PrintStream(outBytes, true, StandardCharsets.UTF_8),
                       new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8));

      assertTrue(outBytes.toString(StandardCharsets.UTF_8).contains("ASSUMED_LEVEL"),
                 "An unmeasured tilt must announce itself: " + outBytes);
   }
}
