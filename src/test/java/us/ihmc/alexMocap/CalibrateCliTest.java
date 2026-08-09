package us.ihmc.alexMocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
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
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.CalibrationResultIO;
import us.ihmc.alexMocap.core.CsvEncoderLog;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.mocap.MocapFrameRecorder;

/**
 * End-to-end exercise of {@code CalibrationRunner --calibrate}: synthetic captures out to two CSVs,
 * the CLI over them, a {@code CalibrationResult} back.
 * <p>
 * PR_PLAN.md's definition of done for PR2 asks for this command to exist and work. Compiling is not
 * the same as working, and the failure modes worth catching here -- a CSV the reader cannot parse,
 * a joint order that does not match the URDF, an unwritten output file -- are all invisible to a
 * unit test of the solver.
 * </p>
 */
public class CalibrateCliTest
{
   /** Writes a synthetic capture set out as the two CSVs the CLI reads. */
   private static SyntheticCaptures.Planted writeCaptureSet(Path directory, SyntheticCaptures.Options options) throws Exception
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

      return planted;
   }

   @Test
   public void testCalibrateProducesAResultFromTwoCsvsAndAUrdf(@TempDir Path directory) throws Exception
   {
      SyntheticCaptures.Planted planted = writeCaptureSet(directory, new SyntheticCaptures.Options().captures(30).noise(0.3e-3));

      ByteArrayOutputStream outBytes = new ByteArrayOutputStream();
      ByteArrayOutputStream errBytes = new ByteArrayOutputStream();

      int exitCode = CalibrationRunner.run(new String[] {"--calibrate", "--input", directory.resolve("capture.csv").toString(), "--encoders",
            directory.resolve("encoders.csv").toString(), "--urdf", SyntheticCaptures.toyUrdfPath().toString(), "--gauge", "pelvis", "--sigma", "0.0003",
            "--world-tilt", "0.08", "--output", directory.resolve("calibration.json").toString(), "--note", "cli test"},
                                           new PrintStream(outBytes, true, StandardCharsets.UTF_8),
                                           new PrintStream(errBytes, true, StandardCharsets.UTF_8));

      String out = outBytes.toString(StandardCharsets.UTF_8);
      String err = errBytes.toString(StandardCharsets.UTF_8);

      assertEquals(0, exitCode, "Expected a clean run.\nstdout:\n" + out + "\nstderr:\n" + err);

      // The report a human reads.
      assertTrue(out.contains("A' calibration report"), out);
      assertTrue(out.contains("monotone            yes"), "J must be monotone.\n" + out);
      assertTrue(out.contains("in-sample RMS"), out);
      assertTrue(out.contains("G2"), "G2 should be reported after the fit.\n" + out);
      assertTrue(!out.contains("HIT ITERATION CAP"), "A' should converge on clean synthetic data.\n" + out);

      // The machine-readable artifact, read back through the PR1 reader.
      Path json = directory.resolve("calibration.json");
      assertTrue(Files.isRegularFile(json), "Expected " + json + " to be written.\n" + out);

      CalibrationResult result = CalibrationResultIO.readWithDenseMarkerSet(json);
      assertTrue(result.isFullySolved(), "Every marker should be solved.");
      assertEquals(planted.clusters.size(), result.getLayouts().size());

      // Provenance: the URDF hash and the measured tilt have to survive the round trip, or a
      // calibration a month old is a set of numbers with nothing to tie it to a robot.
      assertEquals(30, result.getProvenance().captureCount());
      assertTrue(result.getProvenance().hasMeasuredWorldTilt(), "--world-tilt was given, so it must be recorded as measured.");
      assertEquals(Math.toRadians(0.08), result.getProvenance().worldTiltRadians(), 1.0e-12);
      assertEquals(64, result.getProvenance().urdfSha256().length());
      assertEquals("cli test", result.getProvenance().note());
   }

   /** FRAMEWORK.md §11: an unmeasured tilt must announce itself rather than defaulting to zero. */
   @Test
   public void testMissingWorldTiltIsWarnedAboutAndRecordedAsUnmeasured(@TempDir Path directory) throws Exception
   {
      writeCaptureSet(directory, new SyntheticCaptures.Options().captures(20).noise(0.3e-3));

      ByteArrayOutputStream outBytes = new ByteArrayOutputStream();

      int exitCode = CalibrationRunner.run(new String[] {"--calibrate", "--input", directory.resolve("capture.csv").toString(), "--encoders",
            directory.resolve("encoders.csv").toString(), "--urdf", SyntheticCaptures.toyUrdfPath().toString(), "--sigma", "0.0003", "--output",
            directory.resolve("calibration.json").toString()},
                                           new PrintStream(outBytes, true, StandardCharsets.UTF_8),
                                           new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8));

      String out = outBytes.toString(StandardCharsets.UTF_8);
      assertEquals(0, exitCode, out);
      assertTrue(out.contains("theta must be measured"), "An unmeasured world tilt must be warned about.\n" + out);

      CalibrationResult result = CalibrationResultIO.readWithDenseMarkerSet(directory.resolve("calibration.json"));
      assertTrue(!result.getProvenance().hasMeasuredWorldTilt(), "It must be recorded as unmeasured, not as zero.");
   }

   /** Without --gauge the base link of the URDF is used, which is what Δ is defined against. */
   @Test
   public void testGaugeDefaultsToTheUrdfBaseLink(@TempDir Path directory) throws Exception
   {
      writeCaptureSet(directory, new SyntheticCaptures.Options().captures(15).noise(0.3e-3));

      ByteArrayOutputStream outBytes = new ByteArrayOutputStream();

      int exitCode = CalibrationRunner.run(new String[] {"--calibrate", "--input", directory.resolve("capture.csv").toString(), "--encoders",
            directory.resolve("encoders.csv").toString(), "--urdf", SyntheticCaptures.toyUrdfPath().toString(), "--sigma", "0.0003"},
                                           new PrintStream(outBytes, true, StandardCharsets.UTF_8),
                                           new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8));

      assertEquals(0, exitCode, outBytes.toString(StandardCharsets.UTF_8));
      assertTrue(outBytes.toString(StandardCharsets.UTF_8).contains("gauge=pelvis"), outBytes.toString(StandardCharsets.UTF_8));
   }

   /** A joint order that does not match the URDF must fail loudly at the boundary. */
   @Test
   public void testMismatchedJointOrderIsRejected(@TempDir Path directory) throws Exception
   {
      writeCaptureSet(directory, new SyntheticCaptures.Options().captures(10).noise(0.3e-3));

      // Rewrite the encoder header with the joints in a different order.
      Path encoders = directory.resolve("encoders.csv");
      String text = Files.readString(encoders);
      Files.writeString(encoders, text.replace("timestamp_ns,l_hip,l_knee,l_ankle,r_hip,r_knee,r_ankle",
                                               "timestamp_ns,r_hip,r_knee,r_ankle,l_hip,l_knee,l_ankle"));

      ByteArrayOutputStream errBytes = new ByteArrayOutputStream();

      int exitCode = CalibrationRunner.run(new String[] {"--calibrate", "--input", directory.resolve("capture.csv").toString(), "--encoders",
            encoders.toString(), "--urdf", SyntheticCaptures.toyUrdfPath().toString(), "--sigma", "0.0003"},
                                           new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8),
                                           new PrintStream(errBytes, true, StandardCharsets.UTF_8));

      assertTrue(exitCode != 0, "A permuted joint order must not produce a calibration.");
      assertTrue(errBytes.toString(StandardCharsets.UTF_8).contains("Joint order mismatch"), errBytes.toString(StandardCharsets.UTF_8));
   }

   /** The encoder log must round-trip bit-exactly, like the mocap log it is paired with. */
   @Test
   public void testEncoderCsvRoundTrip(@TempDir Path directory) throws Exception
   {
      List<String> jointNames = List.of("a", "b", "c");
      List<EncoderSample> written = new ArrayList<>();

      for (int k = 0; k < 5; k++)
      {
         EncoderSample sample = new EncoderSample(jointNames);
         sample.setTimestampNanoseconds(1_000_000_000L + k);
         sample.setQ(new double[] {0.1 * k, -0.3333333333333333, Math.PI});
         written.add(sample);
      }

      Path file = directory.resolve("q.csv");
      CsvEncoderLog.write(file, written);
      List<EncoderSample> read = CsvEncoderLog.read(file);

      assertEquals(written.size(), read.size());

      for (int k = 0; k < written.size(); k++)
      {
         assertEquals(written.get(k).getTimestampNanoseconds(), read.get(k).getTimestampNanoseconds());
         assertEquals(jointNames, read.get(k).getJointNames());

         for (int j = 0; j < jointNames.size(); j++)
            assertEquals(written.get(k).getQ(j), read.get(k).getQ(j), 0.0, "Round trip must be bit-exact.");
      }
   }
}
