package us.ihmc.alexMocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import us.ihmc.alexMocap.calibration.RobotCaptures;
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.CalibrationResultIO;
import us.ihmc.alexMocap.core.CsvEncoderLog;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.mocap.MocapFrameRecorder;
import us.ihmc.alexMocap.model.URDFLoader;

/**
 * The two <b>shipping</b> CLIs, unchanged, over the real Alex URDF: capture CSV + encoder CSV +
 * URDF → {@code CalibrationRunner --calibrate} → {@code ReplayRunner} → a CoM trajectory.
 * <p>
 * {@code ReplayRunnerTest} already does this on the toy. What this adds is that the same command
 * lines, with no new flags and no {@code --cluster} overrides, work on a 29-joint 30-link robot
 * whose joint names are {@code LEFT_HIP_X} rather than {@code l_hip} and whose CSVs are five times
 * wider. Nothing in {@code CalibrationRunner} or {@code ReplayRunner} was touched for this PR, and
 * that is the claim under test.
 * </p>
 *
 * <h2>Cluster inference is asserted, not relied on</h2>
 * <p>
 * Clusters are inferred from marker names by the prefix before the last underscore (RUNNING.md,
 * "Watch out for -- gates", item 6). Naming a marker {@code LEFT_THIGH_M0} therefore yields the
 * cluster {@code LEFT_THIGH}, which is a real URDF link name -- but Alex's link names contain
 * underscores themselves, so this is a convention colliding with a naming scheme rather than a
 * fact. It is asserted below rather than assumed, because if it ever broke the CLI would infer a
 * cluster named {@code LEFT_THIGH_M0} (or {@code LEFT}), fail at the URDF boundary, and the reason
 * would not be obvious from the error.
 * </p>
 */
public class AlexLegDemoCliTest
{
   /** Writes the three inputs the calibration CLI needs, from a real-model capture set. */
   private static RobotCaptures.Planted write(Path directory, RobotCaptures.Options options) throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(options);

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(directory.resolve("capture.csv"), planted.markers))
      {
         for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
            recorder.write(planted.captureSet.getCapture(k).getMocapFrame());
      }

      List<EncoderSample> encoders = new ArrayList<>();

      for (int k = 0; k < planted.captureSet.getCaptureCount(); k++)
         encoders.add(planted.captureSet.getCapture(k).getEncoderSample());

      CsvEncoderLog.write(directory.resolve("encoders.csv"), encoders);
      Files.copy(RobotCaptures.alexUrdfPath(), directory.resolve("alex.urdf"), StandardCopyOption.REPLACE_EXISTING);

      return planted;
   }

   private static String[] calibrateArguments(Path directory)
   {
      return new String[] {"--calibrate", "--input", directory.resolve("capture.csv").toString(), "--encoders",
            directory.resolve("encoders.csv").toString(), "--urdf", directory.resolve("alex.urdf").toString(), "--sigma", "0.0003", "--world-tilt", "0.08",
            "--output", directory.resolve("calibration.json").toString()};
   }

   private static String[] replayArguments(Path directory, String... extra)
   {
      List<String> arguments = new ArrayList<>(List.of("--input",
                                                       directory.resolve("capture.csv").toString(),
                                                       "--encoders",
                                                       directory.resolve("encoders.csv").toString(),
                                                       "--urdf",
                                                       directory.resolve("alex.urdf").toString(),
                                                       "--calibration",
                                                       directory.resolve("calibration.json").toString(),
                                                       "--output-directory",
                                                       directory.toString(),
                                                       "--world-tilt",
                                                       "0.08"));
      arguments.addAll(List.of(extra));
      return arguments.toArray(new String[0]);
   }

   /**
    * The whole pipeline through both CLIs on the real model, with no flag that the toy run did not
    * also need.
    */
   @Test
   public void testBothClisRunOnTheRealModel(@TempDir Path directory) throws Exception
   {
      RobotCaptures.Planted planted = write(directory, new RobotCaptures.Options().captures(40).noise(0.3e-3));

      ByteArrayOutputStream calibrateOut = new ByteArrayOutputStream();
      ByteArrayOutputStream calibrateErr = new ByteArrayOutputStream();

      int calibrateExit = CalibrationRunner.run(calibrateArguments(directory),
                                                new PrintStream(calibrateOut, true, StandardCharsets.UTF_8),
                                                new PrintStream(calibrateErr, true, StandardCharsets.UTF_8));

      String calibrateText = calibrateOut.toString(StandardCharsets.UTF_8);
      assertEquals(0, calibrateExit, calibrateText + "\n" + calibrateErr.toString(StandardCharsets.UTF_8));

      // Cluster inference: seven clusters, each named for a real URDF link, no --cluster flag.
      for (String link : RobotCaptures.PRIMARY_MARKED_LINKS)
         assertTrue(calibrateText.contains(link + "(4)"), "The CLI should infer the cluster '" + link + "' from marker names alone.\n" + calibrateText);

      assertTrue(calibrateText.contains("gauge=PELVIS_LINK"), "The gauge should default to the URDF root link.\n" + calibrateText);
      assertTrue(calibrateText.contains("G2: PASS"), "G2 should pass on clean data at the target noise.\n" + calibrateText);

      // Provenance: the hash is of the file that was actually parsed.
      CalibrationResult result = CalibrationResultIO.read(directory.resolve("calibration.json"), planted.markers);
      assertEquals(URDFLoader.sha256(RobotCaptures.alexUrdfPath()), result.getProvenance().urdfSha256(), "The recorded URDF hash must be the real one.");
      assertTrue(result.isFullySolved(), "Every marker should be solved.");
      assertEquals(7, result.getLayouts().size());

      // And every inferred cluster name is a link the model actually has -- which is the assertion
      // that would fire if the underscore convention ever collided with Alex's naming.
      for (var layout : result.getLayouts())
         assertTrue(planted.model.hasLink(layout.getLinkName()), "Inferred cluster '" + layout.getLinkName() + "' is not a URDF link.");

      // ---- the runtime pass ----
      ByteArrayOutputStream replayOut = new ByteArrayOutputStream();
      ByteArrayOutputStream replayErr = new ByteArrayOutputStream();

      int replayExit = ReplayRunner.run(replayArguments(directory, "--velocity", "--error-budget"),
                                        new PrintStream(replayOut, true, StandardCharsets.UTF_8),
                                        new PrintStream(replayErr, true, StandardCharsets.UTF_8));

      String replayText = replayOut.toString(StandardCharsets.UTF_8);
      assertEquals(0, replayExit, "Every frame should produce a CoM on clean data.\n" + replayText + "\n" + replayErr.toString(StandardCharsets.UTF_8));

      for (String name : new String[] {"com.csv", "pelvis.csv", "conditioning.csv", "pelvisVelocity.csv"})
         assertTrue(Files.isRegularFile(directory.resolve(name)), name + " should have been written.\n" + replayText);

      List<String> comLines = Files.readAllLines(directory.resolve("com.csv"));
      assertEquals(42, comLines.size(), "One comment, one header, 40 rows.");

      for (int i = 2; i < comLines.size(); i++)
         assertFalse(comLines.get(i).contains("NaN"), "Every frame should have a CoM: " + comLines.get(i));

      // The two lines of this output that actually matter. Everything else is a repeat of what the
      // toy run already showed; these two are Alex's own numbers and they are the reason for the PR.
      assertTrue(replayText.contains("links        30 total, 7 marked, 23 chained"), "The link accounting must be the real robot's.\n" + replayText);
      assertTrue(replayText.contains("chained mass 53.493 of 91.513 kg on encoders"),
                 "58.45% of Alex's mass is on encoders with only the pelvis and legs marked. "
                       + "That number is the headline of this whole demonstration and the runner already prints it.\n" + replayText);

      // §14's conclusion, printed on the real inertials rather than the toy's.
      assertTrue(replayText.contains("as good as the URDF"), replayText);
      assertTrue(replayText.contains("dominant term"), replayText);
   }

   /**
    * The degenerate marked set, driven through the CLI: it exits 0, prints a small residual, writes
    * a complete-looking {@code CalibrationResult}, and the answer is 56 mm wrong.
    * <p>
    * {@link AlexLegDemoTest#testHipXOnlyMarkedSetIsDegenerateAlongX()} establishes the mechanism
    * against planted truth. This asserts the operational consequence: <b>nothing an operator sees
    * at the console distinguishes this run from a good one.</b> There is no exit code, no warning
    * and no gate for it, because the failure is a gauge freedom and the objective it leaves behind
    * is genuinely small.
    * </p>
    */
   @Test
   public void testTheDegenerateMarkedSetLooksPerfectlyHealthyFromTheCli(@TempDir Path directory) throws Exception
   {
      RobotCaptures.Planted planted = write(directory,
                                            new RobotCaptures.Options().captures(20).noise(0.0).marked(RobotCaptures.HIP_X_ONLY_MARKED_LINKS));

      ByteArrayOutputStream out = new ByteArrayOutputStream();
      ByteArrayOutputStream err = new ByteArrayOutputStream();

      int exitCode = CalibrationRunner.run(calibrateArguments(directory),
                                           new PrintStream(out, true, StandardCharsets.UTF_8),
                                           new PrintStream(err, true, StandardCharsets.UTF_8));

      String text = out.toString(StandardCharsets.UTF_8);
      assertEquals(0, exitCode, "The degenerate run exits 0 -- that is the point.\n" + text + "\n" + err.toString(StandardCharsets.UTF_8));

      CalibrationResult result = CalibrationResultIO.read(directory.resolve("calibration.json"), planted.markers);
      assertTrue(result.isFullySolved(), "And it writes a complete-looking result.");

      double worst = 0.0;

      for (var cluster : planted.clusters)
      {
         var truth = planted.plantedLayout(cluster.getLinkName());
         var estimate = result.getLayout(cluster.getLinkName());

         for (int j = 0; j < cluster.getMarkerCount(); j++)
            worst = Math.max(worst, new us.ihmc.euclid.tuple3D.Point3D(truth.getPositionInLinkFrame(j))
                                                                                                      .distance(new us.ihmc.euclid.tuple3D.Point3D(estimate.getPositionInLinkFrame(j))));
      }

      assertTrue(worst > 0.03, "The layout should be badly wrong despite the clean exit. Measured " + 1000 * worst + " mm.");
      assertTrue(text.contains("G2: PASS"), "G2 passes too: a gauge freedom is not a modelling error, and G2 tests the model.\n" + text);
   }
}
