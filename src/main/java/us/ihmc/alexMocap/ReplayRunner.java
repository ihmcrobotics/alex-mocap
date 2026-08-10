package us.ihmc.alexMocap;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.CalibrationResultIO;
import us.ihmc.alexMocap.core.CsvEncoderLog;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.GroundTruthSample;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.frames.GravityAlignedWorldFrame;
import us.ihmc.alexMocap.frames.TiltMeasurement;
import us.ihmc.alexMocap.mocap.CsvReplayMocapSource;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.postprocess.COMErrorBudget;
import us.ihmc.alexMocap.postprocess.PelvisTwistEstimator;
import us.ihmc.alexMocap.runtime.CenterOfMassGroundTruth;
import us.ihmc.alexMocap.runtime.KinematicChainCoupler;
import us.ihmc.alexMocap.runtime.LinkPoseEstimator;
import us.ihmc.alexMocap.runtime.MeasuredLinkPoses;
import us.ihmc.alexMocap.runtime.PelvisStateExtractor;
import us.ihmc.alexMocap.scs2.ConditioningMonitor;
import us.ihmc.alexMocap.scs2.GroundTruthSessionVisualizer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * The runtime entry point: a logged mocap CSV plus a {@code CalibrationResult} in, a CoM
 * trajectory, a pelvis pose trajectory and per-frame conditioning out.
 *
 * <pre>
 * ReplayRunner --input capture.csv --encoders encoders.csv --urdf robot.urdf \
 *              --calibration calibration.json --output-directory out/
 * </pre>
 *
 * <p>
 * Runs F6 → F7 → F9 → F10 per frame, then optionally the offline velocity pass (F10's second half)
 * and the error budget (F11).
 * </p>
 *
 * <h2>Velocity is a separate flag, and off by default</h2>
 * <p>
 * {@code --velocity} runs {@code PelvisTwistEstimator} over the pose trajectory <i>after</i> the
 * whole log is read, because a centred Savitzky-Golay window needs samples from the future
 * (FRAMEWORK.md §13). It is a distinct pass over a completed file, not a column the runtime loop
 * fills, and the CLI keeps that visible rather than blurring it into the per-frame output.
 * </p>
 *
 * <p>
 * Exits 0 if every frame produced a CoM, 1 if any frame was refused, 2 on a usage or I/O error. A
 * refusal is not a warning here: a frame with a missing link has no CoM at all.
 * </p>
 */
public class ReplayRunner
{
   private static final int EXIT_OK = 0;
   private static final int EXIT_REFUSALS = 1;
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
         return replay(arguments, out, err);
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

   private static int replay(Arguments arguments, PrintStream out, PrintStream err) throws IOException
   {
      RobotModelHandle model = RobotModelHandle.fromURDF(arguments.urdf);
      List<EncoderSample> encoderSamples = CsvEncoderLog.read(arguments.encoders);

      TiltMeasurement tilt = Double.isNaN(arguments.worldTiltDegrees)
            ? TiltMeasurement.assumedLevel("no --world-tilt given to ReplayRunner")
            : TiltMeasurement.fromTiltAngles(Math.toRadians(arguments.worldTiltDegrees), 0.0, TiltMeasurement.Method.PRECISION_LEVEL, "--world-tilt");

      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(tilt);

      List<GroundTruthSample> samples = new ArrayList<>();
      List<RigidBodyTransform> pelvisPoses = new ArrayList<>();
      List<EncoderSample> usedEncoders = new ArrayList<>();
      // Hoisted out of the try so the visualizer, which runs after the source is closed, can see
      // them. Null unless --visualize asked for the window.
      List<MocapFrame> retainedFrames = null;
      List<MarkerCluster> viewClusters = null;
      ConditioningMonitor monitor;
      long refusedFrames = 0;

      try (CsvReplayMocapSource source = CsvReplayMocapSource.openWithHeaderMarkerSet(arguments.input))
      {
         List<MarkerId> markers = source.getMarkers();
         CalibrationResult calibration = CalibrationResultIO.read(arguments.calibration, markers);
         List<MarkerCluster> clusters = buildClusters(calibration, markers);

         List<String> linkNames = model.getLinkNames();
         List<String> markedLinks = calibration.getLinkNames();

         LinkPoseEstimator poseEstimator = new LinkPoseEstimator(calibration, clusters, linkNames);
         KinematicChainCoupler coupler = new KinematicChainCoupler(model, linkNames, markedLinks);
         CenterOfMassGroundTruth centerOfMass = CenterOfMassGroundTruth.forWholeRobot(model, world.getMotiveWorld(), world.getGravityAlignedWorld());
         PelvisStateExtractor pelvis = new PelvisStateExtractor(model.getBaseLinkName(), linkNames, world.getGravityAlignedWorld(), world.getMotiveWorld());

         int maximumMarkers = 0;

         for (MarkerCluster cluster : clusters)
            maximumMarkers = Math.max(maximumMarkers, cluster.getMarkerCount());

         monitor = new ConditioningMonitor(markedLinks, maximumMarkers);

         out.println("urdf         " + arguments.urdf);
         out.println("calibration  " + arguments.calibration + "  (" + calibration.getProvenance() + ")");
         out.println("links        " + linkNames.size() + " total, " + markedLinks.size() + " marked, " + coupler.getChainedLinkCount() + " chained");
         out.println(String.format("chained mass %.3f of %.3f kg on encoders (FRAMEWORK.md section 10)", coupler.getChainedMass(), model.getTotalMass()));
         out.println("world tilt   " + tilt);
         out.println();

         MeasuredLinkPoses poses = new MeasuredLinkPoses(linkNames);
         MocapFrame frame = source.createFrame();
         Point3D com = new Point3D();

         // Kept only when the window is going to be opened. A replay of a long session holds one
         // frame at a time on purpose; retaining every frame for a run that prints CSVs and exits
         // would be pure memory for nothing.
         if (arguments.visualize)
         {
            retainedFrames = new ArrayList<>();
            viewClusters = clusters;
         }

         while (!source.isFinished() && samples.size() < encoderSamples.size())
         {
            if (!source.read(frame))
               continue;

            EncoderSample encoders = encoderSamples.get(samples.size());

            poses.clear();
            poseEstimator.estimate(frame, poses);
            coupler.complete(encoders, poses);

            boolean complete = centerOfMass.compute(poses, com);
            pelvis.extract(poses);

            GroundTruthSample sample = new GroundTruthSample(markedLinks);
            sample.setTimestampNanoseconds(frame.getTimestampNanoseconds());

            if (complete)
               sample.setCenterOfMass(com);

            sample.getPelvisPose().set(pelvis.getPelvisPose());

            for (int i = 0; i < markedLinks.size(); i++)
            {
               int linkIndex = poses.indexOf(markedLinks.get(i));
               sample.setConditioning(i, poses.getSigma3(linkIndex), poses.getVisibleCount(linkIndex), poses.isAvailable(linkIndex));
            }

            if (!complete)
            {
               refusedFrames++;

               if (refusedFrames <= 5)
                  err.println("frame " + samples.size() + ": no CoM -- " + centerOfMass.getMissingLinkCount() + " link(s) missing, "
                        + String.format("%.3f kg", centerOfMass.getMissingMass()) + " unmeasured. First reason: " + firstRefusal(poses));
            }

            if (retainedFrames != null)
            {
               MocapFrame copy = source.createFrame();
               copy.set(frame);
               retainedFrames.add(copy);
            }

            monitor.accumulate(sample);
            samples.add(sample);
            pelvisPoses.add(new RigidBodyTransform(pelvis.getPelvisPose()));
            usedEncoders.add(encoders);
         }
      }

      if (samples.isEmpty())
         throw new IllegalArgumentException("No frames were read from " + arguments.input + ".");

      out.print(monitor.toTable());
      out.println();

      if (refusedFrames > 5)
         err.println("... and " + (refusedFrames - 5) + " more refused frames.");

      writeComTrajectory(arguments.outputDirectory.resolve("com.csv"), samples);
      writePelvisTrajectory(arguments.outputDirectory.resolve("pelvis.csv"), samples);
      writeConditioning(arguments.outputDirectory.resolve("conditioning.csv"), samples);
      out.println("wrote com.csv, pelvis.csv and conditioning.csv to " + arguments.outputDirectory);

      if (arguments.velocity)
      {
         PelvisTwistEstimator twist = PelvisTwistEstimator.withCentredWindow(arguments.velocityWindowSeconds, arguments.sampleRateHz);
         twist.compute(pelvisPoses);
         writeVelocity(arguments.outputDirectory.resolve("pelvisVelocity.csv"), samples, twist);

         out.println();
         out.println("velocity second pass (FRAMEWORK.md section 13)");
         out.println("  window          " + twist.getDifferentiator());
         out.println(String.format("  edge samples    %d at each end are NaN; a centred window has no value there", twist.getEdgeSampleCount()));
         out.println(String.format("  expected noise  %.5f m/s at sigma = %.4f mm", twist.getExpectedVelocityNoise(arguments.sigma), 1000.0 * arguments.sigma));
         out.println(String.format("  ContactNet bar  0.0844 / 0.0254 m/s"));
         out.println("  wrote pelvisVelocity.csv");
      }

      if (arguments.errorBudget)
      {
         out.println();
         out.print(errorBudget(model, samples, arguments).toTable());
      }

      if (arguments.visualize)
      {
         out.println();
         out.println("opening the SCS2 visualizer...");
         GroundTruthSessionVisualizer.show(arguments.urdf,
                                            arguments.meshDirectories,
                                            samples,
                                            usedEncoders,
                                            retainedFrames,
                                            viewClusters,
                                            readTruthBasePoses(arguments.truthBase, samples.size(), err),
                                            world.getGravityAlignedWorld(),
                                            arguments.sampleRateHz);
      }

      return refusedFrames == 0 ? EXIT_OK : EXIT_REFUSALS;
   }

   /**
    * Reads planted base poses for the ghost overlay: {@code timestamp,x,y,z,qx,qy,qz,qs} per line.
    * <p>
    * A missing file is not an error -- most replays have no truth -- and neither is a count that
    * does not match: the ghost is a convenience, and refusing to open the window because a debug
    * CSV is stale would be a poor trade. Both are reported, and the ghost is simply dropped.
    * </p>
    */
   private static List<RigidBodyTransformReadOnly> readTruthBasePoses(Path file, int expected, PrintStream err)
   {
      if (file == null)
         return null;

      try
      {
         List<RigidBodyTransformReadOnly> poses = new ArrayList<>();

         for (String line : Files.readAllLines(file, StandardCharsets.UTF_8))
         {
            if (line.isBlank() || line.startsWith("#") || line.startsWith("timestamp"))
               continue;

            String[] columns = line.split(",");

            if (columns.length < 8)
               throw new IOException("expected 8 columns, got " + columns.length + ": " + line);

            RigidBodyTransform pose = new RigidBodyTransform();
            pose.getTranslation().set(Double.parseDouble(columns[1]), Double.parseDouble(columns[2]), Double.parseDouble(columns[3]));
            pose.getRotation().setQuaternion(Double.parseDouble(columns[4]),
                                             Double.parseDouble(columns[5]),
                                             Double.parseDouble(columns[6]),
                                             Double.parseDouble(columns[7]));
            poses.add(pose);
         }

         if (poses.size() != expected)
         {
            err.println("note: " + file + " holds " + poses.size() + " base poses for " + expected + " frames; drawing no ghost.");
            return null;
         }

         return poses;
      }
      catch (IOException | NumberFormatException e)
      {
         err.println("note: could not read " + file + " (" + e.getMessage() + "); drawing no ghost.");
         return null;
      }
   }

   private static us.ihmc.alexMocap.postprocess.ErrorBudgetReport errorBudget(RobotModelHandle model, List<GroundTruthSample> samples, Arguments arguments)
   {
      List<String> linkNames = model.getLinkNames();
      double[] masses = new double[linkNames.size()];
      Point3D[] centersOfMass = new Point3D[linkNames.size()];
      RigidBodyTransform[] poses = new RigidBodyTransform[linkNames.size()];

      for (int i = 0; i < linkNames.size(); i++)
      {
         masses[i] = model.getMass(linkNames.get(i));
         centersOfMass[i] = new Point3D();
         model.packCenterOfMassInLinkFrame(linkNames.get(i), centersOfMass[i]);
         poses[i] = new RigidBodyTransform();
         model.packLinkToBase(linkNames.get(i), poses[i]);
      }

      return new COMErrorBudget(masses, centersOfMass).evaluate(poses, arguments.massUncertainty, arguments.comUncertainty, arguments.sigma);
   }

   private static String firstRefusal(MeasuredLinkPoses poses)
   {
      for (int i = 0; i < poses.getLinkCount(); i++)
      {
         if (poses.getRefusalReason(i) != null)
            return poses.getLinkNames().get(i) + ": " + poses.getRefusalReason(i);
      }

      return "unknown";
   }

   /** Clusters, taken from the calibration's own layouts so they cannot disagree with it. */
   private static List<MarkerCluster> buildClusters(CalibrationResult calibration, List<MarkerId> markers)
   {
      List<MarkerCluster> clusters = new ArrayList<>();

      for (var layout : calibration.getLayouts())
         clusters.add(new MarkerCluster(layout.getLinkName(), layout.getMarkers()));

      return clusters;
   }

   private static void writeComTrajectory(Path file, List<GroundTruthSample> samples) throws IOException
   {
      try (BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         writer.write("# alex-mocap CoM trajectory, gravity-aligned world frame\n");
         writer.write("timestamp_ns,com_x,com_y,com_z\n");

         for (GroundTruthSample sample : samples)
         {
            writer.write(sample.getTimestampNanoseconds() + "," + sample.getCenterOfMass().getX() + "," + sample.getCenterOfMass().getY() + ","
                  + sample.getCenterOfMass().getZ() + "\n");
         }
      }
   }

   private static void writePelvisTrajectory(Path file, List<GroundTruthSample> samples) throws IOException
   {
      try (BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         writer.write("# alex-mocap pelvis pose trajectory, gravity-aligned world frame. No velocity: see FRAMEWORK.md section 13.\n");
         writer.write("timestamp_ns,x,y,z,qx,qy,qz,qs\n");

         us.ihmc.euclid.tuple4D.Quaternion quaternion = new us.ihmc.euclid.tuple4D.Quaternion();

         for (GroundTruthSample sample : samples)
         {
            quaternion.set(sample.getPelvisPose().getRotation());
            writer.write(sample.getTimestampNanoseconds() + "," + sample.getPelvisPose().getTranslationX() + "," + sample.getPelvisPose().getTranslationY()
                  + "," + sample.getPelvisPose().getTranslationZ() + "," + quaternion.getX() + "," + quaternion.getY() + "," + quaternion.getZ() + ","
                  + quaternion.getS() + "\n");
         }
      }
   }

   private static void writeConditioning(Path file, List<GroundTruthSample> samples) throws IOException
   {
      try (BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         List<String> linkNames = samples.get(0).getLinkNames();

         writer.write("# alex-mocap per-frame conditioning. sigma3 is in m^2 (a mean-squared spread).\n");
         writer.write("timestamp_ns");

         for (String link : linkNames)
            writer.write("," + link + "_sigma3," + link + "_visible," + link + "_accepted");

         writer.write("\n");

         for (GroundTruthSample sample : samples)
         {
            writer.write(Long.toString(sample.getTimestampNanoseconds()));

            for (int i = 0; i < linkNames.size(); i++)
               writer.write("," + sample.getSigma3(i) + "," + sample.getVisibleCount(i) + "," + (sample.isPoseAccepted(i) ? 1 : 0));

            writer.write("\n");
         }
      }
   }

   private static void writeVelocity(Path file, List<GroundTruthSample> samples, PelvisTwistEstimator twist) throws IOException
   {
      try (BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         writer.write("# alex-mocap pelvis twist, offline centred Savitzky-Golay second pass (FRAMEWORK.md section 13).\n");
         writer.write("# The first and last " + twist.getEdgeSampleCount() + " samples are NaN: a centred window has no value there.\n");
         writer.write("timestamp_ns,vx,vy,vz,wx,wy,wz\n");

         Vector3D linear = new Vector3D();
         Vector3D angular = new Vector3D();

         for (int i = 0; i < samples.size(); i++)
         {
            twist.packLinearVelocity(i, linear);
            twist.packAngularVelocity(i, angular);

            writer.write(samples.get(i).getTimestampNanoseconds() + "," + linear.getX() + "," + linear.getY() + "," + linear.getZ() + "," + angular.getX()
                  + "," + angular.getY() + "," + angular.getZ() + "\n");
         }
      }
   }

   private static void printUsage(PrintStream stream)
   {
      stream.println("""
            Usage: ReplayRunner --input <csv> --encoders <csv> --urdf <file> --calibration <json>
                                --output-directory <dir> [options]

            Replays a logged capture through F6 -> F7 -> F9 -> F10 and writes a CoM trajectory, a
            pelvis pose trajectory, and per-frame conditioning.

            Required:
              --input <file>            mocap CSV written by MocapFrameRecorder.
              --encoders <file>         encoder CSV (CsvEncoderLog). Needed by F7 chaining.
              --urdf <file>             the URDF the calibration was solved against.
              --calibration <file>      CalibrationResult JSON from `CalibrationRunner --calibrate`.
              --output-directory <dir>  where com.csv, pelvis.csv and conditioning.csv go.

            Optional:
              --world-tilt <deg>        MEASURED F8 world tilt. Without it the run assumes level
                                        and says so; FRAMEWORK.md section 11 forbids assuming it,
                                        and it is worth ~7 mm of CoM height at 0.5 deg.
              --sigma <metres>          measured per-axis mocap noise, for the velocity noise
                                        estimate and the error budget. Default 0.0003.
              --rate <hz>               sample rate. Default 200.
              --velocity                run the offline Savitzky-Golay second pass and write
                                        pelvisVelocity.csv. Not a runtime quantity: a centred
                                        window needs samples from the future (section 13).
              --velocity-window <s>     centred window for that pass. Default 0.1.
              --error-budget            print the F11 CoM error budget (section 14).
              --mass-uncertainty <f>    per-link mass uncertainty for the budget. Default 0.05.
              --com-uncertainty <m>     per-axis link-CoM uncertainty. Default 0.005.
              --visualize               open the SCS2 visualizer afterwards. Needs a display.
              --truth-base <file>       planted base poses (CSV). Draws a translucent ghost at the
                                        reconstructed pose beside the solid robot at truth.
              --mesh-dir <dir>          root for package:// mesh lookups. Repeatable, searched in
                                        order. Defaults to the URDF's own directory. Alex's meshes
                                        are in ihmc-alex-sdk/alex-models/, not beside the URDF.
              --help

            Exit codes:
              0  every frame produced a CoM
              1  at least one frame was refused (a missing link means NO CoM for that frame)
              2  usage or I/O error""");
   }

   static final class Arguments
   {
      Path input;
      Path encoders;
      Path urdf;
      Path calibration;
      Path outputDirectory;
      double worldTiltDegrees = Double.NaN;
      double sigma = 0.0003;
      double sampleRateHz = 200.0;
      boolean velocity = false;
      double velocityWindowSeconds = 0.1;
      boolean errorBudget = false;
      double massUncertainty = 0.05;
      double comUncertainty = 0.005;
      boolean visualize = false;
      boolean help = false;

      /**
       * Roots for {@code package://} mesh lookups, in search order. Repeatable.
       * <p>
       * Empty means "look beside the URDF", which is the old behaviour and is right when a model
       * ships its meshes with it. Alex's does not: the URDF is pinned by sha256 in the provenance
       * and the meshes live in ihmc-alex-sdk, so the two roots have to be nameable separately.
       * </p>
       */
      final List<String> meshDirectories = new ArrayList<>();

      /**
       * Planted base poses, for the ghost overlay. Only a synthetic session has these; a replay of
       * real captures has no truth to draw, and then there is simply no ghost.
       */
      Path truthBase;

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
               case "--input" -> arguments.input = Path.of(value(args, ++i, "--input"));
               case "--encoders" -> arguments.encoders = Path.of(value(args, ++i, "--encoders"));
               case "--urdf" -> arguments.urdf = Path.of(value(args, ++i, "--urdf"));
               case "--calibration" -> arguments.calibration = Path.of(value(args, ++i, "--calibration"));
               case "--output-directory" -> arguments.outputDirectory = Path.of(value(args, ++i, "--output-directory"));
               case "--world-tilt" -> arguments.worldTiltDegrees = Double.parseDouble(value(args, ++i, "--world-tilt"));
               case "--sigma" -> arguments.sigma = Double.parseDouble(value(args, ++i, "--sigma"));
               case "--rate" -> arguments.sampleRateHz = Double.parseDouble(value(args, ++i, "--rate"));
               case "--velocity" -> arguments.velocity = true;
               case "--velocity-window" -> arguments.velocityWindowSeconds = Double.parseDouble(value(args, ++i, "--velocity-window"));
               case "--error-budget" -> arguments.errorBudget = true;
               case "--mass-uncertainty" -> arguments.massUncertainty = Double.parseDouble(value(args, ++i, "--mass-uncertainty"));
               case "--com-uncertainty" -> arguments.comUncertainty = Double.parseDouble(value(args, ++i, "--com-uncertainty"));
               case "--visualize" -> arguments.visualize = true;
               case "--mesh-dir" -> arguments.meshDirectories.add(value(args, ++i, "--mesh-dir"));
               case "--truth-base" -> arguments.truthBase = Path.of(value(args, ++i, "--truth-base"));
               default -> throw new IllegalArgumentException("unknown option '" + args[i] + "'");
            }
         }

         if (arguments.help)
            return arguments;

         require(arguments.input, "--input");
         require(arguments.encoders, "--encoders");
         require(arguments.urdf, "--urdf");
         require(arguments.calibration, "--calibration");
         require(arguments.outputDirectory, "--output-directory");

         return arguments;
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
