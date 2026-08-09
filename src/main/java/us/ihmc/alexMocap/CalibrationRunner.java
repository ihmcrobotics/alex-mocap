package us.ihmc.alexMocap;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.alexMocap.calibration.AlternatingCalibrator;
import us.ihmc.alexMocap.calibration.BaseInitializer;
import us.ihmc.alexMocap.calibration.CalibrationReport;
import us.ihmc.alexMocap.calibration.CaptureSet;
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.CalibrationResultIO;
import us.ihmc.alexMocap.core.Capture;
import us.ihmc.alexMocap.core.CsvEncoderLog;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.gates.BootstrapSpreadGate;
import us.ihmc.alexMocap.gates.GateRunner;
import us.ihmc.alexMocap.gates.RigidityGate;
import us.ihmc.alexMocap.mocap.CsvReplayMocapSource;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.model.URDFLoader;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * Command-line entry point for the offline calibration tooling.
 * <p>
 * Two modes. {@code --gate g1} runs the rigidity gate over a captured mocap CSV and needs nothing
 * else -- no URDF, no encoders. {@code --calibrate} runs A' over a capture set plus a URDF and
 * writes a {@code CalibrationResult}, then reports G2 and G4 on the result.
 * </p>
 *
 * <pre>
 * CalibrationRunner --gate g1 --input capture.csv --sigma 0.0003
 *
 * CalibrationRunner --calibrate --input capture.csv --encoders encoders.csv \
 *                   --urdf robot.urdf --gauge pelvis --sigma 0.0003 --output calibration.json
 * </pre>
 *
 * <p>
 * Exits 0 if every gate passed, 1 if a gate failed or could not be fully evaluated, 2 on a usage
 * or I/O error. <b>Incomplete exits non-zero</b>: a check that could not run is not a check that
 * passed, and a green exit code on an unevaluated gate is how a gate stops protecting anything.
 * </p>
 */
public class CalibrationRunner
{
   private static final int EXIT_PASS = 0;
   private static final int EXIT_GATE_NOT_PASSED = 1;
   private static final int EXIT_USAGE = 2;

   public static void main(String[] args)
   {
      System.exit(run(args, System.out, System.err));
   }

   /**
    * The body of {@link #main}, with streams injected and an exit code returned rather than
    * thrown, so the CLI is testable end to end without forking a JVM.
    */
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
         return EXIT_PASS;
      }

      try
      {
         return arguments.calibrate ? runCalibration(arguments, out, err) : runGate(arguments, out, err);
      }
      catch (IOException e)
      {
         err.println("error: could not read " + arguments.input + ": " + e.getMessage());
         return EXIT_USAGE;
      }
      catch (IllegalArgumentException | IllegalStateException e)
      {
         err.println("error: " + e.getMessage());
         return EXIT_USAGE;
      }
   }

   /**
    * Fits A' over a capture set and reports on it.
    * <p>
    * G2 is run <b>after</b> the fit rather than before, using the solved {@code Δ}. FRAMEWORK.md
    * §15 describes G2 as running at time zero, and it can -- but the back-projection it performs is
    * not {@code Δ}-free, so at {@code Δ = I} the spread is inflated by the base offset and the
    * correlations that say <i>which</i> assumption is wrong get swamped. Run at time zero it is a
    * conservative smoke test; run with the solved {@code Δ} it is a diagnosis. This prints the
    * diagnosis.
    * </p>
    */
   private static int runCalibration(Arguments arguments, PrintStream out, PrintStream err) throws IOException
   {
      if (arguments.urdf == null)
         throw new IllegalArgumentException("--calibrate needs --urdf");
      if (arguments.encoders == null)
         throw new IllegalArgumentException("--calibrate needs --encoders");

      RobotModelHandle model = RobotModelHandle.fromURDF(arguments.urdf);
      String gaugeLink = arguments.gauge == null ? model.getBaseLinkName() : arguments.gauge;

      List<EncoderSample> encoderSamples = CsvEncoderLog.read(arguments.encoders);
      List<Capture> captures = new ArrayList<>();
      List<MarkerId> markers;
      List<MarkerCluster> clusters;

      try (CsvReplayMocapSource source = CsvReplayMocapSource.openWithHeaderMarkerSet(arguments.input))
      {
         markers = source.getMarkers();
         clusters = arguments.clusterSpecs.isEmpty() ? inferClusters(markers) : buildClusters(arguments.clusterSpecs, markers);

         while (!source.isFinished() && captures.size() < encoderSamples.size())
         {
            MocapFrame frame = source.createFrame();

            if (source.read(frame))
               captures.add(new Capture(frame, encoderSamples.get(captures.size())));
         }
      }

      if (captures.isEmpty())
         throw new IllegalArgumentException(arguments.input + " and " + arguments.encoders + " share no captures.");

      // Pairing by row index, which is the only thing two independently written logs agree on.
      // FRAMEWORK.md §18.3 lists a mismatch here as a silent failure -- it reads as an estimator
      // regression, not as a bookkeeping error -- so the skew is printed rather than assumed away.
      if (captures.size() != encoderSamples.size())
         out.println("warning: " + captures.size() + " mocap frames against " + encoderSamples.size() + " encoder samples; using the first "
               + captures.size() + ". Rows are paired BY INDEX.");

      CaptureSet captureSet = new CaptureSet(markers, model.getJointNames(), clusters, gaugeLink, captures);

      out.println("urdf     " + arguments.urdf + "  (sha256 " + URDFLoader.sha256(arguments.urdf).substring(0, 16) + "...)");
      out.println("captures " + captureSet.getCaptureCount());
      out.println("clusters " + describe(clusters) + ", gauge=" + gaugeLink);
      out.println("skew     " + describeSkew(captureSet));
      out.println();

      BaseInitializer.GaugeTracking tracking = BaseInitializer.trackGaugeCluster(captureSet);
      CalibrationReport report = new CalibrationReport();
      CalibrationResult result = new AlternatingCalibrator().calibrate(captureSet, model, tracking, report);

      result.setProvenance(new CalibrationResult.Provenance(arguments.urdf.toString(),
                                                            URDFLoader.sha256(arguments.urdf),
                                                            captureSet.getCaptureCount(),
                                                            report.getIterationCount(),
                                                            report.getFinalObjective(),
                                                            arguments.worldTiltRadians,
                                                            java.time.ZonedDateTime.now().toString(),
                                                            arguments.note == null ? "" : arguments.note));

      out.print(report.toTable());
      out.println();

      if (!report.isConverged())
         out.println("warning: A' hit the iteration cap with J still falling. The result is NOT converged.");
      if (!result.getProvenance().hasMeasuredWorldTilt())
         out.println("warning: no --world-tilt given. FRAMEWORK.md §11: theta must be measured, never assumed (~7 mm of CoM height at 0.5 deg).");

      if (arguments.output != null)
      {
         CalibrationResultIO.write(arguments.output, result);
         out.println("wrote " + arguments.output);
      }

      out.println();

      GateRunner runner = new GateRunner();
      runner.add(new BootstrapSpreadGate(captureSet.getCaptures(),
                                         clusters,
                                         model,
                                         result.getClusterToBase(),
                                         clusterPoses(tracking, captureSet.getCaptureCount()),
                                         arguments.sigma));

      GateRunner.Report gateReport = runner.runAll();
      out.print(gateReport.format());

      return gateReport.isPassed() ? EXIT_PASS : EXIT_GATE_NOT_PASSED;
   }

   private static List<RigidBodyTransformReadOnly> clusterPoses(BaseInitializer.GaugeTracking tracking, int captureCount)
   {
      List<RigidBodyTransformReadOnly> poses = new ArrayList<>(captureCount);

      for (int k = 0; k < captureCount; k++)
         poses.add(tracking.isUsable(k) ? tracking.getClusterToWorld(k) : null);

      return poses;
   }

   private static String describeSkew(CaptureSet captureSet)
   {
      long worst = 0;
      int withTimestamps = 0;

      for (Capture capture : captureSet.getCaptures())
      {
         if (capture.hasTimestamps())
         {
            worst = Math.max(worst, Math.abs(capture.getTimestampSkewNanoseconds()));
            withTimestamps++;
         }
      }

      if (withTimestamps == 0)
         return "UNKNOWN -- no capture has both timestamps, so the mocap/encoder pairing cannot be checked at all (FRAMEWORK.md §18.3)";

      return String.format("worst |mocap - encoder| = %.3f ms over %d captures", worst / 1.0e6, withTimestamps);
   }

   private static int runGate(Arguments arguments, PrintStream out, PrintStream err) throws IOException
   {
      if (!"g1".equals(arguments.gate))
      {
         err.println("error: unknown gate '" + arguments.gate + "'. PR1 ships g1; g2 and g4 arrive with the calibrator.");
         return EXIT_USAGE;
      }

      if (!Files.isReadable(arguments.input))
      {
         err.println("error: cannot read " + arguments.input);
         return EXIT_USAGE;
      }

      long frames;
      RigidityGate gate;

      try (CsvReplayMocapSource source = CsvReplayMocapSource.openWithHeaderMarkerSet(arguments.input))
      {
         List<MarkerId> markers = source.getMarkers();
         List<MarkerCluster> clusters = arguments.clusterSpecs.isEmpty() ? inferClusters(markers) : buildClusters(arguments.clusterSpecs, markers);

         out.println("input    " + arguments.input);
         out.println("markers  " + markers.size());
         out.println("clusters " + describe(clusters));
         out.println("sigma    " + String.format("%.4f mm per axis (measured)", 1000.0 * arguments.sigma));
         out.println();

         gate = new RigidityGate(clusters, arguments.sigma, arguments.sigmaMultiplier, arguments.minimumSamples);

         MocapFrame frame = source.createFrame();

         while (!source.isFinished())
         {
            if (source.read(frame))
               gate.accumulate(frame);
         }

         frames = gate.getFramesAccumulated();
      }

      if (frames == 0)
      {
         err.println("error: " + arguments.input + " contains no frames.");
         return EXIT_USAGE;
      }

      GateRunner.Report report = new GateRunner().add(gate).runAll();
      out.print(report.format());

      return report.isPassed() ? EXIT_PASS : EXIT_GATE_NOT_PASSED;
   }

   /**
    * Groups markers into clusters by the prefix before their last underscore, so
    * {@code PELVIS_1, PELVIS_2, ...} become the {@code PELVIS} cluster.
    * <p>
    * A convention, and the CSV header is the only cluster information a log carries. Use
    * {@code --cluster} when the naming does not follow it, or when a link's markers are named
    * inconsistently -- which is exactly when silently inferring the wrong grouping would be worst.
    * </p>
    */
   static List<MarkerCluster> inferClusters(List<MarkerId> markers)
   {
      Map<String, List<MarkerId>> byPrefix = new LinkedHashMap<>();

      for (MarkerId marker : markers)
      {
         int lastUnderscore = marker.getName().lastIndexOf('_');

         if (lastUnderscore <= 0)
            throw new IllegalArgumentException("Marker '" + marker.getName()
                  + "' has no '<cluster>_<n>' prefix, so clusters cannot be inferred. Pass --cluster explicitly.");

         byPrefix.computeIfAbsent(marker.getName().substring(0, lastUnderscore), key -> new ArrayList<>()).add(marker);
      }

      List<MarkerCluster> clusters = new ArrayList<>(byPrefix.size());

      for (Map.Entry<String, List<MarkerId>> entry : byPrefix.entrySet())
      {
         if (entry.getValue().size() < MarkerCluster.MINIMUM_MARKERS)
            throw new IllegalArgumentException("Inferred cluster '" + entry.getKey() + "' has only " + entry.getValue().size()
                  + " markers. Pass --cluster explicitly if the naming does not match the mounting.");

         clusters.add(new MarkerCluster(entry.getKey(), entry.getValue()));
      }

      return clusters;
   }

   private static List<MarkerCluster> buildClusters(List<String> specs, List<MarkerId> markers)
   {
      List<MarkerCluster> clusters = new ArrayList<>(specs.size());

      for (String spec : specs)
      {
         int equals = spec.indexOf('=');

         if (equals <= 0)
            throw new IllegalArgumentException("--cluster expects <name>=<marker,marker,...>, got '" + spec + "'.");

         String linkName = spec.substring(0, equals).trim();
         String[] names = spec.substring(equals + 1).split(",");
         List<MarkerId> members = new ArrayList<>(names.length);

         for (String name : names)
            members.add(find(markers, name.trim()));

         clusters.add(new MarkerCluster(linkName, members));
      }

      return clusters;
   }

   private static MarkerId find(List<MarkerId> markers, String name)
   {
      for (MarkerId marker : markers)
      {
         if (marker.getName().equals(name))
            return marker;
      }

      throw new IllegalArgumentException("No marker named '" + name + "' in the log.");
   }

   private static String describe(List<MarkerCluster> clusters)
   {
      StringBuilder description = new StringBuilder();

      for (int i = 0; i < clusters.size(); i++)
      {
         MarkerCluster cluster = clusters.get(i);
         description.append(i == 0 ? "" : ", ").append(cluster.getLinkName()).append('(').append(cluster.getMarkerCount()).append(')');

         if (!cluster.hasRecommendedRedundancy())
            description.append('*');
      }

      boolean anyThin = clusters.stream().anyMatch(cluster -> !cluster.hasRecommendedRedundancy());
      return description + (anyThin ? "   (* fewer than 4 markers: no redundancy for G1 to check)" : "");
   }

   private static void printUsage(PrintStream stream)
   {
      stream.println("""
            Usage: CalibrationRunner --gate g1    --input <csv> --sigma <metres> [options]
                   CalibrationRunner --calibrate --input <csv> --encoders <csv> --urdf <file>
                                     --sigma <metres> [options]

            Mode 1 (--gate): runs a pre-flight gate over a captured mocap log. No URDF,
            no encoders. G1 is the gate to run first, always -- it is the only one that is
            purely a mocap-and-mounting question (FRAMEWORK.md §15).

            Mode 2 (--calibrate): runs A' over a capture set plus a URDF, writes a
            CalibrationResult, and reports G2 on the solved result.

            Required:
              --gate <name>        gate to run: 'g1' (rigidity). Mode 1.
              --calibrate          run the calibration. Mode 2.
              --input <file>       mocap CSV written by MocapFrameRecorder.
              --sigma <metres>     MEASURED per-axis mocap position noise at the gantry,
                                   e.g. 0.0003 for 0.3 mm. There is no default: the wand
                                   residual is an average over the whole lab and is not a
                                   substitute (FRAMEWORK.md §17, §20.1).

            Required for --calibrate:
              --encoders <file>    encoder CSV (CsvEncoderLog). Rows are paired with the
                                   mocap rows BY INDEX, and the worst timestamp skew is
                                   printed -- a mispairing is silent otherwise (§18.3).
              --urdf <file>        the URDF to calibrate against.

            Optional:
              --output <file>      write the CalibrationResult as JSON.
              --gauge <link>       link carrying the gauge cluster. Defaults to the URDF
                                   root link, which is what Delta = ^c T_b is defined
                                   against. FRAMEWORK.md §1: it must be the pelvis, not
                                   the torso.
              --world-tilt <deg>   MEASURED F8 world tilt. Recorded in provenance. With no
                                   value the result records NaN and says so, which is the
                                   honest encoding of "nobody measured it" (§11).
              --note <text>        free-form note recorded in provenance.
              --cluster <name>=<m1,m2,...>   define a cluster explicitly. Repeatable.
                                   Without this, clusters are inferred from marker names by
                                   the prefix before the last underscore: PELVIS_1, PELVIS_2
                                   become the PELVIS cluster.
              --sigma-multiplier <k>   threshold is k*sigma. Default 3 (FRAMEWORK.md §15).
                                   Note the noise floor is sqrt(2)*sigma, so k=3 is a 2.1x
                                   margin, not 3x.
              --min-samples <n>    co-visible frames a pair needs before it is judged.
                                   Default 100. Below this a pair is reported NOT EVALUATED.
              --help

            Exit codes:
              0  every gate passed
              1  a gate failed, or could not be fully evaluated
              2  usage or I/O error""");
   }

   /** Parsed command line. Package-private so the CLI test can build one without a string round trip. */
   static final class Arguments
   {
      String gate;
      boolean calibrate = false;
      Path input;
      Path encoders;
      Path urdf;
      Path output;
      String gauge;
      String note;
      double worldTiltRadians = Double.NaN;
      double sigma = Double.NaN;
      double sigmaMultiplier = RigidityGate.DEFAULT_SIGMA_MULTIPLIER;
      int minimumSamples = RigidityGate.DEFAULT_MINIMUM_SAMPLES;
      List<String> clusterSpecs = new ArrayList<>();
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
               case "--gate" -> arguments.gate = value(args, ++i, "--gate");
               case "--calibrate" -> arguments.calibrate = true;
               case "--encoders" -> arguments.encoders = Path.of(value(args, ++i, "--encoders"));
               case "--urdf" -> arguments.urdf = Path.of(value(args, ++i, "--urdf"));
               case "--output" -> arguments.output = Path.of(value(args, ++i, "--output"));
               case "--gauge" -> arguments.gauge = value(args, ++i, "--gauge");
               case "--note" -> arguments.note = value(args, ++i, "--note");
               case "--world-tilt" -> arguments.worldTiltRadians = Math.toRadians(Double.parseDouble(value(args, ++i, "--world-tilt")));
               case "--input" -> arguments.input = Path.of(value(args, ++i, "--input"));
               case "--sigma" -> arguments.sigma = positiveDouble(value(args, ++i, "--sigma"), "--sigma");
               case "--sigma-multiplier" -> arguments.sigmaMultiplier = positiveDouble(value(args, ++i, "--sigma-multiplier"), "--sigma-multiplier");
               case "--min-samples" -> arguments.minimumSamples = Integer.parseInt(value(args, ++i, "--min-samples"));
               case "--cluster" -> arguments.clusterSpecs.add(value(args, ++i, "--cluster"));
               default -> throw new IllegalArgumentException("unknown option '" + args[i] + "'");
            }
         }

         if (arguments.help)
            return arguments;

         if (arguments.gate == null && !arguments.calibrate)
            throw new IllegalArgumentException("one of --gate or --calibrate is required");
         if (arguments.gate != null && arguments.calibrate)
            throw new IllegalArgumentException("--gate and --calibrate are separate modes; pass one");
         if (arguments.input == null)
            throw new IllegalArgumentException("--input is required");
         if (Double.isNaN(arguments.sigma))
            throw new IllegalArgumentException("--sigma is required and has no default; it must be measured at the gantry (FRAMEWORK.md §17)");

         return arguments;
      }

      private static String value(String[] args, int index, String option)
      {
         if (index >= args.length)
            throw new IllegalArgumentException(option + " needs a value");

         return args[index];
      }

      private static double positiveDouble(String text, String option)
      {
         double value;

         try
         {
            value = Double.parseDouble(text);
         }
         catch (NumberFormatException e)
         {
            throw new IllegalArgumentException(option + " expects a number, got '" + text + "'");
         }

         if (!(value > 0.0) || !Double.isFinite(value))
            throw new IllegalArgumentException(option + " must be positive and finite, got " + text);

         return value;
      }
   }
}
