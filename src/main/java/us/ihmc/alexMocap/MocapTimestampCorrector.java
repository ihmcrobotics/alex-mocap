package us.ihmc.alexMocap;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Path;
import java.util.List;

import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.mocap.CsvReplayMocapSource;
import us.ihmc.alexMocap.mocap.MocapFrameRecorder;
import us.ihmc.alexMocap.core.MocapFrame;

/**
 * Shifts every frame's timestamp in a {@code .mocap.csv} by a constant offset and writes a
 * corrected copy, so a lag measured once (e.g. by {@code TouchdownLagEstimator} in the {@code alex}
 * repo, comparing marker-based foot touchdown against the robot's own contact detector) can be
 * applied to the file everything downstream actually reads.
 *
 * <pre>
 * MocapTimestampCorrector --input capture.mocap.csv --output capture.corrected.mocap.csv --offset-ns -45000000
 * </pre>
 *
 * <h2>Why only {@code .mocap.csv}</h2>
 * <p>
 * {@code .encoders.csv}'s {@code timestamp_ns} is read from the OCU's own clock ({@code
 * CsvEncoderLog}, {@code System.nanoTime()}), never from a mocap frame -- it already is the
 * reference clock everything else is measured against. Shifting it would move the reference instead
 * of correcting the thing that is actually offset. See {@code FRAMEWORK.md} section 18.3 on why the
 * two logs are paired by index, not by timestamp, downstream -- this tool exists precisely because
 * that pairing assumes the timestamps already agree, and a measured lag says they do not, yet.
 * </p>
 * <p>
 * A positive {@code --offset-ns} moves every mocap timestamp later (use this when the mocap signal
 * was measured to arrive/reach a state <i>before</i> the robot's own clock says it did, e.g. a
 * negative lag from {@code TouchdownLagEstimator}); a negative offset moves it earlier.
 * </p>
 *
 * <p>
 * Exits 0 on success, 2 on a usage or I/O error.
 * </p>
 */
public class MocapTimestampCorrector
{
   private static final int EXIT_OK = 0;
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
         return correct(arguments, out);
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

   private static int correct(Arguments arguments, PrintStream out) throws IOException
   {
      long framesWritten = 0;
      long firstTimestampBefore = MocapFrame.NO_TIMESTAMP;
      long lastTimestampBefore = MocapFrame.NO_TIMESTAMP;

      try (CsvReplayMocapSource source = CsvReplayMocapSource.openWithHeaderMarkerSet(arguments.input))
      {
         List<MarkerId> markers = source.getMarkers();
         MocapFrame frame = source.createFrame();

         try (MocapFrameRecorder recorder = new MocapFrameRecorder(arguments.output, markers))
         {
            while (!source.isFinished())
            {
               if (!source.read(frame))
                  continue;

               if (firstTimestampBefore == MocapFrame.NO_TIMESTAMP)
                  firstTimestampBefore = frame.getTimestampNanoseconds();
               lastTimestampBefore = frame.getTimestampNanoseconds();

               frame.setTimestampNanoseconds(frame.getTimestampNanoseconds() + arguments.offsetNanoseconds);
               recorder.write(frame);
               framesWritten++;
            }
         }
      }

      if (framesWritten == 0)
         throw new IllegalArgumentException("No frames were read from " + arguments.input + ".");

      out.println("input        " + arguments.input);
      out.println("output       " + arguments.output);
      out.println("offset       " + arguments.offsetNanoseconds + " ns (" + (arguments.offsetNanoseconds / 1.0e6) + " ms)");
      out.println("frames       " + framesWritten);
      out.println(String.format("time range   [%d, %d] ns before  ->  [%d, %d] ns after",
                                 firstTimestampBefore,
                                 lastTimestampBefore,
                                 firstTimestampBefore + arguments.offsetNanoseconds,
                                 lastTimestampBefore + arguments.offsetNanoseconds));

      return EXIT_OK;
   }

   private static void printUsage(PrintStream stream)
   {
      stream.println("""
            Usage: MocapTimestampCorrector --input <csv> --output <csv> --offset-ns <long>
                   MocapTimestampCorrector --input <csv> --output <csv> --offset-ms <double>

            Rewrites a MocapFrameRecorder .mocap.csv with every timestamp_ns shifted by a constant
            offset, to correct a lag measured against the robot's own clock (e.g. by
            TouchdownLagEstimator, comparing marker-based foot touchdown against the robot's own
            contact detector). Does not touch .encoders.csv -- that file's timestamps are the OCU's
            own clock and are the reference, not the thing being corrected.

            Required:
              --input <file>       mocap CSV written by MocapFrameRecorder.
              --output <file>      where the corrected copy is written. Must differ from --input.

            Exactly one of:
              --offset-ns <long>   offset in nanoseconds. Positive moves timestamps later.
              --offset-ms <double> offset in milliseconds, converted to nanoseconds.

            Exit codes:
              0  wrote the corrected file
              2  usage or I/O error""");
   }

   static final class Arguments
   {
      Path input;
      Path output;
      long offsetNanoseconds;
      boolean offsetGiven = false;
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
               case "--input" -> arguments.input = Path.of(value(args, ++i, "--input"));
               case "--output" -> arguments.output = Path.of(value(args, ++i, "--output"));
               case "--offset-ns" ->
               {
                  requireOffsetNotAlreadyGiven(arguments);
                  arguments.offsetNanoseconds = Long.parseLong(value(args, ++i, "--offset-ns"));
                  arguments.offsetGiven = true;
               }
               case "--offset-ms" ->
               {
                  requireOffsetNotAlreadyGiven(arguments);
                  arguments.offsetNanoseconds = Math.round(Double.parseDouble(value(args, ++i, "--offset-ms")) * 1.0e6);
                  arguments.offsetGiven = true;
               }
               default -> throw new IllegalArgumentException("unknown option '" + args[i] + "'");
            }
         }

         if (arguments.help)
            return arguments;

         require(arguments.input, "--input");
         require(arguments.output, "--output");

         if (!arguments.offsetGiven)
            throw new IllegalArgumentException("one of --offset-ns or --offset-ms is required");

         if (arguments.input.equals(arguments.output))
            throw new IllegalArgumentException("--output must differ from --input, refusing to overwrite the source file");

         return arguments;
      }

      private static void requireOffsetNotAlreadyGiven(Arguments arguments)
      {
         if (arguments.offsetGiven)
            throw new IllegalArgumentException("give only one of --offset-ns / --offset-ms");
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
