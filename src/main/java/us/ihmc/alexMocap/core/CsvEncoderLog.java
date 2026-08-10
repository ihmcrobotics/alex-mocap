package us.ihmc.alexMocap.core;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Reads and writes the encoder half of a capture: {@code q^(k)} plus the timestamp it was read at.
 * <p>
 * The mocap half has {@code MocapFrameRecorder} / {@code CsvReplayMocapSource}; this is its
 * counterpart, and the two together are what {@code --calibrate} eats. Same conventions as the
 * mocap log for the same reasons: the header row is the schema, the file is self-describing, and a
 * log written against a different joint order fails loudly rather than being reinterpreted.
 * </p>
 *
 * <h2>Format</h2>
 *
 * <pre>
 * # alex-mocap encoder log, format 1
 * timestamp_ns,l_hip,l_knee,l_ankle,r_hip,r_knee,r_ankle
 * 1000000000,0.1042,0.8813,-0.2210,0.0455,1.4400,0.1188
 * </pre>
 *
 * <h2>Joint names are in the header, and that is the whole point</h2>
 * <p>
 * A bare column order is a permutation waiting to happen, and a permuted {@code q} produces a
 * completely plausible FK result at the wrong configuration -- indistinguishable downstream from a
 * bad calibration ({@link EncoderSample}). Carrying the names lets
 * {@code RobotModelHandle.setConfiguration} check them against the URDF at the boundary, which is
 * the only place the mistake is still cheap.
 * </p>
 */
public class CsvEncoderLog
{
   public static final int FORMAT_VERSION = 1;
   public static final String HEADER_COMMENT = "# alex-mocap encoder log, format " + FORMAT_VERSION;
   public static final String TIMESTAMP_COLUMN = "timestamp_ns";

   private CsvEncoderLog()
   {
   }

   /** Reads every row. The joint order is taken from the header. */
   public static List<EncoderSample> read(Path file) throws IOException
   {
      try (BufferedReader reader = Files.newBufferedReader(file, StandardCharsets.UTF_8))
      {
         List<String> jointNames = null;
         List<EncoderSample> samples = new ArrayList<>();
         String line;
         int lineNumber = 0;

         while ((line = reader.readLine()) != null)
         {
            lineNumber++;

            if (line.isBlank() || line.startsWith("#"))
               continue;

            String[] fields = line.split(",", -1);

            if (jointNames == null)
            {
               if (!fields[0].trim().equals(TIMESTAMP_COLUMN))
                  throw new IOException(file + " line " + lineNumber + ": expected the header to start with '" + TIMESTAMP_COLUMN + "', got '" + fields[0]
                        + "'.");

               jointNames = new ArrayList<>(Arrays.asList(fields).subList(1, fields.length));

               for (int i = 0; i < jointNames.size(); i++)
                  jointNames.set(i, jointNames.get(i).trim());

               if (jointNames.isEmpty())
                  throw new IOException(file + ": the header names no joints.");

               continue;
            }

            if (fields.length != jointNames.size() + 1)
               throw new IOException(file + " line " + lineNumber + ": expected " + (jointNames.size() + 1) + " fields, got " + fields.length + ".");

            EncoderSample sample = new EncoderSample(jointNames);

            try
            {
               sample.setTimestampNanoseconds(Long.parseLong(fields[0].trim()));

               for (int j = 0; j < jointNames.size(); j++)
                  sample.setQ(j, Double.parseDouble(fields[j + 1].trim()));
            }
            catch (NumberFormatException e)
            {
               throw new IOException(file + " line " + lineNumber + ": " + e.getMessage(), e);
            }

            samples.add(sample);
         }

         if (jointNames == null)
            throw new IOException(file + " has no header row.");

         return samples;
      }
   }

   /** Writes samples in the format {@link #read} expects. All samples must share a joint order. */
   public static void write(Path file, List<EncoderSample> samples) throws IOException
   {
      if (samples.isEmpty())
         throw new IllegalArgumentException("Refusing to write an encoder log with no samples; the header alone carries no joint order to recover.");

      try (BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         write(writer, samples);
      }
   }

   public static void write(Writer writer, List<EncoderSample> samples) throws IOException
   {
      List<String> jointNames = samples.get(0).getJointNames();

      writer.write(HEADER_COMMENT);
      writer.write('\n');
      writer.write(TIMESTAMP_COLUMN);

      for (String jointName : jointNames)
      {
         writer.write(',');
         writer.write(jointName);
      }

      writer.write('\n');

      for (EncoderSample sample : samples)
      {
         sample.checkJointOrder(jointNames);
         writer.write(Long.toString(sample.getTimestampNanoseconds()));

         for (int j = 0; j < jointNames.size(); j++)
         {
            writer.write(',');
            // Double.toString: the shortest text that reads back bit-identical, so a round trip
            // through this pair changes nothing.
            writer.write(Double.toString(sample.getQ(j)));
         }

         writer.write('\n');
      }

      writer.flush();
   }
}
