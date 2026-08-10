package us.ihmc.alexMocap.mocap;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;

/**
 * Writes {@link MocapFrame}s to a CSV log that {@link CsvReplayMocapSource} reads back exactly.
 * <p>
 * This pair is the entire test harness for G1 and G2 -- capture once at the gantry, then re-run
 * every gate and every calibration off the file with no cameras in the room. It is worth more than
 * the NatNet client, which is why it exists first.
 * </p>
 *
 * <h2>Format</h2>
 *
 * <pre>
 * # alex-mocap frame log, format 1
 * # invisible markers are recorded as NaN
 * timestamp_ns,PELVIS_1_x,PELVIS_1_y,PELVIS_1_z,PELVIS_2_x,PELVIS_2_y,PELVIS_2_z
 * 1000000000,0.0612,-0.0344,0.0891,NaN,NaN,NaN
 * </pre>
 *
 * <p>
 * <b>Visibility is encoded in the coordinates, not in a separate flag.</b> A visible marker always
 * has a finite position -- {@code MarkerObservation.setVisible} rejects anything else -- so NaN
 * carries the information exactly, and there is no second column that can disagree with the first
 * three. It also plots correctly: NaN reads as a gap in every tool anyone will open this file
 * with, where a zero would read as the marker jumping to the world origin.
 * </p>
 * <p>
 * The header row is the schema. Marker names and their order are recovered from it on read, so a
 * log is self-describing and a log written with a different marker set fails loudly instead of
 * being reinterpreted.
 * </p>
 * <p>
 * Positions are written with {@code Double.toString}, the shortest representation that reads back
 * bit-identical. A round trip through this pair changes nothing.
 * </p>
 */
public class MocapFrameRecorder implements AutoCloseable
{
   public static final int FORMAT_VERSION = 1;
   public static final String HEADER_COMMENT = "# alex-mocap frame log, format " + FORMAT_VERSION;
   public static final String NAN_COMMENT = "# invisible markers are recorded as NaN";
   public static final String TIMESTAMP_COLUMN = "timestamp_ns";

   private final List<MarkerId> markers;
   private final Writer writer;
   private final StringBuilder line = new StringBuilder(256);
   private long framesWritten = 0;

   public MocapFrameRecorder(Path file, List<MarkerId> markers) throws IOException
   {
      this(Files.newBufferedWriter(file, StandardCharsets.UTF_8), markers);
   }

   public MocapFrameRecorder(Writer writer, List<MarkerId> markers) throws IOException
   {
      MarkerId.checkDenseSet(markers);

      this.markers = markers;
      this.writer = writer instanceof BufferedWriter ? writer : new BufferedWriter(writer);

      writeHeader();
   }

   private void writeHeader() throws IOException
   {
      line.setLength(0);
      line.append(HEADER_COMMENT).append('\n');
      line.append(NAN_COMMENT).append('\n');
      line.append(TIMESTAMP_COLUMN);

      for (MarkerId marker : markers)
      {
         line.append(',').append(marker.getName()).append("_x");
         line.append(',').append(marker.getName()).append("_y");
         line.append(',').append(marker.getName()).append("_z");
      }

      line.append('\n');
      writer.write(line.toString());
   }

   /**
    * Appends one frame.
    *
    * @throws IllegalArgumentException if the frame's marker set differs from this recorder's, or
    *                                  if it carries no timestamp. An untimestamped frame in a log
    *                                  cannot be paired with encoders later, and FRAMEWORK.md §18.3
    *                                  is about exactly that pairing.
    */
   public void write(MocapFrame frame) throws IOException
   {
      if (frame.getMarkerCount() != markers.size())
         throw new IllegalArgumentException("Frame has " + frame.getMarkerCount() + " markers; this log is for " + markers.size() + ".");
      if (frame.getTimestampNanoseconds() == MocapFrame.NO_TIMESTAMP)
         throw new IllegalArgumentException("Refusing to log a frame with no timestamp.");

      line.setLength(0);
      line.append(frame.getTimestampNanoseconds());

      for (int i = 0; i < markers.size(); i++)
      {
         MarkerObservation observation = frame.get(i);

         if (!observation.getId().equals(markers.get(i)))
            throw new IllegalArgumentException("Frame marker " + observation.getId() + " does not match log marker " + markers.get(i) + ".");

         if (observation.isVisible())
         {
            line.append(',').append(observation.getPosition().getX());
            line.append(',').append(observation.getPosition().getY());
            line.append(',').append(observation.getPosition().getZ());
         }
         else
         {
            line.append(",NaN,NaN,NaN");
         }
      }

      line.append('\n');
      writer.write(line.toString());
      framesWritten++;
   }

   public long getFramesWritten()
   {
      return framesWritten;
   }

   public void flush() throws IOException
   {
      writer.flush();
   }

   @Override
   public void close() throws IOException
   {
      writer.close();
   }
}
