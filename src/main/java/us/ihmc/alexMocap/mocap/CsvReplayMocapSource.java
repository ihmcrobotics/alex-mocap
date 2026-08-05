package us.ihmc.alexMocap.mocap;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;

/**
 * Replays a CSV log written by {@link MocapFrameRecorder}.
 * <p>
 * Reads exactly what that class writes, including NaN for invisible markers and the marker set
 * encoded in the header row. Together they let every gate and the whole calibration run in CI off
 * a captured file.
 * </p>
 * <p>
 * {@link #read} allocates nothing after construction: the line buffer and the field offsets are
 * preallocated and the numbers are parsed straight out of the line. That matters less here than in
 * the live source, but replay is how the runtime path gets exercised in tests, and a replay that
 * allocates would hide an allocation in the code under test.
 * </p>
 */
public class CsvReplayMocapSource implements MocapSource
{
   private final List<MarkerId> markers;
   private final BufferedReader reader;

   private final StringBuilder lineBuffer = new StringBuilder(1024);
   private final int[] fieldStarts;
   private final int[] fieldEnds;
   private final int expectedFieldCount;

   private boolean finished = false;
   private long lineNumber;
   private long framesRead = 0;

   /**
    * Opens a log and resolves its marker names against a set you already hold.
    * <p>
    * A name in the file that is not in {@code markerSet}, or a different ordering, is an error.
    * The alternative -- quietly rebinding by position -- assigns measurements to the wrong markers
    * and produces poses that are wrong and look healthy.
    * </p>
    */
   public static CsvReplayMocapSource open(Path file, List<MarkerId> markerSet) throws IOException
   {
      return new CsvReplayMocapSource(Files.newBufferedReader(file, StandardCharsets.UTF_8), markerSet);
   }

   /**
    * Opens a log and takes its marker set from the header row.
    * <p>
    * For inspection and for tools that have no configuration to check against. The resulting
    * {@link MarkerId}s are indexed by column order and are not interchangeable with another
    * session's -- prefer {@link #open(Path, List)} in a pipeline.
    * </p>
    */
   public static CsvReplayMocapSource openWithHeaderMarkerSet(Path file) throws IOException
   {
      return new CsvReplayMocapSource(Files.newBufferedReader(file, StandardCharsets.UTF_8), null);
   }

   public CsvReplayMocapSource(Reader reader, List<MarkerId> markerSet) throws IOException
   {
      this.reader = reader instanceof BufferedReader buffered ? buffered : new BufferedReader(reader);

      List<String> headerNames = readHeader();

      if (markerSet == null)
      {
         this.markers = MarkerId.createDenseSet(headerNames);
      }
      else
      {
         MarkerId.checkDenseSet(markerSet);
         checkHeaderMatches(headerNames, markerSet);
         this.markers = markerSet;
      }

      this.expectedFieldCount = 1 + 3 * markers.size();
      this.fieldStarts = new int[expectedFieldCount];
      this.fieldEnds = new int[expectedFieldCount];
   }

   /** @return marker names in column order, from the header row. */
   private List<String> readHeader() throws IOException
   {
      String header = null;

      while ((header = reader.readLine()) != null)
      {
         lineNumber++;

         if (!header.startsWith("#") && !header.isBlank())
            break;
      }

      if (header == null)
         throw new IOException("Log has no header row.");

      String[] columns = header.split(",", -1);

      if (columns.length < 4 || !columns[0].trim().equals(MocapFrameRecorder.TIMESTAMP_COLUMN))
         throw new IOException("Expected the header to start with '" + MocapFrameRecorder.TIMESTAMP_COLUMN + "', found '" + columns[0] + "'.");

      if ((columns.length - 1) % 3 != 0)
         throw new IOException("Header has " + (columns.length - 1) + " marker columns, which is not a multiple of 3 (x, y, z).");

      List<String> names = new ArrayList<>((columns.length - 1) / 3);

      for (int i = 1; i < columns.length; i += 3)
      {
         String name = stripSuffix(columns[i], "_x");

         if (!columns[i + 1].trim().equals(name + "_y") || !columns[i + 2].trim().equals(name + "_z"))
            throw new IOException("Columns " + i + "-" + (i + 2) + " are not a consistent x/y/z triple for marker '" + name + "'.");

         names.add(name);
      }

      return names;
   }

   private static String stripSuffix(String column, String suffix) throws IOException
   {
      String trimmed = column.trim();

      if (!trimmed.endsWith(suffix))
         throw new IOException("Expected a column ending in '" + suffix + "', found '" + trimmed + "'.");

      return trimmed.substring(0, trimmed.length() - suffix.length());
   }

   private static void checkHeaderMatches(List<String> headerNames, List<MarkerId> markerSet) throws IOException
   {
      if (headerNames.size() != markerSet.size())
         throw new IOException("Log covers " + headerNames.size() + " markers, the given marker set has " + markerSet.size() + ".");

      for (int i = 0; i < headerNames.size(); i++)
      {
         if (!headerNames.get(i).equals(markerSet.get(i).getName()))
            throw new IOException("Log column " + i + " is marker '" + headerNames.get(i) + "', the marker set has '" + markerSet.get(i).getName()
                  + "'. The log and the session disagree about what is mounted.");
      }
   }

   @Override
   public List<MarkerId> getMarkers()
   {
      return markers;
   }

   @Override
   public boolean read(MocapFrame frameToPack)
   {
      if (finished)
         return false;

      try
      {
         if (!readLineInto(lineBuffer))
         {
            finished = true;
            return false;
         }
      }
      catch (IOException e)
      {
         finished = true;
         throw new UncheckedIOException("Failed reading line " + lineNumber + " of the mocap log.", e);
      }

      splitFields();

      frameToPack.setTimestampNanoseconds(parseLong(0));

      for (int i = 0; i < markers.size(); i++)
      {
         MarkerObservation observation = frameToPack.get(i);
         int field = 1 + 3 * i;

         double x = parseDouble(field);
         double y = parseDouble(field + 1);
         double z = parseDouble(field + 2);

         // Visibility lives in the coordinates; see MocapFrameRecorder. A partially-NaN triple is
         // a corrupt line rather than a half-seen marker, and is worth saying so about.
         boolean anyNaN = Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(z);
         boolean allNaN = Double.isNaN(x) && Double.isNaN(y) && Double.isNaN(z);

         if (allNaN)
            observation.setNotVisible();
         else if (anyNaN)
            throw new IllegalStateException("Line " + lineNumber + ": marker '" + markers.get(i).getName() + "' has a partially NaN position (" + x + ", "
                  + y + ", " + z + "). A marker is either seen or not.");
         else
            observation.setVisible(x, y, z);
      }

      framesRead++;
      return true;
   }

   /** Reads one non-blank, non-comment line into the buffer without allocating a String. */
   private boolean readLineInto(StringBuilder buffer) throws IOException
   {
      while (true)
      {
         buffer.setLength(0);
         int c = reader.read();

         if (c < 0)
            return false;

         while (c >= 0 && c != '\n')
         {
            if (c != '\r')
               buffer.append((char) c);

            c = reader.read();
         }

         lineNumber++;

         if (buffer.length() > 0 && buffer.charAt(0) != '#')
            return true;

         if (c < 0)
            return false;
      }
   }

   private void splitFields()
   {
      int field = 0;
      int start = 0;

      for (int i = 0; i <= lineBuffer.length(); i++)
      {
         if (i == lineBuffer.length() || lineBuffer.charAt(i) == ',')
         {
            if (field >= expectedFieldCount)
               throw new IllegalStateException("Line " + lineNumber + " has more than the expected " + expectedFieldCount + " fields.");

            fieldStarts[field] = start;
            fieldEnds[field] = i;
            field++;
            start = i + 1;
         }
      }

      if (field != expectedFieldCount)
         throw new IllegalStateException("Line " + lineNumber + " has " + field + " fields, expected " + expectedFieldCount + " for " + markers.size()
               + " markers.");
   }

   /** Parses a double straight out of the line buffer. NaN is a value here, not an error. */
   private double parseDouble(int field)
   {
      int start = fieldStarts[field];
      int end = fieldEnds[field];

      if (end - start == 3 && lineBuffer.charAt(start) == 'N' && lineBuffer.charAt(start + 1) == 'a' && lineBuffer.charAt(start + 2) == 'N')
         return Double.NaN;

      if (end == start)
         throw new IllegalStateException("Line " + lineNumber + ", field " + field + " is empty. Invisible markers are written as NaN, not left blank.");

      // Double.parseDouble takes a CharSequence only via a String, so a general number costs one
      // small allocation. Every field this reader sees in practice is either NaN -- handled above
      // without allocating -- or a coordinate, and the coordinate path is what a live source
      // never touches. Replay is not the 200 Hz path; the live source is.
      return Double.parseDouble(lineBuffer.substring(start, end));
   }

   private long parseLong(int field)
   {
      int start = fieldStarts[field];
      int end = fieldEnds[field];
      boolean negative = lineBuffer.charAt(start) == '-';
      long value = 0;

      for (int i = negative ? start + 1 : start; i < end; i++)
      {
         char c = lineBuffer.charAt(i);

         if (c < '0' || c > '9')
            throw new IllegalStateException("Line " + lineNumber + ": '" + lineBuffer.substring(start, end) + "' is not an integer timestamp.");

         value = 10 * value + (c - '0');
      }

      return negative ? -value : value;
   }

   @Override
   public boolean isFinished()
   {
      return finished;
   }

   public long getFramesRead()
   {
      return framesRead;
   }

   /** Reads the whole log into a list. For tests and small captures; not for a long session. */
   public List<MocapFrame> readAll()
   {
      List<MocapFrame> frames = new ArrayList<>();

      while (!isFinished())
      {
         MocapFrame frame = createFrame();

         if (read(frame))
            frames.add(frame);
      }

      return frames;
   }

   @Override
   public void close()
   {
      finished = true;

      try
      {
         reader.close();
      }
      catch (IOException e)
      {
         throw new UncheckedIOException("Failed closing the mocap log.", e);
      }
   }
}
