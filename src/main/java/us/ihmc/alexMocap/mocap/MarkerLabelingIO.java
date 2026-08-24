package us.ihmc.alexMocap.mocap;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.io.StringWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.MarkerId;

/**
 * Reads and writes a {@link MarkerLabeling} as a two-column text file.
 *
 * <h2>Why this exists</h2>
 * <p>
 * The mapping from Motive's streaming ids to marker names is session data, not code. It changes
 * whenever markers are re-stuck or an asset is rebuilt, it is produced on the mocap machine, and it
 * has to survive the trip to whatever consumes the stream. Without a file format the only way to
 * express it is a {@code Map} literal compiled into something, which is exactly the shape that
 * quietly goes stale between sessions.
 * </p>
 *
 * <h2>Why text and not JSON</h2>
 * <p>
 * It is a list of int-to-string pairs, and it wants to be diffable and hand-editable at a bench.
 * {@code CalibrationResultIO}'s parser is private and specific to that structure; a second copy of
 * it to read two columns would be more code than the format deserves.
 * </p>
 *
 * <h2>The naming rule, which is load-bearing</h2>
 * <p>
 * Marker names are how clusters are inferred: everything before the <b>last</b> underscore is taken
 * as the link name and it must match the URDF exactly. {@code PELVIS_LINK_M0} yields the cluster
 * {@code PELVIS_LINK}; {@code PELVIS_M0} yields {@code PELVIS}, which is not a link, and the run
 * fails at startup. That failure is loud and is the good case -- what this class cannot catch is a
 * marker physically mounted on the thigh and named as a shank marker, which calibrates cleanly
 * against the wrong link. See {@link MarkerLabeling}'s note on what G1 can and cannot see.
 * </p>
 *
 * <h2>Format</h2>
 *
 * <pre>
 * # alex-mocap marker labelling, format 1
 * # motiveId,markerName
 * 65536,PELVIS_LINK_M0
 * 65537,PELVIS_LINK_M1
 * 65538,PELVIS_LINK_M2
 * </pre>
 *
 * <p>
 * Blank lines and {@code #} comments are ignored. Line order sets the dense marker indices when
 * reading with {@link #read(Path)}, so the file is the authority on index assignment and a reordered
 * file is a different marker set -- which is why {@link #write(Path, MarkerLabeling)} preserves
 * marker-set order rather than sorting by id.
 * </p>
 */
public class MarkerLabelingIO
{
   public static final int FORMAT_VERSION = 1;

   private static final String HEADER = "# alex-mocap marker labelling, format " + FORMAT_VERSION;
   private static final String COLUMN_HEADER = "# motiveId,markerName";
   private static final String UNFED_PREFIX = "# unfed (in the marker set, fed by no Motive id): ";

   private MarkerLabelingIO()
   {
   }

   public static void write(Path file, MarkerLabeling labeling) throws IOException
   {
      try (Writer writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         write(writer, labeling);
      }
   }

   /**
    * Writes in marker-set order, not sorted by Motive id, so that reading the file back reproduces
    * the same dense indices.
    * <p>
    * Markers that no Motive id feeds are recorded as a comment rather than dropped in silence. They
    * cannot be represented as rows -- a row is a mapping and there is nothing to map -- but losing
    * the fact that the set contained them would turn a detectable configuration error into a cluster
    * that mysteriously never reaches three visible markers.
    * </p>
    */
   public static void write(Writer writer, MarkerLabeling labeling) throws IOException
   {
      writer.write(HEADER);
      writer.write('\n');
      writer.write(COLUMN_HEADER);
      writer.write('\n');

      List<MarkerId> unfed = new ArrayList<>();

      for (MarkerId marker : labeling.getMarkers())
      {
         int motiveId = labeling.motiveIdOf(marker);

         if (motiveId < 0)
         {
            unfed.add(marker);
            continue;
         }

         writer.write(Integer.toString(motiveId));
         writer.write(',');
         writer.write(marker.getName());
         writer.write('\n');
      }

      if (!unfed.isEmpty())
      {
         writer.write(UNFED_PREFIX);

         for (int i = 0; i < unfed.size(); i++)
         {
            if (i > 0)
               writer.write(", ");

            writer.write(unfed.get(i).getName());
         }

         writer.write('\n');
      }

      writer.flush();
   }

   /**
    * Reads a labelling, building the marker set from the file's own line order.
    * <p>
    * This is the form to use when the file is the definition of the session's marker set, which is
    * the normal case: the mocap machine writes it, and everything downstream inherits both the names
    * and the index assignment from it.
    * </p>
    */
   public static MarkerLabeling read(Path file) throws IOException
   {
      try (Reader reader = Files.newBufferedReader(file, StandardCharsets.UTF_8))
      {
         return read(reader, file.toString());
      }
   }

   public static MarkerLabeling read(Reader reader) throws IOException
   {
      return read(reader, "<reader>");
   }

   private static MarkerLabeling read(Reader reader, String source) throws IOException
   {
      List<Row> rows = parse(reader, source);

      List<String> names = new ArrayList<>(rows.size());

      for (Row row : rows)
         names.add(row.markerName);

      List<MarkerId> markerSet = MarkerId.createDenseSet(names);

      return build(rows, markerSet, source);
   }

   /**
    * Reads a labelling against a marker set you already hold.
    * <p>
    * Use this when something else already fixed the marker set -- a calibration file, say -- so that
    * a disagreement between the two surfaces as an error here rather than as two sets of indices
    * that happen to differ.
    * </p>
    */
   public static MarkerLabeling readAgainst(Path file, List<MarkerId> markerSet) throws IOException
   {
      try (Reader reader = Files.newBufferedReader(file, StandardCharsets.UTF_8))
      {
         return readAgainst(reader, markerSet, file.toString());
      }
   }

   public static MarkerLabeling readAgainst(Reader reader, List<MarkerId> markerSet) throws IOException
   {
      return readAgainst(reader, markerSet, "<reader>");
   }

   private static MarkerLabeling readAgainst(Reader reader, List<MarkerId> markerSet, String source) throws IOException
   {
      return build(parse(reader, source), markerSet, source);
   }

   private static MarkerLabeling build(List<Row> rows, List<MarkerId> markerSet, String source)
   {
      MarkerLabeling.Builder builder = MarkerLabeling.against(markerSet);

      for (Row row : rows)
      {
         try
         {
            builder.add(row.motiveId, row.markerName);
         }
         catch (IllegalArgumentException e)
         {
            // The builder's message says what is wrong; this says where, which is the part that is
            // painful to work out by hand in a file with a hundred rows.
            throw new IllegalArgumentException(source + ":" + row.lineNumber + ": " + e.getMessage(), e);
         }
      }

      return builder.build();
   }

   private static List<Row> parse(Reader reader, String source) throws IOException
   {
      BufferedReader bufferedReader = reader instanceof BufferedReader ? (BufferedReader) reader : new BufferedReader(reader);

      List<Row> rows = new ArrayList<>();
      String line;
      int lineNumber = 0;

      while ((line = bufferedReader.readLine()) != null)
      {
         lineNumber++;

         String trimmed = line.trim();

         if (trimmed.isEmpty() || trimmed.startsWith("#"))
            continue;

         int comma = trimmed.indexOf(',');

         if (comma < 0)
            throw new IOException(source + ":" + lineNumber + ": expected 'motiveId,markerName' but found '" + trimmed + "'.");

         String idText = trimmed.substring(0, comma).trim();
         String markerName = trimmed.substring(comma + 1).trim();

         if (markerName.isEmpty())
            throw new IOException(source + ":" + lineNumber + ": the marker name is empty.");

         if (markerName.indexOf(',') >= 0)
            throw new IOException(source + ":" + lineNumber + ": found a second comma in '" + trimmed + "'. This format is two columns.");

         int motiveId;

         try
         {
            motiveId = Integer.parseInt(idText);
         }
         catch (NumberFormatException e)
         {
            throw new IOException(source + ":" + lineNumber + ": '" + idText + "' is not an integer Motive id.", e);
         }

         if (markerName.lastIndexOf('_') <= 0)
         {
            // Caught here rather than at cluster inference, because by then the message is about a
            // link that does not exist and gives no hint that the marker name is the cause.
            throw new IOException(source + ":" + lineNumber + ": marker name '" + markerName
                                  + "' has no underscore to split on. Cluster inference takes everything before the LAST underscore as the URDF"
                                  + " link name, so a marker on PELVIS_LINK must be named like 'PELVIS_LINK_M0'.");
         }

         rows.add(new Row(motiveId, markerName, lineNumber));
      }

      if (rows.isEmpty())
         throw new IOException(source + ": no marker rows. A labelling with nothing in it would silently label nothing.");

      return rows;
   }

   /** Renders a labelling to a string, for logging and for tests. */
   public static String toText(MarkerLabeling labeling)
   {
      StringWriter writer = new StringWriter();

      try
      {
         write(writer, labeling);
      }
      catch (IOException e)
      {
         throw new IllegalStateException("A StringWriter cannot fail to write.", e);
      }

      return writer.toString();
   }

   private static final class Row
   {
      private final int motiveId;
      private final String markerName;
      private final int lineNumber;

      private Row(int motiveId, String markerName, int lineNumber)
      {
         this.motiveId = motiveId;
         this.markerName = markerName;
         this.lineNumber = lineNumber;
      }
   }
}
