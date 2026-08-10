package us.ihmc.alexMocap.core;

import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Reads and writes a {@link CalibrationResult} as JSON.
 * <p>
 * The JSON is hand-rolled rather than delegated to a binding library. FRAMEWORK.md §19 says core
 * packages see only Euclid, Mecano and EJML, so that the whole calibration stays headless-testable
 * with nothing else on the classpath. The schema here is small and fixed, and writing it out
 * explicitly means the on-disk format is legible in one method instead of being an emergent
 * property of field names and annotations.
 * </p>
 *
 * <h2>Format</h2>
 *
 * <pre>
 * {
 *   "formatVersion": 1,
 *   "provenance": {
 *     "urdf": "alex.urdf",
 *     "urdfSha256": "9f2c...",
 *     "captureCount": 30,
 *     "iterations": 7,
 *     "finalObjective": 3.1e-07,
 *     "worldTiltRadians": 0.0012,
 *     "createdAt": "2026-08-05T14:02:11Z",
 *     "note": ""
 *   },
 *   "delta": [ m00, m01, m02, x, m10, m11, m12, y, m20, m21, m22, z ],
 *   "layouts": {
 *     "pelvis": {
 *       "PELVIS_1": { "p": [0.0612, -0.0344, 0.0891], "k": 30 }
 *     }
 *   }
 * }
 * </pre>
 *
 * <p>
 * {@code delta} is {@code Δ} row-major, 3×4. {@code k} is {@code K_ij}, the capture count the
 * position was averaged over -- dropping it would discard the difference between a marker seen 30
 * times and one seen twice.
 * </p>
 *
 * <h2>Two things the format has to be explicit about</h2>
 * <ul>
 * <li><b>NaN is written as {@code null}.</b> JSON has no NaN literal. A never-observed marker has
 * a NaN position and {@code k = 0}, and that state has to survive a round trip -- writing 0.0
 * instead would turn "never seen" into "seen at the link origin".</li>
 * <li><b>Marker names, not indices.</b> Indices are only meaningful inside one marker set, and a
 * calibration file outlives the process that wrote it. {@link #read(Path, List)} resolves names
 * against the marker set you hand it and throws on anything it cannot resolve, so recalibrating
 * with a changed marker set fails loudly instead of silently rebinding.</li>
 * </ul>
 */
public class CalibrationResultIO
{
   public static final int FORMAT_VERSION = 1;

   /** {@code Δ} on disk is row-major 3×4; the implied (0, 0, 0, 1) row carries no information. */
   private static final int DELTA_ELEMENTS = 12;

   private CalibrationResultIO()
   {
   }

   // ------------------------------------------------------------------------------------------
   // Writing
   // ------------------------------------------------------------------------------------------

   public static void write(Path file, CalibrationResult result) throws IOException
   {
      try (Writer writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8))
      {
         write(writer, result);
      }
   }

   public static void write(Writer writer, CalibrationResult result) throws IOException
   {
      StringBuilder json = new StringBuilder(1024);
      CalibrationResult.Provenance provenance = result.getProvenance();

      json.append("{\n");
      json.append("  \"formatVersion\": ").append(FORMAT_VERSION).append(",\n");

      json.append("  \"provenance\": {\n");
      json.append("    \"urdf\": ").append(quote(provenance.urdf())).append(",\n");
      json.append("    \"urdfSha256\": ").append(quote(provenance.urdfSha256())).append(",\n");
      json.append("    \"captureCount\": ").append(provenance.captureCount()).append(",\n");
      json.append("    \"iterations\": ").append(provenance.iterations()).append(",\n");
      json.append("    \"finalObjective\": ").append(number(provenance.finalObjective())).append(",\n");
      json.append("    \"worldTiltRadians\": ").append(number(provenance.worldTiltRadians())).append(",\n");
      json.append("    \"createdAt\": ").append(quote(provenance.createdAt())).append(",\n");
      json.append("    \"note\": ").append(quote(provenance.note())).append("\n");
      json.append("  },\n");

      // Euclid is asymmetric here: RigidBodyTransform.get(double[]) packs the full 4x4 and needs
      // 16 slots, while set(double[]) reads only the 12 that matter. Elements 0..11 of the 4x4 are
      // exactly the 3x4 that set() expects, in the same order, so writing that prefix round-trips
      // and the trailing (0,0,0,1) row stays off the disk where it carries no information.
      double[] delta = new double[16];
      result.getClusterToBase().get(delta);
      json.append("  \"delta\": [");

      for (int i = 0; i < DELTA_ELEMENTS; i++)
         json.append(i == 0 ? "" : ", ").append(number(delta[i]));

      json.append("],\n");

      json.append("  \"layouts\": {\n");
      List<ClusterLayout> layouts = result.getLayouts();

      for (int l = 0; l < layouts.size(); l++)
      {
         ClusterLayout layout = layouts.get(l);
         json.append("    ").append(quote(layout.getLinkName())).append(": {\n");

         for (int m = 0; m < layout.getMarkerCount(); m++)
         {
            json.append("      ").append(quote(layout.getMarker(m).getName())).append(": { \"p\": [");
            json.append(number(layout.getPositionInLinkFrame(m).getX())).append(", ");
            json.append(number(layout.getPositionInLinkFrame(m).getY())).append(", ");
            json.append(number(layout.getPositionInLinkFrame(m).getZ()));
            json.append("], \"k\": ").append(layout.getObservationCount(m)).append(" }");
            json.append(m == layout.getMarkerCount() - 1 ? "\n" : ",\n");
         }

         json.append("    }").append(l == layouts.size() - 1 ? "\n" : ",\n");
      }

      json.append("  }\n");
      json.append("}\n");

      writer.write(json.toString());
   }

   /** {@code Double.toString} round-trips exactly; NaN and infinities become {@code null}. */
   private static String number(double value)
   {
      return Double.isFinite(value) ? Double.toString(value) : "null";
   }

   private static String quote(String value)
   {
      if (value == null)
         return "null";

      StringBuilder quoted = new StringBuilder(value.length() + 2);
      quoted.append('"');

      for (int i = 0; i < value.length(); i++)
      {
         char c = value.charAt(i);

         switch (c)
         {
            case '"' -> quoted.append("\\\"");
            case '\\' -> quoted.append("\\\\");
            case '\n' -> quoted.append("\\n");
            case '\r' -> quoted.append("\\r");
            case '\t' -> quoted.append("\\t");
            case '\b' -> quoted.append("\\b");
            case '\f' -> quoted.append("\\f");
            default ->
            {
               if (c < 0x20)
                  quoted.append(String.format("\\u%04x", (int) c));
               else
                  quoted.append(c);
            }
         }
      }

      return quoted.append('"').toString();
   }

   // ------------------------------------------------------------------------------------------
   // Reading
   // ------------------------------------------------------------------------------------------

   /**
    * Reads a calibration, resolving marker names against a marker set you already hold.
    * <p>
    * This is the one to use in a pipeline. A name in the file that is not in {@code markerSet} is
    * an error, not a marker to invent: it means the file and the running session disagree about
    * what is mounted on the robot.
    * </p>
    */
   public static CalibrationResult read(Path file, List<MarkerId> markerSet) throws IOException
   {
      try (Reader reader = Files.newBufferedReader(file, StandardCharsets.UTF_8))
      {
         return read(reader, markerSet);
      }
   }

   public static CalibrationResult read(Reader reader, List<MarkerId> markerSet) throws IOException
   {
      Map<String, MarkerId> byName = new LinkedHashMap<>();

      if (markerSet != null)
      {
         for (MarkerId marker : markerSet)
            byName.put(marker.getName(), marker);
      }

      return parse(readAll(reader), byName, markerSet != null);
   }

   /**
    * Reads a calibration with no marker set to resolve against, assigning indices from the order
    * the names appear in the file.
    * <p>
    * For report and inspection tools. The resulting {@link MarkerId}s are <b>not</b> interchangeable
    * with any other session's, so do not feed them to a running pipeline -- that is what
    * {@link #read(Path, List)} is for.
    * </p>
    */
   public static CalibrationResult readWithDenseMarkerSet(Path file) throws IOException
   {
      try (Reader reader = Files.newBufferedReader(file, StandardCharsets.UTF_8))
      {
         return parse(readAll(reader), new LinkedHashMap<>(), false);
      }
   }

   private static String readAll(Reader reader) throws IOException
   {
      StringBuilder contents = new StringBuilder();
      char[] buffer = new char[8192];
      int read;

      while ((read = reader.read(buffer)) >= 0)
         contents.append(buffer, 0, read);

      return contents.toString();
   }

   @SuppressWarnings("unchecked")
   private static CalibrationResult parse(String json, Map<String, MarkerId> byName, boolean strictMarkerSet)
   {
      Object root = new JsonParser(json).parseValue();

      if (!(root instanceof Map))
         throw new IllegalArgumentException("Expected a JSON object at the top level of the calibration file.");

      Map<String, Object> document = (Map<String, Object>) root;
      int formatVersion = (int) requireDouble(document, "formatVersion");

      if (formatVersion != FORMAT_VERSION)
         throw new IllegalArgumentException("Calibration file is format version " + formatVersion + "; this build reads version " + FORMAT_VERSION + ".");

      CalibrationResult result = new CalibrationResult();

      Map<String, Object> provenance = (Map<String, Object>) document.get("provenance");

      if (provenance != null)
      {
         result.setProvenance(new CalibrationResult.Provenance((String) provenance.get("urdf"),
                                                               (String) provenance.get("urdfSha256"),
                                                               (int) optionalDouble(provenance, "captureCount", 0.0),
                                                               (int) optionalDouble(provenance, "iterations", 0.0),
                                                               optionalDouble(provenance, "finalObjective", Double.NaN),
                                                               optionalDouble(provenance, "worldTiltRadians", Double.NaN),
                                                               (String) provenance.getOrDefault("createdAt", ""),
                                                               (String) provenance.getOrDefault("note", "")));
      }

      List<Object> delta = (List<Object>) document.get("delta");

      if (delta == null || delta.size() != DELTA_ELEMENTS)
         throw new IllegalArgumentException(
               "'delta' must be an array of " + DELTA_ELEMENTS + " numbers (row-major 3x4), was " + (delta == null ? "absent" : delta.size()));

      double[] deltaValues = new double[DELTA_ELEMENTS];

      for (int i = 0; i < DELTA_ELEMENTS; i++)
         deltaValues[i] = toDouble(delta.get(i));

      result.getClusterToBase().set(deltaValues);

      Map<String, Object> layouts = (Map<String, Object>) document.get("layouts");

      if (layouts == null)
         throw new IllegalArgumentException("'layouts' is absent.");

      Point3D position = new Point3D();

      for (Map.Entry<String, Object> linkEntry : layouts.entrySet())
      {
         Map<String, Object> markerEntries = (Map<String, Object>) linkEntry.getValue();
         List<MarkerId> members = new ArrayList<>(markerEntries.size());

         for (String markerName : markerEntries.keySet())
         {
            MarkerId marker = byName.get(markerName);

            if (marker == null)
            {
               if (strictMarkerSet)
                  throw new IllegalArgumentException("Calibration file names marker '" + markerName
                        + "', which is not in the marker set given. The file and the session disagree about what is mounted.");

               marker = new MarkerId(markerName, byName.size());
               byName.put(markerName, marker);
            }

            members.add(marker);
         }

         ClusterLayout layout = new ClusterLayout(linkEntry.getKey(), members);

         for (int i = 0; i < members.size(); i++)
         {
            Map<String, Object> markerEntry = (Map<String, Object>) markerEntries.get(members.get(i).getName());
            List<Object> p = (List<Object>) markerEntry.get("p");

            if (p == null || p.size() != 3)
               throw new IllegalArgumentException("Marker '" + members.get(i).getName() + "' on link '" + linkEntry.getKey()
                     + "' must have a 'p' array of 3 numbers.");

            position.set(toDouble(p.get(0)), toDouble(p.get(1)), toDouble(p.get(2)));
            layout.setPositionInLinkFrame(i, position, (int) optionalDouble(markerEntry, "k", 0.0));
         }

         result.addLayout(layout);
      }

      return result;
   }

   private static double requireDouble(Map<String, Object> object, String key)
   {
      Object value = object.get(key);

      if (value == null)
         throw new IllegalArgumentException("'" + key + "' is absent.");

      return toDouble(value);
   }

   private static double optionalDouble(Map<String, Object> object, String key, double defaultValue)
   {
      Object value = object.get(key);
      return value == null ? defaultValue : toDouble(value);
   }

   /** {@code null} decodes to NaN -- see the note on the format above. */
   private static double toDouble(Object value)
   {
      if (value == null)
         return Double.NaN;
      if (value instanceof Double doubleValue)
         return doubleValue;

      throw new IllegalArgumentException("Expected a number, got " + value.getClass().getSimpleName() + ": " + value);
   }

   // ------------------------------------------------------------------------------------------
   // A JSON parser, scoped to what this format needs
   // ------------------------------------------------------------------------------------------

   /**
    * Recursive-descent JSON parser producing {@code LinkedHashMap}, {@code ArrayList},
    * {@code String}, {@code Double}, {@code Boolean} and {@code null}.
    * <p>
    * Objects decode to {@code LinkedHashMap} specifically: marker order within a layout is the
    * order it was written, and preserving it keeps a round trip byte-identical and keeps file
    * diffs readable.
    * </p>
    * <p>
    * Deliberately not a general-purpose JSON library. It handles the whole of JSON's grammar
    * except that it decodes all numbers as {@code double}, which is every number this format has.
    * </p>
    */
   private static final class JsonParser
   {
      private final String text;
      private int position = 0;

      private JsonParser(String text)
      {
         this.text = text;
      }

      private Object parseValue()
      {
         skipWhitespace();

         if (position >= text.length())
            throw error("Unexpected end of input");

         char c = text.charAt(position);

         return switch (c)
         {
            case '{' -> parseObject();
            case '[' -> parseArray();
            case '"' -> parseString();
            case 't' -> parseLiteral("true", Boolean.TRUE);
            case 'f' -> parseLiteral("false", Boolean.FALSE);
            case 'n' -> parseLiteral("null", null);
            default -> parseNumber();
         };
      }

      private Map<String, Object> parseObject()
      {
         Map<String, Object> object = new LinkedHashMap<>();
         expect('{');
         skipWhitespace();

         if (peek() == '}')
         {
            position++;
            return object;
         }

         while (true)
         {
            skipWhitespace();
            String key = parseString();
            skipWhitespace();
            expect(':');
            Object value = parseValue();

            if (object.put(key, value) != null)
               throw error("Duplicate key '" + key + "'");

            skipWhitespace();
            char c = next();

            if (c == '}')
               return object;
            if (c != ',')
               throw error("Expected ',' or '}' but found '" + c + "'");
         }
      }

      private List<Object> parseArray()
      {
         List<Object> array = new ArrayList<>();
         expect('[');
         skipWhitespace();

         if (peek() == ']')
         {
            position++;
            return array;
         }

         while (true)
         {
            array.add(parseValue());
            skipWhitespace();
            char c = next();

            if (c == ']')
               return array;
            if (c != ',')
               throw error("Expected ',' or ']' but found '" + c + "'");
         }
      }

      private String parseString()
      {
         expect('"');
         StringBuilder value = new StringBuilder();

         while (true)
         {
            char c = next();

            if (c == '"')
               return value.toString();

            if (c != '\\')
            {
               value.append(c);
               continue;
            }

            char escape = next();

            switch (escape)
            {
               case '"' -> value.append('"');
               case '\\' -> value.append('\\');
               case '/' -> value.append('/');
               case 'b' -> value.append('\b');
               case 'f' -> value.append('\f');
               case 'n' -> value.append('\n');
               case 'r' -> value.append('\r');
               case 't' -> value.append('\t');
               case 'u' ->
               {
                  if (position + 4 > text.length())
                     throw error("Truncated \\u escape");

                  value.append((char) Integer.parseInt(text.substring(position, position + 4), 16));
                  position += 4;
               }
               default -> throw error("Unknown escape '\\" + escape + "'");
            }
         }
      }

      private Double parseNumber()
      {
         int start = position;

         if (peek() == '-' || peek() == '+')
            position++;

         while (position < text.length())
         {
            char c = text.charAt(position);

            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-')
               position++;
            else
               break;
         }

         if (start == position)
            throw error("Expected a number");

         try
         {
            return Double.valueOf(text.substring(start, position));
         }
         catch (NumberFormatException e)
         {
            throw error("Malformed number '" + text.substring(start, position) + "'");
         }
      }

      private Object parseLiteral(String literal, Object value)
      {
         if (!text.startsWith(literal, position))
            throw error("Expected '" + literal + "'");

         position += literal.length();
         return value;
      }

      private void skipWhitespace()
      {
         while (position < text.length() && Character.isWhitespace(text.charAt(position)))
            position++;
      }

      private char peek()
      {
         if (position >= text.length())
            throw error("Unexpected end of input");

         return text.charAt(position);
      }

      private char next()
      {
         if (position >= text.length())
            throw error("Unexpected end of input");

         return text.charAt(position++);
      }

      private void expect(char expected)
      {
         char c = next();

         if (c != expected)
            throw error("Expected '" + expected + "' but found '" + c + "'");
      }

      private IllegalArgumentException error(String message)
      {
         int line = 1;

         for (int i = 0; i < Math.min(position, text.length()); i++)
         {
            if (text.charAt(i) == '\n')
               line++;
         }

         return new IllegalArgumentException(message + " at line " + line + " (offset " + position + ").");
      }
   }
}
