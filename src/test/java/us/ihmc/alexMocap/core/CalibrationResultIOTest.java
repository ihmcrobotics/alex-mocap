package us.ihmc.alexMocap.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.StringReader;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class CalibrationResultIOTest
{
   private static final List<MarkerId> MARKER_SET = MarkerId.createDenseSet("PELVIS_1",
                                                                           "PELVIS_2",
                                                                           "PELVIS_3",
                                                                           "PELVIS_4",
                                                                           "L_THIGH_1",
                                                                           "L_THIGH_2",
                                                                           "L_THIGH_3",
                                                                           "L_THIGH_4");

   /**
    * The round trip that has to hold exactly. Doubles are written with {@code Double.toString},
    * which is the shortest representation that reads back bit-identical, so this asserts equality
    * rather than a tolerance -- a calibration that drifts in the last bit every time it is saved
    * and loaded is a calibration nobody can reproduce.
    */
   @Test
   public void testRoundTripIsExact(@TempDir Path directory) throws IOException
   {
      Random random = new Random(6180L);
      CalibrationResult original = randomResult(random);
      Path file = directory.resolve("calibration.json");

      CalibrationResultIO.write(file, original);
      CalibrationResult read = CalibrationResultIO.read(file, MARKER_SET);

      assertResultsEqual(original, read);

      // And writing what was read reproduces the file byte for byte. Without this, a round trip
      // could be "equal enough" while quietly reordering markers or losing precision.
      StringWriter rewritten = new StringWriter();
      CalibrationResultIO.write(rewritten, read);
      assertEquals(Files.readString(file), rewritten.toString());
   }

   /**
    * Δ is the quantity most easily corrupted in serialisation, because Euclid's
    * {@code get(double[])} packs a 4×4 while {@code set(double[])} reads a 3×4. Round-tripping a
    * thousand random transforms is what catches an off-by-four in that layout.
    */
   @Test
   public void testDeltaRoundTripsExactlyOverManyTransforms() throws IOException
   {
      Random random = new Random(1729L);

      for (int trial = 0; trial < 1000; trial++)
      {
         CalibrationResult original = new CalibrationResult();
         RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         original.getClusterToBase().set(planted);
         original.addLayout(layout("pelvis", 0, 4, random));

         StringWriter writer = new StringWriter();
         CalibrationResultIO.write(writer, original);
         CalibrationResult read = CalibrationResultIO.read(new StringReader(writer.toString()), MARKER_SET);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertEquals(planted.getRotation().getElement(row, column), read.getClusterToBase().getRotation().getElement(row, column), 0.0);

            assertEquals(planted.getTranslation().getElement(row), read.getClusterToBase().getTranslation().getElement(row), 0.0);
         }
      }
   }

   /**
    * A marker that was never seen has a NaN position and {@code K_ij = 0}. JSON has no NaN
    * literal, so it is written as {@code null} -- and that state must survive, because writing 0.0
    * instead would turn "never seen" into "seen at the link origin", which is a plausible position
    * that registers without complaint.
    */
   @Test
   public void testNeverObservedMarkerSurvivesAsNaN() throws IOException
   {
      CalibrationResult original = new CalibrationResult();
      original.getClusterToBase().setIdentity();

      ClusterLayout layout = new ClusterLayout("pelvis", MARKER_SET.subList(0, 4));
      layout.setPositionInLinkFrame(0, new Point3D(0.06, -0.03, 0.09), 30);
      layout.setPositionInLinkFrame(1, new Point3D(-0.04, 0.05, 0.09), 28);
      layout.setPositionInLinkFrame(2, new Point3D(-0.05, -0.05, 0.09), 30);
      layout.setNotObserved(3);
      original.addLayout(layout);

      assertFalse(original.isFullySolved());
      assertEquals(0, layout.getMinimumObservationCount());

      StringWriter writer = new StringWriter();
      CalibrationResultIO.write(writer, original);
      assertTrue(writer.toString().contains("[null, null, null]"), "A never-observed marker must be written as nulls:\n" + writer);

      CalibrationResult read = CalibrationResultIO.read(new StringReader(writer.toString()), MARKER_SET);
      ClusterLayout readLayout = read.getLayout("pelvis");

      assertTrue(readLayout.getPositionInLinkFrame(3).containsNaN());
      assertEquals(0, readLayout.getObservationCount(3));
      assertFalse(read.isFullySolved());
      assertEquals(30, readLayout.getObservationCount(0));
   }

   /**
    * A file naming a marker the session does not have means the file and the robot disagree about
    * what is mounted. That has to fail loudly: silently inventing the marker would bind a
    * calibrated position to an index nothing else in the session uses.
    */
   @Test
   public void testUnknownMarkerNameIsRejected() throws IOException
   {
      CalibrationResult original = new CalibrationResult();
      original.getClusterToBase().setIdentity();
      ClusterLayout layout = new ClusterLayout("pelvis", MARKER_SET.subList(0, 4));

      for (int i = 0; i < 4; i++)
         layout.setPositionInLinkFrame(i, new Point3D(0.01 * i, 0.02, 0.03), 30);

      original.addLayout(layout);

      StringWriter writer = new StringWriter();
      CalibrationResultIO.write(writer, original);
      String json = writer.toString();

      List<MarkerId> differentSet = MarkerId.createDenseSet("PELVIS_1", "PELVIS_2", "PELVIS_3", "PELVIS_9");

      IllegalArgumentException thrown = assertThrows(IllegalArgumentException.class,
                                                     () -> CalibrationResultIO.read(new StringReader(json), differentSet));
      assertTrue(thrown.getMessage().contains("PELVIS_4"), "The message should name the marker that could not be resolved: " + thrown.getMessage());

      // The lenient reader is for inspection tools and does not throw.
      Path temporary = Files.createTempFile("calibration", ".json");

      try
      {
         Files.writeString(temporary, json);
         CalibrationResult lenient = CalibrationResultIO.readWithDenseMarkerSet(temporary);
         assertEquals(4, lenient.getLayout("pelvis").getMarkerCount());
      }
      finally
      {
         Files.deleteIfExists(temporary);
      }
   }

   @Test
   public void testFormatVersionMismatchIsRejected()
   {
      String json = """
            {
              "formatVersion": 99,
              "delta": [1,0,0,0, 0,1,0,0, 0,0,1,0],
              "layouts": {}
            }
            """;

      IllegalArgumentException thrown = assertThrows(IllegalArgumentException.class,
                                                     () -> CalibrationResultIO.read(new StringReader(json), MARKER_SET));
      assertTrue(thrown.getMessage().contains("99"));
   }

   @Test
   public void testMalformedJsonReportsALine()
   {
      String json = """
            {
              "formatVersion": 1,
              "delta": [1,0,0,0, 0,1,0,0, 0,0,1,0]
              "layouts": {}
            }
            """;

      IllegalArgumentException thrown = assertThrows(IllegalArgumentException.class,
                                                     () -> CalibrationResultIO.read(new StringReader(json), MARKER_SET));
      assertTrue(thrown.getMessage().contains("line 4"), "Parse errors should point at a line: " + thrown.getMessage());
   }

   @Test
   public void testWrongDeltaLengthIsRejected()
   {
      String json = """
            {
              "formatVersion": 1,
              "delta": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
              "layouts": {}
            }
            """;

      IllegalArgumentException thrown = assertThrows(IllegalArgumentException.class,
                                                     () -> CalibrationResultIO.read(new StringReader(json), MARKER_SET));
      assertTrue(thrown.getMessage().contains("16"), "A 4x4 delta should be rejected by length: " + thrown.getMessage());
   }

   /** Provenance is the part a human reads a year later; it must not be lossy. */
   @Test
   public void testProvenanceRoundTrips() throws IOException
   {
      CalibrationResult original = new CalibrationResult();
      original.getClusterToBase().setIdentity();
      original.addLayout(layout("pelvis", 0, 4, new Random(1L)));
      original.setProvenance(new CalibrationResult.Provenance("alex_v3.urdf",
                                                              "9f2c1ab4",
                                                              30,
                                                              7,
                                                              3.1e-7,
                                                              Math.toRadians(0.08),
                                                              "2026-08-05T14:02:11Z",
                                                              "gantry, legs swept \"wide\"\nsecond attempt"));

      StringWriter writer = new StringWriter();
      CalibrationResultIO.write(writer, original);
      CalibrationResult read = CalibrationResultIO.read(new StringReader(writer.toString()), MARKER_SET);

      assertEquals(original.getProvenance(), read.getProvenance());
      assertTrue(read.getProvenance().hasMeasuredWorldTilt());
      // The note carries a quote and a newline; both must survive escaping.
      assertTrue(read.getProvenance().note().contains("\"wide\""));
      assertTrue(read.getProvenance().note().contains("\n"));
   }

   /**
    * An unmeasured world tilt reads back as NaN, not 0.0. FRAMEWORK.md §11 is explicit that θ must
    * be measured and never assumed, and 0.0 is an assumption wearing a measurement's clothes.
    */
   @Test
   public void testUnmeasuredWorldTiltStaysUnmeasured() throws IOException
   {
      CalibrationResult original = new CalibrationResult();
      original.getClusterToBase().setIdentity();
      original.addLayout(layout("pelvis", 0, 4, new Random(2L)));
      original.setProvenance(CalibrationResult.Provenance.unknown());

      assertFalse(original.getProvenance().hasMeasuredWorldTilt());

      StringWriter writer = new StringWriter();
      CalibrationResultIO.write(writer, original);
      CalibrationResult read = CalibrationResultIO.read(new StringReader(writer.toString()), MARKER_SET);

      assertFalse(read.getProvenance().hasMeasuredWorldTilt());
      assertTrue(Double.isNaN(read.getProvenance().worldTiltRadians()));
      assertNull(read.getProvenance().urdfSha256());
   }

   private static CalibrationResult randomResult(Random random)
   {
      CalibrationResult result = new CalibrationResult();
      result.getClusterToBase().set(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      result.addLayout(layout("pelvis", 0, 4, random));
      result.addLayout(layout("l_thigh", 4, 4, random));
      result.setProvenance(new CalibrationResult.Provenance("toy.urdf", "abc123", 30, 9, 4.2e-8, Math.toRadians(0.05), "2026-08-05T00:00:00Z", "seeded"));
      return result;
   }

   private static ClusterLayout layout(String linkName, int firstMarker, int markerCount, Random random)
   {
      ClusterLayout layout = new ClusterLayout(linkName, MARKER_SET.subList(firstMarker, firstMarker + markerCount));

      for (int i = 0; i < markerCount; i++)
         layout.setPositionInLinkFrame(i, EuclidCoreRandomTools.nextPoint3D(random, 0.15), 20 + random.nextInt(11));

      return layout;
   }

   private static void assertResultsEqual(CalibrationResult expected, CalibrationResult actual)
   {
      assertEquals(expected.getProvenance(), actual.getProvenance());
      assertEquals(expected.getLinkNames(), actual.getLinkNames());

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
            assertEquals(expected.getClusterToBase().getRotation().getElement(row, column),
                         actual.getClusterToBase().getRotation().getElement(row, column),
                         0.0);

         assertEquals(expected.getClusterToBase().getTranslation().getElement(row), actual.getClusterToBase().getTranslation().getElement(row), 0.0);
      }

      for (String linkName : expected.getLinkNames())
      {
         ClusterLayout expectedLayout = expected.getLayout(linkName);
         ClusterLayout actualLayout = actual.getLayout(linkName);
         assertEquals(expectedLayout.getMarkers(), actualLayout.getMarkers());

         for (int i = 0; i < expectedLayout.getMarkerCount(); i++)
         {
            assertEquals(expectedLayout.getPositionInLinkFrame(i).getX(), actualLayout.getPositionInLinkFrame(i).getX(), 0.0);
            assertEquals(expectedLayout.getPositionInLinkFrame(i).getY(), actualLayout.getPositionInLinkFrame(i).getY(), 0.0);
            assertEquals(expectedLayout.getPositionInLinkFrame(i).getZ(), actualLayout.getPositionInLinkFrame(i).getZ(), 0.0);
            assertEquals(expectedLayout.getObservationCount(i), actualLayout.getObservationCount(i));
         }
      }
   }
}
