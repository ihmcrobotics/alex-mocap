package us.ihmc.alexMocap.mocap;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.io.StringReader;
import java.util.List;

import us.ihmc.alexMocap.core.MarkerId;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class MarkerLabelingIOTest
{
   private static final String[] NAMES = {"PELVIS_LINK_M0", "PELVIS_LINK_M1", "PELVIS_LINK_M2", "PELVIS_LINK_M3", "LEFT_THIGH_M0", "LEFT_THIGH_M1",
                                          "LEFT_THIGH_M2"};

   /**
    * Motive's packed ids: large, sparse and non-contiguous. The whole point of the format is to carry
    * these unchanged, so the round trip is tested against them rather than against 0..6.
    */
   private static final int[] MOTIVE_IDS = {65536, 65537, 65538, 65539, 131072, 131073, 131074};

   @Test
   public void testRoundTripPreservesIdsNamesAndIndices() throws IOException
   {
      MarkerLabeling original = build();

      String text = MarkerLabelingIO.toText(original);
      MarkerLabeling reread = MarkerLabelingIO.read(new StringReader(text));

      assertEquals(original.getLabelledCount(), reread.getLabelledCount());
      assertEquals(original.getMarkers().size(), reread.getMarkers().size());

      for (int i = 0; i < NAMES.length; i++)
      {
         MarkerId marker = reread.lookup(MOTIVE_IDS[i]);

         assertNotNull(marker, "Motive id " + MOTIVE_IDS[i] + " lost its marker across the round trip.");
         assertEquals(NAMES[i], marker.getName());

         // The dense index matters as much as the name: it is what indexes every observation array
         // downstream, so a round trip that preserved names but permuted indices would relabel the
         // entire capture without changing anything a human would notice in the file.
         assertEquals(i, marker.getIndex(), "The dense index for " + NAMES[i] + " moved.");
      }
   }

   @Test
   public void testWritingIsStable() throws IOException
   {
      String once = MarkerLabelingIO.toText(build());
      String twice = MarkerLabelingIO.toText(MarkerLabelingIO.read(new StringReader(once)));

      // Byte-identical, so the file can live in git and a re-export produces an empty diff rather
      // than a reordering that has to be read to be dismissed.
      assertEquals(once, twice);
   }

   @Test
   public void testCommentsAndBlankLinesAreIgnored() throws IOException
   {
      String text = """
                    # alex-mocap marker labelling, format 1
                    # motiveId,markerName

                    65536,PELVIS_LINK_M0

                    # a note someone left mid-file
                       65537,PELVIS_LINK_M1
                    65538,PELVIS_LINK_M2
                    """;

      MarkerLabeling labeling = MarkerLabelingIO.read(new StringReader(text));

      assertEquals(3, labeling.getLabelledCount());
      assertEquals("PELVIS_LINK_M1", labeling.lookup(65537).getName());
   }

   @Test
   public void testReadAgainstAnExistingMarkerSetKeepsThatSetsIndices() throws IOException
   {
      // Deliberately the reverse of the file's order, so that "the file won" would be visible.
      List<MarkerId> markerSet = MarkerId.createDenseSet(NAMES[2], NAMES[1], NAMES[0]);

      String text = """
                    65536,PELVIS_LINK_M0
                    65537,PELVIS_LINK_M1
                    65538,PELVIS_LINK_M2
                    """;

      MarkerLabeling labeling = MarkerLabelingIO.readAgainst(new StringReader(text), markerSet);

      assertEquals(0, labeling.lookup(65538).getIndex(), "readAgainst must not renumber the set it was given.");
      assertEquals(2, labeling.lookup(65536).getIndex());
   }

   @Test
   public void testDuplicateMotiveIdIsRejectedWithItsLineNumber()
   {
      String text = """
                    65536,PELVIS_LINK_M0
                    65536,PELVIS_LINK_M1
                    """;

      IllegalArgumentException exception = assertThrows(IllegalArgumentException.class, () -> MarkerLabelingIO.read(new StringReader(text)));

      assertTrue(exception.getMessage().contains(":2:"), "The message should name the offending line, was: " + exception.getMessage());
   }

   @Test
   public void testDuplicateMarkerNameIsRejected()
   {
      // Two Motive ids feeding one marker: physically impossible, and it would make the marker's
      // position depend on which id happened to arrive last in the frame.
      String text = """
                    65536,PELVIS_LINK_M0
                    65537,PELVIS_LINK_M0
                    """;

      assertThrows(RuntimeException.class, () -> MarkerLabelingIO.read(new StringReader(text)));
   }

   @Test
   public void testMarkerNameWithoutAnUnderscoreIsRejectedHereRatherThanAtClusterInference()
   {
      IOException exception = assertThrows(IOException.class, () -> MarkerLabelingIO.read(new StringReader("65536,PELVIS\n")));

      assertTrue(exception.getMessage().contains("LAST underscore"),
                 "The message should explain the naming rule, since that is the actual fix. Was: " + exception.getMessage());
   }

   @Test
   public void testMalformedRowsAreRejected()
   {
      assertThrows(IOException.class, () -> MarkerLabelingIO.read(new StringReader("65536 PELVIS_LINK_M0\n")), "no comma");
      assertThrows(IOException.class, () -> MarkerLabelingIO.read(new StringReader("notAnInteger,PELVIS_LINK_M0\n")), "id is not an integer");
      assertThrows(IOException.class, () -> MarkerLabelingIO.read(new StringReader("65536,\n")), "empty marker name");
      assertThrows(IOException.class, () -> MarkerLabelingIO.read(new StringReader("65536,PELVIS_LINK_M0,extra\n")), "three columns");
   }

   @Test
   public void testAnEmptyFileIsRejected()
   {
      // An empty labelling is accepted by every downstream check and labels nothing, so every cluster
      // silently starves. Refusing here is the only place it is still obvious what went wrong.
      assertThrows(IOException.class, () -> MarkerLabelingIO.read(new StringReader("# nothing but a comment\n")));
   }

   @Test
   public void testUnfedMarkersSurviveAsAComment() throws IOException
   {
      List<MarkerId> markerSet = MarkerId.createDenseSet(NAMES[0], NAMES[1], NAMES[2], "LEFT_SHIN_M0");

      MarkerLabeling labeling = MarkerLabeling.against(markerSet)
                                              .add(65536, NAMES[0])
                                              .add(65537, NAMES[1])
                                              .add(65538, NAMES[2])
                                              .build();

      String text = MarkerLabelingIO.toText(labeling);

      assertEquals(1, labeling.getUnfedMarkers().size());
      assertTrue(text.contains("LEFT_SHIN_M0"),
                 "A marker in the set that nothing feeds must not vanish from the file -- it is the evidence for a cluster that will never reach three"
                 + " visible markers. Was:\n" + text);
   }

   private static MarkerLabeling build()
   {
      MarkerLabeling.Builder builder = MarkerLabeling.against(MarkerId.createDenseSet(NAMES));

      for (int i = 0; i < NAMES.length; i++)
         builder.add(MOTIVE_IDS[i], NAMES[i]);

      return builder.build();
   }
}
