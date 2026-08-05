package us.ihmc.alexMocap.mocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.StringReader;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;

/**
 * {@link MocapFrameRecorder} and {@link CsvReplayMocapSource} are one component with a file in the
 * middle. Together they are the entire test harness for G1 and G2 -- capture once at the gantry,
 * re-run every gate in CI off the file -- so the round trip has to be exact, not approximate.
 */
public class CsvRoundTripTest
{
   private static final List<MarkerId> MARKER_SET = MarkerId.createDenseSet("PELVIS_1", "PELVIS_2", "PELVIS_3", "PELVIS_4", "L_THIGH_1", "L_THIGH_2");

   /**
    * The assertion PR1 names: write a list of frames, read it back, and get exactly what went in
    * including the visibility flags. Positions are compared with zero tolerance -- doubles are
    * written with {@code Double.toString}, which round-trips bit-identical, and a log that drifts
    * in the last bit is a log you cannot reproduce a result from.
    */
   @Test
   public void testRoundTripIsExactIncludingVisibility(@TempDir Path directory) throws IOException
   {
      Random random = new Random(90210L);
      List<MocapFrame> written = new ArrayList<>();
      Path file = directory.resolve("capture.csv");

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(file, MARKER_SET))
      {
         for (int f = 0; f < 200; f++)
         {
            MocapFrame frame = new MocapFrame(MARKER_SET);
            frame.setTimestampNanoseconds(1_000_000_000L + f * 5_000_000L);

            for (int m = 0; m < MARKER_SET.size(); m++)
            {
               // ~20% occlusion, so visibility patterns vary frame to frame rather than being
               // constant -- a reader that ignored the flags would still pass a fully-visible log.
               if (random.nextDouble() > 0.2)
                  frame.get(m).setVisible(random.nextDouble(), random.nextDouble(), random.nextDouble());
            }

            recorder.write(frame);
            written.add(frame);
         }

         assertEquals(200, recorder.getFramesWritten());
      }

      List<MocapFrame> read;

      try (CsvReplayMocapSource source = CsvReplayMocapSource.open(file, MARKER_SET))
      {
         read = source.readAll();
         assertTrue(source.isFinished());
      }

      assertEquals(written.size(), read.size());

      int occlusions = 0;

      for (int f = 0; f < written.size(); f++)
      {
         MocapFrame expected = written.get(f);
         MocapFrame actual = read.get(f);

         assertEquals(expected.getTimestampNanoseconds(), actual.getTimestampNanoseconds(), "Frame " + f + " timestamp");
         assertEquals(expected.getVisibleCount(), actual.getVisibleCount(), "Frame " + f + " visible count");

         for (int m = 0; m < MARKER_SET.size(); m++)
         {
            assertEquals(expected.get(m).isVisible(), actual.get(m).isVisible(), "Frame " + f + " marker " + m + " visibility");

            if (expected.get(m).isVisible())
            {
               assertEquals(expected.get(m).getPosition().getX(), actual.get(m).getPosition().getX(), 0.0);
               assertEquals(expected.get(m).getPosition().getY(), actual.get(m).getPosition().getY(), 0.0);
               assertEquals(expected.get(m).getPosition().getZ(), actual.get(m).getPosition().getZ(), 0.0);
            }
            else
            {
               assertTrue(actual.get(m).getPosition().containsNaN(), "An invisible marker must read back as NaN.");
               occlusions++;
            }
         }
      }

      // Guard against the test quietly becoming a fully-visible log, which would assert nothing
      // about occlusion. Expected is 0.2 * 200 * 6 = 240.
      assertTrue(occlusions > 100, "Only " + occlusions + " occlusions in the log; the visibility path is barely exercised.");
   }

   /** A log is self-describing: the marker set comes back off the header row. */
   @Test
   public void testHeaderCarriesTheMarkerSet() throws IOException
   {
      String csv = write(frame(0, true, true, true, true, true, true));

      assertTrue(csv.contains("timestamp_ns,PELVIS_1_x,PELVIS_1_y,PELVIS_1_z,PELVIS_2_x"), "Header:\n" + csv);

      try (CsvReplayMocapSource source = new CsvReplayMocapSource(new StringReader(csv), null))
      {
         assertEquals(MARKER_SET.size(), source.getMarkers().size());

         for (int i = 0; i < MARKER_SET.size(); i++)
         {
            assertEquals(MARKER_SET.get(i).getName(), source.getMarkers().get(i).getName());
            assertEquals(i, source.getMarkers().get(i).getIndex());
         }
      }
   }

   /**
    * A log whose markers do not match the session's is refused. Rebinding by column position would
    * assign measurements to the wrong markers and produce poses that are wrong and look healthy.
    */
   @Test
   public void testMarkerSetMismatchIsRejected() throws IOException
   {
      String csv = write(frame(0, true, true, true, true, true, true));

      List<MarkerId> renamed = MarkerId.createDenseSet("PELVIS_1", "PELVIS_2", "PELVIS_3", "PELVIS_9", "L_THIGH_1", "L_THIGH_2");
      IOException thrown = assertThrows(IOException.class, () -> new CsvReplayMocapSource(new StringReader(csv), renamed));
      assertTrue(thrown.getMessage().contains("PELVIS_4"), thrown.getMessage());

      List<MarkerId> shorter = MarkerId.createDenseSet("PELVIS_1", "PELVIS_2", "PELVIS_3");
      assertThrows(IOException.class, () -> new CsvReplayMocapSource(new StringReader(csv), shorter));

      // Same names, different order: also a mismatch, and the least obvious one.
      List<MarkerId> reordered = MarkerId.createDenseSet("PELVIS_2", "PELVIS_1", "PELVIS_3", "PELVIS_4", "L_THIGH_1", "L_THIGH_2");
      assertThrows(IOException.class, () -> new CsvReplayMocapSource(new StringReader(csv), reordered));
   }

   /**
    * A partially NaN triple is a corrupt line, not a half-seen marker. Silently treating it as
    * occluded would hide file damage; treating it as visible would feed NaN into a registration.
    */
   @Test
   public void testPartiallyNaNPositionIsRejected() throws IOException
   {
      String csv = write(frame(0, true, true, true, true, true, true));
      String corrupted = csv.replaceFirst("(\\n\\d+,)([^,]+)", "$1NaN");

      try (CsvReplayMocapSource source = new CsvReplayMocapSource(new StringReader(corrupted), MARKER_SET))
      {
         MocapFrame frame = source.createFrame();
         IllegalStateException thrown = assertThrows(IllegalStateException.class, () -> source.read(frame));
         assertTrue(thrown.getMessage().contains("partially NaN"), thrown.getMessage());
      }
   }

   @Test
   public void testTruncatedLineIsRejected() throws IOException
   {
      String csv = write(frame(0, true, true, true, true, true, true));
      String truncated = csv.substring(0, csv.length() - 12) + "\n";

      try (CsvReplayMocapSource source = new CsvReplayMocapSource(new StringReader(truncated), MARKER_SET))
      {
         MocapFrame frame = source.createFrame();
         IllegalStateException thrown = assertThrows(IllegalStateException.class, () -> source.read(frame));
         assertTrue(thrown.getMessage().contains("fields"), thrown.getMessage());
      }
   }

   /**
    * A frame with no timestamp cannot be paired with encoders later, and an unpaired capture is
    * FRAMEWORK.md §18.3 waiting to happen. Refuse it at the point it would enter the log.
    */
   @Test
   public void testUntimestampedFrameIsRefused() throws IOException
   {
      StringWriter out = new StringWriter();

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(out, MARKER_SET))
      {
         MocapFrame frame = new MocapFrame(MARKER_SET);
         frame.get(0).setVisible(1.0, 2.0, 3.0);

         assertEquals(MocapFrame.NO_TIMESTAMP, frame.getTimestampNanoseconds());
         assertThrows(IllegalArgumentException.class, () -> recorder.write(frame));
      }
   }

   /** Comment lines and blank lines are skipped, so a log can be annotated by hand. */
   @Test
   public void testCommentsAndBlankLinesAreSkipped() throws IOException
   {
      String csv = write(frame(0, true, true, true, true, true, true), frame(1, false, true, true, true, true, true));
      String annotated = csv.replaceFirst("(?m)^(\\d+,)", "# leg sweep starts here\n\n$1");

      try (CsvReplayMocapSource source = new CsvReplayMocapSource(new StringReader(annotated), MARKER_SET))
      {
         assertEquals(2, source.readAll().size());
      }
   }

   /**
    * The loop shape the interface documents. {@code read} returning false must not be read as end
    * of stream -- for a replay the two coincide, which is exactly why the distinction has to be
    * tested somewhere that a live source's behaviour can be reasoned about against it.
    */
   @Test
   public void testFinishedOnlyAfterTheLastFrame() throws IOException
   {
      String csv = write(frame(0, true, true, true, true, true, true), frame(1, true, true, true, true, true, true));

      try (CsvReplayMocapSource source = new CsvReplayMocapSource(new StringReader(csv), MARKER_SET))
      {
         MocapFrame frame = source.createFrame();

         assertFalse(source.isFinished());
         assertTrue(source.read(frame));
         assertFalse(source.isFinished(), "Two frames in, one read: the stream is not finished.");
         assertTrue(source.read(frame));
         assertFalse(source.read(frame));
         assertTrue(source.isFinished());
         assertEquals(2, source.getFramesRead());
      }
   }

   @Test
   public void testEmptyLogReadsAsZeroFrames() throws IOException
   {
      String csv = write();

      try (CsvReplayMocapSource source = new CsvReplayMocapSource(new StringReader(csv), MARKER_SET))
      {
         assertEquals(0, source.readAll().size());
         assertTrue(source.isFinished());
      }
   }

   @Test
   public void testHeaderlessInputIsRejected()
   {
      assertThrows(IOException.class, () -> new CsvReplayMocapSource(new StringReader("# only a comment\n"), MARKER_SET));
      assertThrows(IOException.class, () -> new CsvReplayMocapSource(new StringReader("nope,a_x,a_y,a_z\n"), MARKER_SET));
      assertThrows(IOException.class, () -> new CsvReplayMocapSource(new StringReader("timestamp_ns,A_x,A_y,B_z\n"), null));
   }

   /** A round trip through a real file, not just a string, including the temp-dir path. */
   @Test
   public void testFileRoundTripPreservesByteContent(@TempDir Path directory) throws IOException
   {
      Path first = directory.resolve("a.csv");
      Path second = directory.resolve("b.csv");

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(first, MARKER_SET))
      {
         recorder.write(frame(0, true, false, true, true, false, true));
         recorder.write(frame(1, true, true, true, true, true, true));
      }

      List<MocapFrame> frames;

      try (CsvReplayMocapSource source = CsvReplayMocapSource.open(first, MARKER_SET))
      {
         frames = source.readAll();
      }

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(second, MARKER_SET))
      {
         for (MocapFrame frame : frames)
            recorder.write(frame);
      }

      assertEquals(Files.readString(first), Files.readString(second), "Rewriting what was read must reproduce the file exactly.");
   }

   private static String write(MocapFrame... frames) throws IOException
   {
      StringWriter out = new StringWriter();

      try (MocapFrameRecorder recorder = new MocapFrameRecorder(out, MARKER_SET))
      {
         for (MocapFrame frame : frames)
            recorder.write(frame);
      }

      return out.toString();
   }

   private static MocapFrame frame(int index, boolean... visible)
   {
      MocapFrame frame = new MocapFrame(MARKER_SET);
      frame.setTimestampNanoseconds(1_000_000_000L + index * 5_000_000L);

      for (int m = 0; m < visible.length; m++)
      {
         if (visible[m])
            frame.get(m).setVisible(0.01 * m + 0.001 * index, -0.02 * m, 0.9 + 0.003 * m);
      }

      return frame;
   }
}
