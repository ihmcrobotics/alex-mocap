package us.ihmc.alexMocap.mocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.AllocationMeasurement;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;

/** {@link MarkerLabeling} and the live-source frame handoff in {@link NatNetMocapSource}. */
public class MocapSourceTest
{
   private static final List<MarkerId> MARKER_SET = MarkerId.createDenseSet("PELVIS_1", "PELVIS_2", "PELVIS_3", "PELVIS_4");

   /**
    * Motive's ids are neither small nor contiguous -- for labelled markers they are typically a
    * composite of rigid-body id and marker index. The lookup has to cope with that without a hash
    * map, so it is a binary search over sorted ids.
    */
   @Test
   public void testLabelingHandlesSparseMotiveIds()
   {
      MarkerLabeling labeling = MarkerLabeling.against(MARKER_SET)
                                              .add(0x00010001, "PELVIS_1")
                                              .add(0x00010002, "PELVIS_2")
                                              .add(0x00010003, "PELVIS_3")
                                              .add(0x007F0001, "PELVIS_4")
                                              .build();

      assertEquals(4, labeling.getLabelledCount());
      assertEquals(0, labeling.indexOf(0x00010001));
      assertEquals(3, labeling.indexOf(0x007F0001));
      assertSame(MARKER_SET.get(2), labeling.lookup(0x00010003));
      assertEquals(0x007F0001, labeling.motiveIdOf(MARKER_SET.get(3)));

      // Unlabelled ids are normal: Motive streams point-cloud markers alongside labelled ones.
      assertEquals(-1, labeling.indexOf(0x00010004));
      assertNull(labeling.lookup(12345));
      assertFalse(labeling.isLabelled(0));
      assertTrue(labeling.getUnfedMarkers().isEmpty());
   }

   @Test
   public void testLabelingMustBeABijection()
   {
      MarkerLabeling.Builder builder = MarkerLabeling.against(MARKER_SET).add(10, "PELVIS_1");

      assertThrows(IllegalArgumentException.class, () -> builder.add(10, "PELVIS_2"), "One Motive id cannot feed two markers.");
      assertThrows(IllegalArgumentException.class, () -> builder.add(11, "PELVIS_1"), "One marker cannot be fed by two Motive ids.");
      assertThrows(IllegalArgumentException.class, () -> builder.add(12, "TORSO_1"), "A marker outside the set is not labellable.");

      // Each rejected add must have left the builder untouched. An earlier version validated after
      // writing, so the first rejection above silently replaced PELVIS_1 with PELVIS_2 on its way
      // out and the second add then looked perfectly legal.
      MarkerLabeling labeling = builder.add(11, "PELVIS_2").build();

      assertEquals(2, labeling.getLabelledCount());
      assertSame(MARKER_SET.get(0), labeling.lookup(10));
      assertSame(MARKER_SET.get(1), labeling.lookup(11));
   }

   /**
    * A marker no Motive id feeds can never become visible, so its cluster can never reach three
    * visible markers. Better to see the list at startup than to debug a cluster that silently
    * never registers.
    */
   @Test
   public void testUnfedMarkersAreReported()
   {
      MarkerLabeling labeling = MarkerLabeling.against(MARKER_SET).add(1, "PELVIS_1").add(2, "PELVIS_3").build();

      assertEquals(List.of(MARKER_SET.get(1), MARKER_SET.get(3)), labeling.getUnfedMarkers());
   }

   @Test
   public void testLabelingFromNamesBuildsItsOwnSet()
   {
      Map<Integer, String> mapping = new LinkedHashMap<>();
      mapping.put(0x1001, "PELVIS_1");
      mapping.put(0x1002, "PELVIS_2");
      mapping.put(0x1003, "PELVIS_3");

      MarkerLabeling labeling = MarkerLabeling.fromNames(mapping);

      assertEquals(3, labeling.getMarkers().size());
      assertEquals(0, labeling.lookup(0x1001).getIndex());
      assertEquals(2, labeling.lookup(0x1003).getIndex());
   }

   /**
    * Markers the set knows about but this frame did not report must come back explicitly not
    * visible, never carried over. A stale position registers without complaint and drags the
    * recovered pose toward where the marker used to be.
    */
   @Test
   public void testUnreportedMarkersDoNotPersistBetweenFrames()
   {
      NatNetMocapSource source = new NatNetMocapSource(labeling());
      MocapFrame frame = source.createFrame();

      source.onFrameReceived(1000L, new int[] {1, 2, 3, 4}, new double[] {0.1, 0, 0, 0.2, 0, 0, 0.3, 0, 0, 0.4, 0, 0}, 4);
      assertTrue(source.read(frame));
      assertEquals(4, frame.getVisibleCount());
      assertEquals(0.3, frame.get(2).getPosition().getX(), 0.0);

      // Second frame: marker 3 has dropped out of Motive's report entirely.
      source.onFrameReceived(2000L, new int[] {1, 2, 4}, new double[] {0.1, 0, 0, 0.2, 0, 0, 0.4, 0, 0}, 3);
      assertTrue(source.read(frame));

      assertEquals(3, frame.getVisibleCount());
      assertFalse(frame.get(2).isVisible());
      assertTrue(frame.get(2).getPosition().containsNaN(), "The dropped marker must be NaN, not its previous position.");
      assertEquals(2000L, frame.getTimestampNanoseconds());
   }

   /**
    * Latest-frame semantics: overwriting is correct for a control loop and is a hole in a capture.
    * The count is what tells the two apart after the fact.
    */
   @Test
   public void testDroppedFramesAreCounted()
   {
      NatNetMocapSource source = new NatNetMocapSource(labeling());
      MocapFrame frame = source.createFrame();

      for (int i = 0; i < 5; i++)
         source.onFrameReceived(1000L + i, new int[] {1, 2, 3, 4}, positions(0.1 * i), 4);

      assertEquals(5, source.getFramesReceived());
      assertEquals(4, source.getDroppedFrameCount(), "Four frames were overwritten before anything read them.");

      assertTrue(source.read(frame));
      assertEquals(1004L, frame.getTimestampNanoseconds(), "The surviving frame must be the newest, not the oldest.");
      assertFalse(source.read(frame), "Nothing new since the last read.");
      assertEquals(1, source.getFramesRead());
   }

   /** Unlabelled markers are discarded and counted, not treated as an error. */
   @Test
   public void testUnlabelledMarkersAreCountedAndIgnored()
   {
      NatNetMocapSource source = new NatNetMocapSource(labeling());
      MocapFrame frame = source.createFrame();

      source.onFrameReceived(1000L, new int[] {1, 999, 2, 998}, positions(0.0), 4);
      assertTrue(source.read(frame));

      assertEquals(2, frame.getVisibleCount());
      assertEquals(2, source.getUnlabelledMarkerCount());
   }

   /** Motive reporting a non-finite position is a dropout, not a measurement. */
   @Test
   public void testNonFinitePositionIsTreatedAsUnseen()
   {
      NatNetMocapSource source = new NatNetMocapSource(labeling());
      MocapFrame frame = source.createFrame();

      double[] positions = {0.1, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.3, Double.POSITIVE_INFINITY, 0.0, 0.4, 0.0, 0.0};
      source.onFrameReceived(1000L, new int[] {1, 2, 3, 4}, positions, 4);
      assertTrue(source.read(frame));

      assertEquals(2, frame.getVisibleCount());
      assertFalse(frame.get(1).isVisible());
      assertFalse(frame.get(2).isVisible());
      assertTrue(frame.get(3).isVisible());
   }

   /**
    * {@code read} returning false must not be read as end of stream on a live source -- that is
    * the whole reason {@link MocapSource} has two methods instead of one.
    */
   @Test
   public void testNotFinishedJustBecauseNoFrameIsWaiting()
   {
      NatNetMocapSource source = new NatNetMocapSource(labeling());
      MocapFrame frame = source.createFrame();

      assertFalse(source.read(frame));
      assertFalse(source.isFinished(), "A quiet moment is not the end of the stream.");

      source.close();
      assertTrue(source.isFinished());
      assertFalse(source.read(frame));
   }

   /** A frame delivered before close is still readable after it; close is not a discard. */
   @Test
   public void testPendingFrameSurvivesClose()
   {
      NatNetMocapSource source = new NatNetMocapSource(labeling());
      MocapFrame frame = source.createFrame();

      source.onFrameReceived(1000L, new int[] {1, 2, 3, 4}, positions(0.0), 4);
      source.close();

      assertFalse(source.isFinished(), "There is still a frame to hand over.");
      assertTrue(source.read(frame));
      assertTrue(source.isFinished());
   }

   /** The producer/consumer handoff under real contention, not just sequential calls. */
   @Test
   public void testConcurrentProducerAndConsumer() throws InterruptedException
   {
      NatNetMocapSource source = new NatNetMocapSource(labeling());
      int frameCount = 20_000;
      CountDownLatch started = new CountDownLatch(1);
      AtomicInteger inconsistent = new AtomicInteger();

      Thread producer = new Thread(() ->
      {
         started.countDown();

         for (int i = 0; i < frameCount; i++)
            source.onFrameReceived(1000L + i, new int[] {1, 2, 3, 4}, positions(0.001 * i), 4);

         source.close();
      });

      producer.start();
      started.await(5, TimeUnit.SECONDS);

      MocapFrame frame = source.createFrame();
      int read = 0;

      while (!source.isFinished())
      {
         if (source.read(frame))
         {
            read++;

            // Every frame must be internally coherent: all four markers carry the same offset the
            // producer stamped them with. A torn read would show up as a mismatch here.
            double base = frame.get(0).getPosition().getX();

            for (int m = 1; m < 4; m++)
            {
               if (Math.abs(frame.get(m).getPosition().getX() - (base + 0.1 * m)) > 1.0e-12)
                  inconsistent.incrementAndGet();
            }
         }
      }

      producer.join(10_000);

      assertEquals(0, inconsistent.get(), "A frame was read while it was being written.");
      assertEquals(frameCount, source.getFramesReceived());
      assertEquals(frameCount, read + source.getDroppedFrameCount(), "Every frame was either read or counted as dropped.");
      assertTrue(read > 0);
   }

   /** The live path runs at 200 Hz and must not allocate. */
   @Test
   public void testLiveHandoffIsAllocationFree()
   {
      NatNetMocapSource source = new NatNetMocapSource(labeling());
      MocapFrame frame = source.createFrame();
      int[] motiveIds = {1, 2, 3, 4};
      double[] positions = positions(0.0);

      AllocationMeasurement.assertAllocationFree("10,000 live frame handoffs", () ->
      {
         for (int i = 0; i < 10_000; i++)
         {
            source.onFrameReceived(1000L + i, motiveIds, positions, 4);
            source.read(frame);
         }
      });

      assertEquals(4, frame.getVisibleCount());
   }

   private static MarkerLabeling labeling()
   {
      return MarkerLabeling.against(MARKER_SET).add(1, "PELVIS_1").add(2, "PELVIS_2").add(3, "PELVIS_3").add(4, "PELVIS_4").build();
   }

   private static double[] positions(double offset)
   {
      return new double[] {offset, 0.0, 0.0, offset + 0.1, 0.0, 0.0, offset + 0.2, 0.0, 0.0, offset + 0.3, 0.0, 0.0};
   }
}
