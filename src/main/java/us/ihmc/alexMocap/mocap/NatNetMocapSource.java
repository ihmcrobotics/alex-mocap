package us.ihmc.alexMocap.mocap;

import java.util.List;

import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;

/**
 * Live capture from Motive over NatNet: the consumer end of the stream.
 *
 * <h2>What this class is</h2>
 * <p>
 * The handoff between the NatNet client's callback thread, which delivers frames when Motive sends
 * them, and the control thread, which reads them when it is ready. That handoff -- labelling,
 * latest-frame semantics, dropped-frame accounting, thread safety -- is implemented here and
 * covered by tests.
 * </p>
 *
 * <h2>What this class is not</h2>
 * <p>
 * It does not speak the NatNet wire protocol and it does not open a socket. It is driven: a NatNet
 * client calls {@link #onFrameReceived} and this class does the rest. The intended client is
 * {@code us.ihmc.mocap}'s in ihmc-open-robotics-software (which is why FRAMEWORK.md §19 nests this
 * project under {@code us.ihmc.alexMocap} -- to avoid a split package with it). That artifact is
 * not on this build's classpath, so the adapter is not written here.
 * </p>
 * <p>
 * Wiring it is a handful of lines in whichever module has the client: on each frame, collect the
 * labelled markers' streaming ids and positions and call {@link #onFrameReceived}. There is
 * deliberately no {@code connect()} method that throws -- a method that compiles and fails at
 * runtime reads as working code, and this seam should be visible at the call site instead.
 * </p>
 * <p>
 * Per PR_PLAN.md this class has no unit test for the connection itself: a mocked NatNet client
 * tests the mock. The manual smoke-test procedure is in RUNNING.md.
 * </p>
 *
 * <h2>Latest-frame semantics, and why frames are dropped on purpose</h2>
 * <p>
 * One staging frame is kept. If Motive delivers a second frame before the consumer has read the
 * first, the first is overwritten and {@link #getDroppedFrameCount()} increments. This is right
 * for a control loop -- the newest pose is the useful one, and queueing would trade latency for
 * completeness in the wrong direction -- but it is wrong for logging, where a dropped frame is a
 * hole in the capture.
 * </p>
 * <p>
 * <b>So check {@link #getDroppedFrameCount()} after any capture you intend to calibrate from.</b>
 * A log with drops still produces a confident calibration; it just does so from fewer captures
 * than you think it used.
 * </p>
 *
 * <h2>Thread safety</h2>
 * <p>
 * {@link #onFrameReceived} and {@link #read} may be called from different threads and synchronise
 * on a private lock. The critical section is a copy of a few dozen doubles; at 200 Hz the lock is
 * held roughly 0.02% of the time, and an uncontended lock costs less than getting a lock-free ring
 * subtly wrong.
 * </p>
 */
public class NatNetMocapSource implements MocapSource
{
   private final Object lock = new Object();
   private final MarkerLabeling labeling;
   private final List<MarkerId> markers;

   /** Written by the producer thread, drained by the consumer. Guarded by {@link #lock}. */
   private final MocapFrame staging;

   private boolean newFrameAvailable = false;
   private boolean closed = false;
   private long framesReceived = 0;
   private long framesRead = 0;
   private long droppedFrameCount = 0;
   private long unlabelledMarkerCount = 0;

   public NatNetMocapSource(MarkerLabeling labeling)
   {
      this.labeling = labeling;
      this.markers = labeling.getMarkers();
      this.staging = new MocapFrame(markers);
   }

   /**
    * Called by the NatNet client for each frame Motive sends.
    * <p>
    * Markers not present in the labelling are counted and ignored: Motive streams unlabelled
    * point-cloud markers alongside the labelled ones, and that is normal rather than an error.
    * Markers in the set that this frame did not report are left explicitly not visible, never
    * carried over from the previous frame.
    * </p>
    *
    * @param timestampNanoseconds frame timestamp. Must share an epoch with the encoder stream --
    *                             FRAMEWORK.md §18.3.
    * @param motiveIds            streaming ids, {@code markerCount} of them.
    * @param positionsXYZ         interleaved xyz in metres, {@code 3 * markerCount} entries.
    * @param markerCount          number of markers reported this frame.
    */
   public void onFrameReceived(long timestampNanoseconds, int[] motiveIds, double[] positionsXYZ, int markerCount)
   {
      if (markerCount < 0 || motiveIds.length < markerCount || positionsXYZ.length < 3 * markerCount)
         throw new IllegalArgumentException("Frame reports " + markerCount + " markers but the arrays are too short.");

      synchronized (lock)
      {
         if (closed)
            return;

         if (newFrameAvailable)
            droppedFrameCount++;

         staging.clear();
         staging.setTimestampNanoseconds(timestampNanoseconds);

         for (int i = 0; i < markerCount; i++)
         {
            int index = labeling.indexOf(motiveIds[i]);

            if (index < 0)
            {
               unlabelledMarkerCount++;
               continue;
            }

            MarkerObservation observation = staging.get(index);
            int p = 3 * i;

            // A marker Motive reports at a non-finite position is a dropout, not a measurement.
            // MarkerObservation.setVisible would reject it; treat it as unseen instead.
            if (Double.isFinite(positionsXYZ[p]) && Double.isFinite(positionsXYZ[p + 1]) && Double.isFinite(positionsXYZ[p + 2]))
               observation.setVisible(positionsXYZ[p], positionsXYZ[p + 1], positionsXYZ[p + 2]);
         }

         framesReceived++;
         newFrameAvailable = true;
         lock.notifyAll();
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
      synchronized (lock)
      {
         if (!newFrameAvailable)
            return false;

         frameToPack.set(staging);
         newFrameAvailable = false;
         framesRead++;
         return true;
      }
   }

   /**
    * Blocks until a frame arrives or the timeout expires.
    * <p>
    * For the recorder, which has nothing else to do between frames. A control loop should use
    * {@link #read} and get on with its other work.
    * </p>
    *
    * @return {@code true} if a frame was packed.
    */
   public boolean readBlocking(MocapFrame frameToPack, long timeoutMilliseconds) throws InterruptedException
   {
      long deadline = System.nanoTime() + timeoutMilliseconds * 1_000_000L;

      synchronized (lock)
      {
         while (!newFrameAvailable && !closed)
         {
            long remaining = deadline - System.nanoTime();

            if (remaining <= 0)
               return false;

            lock.wait(remaining / 1_000_000L, (int) (remaining % 1_000_000L));
         }

         if (!newFrameAvailable)
            return false;

         frameToPack.set(staging);
         newFrameAvailable = false;
         framesRead++;
         return true;
      }
   }

   @Override
   public boolean isFinished()
   {
      synchronized (lock)
      {
         return closed && !newFrameAvailable;
      }
   }

   /** Frames Motive delivered. */
   public long getFramesReceived()
   {
      synchronized (lock)
      {
         return framesReceived;
      }
   }

   /** Frames the consumer actually took. */
   public long getFramesRead()
   {
      synchronized (lock)
      {
         return framesRead;
      }
   }

   /**
    * Frames overwritten before the consumer read them. Non-zero is expected in a control loop and
    * is a hole in the capture for anything you intend to calibrate from.
    */
   public long getDroppedFrameCount()
   {
      synchronized (lock)
      {
         return droppedFrameCount;
      }
   }

   /**
    * Marker observations discarded because no {@link MarkerLabeling} entry claimed them. A steady
    * count is normal -- unlabelled point-cloud markers. A count that matches your labelled marker
    * count means the labelling ids are wrong.
    */
   public long getUnlabelledMarkerCount()
   {
      synchronized (lock)
      {
         return unlabelledMarkerCount;
      }
   }

   @Override
   public void close()
   {
      synchronized (lock)
      {
         closed = true;
         lock.notifyAll();
      }
   }
}
