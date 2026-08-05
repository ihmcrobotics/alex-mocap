package us.ihmc.alexMocap.mocap;

import java.util.List;

import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;

/**
 * A stream of {@link MocapFrame}s, whether from a live NatNet connection or a replayed CSV log.
 * <p>
 * Everything above this interface -- the gates, the calibrator, the runtime loop -- is written
 * against replayed logs and runs unchanged against live capture. That is the whole point of having
 * the interface: no test in this repository needs a camera.
 * </p>
 *
 * <h2>The loop</h2>
 * <p>
 * {@link #read} packs a frame if one is available, and {@link #isFinished} says whether any more
 * will ever arrive. Both are needed, because "nothing right now" and "nothing ever again" are
 * different states and only one of them should end a loop:
 * </p>
 *
 * <pre>
 * MocapFrame frame = source.createFrame();
 *
 * while (!source.isFinished())
 * {
 *    if (source.read(frame))
 *       ... // process
 * }
 * </pre>
 *
 * <p>
 * Writing {@code while (source.read(frame))} instead works perfectly on a CSV replay and then
 * exits on the first dropped packet against live capture. The two-method shape exists so that the
 * obvious loop is also the correct one.
 * </p>
 *
 * <h2>Contract</h2>
 * <ul>
 * <li>{@link #read} packs a caller-owned frame and allocates nothing. Implementations must hold to
 * that -- it runs at 200 Hz.</li>
 * <li>Frames are packed complete: every marker in the set is either visible with a finite position
 * or explicitly not visible. A source never leaves an observation untouched from the previous
 * frame, because a stale position registers without complaint.</li>
 * <li>The marker set is fixed for the life of the source.</li>
 * </ul>
 */
public interface MocapSource extends AutoCloseable
{
   /** The session marker set. Every frame this source packs is addressed by it. */
   List<MarkerId> getMarkers();

   /** A frame sized for this source. Allocate once, outside the loop. */
   default MocapFrame createFrame()
   {
      return new MocapFrame(getMarkers());
   }

   /**
    * Packs the next available frame.
    *
    * @param frameToPack must have been built from {@link #getMarkers()}.
    * @return {@code true} if a frame was packed; {@code false} if none is available right now.
    *         {@code false} does not mean the stream has ended -- see {@link #isFinished()}.
    */
   boolean read(MocapFrame frameToPack);

   /**
    * @return {@code true} once no further frame will ever arrive: end of file for a replay, or a
    *         closed connection for a live source.
    */
   boolean isFinished();

   @Override
   void close();
}
