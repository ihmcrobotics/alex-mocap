package us.ihmc.alexMocap.core;

/**
 * One calibration capture {@code k}: a {@link MocapFrame} paired with the {@link EncoderSample}
 * that goes with it.
 * <p>
 * This is the unit F3, F4 and F5 consume. Everything in FRAMEWORK.md §5-§7 is indexed by
 * {@code k}, and this is what {@code k} names.
 * </p>
 *
 * <h2>The pairing is the risk</h2>
 * <p>
 * Nothing about a wrongly paired capture looks wrong. The mocap frame is valid, the encoder sample
 * is valid, and the calibration that consumes them produces a clean-looking answer at the wrong
 * configuration -- FRAMEWORK.md §18.3. {@link #getTimestampSkewNanoseconds()} exists so the
 * pairing is a number someone can look at rather than an assumption.
 * </p>
 * <p>
 * It is reported, never thresholded. What counts as acceptable skew depends on how fast the robot
 * was moving, and calibration captures are static while runtime frames are not. A gate decides.
 * </p>
 */
public class Capture
{
   private final MocapFrame mocapFrame;
   private final EncoderSample encoderSample;

   public Capture(MocapFrame mocapFrame, EncoderSample encoderSample)
   {
      if (mocapFrame == null || encoderSample == null)
         throw new IllegalArgumentException("A capture needs both a mocap frame and an encoder sample.");

      this.mocapFrame = mocapFrame;
      this.encoderSample = encoderSample;
   }

   /** Allocates a capture with its own frame and sample, sized to the given set and joint order. */
   public static Capture create(java.util.List<MarkerId> markers, java.util.List<String> jointNames)
   {
      return new Capture(new MocapFrame(markers), new EncoderSample(jointNames));
   }

   public MocapFrame getMocapFrame()
   {
      return mocapFrame;
   }

   public EncoderSample getEncoderSample()
   {
      return encoderSample;
   }

   /**
    * Mocap timestamp minus encoder timestamp, in nanoseconds. Positive means the mocap frame is
    * the later of the two.
    *
    * @return the skew, or {@link MocapFrame#NO_TIMESTAMP} if either timestamp is unset -- which is
    *         itself worth noticing, since a capture with no timestamps cannot be checked at all.
    */
   public long getTimestampSkewNanoseconds()
   {
      long mocapTime = mocapFrame.getTimestampNanoseconds();
      long encoderTime = encoderSample.getTimestampNanoseconds();

      if (mocapTime == MocapFrame.NO_TIMESTAMP || encoderTime == MocapFrame.NO_TIMESTAMP)
         return MocapFrame.NO_TIMESTAMP;

      return mocapTime - encoderTime;
   }

   public boolean hasTimestamps()
   {
      return getTimestampSkewNanoseconds() != MocapFrame.NO_TIMESTAMP;
   }

   public void set(Capture other)
   {
      mocapFrame.set(other.mocapFrame);
      encoderSample.set(other.encoderSample);
   }

   @Override
   public String toString()
   {
      return "Capture[" + mocapFrame + ", " + encoderSample + ", skew=" + (hasTimestamps() ? getTimestampSkewNanoseconds() + " ns" : "unknown") + "]";
   }
}
