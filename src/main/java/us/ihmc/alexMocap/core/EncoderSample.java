package us.ihmc.alexMocap.core;

import java.util.List;

/**
 * Joint encoder readings at one instant: {@code q^(k)}, plus the timestamp they were read at.
 * <p>
 * Mutable and reusable, like {@link MocapFrame}, and paired with one by {@link Capture}.
 * </p>
 *
 * <h2>Joint names travel with the vector</h2>
 * <p>
 * A bare {@code double[] q} is an ordering waiting to be got wrong, and a permuted joint vector
 * produces a fully plausible FK result at the wrong configuration. The name list is held by
 * reference and shared across every sample in a session, so carrying it costs nothing per sample
 * and makes {@link #checkJointOrder} possible at the boundary with the URDF.
 * </p>
 *
 * <h2>What q is not</h2>
 * <p>
 * {@code q} is the encoder reading, which under load is not the joint angle: harmonic drives and
 * structural sag put a configuration-correlated offset between them (FRAMEWORK.md §21.2). This
 * type records what the encoder said. Nothing here corrects it, and F4's averaging will not remove
 * it -- that bias survives the mean and G2 is what detects it.
 * </p>
 */
public class EncoderSample
{
   private final List<String> jointNames;
   private final double[] q;
   private long timestampNanoseconds = MocapFrame.NO_TIMESTAMP;

   /**
    * @param jointNames the joint order, matching the URDF. Held by reference, not copied.
    */
   public EncoderSample(List<String> jointNames)
   {
      if (jointNames == null || jointNames.isEmpty())
         throw new IllegalArgumentException("Joint name list must be non-empty.");

      this.jointNames = List.copyOf(jointNames);
      this.q = new double[jointNames.size()];
      java.util.Arrays.fill(q, Double.NaN);
   }

   public List<String> getJointNames()
   {
      return jointNames;
   }

   public int getJointCount()
   {
      return q.length;
   }

   public double getQ(int jointIndex)
   {
      return q[jointIndex];
   }

   /**
    * Copies the joint vector out. There is no accessor returning the backing array: handing it out
    * would let a caller write joint angles without going through this type, and would make the
    * "reusable, shared across a session" contract unenforceable.
    */
   public void getQ(double[] toPack)
   {
      if (toPack.length != q.length)
         throw new IllegalArgumentException("Expected an array of " + q.length + " joints, got " + toPack.length + ".");

      System.arraycopy(q, 0, toPack, 0, q.length);
   }

   public void setQ(int jointIndex, double value)
   {
      q[jointIndex] = value;
   }

   public void setQ(double[] values)
   {
      if (values.length != q.length)
         throw new IllegalArgumentException("Expected " + q.length + " joint values, got " + values.length + ".");

      System.arraycopy(values, 0, q, 0, q.length);
   }

   public long getTimestampNanoseconds()
   {
      return timestampNanoseconds;
   }

   public void setTimestampNanoseconds(long timestampNanoseconds)
   {
      this.timestampNanoseconds = timestampNanoseconds;
   }

   /** Sets every joint to NaN and clears the timestamp. */
   public void clear()
   {
      java.util.Arrays.fill(q, Double.NaN);
      timestampNanoseconds = MocapFrame.NO_TIMESTAMP;
   }

   public void set(EncoderSample other)
   {
      if (!jointNames.equals(other.jointNames))
         throw new IllegalArgumentException("Joint orders differ: " + jointNames + " vs " + other.jointNames + ".");

      System.arraycopy(other.q, 0, q, 0, q.length);
      timestampNanoseconds = other.timestampNanoseconds;
   }

   /**
    * Asserts that this sample's joint order matches an expected one, name for name.
    * <p>
    * Call this once where encoder data meets the robot model. A silent permutation here is
    * indistinguishable downstream from a bad calibration.
    * </p>
    */
   public void checkJointOrder(List<String> expectedJointNames)
   {
      if (!jointNames.equals(expectedJointNames))
         throw new IllegalArgumentException("Joint order mismatch.\n  sample:   " + jointNames + "\n  expected: " + expectedJointNames);
   }

   @Override
   public String toString()
   {
      return "EncoderSample[t=" + timestampNanoseconds + " ns, " + q.length + " joints]";
   }
}
