package us.ihmc.alexMocap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.management.ManagementFactory;

/**
 * Asserts that a batch of work allocates nothing, in a way that does not flake.
 *
 * <h2>Why the minimum over several windows, and not a single window</h2>
 * <p>
 * A single measured window is not reliable. The JIT recompiles and deoptimises on its own
 * schedule, and that bookkeeping is charged to the running thread: measuring one 10,000-iteration
 * window of genuinely allocation-free code gives readings like 2792, 0, 104, 0, 0, 0 -- the
 * non-zero ones being compilation events, not the code under test. Padding the warmup only makes
 * the flake rarer, and a test that fails one run in fifty gets disabled.
 * </p>
 * <p>
 * Taking the minimum over several windows is exact rather than approximate: <b>if the batch
 * allocated even one byte per iteration, no window could read zero.</b> So a zero minimum proves
 * allocation-freedom, and a non-zero minimum is a real regression. Compare the registration
 * mutation check, where injecting one {@code double[4]} into the solve put every window at 480,000
 * bytes.
 * </p>
 * <p>
 * No JVM flags are needed. HotSpot's {@code getThreadAllocatedBytes} includes the in-progress
 * TLAB, so TLAB batching does not hide allocation.
 * </p>
 *
 * <h2>What this does and does not prove</h2>
 * <p>
 * It measures what the JVM <i>actually allocates</i>, which is the quantity that matters -- an
 * allocation the JIT scalar-replaces costs the collector nothing. It is therefore not a proof that
 * the source contains no {@code new}: a short method whose object provably does not escape gets
 * optimised away entirely and reads as zero. Injecting {@code new double[2]} into
 * {@code MocapFrame.clear()} is invisible to this measurement for exactly that reason; assigning
 * the same array to a static field makes it escape and shows up immediately at 320,000 bytes per
 * batch.
 * </p>
 * <p>
 * That is the right trade for this codebase, but it means the guarantee is "does not allocate as
 * compiled and run here", not "contains no allocating expression". Escape analysis is a JIT
 * decision and can change with the JVM or with surrounding code, so keep hot-path methods small
 * and do not rely on it deliberately.
 * </p>
 */
public final class AllocationMeasurement
{
   private static final int WARMUP_BATCHES = 5;
   private static final int MEASURED_BATCHES = 5;

   private AllocationMeasurement()
   {
   }

   /**
    * @param description what the batch does, for the failure message.
    * @param batch       one self-contained unit of work, run repeatedly. Size it so that a
    *                    per-iteration allocation would be unmistakable -- thousands of iterations,
    *                    not one.
    */
   public static void assertAllocationFree(String description, Runnable batch)
   {
      com.sun.management.ThreadMXBean threadBean = (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
      assertTrue(threadBean.isThreadAllocatedMemorySupported(), "This JVM cannot measure per-thread allocation; the assertion would be vacuous.");
      threadBean.setThreadAllocatedMemoryEnabled(true);

      long threadId = Thread.currentThread().getId();
      checkTheMeterWorks(threadBean, threadId);

      for (int i = 0; i < WARMUP_BATCHES; i++)
         batch.run();

      long[] readings = new long[MEASURED_BATCHES];
      long minimum = Long.MAX_VALUE;

      for (int i = 0; i < MEASURED_BATCHES; i++)
      {
         long before = threadBean.getThreadAllocatedBytes(threadId);
         batch.run();
         readings[i] = threadBean.getThreadAllocatedBytes(threadId) - before;
         minimum = Math.min(minimum, readings[i]);
      }

      assertEquals(0L, minimum, description + " allocated on every measured batch: " + java.util.Arrays.toString(readings) + " bytes");
   }

   /** A loop known to allocate must read as allocating, or a zero elsewhere proves nothing. */
   private static void checkTheMeterWorks(com.sun.management.ThreadMXBean threadBean, long threadId)
   {
      Object[] sink = new Object[1024];
      long before = threadBean.getThreadAllocatedBytes(threadId);

      for (int i = 0; i < sink.length; i++)
         sink[i] = new double[8];

      long allocated = threadBean.getThreadAllocatedBytes(threadId) - before;

      assertTrue(allocated > 0, "The allocation meter reported zero for a loop that allocates 1024 arrays.");
      assertTrue(sink[0] != null);
   }
}
