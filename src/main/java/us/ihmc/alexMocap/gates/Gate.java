package us.ihmc.alexMocap.gates;

/**
 * A check that runs <i>before</i> the thing it protects, costs minutes, and answers pass or fail.
 * <p>
 * FRAMEWORK.md §15. The four gates each isolate one class of failure so that when something is
 * wrong, the answer to "wrong where" is already narrowed: G1 is purely a mocap-and-mounting
 * question, G2 is purely a model question, G3 is purely a volume question, G4 is the accuracy
 * claim. Running them in order means a failure at each stage indicts one thing rather than the
 * whole pipeline.
 * </p>
 * <p>
 * A gate takes its input at construction and computes on {@link #run()}. That keeps this interface
 * free of any particular data source, which is what lets {@code gates} depend only on
 * {@code core}, {@code model} and {@code registration} -- notably <b>not</b> on
 * {@code calibration}. G2 evaluates F4 per capture directly, which is exactly what lets it run
 * before the calibrator exists.
 * </p>
 */
public interface Gate
{
   /** Short identifier, e.g. {@code "G1"}. */
   String getName();

   /** What this gate protects, in one line. Printed above its table. */
   String getDescription();

   /**
    * Evaluates the gate over whatever input it was given.
    * <p>
    * Implementations must be deterministic: the same input produces the same verdict. A gate that
    * sometimes passes is worse than no gate.
    * </p>
    */
   GateResult run();
}
