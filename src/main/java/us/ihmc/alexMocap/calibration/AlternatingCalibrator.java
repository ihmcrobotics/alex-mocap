package us.ihmc.alexMocap.calibration;

import java.util.List;

import us.ihmc.alexMocap.calibration.BaseInitializer.GaugeTracking;
import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Approach A′, the alternating loop (FRAMEWORK.md §8):
 *
 * <pre>
 * F3  →  initial layouts
 * repeat:
 *     F5  →  Δ  given layouts
 *     F4  →  layouts given Δ
 * until converged
 * </pre>
 *
 * <p>
 * minimising
 * </p>
 *
 * <pre>
 * J = sum_{i,j,k}  || ^W m_ijk  -  ^W T_c^(k) · Δ · ^b T_i(q^(k)) · ^i p_ij ||²
 * </pre>
 *
 * <h2>Why this and not bundle adjustment</h2>
 * <p>
 * Each step is the exact global minimum of its subproblem, so {@code J} is monotonically
 * non-increasing <i>by construction</i> rather than by line search. That buys: about twenty lines,
 * no Jacobians, no manifold parameterisation, no gauge freedom to fix, and no initial guess that
 * has to be good. The base pose is the only quantity not trusted; joint offsets, link geometry and
 * inertials are trusted to a reasonable degree, and promoting them to unknowns buys nothing while
 * costing all of the above.
 * </p>
 * <p>
 * The monotonicity is the whole argument, so it is recorded rather than assumed:
 * {@link CalibrationReport#getObjectiveSequence()} carries {@code J} after every half-step, and a
 * test asserts the sequence never rises. If it ever does, one of the two solvers is not solving its
 * subproblem and the justification for choosing A′ has evaporated.
 * </p>
 *
 * <h2>Escalation to Approach B is a decision, not a fallback</h2>
 * <p>
 * If this converges to a poor {@code J}, the answer is not to reach for a nonlinear bundle
 * adjustment. §8 is explicit: escalate only if G2 fires <i>with structure</i> -- spread correlated
 * with one joint's excursion indicts that joint's offset, spread correlated with limb load indicts
 * elasticity, and isotropic spread matching {@code σ} indicts nothing at all and means A′ is
 * sufficient. A high {@code J} with no structure in G2 is mocap noise, and Approach B would fit it.
 * </p>
 */
public class AlternatingCalibrator
{
   /** §8: stop when the relative decrease in {@code J} falls below this. */
   public static final double DEFAULT_RELATIVE_TOLERANCE = 1.0e-9;

   /**
    * Or after this many iterations, whichever comes first.
    * <p>
    * <b>FRAMEWORK.md §8 says 50. That is too tight, and the way it fails is quiet.</b> A′ converges
    * linearly, at a rate set by how well the marked links identify {@code Δ}. On real data that is
    * fine: {@code J} flattens onto the mocap noise floor within a few tens of iterations, the
    * relative decrease collapses, and the tolerance stops the loop -- a 30-capture set at
    * {@code σ = 0.3 mm} converges in roughly 30 iterations.
    * </p>
    * <p>
    * It is <i>noiseless or weakly-conditioned</i> data that runs long, because {@code J} keeps
    * falling geometrically toward zero and the relative decrease stays around 0.1 forever. Capping
    * at 50 there does not produce an error; it produces a calibration stopped early, roughly 1 mm
    * from the answer, reporting a small {@code J} and looking converged. That is worse than a
    * failure, and it is why {@link CalibrationReport#isConverged()} exists and why the CLI prints
    * "HIT ITERATION CAP" in bold rather than as a footnote.
    * </p>
    * <p>
    * 500 costs milliseconds -- an iteration is two closed-form solves over a few hundred
    * correspondences -- and buys the margin. The tolerance, not the cap, should be what stops a
    * healthy run.
    * </p>
    */
   public static final int DEFAULT_MAXIMUM_ITERATIONS = 500;

   private final double relativeTolerance;
   private final int maximumIterations;

   private final BootstrapSolver bootstrapSolver = new BootstrapSolver();
   private final MarkerLayoutSolver layoutSolver = new MarkerLayoutSolver();
   private final BaseAlignmentSolver baseSolver = new BaseAlignmentSolver();

   public AlternatingCalibrator()
   {
      this(DEFAULT_RELATIVE_TOLERANCE, DEFAULT_MAXIMUM_ITERATIONS);
   }

   public AlternatingCalibrator(double relativeTolerance, int maximumIterations)
   {
      if (relativeTolerance < 0.0)
         throw new IllegalArgumentException("Relative tolerance must be non-negative, was " + relativeTolerance + ".");
      if (maximumIterations < 1)
         throw new IllegalArgumentException("Maximum iterations must be at least 1, was " + maximumIterations + ".");

      this.relativeTolerance = relativeTolerance;
      this.maximumIterations = maximumIterations;
   }

   /**
    * Runs A′ over every capture in the set.
    *
    * @param reportToPack filled with the objective trace and per-marker residuals. May be
    *                     {@code null}, though discarding it means discarding the only evidence that
    *                     the run converged rather than stopped.
    * @return the calibrated layouts and {@code Δ}. Provenance is left to the caller, which is the
    *         only party that knows the URDF path and the measured world tilt.
    */
   public CalibrationResult calibrate(CaptureSet captureSet, RobotModelHandle model, CalibrationReport reportToPack)
   {
      GaugeTracking tracking = BaseInitializer.trackGaugeCluster(captureSet);
      return calibrate(captureSet, model, tracking, reportToPack);
   }

   /**
    * Runs A′ with gauge tracking that has already been computed.
    * <p>
    * The tracking is a raw-data constant (see {@link BaseInitializer}), so a caller running several
    * calibrations over the same captures -- G4's splits, above all -- computes it once and passes
    * it in. That is not merely an optimisation: recomputing it per split from a different reference
    * capture would change the {@code Δ} convention between splits and make their layouts
    * incomparable.
    * </p>
    */
   public CalibrationResult calibrate(CaptureSet captureSet, RobotModelHandle model, GaugeTracking tracking, CalibrationReport reportToPack)
   {
      return calibrate(captureSet, model, tracking, CalibrationReport.everyCapture(captureSet), reportToPack);
   }

   /**
    * Runs A′ over a chosen subset of captures. This is what G4 fits on.
    */
   public CalibrationResult calibrate(CaptureSet captureSet,
                                      RobotModelHandle model,
                                      GaugeTracking tracking,
                                      int[] captureIndices,
                                      CalibrationReport reportToPack)
   {
      CalibrationReport report = reportToPack == null ? new CalibrationReport() : reportToPack;
      report.setTracking(tracking, captureSet.getCaptureCount());

      // F3: initial layouts, at Δ = I.
      List<ClusterLayout> layouts = bootstrapSolver.bootstrap(captureSet, model, tracking);

      RigidBodyTransform clusterToBase = new RigidBodyTransform();
      BaseInitializer.packInitialClusterToBase(clusterToBase);

      double previousObjective = CalibrationReport.computeObjective(captureSet, model, tracking, clusterToBase, layouts, captureIndices);
      report.setBootstrapObjective(previousObjective);

      boolean converged = false;

      for (int iteration = 0; iteration < maximumIterations && !converged; iteration++)
      {
         // F5: Δ given layouts. Exact global minimiser of J over Δ.
         if (!baseSolver.solve(captureSet, model, tracking, layouts, captureIndices, clusterToBase))
            throw new IllegalStateException("F5 could not solve for Δ: fewer than " + us.ihmc.alexMocap.registration.RigidBodyRegistration.MINIMUM_CORRESPONDENCES
                  + " usable (link, marker, capture) correspondences exist across the whole capture set.");

         double afterBaseStep = CalibrationReport.computeObjective(captureSet, model, tracking, clusterToBase, layouts, captureIndices);

         // F4: layouts given Δ. Exact global minimiser of J over the layouts.
         layoutSolver.solve(captureSet, model, tracking, clusterToBase, captureIndices, layouts);

         double afterMarkerStep = CalibrationReport.computeObjective(captureSet, model, tracking, clusterToBase, layouts, captureIndices);

         // Relative to the previous iteration's end, which is what "the relative decrease in J"
         // means when the iteration is the unit. Guarded against J = 0, which the noiseless
         // synthetic case reaches exactly and which would otherwise divide by zero and never
         // converge.
         double relativeDecrease = previousObjective > 0.0 ? (previousObjective - afterMarkerStep) / previousObjective : 0.0;

         report.addIteration(new CalibrationReport.IterationRecord(iteration, afterBaseStep, afterMarkerStep, relativeDecrease));

         converged = relativeDecrease < relativeTolerance;
         previousObjective = afterMarkerStep;
      }

      report.setConverged(converged);
      report.setBaseStepSigma3(baseSolver.getLastResult().getSigma3());
      report.setObservationCount(CalibrationReport.countObservations(captureSet, model, tracking, clusterToBase, layouts, captureIndices));
      report.setMarkerResiduals(CalibrationReport.computeMarkerResiduals(captureSet, model, tracking, clusterToBase, layouts, captureIndices));

      CalibrationResult result = new CalibrationResult();
      result.getClusterToBase().set(clusterToBase);

      for (ClusterLayout layout : layouts)
         result.addLayout(layout);

      return result;
   }
}
