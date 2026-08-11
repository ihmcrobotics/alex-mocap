package us.ihmc.alexMocap.gates;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * G5: does the robot's measured weight and balance agree with the URDF the CoM is computed from?
 *
 * <h2>What this protects</h2>
 * <p>
 * F11 measures the CoM error budget as mass 4.90 mm / link-CoM 2.73 mm / mocap 0.164 mm on Alex:
 * <b>the CAD terms dominate the mocap term by 33x</b>. Every one of those numbers rests on an
 * <i>assumed</i> 5 % per-link mass uncertainty. Nothing in this pipeline has ever checked that
 * assumption against the robot, and if it is wrong the CoM ground truth is wrong by a margin no
 * amount of camera work reduces. This gate is that check.
 * </p>
 *
 * <h2>Two faults, two independent signals</h2>
 * <p>
 * Ground reaction forces measure the same body the URDF describes, and they separate the two
 * failure modes that a single CoM number confounds:
 * </p>
 * <ul>
 * <li><b>Total mass.</b> Statically, {@code sum(F_z) = M g}. This catches a robot that does not
 * weigh what the model says -- and it is the <i>only</i> one of the two that a total-mass
 * measurement can catch.</li>
 * <li><b>Mass distribution.</b> Statically the centre of pressure sits directly under the centre
 * of mass, so {@code p_xy = c_xy}. A persistent offset means the model puts the mass in the wrong
 * <i>place</i>, which the total mass cannot see at all.</li>
 * </ul>
 * <p>
 * The second check is the one worth having, because scaling every link mass by a constant changes
 * {@code sum(m_i)} and leaves {@code sum(m_i r_i) / sum(m_i)} <b>exactly unchanged</b>. Total mass
 * is the one degree of freedom the CoM is blind to. A model rescaled to match the scales can still
 * be centimetres wrong about where the mass sits, and will report a perfectly healthy total.
 * </p>
 *
 * <h2>Quasi-static only, and the gate will not pretend otherwise</h2>
 * <p>
 * {@code p_xy = c_xy} holds when nothing is accelerating. In general
 * </p>
 *
 * <pre>
 * p = c_xy - (c_z / (c_z_ddot + g)) c_xy_ddot - Ldot / (M (c_z_ddot + g))
 * </pre>
 * <p>
 * and recovering {@code c_ddot} would mean double-differentiating mocap, which FRAMEWORK.md §13
 * already refuses to do once for <i>velocity</i> -- at 0.3 mm marker noise a second derivative is
 * noise with a trend in it. So the caller states which samples are quasi-static and this gate
 * accumulates only those. Samples it was not offered are counted and reported: a run with too few
 * accepted samples is {@code NOT_EVALUATED}, never {@code PASS}. With two states a robot that was
 * never still would produce the most confident possible green, because nothing contradicted it.
 * </p>
 *
 * <h2>The mean is the measurement; the spread is not</h2>
 * <p>
 * A mass-model error is a <b>bias</b>: it does not average down. Centre-of-pressure noise does, as
 * {@code 1/sqrt(N)}. So the threshold is applied to the <i>mean</i> offset and the standard
 * deviation is reported beside it for the reader, exactly as the CoM comparison does with
 * {@code mocapMinusActualComMean}. A floor in the mean is the fault; spread around it is the
 * force plate.
 * </p>
 * <p>
 * Axes are reported separately rather than as one magnitude. A magnitude is positive under any
 * fault and folds two independent measurements into one; separate axes say <i>which way</i> the
 * mass is misplaced, and the axis that moves points at the segment responsible.
 * </p>
 */
public class MassConsistencyGate implements Gate
{
   /** Findings fail beyond this many times the expected uncertainty. Matches G1's convention. */
   public static final double DEFAULT_SIGMA_MULTIPLIER = 3.0;

   /**
    * Quasi-static samples required before either check is evaluated.
    * <p>
    * Lower than G1's 100 because these are whole-body postures rather than mocap frames -- a robot
    * standing still for a second at 200 Hz clears it -- but not so low that one transient reads as
    * a measurement.
    * </p>
    */
   public static final int DEFAULT_MINIMUM_SAMPLES = 50;

   private final double modelTotalMass;
   private final double gravity;
   private final double expectedComUncertainty;
   private final double massUncertaintyFraction;
   private final double sigmaMultiplier;
   private final int minimumSamples;

   private final Statistics offsetX = new Statistics();
   private final Statistics offsetY = new Statistics();
   private final Statistics verticalForce = new Statistics();

   private long samplesOffered;
   private long samplesRejectedAsDynamic;
   private long samplesRejectedAsNaN;

   /**
    * @param modelTotalMass          what the URDF says the robot masses, kg.
    * @param gravity                 magnitude, m/s^2. Passed rather than assumed: it scales the
    *                                implied mass directly, so a hard-coded 9.81 would quietly bias
    *                                the answer by the local deviation.
    * @param expectedComUncertainty  the CoM error the model's own uncertainty already predicts,
    *                                metres -- F11's mass term (4.90 mm on Alex at 5 %). This is the
    *                                bar the measured offset has to clear to mean anything; an
    *                                offset inside it is consistent with a model nobody claimed was
    *                                exact.
    * @param massUncertaintyFraction the per-link mass uncertainty the budget assumed, e.g. 0.05.
    */
   public MassConsistencyGate(double modelTotalMass, double gravity, double expectedComUncertainty, double massUncertaintyFraction)
   {
      this(modelTotalMass, gravity, expectedComUncertainty, massUncertaintyFraction, DEFAULT_SIGMA_MULTIPLIER, DEFAULT_MINIMUM_SAMPLES);
   }

   public MassConsistencyGate(double modelTotalMass,
                              double gravity,
                              double expectedComUncertainty,
                              double massUncertaintyFraction,
                              double sigmaMultiplier,
                              int minimumSamples)
   {
      if (!(modelTotalMass > 0.0))
         throw new IllegalArgumentException("Model total mass must be positive, got " + modelTotalMass + " kg.");
      if (!(gravity > 0.0))
         throw new IllegalArgumentException("Gravity must be positive, got " + gravity + " m/s^2.");
      if (!(expectedComUncertainty >= 0.0) || !Double.isFinite(expectedComUncertainty))
         throw new IllegalArgumentException("Expected CoM uncertainty must be finite and non-negative, got " + expectedComUncertainty
               + " m. There is no default -- it comes from the error budget (F11).");
      if (!(massUncertaintyFraction > 0.0))
         throw new IllegalArgumentException("Mass uncertainty fraction must be positive, got " + massUncertaintyFraction + ".");

      this.modelTotalMass = modelTotalMass;
      this.gravity = gravity;
      this.expectedComUncertainty = expectedComUncertainty;
      this.massUncertaintyFraction = massUncertaintyFraction;
      this.sigmaMultiplier = sigmaMultiplier;
      this.minimumSamples = minimumSamples;
   }

   @Override
   public String getName()
   {
      return "G5";
   }

   @Override
   public String getDescription()
   {
      return "mass model: measured weight and balance must agree with the URDF the CoM is computed from";
   }

   /**
    * One quasi-static sample.
    *
    * @param modelCenterOfMass  the CoM this pipeline computed, in the gravity-aligned world.
    * @param centerOfPressure   the measured CoP, in the same frame. Only x and y are read.
    * @param totalVerticalForce summed vertical ground reaction, newtons.
    * @param quasiStatic        whether the robot was still enough for {@code p_xy = c_xy} to hold.
    *                           False is recorded and reported, not silently dropped.
    */
   public void accumulate(Point3DReadOnly modelCenterOfMass, Point3DReadOnly centerOfPressure, double totalVerticalForce, boolean quasiStatic)
   {
      samplesOffered++;

      if (!quasiStatic)
      {
         samplesRejectedAsDynamic++;
         return;
      }

      // A refused cluster leaves NaN, and NaN is the honest value for "no measurement" -- but it
      // must not enter a mean, where one of them would poison every number downstream.
      if (modelCenterOfMass.containsNaN() || centerOfPressure.containsNaN() || !Double.isFinite(totalVerticalForce))
      {
         samplesRejectedAsNaN++;
         return;
      }

      offsetX.accumulate(modelCenterOfMass.getX() - centerOfPressure.getX());
      offsetY.accumulate(modelCenterOfMass.getY() - centerOfPressure.getY());
      verticalForce.accumulate(totalVerticalForce);
   }

   @Override
   public GateResult run()
   {
      GateResult result = new GateResult(getName());
      long n = offsetX.getCount();

      if (n < minimumSamples)
      {
         String detail = n + " quasi-static samples of " + samplesOffered + " offered (" + samplesRejectedAsDynamic + " moving, "
               + samplesRejectedAsNaN + " unmeasured); " + minimumSamples + " required";
         result.add(GateResult.Finding.notEvaluated("total mass", n, detail));
         result.add(GateResult.Finding.notEvaluated("mass distribution x", n, detail));
         result.add(GateResult.Finding.notEvaluated("mass distribution y", n, detail));
         return result.setSummary("Not evaluated: " + detail + ". INCOMPLETE is not PASS.");
      }

      result.add(totalMassFinding(n));
      result.add(distributionFinding("mass distribution x", offsetX, n));
      result.add(distributionFinding("mass distribution y", offsetY, n));

      return result.setSummary(String.format("%d quasi-static samples; implied mass %.3f kg against the model's %.3f kg; "
            + "mean CoP offset (%.2f, %.2f) mm against a %.2f mm bar",
                                             n,
                                             getImpliedTotalMass(),
                                             modelTotalMass,
                                             1000.0 * offsetX.getMean(),
                                             1000.0 * offsetY.getMean(),
                                             1000.0 * getDistributionThreshold()));
   }

   private GateResult.Finding totalMassFinding(long n)
   {
      double implied = getImpliedTotalMass();
      double error = Math.abs(implied - modelTotalMass);
      double threshold = massUncertaintyFraction * modelTotalMass;

      String detail = String.format("scales say %.3f kg, model says %.3f kg (%+.1f %%); mean vertical GRF %.1f N, sd %.1f N",
                                    implied,
                                    modelTotalMass,
                                    100.0 * (implied - modelTotalMass) / modelTotalMass,
                                    verticalForce.getMean(),
                                    verticalForce.getStandardDeviation());

      if (error > threshold)
      {
         // Worth saying out loud, because it is the intuitive fix and it is a no-op: rescaling every
         // link to match the scales leaves the CoM exactly where it was.
         detail += ". Rescaling every link mass to match would NOT move the CoM -- see 'mass distribution' below for that";
         return GateResult.Finding.fail("total mass", error, threshold, n, detail);
      }

      return GateResult.Finding.pass("total mass", error, threshold, n, detail);
   }

   private GateResult.Finding distributionFinding(String subject, Statistics offset, long n)
   {
      double measured = Math.abs(offset.getMean());
      double threshold = getDistributionThreshold();

      // Mass moment, kg*m: the offset expressed as the quantity actually missing from the model.
      // 17.5 kg misplaced by 0.3 m and 5.25 kg at 1.0 m are the same fault to a CoM, and this is
      // the number that says so -- far more actionable than a millimetre.
      double momentError = modelTotalMass * offset.getMean();

      String detail = String.format("mean %+.2f mm, sd %.2f mm over %d samples; %+.3f kg m of unmodelled mass moment",
                                    1000.0 * offset.getMean(),
                                    1000.0 * offset.getStandardDeviation(),
                                    n,
                                    momentError);

      if (measured > threshold)
         return GateResult.Finding.fail(subject, measured, threshold, n, detail);

      return GateResult.Finding.pass(subject, measured, threshold, n, detail);
   }

   /** {@code sum(F_z) / g}: what the robot actually weighs, kg. */
   public double getImpliedTotalMass()
   {
      return verticalForce.getCount() == 0 ? Double.NaN : verticalForce.getMean() / gravity;
   }

   /** The bar a mean horizontal offset must clear to indict the model, metres. */
   public double getDistributionThreshold()
   {
      return sigmaMultiplier * expectedComUncertainty;
   }

   public double getMeanOffsetX()
   {
      return offsetX.getMean();
   }

   public double getMeanOffsetY()
   {
      return offsetY.getMean();
   }

   public long getQuasiStaticSampleCount()
   {
      return offsetX.getCount();
   }

   public long getSamplesRejectedAsDynamic()
   {
      return samplesRejectedAsDynamic;
   }

   /** Drops all accumulated statistics so the gate can be re-run over a different session. */
   public void reset()
   {
      offsetX.reset();
      offsetY.reset();
      verticalForce.reset();
      samplesOffered = 0;
      samplesRejectedAsDynamic = 0;
      samplesRejectedAsNaN = 0;
   }

   /**
    * Welford running mean and variance: single-pass, allocation-free, and numerically stable where
    * the naive sum-of-squares is not. These offsets are millimetres about a metre-scale coordinate,
    * which is exactly the regime that cancels catastrophically.
    */
   private static final class Statistics
   {
      private long count;
      private double mean;
      private double sumOfSquaredDeviations;

      private void accumulate(double value)
      {
         count++;
         double delta = value - mean;
         mean += delta / count;
         sumOfSquaredDeviations += delta * (value - mean);
      }

      private long getCount()
      {
         return count;
      }

      private double getMean()
      {
         return count == 0 ? Double.NaN : mean;
      }

      private double getStandardDeviation()
      {
         return count < 2 ? Double.NaN : Math.sqrt(sumOfSquaredDeviations / (count - 1));
      }

      private void reset()
      {
         count = 0;
         mean = 0.0;
         sumOfSquaredDeviations = 0.0;
      }
   }
}
