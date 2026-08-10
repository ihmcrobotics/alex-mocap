package us.ihmc.alexMocap.postprocess;

/**
 * A <b>centred</b> Savitzky-Golay first-derivative filter (FRAMEWORK.md §13).
 *
 * <h2>Why not just difference</h2>
 * <p>
 * Raw single differencing at 200 Hz with {@code σ = 0.93 mm} gives {@code √2·σ/Δt ≈ 0.13 m/s}, five
 * times the ContactNet baseline this pipeline exists to validate. A 0.1 s centred window brings
 * that to roughly 0.0037 m/s.
 * </p>
 *
 * <h2>Zero lag, and why it matters more than the noise reduction</h2>
 * <p>
 * §13 is emphatic: lag against an estimator being validated <i>for velocity</i> is a systematic
 * comparison error that reads exactly like estimator bias. A filter that is accurate but delayed
 * passes every accuracy test you would otherwise write.
 * </p>
 * <p>
 * This filter is zero-lag <b>by construction</b>, and the construction is worth spelling out
 * because it is also the whole implementation. Fit a degree-{@code d} polynomial to the
 * {@code 2m+1} samples by least squares and take its derivative at the centre. Over a symmetric
 * window the even and odd basis functions are orthogonal -- {@code sum_k k^a = 0} whenever
 * {@code a} is odd -- so the normal equations <b>decouple</b>, and the derivative at the centre
 * depends only on the odd part of the fit. Writing the odd powers as {@code p = 1, 3, 5, … ≤ d}:
 * </p>
 *
 * <pre>
 * M[a][b] = sum_k k^(p_a + p_b)          (the odd-basis normal matrix)
 * h[k]    = (1/Δt) · sum_a (M⁻¹)[0][a] · k^(p_a)
 * </pre>
 *
 * <p>
 * Every {@code p_a} is odd, so {@code h[-k] = -h[k]} exactly, and {@code h[0] = 0}. An
 * antisymmetric kernel has a purely imaginary frequency response, which is precisely the phase of
 * an ideal differentiator {@code jω} -- zero phase error at every frequency, not merely at low
 * ones. The antisymmetry is asserted in the constructor rather than trusted, since it <i>is</i> the
 * guarantee.
 * </p>
 * <p>
 * For {@code d = 1} or {@code d = 2} this collapses to {@code h[k] = k / (Δt · sum_k k²)}, the
 * ordinary least-squares slope. Degree 2 buys nothing over degree 1 for a first derivative -- the
 * quadratic term is even and cannot influence an odd coefficient -- which is why the useful choices
 * are 2 and 3.
 * </p>
 *
 * <h2>Edges are NaN, deliberately</h2>
 * <p>
 * A centred window has no value within {@code m} samples of either end. The tempting fix -- a
 * one-sided window there -- reintroduces exactly the lag this filter exists to avoid, at exactly
 * the samples nobody scrutinises. {@link #differentiate} writes NaN instead, which propagates
 * visibly and plots as a gap.
 * </p>
 * <p>
 * This is also why it is a post-processing class and not a runtime one: it needs samples from the
 * future, so it cannot execute causally at all. See {@code runtime.PelvisStateExtractor}.
 * </p>
 */
public class SGDifferentiator
{
   private final int halfWindow;
   private final int polynomialOrder;
   private final double samplePeriod;
   private final double[] coefficients;

   /**
    * @param halfWindow      {@code m}; the window spans {@code 2m+1} samples.
    * @param polynomialOrder {@code d}, at least 1 and less than {@code 2m+1}.
    * @param samplePeriod    {@code Δt}, seconds.
    */
   public SGDifferentiator(int halfWindow, int polynomialOrder, double samplePeriod)
   {
      if (halfWindow < 1)
         throw new IllegalArgumentException("The half-window must be at least 1, was " + halfWindow + ".");
      if (polynomialOrder < 1)
         throw new IllegalArgumentException("The polynomial order must be at least 1 to have a derivative, was " + polynomialOrder + ".");
      if (polynomialOrder >= 2 * halfWindow + 1)
         throw new IllegalArgumentException("A degree-" + polynomialOrder + " polynomial through " + (2 * halfWindow + 1)
               + " samples is not a fit; it interpolates, and differentiating it amplifies noise without bound.");
      if (!(samplePeriod > 0.0))
         throw new IllegalArgumentException("The sample period must be positive, was " + samplePeriod + ".");

      this.halfWindow = halfWindow;
      this.polynomialOrder = polynomialOrder;
      this.samplePeriod = samplePeriod;
      this.coefficients = computeCoefficients(halfWindow, polynomialOrder, samplePeriod);

      // The antisymmetry IS the zero-lag guarantee, so it is checked rather than assumed.
      for (int k = 1; k <= halfWindow; k++)
      {
         double sum = coefficients[halfWindow + k] + coefficients[halfWindow - k];

         if (Math.abs(sum) > 1.0e-9 * Math.abs(coefficients[halfWindow + k]) + 1.0e-15)
            throw new IllegalStateException("The kernel is not antisymmetric at k=" + k + " (h[+k] + h[-k] = " + sum + "), so it is not zero-lag.");
      }
   }

   /** A 0.1 s centred window at the given rate, quadratic. FRAMEWORK.md §13's worked example. */
   public static SGDifferentiator centredWindow(double windowSeconds, double sampleRateHz)
   {
      double samplePeriod = 1.0 / sampleRateHz;
      int halfWindow = Math.max(1, (int) Math.round(0.5 * windowSeconds * sampleRateHz));
      return new SGDifferentiator(halfWindow, 2, samplePeriod);
   }

   private static double[] computeCoefficients(int halfWindow, int polynomialOrder, double samplePeriod)
   {
      // Odd powers only: the even ones cannot influence the derivative at the centre of a
      // symmetric window.
      int oddCount = (polynomialOrder + 1) / 2;
      int[] powers = new int[oddCount];

      for (int a = 0; a < oddCount; a++)
         powers[a] = 2 * a + 1;

      double[][] normal = new double[oddCount][oddCount];

      for (int a = 0; a < oddCount; a++)
      {
         for (int b = 0; b < oddCount; b++)
         {
            double sum = 0.0;

            for (int k = -halfWindow; k <= halfWindow; k++)
               sum += Math.pow(k, powers[a] + powers[b]);

            normal[a][b] = sum;
         }
      }

      // We need only the first row of M⁻¹, so solve Mᵀ x = e₀. M is symmetric, so this is M x = e₀.
      double[] firstRowOfInverse = solve(normal, oddCount);

      double[] coefficients = new double[2 * halfWindow + 1];

      for (int k = -halfWindow; k <= halfWindow; k++)
      {
         double value = 0.0;

         for (int a = 0; a < oddCount; a++)
            value += firstRowOfInverse[a] * Math.pow(k, powers[a]);

         coefficients[halfWindow + k] = value / samplePeriod;
      }

      return coefficients;
   }

   /**
    * Gaussian elimination with partial pivoting for {@code M x = e₀}.
    * <p>
    * Hand-rolled rather than pulled from EJML because the system is at most 3×3 for any sane
    * polynomial order, and because FRAMEWORK.md §2 wants exactly one SVD in the project -- keeping
    * a linear-algebra dependency out of {@code postprocess} keeps that claim easy to check.
    * </p>
    */
   private static double[] solve(double[][] matrix, int size)
   {
      double[][] augmented = new double[size][size + 1];

      for (int i = 0; i < size; i++)
      {
         System.arraycopy(matrix[i], 0, augmented[i], 0, size);
         augmented[i][size] = i == 0 ? 1.0 : 0.0;
      }

      for (int column = 0; column < size; column++)
      {
         int pivot = column;

         for (int row = column + 1; row < size; row++)
         {
            if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column]))
               pivot = row;
         }

         double[] swap = augmented[column];
         augmented[column] = augmented[pivot];
         augmented[pivot] = swap;

         if (Math.abs(augmented[column][column]) < 1.0e-300)
            throw new IllegalStateException("The Savitzky-Golay normal matrix is singular; the window is too short for this polynomial order.");

         for (int row = 0; row < size; row++)
         {
            if (row == column)
               continue;

            double factor = augmented[row][column] / augmented[column][column];

            for (int c = column; c <= size; c++)
               augmented[row][c] -= factor * augmented[column][c];
         }
      }

      double[] solution = new double[size];

      for (int i = 0; i < size; i++)
         solution[i] = augmented[i][size] / augmented[i][i];

      return solution;
   }

   /**
    * The convolution kernel, indexed {@code 0 … 2m}, in units of 1/second.
    * <p>
    * Antisymmetric about its centre, which is where the zero-lag property lives.
    * </p>
    */
   public double[] getCoefficients()
   {
      return coefficients.clone();
   }

   public int getHalfWindow()
   {
      return halfWindow;
   }

   public int getWindowLength()
   {
      return 2 * halfWindow + 1;
   }

   public double getWindowSeconds()
   {
      return getWindowLength() * samplePeriod;
   }

   public int getPolynomialOrder()
   {
      return polynomialOrder;
   }

   public double getSamplePeriod()
   {
      return samplePeriod;
   }

   /**
    * The derivative at one sample. The caller must keep the index at least {@code m} from either
    * end.
    */
   public double differentiateAt(double[] signal, int index)
   {
      if (index < halfWindow || index >= signal.length - halfWindow)
         throw new IndexOutOfBoundsException("Index " + index + " is within " + halfWindow + " samples of an end of a " + signal.length
               + "-sample signal; a centred window has no value there.");

      double derivative = 0.0;

      for (int k = -halfWindow; k <= halfWindow; k++)
         derivative += coefficients[halfWindow + k] * signal[index + k];

      return derivative;
   }

   /**
    * Differentiates a whole signal, writing NaN into the {@code m} samples at each end.
    *
    * @param derivativeToPack same length as {@code signal}.
    */
   public void differentiate(double[] signal, double[] derivativeToPack)
   {
      if (derivativeToPack.length != signal.length)
         throw new IllegalArgumentException("Output length " + derivativeToPack.length + " does not match input length " + signal.length + ".");

      if (signal.length < getWindowLength())
         throw new IllegalArgumentException("A " + signal.length + "-sample signal is shorter than the " + getWindowLength()
               + "-sample window; there is nothing this filter can say about it.");

      for (int i = 0; i < halfWindow; i++)
      {
         derivativeToPack[i] = Double.NaN;
         derivativeToPack[signal.length - 1 - i] = Double.NaN;
      }

      for (int i = halfWindow; i < signal.length - halfWindow; i++)
         derivativeToPack[i] = differentiateAt(signal, i);
   }

   /**
    * The noise gain: the factor by which independent per-sample noise of standard deviation
    * {@code σ} is multiplied to give the derivative's noise, {@code sqrt(sum_k h[k]²)}.
    * <p>
    * Multiply by {@code σ} for the expected velocity noise. At a 0.1 s window, 200 Hz and
    * {@code σ = 0.93 mm} this reproduces §13's ~0.0037 m/s, and it is how to choose a window
    * without guessing.
    * </p>
    */
   public double getNoiseGain()
   {
      double sumOfSquares = 0.0;

      for (double coefficient : coefficients)
         sumOfSquares += coefficient * coefficient;

      return Math.sqrt(sumOfSquares);
   }

   @Override
   public String toString()
   {
      return String.format("SGDifferentiator[%d samples (%.3f s), degree %d, noise gain %.2f /s]",
                           getWindowLength(),
                           getWindowSeconds(),
                           polynomialOrder,
                           getNoiseGain());
   }
}
