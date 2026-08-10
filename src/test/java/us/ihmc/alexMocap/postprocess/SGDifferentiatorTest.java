package us.ihmc.alexMocap.postprocess;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

/**
 * F10's velocity half (FRAMEWORK.md §13).
 * <p>
 * The zero-lag test is the one that matters. Lag against an estimator being validated for velocity
 * is a systematic comparison error that reads exactly like estimator bias, and a filter that is
 * accurate but delayed passes every accuracy test one would otherwise write -- so accuracy and lag
 * are checked separately, and the lag test includes a <b>causal filter of equal width as a
 * control</b>. Without that control the zero-lag assertion could pass for a trivial reason.
 * </p>
 */
public class SGDifferentiatorTest
{
   private static final double SAMPLE_RATE = 200.0;
   private static final double SIGMA_MARKER = 0.93e-3;

   /**
    * The noise reaching the differentiator is the <b>pelvis pose</b> noise, not the raw marker
    * noise.
    * <p>
    * F6 registers four pelvis markers to get one pose, so its translational noise is roughly
    * {@code σ/√N}. FRAMEWORK.md §17's 0.0037 m/s figure only comes out if that is what is fed in:
    * with the raw 0.93 mm the same window gives 0.0067 m/s. This is worth stating because the
    * distinction is invisible in §13's prose and is a factor of two in the headline number.
    * </p>
    */
   private static final double SIGMA_PELVIS_POSE = SIGMA_MARKER / Math.sqrt(4.0);

   /** A pelvis swinging at 0.5 Hz with 0.1 m amplitude: a plausible gantry sway. */
   private static double position(double time)
   {
      return 0.1 * Math.sin(2.0 * Math.PI * 0.5 * time);
   }

   private static double velocity(double time)
   {
      return 0.1 * 2.0 * Math.PI * 0.5 * Math.cos(2.0 * Math.PI * 0.5 * time);
   }

   /** The kernel is antisymmetric, which is the whole zero-lag argument. */
   @Test
   public void testKernelIsAntisymmetricAndSumsToZero()
   {
      for (int order : new int[] {1, 2, 3, 4, 5})
      {
         SGDifferentiator differentiator = new SGDifferentiator(10, order, 1.0 / SAMPLE_RATE);
         double[] h = differentiator.getCoefficients();

         double sum = 0.0;

         for (int i = 0; i < h.length; i++)
         {
            assertEquals(-h[h.length - 1 - i], h[i], 1.0e-12, "order " + order + ", tap " + i);
            sum += h[i];
         }

         assertEquals(0.0, h[differentiator.getHalfWindow()], 1.0e-15, "The centre tap of a first-derivative kernel is zero.");
         assertEquals(0.0, sum, 1.0e-12, "An antisymmetric kernel sums to zero, so a constant differentiates to zero.");
      }
   }

   /** A first-derivative filter must return a polynomial's exact derivative up to its own degree. */
   @Test
   public void testExactOnPolynomialsUpToItsOrder()
   {
      double dt = 1.0 / SAMPLE_RATE;

      for (int order : new int[] {2, 3, 4, 5})
      {
         SGDifferentiator differentiator = new SGDifferentiator(12, order, dt);

         double[] signal = new double[64];
         double[] derivative = new double[64];

         // A polynomial of exactly the filter's degree.
         for (int i = 0; i < signal.length; i++)
         {
            double t = i * dt;
            signal[i] = Math.pow(t, order);
         }

         differentiator.differentiate(signal, derivative);

         for (int i = differentiator.getHalfWindow(); i < signal.length - differentiator.getHalfWindow(); i++)
         {
            double t = i * dt;
            double expected = order * Math.pow(t, order - 1);
            assertEquals(expected, derivative[i], 1.0e-6 * Math.max(1.0, Math.abs(expected)), "order " + order + ", sample " + i);
         }
      }
   }

   /** For degree 1 and 2 the kernel is the ordinary least-squares slope. */
   @Test
   public void testDegreeTwoIsTheLeastSquaresSlope()
   {
      int halfWindow = 10;
      double dt = 1.0 / SAMPLE_RATE;
      SGDifferentiator differentiator = new SGDifferentiator(halfWindow, 2, dt);

      double sumOfSquares = 0.0;

      for (int k = -halfWindow; k <= halfWindow; k++)
         sumOfSquares += k * k;

      double[] h = differentiator.getCoefficients();

      for (int k = -halfWindow; k <= halfWindow; k++)
         assertEquals(k / (dt * sumOfSquares), h[halfWindow + k], 1.0e-15, "tap " + k);

      // Degree 1 gives the same kernel: the quadratic term is even and cannot touch an odd one.
      assertArrayEqualsWithin(new SGDifferentiator(halfWindow, 1, dt).getCoefficients(), h, 1.0e-15);
   }

   private static void assertArrayEqualsWithin(double[] expected, double[] actual, double tolerance)
   {
      assertEquals(expected.length, actual.length);

      for (int i = 0; i < expected.length; i++)
         assertEquals(expected[i], actual[i], tolerance, "index " + i);
   }

   /**
    * PR_PLAN.md: analytic sinusoid plus noise at 200 Hz; recovered velocity RMS under 0.005 m/s for
    * a 0.1 s centred window.
    */
   @Test
   public void testVelocityAccuracyUnderRealisticNoise()
   {
      Random random = new Random(20260809L);
      SGDifferentiator differentiator = SGDifferentiator.centredWindow(0.1, SAMPLE_RATE);

      int samples = 2000;
      double dt = 1.0 / SAMPLE_RATE;
      double[] signal = new double[samples];

      for (int i = 0; i < samples; i++)
         signal[i] = position(i * dt) + SIGMA_PELVIS_POSE * random.nextGaussian();

      double[] derivative = new double[samples];
      differentiator.differentiate(signal, derivative);

      double sumOfSquares = 0.0;
      int counted = 0;

      for (int i = differentiator.getHalfWindow(); i < samples - differentiator.getHalfWindow(); i++)
      {
         double error = derivative[i] - velocity(i * dt);
         sumOfSquares += error * error;
         counted++;
      }

      double rms = Math.sqrt(sumOfSquares / counted);

      assertTrue(rms < 0.005, "Velocity RMS " + rms + " m/s exceeds 0.005 m/s.");

      // And it should land near the predicted noise gain times the input noise, which is how a
      // window gets chosen rather than guessed.
      double predicted = differentiator.getNoiseGain() * SIGMA_PELVIS_POSE;
      assertEquals(predicted, rms, 0.4 * predicted, "Measured RMS should track the filter's own noise-gain prediction.");
   }

   /** The raw-marker figure, for contrast: the same window is 1.8× worse if fed marker noise. */
   @Test
   public void testRawMarkerNoiseWouldNotMeetTheTarget()
   {
      SGDifferentiator differentiator = SGDifferentiator.centredWindow(0.1, SAMPLE_RATE);

      double fromPoseNoise = differentiator.getNoiseGain() * SIGMA_PELVIS_POSE;
      double fromMarkerNoise = differentiator.getNoiseGain() * SIGMA_MARKER;

      assertTrue(fromPoseNoise < 0.005, "Fed pelvis-pose noise, the 0.1 s window meets the target: " + fromPoseNoise);
      assertTrue(fromMarkerNoise > 0.005,
                 "Fed raw marker noise it does not (" + fromMarkerNoise + "), which is why §17's 0.0037 m/s only makes sense as a pose-noise figure.");
      assertEquals(2.0, fromMarkerNoise / fromPoseNoise, 1.0e-9, "The ratio is √N for the four pelvis markers.");
   }

   /**
    * <b>Zero lag, with a causal filter of equal width as the control.</b>
    * <p>
    * The centred kernel's cross-correlation with the true velocity must peak at exactly zero lag.
    * The control is a causal backward difference spanning the same {@code 2m+1} samples: it is a
    * perfectly reasonable differentiator, it is <i>accurate</i>, and it peaks at a lag of {@code m}
    * samples. Without it, "peaks at zero" could pass for a trivial reason and nobody would know.
    * </p>
    * <p>
    * At 200 Hz with a 0.1 s window that control lag is 10 samples, i.e. 50 ms. Against a pelvis
    * moving at 0.3 m/s that is 15 mm of apparent position error, and against an EKF it would read
    * as a bias.
    * </p>
    */
   @Test
   public void testZeroLagAndThatACausalFilterOfEqualWidthIsNot()
   {
      SGDifferentiator differentiator = SGDifferentiator.centredWindow(0.1, SAMPLE_RATE);
      int halfWindow = differentiator.getHalfWindow();
      int samples = 4000;
      double dt = 1.0 / SAMPLE_RATE;

      double[] signal = new double[samples];
      double[] truth = new double[samples];

      for (int i = 0; i < samples; i++)
      {
         signal[i] = position(i * dt);
         truth[i] = velocity(i * dt);
      }

      double[] centred = new double[samples];
      differentiator.differentiate(signal, centred);

      // The control: causal, same width, only past samples.
      double[] causal = new double[samples];
      java.util.Arrays.fill(causal, Double.NaN);

      for (int i = 2 * halfWindow; i < samples; i++)
         causal[i] = (signal[i] - signal[i - 2 * halfWindow]) / (2 * halfWindow * dt);

      assertEquals(0, peakCorrelationLag(truth, centred, 2 * halfWindow), "The centred filter must peak at zero lag.");

      int causalLag = peakCorrelationLag(truth, causal, 2 * halfWindow);
      assertTrue(causalLag > 0, "The causal control must show a positive lag, or it is not a control. Got " + causalLag);
      assertEquals(halfWindow, causalLag, 1, "A backward difference over 2m samples has a group delay of m samples.");
   }

   /**
    * The lag at which {@code estimate} best matches {@code truth}, in samples.
    * <p>
    * A positive result means the estimate is <i>delayed</i>: {@code estimate[i + lag]} lines up with
    * {@code truth[i]}.
    * </p>
    */
   private static int peakCorrelationLag(double[] truth, double[] estimate, int maximumLag)
   {
      int bestLag = 0;
      double bestCorrelation = Double.NEGATIVE_INFINITY;

      for (int lag = -maximumLag; lag <= maximumLag; lag++)
      {
         double sum = 0.0;
         int counted = 0;

         for (int i = maximumLag; i < truth.length - maximumLag; i++)
         {
            double value = estimate[i + lag];

            if (Double.isNaN(value))
               continue;

            sum += truth[i] * value;
            counted++;
         }

         if (counted == 0)
            continue;

         double correlation = sum / counted;

         if (correlation > bestCorrelation)
         {
            bestCorrelation = correlation;
            bestLag = lag;
         }
      }

      return bestLag;
   }

   /** Edges are NaN, not one-sided: a one-sided edge would reintroduce the lag this filter avoids. */
   @Test
   public void testEdgesAreNaN()
   {
      SGDifferentiator differentiator = new SGDifferentiator(5, 2, 1.0 / SAMPLE_RATE);
      double[] signal = new double[40];
      double[] derivative = new double[40];

      for (int i = 0; i < signal.length; i++)
         signal[i] = i;

      differentiator.differentiate(signal, derivative);

      for (int i = 0; i < 5; i++)
      {
         assertTrue(Double.isNaN(derivative[i]), "leading edge " + i);
         assertTrue(Double.isNaN(derivative[signal.length - 1 - i]), "trailing edge " + i);
      }

      for (int i = 5; i < signal.length - 5; i++)
         assertEquals(SAMPLE_RATE, derivative[i], 1.0e-9, "interior " + i);

      assertThrows(IndexOutOfBoundsException.class, () -> differentiator.differentiateAt(signal, 2));
   }

   @Test
   public void testDegenerateConfigurationsAreRejected()
   {
      double dt = 1.0 / SAMPLE_RATE;

      assertThrows(IllegalArgumentException.class, () -> new SGDifferentiator(0, 2, dt));
      assertThrows(IllegalArgumentException.class, () -> new SGDifferentiator(5, 0, dt));
      assertThrows(IllegalArgumentException.class, () -> new SGDifferentiator(5, 2, 0.0));
      // Degree 11 through 11 samples interpolates rather than fits.
      assertThrows(IllegalArgumentException.class, () -> new SGDifferentiator(5, 11, dt));
      // Shorter than the window.
      assertThrows(IllegalArgumentException.class, () -> new SGDifferentiator(5, 2, dt).differentiate(new double[8], new double[8]));
   }
}
