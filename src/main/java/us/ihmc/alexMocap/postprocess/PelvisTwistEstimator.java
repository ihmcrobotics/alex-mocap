package us.ihmc.alexMocap.postprocess;

import java.util.List;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * F10's velocity half, run <b>offline as a second pass</b> over a logged pose trajectory
 * (FRAMEWORK.md §13).
 * <p>
 * This is the only place in the project that produces a pelvis velocity, and it lives in
 * {@code postprocess} rather than {@code runtime} because a centred window needs samples from the
 * future and cannot execute causally. {@code runtime.PelvisStateExtractor} deliberately exposes no
 * velocity accessor at all; this class is what fills that gap, after the fact, from the log.
 * </p>
 *
 * <h2>Angular velocity from the rotation matrices</h2>
 * <p>
 * Linear velocity is the differentiator applied to the pelvis origin, one axis at a time. Angular
 * velocity uses
 * </p>
 *
 * <pre>
 * ω̂ = Ṙ Rᵀ        ω = vee(ω̂)
 * </pre>
 *
 * <p>
 * with {@code Ṙ} obtained by running the same filter over each of the nine entries of {@code R}
 * independently. Two remarks on why this rather than differentiating an angle parameterisation.
 * Euler angles and quaternions both need unwrapping, and an unwrapping bug produces a single
 * enormous velocity spike that is easy to mistake for a real event. And {@code Ṙ Rᵀ} is exactly
 * antisymmetric only when {@code R} is exactly a rotation; filtering the entries independently
 * breaks that slightly, so the skew part is taken -- {@code (A - Aᵀ)/2} -- which is the
 * least-squares antisymmetric approximation and degrades gracefully instead of drifting.
 * </p>
 * <p>
 * The residual symmetric part is worth watching, and {@link #getWorstNonSkewResidual()} reports it.
 * Large values mean the pose log is noisy enough that the rotations are not staying on SO(3)
 * through the filter, which is a data problem rather than a filter problem.
 * </p>
 *
 * <h2>Edges are NaN</h2>
 * <p>
 * The first and last {@code m} samples have no centred window and are NaN, not one-sided. See
 * {@link SGDifferentiator}.
 * </p>
 */
public class PelvisTwistEstimator
{
   private final SGDifferentiator differentiator;

   private double[][] linearVelocity;
   private double[][] angularVelocity;
   private double worstNonSkewResidual = Double.NaN;

   public PelvisTwistEstimator(SGDifferentiator differentiator)
   {
      this.differentiator = differentiator;
   }

   /** FRAMEWORK.md §13's recommendation: a 0.1 s centred window at the runtime rate. */
   public static PelvisTwistEstimator withCentredWindow(double windowSeconds, double sampleRateHz)
   {
      return new PelvisTwistEstimator(SGDifferentiator.centredWindow(windowSeconds, sampleRateHz));
   }

   /**
    * Differentiates a logged pose trajectory.
    * <p>
    * Poses must be uniformly sampled at the differentiator's sample period, and expressed in a
    * single fixed frame -- {@code Wg}. Differentiating poses from mixed frames produces a velocity
    * that is wrong in a way no unit check catches.
    * </p>
    *
    * @param poses the trajectory, in order.
    */
   public void compute(List<? extends RigidBodyTransformReadOnly> poses)
   {
      int n = poses.size();

      linearVelocity = new double[3][n];
      angularVelocity = new double[3][n];

      double[] component = new double[n];
      double[] derivative = new double[n];

      // Linear: differentiate the origin, one axis at a time.
      for (int axis = 0; axis < 3; axis++)
      {
         for (int i = 0; i < n; i++)
            component[i] = poses.get(i).getTranslation().getElement(axis);

         differentiator.differentiate(component, derivative);
         System.arraycopy(derivative, 0, linearVelocity[axis], 0, n);
      }

      // Angular: differentiate all nine rotation entries, then take vee(skew(Ṙ Rᵀ)).
      //
      // The orientations are materialised as rotation matrices once, up front, rather than
      // re-read nine times: RigidBodyTransformReadOnly.getRotation() returns an
      // Orientation3DReadOnly with no element access, and converting inside the entry loop would
      // do the same conversion nine times per sample.
      RotationMatrix[] rotations = new RotationMatrix[n];

      for (int i = 0; i < n; i++)
      {
         rotations[i] = new RotationMatrix();
         rotations[i].set(poses.get(i).getRotation());
      }

      double[][] rotationDot = new double[9][n];

      for (int entry = 0; entry < 9; entry++)
      {
         for (int i = 0; i < n; i++)
            component[i] = rotations[i].getElement(entry / 3, entry % 3);

         differentiator.differentiate(component, derivative);
         System.arraycopy(derivative, 0, rotationDot[entry], 0, n);
      }

      worstNonSkewResidual = 0.0;
      int halfWindow = differentiator.getHalfWindow();

      for (int i = 0; i < n; i++)
      {
         if (i < halfWindow || i >= n - halfWindow)
         {
            angularVelocity[0][i] = Double.NaN;
            angularVelocity[1][i] = Double.NaN;
            angularVelocity[2][i] = Double.NaN;
            continue;
         }

         RotationMatrix rotation = rotations[i];
         double[] omegaHat = new double[9];

         // ω̂ = Ṙ Rᵀ
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double sum = 0.0;

               for (int k = 0; k < 3; k++)
                  sum += rotationDot[3 * row + k][i] * rotation.getElement(column, k);

               omegaHat[3 * row + column] = sum;
            }
         }

         // Skew part: (A - Aᵀ)/2. The symmetric remainder is the departure from SO(3).
         double symmetric = 0.0;

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               symmetric = Math.max(symmetric, Math.abs(omegaHat[3 * row + column] + omegaHat[3 * column + row]));
         }

         worstNonSkewResidual = Math.max(worstNonSkewResidual, 0.5 * symmetric);

         angularVelocity[0][i] = 0.5 * (omegaHat[7] - omegaHat[5]);
         angularVelocity[1][i] = 0.5 * (omegaHat[2] - omegaHat[6]);
         angularVelocity[2][i] = 0.5 * (omegaHat[3] - omegaHat[1]);
      }
   }

   /** Convenience for a trajectory held as an array. */
   public void compute(RigidBodyTransform[] poses)
   {
      compute(List.of(poses));
   }

   /** Linear velocity of the pelvis origin at sample {@code i}, m/s. NaN at the edges. */
   public void packLinearVelocity(int index, Vector3D toPack)
   {
      toPack.set(linearVelocity[0][index], linearVelocity[1][index], linearVelocity[2][index]);
   }

   /** Angular velocity at sample {@code i}, rad/s. NaN at the edges. */
   public void packAngularVelocity(int index, Vector3D toPack)
   {
      toPack.set(angularVelocity[0][index], angularVelocity[1][index], angularVelocity[2][index]);
   }

   public double getLinearVelocity(int axis, int index)
   {
      return linearVelocity[axis][index];
   }

   public double getAngularVelocity(int axis, int index)
   {
      return angularVelocity[axis][index];
   }

   public int getSampleCount()
   {
      return linearVelocity == null ? 0 : linearVelocity[0].length;
   }

   /** Samples the centred window could not reach, at each end. */
   public int getEdgeSampleCount()
   {
      return differentiator.getHalfWindow();
   }

   /**
    * The largest symmetric residual of {@code Ṙ Rᵀ} over the trajectory, 1/s. Should be small
    * against the angular velocities themselves; if it is not, the pose log is too noisy for the
    * rotations to survive filtering as rotations.
    */
   public double getWorstNonSkewResidual()
   {
      return worstNonSkewResidual;
   }

   public SGDifferentiator getDifferentiator()
   {
      return differentiator;
   }

   /**
    * Expected velocity noise given the per-sample position noise, m/s: the filter's noise gain
    * times {@code σ}. Compare against the ContactNet baselines before trusting a comparison.
    */
   public double getExpectedVelocityNoise(double positionNoiseStandardDeviation)
   {
      return differentiator.getNoiseGain() * positionNoiseStandardDeviation;
   }
}
