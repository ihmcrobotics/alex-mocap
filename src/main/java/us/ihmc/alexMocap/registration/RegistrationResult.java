package us.ihmc.alexMocap.registration;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Output of one {@link RigidBodyRegistration} solve: the recovered pose, plus the conditioning
 * numbers a caller needs in order to decide whether to believe it.
 * <p>
 * This type reports numbers and nothing else. It carries no threshold, no pass/fail flag derived
 * from {@code sigma3}, and no residual policy -- see FRAMEWORK.md §2 and §9. Gates and estimators
 * decide; the primitive measures. {@link #wasSuccessful()} is the one boolean here and it means
 * only "the solve ran with enough correspondences to produce a pose", never "the pose is good".
 * </p>
 * <p>
 * Mutable and reusable by design. {@link RigidBodyRegistration} packs into a caller-owned
 * instance so that a per-frame runtime loop allocates nothing.
 * </p>
 *
 * @see RigidBodyRegistration
 */
public class RegistrationResult
{
   /**
    * The recovered transform, mapping <b>source</b> points into the <b>target</b> frame.
    * <p>
    * For F6 (FRAMEWORK.md §9) the source points are the calibrated layout {@code ^i p̂_ij} in link
    * frame {@code i} and the targets are the measured world positions {@code ^W m_ij}, so this
    * transform is {@code ^W T̂_i}.
    * </p>
    */
   private final RigidBodyTransform transform = new RigidBodyTransform();

   /**
    * Singular values of the normalised cross-covariance {@code H}, in descending order:
    * {@code sigma1 >= sigma2 >= sigma3 >= 0}.
    * <p>
    * Because {@code H} is normalised by the correspondence count before decomposition, these are
    * mean-squared spreads with units of length², directly comparable between frames that saw
    * different numbers of markers (FRAMEWORK.md §2).
    * </p>
    */
   private double sigma1 = Double.NaN;
   private double sigma2 = Double.NaN;
   private double sigma3 = Double.NaN;

   /** Number of correspondences the solve consumed. For F6 this is the visible marker count. */
   private int correspondenceCount = 0;

   /** Whether enough correspondences were supplied to produce a pose at all. */
   private boolean successful = false;

   /**
    * Whether the raw {@code U Vᵀ} came out with {@code det = -1} and the Umeyama determinant
    * factor had to flip the third column to keep {@code R} in SO(3).
    * <p>
    * Exposed as a diagnostic, not as a fault: on a near-planar cluster with noise -- the realistic
    * case, markers on a flat link face -- a reflection is the expected coin flip and the
    * correction is exactly the reason Umeyama is used over Arun (FRAMEWORK.md §2). It is
    * observable here so that the guard can be seen firing rather than merely assumed to work.
    * </p>
    */
   private boolean reflectionCorrected = false;

   /**
    * Resets to the failed state: NaN transform, NaN singular values, zero correspondences.
    * <p>
    * The transform is set to NaN rather than identity deliberately. A caller that ignores
    * {@link #wasSuccessful()} and uses the pose anyway gets NaN propagating visibly downstream
    * instead of a plausible-looking identity pose.
    * </p>
    */
   public void setToNaN()
   {
      transform.setToNaN();
      sigma1 = Double.NaN;
      sigma2 = Double.NaN;
      sigma3 = Double.NaN;
      correspondenceCount = 0;
      successful = false;
      reflectionCorrected = false;
   }

   public RigidBodyTransform getTransform()
   {
      return transform;
   }

   public double getSigma1()
   {
      return sigma1;
   }

   public double getSigma2()
   {
      return sigma2;
   }

   public double getSigma3()
   {
      return sigma3;
   }

   public int getCorrespondenceCount()
   {
      return correspondenceCount;
   }

   public boolean wasSuccessful()
   {
      return successful;
   }

   public boolean wasReflectionCorrected()
   {
      return reflectionCorrected;
   }

   void setSingularValues(double sigma1, double sigma2, double sigma3)
   {
      this.sigma1 = sigma1;
      this.sigma2 = sigma2;
      this.sigma3 = sigma3;
   }

   void setCorrespondenceCount(int correspondenceCount)
   {
      this.correspondenceCount = correspondenceCount;
   }

   void setSuccessful(boolean successful)
   {
      this.successful = successful;
   }

   void setReflectionCorrected(boolean reflectionCorrected)
   {
      this.reflectionCorrected = reflectionCorrected;
   }
}
