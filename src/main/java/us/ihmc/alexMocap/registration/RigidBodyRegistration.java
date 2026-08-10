package us.ihmc.alexMocap.registration;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * The registration primitive of FRAMEWORK.md §2: the closed-form least-squares rigid transform
 * between two sets of corresponding points.
 * <p>
 * Given {@code L} correspondences between source points <tt>{a_l}</tt> and target points
 * <tt>{b_l}</tt>, this minimises <tt>sum_l ||b_l - (R a_l + t)||²</tt> over
 * <tt>R ∈ SO(3), t ∈ R³</tt>:
 * </p>
 *
 * <pre>
 * H = (1/L) · sum_l (b_l - b̄)(a_l - ā)ᵀ  =  U Σ Vᵀ
 * R = U · diag(1, 1, det(U Vᵀ)) · Vᵀ
 * t = b̄ - R ā
 * </pre>
 *
 * <h2>Three decisions worth defending</h2>
 * <ul>
 * <li><b>Umeyama, not Arun -- the determinant factor.</b> Without it, a noisy or near-planar
 * cluster (markers on a flat link face: the realistic case) can drive the raw <tt>U Vᵀ</tt> to
 * <tt>det = -1</tt>, a reflection. That is not a rotation and downstream code propagates it
 * without complaint. The fix costs one determinant, and when it fires it is reported via
 * {@link RegistrationResult#wasReflectionCorrected()}.</li>
 *
 * <li><b>{@code H} is normalised by {@code L} before decomposition.</b> Scaling by a positive
 * constant leaves {@code U} and {@code V} untouched, so {@code R} and {@code t} are unchanged --
 * but it turns the singular values into mean-squared spreads with units of length², comparable
 * across frames that saw different numbers of markers. Without it, {@code σ₃} drops when a marker
 * occludes for reasons that have nothing to do with geometry, and the conditioning monitor of
 * FRAMEWORK.md §9 reports an artifact of the visible count.</li>
 *
 * <li><b>The singular values are sorted explicitly.</b> EJML does not order them. Without a
 * descending sort, {@code σ₃} is whichever singular value landed third and the rank-deficiency
 * detector of FRAMEWORK.md §18.1 is noise.</li>
 * </ul>
 *
 * <h2>Contract</h2>
 * <p>
 * This class is <b>stateful and not thread safe.</b> It owns preallocated EJML scratch and a
 * growable correspondence buffer. Use one instance per caller; do not share one across threads
 * and do not hand the same instance to two solvers.
 * </p>
 * <p>
 * It is <b>allocation-free in steady state.</b> {@link #addCorrespondence} grows its backing
 * arrays by doubling, which allocates; once a caller has reached its working correspondence count
 * -- or has been constructed with the right capacity up front -- neither
 * {@code addCorrespondence} nor {@link #compute} allocates. This is asserted by a test, not
 * merely intended.
 * </p>
 * <p>
 * It reports numbers and makes no decisions. There is no {@code σ₃} threshold and no residual
 * policy here; refusal is the job of the runtime estimator and the gates (FRAMEWORK.md §9, §15).
 * </p>
 *
 * <h2>Usage</h2>
 *
 * <pre>
 * registration.clear();
 * for (int j = 0; j &lt; markerCount; j++)
 *    if (visible[j])
 *       registration.addCorrespondence(layoutInLinkFrame[j], measuredInWorld[j]);
 *
 * if (registration.compute(result))
 *    ... // result.getTransform() is ^W T̂_i; result.getSigma3() says whether to believe it
 * </pre>
 *
 * <p>
 * Occlusion needs no special handling -- an unseen marker is simply a correspondence that is
 * never added. F5 (FRAMEWORK.md §7) uses the same accumulation to stack every
 * <tt>(link, marker, capture)</tt> triple into a single Procrustes solve.
 * </p>
 *
 * @see RegistrationResult
 */
public class RigidBodyRegistration
{
   /**
    * Three non-collinear correspondences are the algebraic minimum for a 6-DOF pose. Note that
    * "non-collinear" is not checked here -- three collinear markers yield a perfectly well-formed
    * rotation with {@code σ₂ = σ₃ = 0}, which is the silent failure of FRAMEWORK.md §18.1. Callers
    * detect it from {@link RegistrationResult#getSigma3()}, not from the return of
    * {@link #compute}.
    */
   public static final int MINIMUM_CORRESPONDENCES = 3;

   private static final int DEFAULT_INITIAL_CAPACITY = 16;

   /** Interleaved xyz, {@code 3 * correspondenceCount} entries live. */
   private double[] source;
   private double[] target;
   private int correspondenceCount = 0;

   private final SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(3, 3, true, true, true);
   private final DMatrixRMaj crossCovariance = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj U = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj V = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj rotation = new DMatrixRMaj(3, 3);
   private final double[] singularValues = new double[3];

   public RigidBodyRegistration()
   {
      this(DEFAULT_INITIAL_CAPACITY);
   }

   /**
    * @param initialCapacity number of correspondences to preallocate for. Size this to the
    *                        caller's worst case -- total markers on a cluster for F6, or
    *                        {@code links × markers × captures} for F5 -- and the solve never
    *                        allocates.
    */
   public RigidBodyRegistration(int initialCapacity)
   {
      if (initialCapacity < MINIMUM_CORRESPONDENCES)
         throw new IllegalArgumentException("initialCapacity must be at least " + MINIMUM_CORRESPONDENCES + ", was " + initialCapacity);

      source = new double[3 * initialCapacity];
      target = new double[3 * initialCapacity];
   }

   /** Drops all accumulated correspondences. Retains the allocated capacity. */
   public void clear()
   {
      correspondenceCount = 0;
   }

   /**
    * Adds one correspondence. The two points must be expressed in the source and target frames
    * respectively; this class has no frame awareness and will not catch a mismatch.
    */
   public void addCorrespondence(Point3DReadOnly sourcePoint, Point3DReadOnly targetPoint)
   {
      addCorrespondence(sourcePoint.getX(),
                        sourcePoint.getY(),
                        sourcePoint.getZ(),
                        targetPoint.getX(),
                        targetPoint.getY(),
                        targetPoint.getZ());
   }

   public void addCorrespondence(double sourceX, double sourceY, double sourceZ, double targetX, double targetY, double targetZ)
   {
      ensureCapacity(correspondenceCount + 1);

      int i = 3 * correspondenceCount;
      source[i] = sourceX;
      source[i + 1] = sourceY;
      source[i + 2] = sourceZ;
      target[i] = targetX;
      target[i + 1] = targetY;
      target[i + 2] = targetZ;
      correspondenceCount++;
   }

   public int getCorrespondenceCount()
   {
      return correspondenceCount;
   }

   public int getCapacity()
   {
      return source.length / 3;
   }

   /**
    * Solves for the transform taking the accumulated source points into the target frame.
    *
    * @param resultToPack packed with the pose, the descending-sorted singular values of the
    *                     normalised {@code H}, and the correspondence count. Set entirely to NaN
    *                     if fewer than {@link #MINIMUM_CORRESPONDENCES} correspondences were
    *                     accumulated.
    * @return {@code true} if a pose was produced. This says only that the solve ran -- it is
    *         <b>not</b> a quality claim. A rank-deficient cluster returns {@code true} with a
    *         well-formed rotation and {@code σ₃ ≈ 0}.
    */
   public boolean compute(RegistrationResult resultToPack)
   {
      if (correspondenceCount < MINIMUM_CORRESPONDENCES)
      {
         resultToPack.setToNaN();
         return false;
      }

      double invL = 1.0 / correspondenceCount;

      double sourceMeanX = 0.0, sourceMeanY = 0.0, sourceMeanZ = 0.0;
      double targetMeanX = 0.0, targetMeanY = 0.0, targetMeanZ = 0.0;

      for (int l = 0; l < correspondenceCount; l++)
      {
         int i = 3 * l;
         sourceMeanX += source[i];
         sourceMeanY += source[i + 1];
         sourceMeanZ += source[i + 2];
         targetMeanX += target[i];
         targetMeanY += target[i + 1];
         targetMeanZ += target[i + 2];
      }

      sourceMeanX *= invL;
      sourceMeanY *= invL;
      sourceMeanZ *= invL;
      targetMeanX *= invL;
      targetMeanY *= invL;
      targetMeanZ *= invL;

      // H = (1/L) sum_l (b_l - b̄)(a_l - ā)ᵀ, accumulated in a second pass rather than as
      // (1/L)sum b aᵀ - b̄ āᵀ. The one-pass form loses precision by cancellation exactly where it
      // matters: marker coordinates are O(1 m) about a room origin while the spreads carrying all
      // the pose information are O(0.1 m) and the noise is O(0.3 mm).
      double[] h = crossCovariance.data;
      java.util.Arrays.fill(h, 0.0);

      for (int l = 0; l < correspondenceCount; l++)
      {
         int i = 3 * l;
         double ax = source[i] - sourceMeanX;
         double ay = source[i + 1] - sourceMeanY;
         double az = source[i + 2] - sourceMeanZ;
         double bx = target[i] - targetMeanX;
         double by = target[i + 1] - targetMeanY;
         double bz = target[i + 2] - targetMeanZ;

         h[0] += bx * ax;
         h[1] += bx * ay;
         h[2] += bx * az;
         h[3] += by * ax;
         h[4] += by * ay;
         h[5] += by * az;
         h[6] += bz * ax;
         h[7] += bz * ay;
         h[8] += bz * az;
      }

      for (int i = 0; i < 9; i++)
         h[i] *= invL;

      if (!svd.decompose(crossCovariance))
         throw new IllegalStateException("SVD of the cross-covariance failed to converge with " + correspondenceCount + " correspondences.");

      svd.getU(U, false);
      svd.getV(V, false);

      double[] w = svd.getSingularValues();
      singularValues[0] = w[0];
      singularValues[1] = w[1];
      singularValues[2] = w[2];

      sortDescending();

      // R = U diag(1, 1, det(U Vᵀ)) Vᵀ. det(U Vᵀ) = det(U)·det(V), both ±1, so the third column
      // of U is scaled in place and the diagonal factor disappears into a plain U Vᵀ.
      double determinant = determinant3x3(U) * determinant3x3(V);
      boolean reflectionCorrected = determinant < 0.0;

      if (reflectionCorrected)
      {
         U.data[2] = -U.data[2];
         U.data[5] = -U.data[5];
         U.data[8] = -U.data[8];
      }

      CommonOps_DDRM.multTransB(U, V, rotation);

      double[] r = rotation.data;
      // Throws NotARotationMatrixException if this is not orthonormal. It always is -- R is a
      // product of orthogonal matrices with the determinant repaired -- so the check is a
      // tripwire on the algebra above, not a runtime cost worth avoiding.
      resultToPack.getTransform().getRotation().set(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);

      // t = b̄ - R ā
      resultToPack.getTransform()
                  .getTranslation()
                  .set(targetMeanX - (r[0] * sourceMeanX + r[1] * sourceMeanY + r[2] * sourceMeanZ),
                       targetMeanY - (r[3] * sourceMeanX + r[4] * sourceMeanY + r[5] * sourceMeanZ),
                       targetMeanZ - (r[6] * sourceMeanX + r[7] * sourceMeanY + r[8] * sourceMeanZ));

      resultToPack.setSingularValues(singularValues[0], singularValues[1], singularValues[2]);
      resultToPack.setCorrespondenceCount(correspondenceCount);
      resultToPack.setReflectionCorrected(reflectionCorrected);
      resultToPack.setSuccessful(true);
      return true;
   }

   /**
    * Sorts the three singular values into descending order, permuting the matching columns of
    * {@code U} and {@code V} with them.
    * <p>
    * Valid because {@code H = sum_k σ_k u_k v_kᵀ}: permuting the triples
    * {@code (σ_k, u_k, v_k)} together leaves the decomposition intact, and permuting columns of
    * an orthogonal matrix leaves it orthogonal, so {@code det(U)·det(V)} is unchanged by an even
    * permutation and flips sign twice -- once in each factor -- under an odd one. Either way the
    * product is preserved and the determinant repair below is unaffected.
    * </p>
    */
   private void sortDescending()
   {
      for (int i = 0; i < 2; i++)
      {
         int largest = i;
         for (int j = i + 1; j < 3; j++)
         {
            if (singularValues[j] > singularValues[largest])
               largest = j;
         }

         if (largest != i)
         {
            double swap = singularValues[i];
            singularValues[i] = singularValues[largest];
            singularValues[largest] = swap;
            swapColumns(U, i, largest);
            swapColumns(V, i, largest);
         }
      }
   }

   private static void swapColumns(DMatrixRMaj matrix, int columnA, int columnB)
   {
      double[] m = matrix.data;

      for (int row = 0; row < 3; row++)
      {
         int a = 3 * row + columnA;
         int b = 3 * row + columnB;
         double swap = m[a];
         m[a] = m[b];
         m[b] = swap;
      }
   }

   private static double determinant3x3(DMatrixRMaj matrix)
   {
      double[] m = matrix.data;
      return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
   }

   private void ensureCapacity(int correspondences)
   {
      if (3 * correspondences <= source.length)
         return;

      int newCapacity = Math.max(2 * getCapacity(), correspondences);
      source = java.util.Arrays.copyOf(source, 3 * newCapacity);
      target = java.util.Arrays.copyOf(target, 3 * newCapacity);
   }
}
