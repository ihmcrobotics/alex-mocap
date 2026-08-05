package us.ihmc.alexMocap.registration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.management.ManagementFactory;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Tests for the registration primitive of FRAMEWORK.md §2.
 * <p>
 * Every randomised test here uses a fixed seed. Where a statistical property is asserted the
 * threshold is set well above the theoretical value and the theoretical value is stated in a
 * comment, so that a passing test stays passing and a failing one means something changed.
 * </p>
 */
public class RigidBodyRegistrationTest
{
   /** Target per-axis mocap noise in the tight gantry volume (FRAMEWORK.md §17). */
   private static final double MOCAP_SIGMA = 0.3e-3;

   /**
    * Plant a random pose, transform points through it, register, recover it. With no noise this
    * is exact arithmetic and any error above round-off is an algebra bug.
    */
   @Test
   public void testExactRecovery()
   {
      Random random = new Random(2718L);
      RigidBodyRegistration registration = new RigidBodyRegistration();
      RegistrationResult result = new RegistrationResult();

      for (int trial = 0; trial < 1000; trial++)
      {
         RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         int markerCount = 4 + random.nextInt(5);

         registration.clear();

         for (int j = 0; j < markerCount; j++)
         {
            Point3D sourcePoint = EuclidCoreRandomTools.nextPoint3D(random, 0.15);
            Point3D targetPoint = new Point3D(sourcePoint);
            planted.transform(targetPoint);
            registration.addCorrespondence(sourcePoint, targetPoint);
         }

         assertTrue(registration.compute(result));
         assertTrue(result.wasSuccessful());
         assertEquals(markerCount, result.getCorrespondenceCount());
         assertTransformEquals(planted, result.getTransform(), 1.0e-12);
      }
   }

   /**
    * The reason for Umeyama over Arun. A coplanar cluster -- markers on a flat link face, the
    * realistic case -- makes {@code σ₃} noise-dominated, and since
    * {@code sign(det(H)) = det(U)·det(V)}, the sign of the raw {@code U Vᵀ} is then a coin flip.
    * <p>
    * Three assertions, in increasing order of what they buy:
    * <ol>
    * <li>{@code det(R) = +1} and {@code RᵀR = I} on every seed;</li>
    * <li>the guard actually fires on a large fraction of seeds -- a guard that is never exercised
    * is a guard you have never seen work;</li>
    * <li>the corrected rotation is the <i>planted</i> one, not merely <i>a</i> rotation. Fixing
    * the determinant without recovering the right pose would be a silent failure of exactly the
    * kind FRAMEWORK.md §18 is about.</li>
    * </ol>
    */
   @Test
   public void testReflectionGuardOnCoplanarCluster()
   {
      Random random = new Random(31415L);
      RigidBodyRegistration registration = new RigidBodyRegistration();
      RegistrationResult result = new RegistrationResult();
      RotationMatrix rotationError = new RotationMatrix();

      // An asymmetric quad in the z = 0 plane, roughly a 120 mm cluster: coplanar by construction,
      // so the noiseless cross-covariance is exactly rank 2.
      double[][] planarCluster = {{0.060, 0.000, 0.0}, {-0.045, 0.055, 0.0}, {-0.050, -0.048, 0.0}, {0.030, -0.062, 0.0}};

      int trials = 1000;
      int reflectionsCorrected = 0;

      for (int trial = 0; trial < trials; trial++)
      {
         RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         registration.clear();

         for (double[] marker : planarCluster)
         {
            Point3D sourcePoint = new Point3D(marker[0], marker[1], marker[2]);
            Point3D targetPoint = new Point3D(sourcePoint);
            planted.transform(targetPoint);
            targetPoint.add(MOCAP_SIGMA * random.nextGaussian(), MOCAP_SIGMA * random.nextGaussian(), MOCAP_SIGMA * random.nextGaussian());
            registration.addCorrespondence(sourcePoint, targetPoint);
         }

         assertTrue(registration.compute(result));

         if (result.wasReflectionCorrected())
            reflectionsCorrected++;

         RotationMatrix recovered = new RotationMatrix(result.getTransform().getRotation());
         assertEquals(1.0, recovered.determinant(), 1.0e-10, "det(R) must be +1 on trial " + trial);
         assertOrthonormal(recovered, 1.0e-10);

         // Theoretical angular error for a planar cluster is about σ / (√N · r_perp)
         // = 0.3 mm / (2 · 55 mm) ≈ 2.7e-3 rad. Threshold set at ~7x that.
         rotationError.set(planted.getRotation());
         rotationError.multiplyTransposeOther(recovered);
         double angleError = angleOf(rotationError);
         assertTrue(angleError < 2.0e-2, "Reflection-corrected rotation drifted from the planted one on trial " + trial + ": " + angleError + " rad");
      }

      // Noise decides the sign, so the split is ~50/50. A one-sided outcome would mean the test
      // never actually presented the solver with a reflection.
      assertTrue(reflectionsCorrected > trials / 4 && reflectionsCorrected < 3 * trials / 4,
                 "The reflection guard fired on " + reflectionsCorrected + " of " + trials + " seeds; the test is not exercising it.");
   }

   /**
    * The silent failure of FRAMEWORK.md §18.1. Collinear markers still yield a perfectly
    * well-formed rotation matrix -- nothing about the transform betrays that two of its three axes
    * are unconstrained. The only in-band signal is {@code σ₃}, and this asserts that a caller
    * holding nothing but a {@link RegistrationResult} can see it.
    */
   @Test
   public void testRankDeficiencyIsVisibleInSigma3()
   {
      Random random = new Random(1123L);
      RigidBodyRegistration registration = new RigidBodyRegistration();
      RegistrationResult result = new RegistrationResult();

      RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      registration.clear();

      for (int j = 0; j < 5; j++)
      {
         Point3D sourcePoint = new Point3D(0.03 * j, 0.06 * j, -0.02 * j);
         Point3D targetPoint = new Point3D(sourcePoint);
         planted.transform(targetPoint);
         registration.addCorrespondence(sourcePoint, targetPoint);
      }

      // The solve succeeds. That is the point: success is not a quality claim.
      assertTrue(registration.compute(result));
      assertTrue(result.wasSuccessful());

      RotationMatrix recovered = new RotationMatrix(result.getTransform().getRotation());
      assertEquals(1.0, recovered.determinant(), 1.0e-10);
      assertOrthonormal(recovered, 1.0e-10);

      assertTrue(result.getSigma1() > 1.0e-4, "σ₁ should carry the along-line spread, was " + result.getSigma1());
      assertTrue(result.getSigma2() / result.getSigma1() < 1.0e-9, "σ₂/σ₁ = " + result.getSigma2() / result.getSigma1());
      assertTrue(result.getSigma3() / result.getSigma1() < 1.0e-9, "σ₃/σ₁ = " + result.getSigma3() / result.getSigma1());

      // Same line, but with realistic noise: σ₃ is no longer numerically zero, so a caller that
      // tested `σ₃ == 0` would be fooled. It is still four orders of magnitude below σ₁.
      registration.clear();

      for (int j = 0; j < 5; j++)
      {
         Point3D sourcePoint = new Point3D(0.03 * j, 0.06 * j, -0.02 * j);
         Point3D targetPoint = new Point3D(sourcePoint);
         planted.transform(targetPoint);
         targetPoint.add(MOCAP_SIGMA * random.nextGaussian(), MOCAP_SIGMA * random.nextGaussian(), MOCAP_SIGMA * random.nextGaussian());
         registration.addCorrespondence(sourcePoint, targetPoint);
      }

      assertTrue(registration.compute(result));
      assertTrue(result.getSigma3() > 0.0, "Noise makes σ₃ strictly positive; an equality test against zero would not detect this cluster.");
      assertTrue(result.getSigma3() / result.getSigma1() < 1.0e-3, "σ₃/σ₁ = " + result.getSigma3() / result.getSigma1());
   }

   /**
    * EJML does not order singular values. Without the explicit descending sort, {@code σ₃} is
    * whichever value landed third and the conditioning monitor is noise.
    * <p>
    * The cluster is an octahedron with unequal semi-axes {@code (sx, sy, sz)}, so the
    * cross-covariance is {@code H = R · diag(sx², sy², sz²)/3} and the singular values are known
    * in closed form. The semi-axes are deliberately given in scrambled magnitude order.
    * </p>
    */
   @Test
   public void testSingularValuesAreSortedDescending()
   {
      Random random = new Random(5772L);
      RigidBodyRegistration registration = new RigidBodyRegistration();
      RegistrationResult result = new RegistrationResult();

      double sx = 0.05, sy = 0.15, sz = 0.10;
      double[][] octahedron = {{sx, 0, 0}, {-sx, 0, 0}, {0, sy, 0}, {0, -sy, 0}, {0, 0, sz}, {0, 0, -sz}};

      RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      registration.clear();

      for (double[] marker : octahedron)
      {
         Point3D sourcePoint = new Point3D(marker[0], marker[1], marker[2]);
         Point3D targetPoint = new Point3D(sourcePoint);
         planted.transform(targetPoint);
         registration.addCorrespondence(sourcePoint, targetPoint);
      }

      assertTrue(registration.compute(result));

      // (1/6) · sum p pᵀ = diag(sx², sy², sz²)/3, so σ = {0.0075, 0.003333, 0.000833} once sorted.
      assertEquals(sy * sy / 3.0, result.getSigma1(), 1.0e-12);
      assertEquals(sz * sz / 3.0, result.getSigma2(), 1.0e-12);
      assertEquals(sx * sx / 3.0, result.getSigma3(), 1.0e-12);

      // And the ordering holds for arbitrary geometry, not just the constructed case.
      for (int trial = 0; trial < 1000; trial++)
      {
         RigidBodyTransform trialTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         registration.clear();

         for (int j = 0; j < 6; j++)
         {
            Point3D sourcePoint = EuclidCoreRandomTools.nextPoint3D(random, 0.15);
            Point3D targetPoint = new Point3D(sourcePoint);
            trialTransform.transform(targetPoint);
            registration.addCorrespondence(sourcePoint, targetPoint);
         }

         assertTrue(registration.compute(result));
         assertTrue(result.getSigma1() >= result.getSigma2(), "σ₁ < σ₂ on trial " + trial);
         assertTrue(result.getSigma2() >= result.getSigma3(), "σ₂ < σ₃ on trial " + trial);
         assertTrue(result.getSigma3() >= 0.0, "σ₃ < 0 on trial " + trial);
      }
   }

   /**
    * Normalising {@code H} by the correspondence count is what makes {@code σ₃} a mean-squared
    * spread rather than a sum, and therefore comparable between frames that saw different numbers
    * of markers.
    * <p>
    * Isolating that claim needs two clusters with the <i>same</i> mean-squared spread and
    * <i>different</i> marker counts, because removing a marker from a fixed cluster genuinely
    * changes its geometry and no normalisation can or should hide that. The two used here are a
    * regular tetrahedron (L = 4) and a triangular bipyramid with the pole height tuned to
    * {@code h = a√3/2} (L = 5). Both have isotropic covariance, and the tetrahedron circumradius
    * {@code r = a√0.9} makes the two spreads equal: {@code r²/3 = 0.3a² = 0.003 m²}.
    * </p>
    */
   @Test
   public void testSigmaIsNormalisedByCorrespondenceCount()
   {
      Random random = new Random(1618L);
      RigidBodyRegistration registration = new RigidBodyRegistration();
      RegistrationResult fourMarkers = new RegistrationResult();
      RegistrationResult fiveMarkers = new RegistrationResult();

      double a = 0.10;
      double h = a * Math.sqrt(3.0) / 2.0;
      double s = a * Math.sqrt(0.9) / Math.sqrt(3.0);
      double equatorial = a * Math.sqrt(3.0) / 2.0;

      double[][] tetrahedron = {{s, s, s}, {s, -s, -s}, {-s, s, -s}, {-s, -s, s}};
      double[][] bipyramid = {{a, 0, 0}, {-a / 2.0, equatorial, 0}, {-a / 2.0, -equatorial, 0}, {0, 0, h}, {0, 0, -h}};

      RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      registerNoiseless(registration, tetrahedron, planted, fourMarkers);
      registerNoiseless(registration, bipyramid, planted, fiveMarkers);

      assertEquals(4, fourMarkers.getCorrespondenceCount());
      assertEquals(5, fiveMarkers.getCorrespondenceCount());

      double relativeDifference = Math.abs(fiveMarkers.getSigma3() - fourMarkers.getSigma3()) / fourMarkers.getSigma3();

      // The plan's acceptance threshold.
      assertTrue(relativeDifference < 0.10, "σ₃ moved " + 100.0 * relativeDifference + "% between 4 and 5 markers");
      // What the construction actually delivers: the two spreads are equal in closed form, so
      // anything above round-off means the 1/L normalisation is wrong, not merely loose.
      assertEquals(0.003, fourMarkers.getSigma3(), 1.0e-12);
      assertEquals(0.003, fiveMarkers.getSigma3(), 1.0e-12);

      // And the contrast that motivates the normalisation: the un-normalised sums, which are what
      // an implementation without the 1/L would report, are 25% apart for identical geometry.
      double unnormalisedFour = 4.0 * fourMarkers.getSigma3();
      double unnormalisedFive = 5.0 * fiveMarkers.getSigma3();
      assertEquals(0.25, (unnormalisedFive - unnormalisedFour) / unnormalisedFour, 1.0e-9);
   }

   /**
    * The {@code σ/√N} claim that FRAMEWORK.md §6 and §17 rest on, checked at the level of a single
    * registration.
    * <p>
    * Two clusters are used. The first is centred on its own centroid, so {@code ā = 0} and
    * {@code t = b̄} exactly: this isolates the centroid estimator. The second is the same cluster
    * translated a metre off the origin, where {@code t = b̄ - R ā} and a bookkeeping error in that
    * subtraction would show up as inflated spread.
    * </p>
    */
   @Test
   public void testTranslationNoiseScalesAsSigmaOverSqrtN()
   {
      Random random = new Random(1414L);
      RigidBodyRegistration registration = new RigidBodyRegistration();
      RegistrationResult result = new RegistrationResult();

      double s = 0.10 / Math.sqrt(3.0);
      double[][] tetrahedron = {{s, s, s}, {s, -s, -s}, {-s, s, -s}, {-s, -s, s}};
      double[] offsets = {0.0, 1.0};

      for (double offset : offsets)
      {
         int trials = 1000;
         double sumOfSquares = 0.0;
         Point3D sourceCentroid = new Point3D();
         Point3D mappedCentroid = new Point3D();
         Point3D expectedCentroid = new Point3D();

         for (int trial = 0; trial < trials; trial++)
         {
            RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            registration.clear();
            sourceCentroid.setToZero();

            for (double[] marker : tetrahedron)
            {
               Point3D sourcePoint = new Point3D(marker[0] + offset, marker[1], marker[2]);
               Point3D targetPoint = new Point3D(sourcePoint);
               planted.transform(targetPoint);
               targetPoint.add(MOCAP_SIGMA * random.nextGaussian(), MOCAP_SIGMA * random.nextGaussian(), MOCAP_SIGMA * random.nextGaussian());
               registration.addCorrespondence(sourcePoint, targetPoint);
               sourceCentroid.scaleAdd(1.0 / tetrahedron.length, sourcePoint, sourceCentroid);
            }

            assertTrue(registration.compute(result));

            mappedCentroid.set(sourceCentroid);
            result.getTransform().transform(mappedCentroid);
            expectedCentroid.set(sourceCentroid);
            planted.transform(expectedCentroid);

            sumOfSquares += square(mappedCentroid.getX() - expectedCentroid.getX());
            sumOfSquares += square(mappedCentroid.getY() - expectedCentroid.getY());
            sumOfSquares += square(mappedCentroid.getZ() - expectedCentroid.getZ());
         }

         double measured = Math.sqrt(sumOfSquares / (3 * trials));
         double theoretical = MOCAP_SIGMA / Math.sqrt(tetrahedron.length);

         // Theoretical is 0.15 mm. The estimator of a standard deviation from 3000 samples has a
         // relative standard error of 1/√(2·3000) ≈ 1.3%, so the 10% band below is ~8 sigma.
         assertEquals(1.0, measured / theoretical, 0.10, "Centroid error std was " + measured + " m against a theoretical " + theoretical + " m");
      }
   }

   /**
    * Below three correspondences there is no pose to return. The result is NaN rather than
    * identity: a caller that ignores the return value gets NaN propagating visibly downstream
    * instead of a plausible-looking pose at the origin.
    */
   @Test
   public void testBelowMinimumCorrespondencesReturnsNaN()
   {
      RigidBodyRegistration registration = new RigidBodyRegistration();
      RegistrationResult result = new RegistrationResult();

      for (int count = 0; count < RigidBodyRegistration.MINIMUM_CORRESPONDENCES; count++)
      {
         registration.clear();

         for (int j = 0; j < count; j++)
            registration.addCorrespondence(new Point3D(0.1 * j, 0.05, -0.02), new Point3D(0.1 * j + 1.0, 0.05, -0.02));

         assertFalse(registration.compute(result), "Registration should refuse " + count + " correspondences");
         assertFalse(result.wasSuccessful());
         assertTrue(result.getTransform().containsNaN(), "Transform must be NaN with " + count + " correspondences");
         assertTrue(Double.isNaN(result.getSigma1()));
         assertTrue(Double.isNaN(result.getSigma2()));
         assertTrue(Double.isNaN(result.getSigma3()));
         assertEquals(0, result.getCorrespondenceCount());
      }

      // And the boundary case does succeed, so the refusal above is the count and not a typo.
      registration.clear();
      registration.addCorrespondence(new Point3D(0.0, 0.0, 0.0), new Point3D(1.0, 0.0, 0.0));
      registration.addCorrespondence(new Point3D(0.1, 0.0, 0.0), new Point3D(1.1, 0.0, 0.0));
      registration.addCorrespondence(new Point3D(0.0, 0.1, 0.0), new Point3D(1.0, 0.1, 0.0));
      assertTrue(registration.compute(result));
      assertEquals(3, result.getCorrespondenceCount());
   }

   /**
    * Garbage-free is asserted, not intended. The runtime loop of FRAMEWORK.md §9 runs this
    * primitive once per marked cluster per frame at 200 Hz; an allocation per call is an
    * allocation per cluster per frame.
    * <p>
    * The measurement validates itself first: a loop that is known to allocate must read as
    * allocating, otherwise a zero from the real loop would prove nothing.
    * </p>
    */
   @Test
   public void testRegistrationIsAllocationFree()
   {
      com.sun.management.ThreadMXBean threadBean = (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
      assertTrue(threadBean.isThreadAllocatedMemorySupported(), "This JVM cannot measure per-thread allocation; the test would be vacuous.");
      threadBean.setThreadAllocatedMemoryEnabled(true);
      long threadId = Thread.currentThread().getId();

      // Self-check on the meter.
      long beforeControl = threadBean.getThreadAllocatedBytes(threadId);
      Object[] sink = new Object[1024];
      for (int i = 0; i < 1024; i++)
         sink[i] = new Point3D(i, i, i);
      long controlAllocation = threadBean.getThreadAllocatedBytes(threadId) - beforeControl;
      assertTrue(controlAllocation > 0, "The allocation meter reported zero for a loop that allocates 1024 Point3Ds.");
      assertTrue(sink[0] != null);

      Random random = new Random(4669L);
      int markerCount = 6;
      RigidBodyRegistration registration = new RigidBodyRegistration(markerCount);
      RegistrationResult result = new RegistrationResult();

      double[] sourcePoints = new double[3 * markerCount];
      double[] targetPoints = new double[3 * markerCount];
      RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Point3D scratch = new Point3D();

      for (int j = 0; j < markerCount; j++)
      {
         scratch.set(EuclidCoreRandomTools.nextPoint3D(random, 0.15));
         sourcePoints[3 * j] = scratch.getX();
         sourcePoints[3 * j + 1] = scratch.getY();
         sourcePoints[3 * j + 2] = scratch.getZ();
         planted.transform(scratch);
         targetPoints[3 * j] = scratch.getX();
         targetPoints[3 * j + 1] = scratch.getY();
         targetPoints[3 * j + 2] = scratch.getZ();
      }

      // Warm up: class loading, JIT, and EJML's own lazily sized internals all allocate on the
      // first passes and none of that is what this test is about.
      runRegistrations(registration, result, sourcePoints, targetPoints, markerCount, 20_000);

      long before = threadBean.getThreadAllocatedBytes(threadId);
      runRegistrations(registration, result, sourcePoints, targetPoints, markerCount, 10_000);
      long allocated = threadBean.getThreadAllocatedBytes(threadId) - before;

      assertEquals(0L, allocated, "10,000 registrations allocated " + allocated + " bytes after warmup");
      assertTrue(result.wasSuccessful());
   }

   private static void runRegistrations(RigidBodyRegistration registration,
                                        RegistrationResult result,
                                        double[] sourcePoints,
                                        double[] targetPoints,
                                        int markerCount,
                                        int iterations)
   {
      for (int i = 0; i < iterations; i++)
      {
         registration.clear();

         for (int j = 0; j < markerCount; j++)
         {
            int k = 3 * j;
            registration.addCorrespondence(sourcePoints[k],
                                           sourcePoints[k + 1],
                                           sourcePoints[k + 2],
                                           targetPoints[k],
                                           targetPoints[k + 1],
                                           targetPoints[k + 2]);
         }

         registration.compute(result);
      }
   }

   /** Capacity growth is the one allocating path, and it must not corrupt the accumulated data. */
   @Test
   public void testGrowthBeyondInitialCapacity()
   {
      Random random = new Random(8080L);
      RigidBodyRegistration registration = new RigidBodyRegistration(RigidBodyRegistration.MINIMUM_CORRESPONDENCES);
      RegistrationResult result = new RegistrationResult();

      RigidBodyTransform planted = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      registration.clear();

      int markerCount = 200;

      for (int j = 0; j < markerCount; j++)
      {
         Point3D sourcePoint = EuclidCoreRandomTools.nextPoint3D(random, 0.5);
         Point3D targetPoint = new Point3D(sourcePoint);
         planted.transform(targetPoint);
         registration.addCorrespondence(sourcePoint, targetPoint);
      }

      assertTrue(registration.getCapacity() >= markerCount);
      assertTrue(registration.compute(result));
      assertEquals(markerCount, result.getCorrespondenceCount());
      assertTransformEquals(planted, result.getTransform(), 1.0e-12);
   }

   private static void registerNoiseless(RigidBodyRegistration registration, double[][] cluster, RigidBodyTransform planted, RegistrationResult resultToPack)
   {
      registration.clear();

      for (double[] marker : cluster)
      {
         Point3D sourcePoint = new Point3D(marker[0], marker[1], marker[2]);
         Point3D targetPoint = new Point3D(sourcePoint);
         planted.transform(targetPoint);
         registration.addCorrespondence(sourcePoint, targetPoint);
      }

      assertTrue(registration.compute(resultToPack));
   }

   private static void assertTransformEquals(RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
            assertEquals(expected.getRotation().getElement(row, column), actual.getRotation().getElement(row, column), epsilon, "R[" + row + "][" + column + "]");

         assertEquals(expected.getTranslation().getElement(row), actual.getTranslation().getElement(row), epsilon, "t[" + row + "]");
      }
   }

   private static void assertOrthonormal(RotationMatrix matrix, double epsilon)
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            double dot = 0.0;

            for (int k = 0; k < 3; k++)
               dot += matrix.getElement(k, i) * matrix.getElement(k, j);

            assertEquals(i == j ? 1.0 : 0.0, dot, epsilon, "(RᵀR)[" + i + "][" + j + "]");
         }
      }
   }

   /** Rotation angle of R, from trace(R) = 1 + 2cos(θ). Clamped against round-off past ±1. */
   private static double angleOf(RotationMatrix matrix)
   {
      double trace = matrix.getElement(0, 0) + matrix.getElement(1, 1) + matrix.getElement(2, 2);
      return Math.acos(Math.max(-1.0, Math.min(1.0, 0.5 * (trace - 1.0))));
   }

   private static double square(double value)
   {
      return value * value;
   }
}
