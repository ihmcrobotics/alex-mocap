package us.ihmc.alexMocap.core;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * FRAMEWORK.md §0's observation model, as executable code:
 *
 * <pre>
 * ^W m_ijk  =  ^W T_c^(k) · Δ · ^b T_i(q^(k)) · ^i p_ij  +  ε_ijk
 * </pre>
 *
 * <p>
 * Everything in F3-F5 inverts this equation and F6-F9 consume its solution, so it is written once,
 * here, and every package that needs it calls the same three lines. The alternative -- each of
 * {@code calibration}, {@code gates} and {@code runtime} composing the transforms itself -- is four
 * chances to get the multiplication order wrong, and a wrong order produces a plausible pose rather
 * than an exception.
 * </p>
 *
 * <h2>Why this lives in {@code core}</h2>
 * <p>
 * It is needed by {@code calibration} (to evaluate {@code J}), by {@code gates} (G2 back-projects,
 * G4 predicts held-out markers), and later by {@code runtime}. FRAMEWORK.md §19 forbids
 * {@code gates} from importing {@code calibration}, and {@code core} is the one package all three
 * may see -- the same reason {@link CalibrationResult} lives here. It depends on Euclid alone.
 * </p>
 *
 * <h2>Transform directions, spelled out</h2>
 * <p>
 * The names follow {@link CalibrationResult#getClusterToBase()}, which describes the frame
 * relationship rather than the direction of the matrix-vector product. Concretely, in the
 * composition above:
 * </p>
 * <ul>
 * <li>{@code ^b T_i} (here {@code linkToBase}) maps a point in link {@code i} to the base;</li>
 * <li>{@code Δ = ^c T_b} (here {@code clusterToBase}) maps a point in the base to the cluster;</li>
 * <li>{@code ^W T_c} (here {@code clusterToWorld}) maps a point in the cluster to the world.</li>
 * </ul>
 * <p>
 * Read right to left, a marker travels {@code i → b → c → W}.
 * </p>
 *
 * <p>
 * Every method is static and writes into caller-owned output, so nothing here allocates. The inner
 * loops of F4, F5 and G2 run {@code links × markers × captures} times.
 * </p>
 */
public final class ObservationModel
{
   private ObservationModel()
   {
   }

   /**
    * {@code ^W T_i^(k) = ^W T_c^(k) · Δ · ^b T_i(q^(k))}: the pose of link {@code i} in the world,
    * as the <b>model</b> predicts it from the gauge cluster and the encoders.
    * <p>
    * This is the first of the two objects FRAMEWORK.md §0 distinguishes. It is not
    * {@code ^W T̂_i}, which comes from a marker cluster through {@code RigidBodyRegistration} and
    * is what F6 measures at runtime. They are compared, never substituted.
    * </p>
    */
   public static void packLinkToWorld(RigidBodyTransformReadOnly clusterToWorld,
                                      RigidBodyTransformReadOnly clusterToBase,
                                      RigidBodyTransformReadOnly linkToBase,
                                      RigidBodyTransform toPack)
   {
      toPack.set(clusterToWorld);
      toPack.multiply(clusterToBase);
      toPack.multiply(linkToBase);
   }

   /**
    * The predicted world position of a marker, {@code ^W T_i^(k) · ^i p_ij}: the noiseless
    * right-hand side of the observation model.
    */
   public static void packPredictedMarkerPosition(RigidBodyTransformReadOnly linkToWorld, Point3DReadOnly markerInLinkFrame, Point3DBasics toPack)
   {
      toPack.set(markerInLinkFrame);
      linkToWorld.transform(toPack);
   }

   /**
    * Back-projection: {@code ^i p_ij = (^W T_i^(k))^-1 · ^W m_ijk}, a measured world position
    * carried into the link frame.
    * <p>
    * This is the operation F3 performs once (§5) and F4 averages over captures (§6). Their whole
    * content is this line plus, for F4, a mean.
    * </p>
    */
   public static void packMarkerInLinkFrame(RigidBodyTransformReadOnly linkToWorld, Point3DReadOnly measuredInWorld, Point3DBasics toPack)
   {
      toPack.set(measuredInWorld);
      linkToWorld.inverseTransform(toPack);
   }

   /**
    * The squared residual of one observation, in m²: one term of {@code J} (FRAMEWORK.md §8).
    * <p>
    * Squared rather than the norm because {@code J} sums squares, and taking a square root here
    * only to square it again in the caller both costs time and loses a little precision.
    * </p>
    */
   public static double squaredResidual(Point3DReadOnly measuredInWorld, Point3DReadOnly predictedInWorld)
   {
      double dx = measuredInWorld.getX() - predictedInWorld.getX();
      double dy = measuredInWorld.getY() - predictedInWorld.getY();
      double dz = measuredInWorld.getZ() - predictedInWorld.getZ();

      return dx * dx + dy * dy + dz * dz;
   }
}
