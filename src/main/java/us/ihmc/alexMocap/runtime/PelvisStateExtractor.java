package us.ihmc.alexMocap.runtime;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * F10, pelvis state for the EKF comparison (FRAMEWORK.md §13) -- <b>pose only</b>.
 *
 * <h2>This class exposes no velocity, and must never grow one</h2>
 * <p>
 * That is not a simplification or an unfinished corner. §13 works the arithmetic: raw single
 * differencing at 200 Hz with {@code σ = 0.93 mm} gives
 * </p>
 *
 * <pre>
 * √2 · σ / Δt  ≈  0.13 m/s
 * </pre>
 *
 * <p>
 * against ContactNet baselines of 0.0844 and 0.0254 m/s. The noise is five times the quantity being
 * validated. The answer is a <b>centred</b> Savitzky-Golay differentiator run offline over the
 * logged pose trajectory ({@code postprocess.SGDifferentiator}), which reaches about 0.0037 m/s and
 * is zero-lag by construction.
 * </p>
 * <p>
 * A centred window cannot execute causally -- it needs samples from the future -- so there is no
 * correct runtime velocity to expose. If this class had a {@code getTwist()}, someone would call it,
 * and they would compare 0.13 m/s of noise against a 0.025 m/s estimator and conclude the estimator
 * had regressed. {@code PelvisStateExtractorTest} asserts by reflection that no method here returns
 * a velocity or twist type, so the guarantee survives a well-meaning future edit.
 * </p>
 *
 * <h2>Which pelvis frame</h2>
 * <p>
 * {@code ^Wg T̂_b}: the <b>URDF pelvis link frame</b>, in the gravity-aligned world. Not the Motive
 * cluster frame {@code c}, and not the IMU frame. §13 calls these three "the three-pelvis-frames
 * hazard"; {@code frames.PelvisFrameTriad} is where they are kept apart, and
 * {@link #getPelvisFrame()} hands out the one this class means so a caller cannot pick the wrong
 * one by writing a plausible-looking transform multiply.
 * </p>
 * <p>
 * <b>Confirm which frame the EKF publishes before comparing anything.</b> If it reports in the IMU
 * frame, the comparison additionally needs the {@code b → imu} transform, which is an unverified
 * CAD number worth 0.1 m/s of {@code ω × r} error at 1 rad/s.
 * </p>
 */
public class PelvisStateExtractor
{
   private final String pelvisLinkName;
   private final int pelvisLinkIndex;
   private final ReferenceFrame gravityAlignedWorld;
   private final RigidBodyTransform motiveWorldToGravityAligned = new RigidBodyTransform();

   private final RigidBodyTransform pelvisPose = new RigidBodyTransform();
   private boolean poseAvailable = false;

   /**
    * @param pelvisLinkName      the URDF base link.
    * @param linkNames           the link order of the {@link MeasuredLinkPoses} this reads.
    * @param gravityAlignedWorld {@code Wg}, from {@code frames.GravityAlignedWorldFrame}. Poses are
    *                            reported in this frame, so the F8 correction is applied by
    *                            construction rather than by the caller remembering to.
    */
   public PelvisStateExtractor(String pelvisLinkName, java.util.List<String> linkNames, ReferenceFrame gravityAlignedWorld, ReferenceFrame motiveWorld)
   {
      this.pelvisLinkName = pelvisLinkName;
      this.pelvisLinkIndex = linkNames.indexOf(pelvisLinkName);
      this.gravityAlignedWorld = gravityAlignedWorld;

      if (pelvisLinkIndex < 0)
         throw new IllegalArgumentException("The pelvis link '" + pelvisLinkName + "' is not among the reported links " + linkNames + ".");

      motiveWorld.getTransformToDesiredFrame(motiveWorldToGravityAligned, gravityAlignedWorld);
   }

   /**
    * Reads the pelvis pose out of one frame's link poses and carries it into {@code Wg}.
    *
    * @return whether a pose was available. False means F6 refused the pelvis cluster this frame --
    *         which also means there is no CoM, since every chained link hangs off it.
    */
   public boolean extract(MeasuredLinkPoses poses)
   {
      if (!poses.isAvailable(pelvisLinkIndex))
      {
         pelvisPose.setToNaN();
         poseAvailable = false;
         return false;
      }

      pelvisPose.set(motiveWorldToGravityAligned);
      pelvisPose.multiply(poses.getPose(pelvisLinkIndex));
      poseAvailable = true;
      return true;
   }

   /** {@code ^Wg T̂_b} from the last {@link #extract}. NaN if it refused. */
   public RigidBodyTransformReadOnly getPelvisPose()
   {
      return pelvisPose;
   }

   public boolean isPoseAvailable()
   {
      return poseAvailable;
   }

   /** Pelvis origin in {@code Wg}, for a log column that does not need the full pose. */
   public void packPelvisPosition(Point3DBasics toPack)
   {
      toPack.set(new Point3D(pelvisPose.getTranslation()));
   }

   public String getPelvisLinkName()
   {
      return pelvisLinkName;
   }

   /** {@code Wg}: the frame {@link #getPelvisPose()} is expressed in. */
   public ReferenceFrame getPelvisFrame()
   {
      return gravityAlignedWorld;
   }
}
