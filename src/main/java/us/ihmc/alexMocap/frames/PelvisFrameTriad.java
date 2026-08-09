package us.ihmc.alexMocap.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * The three-pelvis-frames hazard of FRAMEWORK.md §13, made structural.
 *
 * <h2>The hazard</h2>
 * <p>
 * Three distinct frames are all called "pelvis":
 * </p>
 * <ol>
 * <li>{@code c} -- the Motive marker-cluster frame, whose convention is whatever the marker
 * constellation happened to define;</li>
 * <li>{@code b} -- the URDF pelvis link frame;</li>
 * <li>the IMU mounting frame.</li>
 * </ol>
 * <p>
 * Calibration gives {@code c → b} for free: that is {@code Δ}. The third is the dangerous one. If
 * the EKF reports velocity in the IMU frame, F10 needs the fixed {@code b → imu} transform, and
 * that is an <b>unverified CAD number</b>. Getting it wrong costs the {@code ω × r} term: at
 * {@code ω = 1 rad/s} and {@code r = 0.1 m} that is 0.1 m/s, against ContactNet baselines of
 * 0.0844 and 0.0254 m/s. It swamps the comparison entirely, and it reads as an estimator
 * regression rather than as a bookkeeping error.
 * </p>
 *
 * <h2>Why this is a frame tree and not three transforms</h2>
 * <p>
 * §13 asks for all three to be named nodes so that a mismatch throws. Three loose
 * {@code RigidBodyTransform}s would compose in any order the caller wrote, and every wrong order
 * produces a valid-looking pose. Chained as frames, {@code changeFrame} either does the right
 * thing or raises {@code ReferenceFrameMismatchException}, and the ordering question disappears.
 * </p>
 *
 * <pre>
 * motiveWorld  W
 *   └─ pelvisCluster  c        pose from F6, set per frame
 *        └─ pelvisLink  b      via Δ = ^c T_b, from the calibration
 *             └─ pelvisImu     via ^b T_imu, an UNVERIFIED CAD number
 * </pre>
 *
 * <p>
 * The cluster hangs off {@code motiveWorld} rather than off {@code Wg} because that is literally
 * where the measurement is made -- F6 registers against raw camera coordinates. The F8 tilt
 * correction then applies itself on any {@code changeFrame} into {@code Wg}, which is the whole
 * point of modelling it as a frame.
 * </p>
 */
public class PelvisFrameTriad
{
   private final RigidBodyTransform clusterPoseInMotiveWorld = new RigidBodyTransform();
   private final RigidBodyTransform clusterToBase = new RigidBodyTransform();
   private final RigidBodyTransform baseToImu = new RigidBodyTransform();

   private final ReferenceFrame clusterFrame;
   private final ReferenceFrame pelvisLinkFrame;
   private final ReferenceFrame imuFrame;

   private boolean imuTransformVerified = false;

   public PelvisFrameTriad(GravityAlignedWorldFrame world)
   {
      this(world, "");
   }

   /**
    * @param nameSuffix appended to all three frame names; see
    *                   {@link GravityAlignedWorldFrame#GravityAlignedWorldFrame(TiltMeasurement, ReferenceFrame, String)}
    *                   for why this exists.
    */
   public PelvisFrameTriad(GravityAlignedWorldFrame world, String nameSuffix)
   {
      clusterFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(FrameNames.PELVIS_CLUSTER + nameSuffix,
                                                                                     world.getMotiveWorld(),
                                                                                     clusterPoseInMotiveWorld);
      pelvisLinkFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(FrameNames.PELVIS_LINK + nameSuffix,
                                                                                        clusterFrame,
                                                                                        clusterToBase);
      imuFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(FrameNames.PELVIS_IMU + nameSuffix, pelvisLinkFrame, baseToImu);
   }

   /** {@code c}: the Motive marker-cluster frame. Its pose comes from F6, per frame. */
   public ReferenceFrame getClusterFrame()
   {
      return clusterFrame;
   }

   /** {@code b}: the URDF pelvis link frame. This is what F10 reports pose in. */
   public ReferenceFrame getPelvisLinkFrame()
   {
      return pelvisLinkFrame;
   }

   /** The IMU mounting frame. Valid only if {@link #isImuTransformVerified()}. */
   public ReferenceFrame getImuFrame()
   {
      return imuFrame;
   }

   /** {@code ^W T_c}, from F6 on the pelvis cluster. Updates the whole chain below it. */
   public void setClusterPoseInMotiveWorld(RigidBodyTransformReadOnly clusterPose)
   {
      clusterPoseInMotiveWorld.set(clusterPose);
      updateFrames();
   }

   /** {@code Δ = ^c T_b}, from {@code CalibrationResult}. A session constant. */
   public void setClusterToBase(RigidBodyTransformReadOnly delta)
   {
      clusterToBase.set(delta);
      updateFrames();
   }

   /**
    * {@code ^b T_imu}, the pelvis-link-to-IMU transform.
    *
    * @param verified whether this was checked against the physical mounting, as FRAMEWORK.md §13
    *                 requires, or merely read out of CAD. There is deliberately no overload that
    *                 omits this: an unverified number that does not announce itself is exactly the
    *                 failure §13 describes, and the cost of it is an apparent estimator regression
    *                 nobody traces back to a transform.
    */
   public void setBaseToImu(RigidBodyTransformReadOnly baseToImu, boolean verified)
   {
      this.baseToImu.set(baseToImu);
      this.imuTransformVerified = verified;
      updateFrames();
   }

   public boolean isImuTransformVerified()
   {
      return imuTransformVerified;
   }

   /**
    * {@code ||ω × r||} at the given angular rate: how much velocity error a wrong IMU lever arm
    * injects into an F10 comparison.
    * <p>
    * FRAMEWORK.md §13 works the example at {@code ω = 1 rad/s}, {@code r = 0.1 m} and gets
    * 0.1 m/s. Exposed as a function so the number can be asserted against the actual mounting and
    * printed in a report, rather than remembered as a worked example.
    * </p>
    */
   public double getVelocityErrorFromLeverArm(double angularRateRadiansPerSecond)
   {
      return angularRateRadiansPerSecond * baseToImu.getTranslation().norm();
   }

   private void updateFrames()
   {
      clusterFrame.update();
      pelvisLinkFrame.update();
      imuFrame.update();
   }

   @Override
   public String toString()
   {
      return "PelvisFrameTriad[|r_imu|=" + baseToImu.getTranslation().norm() + " m, imuTransform="
            + (imuTransformVerified ? "verified" : "UNVERIFIED") + "]";
   }
}
