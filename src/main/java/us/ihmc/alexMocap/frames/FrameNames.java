package us.ihmc.alexMocap.frames;

/**
 * Names of the reference frames this project puts in the Euclid frame tree.
 * <p>
 * They are constants rather than string literals because two of FRAMEWORK.md's three silent
 * failures (§18.2, §18.3) are frame confusions, and the mitigation for both is "make it a named
 * node so a mismatch throws". A name spelled two ways in two files produces two frames, and two
 * frames that should have been one do not throw -- they quietly disagree.
 * </p>
 */
public final class FrameNames
{
   private FrameNames()
   {
   }

   /**
    * {@code W}: the world frame exactly as Motive reports it, tilt and all.
    * <p>
    * Raw mocap belongs here and nowhere else. Its residual tilt relative to gravity is systematic
    * and ungated (FRAMEWORK.md §11), so a CoM computed in this frame carries roughly 7 mm of height
    * error at 0.5°.
    * </p>
    */
   public static final String MOTIVE_WORLD = "motiveWorld";

   /**
    * {@code Wg}: gravity-aligned world, the parent of {@link #MOTIVE_WORLD}. F8.
    * <p>
    * Every quantity that is compared against gravity -- CoM height above all -- belongs here.
    * </p>
    */
   public static final String GRAVITY_ALIGNED_WORLD = "gravityAlignedWorld";

   /**
    * {@code c}: the Motive marker-cluster frame for the pelvis cluster.
    * <p>
    * First of the three frames FRAMEWORK.md §13 warns are all called "pelvis". Its convention is
    * arbitrary -- it is whatever the marker constellation defines -- which is exactly why
    * {@code Δ = ^c T_b} has to be solved for rather than assumed.
    * </p>
    */
   public static final String PELVIS_CLUSTER = "pelvisCluster";

   /** {@code b}: the URDF pelvis link frame. Second of the three. */
   public static final String PELVIS_LINK = "pelvisLink";

   /**
    * The IMU mounting frame. Third of the three, and the dangerous one.
    * <p>
    * Its offset from {@link #PELVIS_LINK} is an unverified CAD number. At {@code ω = 1 rad/s} and
    * {@code r = 0.1 m} the {@code ω × r} term is 0.1 m/s -- enough to swamp a comparison against an
    * estimator whose baseline is 0.025 m/s, and it reads as an estimator regression rather than as
    * a bookkeeping error (FRAMEWORK.md §13).
    * </p>
    */
   public static final String PELVIS_IMU = "pelvisImu";
}
