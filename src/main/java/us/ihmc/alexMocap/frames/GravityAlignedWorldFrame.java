package us.ihmc.alexMocap.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * F8 (FRAMEWORK.md §11): the gravity-aligned world frame {@code Wg}, and Motive's tilted world
 * frame {@code W} hanging beneath it.
 *
 * <h2>A frame node, not a correction function</h2>
 * <p>
 * §11 is explicit about this and the reason is worth restating. A correction applied at call sites
 * can be forgotten at call sites, and forgetting it produces no error -- just a CoM that is ~7 mm
 * low at 0.5° of tilt, which is well inside the range where a number looks plausible. Modelled as
 * a frame instead, anything expressed in {@code Wg} is correct by construction, and asking for a
 * quantity in the wrong frame throws rather than quietly returning the uncorrected value.
 * </p>
 * <p>
 * The tree is built the way §11 describes it, with {@code Wg} as the <b>parent</b> of {@code W}:
 * </p>
 *
 * <pre>
 * (Euclid root)
 *   └─ gravityAlignedWorld   Wg     the room's true frame; CoM height is measured here
 *        └─ motiveWorld      W      what the cameras report; tilted by θ inside Wg
 * </pre>
 *
 * <p>
 * That direction is the physically honest one: the room does not tilt, Motive's estimate of it
 * does. Raw marker positions are created in {@code W}; {@code changeFrame(Wg)} is the entire
 * correction, and it is one call that cannot be half-applied.
 * </p>
 *
 * <h2>Why the frames are not static</h2>
 * <p>
 * The tilt is a capture-session constant, which invites making these two frames global constants.
 * They are instance fields instead, because a session constant is not a process constant: a test
 * suite covers several tilts, and a replay tool may open logs from two sessions. A static frame
 * would make the second one silently inherit the first one's tilt.
 * </p>
 */
public class GravityAlignedWorldFrame
{
   private final TiltMeasurement tilt;
   private final ReferenceFrame gravityAlignedWorld;
   private final ReferenceFrame motiveWorld;

   public GravityAlignedWorldFrame(TiltMeasurement tilt)
   {
      this(tilt, ReferenceFrame.getWorldFrame(), "");
   }

   /**
    * @param tilt       the measured world tilt. FRAMEWORK.md §11: measured, never assumed.
    * @param parent     the frame {@code Wg} is attached to, with an identity transform.
    * @param nameSuffix appended to both frame names. Euclid rejects duplicate frame names under one
    *                   parent, so anything constructing more than one of these in a process -- a
    *                   test suite, a tool comparing two sessions -- needs distinct names. Empty for
    *                   production, where there is exactly one.
    */
   public GravityAlignedWorldFrame(TiltMeasurement tilt, ReferenceFrame parent, String nameSuffix)
   {
      if (tilt == null)
         throw new IllegalArgumentException("A tilt measurement is required. FRAMEWORK.md §11 has no default.");

      this.tilt = tilt;

      gravityAlignedWorld = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(FrameNames.GRAVITY_ALIGNED_WORLD + nameSuffix,
                                                                                              parent,
                                                                                              new RigidBodyTransform());

      RigidBodyTransform motiveToGravityAligned = new RigidBodyTransform();
      tilt.packMotiveWorldToGravityAligned(motiveToGravityAligned);

      motiveWorld = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(FrameNames.MOTIVE_WORLD + nameSuffix,
                                                                                      gravityAlignedWorld,
                                                                                      motiveToGravityAligned);
   }

   /**
    * {@code Wg}. Everything compared against gravity belongs here: CoM height above all
    * (FRAMEWORK.md §12).
    */
   public ReferenceFrame getGravityAlignedWorld()
   {
      return gravityAlignedWorld;
   }

   /**
    * {@code W}, exactly as the cameras report it. Raw marker positions belong here and nowhere
    * else.
    */
   public ReferenceFrame getMotiveWorld()
   {
      return motiveWorld;
   }

   public TiltMeasurement getTiltMeasurement()
   {
      return tilt;
   }

   /**
    * Whether this frame rests on a measurement or on an assumption. A calibration run with this
    * {@code false} has an ungated systematic error in it, and the report should say so.
    */
   public boolean isTiltMeasured()
   {
      return tilt.isMeasured();
   }

   @Override
   public String toString()
   {
      return "GravityAlignedWorldFrame[" + tilt + "]";
   }
}
