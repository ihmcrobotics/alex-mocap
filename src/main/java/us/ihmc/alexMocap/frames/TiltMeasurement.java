package us.ihmc.alexMocap.frames;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * The F8 world tilt {@code θ} (FRAMEWORK.md §11): where gravity actually points in Motive's world
 * frame.
 *
 * <h2>Measured, never assumed</h2>
 * <p>
 * Motive's world frame is level only to the accuracy of its ground-plane calibration, and the
 * residual is <b>systematic and ungated</b>: it does not average out over captures, no gate in
 * §15 looks at it, and it lands directly in CoM height as {@code ||c|| · sin(θ)} -- about 7 mm at
 * 0.5° with {@code ||c|| = 0.8 m}, against a 0.1° target.
 * </p>
 * <p>
 * §11 requires {@code θ} to be measured -- plumb line, precision level, or a long static IMU
 * average -- and never assumed. This type enforces that socially rather than technically: there is
 * no default constructor and no zero-argument "level" instance. The only way to get an untilted
 * measurement is {@link #assumedLevel(String)}, which demands a written justification, reports
 * {@link #isMeasured()} as {@code false}, and prints as {@code ASSUMED_LEVEL} everywhere it
 * appears. A number that is a guess should look like a guess in the report.
 * </p>
 *
 * <h2>Why a direction and not a scalar</h2>
 * <p>
 * §11 writes the error in terms of a single angle {@code θ}, which is right for budgeting: the
 * height error depends only on the magnitude. But <b>correcting</b> the tilt needs its direction
 * too -- a 0.5° tilt toward +x and a 0.5° tilt toward +y produce the same 7 mm of height error and
 * completely different horizontal CoM positions. So the measurement carried here is the full unit
 * vector: where "up" really is, expressed in Motive's coordinates. {@link #getTiltMagnitude()}
 * recovers §11's scalar from it.
 * </p>
 *
 * <h2>A capture-session constant</h2>
 * <p>
 * Immutable. It changes when the mocap ground-plane calibration changes and at no other time, so
 * it is recorded once per session and travels with the calibration in
 * {@code CalibrationResult.Provenance}.
 * </p>
 */
public final class TiltMeasurement
{
   /** How the tilt was established. Recorded so a report can say, not merely imply. */
   public enum Method
   {
      /** A plumb line sighted against the mocap volume. */
      PLUMB_LINE,
      /** A precision level on a surface with known relation to the mocap frame. */
      PRECISION_LEVEL,
      /** A long static average of an IMU's accelerometer, resolved into the mocap frame. */
      STATIC_IMU_AVERAGE,
      /**
       * Not measured. FRAMEWORK.md §11 forbids this for real work; it exists so that "nobody
       * measured it" is representable and visible, rather than being spelled {@code 0.0} and
       * indistinguishable from a measurement that came out level.
       */
      ASSUMED_LEVEL
   }

   /** Unit vector along true up, expressed in Motive's world frame. */
   private final Vector3D upInMotiveWorld = new Vector3D();
   private final Method method;
   private final String note;

   private TiltMeasurement(Vector3DReadOnly upInMotiveWorld, Method method, String note)
   {
      double norm = upInMotiveWorld.norm();

      if (!Double.isFinite(norm) || norm < 1.0e-9)
         throw new IllegalArgumentException("The measured up direction must be a non-degenerate vector, was " + upInMotiveWorld + ".");

      this.upInMotiveWorld.setAndScale(1.0 / norm, upInMotiveWorld);
      this.method = method;
      this.note = note == null ? "" : note;

      if (this.upInMotiveWorld.getZ() <= 0.0)
         throw new IllegalArgumentException("The measured up direction points into Motive's lower half-space (" + upInMotiveWorld
               + "). A tilt of 90° or more is a sign convention error or an axis-order mismatch, not a ground-plane residual.");
   }

   /**
    * @param upInMotiveWorld where true up points, in Motive's coordinates. Need not be normalised.
    *                        For an accelerometer average, this is the negated specific force.
    */
   public static TiltMeasurement fromMeasuredUp(Vector3DReadOnly upInMotiveWorld, Method method, String note)
   {
      if (method == Method.ASSUMED_LEVEL)
         throw new IllegalArgumentException("Use assumedLevel(note) for an unmeasured tilt; passing a direction with ASSUMED_LEVEL is a contradiction.");

      return new TiltMeasurement(upInMotiveWorld, method, note);
   }

   /**
    * Tilt from the two small angles it is usually read off as.
    *
    * @param tiltAboutXRadians rotation of Motive's frame about its {@code +x}, radians.
    * @param tiltAboutYRadians rotation of Motive's frame about its {@code +y}, radians.
    */
   public static TiltMeasurement fromTiltAngles(double tiltAboutXRadians, double tiltAboutYRadians, Method method, String note)
   {
      // Up in Motive's coordinates after tilting the frame by these angles. Exact, not
      // small-angle: the small-angle form is fine at 0.1° but there is no reason to bake in an
      // approximation whose error is invisible when someone later types 5° to test something.
      RigidBodyTransform tilt = new RigidBodyTransform();
      tilt.getRotation().setYawPitchRoll(0.0, tiltAboutYRadians, tiltAboutXRadians);

      Vector3D up = new Vector3D(0.0, 0.0, 1.0);
      tilt.inverseTransform(up);

      return fromMeasuredUp(up, method, note);
   }

   /**
    * An explicitly unmeasured tilt. FRAMEWORK.md §11 forbids this for real work.
    *
    * @param note why nobody measured it. Required, and it shows up in reports.
    */
   public static TiltMeasurement assumedLevel(String note)
   {
      if (note == null || note.isBlank())
         throw new IllegalArgumentException("An assumed-level tilt needs a written justification. FRAMEWORK.md §11: θ must be measured, never assumed.");

      return new TiltMeasurement(new Vector3D(0.0, 0.0, 1.0), Method.ASSUMED_LEVEL, note);
   }

   public Vector3DReadOnly getUpInMotiveWorld()
   {
      return upInMotiveWorld;
   }

   public Method getMethod()
   {
      return method;
   }

   public String getNote()
   {
      return note;
   }

   /** Whether this came from an instrument rather than from an assumption. */
   public boolean isMeasured()
   {
      return method != Method.ASSUMED_LEVEL;
   }

   /** {@code θ}: the angle between Motive's {@code +z} and true up, radians. §11's scalar. */
   public double getTiltMagnitude()
   {
      // Clamped by hand rather than with Math.clamp, which is Java 21 and this toolchain is 17.
      // The clamp matters: normalisation can leave getZ() a few ulps above 1.0, and acos of that
      // is NaN, which would propagate into the CoM correction as a silent poisoning.
      return Math.acos(Math.max(-1.0, Math.min(1.0, upInMotiveWorld.getZ())));
   }

   /**
    * {@code ||c|| · sin(θ)}: the CoM height error this tilt causes, for a CoM at the given distance
    * from the world origin. FRAMEWORK.md §11's budget, as a function rather than as a comment.
    */
   public double getComHeightError(double comDistanceFromOrigin)
   {
      return comDistanceFromOrigin * Math.sin(getTiltMagnitude());
   }

   /**
    * The transform taking a point expressed in Motive's world frame into the gravity-aligned world
    * frame: {@code p_Wg = T · p_W}.
    * <p>
    * Pure rotation. The two frames share an origin -- a tilt is an orientation error, and inventing
    * a translation for it would silently move the world.
    * </p>
    * <p>
    * Constructed as the minimal rotation carrying the measured up direction onto {@code +z}:
    * axis {@code = up × ẑ}, angle {@code = θ}. Minimal is the right choice because a tilt
    * measurement constrains two degrees of freedom, not three -- nothing in a plumb line or an
    * accelerometer average says anything about heading, so the correction must not invent any.
    * </p>
    */
   public void packMotiveWorldToGravityAligned(RigidBodyTransform toPack)
   {
      toPack.setToZero();

      double axisX = upInMotiveWorld.getY();
      double axisY = -upInMotiveWorld.getX();
      double axisNorm = Math.hypot(axisX, axisY);

      // up is already along +z to within rounding: no rotation, and no axis to rotate about.
      if (axisNorm < 1.0e-12)
         return;

      toPack.getRotation().set(new AxisAngle(axisX / axisNorm, axisY / axisNorm, 0.0, getTiltMagnitude()));
   }

   @Override
   public String toString()
   {
      return "TiltMeasurement[" + method + ", θ=" + String.format("%.4f", Math.toDegrees(getTiltMagnitude())) + "°"
            + (note.isEmpty() ? "" : ", " + note) + "]";
   }
}
