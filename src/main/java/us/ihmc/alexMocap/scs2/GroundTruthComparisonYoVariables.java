package us.ihmc.alexMocap.scs2;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * Mocap-derived centre of mass against the simulation's actual one, as YoVariables and two spheres.
 *
 * <h2>What this is measuring</h2>
 * <p>
 * Inside a simulation the true CoM is known exactly, which is the one place this project can ask
 * whether the mocap chain is right rather than merely self-consistent. Everything upstream --
 * marker cloud, Umeyama, chaining, weighted sum -- is collapsed into one number here:
 * {@link #getComErrorMagnitude()}.
 * </p>
 * <p>
 * It is the <b>simulation</b> that is ground truth in this comparison, and the mocap estimate that
 * is on trial. That inverts the naming used everywhere else in this project, where "ground truth"
 * means the mocap answer being handed to the estimator, so the variables say {@code actual} and
 * {@code mocap} rather than anything containing the words "truth" or "estimate".
 * </p>
 *
 * <h2>The error is separated into offset and jitter, deliberately</h2>
 * <p>
 * A constant CoM offset and a noisy CoM are different faults with different causes, and a single
 * magnitude cannot tell them apart. A constant offset is a modelling error -- wrong link masses, a
 * missed F8 tilt correction, a calibration bias -- and it does not average away. Jitter is marker
 * noise reaching the sum, and it does. {@link #getComErrorMean()} and
 * {@link #getComErrorStandardDeviation()} are accumulated online so the two are readable without
 * post-processing a log.
 * </p>
 * <p>
 * This project's own history is the argument for it: a forgotten F8 correction once biased every CoM
 * 7 mm low, which is invisible in a magnitude trace that is already moving by millimetres, and
 * unmissable in a mean.
 * </p>
 *
 * <h2>NaN is propagated, not smoothed over</h2>
 * <p>
 * When a cluster is refused there is no CoM for that frame, and the error is NaN rather than the
 * last good value. NaN frames are excluded from the running statistics and counted separately in
 * {@link #getRefusedFrameCount()} -- a mean silently computed over only the frames that worked is
 * exactly the "green number with no conditioning attached" this project keeps finding.
 * </p>
 */
public class GroundTruthComparisonYoVariables
{
   /** Radius of the two CoM spheres, metres. */
   public static final double COM_RADIUS = 0.03;

   private final YoRegistry registry;

   private final YoFramePoint3D mocapCom;
   private final YoFramePoint3D actualCom;
   private final YoFrameVector3D comError;
   private final YoDouble comErrorMagnitude;

   private final YoDouble comErrorMean;
   private final YoDouble comErrorStandardDeviation;
   private final YoDouble comErrorMaximum;
   private final YoBoolean comValid;
   private final YoDouble validFrameCount;
   private final YoDouble refusedFrameCount;

   private final Vector3D error = new Vector3D();

   // Online accumulation of the magnitude's first two moments. Sum-of-squares rather than Welford
   // because the magnitudes are millimetres about zero -- there is no cancellation to guard against
   // here, and the simpler form is one the reader can check against the variable names.
   private double magnitudeSum;
   private double magnitudeSquaredSum;

   /**
    * @param namePrefix prefix for every variable, so two instances can coexist.
    * @param world      the frame both centres of mass are expressed in. Both must be in the
    *                   <b>same</b> frame -- comparing a gravity-aligned mocap CoM against a
    *                   simulation CoM in the raw world frame silently reports the tilt as an error.
    */
   public GroundTruthComparisonYoVariables(String namePrefix, ReferenceFrame world)
   {
      this.registry = new YoRegistry(namePrefix + "MocapVsActual");

      this.mocapCom = new YoFramePoint3D(namePrefix + "MocapCom", world, registry);
      this.actualCom = new YoFramePoint3D(namePrefix + "ActualCom", world, registry);
      this.comError = new YoFrameVector3D(namePrefix + "MocapMinusActualCom", world, registry);
      this.comErrorMagnitude = new YoDouble(namePrefix + "MocapMinusActualComMagnitude", registry);

      this.comErrorMean = new YoDouble(namePrefix + "MocapMinusActualComMean", registry);
      this.comErrorStandardDeviation = new YoDouble(namePrefix + "MocapMinusActualComStandardDeviation", registry);
      this.comErrorMaximum = new YoDouble(namePrefix + "MocapMinusActualComMaximum", registry);
      this.comValid = new YoBoolean(namePrefix + "MocapComValid", registry);
      this.validFrameCount = new YoDouble(namePrefix + "MocapComValidFrames", registry);
      this.refusedFrameCount = new YoDouble(namePrefix + "MocapComRefusedFrames", registry);

      reset();
   }

   /** Clears the running statistics. The per-frame variables are left alone. */
   public void reset()
   {
      magnitudeSum = 0.0;
      magnitudeSquaredSum = 0.0;
      validFrameCount.set(0.0);
      refusedFrameCount.set(0.0);
      comErrorMean.setToNaN();
      comErrorStandardDeviation.setToNaN();
      comErrorMaximum.setToNaN();
   }

   /**
    * Publishes one frame.
    *
    * @param mocapCenterOfMass  the mocap chain's answer, or a point containing NaN if this frame was
    *                           refused.
    * @param actualCenterOfMass the simulation's own centre of mass, in the same frame.
    */
   public void update(Point3DReadOnly mocapCenterOfMass, Point3DReadOnly actualCenterOfMass)
   {
      actualCom.set(actualCenterOfMass);

      boolean valid = !mocapCenterOfMass.containsNaN();
      comValid.set(valid);

      if (!valid)
      {
         // Everything derived from a refused frame goes NaN with it. Holding the previous error
         // would draw a flat line through a dropout, which reads as a well-behaved estimator.
         mocapCom.setToNaN();
         comError.setToNaN();
         comErrorMagnitude.setToNaN();
         refusedFrameCount.add(1.0);
         return;
      }

      mocapCom.set(mocapCenterOfMass);

      error.sub(mocapCenterOfMass, actualCenterOfMass);
      comError.set(error);

      double magnitude = error.norm();
      comErrorMagnitude.set(magnitude);

      magnitudeSum += magnitude;
      magnitudeSquaredSum += magnitude * magnitude;
      validFrameCount.add(1.0);

      double count = validFrameCount.getValue();
      double mean = magnitudeSum / count;
      comErrorMean.set(mean);

      // Population variance, clamped at zero: with count == 1 the expression is exactly zero up to
      // round-off, and a -1e-19 under the square root would publish NaN on the first frame.
      double variance = Math.max(0.0, magnitudeSquaredSum / count - mean * mean);
      comErrorStandardDeviation.set(Math.sqrt(variance));

      if (!(magnitude <= comErrorMaximum.getValue()))
         comErrorMaximum.set(magnitude);
   }

   /**
    * Both centres of mass, so the offset is visible in the 3-D view and not only on a plot.
    * <p>
    * Gold is the mocap answer -- the same colour {@link GroundTruthYoGraphics} uses for it, so the
    * two views agree -- and green is the simulation's. At a millimetre of error the spheres overlap
    * completely, which is the point: they separate visibly exactly when something is wrong.
    * </p>
    *
    * @return a group to hand to {@code SimulationSession.addYoGraphicDefinition}.
    */
   public YoGraphicGroupDefinition createYoGraphics(String name)
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.setChildren(List.of(YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + "MocapCom", mocapCom, COM_RADIUS, ColorDefinitions.Gold()),
                                YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + "ActualCom", actualCom, COM_RADIUS, ColorDefinitions.LimeGreen())));
      return group;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public YoFramePoint3D getMocapCom()
   {
      return mocapCom;
   }

   public YoFramePoint3D getActualCom()
   {
      return actualCom;
   }

   /** {@code mocap - actual}, per axis. The axis breakdown is what names the fault. */
   public YoFrameVector3D getComError()
   {
      return comError;
   }

   /** {@code |mocap - actual|}. NaN on a refused frame. */
   public YoDouble getComErrorMagnitude()
   {
      return comErrorMagnitude;
   }

   /** Running mean of the magnitude over valid frames. A non-zero floor here is a bias, not noise. */
   public YoDouble getComErrorMean()
   {
      return comErrorMean;
   }

   public YoDouble getComErrorStandardDeviation()
   {
      return comErrorStandardDeviation;
   }

   public YoDouble getComErrorMaximum()
   {
      return comErrorMaximum;
   }

   public YoBoolean getComValid()
   {
      return comValid;
   }

   /** Frames excluded from the statistics because the mocap chain refused them. */
   public YoDouble getRefusedFrameCount()
   {
      return refusedFrameCount;
   }

   public YoDouble getValidFrameCount()
   {
      return validFrameCount;
   }

   /** Convenience for a console summary at the end of a run. */
   public String summary()
   {
      Point3D mocap = new Point3D(mocapCom);
      return String.format("CoM error: mean %.3f mm, sd %.3f mm, max %.3f mm over %.0f valid frames (%.0f refused); last mocap CoM %s",
                           1000.0 * comErrorMean.getValue(),
                           1000.0 * comErrorStandardDeviation.getValue(),
                           1000.0 * comErrorMaximum.getValue(),
                           validFrameCount.getValue(),
                           refusedFrameCount.getValue(),
                           mocap);
   }
}
