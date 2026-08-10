package us.ihmc.alexMocap.scs2;

import java.util.List;

import us.ihmc.alexMocap.core.GroundTruthSample;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Live telemetry for one {@link GroundTruthSample}: CoM, pelvis pose, and the per-link conditioning
 * that says whether to believe either.
 *
 * <h2>Conditioning is exposed, not just the answer</h2>
 * <p>
 * FRAMEWORK.md §9 requires {@code σ₃} and the visible marker count logged <b>every frame, for every
 * cluster</b>. It would be easy to publish only the CoM and the pelvis pose -- they are what anyone
 * asked for -- and that is exactly how a rank-deficient frame becomes a trusted number in someone's
 * plot. Every link gets three variables here, and the refusal flag is one of them.
 * </p>
 * <p>
 * {@code σ₃} is in <b>m²</b>, not m: it is a mean-squared spread, so a 140 mm cluster reads about
 * {@code 0.003}. The variable name carries the unit for that reason.
 * </p>
 *
 * <h2>No velocity variable, deliberately</h2>
 * <p>
 * There is no pelvis velocity here for the same reason {@code runtime.PelvisStateExtractor} has no
 * accessor for one (§13). A YoVariable named {@code pelvisLinearVelocity} would be plotted,
 * compared against the estimator, and would show 0.13 m/s of differencing noise against a
 * 0.025 m/s baseline. Velocity is an offline second pass; if you want it on a plot, run
 * {@code postprocess.PelvisTwistEstimator} over the log and plot that.
 * </p>
 */
public class GroundTruthYoVariables
{
   private final YoRegistry registry;

   private final YoFramePoint3D centerOfMass;
   private final YoFramePose3D pelvisPose;
   private final YoBoolean centerOfMassValid;

   private final List<String> linkNames;
   private final YoDouble[] sigma3;
   private final YoInteger[] visibleCount;
   private final YoBoolean[] poseAccepted;
   private final YoInteger refusalCount;

   /**
    * @param namePrefix          prefix for every variable, so two instances can coexist.
    * @param linkNames           the links to report conditioning for.
    * @param gravityAlignedWorld {@code Wg}: the frame the CoM and pelvis pose are expressed in.
    */
   public GroundTruthYoVariables(String namePrefix, List<String> linkNames, ReferenceFrame gravityAlignedWorld)
   {
      this.registry = new YoRegistry(namePrefix + "GroundTruth");
      this.linkNames = List.copyOf(linkNames);

      this.centerOfMass = new YoFramePoint3D(namePrefix + "Com", gravityAlignedWorld, registry);
      this.pelvisPose = new YoFramePose3D(namePrefix + "PelvisPose", gravityAlignedWorld, registry);
      this.centerOfMassValid = new YoBoolean(namePrefix + "ComValid", registry);
      this.refusalCount = new YoInteger(namePrefix + "RefusedLinkCount", registry);

      this.sigma3 = new YoDouble[linkNames.size()];
      this.visibleCount = new YoInteger[linkNames.size()];
      this.poseAccepted = new YoBoolean[linkNames.size()];

      for (int i = 0; i < linkNames.size(); i++)
      {
         String link = capitalise(linkNames.get(i));
         sigma3[i] = new YoDouble(namePrefix + link + "Sigma3SquaredMetres", registry);
         visibleCount[i] = new YoInteger(namePrefix + link + "VisibleMarkers", registry);
         poseAccepted[i] = new YoBoolean(namePrefix + link + "PoseAccepted", registry);
      }
   }

   private static String capitalise(String text)
   {
      return text.isEmpty() ? text : Character.toUpperCase(text.charAt(0)) + text.substring(1);
   }

   /** Publishes one frame. */
   public void update(GroundTruthSample sample)
   {
      centerOfMass.set(sample.getCenterOfMass());
      centerOfMassValid.set(!sample.getCenterOfMass().containsNaN());
      pelvisPose.set(sample.getPelvisPose());

      int refused = 0;

      for (int i = 0; i < linkNames.size(); i++)
      {
         int index = sample.indexOfLink(linkNames.get(i));
         sigma3[i].set(sample.getSigma3(index));
         visibleCount[i].set(sample.getVisibleCount(index));
         poseAccepted[i].set(sample.isPoseAccepted(index));

         if (!sample.isPoseAccepted(index))
            refused++;
      }

      refusalCount.set(refused);
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public YoFramePoint3D getCenterOfMass()
   {
      return centerOfMass;
   }

   public YoFramePose3D getPelvisPose()
   {
      return pelvisPose;
   }

   public YoBoolean getCenterOfMassValid()
   {
      return centerOfMassValid;
   }

   public List<String> getLinkNames()
   {
      return linkNames;
   }

   public YoDouble getSigma3(int linkIndex)
   {
      return sigma3[linkIndex];
   }

   public YoInteger getVisibleCount(int linkIndex)
   {
      return visibleCount[linkIndex];
   }

   public YoBoolean getPoseAccepted(int linkIndex)
   {
      return poseAccepted[linkIndex];
   }
}
