package us.ihmc.alexMocap.postprocess;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * F11, the CoM error budget (FRAMEWORK.md §14). Differentiating F9:
 *
 * <pre>
 * δ(^W c)  ≈  (1/M) · sum_i [
 *       δm_i · ( ^W T_i ^i c_i  -  ^W c )      ← mass error × lever arm
 *     + m_i  · ^W R_i · δ(^i c_i)              ← link-CoM error
 *     + m_i  · ( δ ^W T_i ) · ^i c_i           ← pose error (mocap)
 * ]
 * </pre>
 *
 * <h2>The conclusion this exists to make concrete</h2>
 * <p>
 * <b>The third term is the only one this pipeline controls, and mocap drives it sub-millimetre.</b>
 * The first two are CAD-sourced and enter weighted by full link mass and lever arm; for a heavy
 * link with a long moment arm they dominate the third by orders of magnitude. So the calibration
 * validates <i>pose</i> tightly and leaves CoM exactly as good as the URDF. Marking a link gives
 * you its pose, never its CoM.
 * </p>
 * <p>
 * That is an uncomfortable result for a project whose deliverable is CoM ground truth, which is why
 * it is worth computing rather than quoting. §14's cheapest available check is to <b>weigh the
 * robot</b> -- it constrains {@code sum_i δm_i} for free, in minutes -- and the only thing that
 * converts the first two terms from assumed to measured is the SESC / force-plate stage of §16.
 * </p>
 *
 * <h2>Plain arrays in, no model</h2>
 * <p>
 * This takes masses, link-frame CoMs and link poses as arrays rather than a {@code RobotModelHandle}
 * and {@code MeasuredLinkPoses}. FRAMEWORK.md §19 allows {@code postprocess} to see {@code core}
 * and nothing else, and F11 is pure arithmetic on numbers -- there is no reason for it to reach for
 * a robot model to do it. The caller assembles the arrays.
 * </p>
 *
 * <h2>{@code sum δm_i = 0}, and what happens when it is not</h2>
 * <p>
 * §14's first term assumes the total mass is pinned by a scale reading, so the mass errors sum to
 * zero. That assumption is load-bearing: it is what makes the term depend on the <i>lever arm</i>
 * {@code (r_i - c)} rather than on absolute position. {@link #packShiftFromMassErrors} therefore
 * checks it, and {@link #packExactShiftFromMassErrors} exists for the unpinned case, which needs
 * the exact expression instead.
 * </p>
 */
public class COMErrorBudget
{
   private final double[] masses;
   private final Point3D[] centersOfMassInLinkFrame;
   private final double totalMass;

   /**
    * @param masses                   {@code m_i} per link, kg.
    * @param centersOfMassInLinkFrame {@code ^i c_i} per link, in that link's own frame.
    */
   public COMErrorBudget(double[] masses, Point3DReadOnly[] centersOfMassInLinkFrame)
   {
      if (masses.length != centersOfMassInLinkFrame.length)
         throw new IllegalArgumentException("Got " + masses.length + " masses and " + centersOfMassInLinkFrame.length + " link CoMs.");

      this.masses = masses.clone();
      this.centersOfMassInLinkFrame = new Point3D[masses.length];

      double sum = 0.0;

      for (int i = 0; i < masses.length; i++)
      {
         this.centersOfMassInLinkFrame[i] = new Point3D(centersOfMassInLinkFrame[i]);
         sum += masses[i];
      }

      this.totalMass = sum;

      if (!(totalMass > 0.0))
         throw new IllegalArgumentException("Total mass must be positive, was " + totalMass + ".");
   }

   public int getLinkCount()
   {
      return masses.length;
   }

   public double getTotalMass()
   {
      return totalMass;
   }

   /** {@code ^W c} for the given poses: F9, repeated here so the budget is self-contained. */
   public void packCenterOfMass(RigidBodyTransformReadOnly[] linkPoses, Vector3DBasics toPack)
   {
      Point3D contribution = new Point3D();
      toPack.setToZero();

      for (int i = 0; i < masses.length; i++)
      {
         contribution.set(centersOfMassInLinkFrame[i]);
         linkPoses[i].transform(contribution);
         contribution.scale(masses[i]);
         toPack.add(contribution);
      }

      toPack.scale(1.0 / totalMass);
   }

   /**
    * §14's first term: {@code (1/M) sum_i δm_i (r_i - c)}, with the mass errors summing to zero.
    *
    * @throws IllegalArgumentException if {@code sum δm_i} is not zero to within a tolerance. Use
    *                                  {@link #packExactShiftFromMassErrors} when the total mass is
    *                                  not pinned.
    */
   public void packShiftFromMassErrors(RigidBodyTransformReadOnly[] linkPoses, double[] massErrors, Vector3DBasics toPack)
   {
      double sum = 0.0;

      for (double error : massErrors)
         sum += error;

      if (Math.abs(sum) > 1.0e-9 * totalMass)
         throw new IllegalArgumentException(String.format("The mass errors sum to %.6g kg, not zero. §14's first term assumes the total is pinned by a "
               + "scale reading; without that, use packExactShiftFromMassErrors.", sum));

      Vector3D centerOfMass = new Vector3D();
      packCenterOfMass(linkPoses, centerOfMass);

      Point3D linkCenterOfMass = new Point3D();
      toPack.setToZero();

      for (int i = 0; i < masses.length; i++)
      {
         linkCenterOfMass.set(centersOfMassInLinkFrame[i]);
         linkPoses[i].transform(linkCenterOfMass);

         toPack.addX(massErrors[i] * (linkCenterOfMass.getX() - centerOfMass.getX()));
         toPack.addY(massErrors[i] * (linkCenterOfMass.getY() - centerOfMass.getY()));
         toPack.addZ(massErrors[i] * (linkCenterOfMass.getZ() - centerOfMass.getZ()));
      }

      toPack.scale(1.0 / totalMass);
   }

   /**
    * The CoM shift from mass errors <b>without</b> assuming the total is pinned, computed by
    * recomputing the CoM rather than linearising:
    *
    * <pre>
    * δc  =  ( sum_i (m_i + δm_i) r_i ) / ( M + sum δm_i )  -  c
    * </pre>
    *
    * <p>
    * For a single link this reduces to {@code δm (r_j - c) / (M + δm)}, which is §14's first term
    * with {@code M} replaced by {@code M + δm} -- so the two agree to first order in
    * {@code δm/M}, and the difference is a good check that the linearisation is being read
    * correctly rather than a different physics.
    * </p>
    */
   public void packExactShiftFromMassErrors(RigidBodyTransformReadOnly[] linkPoses, double[] massErrors, Vector3DBasics toPack)
   {
      Vector3D nominal = new Vector3D();
      packCenterOfMass(linkPoses, nominal);

      Point3D contribution = new Point3D();
      Vector3D perturbed = new Vector3D();
      double perturbedTotal = 0.0;

      for (int i = 0; i < masses.length; i++)
      {
         contribution.set(centersOfMassInLinkFrame[i]);
         linkPoses[i].transform(contribution);
         contribution.scale(masses[i] + massErrors[i]);
         perturbed.add(contribution);
         perturbedTotal += masses[i] + massErrors[i];
      }

      perturbed.scale(1.0 / perturbedTotal);
      toPack.sub(perturbed, nominal);
   }

   /**
    * §14's second term: {@code (1/M) sum_i m_i ^W R_i δ(^i c_i)}.
    *
    * @param linkComErrors {@code δ(^i c_i)} per link, in that link's own frame.
    */
   public void packShiftFromLinkComErrors(RigidBodyTransformReadOnly[] linkPoses, Vector3DReadOnly[] linkComErrors, Vector3DBasics toPack)
   {
      Vector3D rotated = new Vector3D();
      toPack.setToZero();

      for (int i = 0; i < masses.length; i++)
      {
         rotated.set(linkComErrors[i]);
         linkPoses[i].getRotation().transform(rotated);
         rotated.scale(masses[i]);
         toPack.add(rotated);
      }

      toPack.scale(1.0 / totalMass);
   }

   /**
    * §14's third term for a translational pose error per link:
    * {@code (1/M) sum_i m_i δt_i}.
    *
    * @param poseTranslationErrors {@code δ} of each link's origin, in world.
    */
   public void packShiftFromPoseErrors(Vector3DReadOnly[] poseTranslationErrors, Vector3DBasics toPack)
   {
      Vector3D scaled = new Vector3D();
      toPack.setToZero();

      for (int i = 0; i < masses.length; i++)
      {
         scaled.set(poseTranslationErrors[i]);
         scaled.scale(masses[i]);
         toPack.add(scaled);
      }

      toPack.scale(1.0 / totalMass);
   }

   /**
    * The budget: each term's expected magnitude given uncertainties of the kind a CAD model and a
    * mocap system actually carry.
    *
    * @param massFractionUncertainty per-link mass uncertainty as a fraction, e.g. 0.05 for 5%.
    *                                Applied with the total pinned, since §14's first term assumes
    *                                that and weighing the robot is cheap.
    * @param linkComUncertainty      per-axis uncertainty in {@code ^i c_i}, metres.
    * @param poseUncertainty         per-axis uncertainty in each link's measured pose, metres. This
    *                                is the one the pipeline controls: F6's per-frame {@code σ},
    *                                undiluted.
    */
   public ErrorBudgetReport evaluate(RigidBodyTransformReadOnly[] linkPoses,
                                     double massFractionUncertainty,
                                     double linkComUncertainty,
                                     double poseUncertainty)
   {
      Vector3D centerOfMass = new Vector3D();
      packCenterOfMass(linkPoses, centerOfMass);

      // Term 1. Independent per-link mass errors, each m_i * fraction, in RMS -- with the total
      // pinned, so what survives is the lever-arm weighted spread.
      Point3D linkCenterOfMass = new Point3D();
      double massTermSquared = 0.0;

      for (int i = 0; i < masses.length; i++)
      {
         linkCenterOfMass.set(centersOfMassInLinkFrame[i]);
         linkPoses[i].transform(linkCenterOfMass);

         double leverArm = new Vector3D(linkCenterOfMass.getX() - centerOfMass.getX(),
                                        linkCenterOfMass.getY() - centerOfMass.getY(),
                                        linkCenterOfMass.getZ() - centerOfMass.getZ()).norm();
         double massError = massFractionUncertainty * masses[i];
         massTermSquared += massError * leverArm * massError * leverArm;
      }

      double massTerm = Math.sqrt(massTermSquared) / totalMass;

      // Terms 2 and 3. Independent per-link errors of the given per-axis size; a rotation preserves
      // length, so the link-CoM term needs no pose information at all.
      double comTermSquared = 0.0;
      double poseTermSquared = 0.0;

      for (double mass : masses)
      {
         double perLink = mass * linkComUncertainty * Math.sqrt(3.0);
         comTermSquared += perLink * perLink;

         double perLinkPose = mass * poseUncertainty * Math.sqrt(3.0);
         poseTermSquared += perLinkPose * perLinkPose;
      }

      return new ErrorBudgetReport(massTerm,
                                   Math.sqrt(comTermSquared) / totalMass,
                                   Math.sqrt(poseTermSquared) / totalMass,
                                   totalMass,
                                   massFractionUncertainty,
                                   linkComUncertainty,
                                   poseUncertainty);
   }
}
