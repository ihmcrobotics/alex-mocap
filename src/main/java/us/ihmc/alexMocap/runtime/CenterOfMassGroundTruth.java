package us.ihmc.alexMocap.runtime;

import java.util.List;

import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * F9, whole-body CoM (FRAMEWORK.md §12):
 *
 * <pre>
 * ^Wg c  =  (1/M) · sum_i  m_i · ^Wg T̂_i · ^i c_i          M = sum_i m_i
 * </pre>
 *
 * <p>
 * with {@code m_i} and {@code ^i c_i} from the URDF inertial blocks -- <b>assumed</b>, and per §14
 * the dominant term in the whole error budget.
 * </p>
 *
 * <h2>Why this is not Mecano's {@code CenterOfMassCalculator}</h2>
 * <p>
 * §12 says "Mecano's {@code CenterOfMassCalculator} implements this. Do not reimplement it; feed it
 * the measured poses." The instruction is right in spirit and not implementable as written, so it
 * is worth being explicit about what happened instead.
 * </p>
 * <p>
 * {@code CenterOfMassCalculator} computes the CoM of a multi-body system from <b>the tree's own
 * frames</b>, which are a function of its joint configuration. There is no way to hand it a set of
 * independently measured link poses, because in general <i>no joint configuration produces them</i>
 * -- F6 measures each marked link separately, so the poses need not be kinematically consistent,
 * and their inconsistency is exactly the signal the whole method exists to expose. Feeding the
 * calculator a joint configuration instead would make this an FK result, and F9 would be measuring
 * the URDF rather than the robot.
 * </p>
 * <p>
 * So the sum above is written out directly -- five lines, no cleverness -- and Mecano's calculator
 * is used as the <b>oracle in the test</b>: on a configuration where the measured poses <i>are</i>
 * FK-consistent, the two must agree to {@code 1e-9}. That is the check §12 was reaching for, and it
 * is stronger as a test than as an implementation.
 * </p>
 *
 * <h2>A refused link is not a zero-mass link</h2>
 * <p>
 * If a cluster goes rank-deficient and F6 refuses, that link has no pose and cannot contribute. The
 * dishonest options are to skip it (silently computing the CoM of a lighter robot, biased toward
 * whatever remains) or to substitute its last pose (a stale value that looks plausible). This class
 * takes neither: it returns {@code false} and packs NaN, and reports which links were missing. A
 * frame with an incomplete CoM is a frame with no CoM.
 * </p>
 *
 * <h2>The F8 correction is applied here, not by the caller</h2>
 * <p>
 * F6 produces poses in Motive's world frame {@code W}, and §12 wants the CoM in the
 * gravity-aligned frame {@code Wg}. Those differ by the measured world tilt, worth about 7 mm of
 * CoM <i>height</i> at 0.5° -- which is to say, worth more than everything else in this class.
 * </p>
 * <p>
 * An earlier version of this took the poses and returned the CoM in whatever frame they arrived in,
 * leaving the caller to apply the tilt. That is exactly the arrangement §11 spends a paragraph
 * warning against: "a correction applied at call sites can be forgotten at call sites", and
 * forgetting it produces no error, just a CoM that is quietly ~7 mm low. It was in fact forgotten,
 * in {@code ReplayRunner}, within an hour of the class being written. So the transform is a
 * constructor argument now and the output is in {@code Wg} by construction.
 * </p>
 *
 * <h2>Contract</h2>
 * <p>
 * Stateful and not thread safe; allocation-free per frame.
 * </p>
 */
public class CenterOfMassGroundTruth
{
   private final RobotModelHandle model;
   private final List<String> linkNames;
   private final double[] masses;
   private final Point3D[] centersOfMassInLinkFrame;
   private final double totalMass;

   private final Point3D contribution = new Point3D();
   private final RigidBodyTransform motiveWorldToGravityAligned = new RigidBodyTransform();
   private int missingLinkCount;
   private double missingMass;

   /**
    * @param linkNames every link that carries mass, in the order the {@link MeasuredLinkPoses} fed
    *                  to {@link #compute} reports them. Omitting a massive link silently biases the
    *                  result, so this is checked against the model's total mass.
    */
   public CenterOfMassGroundTruth(RobotModelHandle model, List<String> linkNames, ReferenceFrame motiveWorld, ReferenceFrame gravityAlignedWorld)
   {
      motiveWorld.getTransformToDesiredFrame(motiveWorldToGravityAligned, gravityAlignedWorld);

      this.model = model;
      this.linkNames = List.copyOf(linkNames);
      this.masses = new double[linkNames.size()];
      this.centersOfMassInLinkFrame = new Point3D[linkNames.size()];

      double sum = 0.0;

      for (int i = 0; i < linkNames.size(); i++)
      {
         masses[i] = model.getMass(linkNames.get(i));
         centersOfMassInLinkFrame[i] = new Point3D();
         model.packCenterOfMassInLinkFrame(linkNames.get(i), centersOfMassInLinkFrame[i]);
         sum += masses[i];
      }

      this.totalMass = sum;

      if (Math.abs(totalMass - model.getTotalMass()) > 1.0e-9)
         throw new IllegalArgumentException(String.format("The listed links carry %.6f kg but the robot masses %.6f kg. A link left out of this list is "
               + "silently omitted from every CoM, biasing it toward whatever remains.", totalMass, model.getTotalMass()));
   }

   /** Every link of the robot, which is what F9 needs. */
   public static CenterOfMassGroundTruth forWholeRobot(RobotModelHandle model, ReferenceFrame motiveWorld, ReferenceFrame gravityAlignedWorld)
   {
      return new CenterOfMassGroundTruth(model, model.getLinkNames(), motiveWorld, gravityAlignedWorld);
   }

   /**
    * {@code ^Wg c} from the measured link poses, <b>in the gravity-aligned world frame</b>.
    *
    * @param toPack packed with the CoM, or with NaN if any link had no pose.
    * @return whether every link contributed. <b>False means there is no CoM for this frame</b>, not
    *         that the packed value is slightly worse.
    */
   public boolean compute(MeasuredLinkPoses poses, Point3DBasics toPack)
   {
      missingLinkCount = 0;
      missingMass = 0.0;

      toPack.setToZero();

      for (int i = 0; i < linkNames.size(); i++)
      {
         int linkIndex = poses.indexOf(linkNames.get(i));

         if (!poses.isAvailable(linkIndex))
         {
            missingLinkCount++;
            missingMass += masses[i];
            continue;
         }

         contribution.set(centersOfMassInLinkFrame[i]);
         poses.getPose(linkIndex).transform(contribution);
         contribution.scale(masses[i]);
         toPack.add(contribution);
      }

      if (missingLinkCount > 0)
      {
         toPack.setToNaN();
         return false;
      }

      toPack.scale(1.0 / totalMass);

      // F8. Applied here so that no caller can omit it (§11).
      motiveWorldToGravityAligned.transform(toPack);
      return true;
   }

   /** {@code M}. From the URDF -- weigh the robot (FRAMEWORK.md §14) and this becomes checkable. */
   public double getTotalMass()
   {
      return totalMass;
   }

   /** Links that had no pose in the last {@link #compute}. */
   public int getMissingLinkCount()
   {
      return missingLinkCount;
   }

   /** Mass of those links. The size of what was unmeasurable, in the units that matter. */
   public double getMissingMass()
   {
      return missingMass;
   }

   public RobotModelHandle getModel()
   {
      return model;
   }
}
