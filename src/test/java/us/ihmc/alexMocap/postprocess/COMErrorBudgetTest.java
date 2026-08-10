package us.ihmc.alexMocap.postprocess;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.calibration.SyntheticCaptures;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * F11, the CoM error budget (FRAMEWORK.md §14).
 */
public class COMErrorBudgetTest
{
   private record Robot(COMErrorBudget budget, RigidBodyTransform[] poses, double[] masses, Point3D[] centersOfMass, List<String> linkNames, double totalMass)
   {
   }

   /** Vector3D has no distance(); this is the separation of two displacement vectors. */
   private static double separation(Vector3DReadOnly a, Vector3DReadOnly b)
   {
      return Math.sqrt((a.getX() - b.getX()) * (a.getX() - b.getX()) + (a.getY() - b.getY()) * (a.getY() - b.getY())
            + (a.getZ() - b.getZ()) * (a.getZ() - b.getZ()));
   }

   private static Robot toyRobot() throws Exception
   {
      RobotModelHandle model = SyntheticCaptures.toyModel();
      model.setQ(new double[] {0.3, 0.8, -0.2, -0.4, 1.1, 0.15});
      model.updateFrames();

      List<String> linkNames = model.getLinkNames();
      double[] masses = new double[linkNames.size()];
      Point3D[] centersOfMass = new Point3D[linkNames.size()];
      RigidBodyTransform[] poses = new RigidBodyTransform[linkNames.size()];
      double totalMass = 0.0;

      for (int i = 0; i < linkNames.size(); i++)
      {
         masses[i] = model.getMass(linkNames.get(i));
         centersOfMass[i] = new Point3D();
         model.packCenterOfMassInLinkFrame(linkNames.get(i), centersOfMass[i]);
         poses[i] = new RigidBodyTransform();
         model.packLinkToBase(linkNames.get(i), poses[i]);
         totalMass += masses[i];
      }

      return new Robot(new COMErrorBudget(masses, centersOfMass), poses, masses, centersOfMass, linkNames, totalMass);
   }

   /**
    * PR_PLAN.md: perturb one link mass by 1% and assert the CoM shift matches §14's closed form.
    * <p>
    * Done twice, because §14's first term carries an assumption that decides the answer. With the
    * total mass <b>pinned</b> by a scale reading the errors sum to zero and the term is exactly
    * {@code (1/M) Σ δm_i (r_i - c)}. With a single link perturbed and nothing else changed the
    * total is not pinned, the exact answer is {@code δm (r_j - c) / (M + δm)}, and §14's formula is
    * its first-order approximation -- correct to {@code δm/M}, which is 0.18% here.
    * </p>
    */
   @Test
   public void testMassErrorShiftMatchesTheClosedForm() throws Exception
   {
      Robot robot = toyRobot();
      int perturbed = robot.linkNames().indexOf("l_thigh");
      double delta = 0.01 * robot.masses()[perturbed];

      assertEquals(0.05, delta, 1.0e-12, "1% of the 5 kg thigh.");

      // --- Unpinned: one link perturbed, nothing compensating. ---
      double[] singleError = new double[robot.masses().length];
      singleError[perturbed] = delta;

      Vector3D exact = new Vector3D();
      robot.budget().packExactShiftFromMassErrors(robot.poses(), singleError, exact);

      // The closed form for a single unpinned perturbation: δ (r_j - c) / (M + δ).
      Vector3D nominalCom = new Vector3D();
      robot.budget().packCenterOfMass(robot.poses(), nominalCom);

      Point3D linkCom = new Point3D(robot.centersOfMass()[perturbed]);
      robot.poses()[perturbed].transform(linkCom);

      Vector3D closedForm = new Vector3D(linkCom.getX() - nominalCom.getX(), linkCom.getY() - nominalCom.getY(), linkCom.getZ() - nominalCom.getZ());
      closedForm.scale(delta / (robot.totalMass() + delta));

      assertEquals(0.0, separation(closedForm, exact), 1.0e-12, "The exact recomputation must equal δ(r_j - c)/(M + δ).");

      // The shift is small but not negligible: this is the term §14 says dominates the budget.
      assertTrue(exact.norm() > 1.0e-4, "A 1% mass error should move the CoM by more than 0.1 mm, was " + 1000.0 * exact.norm() + " mm.");

      // --- Pinned: §14's own assumption, so its formula is exact to first order. ---
      double[] pinnedError = new double[robot.masses().length];
      pinnedError[perturbed] = delta;

      for (int i = 0; i < pinnedError.length; i++)
      {
         if (i != perturbed)
            pinnedError[i] = -delta / (pinnedError.length - 1);
      }

      Vector3D linearised = new Vector3D();
      robot.budget().packShiftFromMassErrors(robot.poses(), pinnedError, linearised);

      Vector3D exactPinned = new Vector3D();
      robot.budget().packExactShiftFromMassErrors(robot.poses(), pinnedError, exactPinned);

      assertEquals(0.0, separation(linearised, exactPinned), 1.0e-12, "With the total pinned, M is unchanged and §14's formula is exact, not approximate.");
   }

   /** §14's first term assumes a pinned total; feeding it errors that do not sum to zero must fail. */
   @Test
   public void testUnpinnedMassErrorsAreRejectedByTheLinearisedTerm() throws Exception
   {
      Robot robot = toyRobot();
      double[] errors = new double[robot.masses().length];
      errors[0] = 0.05;

      assertThrows(IllegalArgumentException.class, () -> robot.budget().packShiftFromMassErrors(robot.poses(), errors, new Vector3D()));
   }

   /** A rotation preserves length, so a link-CoM error of a given size enters at that size. */
   @Test
   public void testLinkComErrorTerm() throws Exception
   {
      Robot robot = toyRobot();

      // 5 mm along x on every link, in each link's own frame.
      Vector3DReadOnly[] errors = new Vector3DReadOnly[robot.masses().length];

      for (int i = 0; i < errors.length; i++)
         errors[i] = new Vector3D(0.005, 0.0, 0.0);

      Vector3D shift = new Vector3D();
      robot.budget().packShiftFromLinkComErrors(robot.poses(), errors, shift);

      // The links point in different directions, so the mass-weighted average of a common
      // link-frame offset is smaller than the offset itself -- but not zero.
      assertTrue(shift.norm() > 0.0);
      assertTrue(shift.norm() <= 0.005 + 1.0e-12, "A mass-weighted average of 5 mm vectors cannot exceed 5 mm, was " + shift.norm());
   }

   /** A common pose error translates the CoM by exactly that amount: a useful sanity anchor. */
   @Test
   public void testCommonPoseErrorTranslatesTheComExactly() throws Exception
   {
      Robot robot = toyRobot();
      Vector3DReadOnly[] errors = new Vector3DReadOnly[robot.masses().length];

      for (int i = 0; i < errors.length; i++)
         errors[i] = new Vector3D(0.001, -0.002, 0.003);

      Vector3D shift = new Vector3D();
      robot.budget().packShiftFromPoseErrors(errors, shift);

      assertEquals(0.001, shift.getX(), 1.0e-15);
      assertEquals(-0.002, shift.getY(), 1.0e-15);
      assertEquals(0.003, shift.getZ(), 1.0e-15);
   }

   /**
    * <b>§14's conclusion, as an assertion.</b>
    * <p>
    * "The third term is the only one this pipeline controls, and mocap drives it sub-millimetre.
    * The first two are CAD-sourced and enter weighted by full link mass and lever arm; for a heavy
    * link with a long moment arm they dominate the third by orders of magnitude."
    * </p>
    * <p>
    * Which means: <b>the calibration validates pose tightly and leaves CoM exactly as good as the
    * URDF.</b> If this test ever starts reporting "pose" as dominant, either the CAD got much
    * better or the mocap got much worse, and either way the project's priorities have changed.
    * </p>
    */
   @Test
   public void testCadTermsDominateTheMocapTerm() throws Exception
   {
      Robot robot = toyRobot();

      // Plausible uncertainties: 5% on mass, 5 mm on link CoM, and F6's per-frame σ on pose.
      ErrorBudgetReport report = robot.budget().evaluate(robot.poses(), 0.05, 0.005, 0.3e-3);

      assertTrue(report.poseTermMeters() < report.massTermMeters(),
                 "The mocap term must be smaller than the mass term.\n" + report.toTable());
      assertTrue(report.poseTermMeters() < report.linkComTermMeters(),
                 "The mocap term must be smaller than the link-CoM term.\n" + report.toTable());
      assertTrue(!"pose".equals(report.dominantTerm()), "§14: the CoM is as good as the URDF, not as good as the mocap.\n" + report.toTable());

      assertTrue(report.poseTermMeters() < 1.0e-3, "Mocap drives its term sub-millimetre: " + 1000.0 * report.poseTermMeters() + " mm.");

      // Perfect mocap would buy almost nothing. That is the whole point of §14.
      assertTrue(report.improvementFromPerfectMocap() < 1.05,
                 "Perfect mocap should change the budget by under 5%, was " + report.improvementFromPerfectMocap() + "×.\n" + report.toTable());

      assertTrue(report.toTable().contains("as good as the URDF"), "The table should say so in words.");
   }

   /** And the flip side: with CAD good enough, the mocap term does become the limit. */
   @Test
   public void testWithTrustworthyInertialsTheMocapTermDominates() throws Exception
   {
      Robot robot = toyRobot();

      // What a force-plate / SESC stage (§16) might buy: 0.2% on mass, 0.2 mm on link CoM.
      ErrorBudgetReport report = robot.budget().evaluate(robot.poses(), 0.002, 0.0002, 0.93e-3);

      assertEquals("pose", report.dominantTerm(), "With measured inertials the mocap becomes the limit.\n" + report.toTable());
      assertTrue(report.improvementFromPerfectMocap() > 1.2, "And then improving the mocap actually buys something.");
   }
}
