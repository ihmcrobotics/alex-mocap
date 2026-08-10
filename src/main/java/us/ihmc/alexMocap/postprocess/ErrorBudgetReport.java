package us.ihmc.alexMocap.postprocess;

/**
 * The three terms of FRAMEWORK.md §14, sized, plus which one dominates.
 * <p>
 * The point of printing this is to make §14's conclusion arrive as a number rather than as a
 * warning: <b>the calibration validates pose tightly and leaves CoM exactly as good as the
 * URDF.</b> If {@link #dominantTerm()} says anything other than "pose", no amount of mocap work
 * improves the CoM.
 * </p>
 *
 * @param massTermMeters     {@code δm_i × lever arm}, from mass uncertainty. CAD-sourced.
 * @param linkComTermMeters  {@code m_i × δ(^i c_i)}, from link-CoM uncertainty. CAD-sourced.
 * @param poseTermMeters     {@code m_i × δ(^W T_i)}, from mocap. The only one this pipeline
 *                           controls.
 * @param totalMassKilograms {@code M}.
 */
public record ErrorBudgetReport(double massTermMeters, double linkComTermMeters, double poseTermMeters, double totalMassKilograms,
      double massFractionUncertainty, double linkComUncertaintyMeters, double poseUncertaintyMeters)
{
   /** Quadrature sum of the three, metres. They are independent error sources. */
   public double totalMeters()
   {
      return Math.sqrt(massTermMeters * massTermMeters + linkComTermMeters * linkComTermMeters + poseTermMeters * poseTermMeters);
   }

   /** {@code "mass"}, {@code "linkCoM"} or {@code "pose"}. */
   public String dominantTerm()
   {
      if (massTermMeters >= linkComTermMeters && massTermMeters >= poseTermMeters)
         return "mass";

      return linkComTermMeters >= poseTermMeters ? "linkCoM" : "pose";
   }

   /**
    * How much better the CoM would get if mocap were perfect: the ratio of the total to the total
    * with the pose term removed.
    * <p>
    * A value near 1 is §14's conclusion stated as a number -- perfect mocap would change nothing,
    * because the CAD terms already dominate.
    * </p>
    */
   public double improvementFromPerfectMocap()
   {
      double withoutPose = Math.sqrt(massTermMeters * massTermMeters + linkComTermMeters * linkComTermMeters);
      return withoutPose > 0.0 ? totalMeters() / withoutPose : Double.POSITIVE_INFINITY;
   }

   public String toTable()
   {
      StringBuilder table = new StringBuilder();
      table.append("CoM error budget (FRAMEWORK.md section 14)\n");
      table.append(String.format("  total mass                 %.3f kg%n", totalMassKilograms));
      table.append(String.format("  assumed uncertainties      mass %.1f%%, link CoM %.1f mm/axis, pose %.3f mm/axis%n",
                                 100.0 * massFractionUncertainty,
                                 1000.0 * linkComUncertaintyMeters,
                                 1000.0 * poseUncertaintyMeters));
      table.append(String.format("  mass error x lever arm     %8.3f mm   (CAD)%n", 1000.0 * massTermMeters));
      table.append(String.format("  link-CoM error             %8.3f mm   (CAD)%n", 1000.0 * linkComTermMeters));
      table.append(String.format("  pose error (mocap)         %8.3f mm   <- the only one this pipeline controls%n", 1000.0 * poseTermMeters));
      table.append(String.format("  total (quadrature)         %8.3f mm%n", 1000.0 * totalMeters()));
      table.append(String.format("  dominant term              %s%n", dominantTerm()));
      table.append(String.format("  perfect mocap would buy    %.2fx%n", improvementFromPerfectMocap()));

      if (!"pose".equals(dominantTerm()))
      {
         table.append("  NOTE: the CoM is as good as the URDF, not as good as the mocap. Weigh the robot\n");
         table.append("        (section 14) and consider the SESC / force-plate stage (section 16).\n");
      }

      return table.toString();
   }
}
