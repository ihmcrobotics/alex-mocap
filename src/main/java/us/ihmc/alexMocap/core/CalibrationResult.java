package us.ihmc.alexMocap.core;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Everything A′ produces: the calibrated marker layouts, the single global offset {@code Δ}, and
 * the provenance needed to know which capture session and which URDF they came from.
 * <p>
 * <b>This type lives in {@code core}, not in {@code calibration}, on purpose.</b> It is the only
 * thing the offline calibrator and the 200 Hz runtime both touch. Putting it anywhere else would
 * force one of those two packages to import the other, and FRAMEWORK.md §19 forbids that -- the
 * calibrator and the runtime loop must remain separable. {@code PackageDependencyTest} enforces
 * it.
 * </p>
 */
public class CalibrationResult
{
   /**
    * {@code Δ = ^c T_b}: the fixed offset from the Motive pelvis cluster frame to the URDF pelvis
    * link frame. One global constant for the whole session (FRAMEWORK.md §7).
    * <p>
    * The information identifying it comes from the marked links <i>below</i> the pelvis, not from
    * the pelvis cluster itself -- which is why leg marking and wide joint excursion matter, and
    * why a calibration run with the legs held still can look converged and mean nothing.
    * </p>
    */
   private final RigidBodyTransform clusterToBase = new RigidBodyTransform();

   private final List<ClusterLayout> layouts = new ArrayList<>();
   private Provenance provenance = Provenance.unknown();

   /** {@code Δ = ^c T_b}. Mutable; A′ overwrites it each iteration of the base step. */
   public RigidBodyTransform getClusterToBase()
   {
      return clusterToBase;
   }

   public List<ClusterLayout> getLayouts()
   {
      return layouts;
   }

   public void addLayout(ClusterLayout layout)
   {
      if (findLayout(layout.getLinkName()) != null)
         throw new IllegalArgumentException("A layout for link '" + layout.getLinkName() + "' is already present.");

      layouts.add(layout);
   }

   /** @return the layout for a link, or {@code null} if that link carries no cluster. */
   public ClusterLayout findLayout(String linkName)
   {
      for (int i = 0; i < layouts.size(); i++)
      {
         if (layouts.get(i).getLinkName().equals(linkName))
            return layouts.get(i);
      }

      return null;
   }

   public ClusterLayout getLayout(String linkName)
   {
      ClusterLayout layout = findLayout(linkName);

      if (layout == null)
         throw new IllegalArgumentException("No layout for link '" + linkName + "'. Known links: " + getLinkNames() + ".");

      return layout;
   }

   public List<String> getLinkNames()
   {
      List<String> names = new ArrayList<>(layouts.size());

      for (ClusterLayout layout : layouts)
         names.add(layout.getLinkName());

      return names;
   }

   public Provenance getProvenance()
   {
      return provenance;
   }

   public void setProvenance(Provenance provenance)
   {
      this.provenance = provenance == null ? Provenance.unknown() : provenance;
   }

   /** Whether every marker in every layout has a finite calibrated position. */
   public boolean isFullySolved()
   {
      for (ClusterLayout layout : layouts)
      {
         if (!layout.isFullySolved())
            return false;
      }

      return !layouts.isEmpty();
   }

   @Override
   public String toString()
   {
      return "CalibrationResult[" + layouts.size() + " layouts, " + provenance + "]";
   }

   /**
    * Where a calibration came from, recorded alongside it.
    * <p>
    * A layout without provenance is a set of numbers with no way to tell whether it applies to the
    * robot in front of you. Three of these fields answer questions that otherwise get answered by
    * guessing:
    * </p>
    * <ul>
    * <li>{@code urdf} / {@code urdfSha256} -- FRAMEWORK.md §3 marks the URDF <b>assumed</b>, and
    * §21.1 makes its inertial blocks the single largest open question. A calibration is only valid
    * against the URDF it was solved with, and "which URDF was that" is otherwise unanswerable a
    * month later.</li>
    * <li>{@code worldTiltRadians} -- the F8 tilt (§11), a capture-session constant that must be
    * measured and never assumed. It is systematic, ungated, and worth ~7 mm of CoM height at 0.5°.
    * Recording it here does not apply it -- {@code GravityAlignedWorldFrame} does that -- but it
    * makes the number auditable instead of lost.</li>
    * <li>{@code finalObjective} and {@code iterations} -- whether A′ actually converged, or was
    * cut off at the iteration cap with {@code J} still falling.</li>
    * </ul>
    *
    * @param urdf             identifier of the URDF the calibration was solved against.
    * @param urdfSha256       hash of that URDF, or {@code null}. The name alone does not pin a file
    *                         that someone edits in place.
    * @param captureCount     {@code K}.
    * @param iterations       A′ iterations run.
    * @param finalObjective   {@code J} at termination (FRAMEWORK.md §8), in m².
    * @param worldTiltRadians measured F8 tilt {@code θ}, or NaN if it was never measured -- which
    *                         NaN says plainly, where 0.0 would be a claim.
    * @param createdAt        free-form timestamp, ISO-8601 by convention.
    * @param note             anything a human wants the next reader to know.
    */
   public record Provenance(String urdf, String urdfSha256, int captureCount, int iterations, double finalObjective, double worldTiltRadians,
         String createdAt, String note)
   {
      public static Provenance unknown()
      {
         return new Provenance("unknown", null, 0, 0, Double.NaN, Double.NaN, "", "");
      }

      /** Whether the F8 tilt was measured at all. FRAMEWORK.md §11: never assume it. */
      public boolean hasMeasuredWorldTilt()
      {
         return Double.isFinite(worldTiltRadians);
      }

      @Override
      public String toString()
      {
         return "urdf=" + urdf + ", K=" + captureCount + ", iterations=" + iterations + ", J=" + finalObjective
               + (hasMeasuredWorldTilt() ? ", tilt=" + Math.toDegrees(worldTiltRadians) + "°" : ", tilt=UNMEASURED");
      }
   }
}
