package us.ihmc.alexMocap.gates;

import java.util.ArrayList;
import java.util.List;

/**
 * The verdict of one {@link Gate}, plus a finding per thing it checked.
 * <p>
 * FRAMEWORK.md §15: a gate returns pass/fail, not a number to be interpreted later. That is the
 * opposite of the rule for primitives (§2, §9), and the difference is deliberate -- primitives
 * measure, gates decide. So this type <b>does</b> carry a verdict, and the threshold that produced
 * it is recorded alongside every measurement so the decision is auditable rather than opaque.
 * </p>
 *
 * <h2>Why there are three statuses and not two</h2>
 * <p>
 * {@link Status#NOT_EVALUATED} exists because a check that could not run must never be reported as
 * a check that passed. G1 measures a marker pair only in frames where both markers were visible;
 * a pair that was never co-visible produces no number at all. Folding that into PASS would mean a
 * cluster whose markers never appear together gets a green row -- the most confident possible
 * green, since nothing contradicted it.
 * </p>
 * <p>
 * Consequently {@link #isPassed()} is strict: it is true only when every finding passed. An
 * incomplete gate is not a passing gate, and the CLI exits non-zero on it.
 * </p>
 */
public class GateResult
{
   public enum Status
   {
      PASS,
      FAIL,
      /** The check could not be evaluated. Never treat this as a pass. */
      NOT_EVALUATED
   }

   /**
    * One check within a gate: a marker pair for G1, a joint for G2.
    *
    * @param subject    what was checked, e.g. {@code "pelvis: PELVIS_1-PELVIS_3"}.
    * @param status     the verdict for this check.
    * @param measured   the quantity measured, in SI units. NaN if not evaluated.
    * @param threshold  what it was compared against, same units.
    * @param sampleCount how many samples the measurement rests on.
    * @param detail     one line a human can act on.
    */
   public record Finding(String subject, Status status, double measured, double threshold, long sampleCount, String detail)
   {
      public static Finding pass(String subject, double measured, double threshold, long sampleCount, String detail)
      {
         return new Finding(subject, Status.PASS, measured, threshold, sampleCount, detail);
      }

      public static Finding fail(String subject, double measured, double threshold, long sampleCount, String detail)
      {
         return new Finding(subject, Status.FAIL, measured, threshold, sampleCount, detail);
      }

      public static Finding notEvaluated(String subject, long sampleCount, String detail)
      {
         return new Finding(subject, Status.NOT_EVALUATED, Double.NaN, Double.NaN, sampleCount, detail);
      }
   }

   private final String gateName;
   private final List<Finding> findings = new ArrayList<>();
   private String summary = "";

   public GateResult(String gateName)
   {
      this.gateName = gateName;
   }

   public String getGateName()
   {
      return gateName;
   }

   public GateResult add(Finding finding)
   {
      findings.add(finding);
      return this;
   }

   public List<Finding> getFindings()
   {
      return findings;
   }

   public String getSummary()
   {
      return summary;
   }

   public GateResult setSummary(String summary)
   {
      this.summary = summary;
      return this;
   }

   public int countWithStatus(Status status)
   {
      int count = 0;

      for (Finding finding : findings)
      {
         if (finding.status() == status)
            count++;
      }

      return count;
   }

   /** Only the failures, for a report that leads with what is wrong. */
   public List<Finding> getFailures()
   {
      List<Finding> failures = new ArrayList<>();

      for (Finding finding : findings)
      {
         if (finding.status() == Status.FAIL)
            failures.add(finding);
      }

      return failures;
   }

   /**
    * @return {@code true} only if every check ran and every check passed. A gate with nothing to
    *         check does not pass either -- an empty gate is a gate that was misconfigured.
    */
   public boolean isPassed()
   {
      return !findings.isEmpty() && countWithStatus(Status.PASS) == findings.size();
   }

   /** True when nothing failed but something could not be evaluated. */
   public boolean isIncomplete()
   {
      return !isPassed() && countWithStatus(Status.FAIL) == 0;
   }

   public Status getOverallStatus()
   {
      if (isPassed())
         return Status.PASS;

      return countWithStatus(Status.FAIL) > 0 ? Status.FAIL : Status.NOT_EVALUATED;
   }

   @Override
   public String toString()
   {
      return gateName + ": " + getOverallStatus() + " (" + countWithStatus(Status.PASS) + " pass, " + countWithStatus(Status.FAIL) + " fail, "
            + countWithStatus(Status.NOT_EVALUATED) + " not evaluated)";
   }
}
