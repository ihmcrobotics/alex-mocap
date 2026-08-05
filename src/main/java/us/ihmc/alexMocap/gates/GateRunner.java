package us.ihmc.alexMocap.gates;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs gates in order and renders the result as a table a human reads at the gantry.
 * <p>
 * Order matters. FRAMEWORK.md §15 has each gate protect the thing after it, so the first failure
 * is the informative one: if G1 fails, the mounting is wrong and running G2 against slop tells you
 * nothing. {@link #runAll()} therefore evaluates every gate but reports them in order, and the
 * report leads with the earliest failure.
 * </p>
 */
public class GateRunner
{
   private final List<Gate> gates = new ArrayList<>();

   public GateRunner add(Gate gate)
   {
      gates.add(gate);
      return this;
   }

   public List<Gate> getGates()
   {
      return gates;
   }

   public Report runAll()
   {
      List<GateResult> results = new ArrayList<>(gates.size());

      for (Gate gate : gates)
         results.add(gate.run());

      return new Report(gates, results);
   }

   /** The results of one run, with the formatting the CLI prints. */
   public static class Report
   {
      private final List<Gate> gates;
      private final List<GateResult> results;

      public Report(List<Gate> gates, List<GateResult> results)
      {
         this.gates = List.copyOf(gates);
         this.results = List.copyOf(results);
      }

      public List<GateResult> getResults()
      {
         return results;
      }

      /** True only if every gate passed. Incomplete is not passed -- see {@link GateResult}. */
      public boolean isPassed()
      {
         if (results.isEmpty())
            return false;

         for (GateResult result : results)
         {
            if (!result.isPassed())
               return false;
         }

         return true;
      }

      /** The first gate that did not pass, or {@code null}. The one worth acting on. */
      public GateResult getFirstFailure()
      {
         for (GateResult result : results)
         {
            if (!result.isPassed())
               return result;
         }

         return null;
      }

      /**
       * The per-check table. Failures and unevaluated checks are listed in full; passing checks are
       * summarised by count, because forty green rows is how a red one gets missed.
       */
      public String format()
      {
         StringBuilder report = new StringBuilder(1024);

         for (int i = 0; i < results.size(); i++)
         {
            GateResult result = results.get(i);
            Gate gate = gates.get(i);

            report.append(gate.getName()).append(" -- ").append(gate.getDescription()).append('\n');

            if (!result.getSummary().isEmpty())
               report.append("  ").append(result.getSummary()).append('\n');

            report.append('\n');

            List<GateResult.Finding> notable = new ArrayList<>();

            for (GateResult.Finding finding : result.getFindings())
            {
               if (finding.status() != GateResult.Status.PASS)
                  notable.add(finding);
            }

            if (notable.isEmpty())
            {
               report.append(String.format("  all %d checks passed%n", result.getFindings().size()));
            }
            else
            {
               report.append(String.format("  %-14s %-40s %12s %12s %8s%n", "STATUS", "SUBJECT", "MEASURED", "THRESHOLD", "SAMPLES"));

               for (GateResult.Finding finding : notable)
               {
                  report.append(String.format("  %-14s %-40s %12s %12s %8d%n",
                                              finding.status(),
                                              truncate(finding.subject(), 40),
                                              millimetres(finding.measured()),
                                              millimetres(finding.threshold()),
                                              finding.sampleCount()));
                  report.append("                 ").append(finding.detail()).append('\n');
               }

               report.append(String.format("%n  %d passed, %d failed, %d not evaluated%n",
                                           result.countWithStatus(GateResult.Status.PASS),
                                           result.countWithStatus(GateResult.Status.FAIL),
                                           result.countWithStatus(GateResult.Status.NOT_EVALUATED)));
            }

            report.append('\n');
            report.append("  ").append(gate.getName()).append(": ").append(verdictOf(result)).append('\n');

            if (i < results.size() - 1)
               report.append('\n');
         }

         return report.toString();
      }

      private static String verdictOf(GateResult result)
      {
         if (result.isPassed())
            return "PASS";

         return result.isIncomplete() ? "INCOMPLETE -- checks could not be evaluated; this is not a pass" : "FAIL";
      }

      private static String millimetres(double metres)
      {
         return Double.isNaN(metres) ? "-" : String.format("%.4f mm", 1000.0 * metres);
      }

      private static String truncate(String text, int width)
      {
         return text.length() <= width ? text : text.substring(0, width - 1) + "…";
      }

      @Override
      public String toString()
      {
         return format();
      }
   }
}
