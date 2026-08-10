package us.ihmc.alexMocap.scs2;

import java.util.List;

import us.ihmc.alexMocap.core.GroundTruthSample;

/**
 * Session-level accumulation of what {@link GroundTruthYoVariables} publishes per frame: the
 * {@code σ₃} and visible-count histograms FRAMEWORK.md §20.4 asks for, per cluster.
 *
 * <h2>Why a histogram and not a minimum</h2>
 * <p>
 * §20.4's instruction is to sweep the legs and "log visible-count and {@code σ₃} histograms per
 * cluster. If clusters drop below 3 visible markers during leg sweeps, fix the mounting before
 * writing a line of F3-F5." A single worst-case number cannot answer the question that decides
 * that: <b>how often</b>, and <b>in what part of the workspace</b>. A cluster that dips to two
 * markers for one frame in a thousand is a different mounting problem from one that spends a third
 * of the sweep there, and the minimum reports them identically.
 * </p>
 * <p>
 * §21.3 makes the same point for runtime: "Measure visible-count histograms over a real motion
 * before trusting any runtime number."
 * </p>
 *
 * <h2>Headless</h2>
 * <p>
 * This accumulates plain arrays and touches no YoVariable and no toolkit, so it runs in CI and over
 * a replayed log. It lives in {@code scs2} because §19's package map puts it there and because it
 * is the natural companion to the telemetry, not because it needs a display.
 * </p>
 */
public class ConditioningMonitor
{
   private final List<String> linkNames;

   private final long[] framesSeen;
   private final long[] framesAccepted;
   private final long[][] visibleCountHistogram;
   private final double[] worstSigma3;
   private final double[] sigma3Sum;
   private final long[] sigma3Samples;
   private final int maximumMarkersPerCluster;

   private long totalFrames;
   private long framesWithEveryLinkAccepted;

   /**
    * @param maximumMarkersPerCluster the largest cluster's marker count, sizing the histogram.
    */
   public ConditioningMonitor(List<String> linkNames, int maximumMarkersPerCluster)
   {
      this.linkNames = List.copyOf(linkNames);
      this.maximumMarkersPerCluster = maximumMarkersPerCluster;

      this.framesSeen = new long[linkNames.size()];
      this.framesAccepted = new long[linkNames.size()];
      this.visibleCountHistogram = new long[linkNames.size()][maximumMarkersPerCluster + 1];
      this.worstSigma3 = new double[linkNames.size()];
      this.sigma3Sum = new double[linkNames.size()];
      this.sigma3Samples = new long[linkNames.size()];

      java.util.Arrays.fill(worstSigma3, Double.POSITIVE_INFINITY);
   }

   /** Accumulates one frame. Allocation-free. */
   public void accumulate(GroundTruthSample sample)
   {
      totalFrames++;

      if (sample.allPosesAccepted())
         framesWithEveryLinkAccepted++;

      for (int i = 0; i < linkNames.size(); i++)
      {
         int index = sample.indexOfLink(linkNames.get(i));
         framesSeen[i]++;

         int visible = Math.min(sample.getVisibleCount(index), maximumMarkersPerCluster);
         visibleCountHistogram[i][Math.max(0, visible)]++;

         if (sample.isPoseAccepted(index))
         {
            framesAccepted[i]++;

            double sigma = sample.getSigma3(index);

            if (Double.isFinite(sigma))
            {
               worstSigma3[i] = Math.min(worstSigma3[i], sigma);
               sigma3Sum[i] += sigma;
               sigma3Samples[i]++;
            }
         }
      }
   }

   public long getTotalFrames()
   {
      return totalFrames;
   }

   /** Frames in which every marked link produced a pose. The number that gates a CoM trajectory. */
   public long getFramesWithEveryLinkAccepted()
   {
      return framesWithEveryLinkAccepted;
   }

   public double getAcceptedFraction(int linkIndex)
   {
      return framesSeen[linkIndex] == 0 ? Double.NaN : (double) framesAccepted[linkIndex] / framesSeen[linkIndex];
   }

   /** Frames in which this cluster showed exactly {@code count} markers. */
   public long getVisibleCountFrequency(int linkIndex, int count)
   {
      return visibleCountHistogram[linkIndex][count];
   }

   /** Frames below the three-marker floor, where no pose exists under any conditions. */
   public long getFramesBelowPoseMinimum(int linkIndex)
   {
      long below = 0;

      for (int count = 0; count < Math.min(3, visibleCountHistogram[linkIndex].length); count++)
         below += visibleCountHistogram[linkIndex][count];

      return below;
   }

   public double getWorstSigma3(int linkIndex)
   {
      return Double.isInfinite(worstSigma3[linkIndex]) ? Double.NaN : worstSigma3[linkIndex];
   }

   public double getMeanSigma3(int linkIndex)
   {
      return sigma3Samples[linkIndex] == 0 ? Double.NaN : sigma3Sum[linkIndex] / sigma3Samples[linkIndex];
   }

   public List<String> getLinkNames()
   {
      return linkNames;
   }

   /** The table §20.4 sends someone to the gantry to produce. */
   public String toTable()
   {
      StringBuilder table = new StringBuilder();
      table.append(String.format("Conditioning over %d frames (%d with every link accepted, %.1f%%)%n",
                                 totalFrames,
                                 framesWithEveryLinkAccepted,
                                 totalFrames == 0 ? Double.NaN : 100.0 * framesWithEveryLinkAccepted / totalFrames));
      table.append(String.format("  %-12s %8s %10s %12s %12s   %s%n", "link", "accept%", "below 3", "worst s3", "mean s3", "visible-count histogram"));

      for (int i = 0; i < linkNames.size(); i++)
      {
         StringBuilder histogram = new StringBuilder();

         for (int count = 0; count <= maximumMarkersPerCluster; count++)
            histogram.append(count == 0 ? "" : " ").append(count).append(':').append(visibleCountHistogram[i][count]);

         table.append(String.format("  %-12s %7.1f%% %10d %12.3e %12.3e   %s%n",
                                    linkNames.get(i),
                                    100.0 * getAcceptedFraction(i),
                                    getFramesBelowPoseMinimum(i),
                                    getWorstSigma3(i),
                                    getMeanSigma3(i),
                                    histogram));
      }

      table.append("  s3 in m^2 (a mean-squared spread, not a length).\n");

      for (int i = 0; i < linkNames.size(); i++)
      {
         if (getFramesBelowPoseMinimum(i) > 0)
         {
            table.append(String.format("  WARNING: '%s' fell below 3 visible markers in %d frames. FRAMEWORK.md section 20.4: fix the%n",
                                       linkNames.get(i),
                                       getFramesBelowPoseMinimum(i)));
            table.append("           mounting or the camera coverage before trusting anything downstream.\n");
         }
      }

      return table.toString();
   }
}
