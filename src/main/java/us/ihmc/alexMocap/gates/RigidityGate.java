package us.ihmc.alexMocap.gates;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;

/**
 * G1 -- rigidity. FRAMEWORK.md §15.
 *
 * <pre>
 * Var_k( || ^W m_ijk - ^W m_ij'k || )  ≈  0     for all pairs j, j' in a cluster
 * </pre>
 *
 * <p>
 * If a cluster is rigidly mounted, the distance between any two of its markers is constant, and
 * the only thing moving it is mocap noise. Anything larger is mount slop or a label swap.
 * </p>
 * <p>
 * <b>Run this first, always.</b> It consumes raw mocap and nothing else -- no FK, no URDF, no
 * encoders, no calibration -- so a failure indicts the mounting or the labelling and cannot be
 * anything else. Every other gate in the pipeline mixes in a modelling question; this one does not.
 * </p>
 *
 * <h2>What the threshold means</h2>
 * <p>
 * For a genuinely rigid pair, write each measurement as truth plus noise, {@code m = m̄ + e} with
 * {@code e ~ N(0, σ²I)} independent per marker per frame. Perturbing the distance
 * {@code d = ||m_j - m_j'||} to first order gives
 * </p>
 *
 * <pre>
 * δd  ≈  û · (e_j - e_j')          û = the unit vector along the baseline
 *
 * Var(δd)  =  Var(û·e_j) + Var(û·e_j')  =  σ² + σ²  =  2σ²
 * </pre>
 *
 * <p>
 * So <b>a perfectly rigid pair still shows a sample standard deviation of {@code σ√2 ≈ 1.41σ}</b>,
 * not zero. FRAMEWORK.md fixes the threshold at {@code 3σ}, which is a margin of
 * {@code 3 / √2 ≈ 2.1×} over the noise floor -- not 3×, as the bare number suggests. That margin
 * is the headroom you actually have, and it is worth knowing before anyone tightens the constant.
 * </p>
 * <p>
 * The linearisation holds comfortably here: {@code σ / baseline ≈ 0.3 mm / 60 mm ≈ 0.005}. The
 * neglected second-order term biases the mean distance upward by about {@code σ²/d}, some 1.5 µm,
 * and does not materially affect the spread.
 * </p>
 * <p>
 * {@code σ} is the <b>measured</b> per-axis position noise at the gantry, per FRAMEWORK.md §17 and
 * §20.1 -- not the wand residual, which is an average over the whole lab. There is no default for
 * it here, on purpose.
 * </p>
 *
 * <h2>Partial visibility</h2>
 * <p>
 * A pair is measurable only in frames where both of its markers were visible, so each pair carries
 * its own sample count. Two consequences, both handled rather than assumed:
 * </p>
 * <ul>
 * <li>A pair with too few co-visible frames is reported {@link GateResult.Status#NOT_EVALUATED},
 * never as a pass. The sample standard deviation from {@code K} samples has a relative standard
 * error of about {@code 1/√(2K)}; at {@code K = 100} that is 7%, so a rigid pair reads
 * {@code 1.41σ ± 0.10σ} and stays clear of {@code 3σ}. At {@code K = 10} it is 24% and the noise
 * floor alone starts to threaten the threshold. Hence the default minimum of 100.</li>
 * <li>Because it is reported rather than skipped, a cluster whose markers are never co-visible
 * fails to pass -- which is the correct answer, and the one a silent skip would invert.</li>
 * </ul>
 *
 * <h2>How much slop this actually catches</h2>
 * <p>
 * The threshold is on a standard deviation, so what matters is not how far a marker moved but how
 * that movement is <i>distributed over the capture</i>. Two faults with the same total travel
 * {@code a} produce very different spreads:
 * </p>
 * <ul>
 * <li><b>A step</b> -- the mount slips once, partway through -- puts half the samples at each end:
 * {@code std = a/2}.</li>
 * <li><b>A linear creep</b> spreads the samples uniformly over {@code [0, a]}:
 * {@code std = a/√12 ≈ a/3.46}, which is 1.7× smaller for the same travel.</li>
 * </ul>
 * <p>
 * Adding the noise floor in quadrature and solving {@code √(2σ² + (k·a)²) = 3σ} at
 * {@code σ = 0.3 mm} gives, for a pair whose baseline is aligned with the movement, a detection
 * threshold of about <b>1.7 mm for a step</b> and about <b>3.1 mm for a creep</b>. Both are
 * measured and asserted in {@code RigidityGateTest}.
 * </p>
 * <p>
 * <b>The geometry matters as much as the amplitude.</b> Only the component of the movement along a
 * pair's baseline changes that pair's distance -- the {@code û} in the derivation above. A marker
 * that shifts perpendicular to a baseline is invisible to that pair no matter how far it moves.
 * This is why a cluster is checked pairwise rather than by any single number: for a shift to hide
 * from G1 entirely it would have to be perpendicular to <i>every</i> baseline at once, which a
 * non-collinear cluster does not permit.
 * </p>
 *
 * <h2>What G1 cannot catch</h2>
 * <p>
 * A label swap <i>within</i> a cluster changes the inter-marker distances and shows up here. A
 * marker assigned to the wrong <i>link</i> does not: its distances to its (wrong) clustermates are
 * not constant, so it will usually fail -- but a marker swapped between two links whose clusters
 * are geometrically similar can pass. FRAMEWORK.md §21.5.
 * </p>
 *
 * <h2>Usage</h2>
 * <p>
 * Frames are pushed in; this class does not read them. {@code gates} depends on {@code core},
 * {@code model} and {@code registration} only (FRAMEWORK.md §19), so it cannot reach for a
 * {@code MocapSource} -- the caller streams and calls {@link #accumulate}. Accumulation is
 * allocation-free and single-pass, so a 60 s capture at 200 Hz costs nothing to hold.
 * </p>
 */
public class RigidityGate implements Gate
{
   /**
    * FRAMEWORK.md §15: fail if any pair's standard deviation exceeds {@code 3σ}. Chosen, not
    * derived -- the noise floor is {@code √2 σ}, so this is a 2.1× margin.
    */
   public static final double DEFAULT_SIGMA_MULTIPLIER = 3.0;

   /** At 100 co-visible frames the sample std is good to about 7%; see the class comment. */
   public static final int DEFAULT_MINIMUM_SAMPLES = 100;

   private final List<MarkerCluster> clusters;
   private final double perAxisSigma;
   private final double sigmaMultiplier;
   private final int minimumSamples;

   /** Welford accumulators, one per pair, flattened across all clusters. */
   private final List<PairStatistics> pairs = new ArrayList<>();
   private long framesAccumulated = 0;

   public RigidityGate(List<MarkerCluster> clusters, double perAxisSigma)
   {
      this(clusters, perAxisSigma, DEFAULT_SIGMA_MULTIPLIER, DEFAULT_MINIMUM_SAMPLES);
   }

   /**
    * @param clusters        the clusters to check. Each is checked independently; G1 says nothing
    *                        about the relationship between two clusters, because there is no rigid
    *                        relationship between them to check.
    * @param perAxisSigma    measured per-axis mocap position noise, in metres.
    * @param sigmaMultiplier threshold as a multiple of {@code σ}. FRAMEWORK.md says 3.
    * @param minimumSamples  co-visible frames a pair needs before its spread is judged.
    */
   public RigidityGate(List<MarkerCluster> clusters, double perAxisSigma, double sigmaMultiplier, int minimumSamples)
   {
      if (clusters == null || clusters.isEmpty())
         throw new IllegalArgumentException("G1 needs at least one cluster to check.");
      if (!(perAxisSigma > 0.0) || !Double.isFinite(perAxisSigma))
         throw new IllegalArgumentException("Per-axis sigma must be a positive, finite measurement in metres, was " + perAxisSigma
               + ". FRAMEWORK.md §17: it must be measured at the gantry, never assumed.");
      if (!(sigmaMultiplier > 0.0))
         throw new IllegalArgumentException("Sigma multiplier must be positive, was " + sigmaMultiplier + ".");
      if (minimumSamples < 2)
         throw new IllegalArgumentException("A standard deviation needs at least 2 samples, was given " + minimumSamples + ".");

      this.clusters = List.copyOf(clusters);
      this.perAxisSigma = perAxisSigma;
      this.sigmaMultiplier = sigmaMultiplier;
      this.minimumSamples = minimumSamples;

      for (MarkerCluster cluster : this.clusters)
      {
         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            for (int k = j + 1; k < cluster.getMarkerCount(); k++)
               pairs.add(new PairStatistics(cluster.getLinkName(), cluster.getMarker(j), cluster.getMarker(k)));
         }
      }
   }

   @Override
   public String getName()
   {
      return "G1";
   }

   @Override
   public String getDescription()
   {
      return "rigidity: inter-marker distances within a cluster must be constant to within mocap noise";
   }

   /**
    * Folds one frame into the running per-pair statistics. Allocation-free; safe to call at the
    * capture rate.
    */
   public void accumulate(MocapFrame frame)
   {
      for (int i = 0; i < pairs.size(); i++)
         pairs.get(i).accumulate(frame);

      framesAccumulated++;
   }

   public long getFramesAccumulated()
   {
      return framesAccumulated;
   }

   public int getPairCount()
   {
      return pairs.size();
   }

   /** The threshold in metres: {@code sigmaMultiplier × σ}. */
   public double getThreshold()
   {
      return sigmaMultiplier * perAxisSigma;
   }

   /** The spread a perfectly rigid pair still shows, {@code √2 σ}. The floor, not the threshold. */
   public double getNoiseFloor()
   {
      return Math.sqrt(2.0) * perAxisSigma;
   }

   @Override
   public GateResult run()
   {
      GateResult result = new GateResult(getName());
      double threshold = getThreshold();

      for (PairStatistics pair : pairs)
      {
         String subject = pair.linkName + ": " + pair.markerA.getName() + "-" + pair.markerB.getName();

         if (pair.count < minimumSamples)
         {
            result.add(GateResult.Finding.notEvaluated(subject,
                                                       pair.count,
                                                       "only " + pair.count + " co-visible frames, need " + minimumSamples
                                                             + "; the pair was too rarely seen together to judge"));
            continue;
         }

         double standardDeviation = pair.getStandardDeviation();
         String detail = String.format("std %.4f mm over a %.1f mm baseline (floor %.4f mm)",
                                       1000.0 * standardDeviation,
                                       1000.0 * pair.mean,
                                       1000.0 * getNoiseFloor());

         if (standardDeviation > threshold)
            result.add(GateResult.Finding.fail(subject, standardDeviation, threshold, pair.count, detail));
         else
            result.add(GateResult.Finding.pass(subject, standardDeviation, threshold, pair.count, detail));
      }

      result.setSummary(String.format("%d pairs over %d frames, sigma %.4f mm, threshold %.4f mm (%.1f sigma; noise floor %.4f mm = sqrt(2) sigma)",
                                      pairs.size(),
                                      framesAccumulated,
                                      1000.0 * perAxisSigma,
                                      1000.0 * threshold,
                                      sigmaMultiplier,
                                      1000.0 * getNoiseFloor()));
      return result;
   }

   /** Drops all accumulated statistics so the gate can be re-run over a different capture. */
   public void reset()
   {
      for (PairStatistics pair : pairs)
         pair.reset();

      framesAccumulated = 0;
   }

   /**
    * Streaming mean and variance of one pair's distance, by Welford's method.
    * <p>
    * Welford rather than accumulating {@code sum} and {@code sumOfSquares}: the distances here are
    * tightly clustered about a non-zero baseline -- 60 mm ± 0.4 µm is typical -- and the naive form
    * subtracts two nearly equal large numbers to find a tiny one. That is the textbook case for
    * catastrophic cancellation, and at these ratios it costs most of the significant figures the
    * answer depends on.
    * </p>
    */
   private static final class PairStatistics
   {
      private final String linkName;
      private final MarkerId markerA;
      private final MarkerId markerB;

      private long count = 0;
      private double mean = 0.0;
      private double sumOfSquaredDeviations = 0.0;

      private PairStatistics(String linkName, MarkerId markerA, MarkerId markerB)
      {
         this.linkName = linkName;
         this.markerA = markerA;
         this.markerB = markerB;
      }

      private void accumulate(MocapFrame frame)
      {
         MarkerObservation a = frame.get(markerA);
         MarkerObservation b = frame.get(markerB);

         if (!a.isVisible() || !b.isVisible())
            return;

         double dx = a.getPosition().getX() - b.getPosition().getX();
         double dy = a.getPosition().getY() - b.getPosition().getY();
         double dz = a.getPosition().getZ() - b.getPosition().getZ();
         double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

         count++;
         double delta = distance - mean;
         mean += delta / count;
         sumOfSquaredDeviations += delta * (distance - mean);
      }

      /** Sample standard deviation, Bessel-corrected. */
      private double getStandardDeviation()
      {
         return count < 2 ? Double.NaN : Math.sqrt(sumOfSquaredDeviations / (count - 1));
      }

      private void reset()
      {
         count = 0;
         mean = 0.0;
         sumOfSquaredDeviations = 0.0;
      }
   }
}
