package us.ihmc.alexMocap.gates;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * G3 -- volume distortion. FRAMEWORK.md §15.
 *
 * <pre>
 * | ^W len(wand)_side  -  L_known |  ≈  0     for every side of the working volume
 * </pre>
 *
 * <p>
 * Carry a rigid two-marker artifact of fixed, independently known length {@code L_known} (measured
 * with calipers, not by mocap) through the working volume, and check that the length mocap reports
 * agrees with {@code L_known} <b>everywhere it was carried, not only near the middle</b>. A camera
 * volume's calibration is a spatial fit -- lens distortion and calibration-extrapolation error grow
 * with distance from where the calibration wand actually swept, and Motive's own residual is a
 * single number averaged over the whole lab. Two sides that are individually off by equal and
 * opposite amounts cancel in that average and report a small, reassuring residual while a marker
 * cluster sitting near one specific side reads a real, uncaught bias. G1 cannot see this either:
 * it checks a cluster's <i>internal</i> consistency, which a spatial bias does not disturb at all
 * -- a whole rigid cluster can be shifted by the volume's distortion without any pair inside it
 * changing distance from any other.
 * </p>
 *
 * <h2>Why six one-sided regions and not a grid of cells</h2>
 * <p>
 * FRAMEWORK.md's own diagnosis of the coverage gap is <i>directional</i> --
 * "camera coverage ~3 of 4 sides, weak axis points at the missing side" -- so the check is built to
 * answer exactly that question. Every sample is binned relative to {@code volumeCentre} along all
 * three axes at once (a sample at {@code (+x, +y, -z)} contributes to X+, Y+ <b>and</b> Z-), which
 * both matches how "which side is bad" is actually asked and means a single sweep through the room
 * populates every region without the sweep having to be planned around a grid.
 * </p>
 *
 * <h2>The threshold is on a mean, not on a single sample -- deliberately unlike G1</h2>
 * <p>
 * G1 bounds a standard deviation, which does not tighten with more data: a mount is rigid or it
 * is not, and 3σ is a fixed margin over the {@code √2σ} noise floor a perfect pair still shows. G3
 * asks a different question -- is a fixed, known length being measured <i>correctly</i> -- and the
 * more samples a side has, the more precisely its mean length is known, so the same true bias
 * becomes easier to resolve from noise as {@code N} grows. That is the point: a small persistent
 * distortion should get harder to hide behind noise the longer the wand dwells on that side, not
 * impossible to ever catch. The per-side threshold is therefore {@code sigmaMultiplier × √2σ / √N}
 * -- G1's noise floor for a single distance sample, reduced by averaging over {@code N} of them.
 * </p>
 * <p>
 * {@code σ} is the same measured per-axis position noise FRAMEWORK.md §17 asks G1 to use -- one
 * physical property of the cameras, not a separate number to keep in sync.
 * </p>
 *
 * <h2>Usage</h2>
 * <p>
 * A separate capture from G1/G2/G4: nothing here is mounted on the robot. Carry the two-marker
 * artifact through the volume, favouring its edges and corners over its centre -- a sweep that
 * never leaves the middle of the room cannot populate the sides this gate exists to check, and
 * every side with too few samples reports {@link GateResult.Status#NOT_EVALUATED} rather than a
 * silent pass. Frames are pushed in via {@link #accumulate}; this class does not read them, for the
 * same package-boundary reason as G1 (FRAMEWORK.md §19: {@code gates} depends only on {@code core},
 * {@code model} and {@code registration}).
 * </p>
 */
public class VolumeDistortionGate implements Gate
{
   /** Same convention as G1: a chosen margin, not a derived one. */
   public static final double DEFAULT_SIGMA_MULTIPLIER = 3.0;

   /** A side's mean is only as trustworthy as its sample count; below this it is not evaluated. */
   public static final int DEFAULT_MINIMUM_SAMPLES = 30;

   private static final String[] SIDE_NAMES = {"X-", "X+", "Y-", "Y+", "Z-", "Z+"};

   private final MarkerId markerA;
   private final MarkerId markerB;
   private final double knownLengthMeters;
   private final double perAxisSigma;
   private final double sigmaMultiplier;
   private final int minimumSamples;
   private final Point3DReadOnly volumeCentre;

   private final Map<String, SideStatistics> sides = new LinkedHashMap<>();
   private long framesAccumulated = 0;

   public VolumeDistortionGate(MarkerId markerA, MarkerId markerB, double knownLengthMeters, double perAxisSigma, Point3DReadOnly volumeCentre)
   {
      this(markerA, markerB, knownLengthMeters, perAxisSigma, DEFAULT_SIGMA_MULTIPLIER, DEFAULT_MINIMUM_SAMPLES, volumeCentre);
   }

   /**
    * @param markerA           one marker of the rigid two-marker artifact.
    * @param markerB            the other. Order does not matter; the pair is symmetric.
    * @param knownLengthMeters  the artifact's length, measured independently of mocap (calipers),
    *                           in metres.
    * @param perAxisSigma       measured per-axis mocap position noise, in metres -- the same
    *                           quantity G1 uses, FRAMEWORK.md §17.
    * @param sigmaMultiplier    threshold as a multiple of the per-sample noise floor. FRAMEWORK.md
    *                           does not fix this number for G3 the way it does for G1; 3 is carried
    *                           over as the same chosen margin.
    * @param minimumSamples     co-visible frames a side needs, with the wand actually on that side,
    *                           before its mean is judged.
    * @param volumeCentre       the point samples are binned relative to on each axis. Not required
    *                           to be exact -- it only has to put roughly as much of the sweep on
    *                           each side of it as the other, so the world-frame origin from ground
    *                           plane registration is normally the right choice.
    */
   public VolumeDistortionGate(MarkerId markerA,
                               MarkerId markerB,
                               double knownLengthMeters,
                               double perAxisSigma,
                               double sigmaMultiplier,
                               int minimumSamples,
                               Point3DReadOnly volumeCentre)
   {
      if (markerA == null || markerB == null)
         throw new IllegalArgumentException("G3 needs two markers.");
      if (markerA.equals(markerB))
         throw new IllegalArgumentException("G3's two markers must be distinct, both were '" + markerA + "'.");
      if (!(knownLengthMeters > 0.0) || !Double.isFinite(knownLengthMeters))
         throw new IllegalArgumentException("Known artifact length must be positive and finite, was " + knownLengthMeters
               + ". It must be measured independently of mocap -- calipers, not the capture itself.");
      if (!(perAxisSigma > 0.0) || !Double.isFinite(perAxisSigma))
         throw new IllegalArgumentException("Per-axis sigma must be a positive, finite measurement in metres, was " + perAxisSigma
               + ". FRAMEWORK.md §17: it must be measured at the gantry, never assumed.");
      if (!(sigmaMultiplier > 0.0))
         throw new IllegalArgumentException("Sigma multiplier must be positive, was " + sigmaMultiplier + ".");
      if (minimumSamples < 1)
         throw new IllegalArgumentException("A mean needs at least 1 sample, was given " + minimumSamples + ".");
      if (volumeCentre == null)
         throw new IllegalArgumentException("G3 needs a volume centre to bin samples against.");

      this.markerA = markerA;
      this.markerB = markerB;
      this.knownLengthMeters = knownLengthMeters;
      this.perAxisSigma = perAxisSigma;
      this.sigmaMultiplier = sigmaMultiplier;
      this.minimumSamples = minimumSamples;
      this.volumeCentre = volumeCentre;

      for (String side : SIDE_NAMES)
         sides.put(side, new SideStatistics());
   }

   @Override
   public String getName()
   {
      return "G3";
   }

   @Override
   public String getDescription()
   {
      return "volume distortion: a rigid artifact's known length must be measured correctly everywhere in the volume, not merely near the centre";
   }

   /**
    * Folds one frame into the running per-side statistics. Allocation-free; safe to call at the
    * capture rate.
    */
   public void accumulate(MocapFrame frame)
   {
      MarkerObservation a = frame.get(markerA);
      MarkerObservation b = frame.get(markerB);

      if (!a.isVisible() || !b.isVisible())
         return;

      Point3DReadOnly pa = a.getPosition();
      Point3DReadOnly pb = b.getPosition();

      double dx = pa.getX() - pb.getX();
      double dy = pa.getY() - pb.getY();
      double dz = pa.getZ() - pb.getZ();
      double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

      double midX = 0.5 * (pa.getX() + pb.getX());
      double midY = 0.5 * (pa.getY() + pb.getY());
      double midZ = 0.5 * (pa.getZ() + pb.getZ());

      sides.get(midX >= volumeCentre.getX() ? "X+" : "X-").accumulate(distance);
      sides.get(midY >= volumeCentre.getY() ? "Y+" : "Y-").accumulate(distance);
      sides.get(midZ >= volumeCentre.getZ() ? "Z+" : "Z-").accumulate(distance);

      framesAccumulated++;
   }

   public long getFramesAccumulated()
   {
      return framesAccumulated;
   }

   /** The known artifact length this gate checks every side's mean against, in metres. */
   public double getKnownLengthMeters()
   {
      return knownLengthMeters;
   }

   /** The noise floor for a SINGLE distance sample: {@code √2σ}, same derivation as G1. */
   public double getSingleSampleNoiseFloor()
   {
      return Math.sqrt(2.0) * perAxisSigma;
   }

   /** The noise floor for the MEAN of {@code sampleCount} samples: {@code √2σ / √N}. */
   public double getMeanNoiseFloor(long sampleCount)
   {
      return getSingleSampleNoiseFloor() / Math.sqrt(sampleCount);
   }

   /** The threshold in metres for a side with {@code sampleCount} samples: {@code sigmaMultiplier × √2σ / √N}. */
   public double getThreshold(long sampleCount)
   {
      return sigmaMultiplier * getMeanNoiseFloor(sampleCount);
   }

   @Override
   public GateResult run()
   {
      GateResult result = new GateResult(getName());

      for (String side : SIDE_NAMES)
      {
         SideStatistics stats = sides.get(side);

         if (stats.count < minimumSamples)
         {
            result.add(GateResult.Finding.notEvaluated(side,
                                                       stats.count,
                                                       "only " + stats.count + " frames with the wand on this side, need " + minimumSamples
                                                             + "; the sweep did not reach here often enough to judge"));
            continue;
         }

         double threshold = getThreshold(stats.count);
         double bias = Math.abs(stats.mean - knownLengthMeters);
         String detail = String.format("measured %.4f mm vs known %.4f mm (bias %.4f mm, threshold %.4f mm at N=%d, mean noise floor %.4f mm)",
                                       1000.0 * stats.mean,
                                       1000.0 * knownLengthMeters,
                                       1000.0 * bias,
                                       1000.0 * threshold,
                                       stats.count,
                                       1000.0 * getMeanNoiseFloor(stats.count));

         if (bias > threshold)
            result.add(GateResult.Finding.fail(side, bias, threshold, stats.count, detail));
         else
            result.add(GateResult.Finding.pass(side, bias, threshold, stats.count, detail));
      }

      result.setSummary(String.format("known length %.4f mm, sigma %.4f mm, threshold sigmaMultiplier %.1f over %d frames",
                                      1000.0 * knownLengthMeters,
                                      1000.0 * perAxisSigma,
                                      sigmaMultiplier,
                                      framesAccumulated));
      return result;
   }

   /** Drops all accumulated statistics so the gate can be re-run over a different capture. */
   public void reset()
   {
      for (SideStatistics stats : sides.values())
         stats.reset();

      framesAccumulated = 0;
   }

   /** Streaming mean of one side's measured wand length. No cancellation risk: unlike a variance, a mean of positive lengths near a known baseline is well-conditioned in plain floating point. */
   private static final class SideStatistics
   {
      private long count = 0;
      private double mean = 0.0;

      private void accumulate(double distance)
      {
         count++;
         mean += (distance - mean) / count;
      }

      private void reset()
      {
         count = 0;
         mean = 0.0;
      }
   }
}
