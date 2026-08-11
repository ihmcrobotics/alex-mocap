package us.ihmc.alexMocap.calibration;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Does the calibration still work when the markers are scattered over each link rather than grouped
 * into a neat bracket?
 *
 * <h2>The question, and the answer</h2>
 * <p>
 * A bracket -- four markers in a hand-sized patch on one face -- is the convenient case. Markers
 * taped wherever they fit are the real one, and a framework that only works on the former is not
 * much of a framework. Measured on the real Alex model with the leg set, layout recovery against the
 * planted truth:
 * </p>
 * <pre>
 * placement    noise      captures   layout RMS   worst
 * bracket      0          60         0.0000 mm    0.0000 mm
 * bracket      0.3 mm     60         0.2304 mm    0.4051 mm
 * scattered    0          60         0.0000 mm    0.0000 mm
 * scattered    0.3 mm     60         0.7639 mm    1.2277 mm
 * scattered    0.3 mm     200        0.2660 mm    0.4575 mm
 * </pre>
 * <p>
 * <b>Noiseless recovery is exact either way</b>, to the last digit printed. The framework does not
 * care where the markers are; {@code ^i p_ij} is solved for. What scattering costs is
 * <i>conditioning</i>: at equal capture count the layout error is about 3x worse, and 200 captures
 * buys it back.
 * </p>
 *
 * <h2>Why scattering costs anything at all</h2>
 * <p>
 * Not because the limb clusters got worse -- they got bigger, which helps. Because the <b>gauge</b>
 * did. Scattering the pelvis cluster over an ellipsoid around its centre of mass gives a mean
 * cluster radius of 0.148 m against the bracket's 0.165 m, and every capture's base pose comes from
 * registering that cluster, so its angular error multiplies the lever arm out to every other link.
 * CLAUDE.md's measured ordering -- widen the gauge ≫ lower σ ≫ take more captures -- says exactly
 * this would happen.
 * </p>
 * <p>
 * The practical reading: <b>scatter the limb markers, but keep the pelvis gauge a wide outrigger
 * bracket.</b> That is what FRAMEWORK.md §1 asks for anyway, and it is what an operator would build.
 * </p>
 *
 * <h2>In-sample RMS is not layout error, and the gap is large</h2>
 * <p>
 * The calibration report prints in-sample RMS 4.47 mm for the case whose layout error is 0.76 mm --
 * a factor of six. It carries the base-pose fit residual as well as the layout error, which is why
 * {@code CalibrationReport} labels it "NOT an accuracy claim". Anyone reading the demonstration's
 * console and concluding the markers are 5 mm out has misread it; this test measures the thing that
 * matters, which is only measurable because the truth was planted.
 * </p>
 */
public class ScatteredMarkerTest
{
   private static final double SIGMA = 0.3e-3;

   /** Layout recovery against truth, metres: RMS over every marker of every cluster. */
   private static double layoutRootMeanSquare(RobotCaptures.Planted planted, CalibrationResult result)
   {
      double sumSquared = 0.0;
      int count = 0;

      for (ClusterLayout solved : result.getLayouts())
      {
         ClusterLayout truth = planted.plantedLayout(solved.getLinkName());

         for (int j = 0; j < solved.getMarkerCount(); j++)
         {
            double d = new Point3D(solved.getPositionInLinkFrame(j)).distance(new Point3D(truth.getPositionInLinkFrame(j)));
            sumSquared += d * d;
            count++;
         }
      }

      return Math.sqrt(sumSquared / count);
   }

   private static double calibrateAndMeasure(RobotCaptures.Options options) throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(options);
      BaseInitializer.GaugeTracking tracking = BaseInitializer.trackGaugeCluster(planted.captureSet);
      CalibrationResult result = new AlternatingCalibrator().calibrate(planted.captureSet, planted.model, tracking, new CalibrationReport());
      return layoutRootMeanSquare(planted, result);
   }

   private static RobotCaptures.Options scattered(int captures, double noise)
   {
      return new RobotCaptures.Options().captures(captures)
                                        .noise(noise)
                                        .marked(RobotCaptures.PRIMARY_MARKED_LINKS)
                                        .standoff(0.18, 0.12)
                                        .placement(RobotCaptures.MarkerPlacement.SCATTERED);
   }

   /**
    * <b>The claim.</b> With no noise, scattered markers are recovered exactly -- the framework does
    * not care where they are.
    */
   @Test
   public void testScatteredMarkersAreRecoveredExactlyWithoutNoise() throws Exception
   {
      double layoutRms = calibrateAndMeasure(scattered(60, 0.0));

      // Measured 0.0 to printed precision. The threshold is loose against that and still far below
      // anything a real marker set could deliver.
      assertTrue(layoutRms < 1.0e-6, "Scattered layout RMS was " + (1000.0 * layoutRms) + " mm on noiseless data; it should be exact.");
   }

   /** And so are bracketed markers, so the comparison below is like for like. */
   @Test
   public void testBracketedMarkersAreAlsoRecoveredExactlyWithoutNoise() throws Exception
   {
      double layoutRms = calibrateAndMeasure(new RobotCaptures.Options().captures(60)
                                                                        .noise(0.0)
                                                                        .marked(RobotCaptures.PRIMARY_MARKED_LINKS)
                                                                        .standoff(0.18, 0.12));

      assertTrue(layoutRms < 1.0e-6, "Bracket layout RMS was " + (1000.0 * layoutRms) + " mm on noiseless data.");
   }

   /**
    * At the target noise, scattering costs conditioning but stays sub-millimetre.
    * <p>
    * A band, not a point: what it excludes is a recovery that has silently stopped working (RMS
    * climbing past a millimetre) and one that is suspiciously perfect (which would mean the noise is
    * not reaching the solver).
    * </p>
    */
   @Test
   public void testScatteredMarkersStaySubMillimetreAtTheTargetNoise() throws Exception
   {
      double layoutRms = calibrateAndMeasure(scattered(60, SIGMA));

      // Measured 0.7639 mm.
      assertTrue(layoutRms > 0.05e-3, "Layout RMS was " + (1000.0 * layoutRms) + " mm at sigma = 0.3 mm -- is the noise reaching the solver?");
      assertTrue(layoutRms < 1.5e-3, "Layout RMS was " + (1000.0 * layoutRms) + " mm at sigma = 0.3 mm, which is worse than scattering should cost.");
   }

   /** More captures buy the conditioning back, which is the honest fix for a scattered set. */
   @Test
   public void testMoreCapturesRecoverTheConditioningScatteringCosts() throws Exception
   {
      double atSixty = calibrateAndMeasure(scattered(60, SIGMA));
      double atTwoHundred = calibrateAndMeasure(scattered(200, SIGMA));

      // Measured 0.7639 mm -> 0.2660 mm.
      assertTrue(atTwoHundred < atSixty,
                 "200 captures gave " + (1000.0 * atTwoHundred) + " mm against " + (1000.0 * atSixty) + " mm at 60; more data should help.");
      assertTrue(atTwoHundred < 0.5e-3, "At 200 captures the layout RMS was " + (1000.0 * atTwoHundred) + " mm.");
   }
}
