package us.ihmc.alexMocap.scs2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerId;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * The comparison and marker telemetry, on hand-built numbers.
 * <p>
 * Everything here is arithmetic that can be checked by eye. That is the point: these classes are
 * what a reader will believe a CoM error from, so their statistics have to be verifiable without
 * running the pipeline that feeds them.
 * </p>
 */
public class GroundTruthComparisonYoVariablesTest
{
   private static final double EPSILON = 1.0e-12;

   private static GroundTruthComparisonYoVariables comparison(String suffix)
   {
      return new GroundTruthComparisonYoVariables(suffix, ReferenceFrame.getWorldFrame());
   }

   @Test
   public void testIdenticalCentresOfMassGiveZeroError()
   {
      GroundTruthComparisonYoVariables variables = comparison("_zero");
      Point3D point = new Point3D(0.1, -0.2, 0.9);

      variables.update(point, point);

      assertTrue(variables.getComValid().getValue());
      assertEquals(0.0, variables.getComErrorMagnitude().getValue(), EPSILON);
      assertEquals(0.0, variables.getComError().norm(), EPSILON);
      assertEquals(1.0, variables.getValidFrameCount().getValue(), EPSILON);
      assertEquals(0.0, variables.getRefusedFrameCount().getValue(), EPSILON);
   }

   /** The error is {@code mocap - actual}, in that order, per axis. A sign flip here is silent. */
   @Test
   public void testErrorSignAndMagnitude()
   {
      GroundTruthComparisonYoVariables variables = comparison("_sign");

      variables.update(new Point3D(1.0, 2.0, 3.0), new Point3D(0.0, 0.0, 0.0));

      assertEquals(1.0, variables.getComError().getX(), EPSILON);
      assertEquals(2.0, variables.getComError().getY(), EPSILON);
      assertEquals(3.0, variables.getComError().getZ(), EPSILON);
      assertEquals(Math.sqrt(14.0), variables.getComErrorMagnitude().getValue(), EPSILON);

      variables.update(new Point3D(0.0, 0.0, 0.0), new Point3D(1.0, 0.0, 0.0));
      assertEquals(-1.0, variables.getComError().getX(), EPSILON, "mocap - actual, not the other way round.");
   }

   /**
    * Mean and standard deviation over magnitudes chosen so both are exact.
    * <p>
    * Magnitudes 1, 3 along +x: mean 2, population sd 1.
    * </p>
    */
   @Test
   public void testRunningStatisticsAreCorrect()
   {
      GroundTruthComparisonYoVariables variables = comparison("_stats");
      Point3D origin = new Point3D();

      variables.update(new Point3D(1.0, 0.0, 0.0), origin);
      variables.update(new Point3D(3.0, 0.0, 0.0), origin);

      assertEquals(2.0, variables.getComErrorMean().getValue(), EPSILON);
      assertEquals(1.0, variables.getComErrorStandardDeviation().getValue(), EPSILON);
      assertEquals(3.0, variables.getComErrorMaximum().getValue(), EPSILON);
      assertEquals(2.0, variables.getValidFrameCount().getValue(), EPSILON);
   }

   /** A single frame must give sd exactly 0, not NaN from a negative round-off under the root. */
   @Test
   public void testFirstFrameGivesZeroStandardDeviationNotNaN()
   {
      GroundTruthComparisonYoVariables variables = comparison("_single");

      variables.update(new Point3D(0.007, 0.0, 0.0), new Point3D());

      assertEquals(0.0, variables.getComErrorStandardDeviation().getValue(), EPSILON);
      assertFalse(Double.isNaN(variables.getComErrorStandardDeviation().getValue()));
   }

   /**
    * A refused frame is NaN everywhere downstream, is counted, and does not enter the statistics.
    * <p>
    * The last assertion is the one with teeth: a mean that quietly skipped the bad frames while the
    * count kept rising would understate the error and look healthier the worse the occlusion got.
    * </p>
    */
   @Test
   public void testRefusedFrameIsNaNAndExcludedFromStatistics()
   {
      GroundTruthComparisonYoVariables variables = comparison("_refused");
      Point3D origin = new Point3D();

      variables.update(new Point3D(1.0, 0.0, 0.0), origin);
      variables.update(new Point3D(3.0, 0.0, 0.0), origin);

      double meanBefore = variables.getComErrorMean().getValue();

      Point3D refused = new Point3D(Double.NaN, Double.NaN, Double.NaN);
      variables.update(refused, new Point3D(0.5, 0.0, 0.0));

      assertFalse(variables.getComValid().getValue());
      assertTrue(Double.isNaN(variables.getComErrorMagnitude().getValue()), "A refused frame must not hold the last good error.");
      assertTrue(variables.getComError().containsNaN());
      assertTrue(variables.getMocapCom().containsNaN());

      // The actual CoM is still published: it is known even when the mocap answer is not, and it is
      // what tells you where the robot was during the dropout.
      assertEquals(0.5, variables.getActualCom().getX(), EPSILON);

      assertEquals(1.0, variables.getRefusedFrameCount().getValue(), EPSILON);
      assertEquals(2.0, variables.getValidFrameCount().getValue(), EPSILON, "A refused frame is not a valid frame.");
      assertEquals(meanBefore, variables.getComErrorMean().getValue(), EPSILON, "A refused frame must not move the mean.");
   }

   @Test
   public void testResetClearsStatisticsOnly()
   {
      GroundTruthComparisonYoVariables variables = comparison("_reset");
      variables.update(new Point3D(1.0, 0.0, 0.0), new Point3D());

      variables.reset();

      assertEquals(0.0, variables.getValidFrameCount().getValue(), EPSILON);
      assertEquals(0.0, variables.getRefusedFrameCount().getValue(), EPSILON);
      assertTrue(Double.isNaN(variables.getComErrorMean().getValue()));
   }

   // ---------------------------------------------------------------------------------------------
   // MocapMarkerYoVariables
   // ---------------------------------------------------------------------------------------------

   private static MocapMarkerYoVariables markerVariables(String suffix)
   {
      List<MarkerId> markers = MarkerId.createDenseSet("A0", "A1", "A2", "B0", "B1", "B2");
      List<MarkerCluster> clusters = List.of(new MarkerCluster("LINK_A", markers.subList(0, 3)),
                                              new MarkerCluster("LINK_B", markers.subList(3, 6)));

      return new MocapMarkerYoVariables(suffix, clusters, markers, ReferenceFrame.getWorldFrame());
   }

   @Test
   public void testMarkersStartHiddenAndOccludedMarkersGoNaN()
   {
      MocapMarkerYoVariables variables = markerVariables("_markers");

      assertEquals(0, variables.getVisibleMarkerCount().getValue(), "Nothing has been observed yet.");

      MocapFrame frame = new MocapFrame(variables.getMarkers());
      frame.get(0).setVisible(new Point3D(1.0, 2.0, 3.0));
      frame.get(1).setVisible(new Point3D(1.1, 2.0, 3.0));
      // Markers 2..5 left not visible.

      variables.update(frame);

      assertEquals(2, variables.getVisibleMarkerCount().getValue());
      assertEquals(6, variables.getMarkers().size());
   }

   @Test
   public void testYoGraphicsGroupMirrorsTheClusters()
   {
      MocapMarkerYoVariables variables = markerVariables("_graphics");

      // Two clusters, three markers each: the tree the operator navigates in the visualizer should
      // be grouped by link, not a flat list of six spheres.
      assertEquals(2, variables.createYoGraphics("markers").getChildren().size());
   }

   @Test
   public void testForeignFrameIsRejected()
   {
      MocapMarkerYoVariables variables = markerVariables("_foreign");
      MocapFrame foreign = new MocapFrame(MarkerId.createDenseSet("X", "Y"));

      org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class, () -> variables.update(foreign));
   }
}
