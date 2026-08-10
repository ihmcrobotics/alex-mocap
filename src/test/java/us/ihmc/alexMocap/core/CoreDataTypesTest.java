package us.ihmc.alexMocap.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.AllocationMeasurement;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Behaviour of the {@code core} value types, concentrated on the invariants that would otherwise
 * fail silently.
 */
public class CoreDataTypesTest
{
   private static final List<MarkerId> MARKER_SET = MarkerId.createDenseSet("PELVIS_1", "PELVIS_2", "PELVIS_3", "PELVIS_4", "L_THIGH_1", "L_THIGH_2");
   private static final List<String> JOINT_NAMES = List.of("l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll");

   @Test
   public void testDenseSetAssignsIndicesByPosition()
   {
      for (int i = 0; i < MARKER_SET.size(); i++)
         assertEquals(i, MARKER_SET.get(i).getIndex());

      MarkerId.checkDenseSet(MARKER_SET);
      assertThrows(IllegalArgumentException.class, () -> MarkerId.createDenseSet("A", "B", "A"));
      assertThrows(IllegalArgumentException.class, () -> MarkerId.checkDenseSet(List.of(new MarkerId("A", 0), new MarkerId("B", 2))));
   }

   /**
    * The reason {@link MocapFrame#get(MarkerId)} verifies identity instead of trusting the index.
    * Two independently built marker sets assign the same index to different markers, and a frame
    * that trusted the index would hand back someone else's marker -- producing a pose that is
    * wrong and looks entirely healthy.
    */
   @Test
   public void testFrameRejectsMarkerFromADifferentSet()
   {
      MocapFrame frame = new MocapFrame(MARKER_SET);
      List<MarkerId> otherSet = MarkerId.createDenseSet("TORSO_1", "TORSO_2", "TORSO_3");

      assertEquals(1, otherSet.get(1).getIndex());
      assertEquals(1, MARKER_SET.get(1).getIndex());

      IllegalArgumentException thrown = assertThrows(IllegalArgumentException.class, () -> frame.get(otherSet.get(1)));
      assertTrue(thrown.getMessage().contains("TORSO_2"));

      // Out of range is caught by the same check, not by an ArrayIndexOutOfBoundsException.
      assertThrows(IllegalArgumentException.class, () -> frame.get(new MarkerId("PELVIS_1", 99)));
   }

   /**
    * An invisible marker's position is NaN, never stale. A stale position looks plausible,
    * registers without complaint, and drags the recovered pose toward where the marker used to be.
    */
   @Test
   public void testInvisibleMarkerPositionIsNaN()
   {
      MocapFrame frame = new MocapFrame(MARKER_SET);
      MarkerObservation observation = frame.get(MARKER_SET.get(0));

      assertFalse(observation.isVisible());
      assertTrue(observation.getPosition().containsNaN(), "A fresh observation must start NaN, not at the origin.");

      observation.setVisible(1.0, 2.0, 3.0);
      assertTrue(observation.isVisible());
      assertEquals(2.0, observation.getPosition().getY(), 0.0);

      observation.setNotVisible();
      assertFalse(observation.isVisible());
      assertTrue(observation.getPosition().containsNaN(), "Going invisible must clear the position, not leave the last one.");
   }

   @Test
   public void testVisibleCounts()
   {
      MocapFrame frame = new MocapFrame(MARKER_SET);
      MarkerCluster pelvis = new MarkerCluster("pelvis", MARKER_SET.subList(0, 4));

      assertEquals(0, frame.getVisibleCount());
      assertEquals(0, frame.getVisibleCount(pelvis));

      frame.get(MARKER_SET.get(0)).setVisible(0.0, 0.0, 0.0);
      frame.get(MARKER_SET.get(1)).setVisible(0.1, 0.0, 0.0);
      frame.get(MARKER_SET.get(4)).setVisible(0.0, 0.5, 0.0);

      assertEquals(3, frame.getVisibleCount());
      assertEquals(2, frame.getVisibleCount(pelvis), "The cluster count must ignore markers on other links.");

      frame.clear();
      assertEquals(0, frame.getVisibleCount());
      assertEquals(MocapFrame.NO_TIMESTAMP, frame.getTimestampNanoseconds());
   }

   @Test
   public void testFrameCopyIsFaithful()
   {
      MocapFrame source = new MocapFrame(MARKER_SET);
      MocapFrame destination = new MocapFrame(MARKER_SET);

      source.setTimestampNanoseconds(123_456_789L);
      source.get(MARKER_SET.get(0)).setVisible(0.06, -0.03, 0.09);
      source.get(MARKER_SET.get(2)).setVisible(-0.05, 0.05, 0.09);

      destination.set(source);

      assertEquals(123_456_789L, destination.getTimestampNanoseconds());
      assertEquals(2, destination.getVisibleCount());
      assertEquals(0.06, destination.get(MARKER_SET.get(0)).getPosition().getX(), 0.0);
      assertFalse(destination.get(MARKER_SET.get(1)).isVisible());
      assertTrue(destination.get(MARKER_SET.get(1)).getPosition().containsNaN());

      assertThrows(IllegalArgumentException.class, () -> destination.set(new MocapFrame(MarkerId.createDenseSet("A", "B"))));
   }

   /** A cluster below three markers cannot produce a pose at all; that is a config typo. */
   @Test
   public void testClusterMinimumMembers()
   {
      assertThrows(IllegalArgumentException.class, () -> new MarkerCluster("pelvis", MARKER_SET.subList(0, 2)));
      assertThrows(IllegalArgumentException.class, () -> new MarkerCluster("pelvis", MARKER_SET.get(0), MARKER_SET.get(0), MARKER_SET.get(1)));

      MarkerCluster minimal = new MarkerCluster("pelvis", MARKER_SET.subList(0, 3));
      assertEquals(3, minimal.getMarkerCount());
      assertFalse(minimal.hasRecommendedRedundancy(), "Three markers give G1 nothing redundant to check.");

      MarkerCluster recommended = new MarkerCluster("pelvis", MARKER_SET.subList(0, 4));
      assertTrue(recommended.hasRecommendedRedundancy());
      assertTrue(recommended.contains(MARKER_SET.get(3)));
      assertFalse(recommended.contains(MARKER_SET.get(4)));
   }

   /**
    * The instrumentation for FRAMEWORK.md §18.3. A mispaired capture is valid mocap plus valid
    * encoders at the wrong configuration, and nothing about it looks wrong -- so the skew has to
    * be a number someone can read, reported and never thresholded.
    */
   @Test
   public void testCaptureSkewIsReportedNotJudged()
   {
      Capture capture = Capture.create(MARKER_SET, JOINT_NAMES);

      assertFalse(capture.hasTimestamps());
      assertEquals(MocapFrame.NO_TIMESTAMP, capture.getTimestampSkewNanoseconds());

      capture.getMocapFrame().setTimestampNanoseconds(1_000_000_000L);
      assertFalse(capture.hasTimestamps(), "One timestamp is not enough to compute a skew.");

      capture.getEncoderSample().setTimestampNanoseconds(1_000_400_000L);
      assertTrue(capture.hasTimestamps());
      assertEquals(-400_000L, capture.getTimestampSkewNanoseconds(), "Mocap 0.4 ms before the encoders is a skew of -400 us.");
   }

   @Test
   public void testEncoderJointOrderIsCheckable()
   {
      EncoderSample sample = new EncoderSample(JOINT_NAMES);

      assertEquals(JOINT_NAMES.size(), sample.getJointCount());
      assertTrue(Double.isNaN(sample.getQ(0)), "Unset joints must be NaN, not 0.0 -- zero is a legal joint angle.");

      sample.checkJointOrder(JOINT_NAMES);

      List<String> permuted = List.of("l_hip_roll", "l_hip_yaw", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll");
      assertThrows(IllegalArgumentException.class, () -> sample.checkJointOrder(permuted));

      sample.setQ(new double[] {0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
      double[] packed = new double[JOINT_NAMES.size()];
      sample.getQ(packed);
      assertEquals(0.3, packed[2], 0.0);

      assertThrows(IllegalArgumentException.class, () -> sample.setQ(new double[3]));
      assertThrows(IllegalArgumentException.class, () -> sample.getQ(new double[3]));
   }

   @Test
   public void testLayoutTracksObservationCounts()
   {
      ClusterLayout layout = new ClusterLayout(new MarkerCluster("pelvis", MARKER_SET.subList(0, 4)));

      assertFalse(layout.isFullySolved(), "An un-solved layout must not read as solved at the origin.");
      assertEquals(0, layout.getMinimumObservationCount());

      for (int i = 0; i < 4; i++)
         layout.setPositionInLinkFrame(i, new Point3D(0.01 * i, 0.02, 0.03), 30);

      assertTrue(layout.isFullySolved());
      assertEquals(30, layout.getMinimumObservationCount());

      // A marker seen in 3 of 30 captures is ~3x noisier than one seen in all 30, with nothing in
      // the position itself to say so. K_ij is what says so.
      layout.setPositionInLinkFrame(MARKER_SET.get(2), new Point3D(0.5, 0.5, 0.5), 3);
      assertEquals(3, layout.getMinimumObservationCount());
      assertEquals(3, layout.getObservationCount(MARKER_SET.get(2)));
      assertTrue(layout.isFullySolved(), "A sparsely observed marker still has a position; it is just a worse one.");

      layout.setNotObserved(1);
      assertFalse(layout.isFullySolved());

      assertThrows(IllegalArgumentException.class, () -> layout.getPositionInLinkFrame(MARKER_SET.get(5)));
      assertThrows(IllegalArgumentException.class, () -> layout.setPositionInLinkFrame(0, new Point3D(), -1));
   }

   @Test
   public void testCalibrationResultLayoutLookup()
   {
      CalibrationResult result = new CalibrationResult();
      result.addLayout(new ClusterLayout("pelvis", MARKER_SET.subList(0, 4)));
      result.addLayout(new ClusterLayout("l_thigh", MARKER_SET.subList(4, 6)));

      assertEquals(List.of("pelvis", "l_thigh"), result.getLinkNames());
      assertEquals("l_thigh", result.getLayout("l_thigh").getLinkName());
      assertThrows(IllegalArgumentException.class, () -> result.getLayout("torso"));
      assertThrows(IllegalArgumentException.class, () -> result.addLayout(new ClusterLayout("pelvis", MARKER_SET.subList(0, 4))));
      assertFalse(result.isFullySolved());
   }

   /**
    * FRAMEWORK.md §13: a centred Savitzky-Golay window cannot execute causally, so velocity is not
    * a runtime quantity. If {@code GroundTruthSample} carried a velocity field, someone would fill
    * it by differencing and compare 0.13 m/s of noise against a 0.025 m/s estimator.
    * <p>
    * PR3 asserts this by reflection on {@code PelvisStateExtractor}. The same discipline starts
    * here, on the record that type writes into.
    */
   @Test
   public void testGroundTruthSampleExposesNoVelocity()
   {
      for (Method method : GroundTruthSample.class.getDeclaredMethods())
      {
         String name = method.getName().toLowerCase();
         assertFalse(name.contains("velocity") || name.contains("twist") || name.contains("linearrate") || name.contains("angularrate"),
                     "GroundTruthSample must expose no velocity accessor, found: " + method);
      }
   }

   @Test
   public void testGroundTruthSampleConditioning()
   {
      GroundTruthSample sample = new GroundTruthSample(List.of("pelvis", "l_thigh", "r_thigh"));

      assertTrue(sample.getCenterOfMass().containsNaN());
      assertFalse(sample.allPosesAccepted(), "An unwritten sample must not read as fully accepted.");
      assertTrue(Double.isNaN(sample.getSigma3(0)));

      sample.setTimestampNanoseconds(42L);
      sample.setCenterOfMass(new Point3D(0.01, -0.02, 0.81));
      sample.setConditioning(0, 0.0031, 4, true);
      sample.setConditioning(1, 0.0028, 4, true);
      sample.setConditioning(2, 1.2e-7, 2, false);

      assertEquals(2, sample.indexOfLink("r_thigh"));
      assertFalse(sample.allPosesAccepted(), "One refused link means not all poses were accepted.");
      assertFalse(sample.isPoseAccepted(2));
      assertEquals(2, sample.getVisibleCount(2));
      assertEquals(0.81, sample.getCenterOfMass().getZ(), 0.0);
      assertThrows(IllegalArgumentException.class, () -> sample.indexOfLink("torso"));

      GroundTruthSample copy = new GroundTruthSample(List.of("pelvis", "l_thigh", "r_thigh"));
      copy.set(sample);
      assertEquals(42L, copy.getTimestampNanoseconds());
      assertFalse(copy.isPoseAccepted(2));
      assertEquals(1.2e-7, copy.getSigma3(2), 0.0);

      copy.clear();
      assertTrue(copy.getCenterOfMass().containsNaN());
      assertFalse(copy.isPoseAccepted(0));
   }

   /**
    * The runtime path through {@code core} allocates nothing: filling a frame, reading it back,
    * and writing a ground-truth sample all run against preallocated storage.
    */
   @Test
   public void testRuntimePathIsAllocationFree()
   {
      MocapFrame frame = new MocapFrame(MARKER_SET);
      MocapFrame copy = new MocapFrame(MARKER_SET);
      MarkerCluster pelvis = new MarkerCluster("pelvis", MARKER_SET.subList(0, 4));
      GroundTruthSample sample = new GroundTruthSample(List.of("pelvis"));
      Point3D centerOfMass = new Point3D(0.01, -0.02, 0.81);

      AllocationMeasurement.assertAllocationFree("10,000 frame cycles", () -> runFrameLoop(frame, copy, pelvis, sample, centerOfMass, 10_000));

      assertEquals(4, copy.getVisibleCount());
   }

   private static void runFrameLoop(MocapFrame frame,
                                    MocapFrame copy,
                                    MarkerCluster cluster,
                                    GroundTruthSample sample,
                                    Point3D centerOfMass,
                                    int iterations)
   {
      for (int i = 0; i < iterations; i++)
      {
         frame.clear();
         frame.setTimestampNanoseconds(i * 5_000_000L);

         for (int j = 0; j < cluster.getMarkerCount(); j++)
            frame.get(cluster.getMarker(j)).setVisible(0.01 * j, 0.02, 0.03);

         int visible = frame.getVisibleCount(cluster);
         copy.set(frame);

         sample.clear();
         sample.setTimestampNanoseconds(frame.getTimestampNanoseconds());
         sample.setCenterOfMass(centerOfMass);
         sample.setConditioning(0, 0.003, visible, visible >= 3);
      }
   }
}
