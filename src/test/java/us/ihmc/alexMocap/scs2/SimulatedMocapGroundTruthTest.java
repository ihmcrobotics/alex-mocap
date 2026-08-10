package us.ihmc.alexMocap.scs2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.calibration.RobotCaptures;
import us.ihmc.alexMocap.frames.GravityAlignedWorldFrame;
import us.ihmc.alexMocap.frames.TiltMeasurement;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.sim.SimulatedMocapCamera;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

/**
 * The pipeline driven the way the SCS2 track will drive it.
 *
 * <h2>Two models, on purpose</h2>
 * <p>
 * The "simulation" poses one {@link RobotModelHandle} and the pipeline owns a second. That is not
 * tidiness: {@code KinematicChainCoupler.complete} sets the model's configuration every tick, so a
 * shared handle would have the pipeline overwriting the pose it is being asked to measure. The
 * failure would be quiet -- a CoM that is right whenever the two configurations happen to agree --
 * which is why the separation is exercised here rather than only documented.
 * </p>
 */
public class SimulatedMocapGroundTruthTest
{
   private static final long SEED = 20260810L;
   private static final List<String> MARKED_LINKS = List.of(RobotCaptures.PRIMARY_MARKED_LINKS);

   /** Stands in for the simulated robot: the thing being measured. */
   private record Simulation(RobotModelHandle model, CenterOfMassCalculator centerOfMass, JointBasics floatingJoint)
   {
      static Simulation create() throws Exception
      {
         RobotModelHandle model = RobotCaptures.alexModel();
         return new Simulation(model,
                               new CenterOfMassCalculator(model.getRootBody(), ReferenceFrame.getWorldFrame()),
                               model.getRootBody().getChildrenJoints().get(0));
      }

      void poseRandomly(Random random)
      {
         double[] q = new double[model.getJointCount()];

         for (String jointName : RobotCaptures.LEG_JOINTS)
         {
            int index = model.indexOfJoint(jointName);
            double lower = model.getJointLimitLower(index);
            double upper = model.getJointLimitUpper(index);
            q[index] = lower + random.nextDouble() * (upper - lower);
         }

         model.setQ(q);

         RigidBodyTransform basePose = new RigidBodyTransform();
         basePose.getTranslation().set(0.3 * random.nextDouble(), 0.3 * random.nextDouble(), 0.95 + 0.1 * random.nextDouble());
         basePose.getRotation().setYawPitchRoll(random.nextDouble(), 0.15 * random.nextDouble(), 0.15 * random.nextDouble());
         floatingJoint.setJointConfiguration(basePose);

         model.updateFrames();
         centerOfMass.reset();
      }

      SimulatedMocapCamera.LinkPoseSource poseSource()
      {
         return (linkName, poseToPack) ->
         {
            if (!model.hasLink(linkName))
               return false;

            poseToPack.set(model.getLinkFrame(linkName).getTransformToRoot());
            return true;
         };
      }

      double[] jointAngles()
      {
         double[] q = new double[model.getJointCount()];

         for (int j = 0; j < q.length; j++)
            q[j] = model.getQ(j);

         return q;
      }
   }

   private static GravityAlignedWorldFrame level(String suffix)
   {
      return new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("test"), ReferenceFrame.getWorldFrame(), suffix);
   }

   /**
    * A noiseless run must sit on the simulation's centre of mass exactly, for every tick.
    * <p>
    * This is the end-to-end version of the loop-closing check: the pipeline's own model, its own
    * chaining, its own weighted sum, against Mecano's recursion on the simulation's model. Nothing
    * is shared between the two but the joint angles and the marker positions.
    * </p>
    */
   @Test
   public void testNoiselessPipelineTracksTheSimulationExactly() throws Exception
   {
      Simulation simulation = Simulation.create();
      RobotModelHandle pipelineModel = RobotCaptures.alexModel();
      assertNotSame(simulation.model(), pipelineModel, "The pipeline must not share the simulation's frame tree.");

      SimulatedMocapGroundTruth pipeline = SimulatedMocapGroundTruth.demonstration("test", pipelineModel, MARKED_LINKS, 0.0, 0.0, SEED, level("_exact"));

      assertTrue(pipeline.isUsingPlantedLayout(), "demonstration() runs on the planted layout, and must say so.");

      Random random = new Random(SEED);

      for (int tick = 0; tick < 50; tick++)
      {
         simulation.poseRandomly(random);

         boolean valid = pipeline.update(simulation.poseSource(), simulation.jointAngles(), simulation.centerOfMass().getCenterOfMass());

         assertTrue(valid, "tick " + tick + ": a noiseless, unoccluded camera must always produce a CoM.");
         assertEquals(0, pipeline.getComparison().getRefusedFrameCount().getValue(), 0.0);
      }

      // Measured 4.66e-16 m maximum over these 50 ticks.
      assertTrue(pipeline.getComparison().getComErrorMaximum().getValue() < 1.0e-9,
                 "Worst CoM error was " + pipeline.getComparison().getComErrorMaximum().getValue() + " m; " + pipeline.summary());
      assertEquals(50.0, pipeline.getComparison().getValidFrameCount().getValue(), 0.0);
      // 7 clusters x 4 markers.
      assertEquals(28, pipeline.getMarkerVariables().getVisibleMarkerCount().getValue(), "An unoccluded camera sees every marker every tick.");
   }

   /**
    * With marker noise the error is millimetres, and it is jitter rather than a bias.
    * <p>
    * The mean-versus-standard-deviation split is the assertion that matters. Marker noise is
    * zero-mean, so a systematic offset appearing here would mean something in F6-F9 is biased --
    * exactly the fault a magnitude trace hides.
    * </p>
    */
   @Test
   public void testNoisyPipelineIsJitteryNotBiased() throws Exception
   {
      Simulation simulation = Simulation.create();
      RobotModelHandle pipelineModel = RobotCaptures.alexModel();

      SimulatedMocapGroundTruth pipeline = SimulatedMocapGroundTruth.demonstration("testNoisy",
                                                                                    pipelineModel,
                                                                                    MARKED_LINKS,
                                                                                    SimulatedMocapCamera.GANTRY_NOISE_STANDARD_DEVIATION,
                                                                                    0.0,
                                                                                    SEED,
                                                                                    level("_noisy"));

      Random random = new Random(SEED);
      Point3D signedSum = new Point3D();
      int ticks = 200;

      for (int tick = 0; tick < ticks; tick++)
      {
         simulation.poseRandomly(random);
         assertTrue(pipeline.update(simulation.poseSource(), simulation.jointAngles(), simulation.centerOfMass().getCenterOfMass()));
         signedSum.add(pipeline.getComparison().getComError());
      }

      double mean = pipeline.getComparison().getComErrorMean().getValue();
      double standardDeviation = pipeline.getComparison().getComErrorStandardDeviation().getValue();

      // Measured at sigma = 0.3 mm over 200 ticks: mean 0.488 mm, sd 0.243 mm, max 1.349 mm.
      // The band excludes noise that never arrives and noise amplified out of proportion; see
      // SimulatedMocapTest for the same reasoning.
      assertTrue(mean > 0.05e-3 && mean < 5.0e-3, "CoM error mean was " + (1000.0 * mean) + " mm. " + pipeline.summary());
      assertTrue(standardDeviation > 0.0, "A noisy camera must produce a spread, not a constant.");

      // Zero-mean noise: the *signed* error averaged over 200 ticks must be far smaller than the
      // magnitude mean. A signed average comparable to the magnitude would be a bias, not noise.
      signedSum.scale(1.0 / ticks);
      assertTrue(signedSum.norm() < 0.5 * mean,
                 "The signed CoM error averaged " + (1000.0 * signedSum.norm()) + " mm against a magnitude mean of " + (1000.0 * mean)
                       + " mm, which is a bias rather than marker noise.");
   }

   /**
    * Occlusion refuses frames rather than degrading them, and the refusals are counted.
    *
    * <h2>Measured: a 12 % per-marker drop rate refuses 63 % of frames, not 41 %</h2>
    * <p>
    * The naive arithmetic says a four-marker cluster loses two or more with probability
    * {@code 1 - 0.88⁴ - 4·0.12·0.88³ = 7.3 %}, so across seven clusters about {@code 1 - 0.927⁷ =
    * 41 %} of frames should be refused. The measurement is <b>126 of 200</b>, which implies a
    * per-cluster refusal near 13 % -- close to double.
    * </p>
    * <p>
    * The gap is the {@code σ₂} guard, and it is worth understanding rather than tuning away. Dropping
    * one marker of four does not merely leave a workable three-point cluster: it removes a quarter of
    * the constellation's spread, and {@code LinkPoseEstimator}'s threshold is
    * {@code DEFAULT_SIGMA2_FRACTION = 0.25} of nominal {@code σ₂}. A three-marker remnant clears that
    * bar most of the time but not always, and the roughly 6 % of frames between the two numbers are
    * clusters that had enough markers and still could not be trusted.
    * </p>
    * <p>
    * The practical reading: <b>four markers per cluster is not "three plus a spare"</b>. Occlusion
    * budgets computed from {@code MarkerCluster.MINIMUM_MARKERS} alone will be optimistic, and five
    * markers on the limbs would buy more than the count suggests. Note also that this camera's
    * occlusion is memoryless and per-marker, which is the <i>friendly</i> case -- a real dropout
    * takes the same markers for many consecutive frames.
    * </p>
    */
   @Test
   public void testOcclusionProducesRefusalsAndNeverASilentlyWrongCenterOfMass() throws Exception
   {
      Simulation simulation = Simulation.create();
      RobotModelHandle pipelineModel = RobotCaptures.alexModel();

      SimulatedMocapGroundTruth pipeline = SimulatedMocapGroundTruth.demonstration("testOccluded",
                                                                                    pipelineModel,
                                                                                    MARKED_LINKS,
                                                                                    0.0,
                                                                                    0.12,
                                                                                    SEED,
                                                                                    level("_occluded"));

      Random random = new Random(SEED);
      int ticks = 200;
      int refusedTicks = 0;

      for (int tick = 0; tick < ticks; tick++)
      {
         simulation.poseRandomly(random);
         boolean valid = pipeline.update(simulation.poseSource(), simulation.jointAngles(), simulation.centerOfMass().getCenterOfMass());

         if (!valid)
         {
            refusedTicks++;
            assertTrue(pipeline.getCenterOfMass().containsNaN(), "A refused tick must leave NaN, never a partial sum.");
            assertTrue(pipeline.getComparison().getComErrorMagnitude().isNaN());
         }
      }

      // Measured 126 of 200. The band is wide because the seed is what fixes the exact count, but
      // its lower edge sits above the 41 % the naive binomial predicts -- so if the σ₂ guard were
      // ever quietly relaxed, this test would notice.
      assertTrue(refusedTicks > 0, "At a 12 % drop rate some frames must be refused, or the occlusion is not wired in.");
      assertTrue(refusedTicks > 0.45 * ticks && refusedTicks < 0.80 * ticks,
                 "Refused " + refusedTicks + " of " + ticks + " frames; expected around 63 % -- see this test's javadoc for why it is not 41 %.");
      assertEquals(refusedTicks, (int) pipeline.getComparison().getRefusedFrameCount().getValue());
      assertEquals(ticks - refusedTicks, (int) pipeline.getComparison().getValidFrameCount().getValue());

      // Every accepted frame is still exact: occlusion refuses, it does not degrade. A cluster that
      // kept three of four markers registers just as well as one that kept all four.
      assertTrue(pipeline.getComparison().getComErrorMaximum().getValue() < 1.0e-9,
                 "Accepted frames must stay exact under a noiseless camera; worst was "
                       + pipeline.getComparison().getComErrorMaximum().getValue() + " m.");
   }

   /** The pipeline mutates its own model and leaves the simulation's alone. */
   @Test
   public void testPipelineDoesNotDisturbTheSimulationsModel() throws Exception
   {
      Simulation simulation = Simulation.create();
      RobotModelHandle pipelineModel = RobotCaptures.alexModel();
      SimulatedMocapGroundTruth pipeline = SimulatedMocapGroundTruth.demonstration("testIsolation",
                                                                                    pipelineModel,
                                                                                    MARKED_LINKS,
                                                                                    0.0,
                                                                                    0.0,
                                                                                    SEED,
                                                                                    level("_isolation"));

      Random random = new Random(SEED);
      simulation.poseRandomly(random);

      double[] before = simulation.jointAngles();
      RigidBodyTransform pelvisBefore = new RigidBodyTransform(simulation.model().getLinkFrame("PELVIS_LINK").getTransformToRoot());

      pipeline.update(simulation.poseSource(), simulation.jointAngles(), simulation.centerOfMass().getCenterOfMass());

      double[] after = simulation.jointAngles();

      for (int j = 0; j < before.length; j++)
         assertEquals(before[j], after[j], 0.0, "The pipeline moved the simulation's joint " + simulation.model().getJointNames().get(j) + ".");

      assertTrue(pelvisBefore.epsilonEquals(simulation.model().getLinkFrame("PELVIS_LINK").getTransformToRoot(), 0.0),
                 "The pipeline moved the simulation's base.");

      assertFalse(pipeline.getModel() == simulation.model());
   }

   @Test
   public void testGraphicsGroupCarriesMarkersAndBothCentresOfMass() throws Exception
   {
      RobotModelHandle pipelineModel = RobotCaptures.alexModel();
      SimulatedMocapGroundTruth pipeline = SimulatedMocapGroundTruth.demonstration("testGraphics",
                                                                                    pipelineModel,
                                                                                    MARKED_LINKS,
                                                                                    0.0,
                                                                                    0.0,
                                                                                    SEED,
                                                                                    level("_graphics"));

      // The marker cloud and the CoM pair.
      assertEquals(2, ((us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition) pipeline.createYoGraphics("mocap")).getChildren().size());
      assertTrue(pipeline.summary().contains("planted layout"), "The caveat must travel with the summary: " + pipeline.summary());
   }
}
