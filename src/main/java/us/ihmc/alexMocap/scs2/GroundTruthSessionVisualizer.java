package us.ihmc.alexMocap.scs2;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.GroundTruthSample;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Opens SCS2 on a computed ground-truth trajectory: the robot at each logged configuration, with
 * the CoM and pelvis pose drawn over it and every conditioning variable available to plot.
 *
 * <h2>It replays; it does not simulate</h2>
 * <p>
 * The session is built with {@code newDoNothingPhysicsEngineFactory()}. That is the whole trick:
 * SCS2's session machinery gives the timeline, the buffer, the plotting and the 3-D view for free,
 * while the physics engine that would otherwise integrate the robot forward is replaced by nothing
 * at all. Each tick simply pushes one logged sample into the YoVariables and sets the robot's
 * joints to that capture's encoder reading. A simulating session would drop the robot on the floor
 * and overwrite the very poses being inspected.
 * </p>
 *
 * <h2>No test, by design</h2>
 * <p>
 * PR_PLAN.md: "Visualiser opens. It gets no test -- if a JavaFX window does not appear you will
 * know within seconds." Everything worth asserting about the data has already been asserted
 * headlessly by the time it reaches this class, and a test that starts a toolkit would make the
 * suite need a display, which is the one thing FRAMEWORK.md §19 spends a dependency rule to
 * prevent.
 * </p>
 * <p>
 * This is also the only class in the project that touches JavaFX. {@code PackageDependencyTest}
 * enforces that, so the rest of the pipeline stays runnable over SSH.
 * </p>
 */
public class GroundTruthSessionVisualizer
{
   private GroundTruthSessionVisualizer()
   {
   }

   /**
    * Opens the visualizer and blocks until the window is closed.
    *
    * @param urdfFile            the robot to draw.
    * @param samples             the computed ground truth, in order.
    * @param encoderSamples      the matching encoder readings, used to pose the robot.
    * @param gravityAlignedWorld {@code Wg}: the frame the CoM and pelvis pose are expressed in.
    * @param sampleRateHz        playback rate.
    */
   public static void show(Path urdfFile,
                           List<GroundTruthSample> samples,
                           List<EncoderSample> encoderSamples,
                           ReferenceFrame gravityAlignedWorld,
                           double sampleRateHz)
         throws IOException
   {
      if (samples.isEmpty())
         throw new IllegalArgumentException("Nothing to show: the trajectory is empty.");
      if (samples.size() != encoderSamples.size())
         throw new IllegalArgumentException("Got " + samples.size() + " ground truth samples and " + encoderSamples.size() + " encoder samples.");

      RobotDefinition robotDefinition = loadRobotDefinition(urdfFile);

      SimulationSession session = new SimulationSession(PhysicsEngineFactory.newDoNothingPhysicsEngineFactory());
      session.setSessionDTSeconds(1.0 / sampleRateHz);

      Robot robot = session.addRobot(robotDefinition);

      GroundTruthYoVariables variables = new GroundTruthYoVariables("gt", samples.get(0).getLinkNames(), gravityAlignedWorld);
      session.getRootRegistry().addChild(variables.getRegistry());
      session.addYoGraphicDefinition(GroundTruthYoGraphics.create("groundTruth", variables));

      int[] sampleIndex = {0};

      session.addBeforePhysicsCallback(time ->
      {
         int index = Math.min(sampleIndex[0], samples.size() - 1);

         variables.update(samples.get(index));
         setRobotConfiguration(robot, encoderSamples.get(index));

         sampleIndex[0] = index + 1;
      });

      SessionVisualizer.startSessionVisualizer(session);
   }

   /** Poses the drawn robot at one capture's encoder reading. */
   private static void setRobotConfiguration(Robot robot, EncoderSample encoderSample)
   {
      for (int i = 0; i < encoderSample.getJointCount(); i++)
      {
         String jointName = encoderSample.getJointNames().get(i);

         if (robot.getJoint(jointName) instanceof OneDoFJointBasics joint)
            joint.setQ(encoderSample.getQ(i));
      }

      robot.getRootBody().updateFramesRecursively();
   }

   private static RobotDefinition loadRobotDefinition(Path urdfFile) throws IOException
   {
      // Loaded here rather than through model.URDFLoader on purpose: that class returns a Mecano
      // tree and deliberately lets no SCS2 type escape into a signature, which is what keeps the
      // rest of the project free of this dependency. The visualizer is the one place a
      // RobotDefinition is the thing actually wanted.
      try
      {
         return URDFTools.toRobotDefinition(URDFTools.loadURDFModel(new File(urdfFile.toString())));
      }
      catch (Exception e)
      {
         throw new IOException("Failed to load the URDF at " + urdfFile.toAbsolutePath() + " for visualisation: " + e, e);
      }
   }
}
