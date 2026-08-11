package us.ihmc.alex.simulation;

import java.util.List;

import us.ihmc.alex.robotModel.AlexRobotModel;
import us.ihmc.alex.robotModel.AlexVersion;
import us.ihmc.alexMocap.frames.GravityAlignedWorldFrame;
import us.ihmc.alexMocap.frames.TiltMeasurement;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.scs2.SimulatedMocapGroundTruth;
import us.ihmc.alexMocap.sim.SimulatedMocapCamera;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

/**
 * Alex walking, with a simulated OptiTrack marker set on the legs, and the mocap-derived centre of
 * mass drawn against the simulation's real one.
 *
 * <p>
 * Built exactly like {@link AlexFlatGroundWalkingTrack} -- same {@code AlexRobotModel}, same
 * {@code SCS2AvatarSimulationFactory}, same walking controller -- with one extra {@link Controller}
 * attached to the simulated robot. Nothing about the simulation is changed by watching it.
 * </p>
 *
 * <h2>What is on screen</h2>
 * <ul>
 * <li><b>28 marker spheres</b>, four per link, coloured per cluster: pelvis, both thighs, both
 * shins, both feet. Yellow is the pelvis gauge. A marker that is not seen disappears.</li>
 * <li><b>A gold sphere</b>: the centre of mass the mocap chain computes.</li>
 * <li><b>A green sphere</b>: the centre of mass the simulation actually has.</li>
 * </ul>
 * <p>
 * At a millimetre of error the two spheres are on top of each other, which is the point -- they
 * separate visibly exactly when something has gone wrong. The number is
 * {@code mocapMinusActualComMagnitude}, and {@code mocapMinusActualComMean} next to it is what
 * distinguishes a bias from jitter.
 * </p>
 *
 * <h2>The marker set is randomised, and the layout is planted</h2>
 * <p>
 * Marker positions are drawn once from {@link #MARKER_SEED} and are then fixed for the run. The
 * runtime is given that planted layout rather than a calibrated one, so <b>this shows F6-F9 without
 * calibration error</b>. It is not evidence that a calibration would recover the layout; on Alex
 * that carries a further 2.86 mm held-out RMS at FRAMEWORK.md §1's 140 mm gauge bracket. The
 * console banner says so, and so does {@code SimulatedMocapGroundTruth.summary()}.
 * </p>
 *
 * <h2>What this does and does not demonstrate</h2>
 * <p>
 * The mocap chain and the simulation share a URDF, so link masses and link-CoM offsets are
 * <b>identical by construction</b>. On the real robot they are not, and F11 measured that as the
 * dominant term: mass 4.90 mm / link-CoM 2.73 mm / mocap 0.164 mm, CAD dominating by 33x. So the
 * error on screen here is the mocap chain's own, with the largest real-world contributor set to
 * zero. Weigh the robot.
 * </p>
 * <p>
 * Encoders are perfect here too, which matters more than it sounds: with a legs-only marker set,
 * 58.45 % of Alex's mass is posed by forward kinematics rather than by markers, and
 * {@code TORSO_LINK} alone is 24 % of it, hanging off a single {@code SPINE_Z} joint. In simulation
 * that mass costs nothing.
 * </p>
 */
public class AlexMocapGroundTruthTrack
{
   static
   {
      System.setProperty("fastdds.intraprocess.delivery", "true");
   }

   /** Legs, pelvis and feet: the set FRAMEWORK.md's demonstration uses. */
   private static final List<String> MARKED_LINKS = List.of("PELVIS_LINK",
                                                             "LEFT_THIGH",
                                                             "RIGHT_THIGH",
                                                             "LEFT_SHIN",
                                                             "RIGHT_SHIN",
                                                             "LEFT_FOOT",
                                                             "RIGHT_FOOT");

   /** Fixed, so a run is reproducible and two runs are comparable. */
   private static final long MARKER_SEED = 20260810L;

   /** FRAMEWORK.md §17's per-axis marker noise for a tight volume at the gantry. */
   private static final double MARKER_NOISE = SimulatedMocapCamera.GANTRY_NOISE_STANDARD_DEVIATION;

   /**
    * Off by default. Occlusion is the honest thing to turn on when asking "will this survive a real
    * stage", and the wrong thing to leave on when asking "is the chain correct" -- at 12 % it refuses
    * 63 % of frames, and a mostly-NaN trace is hard to read.
    */
   private static final double OCCLUSION_PROBABILITY = Double.parseDouble(System.getProperty("mocap.occlusion", "0.0"));

   private static final boolean createYoVariableServer = Boolean.parseBoolean(System.getProperty("create.yovariable.server", "true"));

   private final AsyncROS2Node asyncROS2Node = new AsyncROS2Node("alex_mocap_ground_truth_track");

   public AlexMocapGroundTruthTrack()
   {
      AlexRobotModel robotModel = new AlexRobotModel(AlexVersion.getPhysicalRealityVersion(), RobotTarget.SCS);

      int recordFrequency = (int) Math.max(1.0, Math.round(robotModel.getFastestControllerDT() / robotModel.getSimulateDT()));
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setAsyncROS2Node(asyncROS2Node);
      avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(true, new HeadingAndVelocityEvaluationScriptParameters());
      avatarSimulationFactory.setCommonAvatarEnvrionmentInterface(new FlatGroundEnvironment());
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSimulationDataRecordTickPeriod(recordFrequency);
      avatarSimulationFactory.setCreateYoVariableServer(createYoVariableServer);
      avatarSimulationFactory.setInitializeEstimatorToActual(true);
      avatarSimulationFactory.setUseImpulseBasedPhysicsEngine(false);
      avatarSimulationFactory.setUseBulletPhysicsEngine(false);
      avatarSimulationFactory.setUsePerfectSensors(false);

      SCS2AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      // A SECOND instance of the model, deliberately. SimulatedMocapGroundTruth poses its model from
      // the encoders every tick to chain the unmarked links; sharing the simulation's tree would
      // have the measurement overwriting what it is measuring, and the result would be correct
      // exactly when the two configurations happened to agree.
      RigidBodyBasics mocapTree = robotModel.getRobotDefinition().newInstance(ReferenceFrame.getWorldFrame());
      RobotModelHandle mocapModel = new RobotModelHandle(mocapTree);

      // The floor is level in simulation, so there is no tilt to correct. On a real stage this is
      // F8's measured plate reading, and getting it wrong biased every CoM 7 mm low once already.
      GravityAlignedWorldFrame world = new GravityAlignedWorldFrame(TiltMeasurement.assumedLevel("simulation floor is level by construction"),
                                                                     ReferenceFrame.getWorldFrame(),
                                                                     "_mocap");

      SimulatedMocapGroundTruth mocap = SimulatedMocapGroundTruth.demonstration("mocap",
                                                                                 mocapModel,
                                                                                 MARKED_LINKS,
                                                                                 MARKER_NOISE,
                                                                                 OCCLUSION_PROBABILITY,
                                                                                 MARKER_SEED,
                                                                                 world);

      Robot robot = avatarSimulation.getRobot();
      robot.getControllerManager().addController(new MocapController(robot, mocapModel, mocap, world));

      SimulationConstructionSet2 scs = avatarSimulation.getSimulationConstructionSet();
      scs.addRegistry(mocap.getRegistry());
      scs.addYoGraphic(mocap.createYoGraphics("mocap"));

      printBanner(mocap);

      avatarSimulation.start();
   }

   private static void printBanner(SimulatedMocapGroundTruth mocap)
   {
      System.out.println("=".repeat(78));
      System.out.println("Alex mocap ground truth  --  simulated OptiTrack on legs + pelvis + feet");
      System.out.println("=".repeat(78));
      System.out.println("marked      " + String.join(", ", MARKED_LINKS));
      System.out.println("markers     " + MARKED_LINKS.size() * 4 + ", seed " + MARKER_SEED);
      System.out.println("noise       " + 1000.0 * MARKER_NOISE + " mm per axis" + (OCCLUSION_PROBABILITY > 0.0 ? ", occlusion "
            + (100.0 * OCCLUSION_PROBABILITY) + " %" : ", no occlusion"));
      System.out.println("layout      " + (mocap.isUsingPlantedLayout() ? "PLANTED -- excludes calibration error" : "calibrated"));
      System.out.println();
      System.out.println("Watch:  mocapMocapMinusActualComMagnitude   per-frame |mocap - actual|");
      System.out.println("        mocapMocapMinusActualComMean        bias if it does not go to zero");
      System.out.println("        mocapVisibleMarkerCount             28 unless something is occluded");
      System.out.println("        mocapRefusedClusterCount            clusters with no usable pose");
      System.out.println("=".repeat(78));
   }

   /**
    * Reads the simulated robot every tick and drives the mocap chain from it.
    * <p>
    * The link poses come from the simulation's own frames, so the "markers" are attached to the
    * robot that is actually walking. The joint angles are read from the same tree, which is the
    * perfect-encoder assumption -- see the class javadoc for what that hides.
    * </p>
    */
   private static class MocapController implements Controller
   {
      private final SimulatedMocapGroundTruth mocap;
      private final CenterOfMassCalculator actualCenterOfMass;
      private final FramePoint3D actualCenterOfMassInWorld = new FramePoint3D();
      private final ReferenceFrame gravityAlignedWorld;
      private final OneDoFJointBasics[] simulationJoints;
      private final Robot robot;
      private final double[] jointAngles;

      MocapController(Robot robot, RobotModelHandle mocapModel, SimulatedMocapGroundTruth mocap, GravityAlignedWorldFrame world)
      {
         this.robot = robot;
         this.mocap = mocap;
         this.gravityAlignedWorld = world.getGravityAlignedWorld();
         this.actualCenterOfMass = new CenterOfMassCalculator(robot.getRootBody(), ReferenceFrame.getWorldFrame());

         // Resolved once, in the mocap model's joint order, so update() is a straight array fill.
         // A name the simulation does not have is a wiring fault and should stop the run here rather
         // than silently leave a joint at zero.
         List<String> jointNames = mocapModel.getJointNames();
         this.simulationJoints = new OneDoFJointBasics[jointNames.size()];
         this.jointAngles = new double[jointNames.size()];

         for (int j = 0; j < jointNames.size(); j++)
         {
            String name = jointNames.get(j);

            if (!(robot.getJoint(name) instanceof OneDoFJointBasics joint))
               throw new IllegalStateException("The simulated robot has no one-DoF joint named '" + name
                     + "', which the mocap model expects. The two models have diverged.");

            simulationJoints[j] = joint;
         }
      }

      @Override
      public void doControl()
      {
         for (int j = 0; j < simulationJoints.length; j++)
            jointAngles[j] = simulationJoints[j].getQ();

         actualCenterOfMass.reset();
         actualCenterOfMassInWorld.setIncludingFrame(ReferenceFrame.getWorldFrame(), actualCenterOfMass.getCenterOfMass());
         actualCenterOfMassInWorld.changeFrame(gravityAlignedWorld);

         mocap.update(this::packLinkPose, jointAngles, actualCenterOfMassInWorld);
      }

      /** {@code ^W T_i} straight off the simulated robot's own link frame. */
      private boolean packLinkPose(String linkName, us.ihmc.euclid.transform.RigidBodyTransform poseToPack)
      {
         RigidBodyBasics body = robot.getRigidBody(linkName);

         if (body == null || body.getParentJoint() == null)
            return false;

         // The link frame, not the body-fixed (centre-of-mass) frame. Mecano's body-fixed frame is
         // the link CoM frame -- it appears in the tree as `l_thighCoM` -- and using it here would
         // silently shift every marker by the link's CoM offset.
         poseToPack.set(body.getParentJoint().getFrameAfterJoint().getTransformToRoot());
         return true;
      }

      @Override
      public us.ihmc.yoVariables.registry.YoRegistry getYoRegistry()
      {
         // Registered on the SCS root instead, so the variables sit beside the other ground-truth
         // telemetry rather than under the robot's controller tree.
         return null;
      }
   }

   public static void main(String[] args)
   {
      new AlexMocapGroundTruthTrack();
   }
}
