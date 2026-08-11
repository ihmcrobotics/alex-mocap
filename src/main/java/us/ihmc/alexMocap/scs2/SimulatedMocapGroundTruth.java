package us.ihmc.alexMocap.scs2;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.CalibrationResult;
import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.core.MocapFrame;
import us.ihmc.alexMocap.frames.GravityAlignedWorldFrame;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.alexMocap.runtime.CenterOfMassGroundTruth;
import us.ihmc.alexMocap.runtime.KinematicChainCoupler;
import us.ihmc.alexMocap.runtime.LinkPoseEstimator;
import us.ihmc.alexMocap.runtime.MeasuredLinkPoses;
import us.ihmc.alexMocap.sim.MarkerConstellation;
import us.ihmc.alexMocap.sim.MarkerConstellation.MarkerPlacement;
import us.ihmc.alexMocap.sim.SimulatedMocapCamera;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The whole mocap chain as one tickable object, for running inside a simulation.
 *
 * <p>
 * One {@link #update} is: photograph the robot ({@code sim}), recover the marked links' poses (F6),
 * chain the unmarked ones through the encoders (F7), sum the centre of mass (F9), and compare it
 * against the simulation's own. Everything is published as YoVariables and 3-D graphics.
 * </p>
 *
 * <h2>This owns its model, and the caller must not share one</h2>
 * <p>
 * {@link KinematicChainCoupler#complete} sets the model's configuration from the encoder sample, so
 * the {@link RobotModelHandle} handed in here is <b>mutated every tick</b>. Handing in the same
 * handle a simulation is posing from -- or that a second pipeline holds -- makes the two fight over
 * one frame tree, and the symptom is a CoM that is subtly wrong in a way that depends on call order.
 * {@code RobotModelHandle}'s own constructor says the same thing; it is repeated here because this
 * is the class most likely to be wired up next to a live robot model.
 * </p>
 *
 * <h2>{@link #demonstration} does not calibrate</h2>
 * <p>
 * The demonstration factory builds the runtime on the constellation's <b>planted</b> layout: the
 * markers are exactly where the camera thinks they are. That is a legitimate and useful thing to
 * show -- it isolates F6-F9 from calibration error, so a CoM error seen this way is the runtime's
 * and nothing else's -- but it is emphatically <b>not</b> evidence that a calibration would recover
 * that layout. Two numbers to keep apart:
 * </p>
 * <ul>
 * <li>with the planted layout, CoM error is marker noise propagated through F6-F9;</li>
 * <li>with a calibrated layout it also carries the layout error, measured at <b>2.86 mm</b> held-out
 * RMS on Alex at FRAMEWORK.md §1's 140 mm gauge bracket and §17's 0.3 mm noise -- against a 2.2 mm
 * TALOS bar, on synthetic data with a perfect URDF.</li>
 * </ul>
 * <p>
 * Reporting the first as though it were the second is the single easiest way to overstate what this
 * project can do. Use {@link #isUsingPlantedLayout()} in any summary that leaves the machine.
 * </p>
 *
 * <h2>What the marked set leaves out</h2>
 * <p>
 * With the legs-and-pelvis set, <b>58.45 % of Alex's mass is posed by forward kinematics rather than
 * by markers</b> -- {@code TORSO_LINK} alone is 24 % of it, chained through a single {@code SPINE_Z}
 * joint that is itself one of the estimator's states. In simulation the encoders are perfect, so
 * that mass costs nothing and the CoM error understates what a real stage would give. It is the
 * highest-leverage thing to fix, and one torso cluster takes the FK-posed fraction to 34.18 %.
 * </p>
 */
public class SimulatedMocapGroundTruth
{
   private final YoRegistry registry;

   private final RobotModelHandle model;
   private final SimulatedMocapCamera camera;
   private final LinkPoseEstimator estimator;
   private final KinematicChainCoupler coupler;
   private final CenterOfMassGroundTruth centerOfMassGroundTruth;
   private final boolean usingPlantedLayout;

   private final MocapMarkerYoVariables markerVariables;
   private final GroundTruthComparisonYoVariables comparison;

   private final MocapFrame frame;
   private final MeasuredLinkPoses poses;
   private final EncoderSample encoders;
   private final Point3D centerOfMass = new Point3D();

   private final List<String> markedLinks;
   private final YoBoolean[] clusterAccepted;
   private final YoDouble[] clusterSigma3;
   private final YoInteger refusedClusterCount;
   private final YoDouble missingMass;

   /**
    * @param namePrefix     prefix for every variable, so two instances can coexist.
    * @param model          <b>owned and mutated.</b> See the class javadoc.
    * @param calibration    the layout the runtime believes. Must cover every marked link.
    * @param constellation  the marker set the camera observes.
    * @param camera         the sensor simulator.
    * @param world          supplies the motive and gravity-aligned frames the CoM is expressed in.
    * @param plantedLayout  whether {@code calibration} is the constellation's own truth rather than
    *                       a solved one. Recorded, not used -- see {@link #isUsingPlantedLayout()}.
    */
   public SimulatedMocapGroundTruth(String namePrefix,
                                    RobotModelHandle model,
                                    CalibrationResult calibration,
                                    MarkerConstellation constellation,
                                    SimulatedMocapCamera camera,
                                    GravityAlignedWorldFrame world,
                                    boolean plantedLayout)
   {
      this.registry = new YoRegistry(namePrefix + "SimulatedMocap");
      this.model = model;
      this.camera = camera;
      this.usingPlantedLayout = plantedLayout;
      this.markedLinks = constellation.getMarkedLinks();

      List<String> linkNames = model.getLinkNames();

      this.estimator = new LinkPoseEstimator(calibration, constellation.getClusters(), linkNames);
      this.coupler = new KinematicChainCoupler(model, linkNames, markedLinks);
      this.centerOfMassGroundTruth = CenterOfMassGroundTruth.forWholeRobot(model, world.getMotiveWorld(), world.getGravityAlignedWorld());

      this.frame = camera.newFrame();
      this.poses = new MeasuredLinkPoses(linkNames);
      this.encoders = new EncoderSample(model.getJointNames());

      // The cloud is in the motive world -- markers are raw observations, and F8's tilt correction
      // is applied downstream of them by CenterOfMassGroundTruth. Publishing them in the
      // gravity-aligned frame would draw a cloud that does not sit on the robot.
      this.markerVariables = new MocapMarkerYoVariables(namePrefix, constellation.getClusters(), constellation.getMarkers(), world.getMotiveWorld());
      this.comparison = new GroundTruthComparisonYoVariables(namePrefix, world.getGravityAlignedWorld());

      this.clusterAccepted = new YoBoolean[markedLinks.size()];
      this.clusterSigma3 = new YoDouble[markedLinks.size()];
      this.refusedClusterCount = new YoInteger(namePrefix + "RefusedClusterCount", registry);
      this.missingMass = new YoDouble(namePrefix + "MissingMassKilograms", registry);

      for (int i = 0; i < markedLinks.size(); i++)
      {
         String link = markedLinks.get(i);
         clusterAccepted[i] = new YoBoolean(namePrefix + link + "PoseAccepted", registry);
         // σ₃ is in m², a mean-squared spread: a 140 mm cluster reads about 0.003. The name carries
         // the unit because a reader who assumes metres is out by three orders of magnitude.
         clusterSigma3[i] = new YoDouble(namePrefix + link + "Sigma3SquaredMetres", registry);
      }

      registry.addChild(markerVariables.getRegistry());
      registry.addChild(comparison.getRegistry());
   }

   /**
    * The runtime on the constellation's planted layout: no calibration, so no calibration error.
    * <p>
    * Read {@link #demonstration} in the class javadoc before quoting a number obtained this way.
    * </p>
    *
    * @param model  <b>owned and mutated.</b>
    * @param seed   fixed seed for both the layout draw and the camera noise.
    */
   public static SimulatedMocapGroundTruth demonstration(String namePrefix,
                                                         RobotModelHandle model,
                                                         List<String> markedLinks,
                                                         double markerNoiseStandardDeviation,
                                                         double occlusionProbability,
                                                         long seed,
                                                         GravityAlignedWorldFrame world)
   {
      return demonstration(namePrefix, model, markedLinks, markerNoiseStandardDeviation, occlusionProbability, seed, world, MarkerPlacement.BRACKET);
   }

   /**
    * As {@link #demonstration}, with the marker placement chosen.
    *
    * @param placement {@link MarkerPlacement#SCATTERED} is what {@code AlexLegDemo} uses -- markers
    *                  spread over each segment rather than patched onto one face of it.
    */
   public static SimulatedMocapGroundTruth demonstration(String namePrefix,
                                                         RobotModelHandle model,
                                                         List<String> markedLinks,
                                                         double markerNoiseStandardDeviation,
                                                         double occlusionProbability,
                                                         long seed,
                                                         GravityAlignedWorldFrame world,
                                                         MarkerPlacement placement)
   {
      MarkerConstellation constellation = MarkerConstellation.random(model,
                                                                     markedLinks,
                                                                     seed,
                                                                     MarkerConstellation.DEFAULT_MARKERS_PER_CLUSTER,
                                                                     MarkerConstellation.DEFAULT_GAUGE_SPREAD,
                                                                     MarkerConstellation.DEFAULT_LIMB_SPREAD,
                                                                     MarkerConstellation.DEFAULT_GAUGE_STANDOFF,
                                                                     MarkerConstellation.DEFAULT_LIMB_STANDOFF,
                                                                     placement);
      SimulatedMocapCamera camera = new SimulatedMocapCamera(constellation, markerNoiseStandardDeviation, occlusionProbability, seed);

      CalibrationResult planted = new CalibrationResult();

      for (ClusterLayout layout : constellation.getTrueLayouts())
         planted.addLayout(layout);

      return new SimulatedMocapGroundTruth(namePrefix, model, planted, constellation, camera, world, true);
   }

   /**
    * One tick.
    *
    * @param linkPoses          where the simulated links actually are, {@code ^W T_i}.
    * @param jointAngles        the encoder readings, in {@link RobotModelHandle}'s joint order. Used
    *                           to chain the unmarked links.
    * @param actualCenterOfMass the simulation's own centre of mass, in the gravity-aligned world
    *                           frame -- the same frame the mocap CoM is computed in. Passing one in
    *                           the raw world frame reports the floor tilt as CoM error.
    * @return whether the mocap chain produced a centre of mass this tick.
    */
   public boolean update(SimulatedMocapCamera.LinkPoseSource linkPoses, double[] jointAngles, Point3DReadOnly actualCenterOfMass)
   {
      if (jointAngles.length != encoders.getJointCount())
         throw new IllegalArgumentException("Expected " + encoders.getJointCount() + " joint angles, got " + jointAngles.length + ".");

      camera.observe(linkPoses, frame);
      markerVariables.update(frame);

      estimator.estimate(frame, poses);

      encoders.setQ(jointAngles);
      coupler.complete(encoders, poses);

      boolean valid = centerOfMassGroundTruth.compute(poses, centerOfMass);

      int refused = 0;

      for (int i = 0; i < markedLinks.size(); i++)
      {
         int index = poses.indexOf(markedLinks.get(i));
         boolean accepted = poses.getSource(index) == MeasuredLinkPoses.Source.MEASURED;

         clusterAccepted[i].set(accepted);
         clusterSigma3[i].set(poses.getSigma3(index));

         if (!accepted)
            refused++;
      }

      refusedClusterCount.set(refused);
      missingMass.set(centerOfMassGroundTruth.getMissingMass());

      comparison.update(centerOfMass, actualCenterOfMass);

      return valid;
   }

   /** Every graphic this pipeline draws: the marker cloud, and the two centres of mass. */
   public YoGraphicDefinition createYoGraphics(String name)
   {
      List<YoGraphicDefinition> children = new ArrayList<>();
      children.add(markerVariables.createYoGraphics(name + "Markers"));
      children.add(comparison.createYoGraphics(name + "Com"));

      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.setChildren(children);

      return group;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   /** The mocap chain's centre of mass from the last tick, or NaN if it was refused. */
   public Point3DReadOnly getCenterOfMass()
   {
      return centerOfMass;
   }

   public GroundTruthComparisonYoVariables getComparison()
   {
      return comparison;
   }

   public MocapMarkerYoVariables getMarkerVariables()
   {
      return markerVariables;
   }

   /** The model this pipeline mutates. Do not pose it from anywhere else. */
   public RobotModelHandle getModel()
   {
      return model;
   }

   /**
    * Where the last {@link #update} put each link: {@code ^W T_i}, measured for the marked links and
    * chained through the encoders for the rest.
    * <p>
    * This is the reconstruction itself rather than a summary of it, which is what a caller drawing a
    * ghost robot at the measured pose needs. Live and overwritten every tick -- read it inside the
    * tick, do not retain it. Check {@link MeasuredLinkPoses#getSource} before trusting a pose: a
    * refused cluster leaves NaN, and NaN is the honest value, not a defect to be defaulted away.
    * </p>
    */
   public MeasuredLinkPoses getLinkPoses()
   {
      return poses;
   }

   /**
    * True when the runtime is running on the constellation's planted layout rather than a solved
    * one.
    * <p>
    * Any summary that leaves the machine should say so. See the class javadoc for the two numbers
    * this distinguishes.
    * </p>
    */
   public boolean isUsingPlantedLayout()
   {
      return usingPlantedLayout;
   }

   /** One line for a console summary, carrying the caveat with it. */
   public String summary()
   {
      return comparison.summary() + (usingPlantedLayout ? "  [planted layout: excludes calibration error]" : "  [calibrated layout]");
   }
}
