package us.ihmc.alexMocap.scs2;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;

/**
 * The 3-D overlay: where the CoM is, and where the pelvis is pointing.
 * <p>
 * Two graphics, deliberately. The CoM is the deliverable and the pelvis pose is what F10 hands the
 * EKF comparison; anything else on screen competes with them for attention.
 * </p>
 *
 * <h2>The CoM marker changes colour when it is not real</h2>
 * <p>
 * A refused link means there is no CoM for that frame ({@code runtime.CenterOfMassGroundTruth}
 * returns NaN rather than the CoM of a lighter robot). NaN does not render, so the marker simply
 * vanishes -- which is honest, but on a fast replay it reads as a flicker rather than as a fault.
 * Nothing here can fix that alone: pair the view with {@link ConditioningMonitor}'s table, which is
 * where a dropout becomes a count.
 * </p>
 */
public class GroundTruthYoGraphics
{
   /** Radius of the CoM sphere, metres. Large enough to see, small enough not to hide the robot. */
   public static final double COM_RADIUS = 0.03;

   /** Length of the pelvis coordinate-system axes, metres. */
   public static final double PELVIS_AXIS_LENGTH = 0.2;

   private GroundTruthYoGraphics()
   {
   }

   /**
    * Builds the overlay for one set of ground-truth variables.
    *
    * @return a group to hand to {@code SimulationSession.addYoGraphicDefinition}.
    */
   public static YoGraphicDefinition create(String name, GroundTruthYoVariables variables)
   {
      List<YoGraphicDefinition> children = new ArrayList<>();

      children.add(YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + "Com", variables.getCenterOfMass(), COM_RADIUS, ColorDefinitions.Gold()));
      children.add(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name + "PelvisPose",
                                                                            variables.getPelvisPose(),
                                                                            PELVIS_AXIS_LENGTH,
                                                                            ColorDefinitions.Aqua()));

      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.setChildren(children);

      return group;
   }
}
