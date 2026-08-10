package us.ihmc.alexMocap.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.core.ClusterLayout;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Where the markers sit on the links.
 *
 * <h2>Why this matters beyond appearances</h2>
 * <p>
 * Without a standoff every marker lands within a few centimetres of its link's centre of mass --
 * that is, buried inside the link. The calibration does not care: {@code ^i p_ij} is solved for, so
 * a cluster inside the thigh fits exactly as well as one bolted to it. Two things do.
 * </p>
 * <ul>
 * <li>The mesh is drawn over them, so the markers are invisible, which makes a mocap demonstration
 * in which you cannot see the mocap.</li>
 * <li>It is not where markers go. The lever arm from a link's frame to its markers is what converts
 * cluster orientation error into position error, and a cluster at {@code ^i c_i} has a shorter one
 * than any real bracket.</li>
 * </ul>
 */
public class MarkerStandoffTest
{
   /** The demonstration's values. */
   private static final double GAUGE_STANDOFF = 0.18;
   private static final double LIMB_STANDOFF = 0.12;

   /** Alex's thighs and shins are roughly this in radius; a marker inside it would be occluded. */
   private static final double LIMB_RADIUS = 0.08;

   private static double distanceFromLinkCentreOfMass(RobotModelHandle model, ClusterLayout layout, int marker)
   {
      Point3D centreOfMass = new Point3D();
      model.packCenterOfMassInLinkFrame(layout.getLinkName(), centreOfMass);
      return centreOfMass.distance(new Point3D(layout.getPositionInLinkFrame(marker)));
   }

   /** <b>The property.</b> Every marker sits outside the segment it is attached to. */
   @Test
   public void testStandoffPutsEveryMarkerOutsideItsLink() throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(new RobotCaptures.Options().captures(4)
                                                                                         .noise(0.0)
                                                                                         .standoff(GAUGE_STANDOFF, LIMB_STANDOFF));
      RobotModelHandle model = planted.model;

      for (ClusterLayout layout : planted.layouts)
      {
         for (int j = 0; j < layout.getMarkerCount(); j++)
         {
            double distance = distanceFromLinkCentreOfMass(model, layout, j);

            assertTrue(distance > LIMB_RADIUS,
                       layout.getLinkName() + " marker " + j + " is " + distance + " m from the link centre of mass, inside a "
                             + LIMB_RADIUS + " m segment -- it would be drawn inside the mesh.");
         }
      }
   }

   /** And without it they are inside, which is what makes the test above worth having. */
   @Test
   public void testWithoutStandoffMarkersAreBuriedInTheLink() throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(new RobotCaptures.Options().captures(4).noise(0.0));
      RobotModelHandle model = planted.model;

      double closest = Double.POSITIVE_INFINITY;

      for (ClusterLayout layout : planted.layouts)
      {
         for (int j = 0; j < layout.getMarkerCount(); j++)
            closest = Math.min(closest, distanceFromLinkCentreOfMass(model, layout, j));
      }

      assertTrue(closest < LIMB_RADIUS,
                 "Closest marker was " + closest + " m from its link centre of mass without a standoff. If that is now outside "
                       + LIMB_RADIUS + " m the default changed and standoff() may no longer be needed.");
   }

   /**
    * The standoff is lateral, not along the limb.
    * <p>
    * An offset along a segment's long axis would put the thigh's markers somewhere near the knee,
    * which calibrates fine and is wrong. The generator offsets in the link frame's x-y plane, so the
    * z displacement stays whatever the small fixed centroid offset and the spread give it.
    * </p>
    */
   @Test
   public void testStandoffIsLateralRatherThanAlongTheLimb() throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(new RobotCaptures.Options().captures(4)
                                                                                         .noise(0.0)
                                                                                         .standoff(GAUGE_STANDOFF, LIMB_STANDOFF));
      RobotModelHandle model = planted.model;
      Point3D centreOfMass = new Point3D();

      for (ClusterLayout layout : planted.layouts)
      {
         model.packCenterOfMassInLinkFrame(layout.getLinkName(), centreOfMass);

         for (int j = 0; j < layout.getMarkerCount(); j++)
         {
            Point3D marker = new Point3D(layout.getPositionInLinkFrame(j));
            double lateral = Math.hypot(marker.getX() - centreOfMass.getX(), marker.getY() - centreOfMass.getY());
            double axial = Math.abs(marker.getZ() - centreOfMass.getZ());

            assertTrue(lateral > axial,
                       layout.getLinkName() + " marker " + j + " is displaced " + axial + " m along the limb against " + lateral
                             + " m across it; the standoff went the wrong way.");
         }
      }
   }

   /** All four markers of a cluster share one face, rather than ringing the segment. */
   @Test
   public void testAClustersMarkersShareOneFace() throws Exception
   {
      RobotCaptures.Planted planted = RobotCaptures.generate(new RobotCaptures.Options().captures(4)
                                                                                         .noise(0.0)
                                                                                         .standoff(GAUGE_STANDOFF, LIMB_STANDOFF));
      RobotModelHandle model = planted.model;
      Point3D centreOfMass = new Point3D();

      for (ClusterLayout layout : planted.layouts)
      {
         model.packCenterOfMassInLinkFrame(layout.getLinkName(), centreOfMass);

         double minimumAzimuth = Double.POSITIVE_INFINITY;
         double maximumAzimuth = Double.NEGATIVE_INFINITY;

         for (int j = 0; j < layout.getMarkerCount(); j++)
         {
            Point3D marker = new Point3D(layout.getPositionInLinkFrame(j));
            double azimuth = Math.atan2(marker.getY() - centreOfMass.getY(), marker.getX() - centreOfMass.getX());
            minimumAzimuth = Math.min(minimumAzimuth, azimuth);
            maximumAzimuth = Math.max(maximumAzimuth, azimuth);
         }

         // A bracket spans a modest arc. Scattered markers would span most of a circle.
         assertTrue(maximumAzimuth - minimumAzimuth < Math.PI / 2.0,
                    layout.getLinkName() + " markers span " + Math.toDegrees(maximumAzimuth - minimumAzimuth)
                          + " deg of azimuth; they are ringing the segment rather than sharing a face.");
      }
   }

   /** Turning the standoff off must not disturb the random stream, or every seeded dataset moves. */
   @Test
   public void testStandoffOffLeavesTheRandomStreamUntouched() throws Exception
   {
      RobotCaptures.Planted before = RobotCaptures.generate(new RobotCaptures.Options().captures(4).noise(0.0));
      RobotCaptures.Planted after = RobotCaptures.generate(new RobotCaptures.Options().captures(4).noise(0.0).standoff(Double.NaN, Double.NaN));

      for (int c = 0; c < before.layouts.size(); c++)
      {
         ClusterLayout a = before.layouts.get(c);
         ClusterLayout b = after.layouts.get(c);

         for (int j = 0; j < a.getMarkerCount(); j++)
         {
            assertEquals(a.getPositionInLinkFrame(j).getX(),
                         b.getPositionInLinkFrame(j).getX(),
                         0.0,
                         "A NaN standoff drew from the random stream; every fixed-seed dataset in the project would shift.");
         }
      }
   }
}
