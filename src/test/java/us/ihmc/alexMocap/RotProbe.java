package us.ihmc.alexMocap;

import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.calibration.RobotCaptures;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;

/** THROWAWAY. Why does LEFT_SHOULDER_Y_LINK report identity rotation despite rpy=0.698132? */
public class RotProbe
{
   private static final Path OUT = Path.of("/tmp/claude-1001/-home-llibshutz-alex-alex-mocap/1560b6e6-f8da-427c-9347-54446624fdf3/scratchpad/rot.txt");

   @Test
   public void probe() throws Exception
   {
      Files.createDirectories(OUT.getParent());

      try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(OUT)))
      {
         RobotModelHandle model = RobotCaptures.alexModel();
         model.setQ(new double[model.getJointCount()]);
         model.updateFrames();

         RigidBodyTransform t = new RigidBodyTransform();

         for (String link : new String[] {"TORSO_LINK", "LEFT_SHOULDER_Y_LINK", "LEFT_SHOULDER_X_LINK", "LEFT_THIGH"})
         {
            model.packLinkToBase(link, t);
            out.println(link + " ^bT =\n" + t);
            out.println("  parentJoint " + model.getLink(link).getParentJoint().getName() + " class "
                  + model.getLink(link).getParentJoint().getClass().getSimpleName());
            out.println("  frameAfterJoint  " + model.getLink(link).getParentJoint().getFrameAfterJoint().getName());
            out.println("  frameBeforeJoint " + model.getLink(link).getParentJoint().getFrameBeforeJoint().getName());
            out.println("  transformToParent of frameBeforeJoint:\n" + model.getLink(link).getParentJoint().getFrameBeforeJoint().getTransformToParent());
            out.println();
         }

         // Now set LEFT_SHOULDER_Y to a non-zero angle and see whether the axis is x or y in base.
         model.setQ(model.indexOfJoint("LEFT_SHOULDER_Y"), 0.3);
         model.updateFrames();
         model.packLinkToBase("LEFT_SHOULDER_Y_LINK", t);
         out.println("LEFT_SHOULDER_Y = 0.3, ^bT_LEFT_SHOULDER_Y_LINK =\n" + t);
      }
   }
}
