package us.ihmc.alexMocap;

import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/** THROWAWAY. Step 0 of the plan: does the real Alex URDF load at all? */
public class Step0Probe
{
   private static final Path SOURCE = Path.of("/home/llibshutz/alex/invariant-estimation/assets/alex_with_imus.urdf");
   private static final Path OUT = Path.of("/tmp/claude-1001/-home-llibshutz-alex-alex-mocap/1560b6e6-f8da-427c-9347-54446624fdf3/scratchpad/step0.txt");

   @Test
   public void probe() throws Exception
   {
      Files.createDirectories(OUT.getParent());

      try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(OUT)))
      {
         try
         {
            RobotModelHandle model = RobotModelHandle.fromURDF(SOURCE);
            out.println("LOADED");
            out.println("toString      " + model);
            out.println("jointCount    " + model.getJointCount());
            out.println("linkCount     " + model.getLinkNames().size());
            out.println("baseLink      " + model.getBaseLinkName());
            out.printf("totalMass     %.6f%n", model.getTotalMass());

            out.println("joints        " + model.getJointNames());

            model.updateFrames();

            List<String> badMass = new ArrayList<>();
            List<String> badCom = new ArrayList<>();
            List<String> zeroMass = new ArrayList<>();
            Point3D com = new Point3D();

            for (String link : model.getLinkNames())
            {
               double m = model.getMass(link);
               if (!Double.isFinite(m))
                  badMass.add(link + "=" + m);
               if (m == 0.0)
                  zeroMass.add(link);

               try
               {
                  model.packCenterOfMassInLinkFrame(link, com);
                  if (com.containsNaN() || !Double.isFinite(com.norm()))
                     badCom.add(link + "=" + com);
               }
               catch (Exception e)
               {
                  badCom.add(link + " THREW " + e);
               }
            }

            out.println("nonFiniteMass " + badMass);
            out.println("zeroMass      " + zeroMass);
            out.println("nonFiniteCoM  " + badCom);

            out.println();
            out.println("links (name, mass, ^i c_i):");
            for (String link : model.getLinkNames())
            {
               model.packCenterOfMassInLinkFrame(link, com);
               out.printf("  %-28s %10.5f  (%9.5f %9.5f %9.5f)%n", link, model.getMass(link), com.getX(), com.getY(), com.getZ());
            }

            out.println();
            out.println("joint limits:");
            RigidBodyTransform t = new RigidBodyTransform();
            for (String link : model.getLinkNames())
            {
               model.packLinkToBase(link, t);
               out.printf("  ^bT_%-26s %s%n", link, t.getTranslation());
            }

            out.println();
            out.println("parent chain:");
            for (String link : model.getLinkNames())
               out.println("  " + link + " <- " + model.getParentLinkName(link));
         }
         catch (Throwable t)
         {
            out.println("THREW " + t.getClass().getName() + ": " + t.getMessage());
            t.printStackTrace(out);
         }
      }
   }
}
