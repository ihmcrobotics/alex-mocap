package us.ihmc.alexMocap.runtime;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * F7, FK chaining for unmarked links (FRAMEWORK.md §10).
 *
 * <pre>
 * ^W T̂_i  =  ^W T̂_p · ^p T_i(q)
 * </pre>
 *
 * <p>
 * For a link with no cluster, walk to the nearest marked ancestor {@code p} and chain through the
 * joints between.
 * </p>
 *
 * <h2>A deliberate trade, and it should feel like one</h2>
 * <p>
 * This <b>reintroduces encoder dependence</b> for the chained link -- the exact dependence F6
 * exists to remove. §10 accepts it on a specific bargain: mark links in descending order of
 * {@code mass × lever arm}, and chain only the light, close-in ones whose CoM contribution is
 * small. The bargain fails silently if someone leaves a heavy link unmarked, because a chained pose
 * looks identical to a measured one once it is a transform.
 * </p>
 * <p>
 * That is why {@link MeasuredLinkPoses} records {@link MeasuredLinkPoses.Source#CHAINED} and why
 * this class reports {@link #getChainedMass} -- so "how much of the CoM rests on encoders" is a
 * number someone can look at rather than an assumption.
 * </p>
 *
 * <h2>The ancestor is resolved once, not per frame</h2>
 * <p>
 * Which link chains from which is fixed by the marker mounting, not by the data, so the walk up the
 * tree happens at construction. Per frame this is one transform multiply per unmarked link.
 * </p>
 * <p>
 * Note the consequence of an <b>ancestor that was refused</b>: F6 declining a pose for a marked
 * link takes every link chained from it with it. That is correct -- there is nothing to chain from
 * -- and it means a single near-collinear cluster can remove several links from the CoM sum. The
 * count is reported per frame rather than being absorbed.
 * </p>
 */
public class KinematicChainCoupler
{
   private final RobotModelHandle model;

   /** One entry per unmarked link, resolved at construction. */
   private final int[] chainedLinkIndices;
   private final String[] chainedLinkNames;
   private final int[] ancestorIndices;
   private final String[] ancestorNames;

   private final RigidBodyTransform linkToAncestor = new RigidBodyTransform();
   private final RigidBodyTransform linkToWorld = new RigidBodyTransform();

   /**
    * @param model      the FK reference.
    * @param linkNames  every link reported by the {@link MeasuredLinkPoses} this will fill.
    * @param markedLinks the links that carry clusters, and so are filled by F6 instead.
    */
   public KinematicChainCoupler(RobotModelHandle model, List<String> linkNames, List<String> markedLinks)
   {
      this.model = model;

      List<Integer> indices = new ArrayList<>();
      List<String> names = new ArrayList<>();
      List<Integer> ancestors = new ArrayList<>();
      List<String> ancestorNameList = new ArrayList<>();

      for (int i = 0; i < linkNames.size(); i++)
      {
         String linkName = linkNames.get(i);

         if (markedLinks.contains(linkName))
            continue;

         String ancestor = nearestMarkedAncestor(linkName, markedLinks);

         if (ancestor == null)
            throw new IllegalArgumentException("Link '" + linkName + "' has no marked ancestor, so its pose cannot be established at all. "
                  + "Mark it, mark something above it, or drop it from the reported links.");

         indices.add(i);
         names.add(linkName);
         ancestors.add(linkNames.indexOf(ancestor));
         ancestorNameList.add(ancestor);
      }

      this.chainedLinkIndices = indices.stream().mapToInt(Integer::intValue).toArray();
      this.chainedLinkNames = names.toArray(new String[0]);
      this.ancestorIndices = ancestors.stream().mapToInt(Integer::intValue).toArray();
      this.ancestorNames = ancestorNameList.toArray(new String[0]);
   }

   private String nearestMarkedAncestor(String linkName, List<String> markedLinks)
   {
      String current = model.getParentLinkName(linkName);

      while (current != null)
      {
         if (markedLinks.contains(current))
            return current;

         current = model.getParentLinkName(current);
      }

      return null;
   }

   /**
    * Fills every unmarked link from its marked ancestor.
    * <p>
    * The model is set to {@code q} first, so the caller does not have to remember to. An unmarked
    * link whose ancestor was refused stays {@link MeasuredLinkPoses.Source#NONE}, with a reason
    * naming the ancestor.
    * </p>
    *
    * @return how many links were chained. Links whose ancestor had no pose are not counted.
    */
   public int complete(EncoderSample encoderSample, MeasuredLinkPoses poses)
   {
      model.setConfiguration(encoderSample);
      int chained = 0;

      for (int c = 0; c < chainedLinkIndices.length; c++)
      {
         int linkIndex = chainedLinkIndices[c];
         int ancestorIndex = ancestorIndices[c];

         if (!poses.isAvailable(ancestorIndex))
         {
            poses.setRefused(linkIndex, Double.NaN, 0, "chained from '" + ancestorNames[c] + "', which has no pose this frame");
            continue;
         }

         model.packLinkToLink(chainedLinkNames[c], ancestorNames[c], linkToAncestor);

         linkToWorld.set(poses.getPose(ancestorIndex));
         linkToWorld.multiply(linkToAncestor);

         poses.setChained(linkIndex, linkToWorld);
         chained++;
      }

      return chained;
   }

   public int getChainedLinkCount()
   {
      return chainedLinkIndices.length;
   }

   public String getChainedLinkName(int index)
   {
      return chainedLinkNames[index];
   }

   public String getAncestorName(int index)
   {
      return ancestorNames[index];
   }

   /**
    * Total mass whose pose rests on encoders rather than on markers.
    * <p>
    * The number §10's trade is made against. Compare it to {@link RobotModelHandle#getTotalMass()}:
    * if a large fraction of the robot is chained, the CoM is an FK result wearing a mocap costume,
    * and the fix is a marker cluster rather than anything in software.
    * </p>
    */
   public double getChainedMass()
   {
      double mass = 0.0;

      for (String linkName : chainedLinkNames)
         mass += model.getMass(linkName);

      return mass;
   }

   @Override
   public String toString()
   {
      return "KinematicChainCoupler[" + chainedLinkIndices.length + " chained links, " + String.format("%.2f", getChainedMass()) + " kg on encoders]";
   }
}
