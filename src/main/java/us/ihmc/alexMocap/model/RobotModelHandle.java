package us.ihmc.alexMocap.model;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.alexMocap.core.EncoderSample;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * F1 (FRAMEWORK.md §3): the FK reference. Set {@code q}, update frames, read {@code ^b T_i(q)}.
 * <p>
 * This is the first of the two objects FRAMEWORK.md §0 is at pains to distinguish. It answers
 * <b>where the model says the link is</b>, from the URDF and the encoders alone. It contains no
 * mocap and does not know where the robot is in the room. The other object -- where the link
 * <i>actually</i> is -- comes from a marker cluster through {@code RigidBodyRegistration}, and the
 * calibration compares the two. They are never substituted for one another, and keeping them in
 * separate types is how that stays true.
 * </p>
 *
 * <h2>Which frame is "the link frame"</h2>
 * <p>
 * <b>The URDF link frame, taken as {@code parentJoint.getFrameAfterJoint()}.</b> This is a
 * deliberate departure from FRAMEWORK.md §3, which says to read
 * {@code RigidBodyBasics.getBodyFixedFrame()}. That instruction does not do what it appears to:
 * Mecano's body-fixed frame is the link's <b>centre-of-mass frame</b> (it comes out of the tree
 * literally named {@code l_thighCoM}), positioned at the inertia pose, not at the URDF link
 * origin.
 * </p>
 * <p>
 * Either choice is internally consistent for the calibration itself, because {@code ^i p_ij} is
 * unknown and solved for -- a constant offset between the two conventions is absorbed into the
 * layouts and into {@code Δ}, and {@code J} is identical. What decides it is FRAMEWORK.md §12 and
 * §14. §12 computes CoM as {@code sum_i m_i · ^Wg T̂_i · ^i c_i}, and §14's error budget carries a
 * {@code δ(^i c_i)} term for "link-CoM error". Both presuppose that {@code ^i c_i} is a real,
 * generally non-zero vector read from the URDF inertial block. In the CoM frame {@code ^i c_i} is
 * identically zero, that term vanishes, and the second-largest entry in the error budget silently
 * stops existing. So the link frame must be the URDF link frame.
 * </p>
 * <p>
 * The practical consequence: a calibrated {@code ^i p̂_ij} printed by this pipeline is directly
 * comparable to a CAD marker position, which is what makes a layout auditable by a human.
 * </p>
 *
 * <h2>The floating joint SCS2 adds, and why it does not contaminate FK</h2>
 * <p>
 * SCS2 does not instantiate the URDF root link as the tree root. It creates a synthetic body
 * called {@code rootBody} and attaches the URDF root link beneath it through a
 * {@code SixDoFJoint}. That joint is not in the URDF and is not an encoder-backed degree of
 * freedom.
 * </p>
 * <p>
 * This matters because FRAMEWORK.md §0 rests on {@code ^b T_i(q)} being <b>a function of joint
 * angles alone</b> -- it is the entire answer to the "apparent circularity". Had the base frame
 * been taken as the synthetic root's frame, every {@code ^b T_i} would have included the floating
 * joint's transform, and the FK reference would have quietly depended on where someone last put
 * the robot.
 * </p>
 * <p>
 * Defining {@code b} as the frame <i>after</i> the floating joint removes it from every transform
 * this class reports: the floating joint sits above {@code b} and cancels in
 * {@code ^b T_i = (^root T_b)^-1 · ^root T_i}. The base is therefore free to be anywhere without
 * changing a single number here, which is a property worth asserting rather than believing --
 * {@code RobotModelHandleTest} sets the floating joint to a random pose and checks that no
 * {@code ^b T_i} moves.
 * </p>
 *
 * <h2>Status: assumed</h2>
 * <p>
 * There is no algorithm here. Everything this class reports is the URDF's claim about joint
 * offsets and link geometry, trusted to a reasonable degree (FRAMEWORK.md §3). That trust is
 * exactly what {@code BootstrapSpreadGate} (G2) tests, and F4's averaging will not repair it if it
 * is misplaced -- a joint offset or a wrong link length is correlated with configuration and
 * survives the mean as a bias (§6).
 * </p>
 *
 * <h2>Joint order</h2>
 * <p>
 * Joints are indexed in Mecano's subtree order, which is deterministic for a given URDF but is
 * <b>not</b> guaranteed to match the column order of anyone's encoder log. A permuted joint vector
 * produces a completely plausible FK result at the wrong configuration, so
 * {@link #setConfiguration(EncoderSample)} checks names rather than trusting positions. Call it
 * instead of {@link #setQ(double[])} wherever the data came from outside this process.
 * </p>
 *
 * <h2>Contract</h2>
 * <p>
 * Stateful and <b>not thread safe</b>: it owns a mutable Mecano tree whose reference frames are
 * mutated by {@link #updateFrames()}. One instance per caller. Two solvers sharing one handle
 * would silently evaluate each other's configurations.
 * </p>
 * <p>
 * {@link #packLinkToBase} is allocation-free, so the per-capture inner loops of F3, F4 and F5 --
 * and G2, which evaluates F4 once per capture -- do not allocate.
 * </p>
 */
public class RobotModelHandle
{
   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics baseLink;
   private final ReferenceFrame baseFrame;

   private final OneDoFJointBasics[] joints;
   private final List<String> jointNames;
   private final Map<String, Integer> jointIndicesByName;

   /** Excludes SCS2's synthetic root body, which is not a URDF link and has no link frame. */
   private final Map<String, RigidBodyBasics> bodiesByName = new LinkedHashMap<>();

   /**
    * @param rootBody root of an instantiated Mecano tree, as returned by {@link URDFLoader#load}.
    *                 Held by reference and mutated; do not share it with another handle.
    */
   public RobotModelHandle(RigidBodyBasics rootBody)
   {
      if (rootBody == null)
         throw new IllegalArgumentException("Root body must not be null.");

      this.rootBody = rootBody;

      List<? extends JointBasics> rootJoints = rootBody.getChildrenJoints();

      if (rootJoints.size() != 1)
         throw new IllegalArgumentException("Expected exactly one joint below the synthetic root body, found " + rootJoints.size()
               + ". A URDF has a single root link, so more than one here means the tree was not built by URDFLoader.");

      // The URDF root link: the base, frame `b`. Its frameAfterJoint is the URDF link frame, and
      // it sits below the SixDoFJoint SCS2 inserts -- see the class javadoc.
      this.baseLink = rootJoints.get(0).getSuccessor();
      this.baseFrame = rootJoints.get(0).getFrameAfterJoint();

      JointBasics[] allJoints = MultiBodySystemTools.collectSubtreeJoints(rootBody);
      List<OneDoFJointBasics> oneDoFJoints = new ArrayList<>();

      for (JointBasics joint : allJoints)
      {
         if (joint instanceof OneDoFJointBasics oneDoFJoint)
            oneDoFJoints.add(oneDoFJoint);
      }

      if (oneDoFJoints.isEmpty())
         throw new IllegalArgumentException("Robot '" + rootBody.getName() + "' has no one-DoF joints; there is no q to set.");

      this.joints = oneDoFJoints.toArray(new OneDoFJointBasics[0]);

      List<String> names = new ArrayList<>(joints.length);
      Map<String, Integer> indices = new LinkedHashMap<>();

      for (int i = 0; i < joints.length; i++)
      {
         String name = joints[i].getName();

         if (indices.put(name, i) != null)
            throw new IllegalArgumentException("Robot '" + rootBody.getName() + "' has two joints named '" + name + "'.");

         names.add(name);
      }

      this.jointNames = Collections.unmodifiableList(names);
      this.jointIndicesByName = Collections.unmodifiableMap(indices);

      for (RigidBodyBasics body : rootBody.subtreeIterable())
      {
         // The synthetic root has no parent joint and therefore no URDF link frame. Excluding it
         // keeps getLinkNames() a list of real URDF links, so a MarkerCluster link name that
         // matches nothing fails loudly instead of matching scaffolding.
         if (body.getParentJoint() == null)
            continue;

         if (bodiesByName.put(body.getName(), body) != null)
            throw new IllegalArgumentException("Robot '" + rootBody.getName() + "' has two links named '" + body.getName()
                  + "'. Cluster-to-link assignment is by name (MarkerCluster.getLinkName), so this is unresolvable.");
      }
   }

   /**
    * Loads a URDF and wraps it.
    *
    * @see URDFLoader#load(Path)
    */
   public static RobotModelHandle fromURDF(Path urdfFile) throws IOException
   {
      return new RobotModelHandle(URDFLoader.load(urdfFile));
   }

   /** One-DoF joint names of a tree, in the order this class indexes them. */
   static List<String> jointNamesOf(RigidBodyBasics rootBody)
   {
      return new RobotModelHandle(rootBody).getJointNames();
   }

   /**
    * The tree root. This is SCS2's synthetic {@code rootBody}, <b>not</b> the URDF root link --
    * see {@link #getBaseLinkName()}.
    */
   public RigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   /** Name of the URDF root link: the pelvis, frame {@code b}. */
   public String getBaseLinkName()
   {
      return baseLink.getName();
   }

   /**
    * The base frame {@code b} of FRAMEWORK.md §0: the URDF pelvis link frame.
    * <p>
    * <b>Not</b> the Motive marker-cluster frame {@code c}, and not the IMU frame. Those are three
    * distinct frames all called "pelvis" (§13), and {@code Δ = ^c T_b} is precisely the offset
    * between the first two -- which is what the calibration solves for.
    * </p>
    */
   public ReferenceFrame getBaseFrame()
   {
      return baseFrame;
   }

   public List<String> getJointNames()
   {
      return jointNames;
   }

   public int getJointCount()
   {
      return joints.length;
   }

   /** @throws IllegalArgumentException if no joint has that name. */
   public int indexOfJoint(String jointName)
   {
      Integer index = jointIndicesByName.get(jointName);

      if (index == null)
         throw new IllegalArgumentException("No joint named '" + jointName + "'. Known joints: " + jointNames + ".");

      return index;
   }

   public double getQ(int jointIndex)
   {
      return joints[jointIndex].getQ();
   }

   public void setQ(int jointIndex, double q)
   {
      joints[jointIndex].setQ(q);
   }

   /**
    * Sets every joint from a raw vector in this handle's own index order.
    * <p>
    * Prefer {@link #setConfiguration(EncoderSample)} for anything that came from outside this
    * process: this overload cannot detect a permutation, and a permuted {@code q} is
    * indistinguishable downstream from a bad calibration.
    * </p>
    */
   public void setQ(double[] q)
   {
      if (q.length != joints.length)
         throw new IllegalArgumentException("Expected " + joints.length + " joint values, got " + q.length + ".");

      for (int i = 0; i < joints.length; i++)
         joints[i].setQ(q[i]);
   }

   /** Recomputes every body-fixed frame from the current {@code q}. */
   public void updateFrames()
   {
      rootBody.updateFramesRecursively();
   }

   /**
    * Sets {@code q} from an encoder sample and updates the frames: the whole of "evaluate the model
    * at capture {@code k}".
    * <p>
    * The sample's joint order is checked against this model's, name for name, before anything is
    * set. That check is the entire reason to call this rather than {@link #setQ(double[])}.
    * </p>
    */
   public void setConfiguration(EncoderSample encoderSample)
   {
      encoderSample.checkJointOrder(jointNames);

      for (int i = 0; i < joints.length; i++)
         joints[i].setQ(encoderSample.getQ(i));

      updateFrames();
   }

   public boolean hasLink(String linkName)
   {
      return bodiesByName.containsKey(linkName);
   }

   /** The URDF links, excluding SCS2's synthetic root body. */
   public List<String> getLinkNames()
   {
      return List.copyOf(bodiesByName.keySet());
   }

   /** @throws IllegalArgumentException if no link has that name. */
   public RigidBodyBasics getLink(String linkName)
   {
      RigidBodyBasics body = bodiesByName.get(linkName);

      if (body == null)
         throw new IllegalArgumentException("No link named '" + linkName + "'. Known links: " + bodiesByName.keySet() + ".");

      return body;
   }

   /**
    * The URDF link frame of a link, at the current {@code q}.
    * <p>
    * This is {@code parentJoint.getFrameAfterJoint()}, not {@code getBodyFixedFrame()} -- the
    * latter is the centre-of-mass frame. See the class javadoc for why the distinction is
    * load-bearing rather than cosmetic.
    * </p>
    * <p>
    * Returned rather than copied so that {@code FramePoint3D.changeFrame} can do the
    * back-projection of F3 and F4 directly (FRAMEWORK.md §19 lists that as what Euclid gives
    * free). The frame is live: it moves when {@link #updateFrames()} runs.
    * </p>
    */
   public ReferenceFrame getLinkFrame(String linkName)
   {
      return getLink(linkName).getParentJoint().getFrameAfterJoint();
   }

   /**
    * {@code ^b T_i(q)}: the transform taking a point in link {@code i}'s frame into the base frame
    * {@code b}, at the current configuration.
    * <p>
    * This is the quantity the observation model of FRAMEWORK.md §0 is written in terms of, and the
    * one F3, F4 and F5 all consume. It is a function of joint angles alone -- the floating joint
    * SCS2 inserts above {@code b} cancels out of it entirely.
    * </p>
    * <p>
    * Allocation-free. Call {@link #updateFrames()} first, or {@link #setConfiguration} which does.
    * </p>
    */
   public void packLinkToBase(String linkName, RigidBodyTransform toPack)
   {
      getLinkFrame(linkName).getTransformToDesiredFrame(toPack, baseFrame);
   }

   /** Mass of one link, {@code m_i}, from its URDF inertial block. Assumed, not measured. */
   public double getMass(String linkName)
   {
      RigidBodyBasics body = getLink(linkName);
      return body.getInertia() == null ? 0.0 : body.getInertia().getMass();
   }

   /**
    * {@code ^i c_i}: the link's centre of mass, expressed in its URDF link frame.
    * <p>
    * Read by converting Mecano's CoM frame origin into the link frame rather than by calling
    * {@code getCenterOfMassOffset()}, which is expressed in the CoM frame and is therefore
    * identically zero. This is the {@code ^i c_i} of FRAMEWORK.md §12 and the quantity §14's
    * {@code δ(^i c_i)} term perturbs.
    * </p>
    */
   public void packCenterOfMassInLinkFrame(String linkName, Point3DBasics toPack)
   {
      RigidBodyBasics body = getLink(linkName);
      toPack.setToZero();
      body.getBodyFixedFrame().getTransformToDesiredFrame(getLinkFrame(linkName)).transform(toPack);
   }

   /** Total mass, {@code M = sum_i m_i}. From the URDF inertial blocks -- assumed, not measured. */
   public double getTotalMass()
   {
      double totalMass = 0.0;

      for (RigidBodyBasics body : rootBody.subtreeIterable())
      {
         if (body.getInertia() != null)
            totalMass += body.getInertia().getMass();
      }

      return totalMass;
   }

   @Override
   public String toString()
   {
      return "RobotModelHandle[" + rootBody.getName() + ", base=" + getBaseLinkName() + ", " + joints.length + " joints, " + bodiesByName.size() + " links]";
   }
}
