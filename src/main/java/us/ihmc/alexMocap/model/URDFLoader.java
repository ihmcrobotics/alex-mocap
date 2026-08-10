package us.ihmc.alexMocap.model;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HexFormat;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

/**
 * Loads a URDF into a Mecano rigid-body tree. This is the only place in the project that touches
 * SCS2 during PR1-PR2, and the whole of F1 (FRAMEWORK.md §3) rests on it.
 *
 * <h2>SCS2 stops here</h2>
 * <p>
 * FRAMEWORK.md §19 requires the calibration to stay headless-testable. The rule as written --
 * "core packages see only Euclid, Mecano, and EJML" -- is narrowed by this class to what it was
 * actually protecting: only {@code scs2-definition} is used, which pulls no JavaFX and pins the
 * same euclid and mecano versions this project already declares. The visualizer
 * ({@code scs2-session-visualizer-jfx}) remains banned everywhere outside the {@code scs2}
 * package, and {@code PackageDependencyTest} enforces both halves of that by scanning compiled
 * classes rather than trusting this paragraph.
 * </p>
 * <p>
 * The containment is structural, not conventional: <b>no SCS2 type appears in any signature
 * here.</b> {@link #load} takes a {@link Path} and returns a Mecano {@link RigidBodyBasics}.
 * {@code URDFModel} and {@code RobotDefinition} are locals inside one method. A caller cannot
 * reach an SCS2 type through this class even by accident, which is why the Gradle dependency is
 * declared {@code implementation} rather than {@code api}.
 * </p>
 *
 * <h2>Why the hash is here</h2>
 * <p>
 * {@code CalibrationResult.Provenance} records which URDF a calibration was solved against, and a
 * file name does not pin a file that someone edits in place. {@link #sha256} lives next to the
 * loader so the hash is taken from the same bytes that were parsed, rather than recomputed later
 * from a path that may by then point somewhere else.
 * </p>
 */
public final class URDFLoader
{
   private URDFLoader()
   {
   }

   /**
    * Loads a URDF and instantiates it as a Mecano tree rooted at {@link ReferenceFrame#getWorldFrame()}.
    *
    * @see #load(Path, ReferenceFrame)
    */
   public static RigidBodyBasics load(Path urdfFile) throws IOException
   {
      return load(urdfFile, ReferenceFrame.getWorldFrame());
   }

   /**
    * Loads a URDF and instantiates it as a Mecano tree.
    * <p>
    * <b>The returned body is not the URDF root link.</b> SCS2 instantiates a synthetic body named
    * {@code rootBody} and attaches the URDF root link beneath it through a {@code SixDoFJoint} that
    * appears nowhere in the URDF. So the tree is {@code rootBody -> [6-DoF] -> pelvis -> ...}, and
    * the thing a caller wants -- the base frame {@code b} of FRAMEWORK.md §0 -- is the frame
    * <i>after</i> that floating joint.
    * </p>
    * <p>
    * This is worth stating at the loader rather than leaving it to be discovered, because taking
    * the synthetic root's frame as {@code b} is both the obvious reading and wrong: every
    * {@code ^b T_i} would then include the floating joint, and §0's load-bearing claim that
    * {@code ^b T_i(q)} is a function of joint angles alone would quietly stop holding. Where the
    * robot is in the room is F5's answer, not the model's. {@link RobotModelHandle} is what
    * resolves this correctly; prefer {@link RobotModelHandle#fromURDF(Path)} to calling this
    * directly.
    * </p>
    *
    * @param urdfFile    the file to parse.
    * @param parentFrame the stationary frame the root body is attached to.
    * @return the root body of the instantiated tree.
    * @throws IOException              if the file cannot be read.
    * @throws IllegalArgumentException if it parses but is not a usable robot.
    */
   public static RigidBodyBasics load(Path urdfFile, ReferenceFrame parentFrame) throws IOException
   {
      if (urdfFile == null)
         throw new IllegalArgumentException("URDF path must not be null.");
      if (!Files.isRegularFile(urdfFile))
         throw new IOException("No URDF file at " + urdfFile.toAbsolutePath() + ".");

      File file = urdfFile.toFile();
      URDFModel urdfModel;

      try
      {
         urdfModel = URDFTools.loadURDFModel(file);
      }
      catch (Exception e)
      {
         // JAXBException and friends. Two things go in the message deliberately.
         //
         // The path, because a parse failure carrying only a JAXB stack trace sends you looking at
         // the parser instead of at the file.
         //
         // describeCause(e) rather than e.getMessage(), because JAXB's UnmarshalException returns
         // null from getMessage() and keeps the only useful text -- the SAXParseException with the
         // line and column -- in its cause. Reporting getMessage() directly produces the literal
         // string "Failed to parse URDF at /path/to.urdf: null", which names the file and then
         // withholds everything that would let you fix it.
         throw new IOException("Failed to parse URDF at " + urdfFile.toAbsolutePath() + ": " + describeCause(e), e);
      }

      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel);

      if (robotDefinition.getRootBodyDefinition() == null)
         throw new IllegalArgumentException("URDF at " + urdfFile.toAbsolutePath() + " has no root link.");

      RigidBodyBasics rootBody = robotDefinition.newInstance(parentFrame);

      // Deliberately counts one-DoF joints in the whole subtree, not rootBody.getChildrenJoints().
      // The latter is never empty -- it always holds the synthetic SixDoFJoint -- so checking it
      // would pass a URDF consisting of nothing but a root link.
      long articulatedJointCount = rootBody.subtreeStream().filter(body -> body.getParentJoint() instanceof OneDoFJointBasics).count();

      if (articulatedJointCount == 0)
         throw new IllegalArgumentException("URDF at " + urdfFile.toAbsolutePath() + " has no articulated joints below its root link. "
               + "There is nothing for F5 to identify Δ from (FRAMEWORK.md §7).");

      return rootBody;
   }

   /**
    * The deepest non-empty message in a throwable chain, prefixed by its type.
    * <p>
    * Walks to the root cause because the wrapper layers are the uninformative ones: JAXB's
    * {@code UnmarshalException} has a null message and Xerces' {@code SAXParseException}
    * underneath it has the line, the column, and what was actually wrong.
    * </p>
    */
   private static String describeCause(Throwable throwable)
   {
      Throwable deepest = throwable;

      while (deepest.getCause() != null && deepest.getCause() != deepest)
         deepest = deepest.getCause();

      String message = deepest.getMessage();
      return message == null || message.isBlank() ? deepest.toString() : deepest.getClass().getSimpleName() + ": " + message;
   }

   /**
    * SHA-256 of the URDF file's bytes, lowercase hex, for
    * {@code CalibrationResult.Provenance.urdfSha256}.
    */
   public static String sha256(Path urdfFile) throws IOException
   {
      MessageDigest digest;

      try
      {
         digest = MessageDigest.getInstance("SHA-256");
      }
      catch (NoSuchAlgorithmException e)
      {
         throw new IllegalStateException("SHA-256 is required of every JVM but is missing.", e);
      }

      return HexFormat.of().formatHex(digest.digest(Files.readAllBytes(urdfFile)));
   }

   /**
    * Names of the URDF's one-DoF joints, in the order {@link RobotModelHandle} indexes them.
    * <p>
    * Available without instantiating a tree so that a caller reading a capture CSV can check its
    * encoder column order against the URDF before building anything -- {@link
    * us.ihmc.alexMocap.core.EncoderSample#checkJointOrder} is the other half of that.
    * </p>
    */
   public static List<String> readJointNames(Path urdfFile) throws IOException
   {
      return RobotModelHandle.jointNamesOf(load(urdfFile));
   }
}
