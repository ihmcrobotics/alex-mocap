package us.ihmc.alexMocap.gates;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.alexMocap.core.Capture;
import us.ihmc.alexMocap.core.MarkerCluster;
import us.ihmc.alexMocap.core.MarkerObservation;
import us.ihmc.alexMocap.core.ObservationModel;
import us.ihmc.alexMocap.model.RobotModelHandle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * G2, the bootstrap-spread gate (FRAMEWORK.md §15).
 * <p>
 * Evaluate F4 <i>per capture</i> instead of averaging, giving {@code K} independent estimates of
 * the same {@code ^i p_ij}. If FK were exact and mocap noise zero-mean, their spread would be pure
 * mocap noise. Anything larger is systematic.
 * </p>
 *
 * <h2>The gate that says which, not merely that</h2>
 * <p>
 * A spread number alone is nearly useless, because §8's escalation table asks a question a scalar
 * cannot answer: does the spread correlate with <i>a particular joint's excursion</i> (indicting
 * that joint's offset or the link geometry below it), with <i>limb load</i> (indicting joint
 * elasticity under gravity), or with <i>nothing</i> (in which case it is mocap noise and A′ is
 * sufficient)? Those three lead to three different responses, one of which is "do nothing".
 * </p>
 * <p>
 * So this gate reports, per marker, the Pearson correlation between the per-capture deviation and
 * each joint's excursion, and against a gravitational load proxy. {@link #getDiagnoses()} carries
 * the structured version; the findings carry the headline.
 * </p>
 *
 * <h2>Δ is an input, and why that is not cheating</h2>
 * <p>
 * §15 says G2 "needs no optimiser, no initial guess, and no ground truth", and that it "runs at
 * time zero". That is true of the <i>algorithm</i> -- there is nothing to solve here -- but the
 * back-projection is not {@code Δ}-free: a wrong {@code Δ} contributes
 * {@code ^b T_i(q^(k))^-1 · (Δ_wrong^-1 Δ_true) · ^b T_i(q^(k))} to each estimate, which varies
 * with {@code k} and therefore inflates the spread.
 * </p>
 * <p>
 * The consequence is worth being precise about, because it decides how to read a failure. A wrong
 * {@code Δ} can only make the spread <b>larger</b>, never smaller, so G2 run at time zero with
 * {@code Δ = I} is <b>conservative</b>: it can cry wolf, it cannot miss a real systematic error.
 * That is the safe direction for a gate. For the diagnostic reading -- which joint -- pass the
 * solved {@code Δ}, or the structure will be swamped by the {@code Δ} error's own correlation with
 * configuration.
 * </p>
 *
 * <h2>Why it does not import the calibrator</h2>
 * <p>
 * FRAMEWORK.md §19 forbids {@code gates → calibration}, and this is the gate that rule exists for.
 * Everything needed is passed in at construction: raw captures, the model, {@code Δ}, and the gauge
 * cluster poses. G2 can therefore run before {@code AlternatingCalibrator} exists, which is exactly
 * what lets it tell you whether Approach B is necessary before anyone writes it.
 * </p>
 */
public class BootstrapSpreadGate implements Gate
{
   /**
    * Above this magnitude, a correlation is called structure rather than coincidence.
    * <p>
    * <b>Chosen</b>, not derived. At {@code K = 30} the 95% two-sided critical value for a Pearson
    * correlation under the null is about 0.36, so 0.5 is deliberately clear of the noise floor. It
    * is the threshold for the <i>narrative</i> ("this indicts the knee"), never for the pass/fail
    * verdict, which rests on the spread alone.
    * </p>
    */
   public static final double STRUCTURE_CORRELATION_THRESHOLD = 0.5;

   /** §15: fail if the spread exceeds this multiple of the measured per-axis mocap noise. */
   public static final double DEFAULT_SPREAD_SIGMA_MULTIPLE = 3.0;

   /**
    * What the spread of one marker's per-capture back-projections looks like, and what it tracks.
    *
    * @param spreadMeters              RMS distance of the per-capture estimates from their mean.
    * @param expectedSpreadMeters      what pure mocap noise alone would produce.
    * @param strongestJointName        the joint whose excursion best explains the deviation.
    * @param strongestJointCorrelation that joint's Pearson correlation, in {@code [-1, 1]}.
    * @param loadCorrelation           correlation against the gravitational load proxy.
    */
   public record SpreadDiagnosis(String linkName, String markerName, int captureCount, double spreadMeters, double expectedSpreadMeters,
         String strongestJointName, double strongestJointCorrelation, double loadCorrelation)
   {
      /**
       * Which row of §8's escalation table this marker points at.
       * <p>
       * <b>A caveat that matters when reading this string.</b> Separating "joint offset" from
       * "elasticity" by correlation alone is weak, because elastic deflection <i>is</i> a function
       * of joint angles -- it enters through the gravitational torque -- so it correlates strongly
       * with joint excursion as well as with load. Measured on an injected compliance, joint
       * correlation usually wins the comparison below even though the true cause is load.
       * </p>
       * <p>
       * The reliable discriminator is <b>spatial</b>, not statistical, and it lives at the level of
       * the whole gate rather than one marker: a joint offset raises spread only on the links below
       * it on <i>one</i> branch, while elasticity raises it on <i>every</i> loaded branch at once.
       * Read this field as a hint and the pattern of failing links as the evidence.
       * </p>
       */
      public String indictment()
      {
         boolean jointStructure = Math.abs(strongestJointCorrelation) >= STRUCTURE_CORRELATION_THRESHOLD;
         boolean loadStructure = Math.abs(loadCorrelation) >= STRUCTURE_CORRELATION_THRESHOLD;

         if (loadStructure && Math.abs(loadCorrelation) >= Math.abs(strongestJointCorrelation))
            return "limb load -> joint elasticity under gravity; promote elastic parameters";
         if (jointStructure)
            return "excursion of '" + strongestJointName + "' -> that joint's offset, or link geometry below it";

         return "nothing -- isotropic, consistent with mocap noise; A' is sufficient";
      }
   }

   private final List<Capture> captures;
   private final List<MarkerCluster> clusters;
   private final RobotModelHandle model;
   private final RigidBodyTransform clusterToBase = new RigidBodyTransform();
   private final List<RigidBodyTransformReadOnly> clusterToWorld;
   private final double perAxisNoiseStandardDeviation;
   private final double spreadSigmaMultiple;

   private final List<SpreadDiagnosis> diagnoses = new ArrayList<>();

   /** Gauge cluster geometry, measured from raw mocap by {@link #measureGaugeGeometry()}. */
   private int gaugeMarkerCount;
   private double gaugeRadius;

   /**
    * @param captures                      the calibration captures.
    * @param clusters                      every marked cluster.
    * @param model                         the FK reference.
    * @param clusterToBase                 {@code Δ}. Identity for a time-zero run; see the class
    *                                      javadoc for how that changes the reading.
    * @param clusterToWorld                {@code ^W T_c^(k)} per capture, {@code null} where the
    *                                      gauge cluster could not be tracked.
    * @param perAxisNoiseStandardDeviation the <b>measured</b> per-axis mocap noise {@code σ}. This
    *                                      is what FRAMEWORK.md §20.1 sends someone to the gantry
    *                                      for; the wand residual is an average over the whole lab
    *                                      and is not a substitute.
    */
   public BootstrapSpreadGate(List<Capture> captures,
                              List<MarkerCluster> clusters,
                              RobotModelHandle model,
                              RigidBodyTransformReadOnly clusterToBase,
                              List<RigidBodyTransformReadOnly> clusterToWorld,
                              double perAxisNoiseStandardDeviation)
   {
      this(captures, clusters, model, clusterToBase, clusterToWorld, perAxisNoiseStandardDeviation, DEFAULT_SPREAD_SIGMA_MULTIPLE);
   }

   public BootstrapSpreadGate(List<Capture> captures,
                              List<MarkerCluster> clusters,
                              RobotModelHandle model,
                              RigidBodyTransformReadOnly clusterToBase,
                              List<RigidBodyTransformReadOnly> clusterToWorld,
                              double perAxisNoiseStandardDeviation,
                              double spreadSigmaMultiple)
   {
      if (captures.size() != clusterToWorld.size())
         throw new IllegalArgumentException("Got " + captures.size() + " captures but " + clusterToWorld.size() + " cluster poses.");
      if (!(perAxisNoiseStandardDeviation > 0.0))
         throw new IllegalArgumentException("The per-axis mocap noise must be a positive measured value, was " + perAxisNoiseStandardDeviation
               + ". FRAMEWORK.md §20.1: measure it at the gantry; the wand residual is not a substitute.");

      this.captures = captures;
      this.clusters = clusters;
      this.model = model;
      this.clusterToBase.set(clusterToBase);
      this.clusterToWorld = clusterToWorld;
      this.perAxisNoiseStandardDeviation = perAxisNoiseStandardDeviation;
      this.spreadSigmaMultiple = spreadSigmaMultiple;
   }

   @Override
   public String getName()
   {
      return "G2";
   }

   @Override
   public String getDescription()
   {
      return "Bootstrap spread: per-capture back-projections of each marker must agree to within mocap noise. "
            + "Structure in the disagreement says which model assumption is wrong (FRAMEWORK.md §15, §8).";
   }

   @Override
   public GateResult run()
   {
      diagnoses.clear();
      measureGaugeGeometry();
      GateResult result = new GateResult(getName());

      List<Integer> usableCaptures = new ArrayList<>();

      for (int k = 0; k < captures.size(); k++)
      {
         if (clusterToWorld.get(k) != null)
            usableCaptures.add(k);
      }

      // Per-capture regressors, evaluated once: each joint's angle, and a gravitational load proxy.
      int jointCount = model.getJointCount();
      double[][] jointAngles = new double[usableCaptures.size()][jointCount];
      double[] load = new double[usableCaptures.size()];

      for (int index = 0; index < usableCaptures.size(); index++)
      {
         int k = usableCaptures.get(index);
         model.setConfiguration(captures.get(k).getEncoderSample());

         for (int j = 0; j < jointCount; j++)
            jointAngles[index][j] = model.getQ(j);

         load[index] = gravitationalLoadProxy();
      }

      RigidBodyTransform linkToBase = new RigidBodyTransform();
      RigidBodyTransform linkToWorld = new RigidBodyTransform();
      Point3D backProjected = new Point3D();

      for (MarkerCluster cluster : clusters)
      {
         for (int j = 0; j < cluster.getMarkerCount(); j++)
         {
            List<Point3D> estimates = new ArrayList<>();
            List<Integer> contributingCaptures = new ArrayList<>();
            double leverArmSum = 0.0;

            for (int index = 0; index < usableCaptures.size(); index++)
            {
               int k = usableCaptures.get(index);
               MarkerObservation observation = captures.get(k).getMocapFrame().get(cluster.getMarker(j));

               if (!observation.isVisible())
                  continue;

               model.setConfiguration(captures.get(k).getEncoderSample());
               model.packLinkToBase(cluster.getLinkName(), linkToBase);
               ObservationModel.packLinkToWorld(clusterToWorld.get(k), clusterToBase, linkToBase, linkToWorld);
               ObservationModel.packMarkerInLinkFrame(linkToWorld, observation.getPosition(), backProjected);

               estimates.add(new Point3D(backProjected));
               contributingCaptures.add(index);

               // Lever arm from the base origin to this marker at this capture: what multiplies
               // the gauge cluster's angular error.
               Point3D inBase = new Point3D(backProjected);
               linkToBase.transform(inBase);
               leverArmSum += inBase.norm();
            }

            String subject = cluster.getLinkName() + ": " + cluster.getMarker(j).getName();

            // Two estimates define a mean with zero degrees of freedom left over; the spread of
            // such a pair carries no information about whether the model is right.
            if (estimates.size() < 3)
            {
               result.add(GateResult.Finding.notEvaluated(subject,
                                                          estimates.size(),
                                                          "Visible in only " + estimates.size()
                                                                + " usable captures; at least 3 are needed for a spread to mean anything."));
               continue;
            }

            double leverArm = leverArmSum / estimates.size();
            double expectedSpread = expectedSpreadFromNoise(leverArm);
            addFinding(result, cluster, j, subject, estimates, contributingCaptures, jointAngles, load, expectedSpread, spreadSigmaMultiple * expectedSpread);
         }
      }

      long failures = result.getFailures().size();
      result.setSummary(failures == 0 ? "No marker's per-capture spread exceeds " + spreadSigmaMultiple + "σ; nothing indicts the model."
            : failures + " marker(s) spread beyond " + spreadSigmaMultiple + "σ. See the indictment column and FRAMEWORK.md §8's escalation table.");

      return result;
   }

   private void addFinding(GateResult result,
                           MarkerCluster cluster,
                           int markerIndex,
                           String subject,
                           List<Point3D> estimates,
                           List<Integer> contributingCaptures,
                           double[][] jointAngles,
                           double[] load,
                           double expectedSpread,
                           double threshold)
   {
      Point3D mean = new Point3D();

      for (Point3D estimate : estimates)
         mean.add(estimate);

      mean.scale(1.0 / estimates.size());

      // Signed components of the deviation, not its magnitude.
      //
      // Correlating |deviation| against a joint angle finds almost nothing, and the reason is
      // geometric rather than statistical: a small joint-offset error displaces a marker by
      // roughly a linear function of the joints below it, so the SIGNED displacement tracks those
      // angles while its magnitude behaves like |linear| -- a V shape, whose Pearson correlation
      // with the angle is near zero however strong the underlying dependence. Measured on the
      // 0.5° l_hip injection, magnitudes reported "indicts nothing" on markers that were failing
      // the spread test by 3.2σ.
      double[][] deviation = new double[3][estimates.size()];
      double sumOfSquares = 0.0;

      for (int i = 0; i < estimates.size(); i++)
      {
         deviation[0][i] = estimates.get(i).getX() - mean.getX();
         deviation[1][i] = estimates.get(i).getY() - mean.getY();
         deviation[2][i] = estimates.get(i).getZ() - mean.getZ();
         sumOfSquares += estimates.get(i).distanceSquared(mean);
      }

      double spread = Math.sqrt(sumOfSquares / estimates.size());

      // Which regressor best explains the deviation.
      String strongestJoint = "";
      double strongestJointCorrelation = 0.0;

      for (int j = 0; j < model.getJointCount(); j++)
      {
         double[] excursion = new double[estimates.size()];

         for (int i = 0; i < estimates.size(); i++)
            excursion[i] = jointAngles[contributingCaptures.get(i)][j];

         double correlation = strongestComponentCorrelation(excursion, deviation);

         if (Math.abs(correlation) > Math.abs(strongestJointCorrelation))
         {
            strongestJointCorrelation = correlation;
            strongestJoint = model.getJointNames().get(j);
         }
      }

      double[] loadForMarker = new double[estimates.size()];

      for (int i = 0; i < estimates.size(); i++)
         loadForMarker[i] = load[contributingCaptures.get(i)];

      double loadCorrelation = strongestComponentCorrelation(loadForMarker, deviation);

      SpreadDiagnosis diagnosis = new SpreadDiagnosis(cluster.getLinkName(),
                                                      cluster.getMarker(markerIndex).getName(),
                                                      estimates.size(),
                                                      spread,
                                                      expectedSpread,
                                                      strongestJoint,
                                                      strongestJointCorrelation,
                                                      loadCorrelation);
      diagnoses.add(diagnosis);

      String detail = String.format("spread %.3f mm vs %.3f mm expected; indicts %s", 1000.0 * spread, 1000.0 * expectedSpread, diagnosis.indictment());

      if (spread > threshold)
         result.add(GateResult.Finding.fail(subject, spread, threshold, estimates.size(), detail));
      else
         result.add(GateResult.Finding.pass(subject, spread, threshold, estimates.size(), detail));
   }

   /**
    * The spread pure mocap noise alone would produce, for a marker at the given lever arm from the
    * base origin.
    *
    * <h2>Three terms, and the third is usually the largest</h2>
    * <p>
    * The obvious term is <b>direct measurement noise</b>. Back-projection applies
    * {@code (^W T_i^(k))^-1}, and a rotation preserves length, so a per-axis {@code σ} arrives in
    * the link frame still {@code σ} per axis, contributing {@code 3σ²} to the squared 3-D spread.
    * </p>
    * <p>
    * A gate built on that term alone <b>fires on clean data</b>, because the back-projection also
    * inherits the error in {@code ^W T_c^(k)} -- and that pose is estimated from a handful of gauge
    * markers, not measured. It contributes two further terms:
    * </p>
    * <ul>
    * <li>a <b>translational</b> part, {@code σ/√N_g}, from the gauge centroid;</li>
    * <li>an <b>angular</b> part, {@code σ / (√N_g · r_g)} -- FRAMEWORK.md §1's scaling -- which
    * displaces a marker by that angle times its lever arm from the base. Two effective degrees of
    * freedom, since rotation about the lever direction moves nothing.</li>
    * </ul>
    * <p>
    * For the far links the angular term dominates everything else: at {@code σ = 0.3 mm} with four
    * gauge markers spread over 140 mm, it is about 2 mrad, which is 1.3 mm at a 0.6 m lever against
    * a 0.5 mm direct term. That is why this gate's expectation is <b>per marker</b> rather than one
    * number for the session, and it is the same physics that makes a wide pelvis cluster a hardware
    * requirement rather than a nicety.
    * </p>
    *
    * @param leverArm distance from the base origin to the marker, metres.
    */
   private double expectedSpreadFromNoise(double leverArm)
   {
      int k = Math.max(2, captures.size());
      double finiteSampleCorrection = Math.sqrt((k - 1.0) / k);

      double direct = 3.0 * perAxisNoiseStandardDeviation * perAxisNoiseStandardDeviation;
      double gaugeTranslation = 3.0 * perAxisNoiseStandardDeviation * perAxisNoiseStandardDeviation / gaugeMarkerCount;

      double gaugeAngle = perAxisNoiseStandardDeviation / (Math.sqrt(gaugeMarkerCount) * gaugeRadius);
      double gaugeRotation = 2.0 * gaugeAngle * leverArm * gaugeAngle * leverArm;

      return Math.sqrt(direct + gaugeTranslation + gaugeRotation) * finiteSampleCorrection;
   }

   /**
    * Geometry of the gauge cluster, read from raw mocap: the number of markers it presents and the
    * RMS distance of those markers from their centroid.
    * <p>
    * Taken from the data rather than from a configured constant so that the expectation reflects
    * the cluster that was actually mounted -- including a marker that never appeared, which makes
    * the cluster both smaller and worse conditioned than the drawing says.
    * </p>
    */
   private void measureGaugeGeometry()
   {
      MarkerCluster gauge = null;

      for (MarkerCluster cluster : clusters)
      {
         if (cluster.getLinkName().equals(model.getBaseLinkName()))
            gauge = cluster;
      }

      if (gauge == null)
         throw new IllegalArgumentException("No cluster is mounted on the base link '" + model.getBaseLinkName()
               + "'. Without the gauge cluster there is no ^W T_c^(k) and nothing here is defined.");

      for (int k = 0; k < captures.size(); k++)
      {
         if (clusterToWorld.get(k) == null)
            continue;

         Point3D centroid = new Point3D();
         int visible = 0;

         for (int j = 0; j < gauge.getMarkerCount(); j++)
         {
            MarkerObservation observation = captures.get(k).getMocapFrame().get(gauge.getMarker(j));

            if (observation.isVisible())
            {
               centroid.add(observation.getPosition());
               visible++;
            }
         }

         if (visible < 3)
            continue;

         centroid.scale(1.0 / visible);
         double sumOfSquares = 0.0;

         for (int j = 0; j < gauge.getMarkerCount(); j++)
         {
            MarkerObservation observation = captures.get(k).getMocapFrame().get(gauge.getMarker(j));

            if (observation.isVisible())
               sumOfSquares += observation.getPosition().distanceSquared(centroid);
         }

         gaugeMarkerCount = visible;
         gaugeRadius = Math.sqrt(sumOfSquares / visible);
         return;
      }

      throw new IllegalArgumentException("The gauge cluster '" + gauge.getLinkName() + "' is never visible enough to define a pose.");
   }

   /**
    * A scalar standing in for "how gravity-loaded the structure is in this pose": the total
    * gravitational moment about the base origin, {@code sum_i m_i · ||horizontal offset of ^b c_i||}.
    * <p>
    * A proxy, deliberately. The exact quantity elasticity responds to is the torque at each
    * individual joint, which depends on compliances nobody has measured. What this needs to be is
    * monotone in limb extension and uncorrelated with any single joint's excursion, so that
    * load-driven spread separates from offset-driven spread -- and the total moment about the base
    * is both.
    * </p>
    */
   private double gravitationalLoadProxy()
   {
      RigidBodyTransform linkToBase = new RigidBodyTransform();
      Point3D centerOfMass = new Point3D();
      double moment = 0.0;

      for (String linkName : model.getLinkNames())
      {
         model.packCenterOfMassInLinkFrame(linkName, centerOfMass);
         model.packLinkToBase(linkName, linkToBase);
         linkToBase.transform(centerOfMass);

         moment += model.getMass(linkName) * Math.hypot(centerOfMass.getX(), centerOfMass.getY());
      }

      return moment;
   }

   /** Structured per-marker diagnoses from the last {@link #run()}. */
   public List<SpreadDiagnosis> getDiagnoses()
   {
      return diagnoses;
   }

   /** @return the diagnosis for one marker, or {@code null} if it was not evaluated. */
   public SpreadDiagnosis findDiagnosis(String linkName, String markerName)
   {
      for (SpreadDiagnosis diagnosis : diagnoses)
      {
         if (diagnosis.linkName().equals(linkName) && diagnosis.markerName().equals(markerName))
            return diagnosis;
      }

      return null;
   }

   /**
    * The strongest correlation between a regressor and any single component of the deviation.
    * <p>
    * Taking the best of the three axes rather than a combined statistic keeps the answer
    * interpretable: the reported number is a correlation someone can go and plot.
    * </p>
    */
   private static double strongestComponentCorrelation(double[] regressor, double[][] deviation)
   {
      double strongest = 0.0;

      for (int axis = 0; axis < 3; axis++)
      {
         double correlation = pearson(regressor, deviation[axis]);

         if (Math.abs(correlation) > Math.abs(strongest))
            strongest = correlation;
      }

      return strongest;
   }

   /** Pearson correlation. Zero when either input is constant, which is the honest answer. */
   static double pearson(double[] x, double[] y)
   {
      int n = x.length;

      if (n < 2)
         return 0.0;

      double meanX = 0.0, meanY = 0.0;

      for (int i = 0; i < n; i++)
      {
         meanX += x[i];
         meanY += y[i];
      }

      meanX /= n;
      meanY /= n;

      double covariance = 0.0, varianceX = 0.0, varianceY = 0.0;

      for (int i = 0; i < n; i++)
      {
         double dx = x[i] - meanX;
         double dy = y[i] - meanY;
         covariance += dx * dy;
         varianceX += dx * dx;
         varianceY += dy * dy;
      }

      if (varianceX <= 0.0 || varianceY <= 0.0)
         return 0.0;

      return covariance / Math.sqrt(varianceX * varianceY);
   }
}
