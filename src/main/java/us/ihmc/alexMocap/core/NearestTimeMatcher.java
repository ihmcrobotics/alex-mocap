package us.ihmc.alexMocap.core;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ToLongFunction;

/**
 * Nearest-neighbour-in-time matching of two time-ordered sample lists, within a maximum delta.
 * <p>
 * Extracted out of {@code EstimatorComparisonRunner} (this project's original, and for a long time
 * only, consumer) so a second caller -- {@code alex}'s NEES consistency checker -- does not have to
 * reimplement it. FRAMEWORK.md §2 states the rule this exists to satisfy: "there must be exactly one
 * implementation." Time alignment between a ground-truth log and an estimator's own log is exactly
 * the kind of logic that looks trivial enough to copy-paste and is not: getting the search-window
 * bookkeeping, the tie-break rule, and the "give up once we've passed it" early-exit all subtly wrong
 * in one of two copies is a much easier way to silently corrupt a comparison than getting any of the
 * surrounding statistics wrong.
 * </p>
 *
 * <h2>Why nearest-neighbour, never interpolation</h2>
 * <p>
 * Interpolating either side's trajectory to hit the other side's timestamps would smooth over
 * exactly the behaviour (e.g. a step change on contact, or a covariance jump right after a filter
 * reseed) that a comparison against ground truth exists to reveal. See
 * {@code EstimatorComparisonRunner}'s own class javadoc for the fuller argument.
 * </p>
 *
 * <h2>Complexity</h2>
 * <p>
 * Both lists must be sorted ascending by timestamp. The search for each truth sample resumes from
 * where the previous truth sample's nearest candidate was found (a moving lower bound), so the whole
 * match is amortised O(|truths| + |candidates|), not O(|truths| * |candidates|).
 * </p>
 */
public final class NearestTimeMatcher
{
   private NearestTimeMatcher()
   {
   }

   /** One matched pair: a truth sample, the nearest candidate within tolerance, and how far apart they were. */
   public record Matched<T, E>(T truth, E candidate, long timeDeltaNanoseconds)
   {
   }

   /**
    * Matches each element of {@code truths} to its nearest-in-time element of {@code candidates},
    * dropping truth samples whose nearest candidate is farther than {@code maxTimeDeltaNanoseconds}
    * away (rather than forcing a match), and preserving {@code truths}' order.
    *
    * @param truths                    time-ordered samples to find a match for.
    * @param candidates                time-ordered samples to match against.
    * @param truthTimestamp            extracts a truth sample's timestamp, nanoseconds.
    * @param candidateTimestamp        extracts a candidate sample's timestamp, nanoseconds.
    * @param maxTimeDeltaNanoseconds   matches farther apart than this are dropped, not reported.
    * @return one {@link Matched} per truth sample that found a candidate within tolerance, in {@code truths}' order.
    */
   public static <T, E> List<Matched<T, E>> match(List<T> truths, List<E> candidates, ToLongFunction<T> truthTimestamp,
                                                   ToLongFunction<E> candidateTimestamp, long maxTimeDeltaNanoseconds)
   {
      List<Matched<T, E>> matches = new ArrayList<>();
      int searchStart = 0;

      for (T truth : truths)
      {
         long targetTimestamp = truthTimestamp.applyAsLong(truth);
         int nearestIndex = nearestIndex(candidates, candidateTimestamp, targetTimestamp, searchStart);

         if (nearestIndex < 0)
            continue;

         E candidate = candidates.get(nearestIndex);
         long deltaNanoseconds = Math.abs(candidateTimestamp.applyAsLong(candidate) - targetTimestamp);

         if (deltaNanoseconds > maxTimeDeltaNanoseconds)
            continue;

         // Both lists are time-ordered, so the next truth sample's nearest candidate can never be
         // earlier than this one's -- resuming here, rather than from 0, is what keeps this amortised.
         searchStart = nearestIndex;

         matches.add(new Matched<>(truth, candidate, deltaNanoseconds));
      }

      return matches;
   }

   /** Linear search from a moving lower bound -- both lists are time-ordered, so this is amortised O(n). */
   private static <E> int nearestIndex(List<E> candidates, ToLongFunction<E> candidateTimestamp, long targetTimestamp, int searchStart)
   {
      int best = -1;
      long bestDelta = Long.MAX_VALUE;

      for (int i = Math.max(0, searchStart); i < candidates.size(); i++)
      {
         long candidateTimeNanoseconds = candidateTimestamp.applyAsLong(candidates.get(i));
         long delta = Math.abs(candidateTimeNanoseconds - targetTimestamp);

         if (delta < bestDelta)
         {
            bestDelta = delta;
            best = i;
         }
         else if (candidateTimeNanoseconds - targetTimestamp > bestDelta)
         {
            // Both lists are sorted by time, so once the candidate list has moved this far past the
            // target the delta can only grow -- stop rather than scan the rest of the list per sample.
            break;
         }
      }

      return best;
   }
}
