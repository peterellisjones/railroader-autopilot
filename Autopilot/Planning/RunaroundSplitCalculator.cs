using System;
using System.Collections.Generic;

namespace Autopilot.Planning
{
    /// <summary>
    /// Determines the optimal split point for a runaround.
    ///
    /// Given a train [tail(0) ... inner(N-1) LOCO], the loco detaches at
    /// some point K, goes around, and couples to the tail end.
    ///
    /// After runaround with K cars kept:
    ///   - Kept cars (inner K) become sideA, new tail = car[N-1] (innermost)
    ///   - Detached cars (outer N-K) become sideB, new tail = car[N-K-1] (at split)
    ///
    /// The optimal split maximizes consecutive deliverable cars from BOTH
    /// new tails, minimizing future runarounds.
    /// </summary>
    public static class RunaroundSplitCalculator
    {
        /// <summary>
        /// Calculate how many cars to keep with the loco during a runaround.
        /// Cars are ordered tail(0) to loco(N-1).
        ///
        /// For each possible split, scores = consecutive deliverable from
        /// each new tail. Picks the split with the highest total score.
        /// </summary>
        /// <param name="carCount">Total non-loco cars.</param>
        /// <param name="isLocoOrTender">True if car at index i is a loco/tender.</param>
        /// <param name="isDeliverableDirect">
        /// True if car at index i can be delivered directly (no runaround)
        /// from the tail direction. Checked from the outer end inward.
        /// </param>
        /// <param name="isDeliverableAfterFlip">
        /// True if car at index i can be delivered after a runaround
        /// (from the inner direction). Checked from the inner end outward.
        /// </param>
        /// <returns>Number of cars to keep from the inner end (includes tenders).</returns>
        public static int CalculateCarsToKeep(
            int carCount,
            Func<int, bool> isLocoOrTender,
            Func<int, bool> isDeliverableDirect,
            Func<int, bool> isDeliverableAfterFlip)
        {
            if (carCount <= 0) return 0;

            // Count tenders at the inner end — they always stay with loco
            int tenderCount = 0;
            for (int i = carCount - 1; i >= 0; i--)
            {
                if (isLocoOrTender(i))
                    tenderCount++;
                else
                    break;
            }

            int freightCount = carCount - tenderCount;
            if (freightCount <= 0) return tenderCount;

            int bestKeep = 0;
            int bestScore = 0;

            // Try each split: keep K freight cars from the inner end (+ tenders)
            for (int k = 0; k <= freightCount; k++)
            {
                int detached = freightCount - k; // outer cars, become sideB

                // Score from sideB (detached side): after ruaround, the loco is
                // at the old tail end. The car at the split point (index detached-1)
                // is the new sideB tail. The loco pushes from the old tail side
                // toward the new tail. These cars moved from the inner end to the
                // tail — use isDeliverableAfterFlip (not direct).
                int scoreSideB = 0;
                for (int i = detached - 1; i >= 0; i--)
                {
                    if (isLocoOrTender(i)) continue;
                    if (isDeliverableAfterFlip(i))
                        scoreSideB++;
                    else
                        break;
                }

                // Score from sideA (kept side): the kept inner cars become the far end.
                // The innermost kept car (index freightCount-1) is the new sideA tail.
                // Count consecutive deliverable from there outward.
                // These are "after flip" deliveries since they're on the opposite side.
                int scoreSideA = 0;
                for (int i = freightCount - 1; i >= detached; i--)
                {
                    if (isLocoOrTender(i)) continue;
                    if (isDeliverableAfterFlip(i))
                        scoreSideA++;
                    else
                        break;
                }

                int totalScore = scoreSideA + scoreSideB;

                if (totalScore > bestScore || (totalScore == bestScore && k < bestKeep))
                {
                    bestScore = totalScore;
                    bestKeep = k;
                }
            }

            return bestKeep + tenderCount;
        }

        /// <summary>
        /// Simplified version: loco always goes alone (keeps 0 freight cars).
        /// Only keeps tenders.
        /// </summary>
        public static int CalculateCarsToKeepSimple(int carCount, Func<int, bool> isLocoOrTender)
        {
            int tenderCount = 0;
            for (int i = carCount - 1; i >= 0; i--)
            {
                if (isLocoOrTender(i))
                    tenderCount++;
                else
                    break;
            }
            return tenderCount;
        }
    }
}
