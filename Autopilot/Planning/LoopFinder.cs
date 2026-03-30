using System;
using System.Collections.Generic;
using System.Linq;
using Autopilot.TrackGraph;
using Autopilot.Model;

namespace Autopilot.Planning
{
    /// <summary>
    /// Detects passing sidings and wyes where the train fits, for runaround maneuvers.
    /// Walks outward from the locomotive, finds switches that form loops, and checks
    /// whether the train fits on any branch.
    ///
    /// Operates entirely on string IDs via IGraphAdapter — no game type dependencies.
    /// </summary>
    public class LoopFinder
    {
        private const float MaxSearchDistance = 3200f; // ~2 miles
        private const int MaxInitialWalkSegments = 50;
        private const int MaxExitWalkSegments = 30;

        private readonly IGraphAdapter _graph;
        private readonly Action<string> _log;
        private IEnumerable<string>? _visitedSwitches;

        private void Log(string msg) => _log?.Invoke(msg);

        /// <summary>
        /// Path data from BFS through switch topology. Records how we reached a switch
        /// from a given exit of another switch.
        /// </summary>
        private class ExitPath
        {
            public string SwitchId;
            public List<string> SegmentIds;
            public float Distance;
            public List<string> IntermediateSwitchIds;
            public string ArrivalSegId;
        }

        public LoopFinder(IGraphAdapter graph, Action<string> log = null)
        {
            _graph = graph;
            _log = log;
        }

        /// <summary>
        /// Find the nearest loop (passing siding or wye) where the train fits.
        /// Walks from the start segment in both directions, testing each switch found.
        /// </summary>
        /// <param name="startSegId">Segment the loco is on.</param>
        /// <param name="distToA">Distance from loco to end A of the start segment.</param>
        /// <param name="distToB">Distance from loco to end B of the start segment.</param>
        /// <param name="trainLength">Total train length in meters.</param>
        /// <param name="visitedSwitches">Switches already tried (skip these).</param>
        /// <param name="canReachSwitch">
        /// Tests if the loco can route to a point on segment segId near end A (true) or B (false).
        /// </param>
        /// <param name="visitedLoopKeys">
        /// Loop keys already tried (sorted switch ID strings). Loops with
        /// matching keys are skipped. A loop key is its switches sorted and
        /// joined, e.g., "Na|Nb". Loops can share switches but not identical sets.
        /// </param>
        public LoopInfo FindNearestFittingLoop(
            string startSegId, float distToA, float distToB,
            float trainLength, IEnumerable<string>? visitedSwitches,
            Func<string, bool, bool> canReachSwitch,
            IEnumerable<string>? visitedLoopKeys = null,
            Func<string, bool> isSegmentBlocked = null,
            Func<string, string, float, bool> hasReversalClearance = null,
            float locoLength = 0f)
        {
            _visitedSwitches = visitedSwitches;

            var allSwitches = FindNearbySwitches(startSegId, distToA, distToB);
            Log($"Found {allSwitches.Count} switches within search range");

            // Collect fitting loops. allSwitches is sorted by distance, so we can
            // early-exit once we have an all-fit candidate and the distance to
            // the next switch exceeds its total score.
            var candidates = new List<(LoopInfo loop, float farSwitchDist, int fittingBranches)>();
            float bestAllFitTotal = float.MaxValue;

            foreach (var (switchId, arrivalSegId, distance) in allSwitches)
            {
                // Early exit: only when we have an all-fit candidate (highest priority).
                // A farther switch can't beat an all-fit loop if just the distance
                // to the switch already exceeds the all-fit total.
                if (bestAllFitTotal < float.MaxValue && distance >= bestAllFitTotal)
                {
                    Log($"Early exit: switch distance {distance:F0}m >= best all-fit total {bestAllFitTotal:F0}m");
                    break;
                }
                var loopResult = TryFindLoop(switchId, arrivalSegId);
                if (!loopResult.HasValue)
                    continue;

                var (switchBId, branches) = loopResult.Value;

                // Build loop key from sorted switch IDs
                var switchIds = new List<string> { switchId, switchBId };
                switchIds.Sort();
                string loopKey = string.Join("|", switchIds);

                if (visitedLoopKeys != null && visitedLoopKeys.Contains(loopKey))
                {
                    Log($"Switch {switchId}: loop {loopKey} already visited — skipping");
                    continue;
                }

                Log($"Switch {switchId}: loop {loopKey} with {branches.Count} branches");

                // Check reversal clearance at both switches before evaluating branches.
                // Use full train length — during runaround the loco may keep some
                // cars, so the entire consist needs room to reverse at the switch.
                if (hasReversalClearance != null && trainLength > 0f)
                {
                    bool clearA = CheckReversalClearance(switchId, trainLength, hasReversalClearance);
                    bool clearB = CheckReversalClearance(switchBId, trainLength, hasReversalClearance);
                    if (!clearA || !clearB)
                    {
                        Log($"Switch {switchId}: reversal blocked at {(!clearA ? switchId : switchBId)} — skipping loop");
                        continue;
                    }
                }

                // Prefer mainline branches
                var sortedBranches = branches.OrderByDescending(b => IsMainlineBranch(b, switchId, switchBId)).ToList();

                // Count how many branches fit and find the best fitting branch
                int fittingBranchCount = 0;
                LoopInfo bestForThisLoop = null;
                float farDist = 0f;

                foreach (var branch in sortedBranches)
                {
                    var entryResult = DetermineEntry(switchId, switchBId, branch, canReachSwitch);
                    if (entryResult == null) continue;

                    // Check if branch segments are clear of non-train cars
                    if (isSegmentBlocked != null)
                    {
                        bool blocked = false;
                        foreach (var segId in branch.SegmentIds)
                        {
                            if (isSegmentBlocked(segId))
                            {
                                Log($"Switch {switchId}: branch segment {segId} blocked — skipping branch");
                                blocked = true;
                                break;
                            }
                        }
                        if (blocked) continue;
                    }

                    var (entrySwitchId, exitSwitchId) = entryResult.Value;
                    float foulingAtFar = (exitSwitchId == switchBId)
                        ? branch.FoulingAtEnd : branch.FoulingAtStart;
                    float fittingLength = branch.Length - foulingAtFar;

                    if (fittingLength < trainLength) continue;

                    fittingBranchCount++;

                    // Distance to the exit switch (the one the loco needs to reach for runaround)
                    float thisFarDist = distance + branch.Length;

                    if (bestForThisLoop == null)
                    {
                        bestForThisLoop = new LoopInfo(switchId, switchBId,
                            branches, branch, entrySwitchId, exitSwitchId);
                        farDist = thisFarDist;
                    }
                }

                if (bestForThisLoop != null)
                {
                    candidates.Add((bestForThisLoop, farDist, fittingBranchCount));

                    // Log candidate score
                    float circumference = 0f;
                    foreach (var br in bestForThisLoop.Branches) circumference += br.Length;
                    float nearDist = farDist - bestForThisLoop.FittingBranch.Length;
                    float total = nearDist + circumference;
                    Log($"Candidate: {switchId}↔{bestForThisLoop.SwitchBId}, nearDist={nearDist:F0}m, " +
                        $"circumference={circumference:F0}m, total={total:F0}m, fit={fittingBranchCount}/{bestForThisLoop.Branches.Count}");

                    // Track best all-fit total for early exit
                    if (fittingBranchCount >= bestForThisLoop.Branches.Count)
                    {
                        if (total < bestAllFitTotal)
                            bestAllFitTotal = total;
                    }
                }
            }

            if (candidates.Count == 0)
            {
                Log("No fitting loop found");
                return null;
            }

            // Sort: prefer all-branches-fit, then minimize total runaround travel
            // (distance to nearest switch + loop circumference).
            candidates.Sort((a, b) =>
            {
                // Prefer loops where ALL branches fit (better for runaround)
                bool aAllFit = a.fittingBranches >= a.loop.Branches.Count;
                bool bAllFit = b.fittingBranches >= b.loop.Branches.Count;
                if (aAllFit != bAllFit)
                    return bAllFit.CompareTo(aAllFit); // true first

                // Total travel = distance to near switch + loop circumference.
                // Near switch distance = farSwitchDist - fitting branch length.
                // Loop circumference = sum of all branch lengths.
                float aCircumference = 0f;
                foreach (var br in a.loop.Branches) aCircumference += br.Length;
                float aNearDist = a.farSwitchDist - a.loop.FittingBranch.Length;
                float aTotal = aNearDist + aCircumference;

                float bCircumference = 0f;
                foreach (var br in b.loop.Branches) bCircumference += br.Length;
                float bNearDist = b.farSwitchDist - b.loop.FittingBranch.Length;
                float bTotal = bNearDist + bCircumference;

                return aTotal.CompareTo(bTotal);
            });

            var best = candidates[0];
            Log($"Best loop: {best.loop.SwitchAId}↔{best.loop.SwitchBId}, " +
                $"farDist={best.farSwitchDist:F0}m, fittingBranches={best.fittingBranches}/{best.loop.Branches.Count}");

            return best.loop;
        }

        /// <summary>
        /// BFS from the loco's segment to find all switches within search range.
        /// Branches at junction switches — at each switch, ALL forward exits are explored.
        /// </summary>
        private List<(string nodeId, string arrivalSegId, float distance)>
            FindNearbySwitches(string startSegId, float distToA, float distToB)
        {
            var result = new List<(string, string, float)>();
            var visitedSegIds = new HashSet<string> { startSegId };

            Log($"BFS from segment {startSegId}, distToA={distToA:F1}m, distToB={distToB:F1}m");

            // Seed BFS from both ends of the loco's segment
            var queue = new Queue<(string segId, bool walkEndA, float dist)>();
            queue.Enqueue((startSegId, true, distToA));   // walk toward End.A
            queue.Enqueue((startSegId, false, distToB));  // walk toward End.B

            int iterations = 0;
            while (queue.Count > 0 && iterations < MaxInitialWalkSegments * 3)
            {
                iterations++;
                var (segId, walkEndA, dist) = queue.Dequeue();
                if (dist > MaxSearchDistance)
                    continue;

                var nodeId = _graph.GetNodeAtEnd(segId, walkEndA);
                if (nodeId == null)
                    continue;

                if (_graph.IsSwitch(nodeId))
                {
                    if (_visitedSwitches == null || !_visitedSwitches.Contains(nodeId))
                        result.Add((nodeId, segId, dist));

                    // Explore ALL exits from this switch (not just forward-traversal).
                    // Forward-traversal rules apply to FollowExit (loop path validation),
                    // NOT to the initial switch search. We need to find switches in ALL
                    // directions — e.g., a junction switch may have the loop on an exit
                    // that isn't reachable via forward traversal from the arrival direction.
                    var (enter, exitN, exitR) = _graph.GetSwitchLegs(nodeId);
                    var nextSegs = new List<string> { enter, exitN, exitR };

                    foreach (var nextId in nextSegs)
                    {
                        if (nextId == null || visitedSegIds.Contains(nextId))
                            continue;
                        visitedSegIds.Add(nextId);

                        var otherNodeId = _graph.GetOtherNode(nextId, nodeId);
                        if (otherNodeId == null)
                            continue;
                        bool nextWalkEndA = _graph.IsEndA(nextId, otherNodeId);
                        queue.Enqueue((nextId, nextWalkEndA, dist + _graph.GetLength(nextId)));
                    }
                }
                else
                {
                    // Non-switch node: use GetReachableSegment
                    var nextId = _graph.GetReachableSegment(segId, walkEndA);
                    if (nextId == null || visitedSegIds.Contains(nextId))
                        continue;
                    visitedSegIds.Add(nextId);

                    var otherNodeId = _graph.GetOtherNode(nextId, nodeId);
                    if (otherNodeId == null)
                        continue;
                    bool nextWalkEndA = _graph.IsEndA(nextId, otherNodeId);
                    queue.Enqueue((nextId, nextWalkEndA, dist + _graph.GetLength(nextId)));
                }
            }

            result.Sort((a, b) => a.Item3.CompareTo(b.Item3));
            Log($"BFS found {result.Count} switches within {MaxSearchDistance}m");
            return result;
        }

        /// <summary>
        /// BFS from one exit of a switch, following switch topology rules.
        /// Returns a dictionary of reachable switch IDs to the path data for reaching them.
        /// </summary>
        private Dictionary<string, ExitPath> FollowExit(
            string startSwitchId, string exitSegId)
        {
            var result = new Dictionary<string, ExitPath>();
            var visitedSwitchIds = new HashSet<string> { startSwitchId };

            // BFS queue: (currentSegId, walkEndA, depth, pathSegIds, totalDistance, intermediateSwitchIds)
            var queue = new Queue<(string segId, bool walkEndA, int depth,
                List<string> pathSegIds, float dist, List<string> intermediateIds)>();

            // Start: walk away from the startSwitch along the exitSegment
            var otherNodeId = _graph.GetOtherNode(exitSegId, startSwitchId);
            if (otherNodeId == null)
                return result;

            bool farEndIsA = _graph.IsEndA(exitSegId, otherNodeId);

            queue.Enqueue((exitSegId, farEndIsA, 1,
                new List<string> { exitSegId }, _graph.GetLength(exitSegId),
                new List<string>()));

            while (queue.Count > 0)
            {
                var (segId, walkEndA, depth, pathSegIds, dist, intermediateIds) = queue.Dequeue();
                if (depth > MaxExitWalkSegments)
                    continue;

                var nodeId = _graph.GetNodeAtEnd(segId, walkEndA);
                if (nodeId == null)
                    continue;

                if (_graph.IsSwitch(nodeId))
                {
                    if (visitedSwitchIds.Contains(nodeId))
                        continue;

                    visitedSwitchIds.Add(nodeId);

                    // Record this switch as reachable
                    result[nodeId] = new ExitPath
                    {
                        SwitchId = nodeId,
                        SegmentIds = new List<string>(pathSegIds),
                        Distance = dist,
                        IntermediateSwitchIds = new List<string>(intermediateIds),
                        ArrivalSegId = segId
                    };

                    // Determine traversal from this switch
                    var (enter, exitNormal, exitReverse) = _graph.GetSwitchLegs(nodeId);

                    var nextIntermediateIds = new List<string>(intermediateIds) { nodeId };

                    if (segId == enter)
                    {
                        // Arrived from enter -> explore both exits
                        EnqueueExit(queue, nodeId, exitNormal, depth, pathSegIds, dist, nextIntermediateIds);
                        EnqueueExit(queue, nodeId, exitReverse, depth, pathSegIds, dist, nextIntermediateIds);
                    }
                    else if (segId == exitNormal || segId == exitReverse)
                    {
                        // Arrived from an exit -> can only go to enter
                        EnqueueExit(queue, nodeId, enter, depth, pathSegIds, dist, nextIntermediateIds);
                    }
                    // exit->exit is invalid, skip
                }
                else
                {
                    // Non-switch node: use GetReachableSegment
                    var nextId = _graph.GetReachableSegment(segId, walkEndA);
                    if (nextId == null)
                        continue;

                    var newPath = new List<string>(pathSegIds) { nextId };
                    float newDist = dist + _graph.GetLength(nextId);

                    var nextOtherNodeId = _graph.GetOtherNode(nextId, nodeId);
                    if (nextOtherNodeId == null)
                        continue;

                    bool nextWalkEndA = _graph.IsEndA(nextId, nextOtherNodeId);
                    queue.Enqueue((nextId, nextWalkEndA, depth + 1, newPath, newDist, intermediateIds));
                }
            }

            return result;
        }

        /// <summary>
        /// Helper to enqueue a next segment during BFS from a switch.
        /// </summary>
        private void EnqueueExit(
            Queue<(string segId, bool walkEndA, int depth,
                List<string> pathSegIds, float dist, List<string> intermediateIds)> queue,
            string switchNodeId, string nextSegId, int currentDepth,
            List<string> currentPath, float currentDist,
            List<string> intermediateIds)
        {
            if (nextSegId == null)
                return;

            var otherNodeId = _graph.GetOtherNode(nextSegId, switchNodeId);
            if (otherNodeId == null)
                return;

            bool farEndIsA = _graph.IsEndA(nextSegId, otherNodeId);
            var newPath = new List<string>(currentPath) { nextSegId };
            float newDist = currentDist + _graph.GetLength(nextSegId);

            queue.Enqueue((nextSegId, farEndIsA, currentDepth + 1, newPath, newDist, intermediateIds));
        }

        /// <summary>
        /// Try to find a loop starting from a switch. Gets the switch's three legs,
        /// follows the two non-arrival exits via BFS, and looks for a common switch.
        /// </summary>
        private (string switchBId, List<LoopBranch> branches)? TryFindLoop(
            string switchAId, string arrivalSegId)
        {
            var (enter, exitNormal, exitReverse) = _graph.GetSwitchLegs(switchAId);

            Log($"  TryFindLoop({switchAId}): enter={enter}, exitN={exitNormal}, exitR={exitReverse}");

            // Collect all non-null legs
            var legs = new List<string>();
            if (enter != null) legs.Add(enter);
            if (exitNormal != null) legs.Add(exitNormal);
            if (exitReverse != null) legs.Add(exitReverse);

            if (legs.Count < 2)
                return null;

            // BFS from each leg — a loop exists between ANY pair that shares a common switch.
            // We must check all pairs, not just the two non-arrival legs, because the BFS
            // might arrive via one of the loop branches.
            var reachable = new Dictionary<string, Dictionary<string, ExitPath>>();
            foreach (var leg in legs)
            {
                reachable[leg] = FollowExit(switchAId, leg);
                Log($"  leg({leg}): reached {reachable[leg].Count} switches [{string.Join(",", reachable[leg].Keys)}]");
            }

            // Check all pairs of legs for common switches
            string bestSwitchBId = null;
            float bestDist = float.MaxValue;
            string bestLeg1 = null, bestLeg2 = null;

            for (int i = 0; i < legs.Count; i++)
            {
                for (int j = i + 1; j < legs.Count; j++)
                {
                    var r1 = reachable[legs[i]];
                    var r2 = reachable[legs[j]];

                    foreach (var id in r1.Keys)
                    {
                        if (r2.ContainsKey(id))
                        {
                            float combinedDist = r1[id].Distance + r2[id].Distance;
                            if (combinedDist < bestDist)
                            {
                                bestDist = combinedDist;
                                bestSwitchBId = id;
                                bestLeg1 = legs[i];
                                bestLeg2 = legs[j];
                            }
                        }
                    }
                }
            }

            if (bestSwitchBId == null)
            {
                Log($"  No common switches across any pair — not a loop");
                return null;
            }

            Log($"  Loop: {switchAId}↔{bestSwitchBId} via legs ({bestLeg1},{bestLeg2})");

            var path1 = reachable[bestLeg1][bestSwitchBId];
            var path2 = reachable[bestLeg2][bestSwitchBId];

            // Build branches
            var branch1 = BuildBranch(switchAId, bestSwitchBId, bestLeg1, path1);
            var branch2 = BuildBranch(switchAId, bestSwitchBId, bestLeg2, path2);

            var branches = new List<LoopBranch> { branch1, branch2 };
            return (bestSwitchBId, branches);
        }

        /// <summary>
        /// Build a LoopBranch from the path data.
        /// </summary>
        private LoopBranch BuildBranch(string switchAId, string switchBId,
            string startLegSegId, ExitPath pathData)
        {
            float foulingAtStart = CalculateSwitchFouling(switchAId, startLegSegId);
            float foulingAtEnd = CalculateSwitchFouling(switchBId, pathData.ArrivalSegId);

            return new LoopBranch(
                pathData.SegmentIds,
                pathData.Distance,
                foulingAtStart,
                foulingAtEnd,
                pathData.IntermediateSwitchIds,
                startLegSegId,
                pathData.ArrivalSegId);
        }

        /// <summary>
        /// Calculate fouling distance at a switch for a given branch leg.
        /// If the branch connects to the enter (stem) side, fouling is minimal (2m).
        /// If the branch connects to an exit side, use the full fouling distance.
        /// </summary>
        private float CalculateSwitchFouling(string switchId, string branchLegSegId)
        {
            var (enter, _, _) = _graph.GetSwitchLegs(switchId);

            if (branchLegSegId == enter)
            {
                // Stem side — no fouling concern, just a small safety margin
                return 2f;
            }

            // Exit side — full fouling distance
            return _graph.GetFoulingDistance(switchId);
        }

        /// <summary>
        /// Determine which switch is the entry and which is the far switch.
        /// Tests routability to each switch via the canReachSwitch callback.
        /// </summary>
        private (string entrySwitchId, string exitSwitchId)? DetermineEntry(
            string switchAId, string switchBId, LoopBranch branch,
            Func<string, bool, bool> canReachSwitch)
        {
            // Test if loco can reach switchA via the branch's start leg
            bool canReachA = TestReachSwitch(switchAId, branch.StartLegSegId, canReachSwitch);
            // Test if loco can reach switchB via the branch's end leg
            bool canReachB = TestReachSwitch(switchBId, branch.EndLegSegId, canReachSwitch);

            if (canReachA && !canReachB)
                return (switchAId, switchBId);
            else if (canReachB && !canReachA)
                return (switchBId, switchAId);
            else if (canReachA && canReachB)
                // Both reachable — pick switchA as entry (it's closer to loco along the initial walk)
                return (switchAId, switchBId);
            else
                // Neither switch is routable
                return null;
        }

        /// <summary>
        /// Test if the loco can reach a switch via the given leg segment.
        /// The leg segment connects to the switch at one end; we test routability
        /// to a point on that segment near the switch end.
        /// </summary>
        private bool TestReachSwitch(string switchId, string legSegId,
            Func<string, bool, bool> canReachSwitch)
        {
            // Determine which end of the leg segment is at the switch
            bool switchIsAtEndA = _graph.IsEndA(legSegId, switchId);
            // canReachSwitch wants (segId, endA) where endA indicates which end to approach
            return canReachSwitch(legSegId, switchIsAtEndA);
        }

        /// <summary>
        /// Check if the stem (enter leg) at a switch has enough clear space for the
        /// train to pull out and reverse. Required clearance = stemFouling + trainLength.
        /// Uses stem-side fouling (2m), not exit-leg fouling which is much larger.
        /// </summary>
        private bool CheckReversalClearance(string switchId, float locoLength,
            Func<string, string, float, bool> hasReversalClearance)
        {
            var (enter, _, _) = _graph.GetSwitchLegs(switchId);
            if (enter == null) return false;
            float fouling = CalculateSwitchFouling(switchId, enter); // stem = 2m
            float required = fouling + locoLength;
            return hasReversalClearance(switchId, enter, required);
        }

        /// <summary>
        /// Check if a branch uses the exitNormal (straight/mainline) leg at both
        /// of its switches. The mainline branch goes straight through; the siding
        /// branch diverges via exitReverse.
        /// </summary>
        private bool IsMainlineBranch(LoopBranch branch, string switchAId, string switchBId)
        {
            var (_, exitNormalA, _) = _graph.GetSwitchLegs(switchAId);
            var (_, exitNormalB, _) = _graph.GetSwitchLegs(switchBId);

            // The branch connects to switchA via StartLegSegId and to switchB via EndLegSegId.
            // If both legs are the exitNormal of their respective switches, it's the mainline.
            bool normalAtA = (branch.StartLegSegId == exitNormalA);
            bool normalAtB = (branch.EndLegSegId == exitNormalB);

            return normalAtA || normalAtB; // prefer if at least one end is the normal exit
        }
    }
}
