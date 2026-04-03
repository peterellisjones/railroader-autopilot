using System;
using System.Collections.Generic;
using Model;
using Model.Definition;
using Track;
using Autopilot.Model;
using Autopilot.Services;
using Autopilot.TrackGraph;

namespace Autopilot.Planning
{
    public class LoopValidator
    {
        private readonly RouteChecker _routeChecker;
        private readonly TrainService _trainService;
        private readonly PlanningLogger _logger;

        public LoopValidator(RouteChecker routeChecker, TrainService trainService, PlanningLogger logger)
        {
            _routeChecker = routeChecker;
            _trainService = trainService;
            _logger = logger;
        }

        /// <summary>
        /// Find a fitting loop from the loco's current position with all safety checks:
        /// blocked branches, reversal clearance, routability.
        /// </summary>
        public LoopInfo? FindFittingLoop(BaseLocomotive loco, GameGraphAdapter adapter,
            IEnumerable<string>? visitedSwitches = null, IEnumerable<string>? visitedLoopKeys = null)
        {
            var locoLoc = loco.LocationF;
            if (locoLoc.segment == null) return null;

            float trainLength = _trainService.GetTrainLength(loco);
            adapter.RegisterSegment(locoLoc.segment);

            var finder = new LoopFinder(adapter, msg => _logger.Log("LoopValidator", msg));

            Func<string, bool, bool> canReachSwitch = (segId, endA) =>
            {
                var seg = adapter.GetSegment(segId);
                if (seg == null) return false;
                var end = endA ? TrackSegment.End.A : TrackSegment.End.B;
                try
                {
                    var atSwitch = new Location(seg, 0f, end);
                    var testLoc = Graph.Shared.LocationByMoving(atSwitch, 0.1f);
                    var testPos = DirectedPosition.FromLocation(testLoc);
                    return _routeChecker.CanRouteTo(loco, testPos);
                }
                catch { return false; }
            };

            var trainCars = new HashSet<Car>(_trainService.GetCoupled(loco));
            var nonTrainCars = new List<Car>();
            foreach (var car in TrainController.Shared.Cars)
            {
                if (!trainCars.Contains(car) && car.LocationA.segment != null)
                    nonTrainCars.Add(car);
            }

            var occupiedSegIds = new HashSet<string>();
            foreach (var car in nonTrainCars)
            {
                if (car.LocationA.segment != null)
                {
                    occupiedSegIds.Add(car.LocationA.segment.id);
                    adapter.RegisterSegment(car.LocationA.segment);
                }
                if (car.LocationB.segment != null)
                {
                    occupiedSegIds.Add(car.LocationB.segment.id);
                    adapter.RegisterSegment(car.LocationB.segment);
                }
            }

            Func<string, bool> isSegmentBlocked = segId => occupiedSegIds.Contains(segId);

            float locoLength = 0f;
            int locoCarCount = 0;
            foreach (var car in _trainService.GetCoupled(loco))
            {
                var arch = car.Archetype;
                if (arch == CarArchetype.LocomotiveDiesel
                    || arch == CarArchetype.LocomotiveSteam
                    || arch == CarArchetype.Tender)
                {
                    locoLength += car.carLength;
                    locoCarCount++;
                }
            }
            if (locoCarCount > 1)
                locoLength += (locoCarCount - 1) * AutopilotConstants.ConsistGapPerCar;

            Func<string, string, float, bool> hasReversalClearance = (switchId, stemSegId, requiredLength) =>
            {
                var seg = adapter.GetSegment(stemSegId);
                var node = adapter.GetNode(switchId);
                if (seg == null || node == null) return false;

                // DFS to find the maximum reachable distance from the loop switch
                // along the stem direction. At switches, explores all branches and
                // takes the best. Stops at dead ends, blocked segments, or depth limit.
                float availableSpace = ReversalClearanceChecker.WalkMaxDistance(seg, node, requiredLength, occupiedSegIds);

                if (availableSpace < requiredLength)
                {
                    _logger.Log("LoopValidator", $"Reversal blocked: stem at switch {switchId} too short ({availableSpace:F1}m < {requiredLength:F1}m)");
                    return false;
                }

                var switchEnd = seg.EndForNode(node);
                float stemLen = seg.GetLength();
                foreach (var car in nonTrainCars)
                {
                    foreach (var loc in new[] { car.LocationA, car.LocationB })
                    {
                        if (loc.segment != seg) continue;
                        float distFromSwitch;
                        if (switchEnd == TrackSegment.End.A)
                            distFromSwitch = loc.EndIsA ? loc.distance : stemLen - loc.distance;
                        else
                            distFromSwitch = loc.EndIsA ? stemLen - loc.distance : loc.distance;

                        if (distFromSwitch < requiredLength)
                        {
                            _logger.Log("LoopValidator", $"Reversal blocked: {car.DisplayName} on stem {stemSegId} at {distFromSwitch:F1}m from switch {switchId} (need {requiredLength:F1}m)");
                            return false;
                        }
                    }
                }
                return true;
            };

            float distToEnd = locoLoc.DistanceUntilEnd();
            float distToOther = locoLoc.segment.GetLength() - distToEnd;
            float distToA = locoLoc.EndIsA ? distToOther : distToEnd;
            float distToB = locoLoc.EndIsA ? distToEnd : distToOther;

            return finder.FindNearestFittingLoop(
                locoLoc.segment.id, distToA, distToB,
                trainLength, visitedSwitches, canReachSwitch,
                visitedLoopKeys,
                isSegmentBlocked: isSegmentBlocked,
                hasReversalClearance: hasReversalClearance,
                locoLength: locoLength);
        }

        /// <summary>
        /// Check if the loco is currently on a clear, fitting loop suitable for
        /// runarounds or splits. Reuses the full LoopFinder with blocked-branch
        /// and reversal-clearance checks. Verifies the returned loop is the one
        /// the train is actually on (not a distant clear loop).
        /// </summary>
        public bool IsOnClearLoop(BaseLocomotive loco)
        {
            var adapter = new GameGraphAdapter();
            var loop = FindFittingLoop(loco, adapter);
            if (loop == null) return false;

            // Verify the train is ON this loop, not a distant one.
            // Check if the loco's segment is part of the loop's branches
            // or the stem (enter leg) of either switch.
            var locoSegId = loco.LocationF.segment?.id;
            if (locoSegId == null) return false;

            foreach (var branch in loop.Branches)
            {
                if (branch.SegmentIds.Contains(locoSegId))
                    return true;
            }

            // Also check stem segments at each switch
            var (enterA, _, _) = adapter.GetSwitchLegs(loop.SwitchAId);
            var (enterB, _, _) = adapter.GetSwitchLegs(loop.SwitchBId);
            if (locoSegId == enterA || locoSegId == enterB)
                return true;

            _logger.Log("LoopValidator", $"Runaround feasibility: found loop {loop.SwitchAId}↔{loop.SwitchBId} but train is not on it (seg={locoSegId})");
            return false;
        }

        public (DirectedPosition? location, string? loopKey) GetRepositionLocation(
            BaseLocomotive loco, IEnumerable<string>? visitedSwitches,
            IEnumerable<string>? visitedLoopKeys = null,
            List<DirectedPosition>? deliveryDestinations = null)
        {
            // Try multiple loops and pick the one with the shortest route distance.
            // The loop finder ranks by BFS graph distance, but a loop that's close
            // by graph distance might be far by actual route (or enter mid-branch).
            var adapter = new GameGraphAdapter();
            var triedLoopKeys = new HashSet<string>(visitedLoopKeys ?? Array.Empty<string>());
            var allResults = new List<(DirectedPosition waypoint, float routeDist, string loopKey, string desc)>();

            const int MaxLoopAttempts = 5;
            for (int attempt = 0; attempt < MaxLoopAttempts; attempt++)
            {
                var loop = FindFittingLoop(loco, adapter, visitedSwitches, triedLoopKeys);
                if (loop == null) break;

                var result = EvaluateLoopForReposition(loco, loop, adapter, deliveryDestinations);
                if (result.HasValue)
                    allResults.Add(result.Value);

                triedLoopKeys.Add(loop.LoopKey);
            }

            if (allResults.Count == 0)
            {
                _logger.Log("LoopValidator", "GetRepositionLocation: no fitting loop found");
                return (null, null);
            }

            // Pick the closest by route distance
            allResults.Sort((a, b) => a.routeDist.CompareTo(b.routeDist));
            var best = allResults[0];
            _logger.Log("LoopValidator", $"GetRepositionLocation: best of {allResults.Count} loop(s): {best.desc}, routeDist={best.routeDist:F0}m");
            return (best.waypoint, best.loopKey);
        }

        private (DirectedPosition waypoint, float routeDist, string loopKey, string desc)?
            EvaluateLoopForReposition(BaseLocomotive loco, LoopInfo loop, GameGraphAdapter adapter,
            List<DirectedPosition>? deliveryDestinations = null)
        {
            float trainLength = _trainService.GetTrainLength(loco);

            // Try waypoints on ALL fitting branches (not just FittingBranch).
            // For a runaround, the train parks on the siding — which might be
            // the shorter branch, not the mainline. Try all branches and pick
            // the one with the closest reachable waypoint.
            DirectedPosition bestWaypoint = default;
            float bestRouteDist = float.MaxValue;
            string? bestSwitchId = null;
            float bestWaypointDist = 0f;
            float bestFouling = 0f;
            string? bestBranchDesc = null;

            foreach (var branch in loop.Branches)
            {
                // Check if the train fits on this branch (same logic as LoopFinder)
                float fittingLength = branch.Length - System.Math.Min(branch.FoulingAtStart, branch.FoulingAtEnd);
                if (fittingLength < trainLength) continue;

                // Entry candidates: both endpoint switches
                var candidates = new[]
                {
                    new { SwitchId = loop.SwitchAId, SegId = branch.StartLegSegId,
                          FoulingNear = branch.FoulingAtStart, FoulingFar = branch.FoulingAtEnd },
                    new { SwitchId = loop.SwitchBId, SegId = branch.EndLegSegId,
                          FoulingNear = branch.FoulingAtEnd, FoulingFar = branch.FoulingAtStart }
                };

                // Build intermediate switch positions for gap optimization
                var intermediateSwitchPositions = new List<(float position, float fouling)>();
                {
                    var intSwitchIds = new HashSet<string>(branch.IntermediateSwitchIds ?? new List<string>());
                    var visited = new HashSet<string>();
                    float cumDist = 0f;
                    foreach (var segId in branch.SegmentIds)
                    {
                        float segLen = adapter.GetLength(segId);
                        foreach (var endA in new[] { true, false })
                        {
                            string nodeId = adapter.GetNodeAtEnd(segId, endA);
                            if (nodeId == null || !intSwitchIds.Contains(nodeId) || visited.Contains(nodeId))
                                continue;
                            visited.Add(nodeId);

                            float position = endA ? cumDist : cumDist + segLen;

                            var (enter, exitN, exitR) = adapter.GetSwitchLegs(nodeId);
                            bool isEnterLeg = (segId == enter);
                            float fouling = isEnterLeg ? 2f : adapter.GetFoulingDistance(nodeId);

                            intermediateSwitchPositions.Add((position, fouling));
                        }
                        cumDist += segLen;
                    }
                }

                foreach (var candidate in candidates)
                {
                    var seg = adapter.GetSegment(candidate.SegId);
                    var node = adapter.GetNode(candidate.SwitchId);
                    if (seg == null || node == null) continue;

                    float? gapPosition = Services.TrackWalker.FindBestGapPosition(
                        branch.Length, candidate.FoulingNear, candidate.FoulingFar,
                        intermediateSwitchPositions, trainLength);

                    float maxDist = branch.Length - candidate.FoulingFar;
                    float waypointDist = gapPosition.HasValue
                        ? gapPosition.Value
                        : System.Math.Min(trainLength + candidate.FoulingNear, maxDist);

                    if (waypointDist < candidate.FoulingNear)
                        continue;

                    try
                    {
                        var end = seg.EndForNode(node);
                        var atSwitch = new Location(seg, 0f, end);
                        var waypointLoc = Graph.Shared.LocationByMoving(atSwitch, waypointDist);
                        var waypointPos = DirectedPosition.FromLocation(waypointLoc);

                        // Get route with steps to detect mid-branch entry
                        var coupled = _trainService.GetCoupled(loco);
                        var (routeFound, routeSteps) = _routeChecker.CanRouteToWithSteps(
                            loco, waypointPos, trainLength, coupled);
                        if (!routeFound)
                        {
                            _logger.Log("LoopValidator", $"GetRepositionLocation: can't route to waypoint near {candidate.SwitchId} on branch [{branch.SegmentIds[0]}...]");
                            continue;
                        }

                        // Check if route enters through a loop switch (Strategy 1)
                        // or mid-branch via an intermediate junction (Strategy 2/3)
                        bool entersViaLoopSwitch = false;
                        foreach (var step in routeSteps)
                        {
                            if (step.Node != null &&
                                (step.Node.id == loop.SwitchAId || step.Node.id == loop.SwitchBId))
                            {
                                entersViaLoopSwitch = true;
                                break;
                            }
                        }

                        string strategy;
                        if (entersViaLoopSwitch)
                        {
                            // Strategy 1: via loop switch — waypoint is already correct
                            strategy = "ViaLoopSwitch";
                        }
                        else
                        {
                            // Mid-branch entry — find the junction point (intermediate
                            // switch where route enters the branch)
                            var branchSegSet = new HashSet<string>(branch.SegmentIds);
                            var intSwitchSet = new HashSet<string>(branch.IntermediateSwitchIds ?? new List<string>());
                            string? junctionId = null;
                            float junctionDistFromStart = 0f;

                            foreach (var step in routeSteps)
                            {
                                if (step.Node != null && intSwitchSet.Contains(step.Node.id))
                                {
                                    junctionId = step.Node.id;
                                    break;
                                }
                            }

                            if (junctionId != null)
                            {
                                // Find junction position along the branch
                                foreach (var isp in intermediateSwitchPositions)
                                {
                                    // Match by checking if this position corresponds to our junction
                                    var visited2 = new HashSet<string>();
                                    float cumDist2 = 0f;
                                    foreach (var segId2 in branch.SegmentIds)
                                    {
                                        float segLen2 = adapter.GetLength(segId2);
                                        foreach (var endA2 in new[] { true, false })
                                        {
                                            string nodeId2 = adapter.GetNodeAtEnd(segId2, endA2);
                                            if (nodeId2 == junctionId && !visited2.Contains(nodeId2))
                                            {
                                                junctionDistFromStart = endA2 ? cumDist2 : cumDist2 + segLen2;
                                                visited2.Add(nodeId2);
                                            }
                                        }
                                        cumDist2 += segLen2;
                                    }
                                }
                            }

                            // Distance from junction to each loop switch along the branch
                            float junctionToStart = junctionDistFromStart; // toward SwitchA
                            float junctionToEnd = branch.Length - junctionDistFromStart; // toward SwitchB

                            // Distance from junction to the candidate's switch
                            bool candidateIsStart = candidate.SwitchId == loop.SwitchAId;
                            float junctionToCandidate = candidateIsStart ? junctionToStart : junctionToEnd;
                            float junctionToOther = candidateIsStart ? junctionToEnd : junctionToStart;

                            if (junctionToCandidate >= trainLength)
                            {
                                // Strategy 2: mid-branch direct — train fits between
                                // junction and this switch. Waypoint measured from the
                                // switch: place the front so the tail just clears the
                                // junction (front = junction - trainLength from switch).
                                float foulingAtJunction = 2f; // small clearance past junction
                                waypointDist = junctionToCandidate - trainLength - foulingAtJunction;
                                if (waypointDist < candidate.FoulingNear)
                                    waypointDist = candidate.FoulingNear;
                                waypointLoc = Graph.Shared.LocationByMoving(atSwitch, waypointDist);
                                waypointPos = DirectedPosition.FromLocation(waypointLoc);
                                strategy = "MidBranchDirect";
                                _logger.Log("LoopValidator", $"GetRepositionLocation: mid-branch via junction {junctionId}, " +
                                    $"junctionToSwitch={junctionToCandidate:F0}m, trainLen={trainLength:F0}m, " +
                                    $"waypointFromSwitch={waypointDist:F0}m — fits (Strategy 2)");
                            }
                            else if (junctionToOther >= trainLength)
                            {
                                // Space toward the other switch — skip this candidate,
                                // the other candidate for the same branch will handle it
                                _logger.Log("LoopValidator", $"GetRepositionLocation: mid-branch via junction {junctionId}, " +
                                    $"junctionToSwitch={junctionToCandidate:F0}m < trainLen={trainLength:F0}m — " +
                                    $"but {junctionToOther:F0}m toward other switch, skipping");
                                continue;
                            }
                            else
                            {
                                // Strategy 3: pull-through — train doesn't fit between
                                // junction and either switch. Set waypoint past the closer
                                // switch on the stem so the train pulls fully through.
                                // After reaching it, replan will enter the branch via the switch.
                                float overshoot = trainLength - junctionToCandidate + candidate.FoulingNear;
                                var pullThroughLoc = Graph.Shared.LocationByMoving(atSwitch, -overshoot, false, true);
                                waypointPos = DirectedPosition.FromLocation(pullThroughLoc);

                                if (!_routeChecker.CanRouteTo(loco, waypointPos))
                                {
                                    _logger.Log("LoopValidator", $"GetRepositionLocation: pull-through past {candidate.SwitchId} not routable — skipping");
                                    continue;
                                }

                                strategy = "PullThrough";
                                waypointDist = overshoot;
                                _logger.Log("LoopValidator", $"GetRepositionLocation: mid-branch via junction {junctionId}, " +
                                    $"neither direction fits (toCandidate={junctionToCandidate:F0}m, toOther={junctionToOther:F0}m, " +
                                    $"trainLen={trainLength:F0}m) — pull-through {overshoot:F0}m past {candidate.SwitchId} (Strategy 3)");
                            }
                        }

                        // Verify that the runaround approach works from this position.
                        // CheckApproachDirection routes from the TAIL's outward end,
                        // not the front (waypoint). Compute the tail position by
                        // walking trainLength back from the waypoint along the branch.
                        if (deliveryDestinations != null && deliveryDestinations.Count > 0)
                        {
                            // Tail position = trainLength behind the front (waypoint)
                            var wpLoc = waypointPos.ToLocation();
                            Location tailLoc;
                            try
                            {
                                tailLoc = Graph.Shared.LocationByMoving(
                                    wpLoc.Flipped(), trainLength, false, true).Flipped();
                            }
                            catch
                            {
                                tailLoc = wpLoc; // fallback
                            }

                            bool anyDestReachable = false;
                            foreach (var dest in deliveryDestinations)
                            {
                                var destLoc = dest.ToLocation();
                                if (tailLoc.segment == null || destLoc.segment == null)
                                    continue;

                                // Try both directions from the tail (same as ApproachAnalyzer)
                                var stepsA = new List<Track.Search.RouteSearch.Step>();
                                bool foundA = Track.Search.RouteSearch.FindRoute(
                                    Graph.Shared, tailLoc, destLoc,
                                    RouteChecker.DefaultHeuristic, stepsA,
                                    out Track.Search.RouteSearch.Metrics metricsA,
                                    checkForCars: false, trainLength: 0f,
                                    maxIterations: 5000, enableLogging: false);

                                var stepsB = new List<Track.Search.RouteSearch.Step>();
                                bool foundB = Track.Search.RouteSearch.FindRoute(
                                    Graph.Shared, tailLoc.Flipped(), destLoc,
                                    RouteChecker.DefaultHeuristic, stepsB,
                                    out Track.Search.RouteSearch.Metrics metricsB,
                                    checkForCars: false, trainLength: 0f,
                                    maxIterations: 5000, enableLogging: false);

                                // Pick the shorter route (same as ApproachAnalyzer)
                                bool useA = foundA && (!foundB || metricsA.Distance <= metricsB.Distance);
                                var bestSteps = useA ? stepsA : stepsB;
                                bool found = useA ? foundA : foundB;
                                if (!found) continue;

                                int destReversals = ReversalCounter.FromSteps(bestSteps);
                                if (destReversals % 2 == 0)
                                {
                                    anyDestReachable = true;
                                    break;
                                }
                                else
                                {
                                    _logger.Log("LoopValidator", $"GetRepositionLocation: tail→{dest.Segment?.id} has {destReversals} reversal(s) (odd) — runaround won't help");
                                }
                            }

                            if (!anyDestReachable)
                            {
                                _logger.Log("LoopValidator", $"GetRepositionLocation: no delivery destination reachable with even reversals from {candidate.SwitchId} — skipping");
                                continue;
                            }
                        }

                        float routeDist = _routeChecker.GraphDistanceToLoco(loco, waypointPos)?.Distance ?? float.MaxValue;
                        _logger.Log("LoopValidator", $"GetRepositionLocation: candidate switch={candidate.SwitchId}, " +
                            $"branch=[{branch.SegmentIds[0]}...{branch.SegmentIds[branch.SegmentIds.Count-1]}] len={branch.Length:F0}m, " +
                            $"waypointDist={waypointDist:F0}m, routeDist={routeDist:F0}m, seg={waypointPos.Segment?.id}, strategy={strategy}");

                        if (routeDist < bestRouteDist)
                        {
                            bestWaypoint = waypointPos;
                            bestRouteDist = routeDist;
                            bestSwitchId = candidate.SwitchId;
                            bestWaypointDist = waypointDist;
                            bestFouling = candidate.FoulingNear;
                            bestBranchDesc = $"[{branch.SegmentIds[0]}...] len={branch.Length:F0}m, {strategy}";
                        }
                    }
                    catch (Exception ex)
                    {
                        _logger.Log("LoopValidator", $"GetRepositionLocation: waypoint near {candidate.SwitchId} failed: {ex.Message}");
                    }
                }
            }

            if (bestSwitchId != null)
            {
                var desc = $"loop {loop.SwitchAId}↔{loop.SwitchBId}, " +
                    $"waypointSwitch={bestSwitchId}, branch={bestBranchDesc}, " +
                    $"trainLen={trainLength:F0}m, fouling={bestFouling:F1}m, " +
                    $"waypointDist={bestWaypointDist:F0}m";
                _logger.Log("LoopValidator", $"GetRepositionLocation: {desc}, routeDist={bestRouteDist:F0}m, waypoint on {bestWaypoint.Segment?.id}");
                return (bestWaypoint, bestRouteDist, loop.LoopKey, desc);
            }

            _logger.Log("LoopValidator", $"GetRepositionLocation: loop {loop.SwitchAId}↔{loop.SwitchBId} — couldn't create waypoint at either switch");
            return null;
        }

    }
}
