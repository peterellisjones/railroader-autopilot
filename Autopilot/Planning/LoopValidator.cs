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
            IEnumerable<string>? visitedLoopKeys = null)
        {
            var adapter = new GameGraphAdapter();
            var loop = FindFittingLoop(loco, adapter, visitedSwitches, visitedLoopKeys);

            if (loop == null)
            {
                _logger.Log("LoopValidator", "GetRepositionLocation: no fitting loop found");
                return (null, null);
            }

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

                        if (!_routeChecker.CanRouteTo(loco, waypointPos))
                        {
                            _logger.Log("LoopValidator", $"GetRepositionLocation: can't route to waypoint near {candidate.SwitchId} on branch [{branch.SegmentIds[0]}...]");
                            continue;
                        }

                        float routeDist = _routeChecker.GraphDistanceToLoco(loco, waypointPos)?.Distance ?? float.MaxValue;
                        _logger.Log("LoopValidator", $"GetRepositionLocation: candidate switch={candidate.SwitchId}, " +
                            $"branch=[{branch.SegmentIds[0]}...{branch.SegmentIds[branch.SegmentIds.Count-1]}] len={branch.Length:F0}m, " +
                            $"waypointDist={waypointDist:F0}m, routeDist={routeDist:F0}m, seg={waypointPos.Segment?.id}");

                        if (routeDist < bestRouteDist)
                        {
                            bestWaypoint = waypointPos;
                            bestRouteDist = routeDist;
                            bestSwitchId = candidate.SwitchId;
                            bestWaypointDist = waypointDist;
                            bestFouling = candidate.FoulingNear;
                            bestBranchDesc = $"[{branch.SegmentIds[0]}...] len={branch.Length:F0}m";
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
                _logger.Log("LoopValidator", $"GetRepositionLocation: loop {loop.SwitchAId}↔{loop.SwitchBId}, " +
                    $"waypointSwitch={bestSwitchId}, branch={bestBranchDesc}, " +
                    $"trainLen={trainLength:F0}m, fouling={bestFouling:F1}m, " +
                    $"waypointDist={bestWaypointDist:F0}m, routeDist={bestRouteDist:F0}m, " +
                    $"waypoint on {bestWaypoint.Segment?.id}");
                return (bestWaypoint, loop.LoopKey);
            }

            _logger.Log("LoopValidator", "GetRepositionLocation: couldn't create waypoint at either switch");
            return (null, null);
        }
    }
}
