using System;
using System.Collections.Generic;
using System.Linq;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Services;
using Autopilot.TrackGraph;

namespace Autopilot.Planning
{
    public class SplitFinder
    {
        private readonly TrainService _trainService;
        private readonly FeasibilityChecker _checker;
        private readonly DestinationSelector _destinationSelector;

        // Testable planning fields (used by interface-based constructor path)
        private readonly ITrainService? _iTrainService;
        private readonly IGraphAdapter? _iGraph;
        private readonly FeasibilityChecker? _iChecker;

        public SplitFinder(TrainService trainService, FeasibilityChecker checker, DestinationSelector destinationSelector)
        {
            _trainService = trainService;
            _checker = checker;
            _destinationSelector = destinationSelector;
        }

        /// <summary>Constructor for testable planning.</summary>
        public SplitFinder(ITrainService trainService, IGraphAdapter graph, FeasibilityChecker checker)
        {
            _iTrainService = trainService;
            _iGraph = graph;
            _iChecker = checker;
            _trainService = null!;
            _checker = null!;
            _destinationSelector = null!;
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot SplitFinder: {msg}");
        private void LogDebug(string msg) { if (Loader.Settings?.verboseLogging == true) Log(msg); }

        public SplitInfo? FindBestSplit(BaseLocomotive loco, CarGroup group, IEnumerable<string>? skippedCarIds = null)
        {
            if (group.IsEmpty || group.Count < 2)
                return null;

            var cars = group.Cars;
            var graph = Graph.Shared;

            var destGroups = GroupByDestination(cars, loco, skippedCarIds);
            if (destGroups.Count < 2)
                return null;

            Log($"Found {destGroups.Count} destination groups");

            for (int keepCount = 1; keepCount < destGroups.Count; keepCount++)
            {
                var tailGroup = destGroups[keepCount - 1];
                var destLoc = tailGroup.destLocation;
                if (destLoc.Segment == null) continue;

                var keptCars = new List<Car>();
                for (int g = 0; g < keepCount; g++)
                    keptCars.AddRange(destGroups[g].cars);

                var droppedCars = new List<Car>();
                for (int g = keepCount; g < destGroups.Count; g++)
                    droppedCars.AddRange(destGroups[g].cars);

                var shorterConsist = new List<Car> { (Car)loco };
                shorterConsist.AddRange(keptCars);
                float shorterLength = _trainService.GetConsistLength(shorterConsist);

                var blockedSwitches = ComputeBlockedSwitches(droppedCars, keptCars, loco, group);

                LogDebug($"Split at {keepCount} groups: kept={keptCars.Count}, dropped={droppedCars.Count}, " +
                    $"trainLen={shorterLength:F0}m, blockedSwitches={blockedSwitches.Count}");

                var (found, steps) = _checker.CanRouteToWithSteps(
                    loco, destLoc, shorterLength,
                    shorterConsist);

                if (!found)
                {
                    LogDebug($"  Tail group {tailGroup.destName}: route not found");
                    continue;
                }

                if (!IsRouteSafe(steps, blockedSwitches))
                {
                    foreach (var step in steps)
                    {
                        if (step.Node != null && blockedSwitches.Contains(step.Node.id))
                            LogDebug($"  Route hits blocked switch {step.Node.id}");
                    }
                    LogDebug($"  Tail group {tailGroup.destName}: route goes through blocked switches");
                    LogDebug($"  Blocked set: [{string.Join(",", blockedSwitches)}]");
                    continue;
                }

                var approachTarget = tailGroup.approachTarget;
                if (!_checker.ApproachAnalyzer.CheckApproachDirection(loco, group, approachTarget))
                {
                    LogDebug($"  Tail group {tailGroup.destName}: approach wrong (tail doesn't lead)");
                    continue;
                }

                LogDebug($"  Tail group {tailGroup.destName} is deliverable ({tailGroup.cars.Count} cars)");
                var split = BuildSplitInfo(loco, keptCars, droppedCars, group, graph);
                Log($"Split found: keep {keptCars.Count} cars (tail={tailGroup.destName}), drop {droppedCars.Count}");
                return split;
            }

            Log("No valid split found");
            return null;
        }

        // =================================================================
        // Testable overload using ITrainService + IGraphAdapter
        // =================================================================

        /// <summary>
        /// Find the best split point using abstract interfaces (no game types).
        /// Groups cars by destination and tries each cut point, checking feasibility
        /// via the abstract FeasibilityChecker.CanDeliver overload.
        /// </summary>
        public SplitInfo? FindBestSplit(CarGroup group, IReadOnlyCollection<string>? skippedCarIds = null)
        {
            if (_iTrainService == null || _iChecker == null)
                return null;

            if (group.IsEmpty || group.Count < 2)
                return null;

            var cars = group.Cars;

            var destGroups = GroupByDestinationAbstract(cars, skippedCarIds);
            if (destGroups.Count < 2)
                return null;

            for (int keepCount = 1; keepCount < destGroups.Count; keepCount++)
            {
                var tailGroup = destGroups[keepCount - 1];

                if (tailGroup.destLocation.SegmentId == null) continue;

                // Check feasibility via the abstract CanDeliver overload
                if (!group.TailOutwardEnd.HasValue || !group.TailInwardEnd.HasValue)
                    continue;

                if (!_iChecker.CanDeliver(group.TailOutwardEnd.Value,
                        group.TailInwardEnd.Value, tailGroup.approachTarget))
                    continue;

                // Build the split info from abstract types
                var keptCars = new List<ICar>();
                for (int g = 0; g < keepCount; g++)
                    keptCars.AddRange(destGroups[g].cars);

                var droppedCars = new List<ICar>();
                for (int g = keepCount; g < destGroups.Count; g++)
                    droppedCars.AddRange(destGroups[g].cars);

                var splitCar = keptCars[keptCars.Count - 1];
                var firstDropped = droppedCars[0];

                // Determine split end: which end of splitCar faces the first dropped car
                var splitEnd = splitCar.CoupledTo(Car.LogicalEnd.A)?.id == firstDropped.id
                    ? Car.LogicalEnd.A : Car.LogicalEnd.B;

                // Couple target is the first dropped car
                var coupleTarget = firstDropped;
                var freeEnd = coupleTarget.CoupledTo(Car.LogicalEnd.A) != null
                    ? Car.LogicalEnd.B : Car.LogicalEnd.A;

                // Get the couple target's position for the waypoint
                var coupleTargetPos = freeEnd == Car.LogicalEnd.A
                    ? coupleTarget.EndA : coupleTarget.EndB;
                var dropGp = new GraphPosition(coupleTargetPos.Segment?.id,
                    coupleTargetPos.DistanceFromA, coupleTargetPos.Facing);
                var graphCoupleWp = new GraphCoupleWaypoint(
                    coupleTargetPos.Segment?.id, coupleTargetPos.DistanceFromA, coupleTargetPos.Facing);

                return new SplitInfo(splitCar, splitEnd, droppedCars,
                    dropGp, coupleTarget, graphCoupleWp);
            }

            return null;
        }

        /// <summary>
        /// Group cars by destination using ITrainService for waybill queries.
        /// Works from the tail end (index 0) backward, grouping consecutive cars
        /// with the same destination track.
        /// </summary>
        private List<(List<ICar> cars, GraphPosition destLocation, string destName, GraphPosition approachTarget)>
            GroupByDestinationAbstract(IReadOnlyList<ICar> cars, IReadOnlyCollection<string>? skippedCarIds)
        {
            var groups = new List<(List<ICar>, GraphPosition, string, GraphPosition)>();

            int i = cars.Count - 1;
            while (i >= 0)
            {
                var car = cars[i];

                if (car.IsLocoOrTender || !_iTrainService!.HasWaybill(car)
                    || (skippedCarIds != null && skippedCarIds.Contains(car.id)))
                {
                    i--;
                    continue;
                }

                var destTrackId = _iTrainService.GetDestinationTrackId(car);
                var destName = _iTrainService.GetDestinationName(car);
                if (destTrackId == null || destName == null)
                {
                    i--;
                    continue;
                }

                var candidates = _iTrainService.GetDestinationCandidates(car);
                if (candidates.Count == 0)
                {
                    i--;
                    continue;
                }

                var destLocation = candidates[0].Location;
                var approachTarget = candidates[0].ApproachTarget;

                var carGroup = new List<ICar> { car };
                while (i - 1 >= 0)
                {
                    var nextCar = cars[i - 1];
                    if (nextCar.IsLocoOrTender || !_iTrainService.HasWaybill(nextCar))
                        break;
                    var nextDestTrackId = _iTrainService.GetDestinationTrackId(nextCar);
                    if (nextDestTrackId != destTrackId)
                        break;
                    carGroup.Add(nextCar);
                    i--;
                }

                groups.Add((carGroup, destLocation, destName, approachTarget));
                i--;
            }

            return groups;
        }

        private List<(List<Car> cars, DirectedPosition destLocation, string destName, SpanBoundary approachTarget)> GroupByDestination(
            IReadOnlyList<ICar> cars, BaseLocomotive loco, IEnumerable<string>? skippedCarIds = null)
        {
            var groups = new List<(List<Car>, DirectedPosition, string, SpanBoundary)>();

            int i = cars.Count - 1;
            while (i >= 0)
            {
                var car = cars[i];

                if (car.IsLocoOrTender || car.Waybill == null
                    || (skippedCarIds != null && skippedCarIds.Contains(car.id)))
                {
                    i--;
                    continue;
                }

                DirectedPosition destLoc;
                string destName;
                SpanBoundary approachTarget;
                try
                {
                    var dest = car.Waybill.Value.Destination;
                    destLoc = _destinationSelector.GetDestinationLocation(dest, loco);
                    destName = dest.DisplayName;
                    var destCandidates = _destinationSelector.GetDestinationCandidates(dest, loco);
                    if (destCandidates.Count > 0)
                    {
                        approachTarget = destCandidates[0].approachTarget;
                    }
                    else
                    {
                        approachTarget = new SpanBoundary(destLoc.Segment?.id, destLoc.DistanceFromA, destLoc.Facing);
                    }
                }
                catch
                {
                    i--;
                    continue;
                }

                if (destLoc.Segment == null)
                {
                    i--;
                    continue;
                }

                var gameCar = (car as CarAdapter)?.Car;
                var carGroup = new List<Car> { gameCar };
                while (i - 1 >= 0)
                {
                    var nextCar = cars[i - 1];
                    if (nextCar.IsLocoOrTender || nextCar.Waybill == null)
                        break;
                    DirectedPosition nextDestLoc;
                    try
                    {
                        nextDestLoc = _destinationSelector.GetDestinationLocation(
                            nextCar.Waybill.Value.Destination, loco);
                    }
                    catch { break; }
                    if (nextDestLoc.Segment == null || nextDestLoc.Segment != destLoc.Segment)
                        break;
                    carGroup.Add((nextCar as CarAdapter)?.Car);
                    i--;
                }

                groups.Add((carGroup, destLoc, destName, approachTarget));
                i--;
            }

            return groups;
        }

        private HashSet<string> ComputeBlockedSwitches(
            List<Car> droppedCars, List<Car> keptCars, BaseLocomotive loco, CarGroup group)
        {
            var graph = Graph.Shared;

            var droppedSegIds = new HashSet<string>();
            foreach (var car in droppedCars)
            {
                if (car.LocationA.segment != null) droppedSegIds.Add(car.LocationA.segment.id);
                if (car.LocationB.segment != null) droppedSegIds.Add(car.LocationB.segment.id);
            }

            var keptSegIds = new HashSet<string>();
            if (loco.LocationF.segment != null) keptSegIds.Add(loco.LocationF.segment.id);
            if (loco.LocationR.segment != null) keptSegIds.Add(loco.LocationR.segment.id);
            foreach (var car in keptCars)
            {
                if (car.LocationA.segment != null) keptSegIds.Add(car.LocationA.segment.id);
                if (car.LocationB.segment != null) keptSegIds.Add(car.LocationB.segment.id);
            }

            var candidateSwitches = new HashSet<string>();
            foreach (var car in droppedCars)
            {
                foreach (var loc in new[] { car.LocationA, car.LocationB })
                {
                    if (loc.segment == null) continue;
                    foreach (var end in new[] { TrackSegment.End.A, TrackSegment.End.B })
                    {
                        var node = loc.segment.NodeForEnd(end);
                        if (node != null && graph.IsSwitch(node))
                            candidateSwitches.Add(node.id);
                    }
                }
            }

            var blocked = new HashSet<string>();
            foreach (var switchId in candidateSwitches)
            {
                if (!IsSwitchOnKeptOrLocoSegments(switchId, keptCars, loco))
                    blocked.Add(switchId);
            }

            var tailCar = group.TailCar;
            if (tailCar != null && group.TailOutwardEnd.HasValue && group.TailInwardEnd.HasValue)
            {
                var tailOutward = group.TailOutwardEnd.Value;
                var tailInward = group.TailInwardEnd.Value;

                // Resolve GraphPosition to game segment for the walk
                TrackSegment? outwardSeg = null;
                TrackSegment? inwardSeg = null;
                if (tailOutward.SegmentId != null)
                {
                    foreach (var seg in graph.Segments)
                    {
                        if (seg.id == tailOutward.SegmentId) { outwardSeg = seg; break; }
                    }
                }
                if (tailInward.SegmentId != null)
                {
                    foreach (var seg in graph.Segments)
                    {
                        if (seg.id == tailInward.SegmentId) { inwardSeg = seg; break; }
                    }
                }

                if (outwardSeg != null)
                {
                    TrackSegment.End walkEnd;
                    if (outwardSeg == inwardSeg)
                    {
                        walkEnd = tailOutward.DistanceFromA < tailInward.DistanceFromA
                            ? TrackSegment.End.A : TrackSegment.End.B;
                    }
                    else
                    {
                        var cn = inwardSeg != null ? Services.TrackWalker.FindSharedNode(outwardSeg, inwardSeg) : null;
                        var ce = cn != null ? outwardSeg.EndForNode(cn) : TrackSegment.End.A;
                        walkEnd = ce == TrackSegment.End.A ? TrackSegment.End.B : TrackSegment.End.A;
                    }

                    var result = Services.TrackWalker.WalkToSwitch(
                        outwardSeg, walkEnd, maxSegments: 10);
                    if (result.HasValue && !blocked.Contains(result.Value.switchNode.id))
                    {
                        blocked.Add(result.Value.switchNode.id);
                        LogDebug($"  Switch beyond dropped cars: {result.Value.switchNode.id}");
                    }
                }
            }

            LogDebug($"  Blocked switches: [{string.Join(",", blocked)}]");
            return blocked;
        }

        public static bool IsRouteSafe(List<RouteSearch.Step> routeSteps, HashSet<string> blockedSwitches)
        {
            foreach (var step in routeSteps)
            {
                if (step.Node != null && blockedSwitches.Contains(step.Node.id))
                    return false;
            }
            return true;
        }

        private static bool IsSwitchOnSegment(string switchId, Location loc)
        {
            if (loc.segment == null) return false;
            var nodeA = loc.segment.NodeForEnd(TrackSegment.End.A);
            var nodeB = loc.segment.NodeForEnd(TrackSegment.End.B);
            return (nodeA != null && nodeA.id == switchId)
                || (nodeB != null && nodeB.id == switchId);
        }

        private static bool IsSwitchOnKeptOrLocoSegments(
            string switchId, List<Car> keptCars, BaseLocomotive loco)
        {
            foreach (var car in keptCars)
            {
                if (IsSwitchOnSegment(switchId, car.LocationA)
                    || IsSwitchOnSegment(switchId, car.LocationB))
                    return true;
            }
            return IsSwitchOnSegment(switchId, loco.LocationF)
                || IsSwitchOnSegment(switchId, loco.LocationR);
        }

        private SplitInfo BuildSplitInfo(BaseLocomotive loco, List<Car> keptCars,
            List<Car> droppedCars, CarGroup group, Graph graph)
        {
            var splitCar = keptCars[keptCars.Count - 1];
            var firstDropped = droppedCars[0];
            var splitEnd = splitCar.CoupledTo(Car.LogicalEnd.A) == firstDropped
                ? Car.LogicalEnd.A : Car.LogicalEnd.B;
            var coupleTarget = firstDropped;

            var freeEnd = coupleTarget.CoupledTo(Car.LogicalEnd.A) != null
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            var coupleWaypoint = CoupleLocationCalculator.GetCoupleLocationForEnd(
                new CarAdapter(coupleTarget), freeEnd, graph);

            // Convert to abstract types
            var dropGp = new GraphPosition(coupleWaypoint.Segment?.id,
                coupleWaypoint.DistanceFromA, coupleWaypoint.Facing);
            var graphCoupleWp = new GraphCoupleWaypoint(
                coupleWaypoint.Segment?.id, coupleWaypoint.DistanceFromA, coupleWaypoint.Facing);

            // Wrap game Cars as ICars
            ICar splitCarICar = new CarAdapter(splitCar);
            ICar coupleTargetICar = new CarAdapter(coupleTarget);
            IReadOnlyList<ICar> droppedICars = droppedCars.Select(c => (ICar)new CarAdapter(c)).ToList();

            return new SplitInfo(splitCarICar, splitEnd, droppedICars,
                dropGp, coupleTargetICar, graphCoupleWp);
        }
    }
}
