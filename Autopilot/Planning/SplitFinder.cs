using System;
using System.Collections.Generic;
using System.Linq;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class SplitFinder
    {
        private readonly TrainService _trainService;
        private readonly FeasibilityChecker _checker;
        private readonly DestinationSelector _destinationSelector;

        public SplitFinder(TrainService trainService, FeasibilityChecker checker, DestinationSelector destinationSelector)
        {
            _trainService = trainService;
            _checker = checker;
            _destinationSelector = destinationSelector;
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot SplitFinder: {msg}");
        private void LogDebug(string msg) { if (Loader.Settings?.verboseLogging == true) Log(msg); }

        /// <summary>
        /// Find the best split point for the train. Groups cars by destination,
        /// tries progressively longer kept-car prefixes from the loco end.
        /// Returns SplitInfo for the split that maximizes deliverable cars
        /// with routes that don't pass through blocked switches.
        /// Returns null if no valid split found.
        /// </summary>
        public SplitInfo? FindBestSplit(BaseLocomotive loco, CarGroup group, IEnumerable<Car>? skippedCars = null)
        {
            if (group.IsEmpty || group.Count < 2)
                return null;

            var cars = group.Cars;
            var graph = Graph.Shared;

            // Group cars by destination from loco-end toward tail.
            // CarGroup.Cars is ordered tail-to-loco, so loco-end is at the end.
            var destGroups = GroupByDestination(cars, loco, skippedCars);
            if (destGroups.Count < 2)
                return null; // Can't split — all one group

            Log($"Found {destGroups.Count} destination groups");

            // Walk from loco toward tail. For each group, check if it could be
            // the TAIL of the kept consist (i.e., the first delivery after splitting).
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

                // Only check the TAIL group — that's the first delivery after splitting.
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

                LogDebug($"  Tail group {tailGroup.destName} is deliverable ({tailGroup.cars.Count} cars)");
                var split = BuildSplitInfo(loco, keptCars, droppedCars, group, graph);
                Log($"Split found: keep {keptCars.Count} cars (tail={tailGroup.destName}), drop {droppedCars.Count}");
                return split;
            }

            Log("No valid split found");
            return null;
        }

        /// <summary>
        /// Group cars by destination segment, ordered from loco-end to tail.
        /// </summary>
        private List<(List<Car> cars, DirectedPosition destLocation, string destName)> GroupByDestination(
            IReadOnlyList<ICar> cars, BaseLocomotive loco, IEnumerable<Car>? skippedCars = null)
        {
            var groups = new List<(List<Car>, DirectedPosition, string)>();

            // Iterate from loco-end (last index) toward tail (first index)
            int i = cars.Count - 1;
            while (i >= 0)
            {
                var car = cars[i];

                var gameCarCheck = (car as CarAdapter)?.Car;
                if (car.IsLocoOrTender || car.Waybill == null
                    || (skippedCars != null && gameCarCheck != null && skippedCars.Contains(gameCarCheck)))
                {
                    i--;
                    continue;
                }

                DirectedPosition destLoc;
                string destName;
                try
                {
                    var dest = car.Waybill.Value.Destination;
                    destLoc = _destinationSelector.GetDestinationLocation(dest, loco);
                    destName = dest.DisplayName;
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

                // Group consecutive cars (toward tail) with the same destination segment
                var group = new List<Car> { (car as CarAdapter)?.Car };
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
                    group.Add((nextCar as CarAdapter)?.Car);
                    i--;
                }

                groups.Add((group, destLoc, destName));
                i--;
            }

            return groups;
        }

        /// <summary>
        /// Compute the set of switches blocked by the dropped cars.
        /// </summary>
        private HashSet<string> ComputeBlockedSwitches(
            List<Car> droppedCars, List<Car> keptCars, BaseLocomotive loco, CarGroup group)
        {
            var graph = Graph.Shared;

            // Step 1: Segments occupied by dropped cars
            var droppedSegIds = new HashSet<string>();
            foreach (var car in droppedCars)
            {
                if (car.LocationA.segment != null) droppedSegIds.Add(car.LocationA.segment.id);
                if (car.LocationB.segment != null) droppedSegIds.Add(car.LocationB.segment.id);
            }

            // Step 2: Segments occupied by kept cars + loco (these switches must NOT be blocked)
            var keptSegIds = new HashSet<string>();
            if (loco.LocationF.segment != null) keptSegIds.Add(loco.LocationF.segment.id);
            if (loco.LocationR.segment != null) keptSegIds.Add(loco.LocationR.segment.id);
            foreach (var car in keptCars)
            {
                if (car.LocationA.segment != null) keptSegIds.Add(car.LocationA.segment.id);
                if (car.LocationB.segment != null) keptSegIds.Add(car.LocationB.segment.id);
            }

            // Step 3: Find all switches on dropped car segments
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

            // Step 4: Remove switches that are ALSO on kept car / loco segments.
            var blocked = new HashSet<string>();
            foreach (var switchId in candidateSwitches)
            {
                if (!IsSwitchOnKeptOrLocoSegments(switchId, keptCars, loco))
                    blocked.Add(switchId);
            }

            // Step 5: Find the first switch BEYOND the dropped cars (tail direction)
            var tailCar = group.TailCar;
            if (tailCar != null && group.TailOutwardEnd.HasValue && group.TailInwardEnd.HasValue)
            {
                var tailOutward = group.TailOutwardEnd.Value;
                var tailInward = group.TailInwardEnd.Value;
                if (tailOutward.Segment != null)
                {
                    // Determine outward direction from canonical position
                    TrackSegment.End walkEnd;
                    if (tailOutward.Segment == tailInward.Segment)
                    {
                        walkEnd = tailOutward.DistanceFromA < tailInward.DistanceFromA
                            ? TrackSegment.End.A : TrackSegment.End.B;
                    }
                    else
                    {
                        var cn = Services.TrackWalker.FindSharedNode(tailOutward.Segment, tailInward.Segment);
                        var ce = cn != null ? tailOutward.Segment.EndForNode(cn) : TrackSegment.End.A;
                        walkEnd = ce == TrackSegment.End.A ? TrackSegment.End.B : TrackSegment.End.A;
                    }

                    var result = Services.TrackWalker.WalkToSwitch(
                        tailOutward.Segment, walkEnd, maxSegments: 10);
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

        /// <summary>
        /// Check if a route is safe — no route steps pass through blocked switches.
        /// </summary>
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

        /// <summary>
        /// Build a SplitInfo from the kept/dropped car lists.
        /// </summary>
        private SplitInfo BuildSplitInfo(BaseLocomotive loco, List<Car> keptCars,
            List<Car> droppedCars, CarGroup group, Graph graph)
        {
            // Split car is the last kept car (closest to the dropped cars)
            var splitCar = keptCars[keptCars.Count - 1];

            // Split end faces the dropped cars
            var firstDropped = droppedCars[0];
            var splitEnd = splitCar.CoupledTo(Car.LogicalEnd.A) == firstDropped
                ? Car.LogicalEnd.A : Car.LogicalEnd.B;

            // Couple target is the first dropped car
            var coupleTarget = firstDropped;

            // Couple location: free (uncoupled) end of the couple target.
            // Don't use ClosestLogicalEndTo (crow-flies).
            var freeEnd = coupleTarget.CoupledTo(Car.LogicalEnd.A) != null
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            var coupleLoc = CoupleLocationCalculator.GetCoupleLocationForEnd(
                new CarAdapter(coupleTarget), freeEnd, graph).ToLocation();

            var coupleLocation = DirectedPosition.FromLocation(coupleLoc);

            return new SplitInfo(splitCar, splitEnd, droppedCars,
                coupleLocation, coupleTarget, coupleLocation);
        }
    }
}
