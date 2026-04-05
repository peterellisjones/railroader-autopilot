// Autopilot/Planning/DeliveryPlanner.cs
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
    public class DeliveryPlanner
    {
        private readonly TrainService _trainService;
        private readonly FeasibilityChecker _checker;
        private readonly DestinationSelector _destinationSelector;
        private readonly DeliverabilityAnalyzer _deliverabilityAnalyzer;
        private readonly RunaroundBuilder _runaroundBuilder;

        // Testable planning fields (used by interface-based constructor path)
        private readonly ITrainService? _iTrainService;
        private readonly IGraphAdapter? _iGraph;
        private readonly FeasibilityChecker? _iChecker;
        private readonly DeliverabilityAnalyzer? _iAnalyzer;

        public DeliveryPlanner(TrainService trainService)
        {
            _trainService = trainService;
            _checker = new FeasibilityChecker(trainService);
            _destinationSelector = new DestinationSelector(trainService, _checker.RouteChecker);
            _deliverabilityAnalyzer = new DeliverabilityAnalyzer(_destinationSelector);
            _runaroundBuilder = new RunaroundBuilder(_deliverabilityAnalyzer);
        }

        /// <summary>Constructor for testable planning.</summary>
        public DeliveryPlanner(ITrainService trainService, IGraphAdapter graph)
        {
            _iTrainService = trainService;
            _iGraph = graph;
            _iChecker = new FeasibilityChecker(trainService, graph);
            _iAnalyzer = new DeliverabilityAnalyzer(trainService);
            _trainService = null!;
            _checker = null!;
            _destinationSelector = null!;
            _deliverabilityAnalyzer = null!;
            _runaroundBuilder = null!;
        }

        private void ClearCaches()
        {
            _trainService.ClearPlanCaches();
            _destinationSelector.ClearCache();
            _checker.RouteChecker.ClearCache();
            _checker.ClearCache();
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot Planner: {msg}");

        private static void RegisterSegmentById(TrackGraph.GameGraphAdapter adapter, string segmentId)
        {
            if (segmentId == null) return;
            var graph = Graph.Shared;
            foreach (var seg in graph.Segments)
            {
                if (seg.id == segmentId)
                {
                    adapter.RegisterSegment(seg);
                    return;
                }
            }
        }

        public DeliveryPlan BuildPlan(BaseLocomotive loco,
            IEnumerable<string>? visitedSwitches = null,
            IEnumerable<string>? visitedLoopKeys = null,
            IEnumerable<string>? skippedCarIds = null)
        {
            ClearCaches();
            var layout = ConsistLayout.Create(loco, _trainService);
            var warnings = new List<string>();

            Log($"Consist: sideA={layout.SideA.Count} cars, sideB={layout.SideB.Count} cars");

            if (!layout.HasWaybilledCars)
            {
                Log("No waybilled cars remain — complete.");
                return new DeliveryPlan(new List<DeliveryStep>(), warnings);
            }

            // Check both sides for direct deliveries.
            var stepsA = _deliverabilityAnalyzer.GetDeliverableSteps(loco, layout.SideA, _checker, skippedCarIds, maxSteps: 1);
            var stepsB = _deliverabilityAnalyzer.GetDeliverableSteps(loco, layout.SideB, _checker, skippedCarIds, maxSteps: 1);

            Log($"Direct deliveries: sideA={stepsA.Count}{(stepsA.Count > 0 ? $" (first: {stepsA[0].DestinationName})" : "")}, sideB={stepsB.Count}{(stepsB.Count > 0 ? $" (first: {stepsB[0].DestinationName})" : "")}");

            bool needsConsolidation = SidesShareDestination(layout.SideA, layout.SideB, skippedCarIds);

            if (needsConsolidation)
            {
                Log("Both sides have cars for same destination — need split to consolidate");
            }
            else if (stepsA.Count > 0 || stepsB.Count > 0)
            {
                var bestSteps = PickCloserDelivery(loco, stepsA, stepsB);
                return new DeliveryPlan(bestSteps, warnings,
                    reason: $"Delivering {bestSteps[0].CarNames} to {bestSteps[0].DestinationName}");
            }

            Log(needsConsolidation
                ? "Same destination on both sides — checking runaround to consolidate..."
                : "No direct deliveries — checking runarounds...");
            var flippedA = layout.SideA.Reversed();
            var flippedB = layout.SideB.Reversed();
            var flippedStepsA = _deliverabilityAnalyzer.GetDeliverableSteps(loco, flippedA, _checker, skippedCarIds, maxSteps: 3);
            var flippedStepsB = _deliverabilityAnalyzer.GetDeliverableSteps(loco, flippedB, _checker, skippedCarIds, maxSteps: 3);
            int scoreA = flippedStepsA.Count;
            int scoreB = flippedStepsB.Count;

            if (needsConsolidation)
            {
                bool aDeliverable = flippedStepsA.Count > 0;
                bool bDeliverable = flippedStepsB.Count > 0;

                if (aDeliverable && !bDeliverable)
                    scoreA += 1000;
                else if (bDeliverable && !aDeliverable)
                    scoreB += 1000;
                else
                {
                    scoreA += CountSharedDestCarsFromWaybills(flippedStepsA, layout.SideB, skippedCarIds);
                    scoreB += CountSharedDestCarsFromWaybills(flippedStepsB, layout.SideA, skippedCarIds);
                }
            }

            Log($"Runaround scores: sideA={scoreA}, sideB={scoreB}");

            var loopStatus = _checker.GetLoopStatus(loco);

            if (!loopStatus.CanRunaround)
            {
                float trainLen = _trainService.GetTrainLength(loco);
                var (nearbyLoc, nearbyLoopKey) = _checker.GetRepositionLocation(loco, visitedSwitches, visitedLoopKeys);
                if (nearbyLoc.HasValue)
                {
                    float nearbyDist = _checker.RouteChecker.GraphDistanceToLoco(loco, nearbyLoc.Value)?.Distance ?? float.MaxValue;
                    if (nearbyDist < trainLen)
                    {
                        var adapter = new TrackGraph.GameGraphAdapter();
                        var nearbyLoop = _checker.LoopValidator.FindFittingLoop(loco, adapter, visitedSwitches, new HashSet<string>());
                        if (nearbyLoop != null)
                        {
                            Log($"Loop status: segment check failed but train is {nearbyDist:F0}m from loop {nearbyLoop.SwitchAId}↔{nearbyLoop.SwitchBId} — treating as on-loop");
                            loopStatus = LoopStatus.OnLoop(nearbyLoop);
                        }
                    }
                }
            }

            Log($"Loop status: canRunaround={loopStatus.CanRunaround}" +
                (loopStatus.Loop != null ? $", loop={loopStatus.Loop.SwitchAId}↔{loopStatus.Loop.SwitchBId}" : ""));

            if (scoreA >= scoreB && scoreA > 0)
            {
                var runaround = _runaroundBuilder.BuildRunaroundAction(loco, layout.SideA, Track.Graph.Shared, _trainService, _checker, skippedCarIds);
                if (runaround != null && loopStatus.CanRunaround)
                {
                    var destName = flippedStepsA.Count > 0 ? flippedStepsA[0].DestinationName : "?";
                    return new DeliveryPlan(new List<DeliveryStep>(), warnings, runaround,
                        reason: needsConsolidation
                            ? $"Consolidation runaround — both sides have cars for {destName}"
                            : $"Runaround to deliver to {destName}");
                }
                Log(loopStatus.CanRunaround
                    ? "SideA runaround: couldn't build action"
                    : "SideA runaround not feasible — train not fully on loop");
                scoreA = 0;
            }
            if (scoreB > 0)
            {
                var runaround = _runaroundBuilder.BuildRunaroundAction(loco, layout.SideB, Track.Graph.Shared, _trainService, _checker, skippedCarIds);
                if (runaround != null && loopStatus.CanRunaround)
                {
                    var destName = flippedStepsB.Count > 0 ? flippedStepsB[0].DestinationName : "?";
                    return new DeliveryPlan(new List<DeliveryStep>(), warnings, runaround,
                        reason: needsConsolidation
                            ? $"Consolidation runaround — both sides have cars for {destName}"
                            : $"Runaround to deliver to {destName}");
                }
                Log(loopStatus.CanRunaround
                    ? "SideB runaround: couldn't build action"
                    : "SideB runaround not feasible — train not fully on loop");
            }

            bool hasDeliverableCars = false;
            foreach (var car in layout.SideA.Cars.Concat(layout.SideB.Cars))
            {
                if (car.Waybill == null) continue;
                if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                var space = _destinationSelector.GetAvailableSpace(car.Waybill.Value.Destination, loco);
                if (space >= 2f)
                {
                    hasDeliverableCars = true;
                    break;
                }
            }

            bool anyFlippedDeliverable = flippedStepsA.Count > 0 || flippedStepsB.Count > 0;

            // Check if a split would help: try routing to each destination
            // with the minimum possible split train (loco + tail car only).
            // If the route works with that shorter length, splitting is viable.
            bool splitViable = false;
            string splitDestName = null;
            if (!anyFlippedDeliverable && hasDeliverableCars)
            {
                var coupled = _trainService.GetCoupled(loco);
                var adapter = new TrackGraph.GameGraphAdapter();
                foreach (var side in new[] { layout.SideA, layout.SideB })
                {
                    if (side.IsEmpty) continue;
                    // Iterate from loco end (last in Cars list) to match
                    // SplitFinder, which keeps loco-end cars and drops tail.
                    for (int ci = side.Cars.Count - 1; ci >= 0; ci--)
                    {
                        var car = side.Cars[ci];
                        if (car.Waybill == null) continue;
                        if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                        var dest = car.Waybill.Value.Destination;
                        var candidates = _destinationSelector.GetDestinationCandidates(dest, loco);
                        float splitLen = loco.carLength + car.CarLength + AutopilotConstants.ConsistGapPerCar;
                        foreach (var c in candidates)
                        {
                            if (c.availableSpace < 2f) continue;
                            RegisterSegmentById(adapter, c.approachTarget.SegmentId);
                            if (_checker.CanRouteTo(loco, c.approachTarget.ToDirectedPosition(adapter), splitLen, coupled))
                            {
                                splitViable = true;
                                splitDestName = dest.DisplayName;
                                Log($"Split viable: {dest.DisplayName} reachable at {splitLen:F0}m (full train {_trainService.GetTrainLength(loco):F0}m)");
                                break;
                            }
                        }
                        if (splitViable) break;
                    }
                    if (splitViable) break;
                }
                if (!splitViable)
                    Log("Split not viable — even shortest split train can't reach any destination");
            }

            if (hasDeliverableCars && !loopStatus.CanRunaround && (anyFlippedDeliverable || splitViable))
            {
                Log("Not on loop — checking reposition...");

                var deliveryDests = new List<SpanBoundary>();
                var seenDestIds = new HashSet<string>();
                foreach (var car in layout.SideA.Cars.Concat(layout.SideB.Cars))
                {
                    if (!WaybillHelper.IsPendingDelivery(car)) continue;
                    if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                    var dest = car.Waybill.Value.Destination;
                    if (!seenDestIds.Add(dest.Identifier)) continue;
                    foreach (var span in dest.Spans)
                    {
                        if (span.lower != null)
                        {
                            var pos = DirectedPosition.FromLocation(span.lower.Value);
                            var boundary = new SpanBoundary(pos.Segment?.id, pos.DistanceFromA, pos.Facing);
                            deliveryDests.Add(boundary);
                            Log($"Delivery dest for reversal check: {dest.DisplayName} → {pos.Segment?.id}");
                            break;
                        }
                    }
                }
                Log($"Total delivery destinations for reversal check: {deliveryDests.Count}");

                var (repositionLoc, loopKey) = _checker.GetRepositionLocation(loco, visitedSwitches, visitedLoopKeys, deliveryDests);
                if (repositionLoc.HasValue)
                {
                    string destName;
                    string action;
                    if (anyFlippedDeliverable)
                    {
                        action = "runaround";
                        destName = null;
                        if (scoreA > 0 && flippedStepsA.Count > 0)
                            destName = flippedStepsA[0].DestinationName;
                        else if (scoreB > 0 && flippedStepsB.Count > 0)
                            destName = flippedStepsB[0].DestinationName;
                    }
                    else
                    {
                        action = "split";
                        destName = splitDestName;
                    }
                    string reason = destName != null
                        ? $"Repositioning to loop (need {action} to deliver to {destName})"
                        : "Repositioning to loop";

                    // Convert DirectedPosition to GraphPosition for the plan
                    var gpReposition = new GraphPosition(repositionLoc.Value.Segment?.id,
                        repositionLoc.Value.DistanceFromA, repositionLoc.Value.Facing);

                    Log(reason);
                    return new DeliveryPlan(new List<DeliveryStep>(), warnings,
                        repositionLocation: gpReposition, reason: reason,
                        repositionLoopKey: loopKey);
                }
            }

            // Split is last resort
            Log("No reposition available — checking splits...");
            var splitFinder = new SplitFinder(_trainService, _checker, _destinationSelector);
            var splitA = layout.SideA.IsEmpty ? null : splitFinder.FindBestSplit(loco, layout.SideA, skippedCarIds);
            var splitB = layout.SideB.IsEmpty ? null : splitFinder.FindBestSplit(loco, layout.SideB, skippedCarIds);

            var bestSplit = splitA;
            if (splitB != null && (bestSplit == null || splitB.DroppedCars.Count < bestSplit.DroppedCars.Count))
                bestSplit = splitB;

            if (bestSplit != null)
            {
                Log($"Split found: drop {bestSplit.DroppedCars.Count} cars");
                return new DeliveryPlan(new List<DeliveryStep>(), warnings, split: bestSplit,
                    reason: $"Splitting — dropping {bestSplit.DroppedCars.Count} car(s) to shorten train");
            }

            warnings.Add("No deliverable cars found.");
            return new DeliveryPlan(new List<DeliveryStep>(), warnings);
        }

        private static int CountSharedDestCars(List<DeliveryStep> flippedSteps, List<DeliveryStep> otherSteps)
        {
            var dests = new HashSet<string>();
            foreach (var step in flippedSteps)
                dests.Add(step.DestinationName);

            int count = 0;
            foreach (var step in otherSteps)
            {
                if (dests.Contains(step.DestinationName))
                    count += step.Cars.Count;
            }
            return count;
        }

        private static bool SharesDestination(List<DeliveryStep> stepsA, List<DeliveryStep> stepsB)
        {
            var destsA = new HashSet<string>();
            foreach (var step in stepsA)
                destsA.Add(step.DestinationName);
            foreach (var step in stepsB)
            {
                if (destsA.Contains(step.DestinationName))
                    return true;
            }
            return false;
        }

        private static int CountSharedDestCarsFromWaybills(
            List<DeliveryStep> flippedSteps, CarGroup otherSide, IEnumerable<string>? skippedCarIds)
        {
            var dests = new HashSet<string>();
            foreach (var step in flippedSteps)
                dests.Add(step.DestinationName);

            int count = 0;
            foreach (var car in otherSide.Cars)
            {
                if (car.Waybill == null) continue;
                if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                if (dests.Contains(car.Waybill.Value.Destination.DisplayName))
                    count++;
            }
            return count;
        }

        private static bool SidesShareDestination(CarGroup sideA, CarGroup sideB, IEnumerable<string>? skippedCarIds)
        {
            if (sideA.IsEmpty || sideB.IsEmpty) return false;

            var destsA = new HashSet<string>();
            foreach (var car in sideA.Cars)
            {
                if (car.Waybill == null) continue;
                if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                destsA.Add(car.Waybill.Value.Destination.DisplayName);
            }

            foreach (var car in sideB.Cars)
            {
                if (car.Waybill == null) continue;
                if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                if (destsA.Contains(car.Waybill.Value.Destination.DisplayName))
                    return true;
            }
            return false;
        }

        private List<DeliveryStep> PickCloserDelivery(BaseLocomotive loco,
            List<DeliveryStep> stepsA, List<DeliveryStep> stepsB)
        {
            if (stepsA.Count == 0) return stepsB;
            if (stepsB.Count == 0) return stepsA;

            if (stepsA.Count != stepsB.Count)
                return stepsA.Count > stepsB.Count ? stepsA : stepsB;

            // Convert GraphPosition to DirectedPosition for distance check
            var adapter = new TrackGraph.GameGraphAdapter();
            var dpA = ConvertToDirectedPosition(stepsA[0].DestinationLocation, adapter);
            var dpB = ConvertToDirectedPosition(stepsB[0].DestinationLocation, adapter);
            float distA = _trainService.GraphDistanceToLoco(loco, dpA)?.Distance ?? float.MaxValue;
            float distB = _trainService.GraphDistanceToLoco(loco, dpB)?.Distance ?? float.MaxValue;
            return distA <= distB ? stepsA : stepsB;
        }

        private static DirectedPosition ConvertToDirectedPosition(GraphPosition gp, TrackGraph.GameGraphAdapter adapter)
        {
            if (gp.SegmentId != null)
            {
                var graph = Track.Graph.Shared;
                foreach (var seg in graph.Segments)
                {
                    if (seg.id == gp.SegmentId)
                    {
                        adapter.RegisterSegment(seg);
                        break;
                    }
                }
            }
            return adapter.ToDirectedPosition(gp);
        }

        // =================================================================
        // Testable BuildPlan using ITrainService + IGraphAdapter
        // =================================================================

        /// <summary>
        /// Build a delivery plan using abstract interfaces (no game types).
        /// Covers the core planning logic: consist layout, deliverability analysis,
        /// same-destination consolidation check, and direct delivery selection.
        /// Runaround/reposition/split branches are deferred to a later task.
        /// </summary>
        public DeliveryPlan BuildPlan(
            IEnumerable<string>? visitedSwitches = null,
            IEnumerable<string>? visitedLoopKeys = null,
            IReadOnlyCollection<string>? skippedCarIds = null)
        {
            _iTrainService!.ClearPlanCaches();
            var layout = ConsistLayout.Create(_iTrainService);
            var warnings = new List<string>();

            if (!layout.HasWaybilledCars)
            {
                return new DeliveryPlan(new List<DeliveryStep>(), warnings);
            }

            // Check both sides for direct deliveries.
            var stepsA = _iAnalyzer!.GetDeliverableSteps(layout.SideA, _iChecker!, skippedCarIds, maxSteps: 1);
            var stepsB = _iAnalyzer.GetDeliverableSteps(layout.SideB, _iChecker!, skippedCarIds, maxSteps: 1);

            bool needsConsolidation = SidesShareDestinationAbstract(
                layout.SideA, layout.SideB, _iTrainService, skippedCarIds);

            if (needsConsolidation)
            {
                // Both sides have cars for the same destination.
                // In the full planner this triggers runaround logic.
                // For now, return empty steps with an informational warning.
                warnings.Add("Both sides have cars for same destination — runaround needed (not yet supported in abstract path).");
                return new DeliveryPlan(new List<DeliveryStep>(), warnings);
            }

            if (stepsA.Count > 0 || stepsB.Count > 0)
            {
                var bestSteps = PickCloserDeliveryAbstract(stepsA, stepsB);
                return new DeliveryPlan(bestSteps, warnings,
                    reason: $"Delivering {bestSteps[0].CarNames} to {bestSteps[0].DestinationName}");
            }

            // No direct deliveries — check if flipping either side
            // (runaround) would produce deliverable steps.
            var flippedA = layout.SideA.Reversed();
            var flippedB = layout.SideB.Reversed();
            var flippedStepsA2 = _iAnalyzer.GetDeliverableSteps(flippedA, _iChecker!, skippedCarIds, maxSteps: 3);
            var flippedStepsB2 = _iAnalyzer.GetDeliverableSteps(flippedB, _iChecker!, skippedCarIds, maxSteps: 3);

            var loopStatus = _iTrainService.GetLoopStatus();

            bool hasDeliverableCars = false;
            foreach (var car in layout.SideA.Cars.Concat(layout.SideB.Cars))
            {
                if (!_iTrainService.HasWaybill(car)) continue;
                if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                float space = _iTrainService.GetAvailableSpace(car);
                if (space >= 2f)
                {
                    hasDeliverableCars = true;
                    break;
                }
            }

            bool anyFlippedDeliverable2 = flippedStepsA2.Count > 0 || flippedStepsB2.Count > 0;

            // Check split viability: in the abstract path, CanRouteTo doesn't
            // model train length. If a route is completely blocked (CanRouteTo
            // fails), split can't help. Check if any destination is routable.
            bool splitViable2 = false;
            if (!anyFlippedDeliverable2 && hasDeliverableCars)
            {
                foreach (var car in layout.SideA.Cars.Concat(layout.SideB.Cars))
                {
                    if (!_iTrainService.HasWaybill(car)) continue;
                    if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                    var candidates = _iTrainService.GetDestinationCandidates(car);
                    foreach (var c in candidates)
                    {
                        if (_iChecker!.RouteChecker.CanRouteTo(c.ApproachTarget))
                        {
                            splitViable2 = true;
                            break;
                        }
                    }
                    if (splitViable2) break;
                }
            }

            if (hasDeliverableCars && !loopStatus.CanRunaround
                && (anyFlippedDeliverable2 || splitViable2))
            {
                var deliveryDests = new List<GraphPosition>();
                foreach (var car in layout.SideA.Cars.Concat(layout.SideB.Cars))
                {
                    if (!_iTrainService.HasWaybill(car)) continue;
                    if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                    var candidates = _iTrainService.GetDestinationCandidates(car);
                    if (candidates.Count > 0)
                        deliveryDests.Add(candidates[0].ApproachTarget);
                }

                var (repositionLoc, loopKey) = _iTrainService.GetRepositionLocation(
                    visitedSwitches, visitedLoopKeys, deliveryDests);
                if (repositionLoc.HasValue)
                {
                    string action = anyFlippedDeliverable2 ? "runaround" : "split";
                    return new DeliveryPlan(new List<DeliveryStep>(), warnings,
                        repositionLocation: repositionLoc.Value,
                        reason: $"Repositioning to loop (need {action})",
                        repositionLoopKey: loopKey);
                }
            }

            if (!hasDeliverableCars)
            {
                warnings.Add("No deliverable cars found.");
            }

            return new DeliveryPlan(new List<DeliveryStep>(), warnings);
        }

        /// <summary>
        /// Check if both sides have cars going to the same destination using ITrainService.
        /// </summary>
        private static bool SidesShareDestinationAbstract(CarGroup sideA, CarGroup sideB,
            ITrainService trainService, IReadOnlyCollection<string>? skippedCarIds)
        {
            if (sideA.IsEmpty || sideB.IsEmpty) return false;

            var destsA = new HashSet<string>();
            foreach (var car in sideA.Cars)
            {
                if (!trainService.HasWaybill(car)) continue;
                if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                var name = trainService.GetDestinationName(car);
                if (name != null) destsA.Add(name);
            }

            foreach (var car in sideB.Cars)
            {
                if (!trainService.HasWaybill(car)) continue;
                if (skippedCarIds != null && skippedCarIds.Contains(car.id)) continue;
                var name = trainService.GetDestinationName(car);
                if (name != null && destsA.Contains(name))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Pick the closer delivery between two side options using IGraphAdapter for distance.
        /// </summary>
        private List<DeliveryStep> PickCloserDeliveryAbstract(
            List<DeliveryStep> stepsA, List<DeliveryStep> stepsB)
        {
            if (stepsA.Count == 0) return stepsB;
            if (stepsB.Count == 0) return stepsA;

            if (stepsA.Count != stepsB.Count)
                return stepsA.Count > stepsB.Count ? stepsA : stepsB;

            // Use IGraphAdapter to find route distances from the loco to each destination.
            var locoFront = _iTrainService!.GetLocoFront();
            var locoRear = _iTrainService.GetLocoRear();
            var coupledIds = _iTrainService.GetCoupled().Select(c => c.id).ToList();

            var destA = stepsA[0].DestinationLocation;
            var destB = stepsB[0].DestinationLocation;

            var routeA = _iGraph!.FindBestRoute(locoFront.Undirected, destA, coupledIds);
            var routeB = _iGraph.FindBestRoute(locoFront.Undirected, destB, coupledIds);

            float distA = routeA?.Distance ?? float.MaxValue;
            float distB = routeB?.Distance ?? float.MaxValue;

            return distA <= distB ? stepsA : stepsB;
        }
    }
}
