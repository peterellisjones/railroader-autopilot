// Autopilot/Planning/DeliveryPlanner.cs
using System.Collections.Generic;
using System.Linq;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class DeliveryPlanner
    {
        private readonly TrainService _trainService;
        private readonly FeasibilityChecker _checker;
        private readonly DestinationSelector _destinationSelector;
        private readonly DeliverabilityAnalyzer _deliverabilityAnalyzer;
        private readonly RunaroundBuilder _runaroundBuilder;

        public DeliveryPlanner(TrainService trainService)
        {
            _trainService = trainService;
            _checker = new FeasibilityChecker(trainService);
            _destinationSelector = new DestinationSelector(trainService, _checker.RouteChecker);
            _deliverabilityAnalyzer = new DeliverabilityAnalyzer(_destinationSelector);
            _runaroundBuilder = new RunaroundBuilder(_deliverabilityAnalyzer);
        }

        private void ClearCaches()
        {
            _trainService.ClearPlanCaches();
            _destinationSelector.ClearCache();
            _checker.RouteChecker.ClearCache();
            _checker.ClearCache();
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot Planner: {msg}");

        public DeliveryPlan BuildPlan(BaseLocomotive loco,
            IEnumerable<string>? visitedSwitches = null,
            IEnumerable<string>? visitedLoopKeys = null,
            IEnumerable<Car>? skippedCars = null)
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

            // Check both sides for direct deliveries. Only need the first step —
            // we deliver one step at a time and replan after each.
            var stepsA = _deliverabilityAnalyzer.GetDeliverableSteps(loco, layout.SideA, _checker, skippedCars, maxSteps: 1);
            var stepsB = _deliverabilityAnalyzer.GetDeliverableSteps(loco, layout.SideB, _checker, skippedCars, maxSteps: 1);

            Log($"Direct deliveries: sideA={stepsA.Count}{(stepsA.Count > 0 ? $" (first: {stepsA[0].DestinationName})" : "")}, sideB={stepsB.Count}{(stepsB.Count > 0 ? $" (first: {stepsB[0].DestinationName})" : "")}");

            // If both sides have waybilled cars for the same destination,
            // consolidate first (runaround to put all cars on one side).
            // Otherwise delivering one side traps the loco in the siding
            // and the other side's cars require backing out and re-entering.
            // Check raw waybill destinations, not just deliverable steps —
            // approach direction may differ per side but the destination
            // conflict still exists.
            bool needsConsolidation = SidesShareDestination(layout.SideA, layout.SideB, skippedCars);

            if (needsConsolidation)
            {
                Log("Both sides have cars for same destination — need split to consolidate");
                // Fall through past direct delivery AND runaround to the split logic
            }
            else if (stepsA.Count > 0 || stepsB.Count > 0)
            {
                var bestSteps = PickCloserDelivery(loco, stepsA, stepsB);
                return new DeliveryPlan(bestSteps, warnings,
                    reason: $"Delivering {bestSteps[0].CarNames} to {bestSteps[0].DestinationName}");
            }

            // Check runarounds. When consolidation is needed (same dest on both
            // sides), the runaround detaches one side, the loco keeps the other
            // side's cars and goes around. The kept cars couple to the detached
            // cars, consolidating everything to one side.
            Log(needsConsolidation
                ? "Same destination on both sides — checking runaround to consolidate..."
                : "No direct deliveries — checking runarounds...");
            var flippedA = layout.SideA.Reversed();
            var flippedB = layout.SideB.Reversed();
            // Limit to a few steps for scoring — we only need to compare sides,
            // not enumerate all 50+ cars.
            var flippedStepsA = _deliverabilityAnalyzer.GetDeliverableSteps(loco, flippedA, _checker, skippedCars, maxSteps: 3);
            var flippedStepsB = _deliverabilityAnalyzer.GetDeliverableSteps(loco, flippedB, _checker, skippedCars, maxSteps: 3);
            int scoreA = flippedStepsA.Count;
            int scoreB = flippedStepsB.Count;

            // When consolidation is needed, boost the score for the side whose
            // runaround puts more same-destination cars together.
            if (needsConsolidation)
            {
                // After runaround on sideA: sideA cars join sideB → count matching
                // sideB destinations that appear in flipped sideA steps
                scoreA += CountSharedDestCars(flippedStepsA, stepsB);
                scoreB += CountSharedDestCars(flippedStepsB, stepsA);
            }

            Log($"Runaround scores: sideA={scoreA}, sideB={scoreB}");

            // Single loop status check — used for both runaround feasibility
            // and reposition decisions. CanRunaround = entire consist on loop.
            var loopStatus = _checker.GetLoopStatus(loco);
            Log($"Loop status: canRunaround={loopStatus.CanRunaround}" +
                (loopStatus.Loop != null ? $", loop={loopStatus.Loop.SwitchAId}↔{loopStatus.Loop.SwitchBId}" : ""));

            if (scoreA >= scoreB && scoreA > 0)
            {
                var runaround = _runaroundBuilder.BuildRunaroundAction(loco, layout.SideA, Track.Graph.Shared, _trainService, _checker, skippedCars);
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
                var runaround = _runaroundBuilder.BuildRunaroundAction(loco, layout.SideB, Track.Graph.Shared, _trainService, _checker, skippedCars);
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

            // Check if there ARE deliverable cars (just not from this position).
            // Only count cars whose destinations have available space — cars
            // for full sidings won't benefit from repositioning or runarounds.
            bool hasDeliverableCars = false;
            foreach (var car in layout.SideA.Cars.Concat(layout.SideB.Cars))
            {
                if (car.Waybill == null) continue;
                if (skippedCars != null && skippedCars.Contains((car as CarAdapter)?.Car)) continue;
                var space = _destinationSelector.GetAvailableSpace(car.Waybill.Value.Destination, loco);
                if (space >= 2f)
                {
                    hasDeliverableCars = true;
                    break;
                }
            }

            if (hasDeliverableCars && !loopStatus.CanRunaround)
            {
                Log("No runarounds feasible — checking reposition to loop...");

                // Collect delivery span locations so the loop evaluator can verify
                // approach feasibility from each candidate waypoint. Use the raw
                // span lower bound (same target CheckApproachDirection routes to),
                // not GetDestinationLocation which returns a coupling waypoint that
                // may be on a different segment.
                var deliveryDests = new List<DirectedPosition>();
                var seenDestIds = new HashSet<string>();
                foreach (var car in layout.SideA.Cars.Concat(layout.SideB.Cars))
                {
                    if (car.Waybill == null) continue;
                    if (car.Waybill.Value.Completed) continue;
                    if (skippedCars != null && skippedCars.Contains((car as CarAdapter)?.Car)) continue;
                    var dest = car.Waybill.Value.Destination;
                    if (!seenDestIds.Add(dest.Identifier)) continue;
                    foreach (var span in dest.Spans)
                    {
                        if (span.lower != null)
                        {
                            var pos = DirectedPosition.FromLocation(span.lower.Value);
                            deliveryDests.Add(pos);
                            Log($"Delivery dest for reversal check: {dest.DisplayName} → {pos.Segment?.id}");
                            break;
                        }
                    }
                }
                Log($"Total delivery destinations for reversal check: {deliveryDests.Count}");

                var (repositionLoc, loopKey) = _checker.GetRepositionLocation(loco, visitedSwitches, visitedLoopKeys, deliveryDests);
                if (repositionLoc.HasValue)
                {
                    // Find a destination name for the reason message.
                    // Try runaround candidates first, then any waybilled car.
                    string destName = null;
                    if (scoreA > 0 && flippedStepsA.Count > 0)
                        destName = flippedStepsA[0].DestinationName;
                    else if (scoreB > 0 && flippedStepsB.Count > 0)
                        destName = flippedStepsB[0].DestinationName;
                    else
                    {
                        foreach (var car in layout.SideA.Cars.Concat(layout.SideB.Cars))
                        {
                            if (car.Waybill != null && (skippedCars == null || !skippedCars.Contains((car as CarAdapter)?.Car)))
                            {
                                destName = car.Waybill.Value.Destination.DisplayName;
                                break;
                            }
                        }
                    }
                    string reason = destName != null
                        ? $"Repositioning to loop (need runaround to deliver to {destName})"
                        : "Repositioning to loop";

                    Log(reason);
                    return new DeliveryPlan(new List<DeliveryStep>(), warnings,
                        repositionLocation: repositionLoc.Value, reason: reason,
                        repositionLoopKey: loopKey);
                }
            }

            // Split is last resort — dropped cars block the railway.
            Log("No reposition available — checking splits...");
            var splitFinder = new SplitFinder(_trainService, _checker, _destinationSelector);
            var splitA = layout.SideA.IsEmpty ? null : splitFinder.FindBestSplit(loco, layout.SideA, skippedCars);
            var splitB = layout.SideB.IsEmpty ? null : splitFinder.FindBestSplit(loco, layout.SideB, skippedCars);

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

        /// <summary>
        /// Count cars in otherSteps whose destination matches any destination in flippedSteps.
        /// Used to boost runaround scores for consolidation.
        /// </summary>
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

        /// <summary>
        /// Check if any destination appears in both step lists.
        /// </summary>
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

        /// <summary>
        /// Check if both sides of the consist have waybilled cars for the
        /// same destination. Uses raw waybill data, not deliverability.
        /// </summary>
        private static bool SidesShareDestination(CarGroup sideA, CarGroup sideB, IEnumerable<Car>? skippedCars)
        {
            if (sideA.IsEmpty || sideB.IsEmpty) return false;

            var destsA = new HashSet<string>();
            foreach (var car in sideA.Cars)
            {
                if (car.Waybill == null) continue;
                var gameCar = (car as CarAdapter)?.Car;
                if (skippedCars != null && gameCar != null && skippedCars.Contains(gameCar)) continue;
                destsA.Add(car.Waybill.Value.Destination.DisplayName);
            }

            foreach (var car in sideB.Cars)
            {
                if (car.Waybill == null) continue;
                var gameCar = (car as CarAdapter)?.Car;
                if (skippedCars != null && gameCar != null && skippedCars.Contains(gameCar)) continue;
                if (destsA.Contains(car.Waybill.Value.Destination.DisplayName))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Pick the better delivery side. Prefer more deliverable steps;
        /// when equal, prefer the side whose first delivery is closer.
        /// </summary>
        private List<DeliveryStep> PickCloserDelivery(BaseLocomotive loco,
            List<DeliveryStep> stepsA, List<DeliveryStep> stepsB)
        {
            if (stepsA.Count == 0) return stepsB;
            if (stepsB.Count == 0) return stepsA;

            // More deliverable steps wins
            if (stepsA.Count != stepsB.Count)
                return stepsA.Count > stepsB.Count ? stepsA : stepsB;

            // Equal count — prefer closer first delivery (check both loco directions)
            float distA = _trainService.GraphDistanceToLoco(loco, stepsA[0].DestinationLocation)?.Distance ?? float.MaxValue;
            float distB = _trainService.GraphDistanceToLoco(loco, stepsB[0].DestinationLocation)?.Distance ?? float.MaxValue;
            return distA <= distB ? stepsA : stepsB;
        }
    }
}
