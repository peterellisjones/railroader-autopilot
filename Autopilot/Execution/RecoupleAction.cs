using Model;
using Model.AI;
using Track;
using Autopilot.Model;
using Autopilot.Services;
using Autopilot.Planning;
using static Autopilot.Services.CoupleLocationCalculator;

namespace Autopilot.Execution
{
    public class RecoupleAction : IAction
    {
        private enum Phase { MovingToDropPoint, Stabilizing }

        private readonly SplitInfo _split;
        private Car _coupleTarget;
        private Phase _phase;
        private float _waitTimer;

        public string StatusMessage { get; private set; }

        public RecoupleAction(SplitInfo split, BaseLocomotive loco, TrainService trainService)
        {
            _split = split;
            StatusMessage = "Returning to recouple dropped cars...";

            // Choose which end of the dropped car chain to couple to.
            // Coupling from the original side preserves car order.
            // Coupling from the other side flips the order (like a runaround),
            // which may enable more direct deliveries without needing a
            // separate runaround later.
            //
            // Check approach direction from each end to the dropped cars'
            // destinations. Pick the end that makes more destinations
            // directly deliverable (tail-leads).
            var graph = Graph.Shared;
            var firstDropped = split.DroppedCars[0];
            var lastDropped = split.DroppedCars[split.DroppedCars.Count - 1];

            int scoreFirst = CountDeliverableFromEnd(firstDropped, split, loco, graph);
            int scoreLast = CountDeliverableFromEnd(lastDropped, split, loco, graph);

            var locoPos = DirectedPosition.FromLocation(loco.LocationF);

            if (scoreLast > scoreFirst)
            {
                _coupleTarget = lastDropped;
                var coupleLoc = GetCoupleLocation(lastDropped, locoPos, graph);
                Loader.Mod.Logger.Log($"Autopilot Recouple: coupling to far end {lastDropped.DisplayName} " +
                    $"(deliverable: {scoreLast} vs {scoreFirst} from near end)");
                trainService.SetWaypointWithCouple(loco, coupleLoc, lastDropped.id);
            }
            else
            {
                _coupleTarget = firstDropped;
                var coupleLoc = GetCoupleLocation(firstDropped, locoPos, graph);
                Loader.Mod.Logger.Log($"Autopilot Recouple: coupling to near end {firstDropped.DisplayName} " +
                    $"(deliverable: {scoreFirst} vs {scoreLast} from far end)");
                trainService.SetWaypointWithCouple(loco, coupleLoc, firstDropped.id);
            }

            _phase = Phase.MovingToDropPoint;
            _waitTimer = 0f;
        }

        /// <summary>
        /// Count how many dropped cars have destinations reachable with
        /// tail-leads if we couple from the given end car. Routes from
        /// that car's position to each destination and checks reversal parity.
        /// Even reversals = tail leads after coupling from this end.
        /// </summary>
        private static int CountDeliverableFromEnd(Car endCar, SplitInfo split, BaseLocomotive loco, Graph graph)
        {
            int count = 0;
            var endLoc = endCar.LocationF;

            foreach (var car in split.DroppedCars)
            {
                if (!car.Waybill.HasValue) continue;
                var dest = car.Waybill.Value.Destination;
                foreach (var span in dest.Spans)
                {
                    if (!span.lower.HasValue) continue;
                    var destLoc = span.lower.Value;
                    if (destLoc.segment == null) continue;

                    var steps = new System.Collections.Generic.List<Track.Search.RouteSearch.Step>();
                    bool found = Track.Search.RouteSearch.FindRoute(
                        graph, endLoc, destLoc,
                        RouteChecker.DefaultHeuristic, steps,
                        out Track.Search.RouteSearch.Metrics _,
                        checkForCars: false, trainLength: 0f,
                        maxIterations: 3000, enableLogging: false);

                    if (found && ReversalCounter.FromSteps(steps) % 2 == 0)
                        count++;
                    break;
                }
            }
            return count;
        }

        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            switch (_phase)
            {
                case Phase.MovingToDropPoint:
                    if (!trainService.IsWaypointMode(loco))
                        return new ActionFailed("Player took manual control during recouple — autopilot paused.");

                    var consist = trainService.GetCoupled(loco);
                    if (consist.Contains(_coupleTarget))
                    {
                        Loader.Mod.Logger.Log("Autopilot Recouple: coupling detected");
                        StatusMessage = "Recoupling — waiting for stop...";
                        _phase = Phase.Stabilizing;
                        _waitTimer = 0f;
                        return new InProgress();
                    }

                    // Check for AE planner errors
                    var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
                    var status = persistence.PlannerStatus ?? "";
                    if (status.Contains("blocked") || status.Contains("Blocked"))
                        return new ActionFailed($"Recouple: {status}. Can the loco reach the dropped cars?");

                    if (_waitTimer > 300f)
                        return new ActionFailed("Recouple: timed out returning to dropped cars.");
                    return new InProgress();

                case Phase.Stabilizing:
                    var consist2 = trainService.GetCoupled(loco);
                    if (!consist2.Contains(_coupleTarget))
                    {
                        _phase = Phase.MovingToDropPoint;
                        return new InProgress();
                    }

                    if (!trainService.IsStoppedForDuration(loco, 2f))
                        return new InProgress();

                    Loader.Mod.Logger.Log("Autopilot Recouple: connecting air and releasing handbrakes");
                    foreach (var car in _split.DroppedCars)
                        trainService.SetHandbrake(car, false);
                    trainService.ConnectAirOnCoupled(loco);
                    trainService.UpdateCarsForAE(loco);

                    StatusMessage = "Recouple complete — re-planning...";
                    return new ActionReplan();

                default:
                    return new InProgress();
            }
        }
    }
}
