using Model;
using Model.AI;
using Track;
using Autopilot.Model;
using Autopilot.Services;
using static Autopilot.Services.CoupleLocationCalculator;

namespace Autopilot.Execution
{
    public class RecoupleAction : IAction
    {
        private enum Phase { MovingToDropPoint, Stabilizing }

        private readonly SplitInfo _split;
        private Car _coupleTarget;
        private string _initError;
        private Phase _phase;
        private float _waitTimer;

        public string StatusMessage { get; private set; }

        public RecoupleAction(SplitInfo split, BaseLocomotive loco, TrainService trainService)
        {
            _split = split;
            StatusMessage = "Returning to recouple dropped cars...";

            // Pick the closest reachable end of the dropped car chain.
            // The planner will handle runarounds if needed after recoupling.
            var graph = Graph.Shared;
            var firstDropped = split.DroppedCars[0];
            var lastDropped = split.DroppedCars[split.DroppedCars.Count - 1];
            var locoPos = DirectedPosition.FromLocation(loco.LocationF);

            // Use the free (uncoupled) end of each end car — not crow-flies
            // nearest, which can pick the coupled end and place the waypoint
            // between cars in the chain.
            var firstFreeEnd = firstDropped.CoupledTo(Car.LogicalEnd.A) != null
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            var lastFreeEnd = lastDropped.CoupledTo(Car.LogicalEnd.A) != null
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;

            var firstLoc = GetCoupleLocationForEnd(new CarAdapter(firstDropped), firstFreeEnd, graph);
            var lastLoc = GetCoupleLocationForEnd(new CarAdapter(lastDropped), lastFreeEnd, graph);

            // Skip ends that face buffer stops
            float distFirst = float.MaxValue;
            float distLast = float.MaxValue;
            if (firstLoc.HasValue)
            {
                var result = Planning.RouteChecker.RouteDistanceWithCars(loco.LocationF, firstLoc.Value.ToLocation(),
                    trainService.GetCoupled(loco));
                distFirst = result?.Distance ?? float.MaxValue;
            }
            if (lastLoc.HasValue)
            {
                var result = Planning.RouteChecker.RouteDistanceWithCars(loco.LocationF, lastLoc.Value.ToLocation(),
                    trainService.GetCoupled(loco));
                distLast = result?.Distance ?? float.MaxValue;
            }

            if (lastLoc.HasValue && distLast < distFirst)
            {
                _coupleTarget = lastDropped;
                Loader.Mod.Logger.Log($"Autopilot Recouple: coupling to far end {lastDropped.DisplayName} " +
                    $"(dist={distLast:F0}m vs {distFirst:F0}m)");
                trainService.SetWaypointWithCouple(loco, lastLoc.Value, lastDropped.id);
            }
            else if (firstLoc.HasValue)
            {
                _coupleTarget = firstDropped;
                Loader.Mod.Logger.Log($"Autopilot Recouple: coupling to near end {firstDropped.DisplayName} " +
                    $"(dist={distFirst:F0}m vs {distLast:F0}m)");
                trainService.SetWaypointWithCouple(loco, firstLoc.Value, firstDropped.id);
            }
            else
            {
                _initError = "Cannot reach dropped cars — both ends face buffer stops.";
                StatusMessage = _initError;
                return;
            }

            _phase = Phase.MovingToDropPoint;
            _waitTimer = 0f;
        }


        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            if (_initError != null)
                return new ActionFailed(_initError);

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
