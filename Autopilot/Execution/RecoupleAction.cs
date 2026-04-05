using System.Collections.Generic;
using System.Linq;
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
        private readonly List<Car> _droppedCars;
        private Car _coupleTarget;
        private string _initError;
        private Phase _phase;
        private float _waitTimer;

        public string StatusMessage { get; private set; }

        public RecoupleAction(SplitInfo split, BaseLocomotive loco, TrainService trainService)
        {
            _split = split;
            _droppedCars = PlanUnwrapper.UnwrapCars(split.DroppedCars);
            StatusMessage = "Returning to recouple dropped cars...";

            // Pick the closest reachable end of the dropped car chain.
            var graph = Graph.Shared;
            var firstDropped = _droppedCars[0];
            var lastDropped = _droppedCars[_droppedCars.Count - 1];
            var locoPos = DirectedPosition.FromLocation(loco.LocationF);

            var firstFreeEnd = firstDropped.CoupledTo(Car.LogicalEnd.A) != null
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            var lastFreeEnd = lastDropped.CoupledTo(Car.LogicalEnd.A) != null
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;

            var firstLoc = GetCoupleLocationForEnd(new CarAdapter(firstDropped), firstFreeEnd, graph);
            var lastLoc = GetCoupleLocationForEnd(new CarAdapter(lastDropped), lastFreeEnd, graph);

            var resultFirst = Planning.RouteChecker.RouteDistanceWithCars(loco.LocationF, firstLoc.ToLocation(),
                trainService.GetCoupled(loco));
            var resultLast = Planning.RouteChecker.RouteDistanceWithCars(loco.LocationF, lastLoc.ToLocation(),
                trainService.GetCoupled(loco));

            float distFirst = resultFirst?.Distance ?? float.MaxValue;
            float distLast = resultLast?.Distance ?? float.MaxValue;

            if (distLast < distFirst)
            {
                _coupleTarget = lastDropped;
                Loader.Mod.Logger.Log($"Autopilot Recouple: coupling to far end {lastDropped.DisplayName} " +
                    $"(dist={distLast:F0}m vs {distFirst:F0}m)");
                trainService.SetWaypointWithCouple(loco, lastLoc, lastDropped.id);
            }
            else
            {
                _coupleTarget = firstDropped;
                Loader.Mod.Logger.Log($"Autopilot Recouple: coupling to near end {firstDropped.DisplayName} " +
                    $"(dist={distFirst:F0}m vs {distLast:F0}m)");
                trainService.SetWaypointWithCouple(loco, firstLoc, firstDropped.id);
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
                    foreach (var car in _droppedCars)
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
