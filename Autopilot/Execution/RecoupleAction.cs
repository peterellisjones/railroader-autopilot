using Model;
using Model.AI;
using Track;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Execution
{
    public class RecoupleAction : IAction
    {
        private enum Phase { MovingToDropPoint, Stabilizing }

        private readonly SplitInfo _split;
        private Phase _phase;
        private float _waitTimer;

        public string StatusMessage { get; private set; }

        public RecoupleAction(SplitInfo split, BaseLocomotive loco, TrainService trainService)
        {
            _split = split;
            StatusMessage = "Returning to recouple dropped cars...";

            Loader.Mod.Logger.Log($"Autopilot Recouple: setting waypoint to couple with {split.CoupleTarget.DisplayName}");
            trainService.SetWaypointWithCouple(loco, split.CoupleLocation, split.CoupleTarget.id);

            _phase = Phase.MovingToDropPoint;
            _waitTimer = 0f;
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
                    if (consist.Contains(_split.CoupleTarget))
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
                    if (!consist2.Contains(_split.CoupleTarget))
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
