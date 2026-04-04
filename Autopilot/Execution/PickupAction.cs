using System.Collections.Generic;
using Model;
using Model.AI;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Execution
{
    public class PickupAction : IAction
    {
        private enum Phase { MovingToTarget, Stabilizing }

        private readonly PickupTarget _target;
        private Phase _phase;
        private float _waitTimer;
        private float _stuckTimer;

        public string StatusMessage { get; private set; }

        /// <summary>Number of target cars in this pickup (for state machine counting).</summary>
        public int TargetCarCount => _target.TargetCars.Count;

        public PickupAction(PickupTarget target, BaseLocomotive loco, TrainService trainService)
        {
            _target = target;
            StatusMessage = $"Moving to pick up {target.CoupleTarget.DisplayName}...";

            Loader.Mod.Logger.Log($"Autopilot Pickup: setting waypoint to couple with {target.CoupleTarget.DisplayName}");
            trainService.SetWaypointWithCouple(loco, target.CoupleLocation, target.CoupleTarget.id);

            _phase = Phase.MovingToTarget;
            _waitTimer = 0f;
        }

        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            switch (_phase)
            {
                case Phase.MovingToTarget:
                    if (!trainService.IsWaypointMode(loco))
                        return new ActionFailed("Player took manual control during pickup — autopilot paused.");

                    var consist = trainService.GetCoupled(loco);
                    if (consist.Contains(_target.CoupleTarget))
                    {
                        Loader.Mod.Logger.Log("Autopilot Pickup: coupling detected");
                        StatusMessage = "Coupled — waiting for stop...";
                        _phase = Phase.Stabilizing;
                        _waitTimer = 0f;
                        return new InProgress();
                    }

                    var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
                    var status = persistence.PlannerStatus ?? "";
                    var orders = persistence.Orders;
                    bool stopped = trainService.IsStopped(loco);
                    bool wpSatisfied = trainService.IsWaypointSatisfied(loco);
                    bool wpMode = trainService.IsWaypointMode(loco);

                    // Check for AE planner errors — only skip the couple
                    // target so the chain can be retried from its other end.
                    if (status.Contains("blocked") || status.Contains("Blocked")
                        || status.Contains("End of Track") || status.Contains("too long"))
                    {
                        Loader.Mod.Logger.Log($"Autopilot Pickup: can't reach {_target.CoupleTarget.DisplayName}: {status} — skipping approach");
                        return new ActionReplan(new List<Car> { _target.CoupleTarget });
                    }

                    // If the AE isn't moving toward the car, skip this
                    // approach but only mark the couple target as skipped —
                    // not all target cars. The same chain may be reachable
                    // from its other end on the next planning cycle.
                    if (wpSatisfied || (stopped && status.Contains("At waypoint") && _stuckTimer > 5f))
                    {
                        Loader.Mod.Logger.Log($"Autopilot Pickup: AE not moving to {_target.CoupleTarget.DisplayName} (status={status}) — skipping approach");
                        return new ActionReplan(new List<Car> { _target.CoupleTarget });
                    }

                    if (trainService.IsStopped(loco))
                    {
                        _stuckTimer += AutopilotController.TickInterval;
                        if (_stuckTimer > Loader.Settings.stuckTimeoutSeconds)
                            return new ActionFailed($"Train stuck for {Loader.Settings.stuckTimeoutSeconds:0}s reaching {_target.CoupleTarget.DisplayName}. Is the route blocked?");
                    }
                    else
                    {
                        _stuckTimer = 0f;
                    }
                    return new InProgress();

                case Phase.Stabilizing:
                    // Verify still coupled (bounce detection)
                    var consist2 = trainService.GetCoupled(loco);
                    if (!consist2.Contains(_target.CoupleTarget))
                    {
                        _phase = Phase.MovingToTarget;
                        return new InProgress();
                    }

                    if (!trainService.IsStoppedForDuration(loco, 2f))
                        return new InProgress();

                    Loader.Mod.Logger.Log($"Autopilot Pickup: releasing handbrakes and connecting air");
                    foreach (var car in trainService.GetCoupled(loco))
                        trainService.SetHandbrake(car, false);
                    trainService.ConnectAirOnCoupled(loco);
                    trainService.UpdateCarsForAE(loco);

                    StatusMessage = $"Picked up {_target.CarNames} — replanning...";
                    return new ActionReplan();

                default:
                    return new InProgress();
            }
        }
    }
}
