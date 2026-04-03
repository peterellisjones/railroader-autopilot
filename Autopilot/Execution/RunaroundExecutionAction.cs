using Model;
using Track;
using Autopilot.Model;
using Autopilot.Services;
using static Autopilot.Services.CoupleLocationCalculator;

namespace Autopilot.Execution
{
    public class RunaroundExecutionAction : IAction
    {
        private enum Phase
        {
            Uncoupling,
            WaitingForDecouple,
            UpdatingAE,
            SettingWaypoint,
            WaitingForCouple,
            Stabilizing
        }

        private readonly RunaroundAction _runaround;
        private Phase _phase;
        private float _waitTimer;
        private float _stuckTimer;
        private string _statusMessage;
        private string _initError;

        public string StatusMessage => _statusMessage;

        public RunaroundExecutionAction(RunaroundAction runaround, BaseLocomotive loco, TrainService trainService)
        {
            _runaround = runaround;

            var consist = trainService.GetCoupled(loco);
            var splitCar = runaround.SplitCar;
            var splitEnd = runaround.SplitEnd;

            // Verify the split car is still in the consist
            if (!consist.Contains(splitCar))
            {
                _initError = "Runaround failed — consist has changed.";
                _statusMessage = _initError;
                return;
            }

            Loader.Mod.Logger.Log($"Autopilot Runaround: uncoupling {splitCar.DisplayName} at end {splitEnd}, CoupledTo(A)={splitCar.CoupledTo(Car.LogicalEnd.A)?.DisplayName ?? "null"}, CoupledTo(B)={splitCar.CoupledTo(Car.LogicalEnd.B)?.DisplayName ?? "null"}");
            trainService.Uncouple(splitCar, splitEnd);
            trainService.UpdateCarsForAE(loco);

            // Set handbrakes and bleed air on disconnected cars
            DisconnectHelper.DisconnectCars(runaround.DisconnectedCars, trainService);

            _statusMessage = "Runaround: uncoupling...";
            _phase = Phase.Uncoupling;
            _waitTimer = 0f;
        }

        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            if (_initError != null)
                return new ActionFailed(_initError);

            switch (_phase)
            {
                case Phase.Uncoupling:
                    return TickUncoupling(loco, trainService);
                case Phase.WaitingForDecouple:
                    return TickWaitingForDecouple(loco, trainService);
                case Phase.UpdatingAE:
                    return TickUpdatingAE(loco, trainService);
                case Phase.SettingWaypoint:
                    return TickSettingWaypoint(loco, trainService);
                case Phase.WaitingForCouple:
                    return TickWaitingForCouple(loco, trainService);
                case Phase.Stabilizing:
                    return TickStabilizing(loco, trainService);
                default:
                    return new ActionFailed("RunaroundExecutionAction: unknown phase.");
            }
        }

        private ActionOutcome TickUncoupling(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            var consist = trainService.GetCoupled(loco);
            bool stillInConsist = consist.Contains(_runaround.SplitCar);
            if (stillInConsist)
            {
                if (_waitTimer > AutopilotConstants.DecoupleWaitSeconds)
                    return new ActionFailed("Runaround: cars did not decouple.");
                return new InProgress();
            }

            _phase = Phase.WaitingForDecouple;
            _waitTimer = 0f;
            return new InProgress();
        }

        private ActionOutcome TickWaitingForDecouple(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            if (trainService.IsCoupled(_runaround.SplitCar, _runaround.SplitEnd))
            {
                _waitTimer = 0f;
                return new InProgress();
            }

            _phase = Phase.UpdatingAE;
            _waitTimer = 0f;
            return new InProgress();
        }

        private ActionOutcome TickUpdatingAE(BaseLocomotive loco, TrainService trainService)
        {
            Loader.Mod.Logger.Log("Autopilot Runaround: updating AE cars after decouple");
            trainService.UpdateCarsForAE(loco);

            _phase = Phase.SettingWaypoint;
            _waitTimer = 0f;
            return new InProgress();
        }

        private ActionOutcome TickSettingWaypoint(BaseLocomotive loco, TrainService trainService)
        {
            var target = _runaround.CoupleTarget;
            var graph = Graph.Shared;

            // Find the far end of the target so the loco routes around to the far side.
            var locoPos = DirectedPosition.FromLocation(loco.LocationF);
            var coupleLocation = GetCoupleLocation(target, locoPos, graph);

            Loader.Mod.Logger.Log($"Autopilot Runaround: setting coupling waypoint to {target.DisplayName}");
            _statusMessage = $"Runaround: routing to {target.DisplayName}...";
            trainService.SetWaypointWithCouple(loco, coupleLocation, target.id);

            _phase = Phase.WaitingForCouple;
            _waitTimer = 0f;
            return new InProgress();
        }

        private ActionOutcome TickWaitingForCouple(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            var consist = trainService.GetCoupled(loco);
            bool coupled = consist.Contains(_runaround.CoupleTarget);

            if (coupled)
            {
                Loader.Mod.Logger.Log("Autopilot Runaround: coupling detected — waiting for stop");
                _statusMessage = "Runaround: coupled, waiting for stop...";
                _phase = Phase.Stabilizing;
                return new InProgress();
            }

            var persistence = new global::Model.AI.AutoEngineerPersistence(loco.KeyValueObject);
            var status = persistence.PlannerStatus ?? "";
            if (status.Contains("blocked") || status.Contains("Blocked"))
                return new ActionFailed($"Runaround: {status}. Is there a runaround track available?");

            if (trainService.IsStopped(loco))
            {
                _stuckTimer += AutopilotController.TickInterval;
                if (_stuckTimer > Loader.Settings.stuckTimeoutSeconds)
                    return new ActionFailed($"Train stuck for {Loader.Settings.stuckTimeoutSeconds:0}s during runaround. Is there a runaround track available?");
            }
            else
            {
                _stuckTimer = 0f;
            }

            return new InProgress();
        }

        private ActionOutcome TickStabilizing(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            var consist = trainService.GetCoupled(loco);
            bool coupled = consist.Contains(_runaround.CoupleTarget);

            if (!coupled)
            {
                Loader.Mod.Logger.Log("Autopilot Runaround: decoupled again, waiting for stable coupling");
                _phase = Phase.WaitingForCouple;
                return new InProgress();
            }

            if (!trainService.IsStoppedForDuration(loco, 2f))
                return new InProgress();

            Loader.Mod.Logger.Log("Autopilot Runaround: stopped — connecting air and releasing handbrakes");
            foreach (var car in _runaround.DisconnectedCars)
                trainService.SetHandbrake(car, false);
            trainService.ConnectAirOnCoupled(loco);
            trainService.UpdateCarsForAE(loco);

            _statusMessage = "Runaround complete — re-planning...";
            return new ActionReplan();
        }
    }
}
