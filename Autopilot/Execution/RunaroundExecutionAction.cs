using System.Linq;
using Model;
using Autopilot.Model;
using Autopilot.Services;
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
        private readonly Car _splitCar;
        private readonly Car _coupleTargetCar;
        private Phase _phase;
        private float _waitTimer;
        private float _stuckTimer;
        private string _statusMessage;
        private string _initError;

        public string StatusMessage => _statusMessage;

        public RunaroundExecutionAction(RunaroundAction runaround, BaseLocomotive loco, TrainService trainService)
        {
            _runaround = runaround;
            _splitCar = PlanUnwrapper.UnwrapCar(runaround.SplitCar);
            _coupleTargetCar = PlanUnwrapper.UnwrapCar(runaround.CoupleTarget);

            var consist = trainService.GetCoupled(loco);
            var splitEnd = runaround.SplitEnd;

            // Verify the split car is still in the consist
            if (!consist.Contains(_splitCar))
            {
                _initError = "Runaround failed — consist has changed.";
                _statusMessage = _initError;
                return;
            }

            Loader.Mod.Logger.Log($"Autopilot Runaround: uncoupling {_splitCar.DisplayName} at end {splitEnd}, CoupledTo(A)={_splitCar.CoupledTo(Car.LogicalEnd.A)?.DisplayName ?? "null"}, CoupledTo(B)={_splitCar.CoupledTo(Car.LogicalEnd.B)?.DisplayName ?? "null"}");
            trainService.Uncouple(_splitCar, splitEnd);
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
            bool stillInConsist = consist.Contains(_splitCar);
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

            if (trainService.IsCoupled(_splitCar, _runaround.SplitEnd))
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
            // Use the pre-computed couple location from planning. Convert to game type.
            var coupleLocation = PlanUnwrapper.ToCoupleWaypoint(_runaround.CoupleLocation,
                CreateAdapter(_runaround.CoupleLocation.SegmentId));

            Loader.Mod.Logger.Log($"Autopilot Runaround: setting coupling waypoint to {_coupleTargetCar.DisplayName}");
            _statusMessage = $"Runaround: routing to {_coupleTargetCar.DisplayName}...";
            trainService.SetWaypointWithCouple(loco, coupleLocation, _coupleTargetCar.id);

            _phase = Phase.WaitingForCouple;
            _waitTimer = 0f;
            return new InProgress();
        }

        private ActionOutcome TickWaitingForCouple(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            var consist = trainService.GetCoupled(loco);
            bool coupled = consist.Contains(_coupleTargetCar);

            if (coupled)
            {
                Loader.Mod.Logger.Log($"Autopilot Runaround: coupling detected — releasing handbrakes and connecting air");

                foreach (var car in _runaround.DisconnectedCars)
                {
                    var gameCar = (car as CarAdapter)?.Car;
                    if (gameCar != null) trainService.SetHandbrake(gameCar, false);
                }
                trainService.ConnectAirOnCoupled(loco);
                trainService.UpdateCarsForAE(loco);

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
            bool coupled = consist.Contains(_coupleTargetCar);

            if (!coupled)
            {
                Loader.Mod.Logger.Log("Autopilot Runaround: decoupled again, waiting for stable coupling");
                _phase = Phase.WaitingForCouple;
                return new InProgress();
            }

            if (!trainService.IsStoppedForDuration(loco, 2f))
                return new InProgress();

            Loader.Mod.Logger.Log("Autopilot Runaround: stopped — re-planning");

            _statusMessage = "Runaround complete — re-planning...";
            return new ActionReplan();
        }

        private static Autopilot.TrackGraph.GameGraphAdapter CreateAdapter(string segmentId)
        {
            var adapter = new Autopilot.TrackGraph.GameGraphAdapter();
            if (segmentId != null)
            {
                var graph = Track.Graph.Shared;
                foreach (var seg in graph.Segments)
                {
                    if (seg.id == segmentId)
                    {
                        adapter.RegisterSegment(seg);
                        break;
                    }
                }
            }
            return adapter;
        }
    }
}
