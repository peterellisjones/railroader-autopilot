using System.Collections.Generic;
using System.Linq;
using Model;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Execution
{
    public class SplitAction : IAction
    {
        private enum Phase { Uncoupling, WaitingForDecouple }

        private readonly SplitInfo _split;
        private readonly Car _splitCar;
        private readonly List<Car> _droppedCars;
        private Phase _phase;
        private float _waitTimer;

        public string StatusMessage { get; private set; }

        public SplitAction(SplitInfo split, BaseLocomotive loco, TrainService trainService)
        {
            _split = split;
            _splitCar = PlanUnwrapper.UnwrapCar(split.SplitCar);
            _droppedCars = PlanUnwrapper.UnwrapCars(split.DroppedCars);
            StatusMessage = $"Splitting train — dropping {split.DroppedCars.Count} car(s)...";

            Loader.Mod.Logger.Log($"Autopilot Split: uncoupling at {_splitCar.DisplayName} end {split.SplitEnd}");
            trainService.Uncouple(_splitCar, split.SplitEnd);
            trainService.UpdateCarsForAE(loco);

            DisconnectHelper.DisconnectCars(_droppedCars, trainService);

            _phase = Phase.Uncoupling;
            _waitTimer = 0f;
        }

        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            switch (_phase)
            {
                case Phase.Uncoupling:
                    var consist = trainService.GetCoupled(loco);
                    bool stillCoupled = _droppedCars.Any(c => consist.Contains(c));

                    if (!stillCoupled)
                    {
                        _phase = Phase.WaitingForDecouple;
                        _waitTimer = 0f;
                        return new InProgress();
                    }

                    if (_waitTimer > AutopilotConstants.DecoupleWaitSeconds)
                        return new ActionFailed("Split: cars did not decouple.");
                    return new InProgress();

                case Phase.WaitingForDecouple:
                    if (trainService.IsStoppedForDuration(loco, 0.5f))
                    {
                        StatusMessage = "Split complete — re-planning with shorter consist...";
                        return new ActionReplan();
                    }
                    return new InProgress();

                default:
                    return new InProgress();
            }
        }
    }
}
