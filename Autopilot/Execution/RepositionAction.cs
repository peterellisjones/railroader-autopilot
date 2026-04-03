using Model;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Execution
{
    public class RepositionAction : IAction
    {
        private enum Phase { Moving, WaitingForStop }

        private Phase _phase;
        private float _stuckTimer;

        public string StatusMessage { get; private set; }

        public RepositionAction(DirectedPosition waypoint, BaseLocomotive loco,
            TrainService trainService, string? reason = null)
        {
            StatusMessage = reason ?? "Repositioning to loop...";
            trainService.SetWaypoint(loco, waypoint);
            _phase = Phase.Moving;
        }

        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            switch (_phase)
            {
                case Phase.Moving:
                    if (!trainService.IsWaypointMode(loco))
                        return new ActionFailed(
                            "Player took manual control during reposition — autopilot paused.");

                    if (trainService.IsWaypointSatisfied(loco) && trainService.IsStopped(loco))
                    {
                        _phase = Phase.WaitingForStop;
                    }
                    else if (trainService.IsStopped(loco))
                    {
                        _stuckTimer += AutopilotController.TickInterval;
                        if (_stuckTimer > Loader.Settings.stuckTimeoutSeconds)
                            return new ActionFailed($"Train stuck for {Loader.Settings.stuckTimeoutSeconds:0}s repositioning to loop. Is the route blocked?");
                    }
                    else
                    {
                        _stuckTimer = 0f;
                    }
                    return new InProgress();

                case Phase.WaitingForStop:
                    if (trainService.IsStoppedForDuration(loco, 0.5f))
                        return new ActionReplan();
                    return new InProgress();

                default:
                    return new InProgress();
            }
        }
    }
}
