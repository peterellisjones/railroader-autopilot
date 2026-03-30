using Model;
using Autopilot.Services;

namespace Autopilot.Execution
{
    public interface IAction
    {
        ActionOutcome Tick(BaseLocomotive loco, TrainService trainService);
        string StatusMessage { get; }
    }
}
