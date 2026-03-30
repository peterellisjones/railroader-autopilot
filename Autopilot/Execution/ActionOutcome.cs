using System.Collections.Generic;
using Model;

namespace Autopilot.Execution
{
    public abstract record ActionOutcome;

    public sealed record InProgress : ActionOutcome;

    public sealed record ActionDone : ActionOutcome;

    public sealed record ActionReplan(List<Car>? SkippedCars = null) : ActionOutcome;

    public sealed record ActionFailed(string Reason) : ActionOutcome;
}
