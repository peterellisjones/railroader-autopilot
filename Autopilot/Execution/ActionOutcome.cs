using System.Collections.Generic;
using Autopilot.Model;

namespace Autopilot.Execution
{
    public abstract record ActionOutcome;

    public sealed record InProgress : ActionOutcome;

    public sealed record ActionDone : ActionOutcome;

    public sealed record ActionReplan(List<ICar>? SkippedCars = null) : ActionOutcome;

    public sealed record ActionFailed(string Reason) : ActionOutcome;
}
