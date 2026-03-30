using Autopilot.Model;

namespace Autopilot.Execution
{
    public abstract record AutopilotPhase
    {
        public abstract string StatusMessage { get; }
    }

    public sealed record Idle : AutopilotPhase
    {
        public override string StatusMessage => "Idle";
    }

    public sealed record PlanningPhase(PlanningContext Context, AutopilotMode Mode, string? TargetDestination, int PickupCount = 0)
        : AutopilotPhase
    {
        public override string StatusMessage => "Planning next move...";
    }

    public sealed record Executing(
        DeliveryPlan? Plan,
        IAction CurrentAction,
        PlanningContext Context,
        AutopilotMode Mode,
        string? TargetDestination,
        int PickupCount,
        DirectedPosition? CurrentWaypoint)
        : AutopilotPhase
    {
        public override string StatusMessage => CurrentAction.StatusMessage;
    }

    public sealed record Completed(string Message, int PickupCount = 0) : AutopilotPhase
    {
        public override string StatusMessage => Message;
    }

    public sealed record Failed(string ErrorMessage, DeliveryPlan? LastPlan) : AutopilotPhase
    {
        public override string StatusMessage => $"Error: {ErrorMessage}";
    }
}
