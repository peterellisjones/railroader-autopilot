namespace Autopilot.Model
{
    public class SavedAutopilotState
    {
        public AutopilotMode Mode { get; }
        public string? TargetDestination { get; }
        public int PickupCount { get; }
        public PlanningContext Context { get; }

        public SavedAutopilotState(AutopilotMode mode, string? targetDestination, int pickupCount, PlanningContext context)
        {
            Mode = mode;
            TargetDestination = targetDestination;
            PickupCount = pickupCount;
            Context = context;
        }
    }
}
