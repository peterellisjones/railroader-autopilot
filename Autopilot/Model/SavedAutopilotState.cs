namespace Autopilot.Model
{
    public class SavedAutopilotState
    {
        public AutopilotMode Mode { get; }
        public string? TargetDestination { get; }
        public int PickupCount { get; }
        public PlanningContext Context { get; }
        public bool DeliverAfterPickup { get; }

        public SavedAutopilotState(AutopilotMode mode, string? targetDestination, int pickupCount, PlanningContext context, bool deliverAfterPickup)
        {
            Mode = mode;
            TargetDestination = targetDestination;
            PickupCount = pickupCount;
            Context = context;
            DeliverAfterPickup = deliverAfterPickup;
        }
    }
}
