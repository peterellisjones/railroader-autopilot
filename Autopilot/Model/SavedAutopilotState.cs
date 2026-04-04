namespace Autopilot.Model
{
    public class SavedAutopilotState
    {
        public AutopilotMode Mode { get; }
        public PickupFilter? PickupFilter { get; }
        public int PickupCount { get; }
        public PlanningContext Context { get; }
        public bool DeliverAfterPickup { get; }

        public SavedAutopilotState(AutopilotMode mode, PickupFilter? pickupFilter, int pickupCount, PlanningContext context, bool deliverAfterPickup)
        {
            Mode = mode;
            PickupFilter = pickupFilter;
            PickupCount = pickupCount;
            Context = context;
            DeliverAfterPickup = deliverAfterPickup;
        }
    }
}
