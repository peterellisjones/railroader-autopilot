using System.Collections.Generic;

namespace Autopilot.Model
{
    public class DeliveryPlan
    {
        public List<DeliveryStep> Steps { get; }
        public RunaroundAction? Runaround { get; }
        public DirectedPosition? RepositionLocation { get; }
        public string? RepositionLoopKey { get; }
        public SplitInfo? Split { get; }
        public List<string> Warnings { get; }
        /// <summary>Human-readable reason for the plan decision (shown in UI).</summary>
        public string? Reason { get; }

        public DeliveryPlan(List<DeliveryStep> steps, List<string> warnings,
            RunaroundAction? runaround = null, DirectedPosition? repositionLocation = null,
            SplitInfo? split = null, string? reason = null,
            string? repositionLoopKey = null)
        {
            Steps = steps;
            Warnings = warnings;
            Runaround = runaround;
            RepositionLocation = repositionLocation;
            RepositionLoopKey = repositionLoopKey;
            Split = split;
            Reason = reason;
        }

        public bool HasDeliveries => Steps.Count > 0;
        public bool NeedsRunaround => Runaround != null;
        public bool NeedsReposition => RepositionLocation.HasValue;
        public bool NeedsSplit => Split != null;
    }
}
