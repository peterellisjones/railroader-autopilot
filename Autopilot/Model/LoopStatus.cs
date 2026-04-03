namespace Autopilot.Model
{
    /// <summary>
    /// Whether the entire train is on a loop and can do a runaround.
    /// Two states: on a loop (can runaround) or not (need to reposition).
    /// </summary>
    public class LoopStatus
    {
        /// <summary>The loop the train is on, if any.</summary>
        public LoopInfo? Loop { get; }

        /// <summary>True if every segment the consist occupies is on the loop.</summary>
        public bool CanRunaround { get; }

        private LoopStatus(LoopInfo? loop, bool canRunaround)
        {
            Loop = loop;
            CanRunaround = canRunaround;
        }

        public static LoopStatus OnLoop(LoopInfo loop) => new LoopStatus(loop, true);
        public static LoopStatus NotOnLoop() => new LoopStatus(null, false);
    }
}
