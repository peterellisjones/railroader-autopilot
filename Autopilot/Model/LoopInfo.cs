using System.Collections.Generic;

namespace Autopilot.Model
{
    /// <summary>
    /// A detected loop: two switches connected by two or more independent routes.
    /// Includes the fitting branch and entry/far switch IDs.
    /// Uses string IDs instead of game types for testability.
    /// Waypoint Location is computed by the caller (FeasibilityChecker).
    /// </summary>
    public class LoopInfo
    {
        /// <summary>First switch ID of the loop (closer to the loco along the initial walk).</summary>
        public string SwitchAId { get; }

        /// <summary>Second switch ID of the loop.</summary>
        public string SwitchBId { get; }

        /// <summary>All branches of the loop (2 for passing siding, 3 for wye).</summary>
        public List<LoopBranch> Branches { get; }

        /// <summary>The branch the train fits on.</summary>
        public LoopBranch FittingBranch { get; }

        /// <summary>Switch ID where the train enters the loop (reachable from current position).</summary>
        public string EntrySwitchId { get; }

        /// <summary>Switch ID at the far end of the fitting branch from the entry.</summary>
        public string ExitSwitchId { get; }

        /// <summary>Unique key: sorted switch IDs joined by |. Used to track visited loops.</summary>
        public string LoopKey { get; }

        public LoopInfo(string switchAId, string switchBId,
            List<LoopBranch> branches, LoopBranch fittingBranch,
            string entrySwitchId, string exitSwitchId)
        {
            SwitchAId = switchAId;
            SwitchBId = switchBId;
            Branches = branches;
            FittingBranch = fittingBranch;
            EntrySwitchId = entrySwitchId;
            ExitSwitchId = exitSwitchId;

            var ids = new List<string> { switchAId, switchBId };
            ids.Sort();
            LoopKey = string.Join("|", ids);
        }
    }
}
