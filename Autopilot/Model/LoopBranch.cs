using System.Collections.Generic;

namespace Autopilot.Model
{
    /// <summary>
    /// One branch of a loop — the path between two switches along one route.
    /// A passing siding has 2 branches (main + siding). A wye has 3.
    /// Uses string IDs instead of game types for testability.
    /// </summary>
    public class LoopBranch
    {
        /// <summary>Segment IDs forming this branch, ordered from start switch to end switch.</summary>
        public List<string> SegmentIds { get; }

        /// <summary>Total length of all segments.</summary>
        public float Length { get; }

        /// <summary>Fouling distance at the start switch for this branch's leg.</summary>
        public float FoulingAtStart { get; }

        /// <summary>Fouling distance at the end switch for this branch's leg.</summary>
        public float FoulingAtEnd { get; }

        /// <summary>Usable length after subtracting fouling at both ends.</summary>
        public float UsableLength => Length - FoulingAtStart - FoulingAtEnd;

        /// <summary>Switches along this branch that are not the loop endpoints.</summary>
        public List<string> IntermediateSwitchIds { get; }

        /// <summary>Which leg segment ID of the start switch this branch connects to.</summary>
        public string StartLegSegId { get; }

        /// <summary>Which leg segment ID of the end switch this branch connects to.</summary>
        public string EndLegSegId { get; }

        public LoopBranch(List<string> segmentIds, float length,
            float foulingAtStart, float foulingAtEnd,
            List<string> intermediateSwitchIds,
            string startLegSegId, string endLegSegId)
        {
            SegmentIds = segmentIds;
            Length = length;
            FoulingAtStart = foulingAtStart;
            FoulingAtEnd = foulingAtEnd;
            IntermediateSwitchIds = intermediateSwitchIds;
            StartLegSegId = startLegSegId;
            EndLegSegId = endLegSegId;
        }
    }
}
