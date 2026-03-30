using System;
using System.Diagnostics;
using Track;

namespace Autopilot.Model
{
    /// <summary>
    /// A point on a track segment with no directional information.
    /// Always stores distance from End A (canonical form) so two positions
    /// on the same segment are directly comparable without conversion.
    /// </summary>
    public readonly record struct TrackPosition(TrackSegment Segment, float DistanceFromA)
    {
        public float DistanceFromB => Segment.GetLength() - DistanceFromA;

        /// <summary>
        /// Distance between two positions on the SAME segment.
        /// Debug-asserts if segments differ — cross-segment distance requires route search.
        /// </summary>
        public float DistanceTo(TrackPosition other)
        {
            Debug.Assert(other.Segment == Segment, "Cannot compare positions on different segments");
            return Math.Abs(DistanceFromA - other.DistanceFromA);
        }

        /// <summary>Convert from game Location to canonical TrackPosition.</summary>
        public static TrackPosition FromLocation(Location loc)
        {
            float distFromA = loc.EndIsA ? loc.distance : loc.segment.GetLength() - loc.distance;
            return new TrackPosition(loc.segment, distFromA);
        }
    }
}
